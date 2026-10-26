//! Clean-room mesh → analytic solid: region-grow clusters, fit plane / cylinder / sphere,
//! sew [`FaceSpec`]s. Matches the process stages in `TEMP_REFERENCE/PROCESS_SPEC.md`
//! (cluster → fit → sew), without vendor WASM heuristics.

use std::collections::{BTreeMap, HashMap, HashSet}; // HashSet: cluster membership during grow
use std::f64::consts::{PI, TAU};

use solidcraft_geom::Vec3;

use crate::meshbody::TriMesh;
use crate::sew::{EdgeSpec, FaceSpec, SurfSpec, nonsimple_faces, sew};
use crate::{Body, KernelError, Result};

const MAX_FACES: usize = 50_000;
const MAX_LOOP_EDGES: usize = 8_192;
const MAX_CLUSTER_TRIS: usize = 100_000;

fn invalid(message: &str) -> KernelError {
    KernelError::Invalid(format!("mesh conversion: {message}"))
}

/// Last resort: each triangle becomes a planar face. Preserves volume for closed meshes
/// when analytic / prismatic recognition cannot sew.
pub fn convert_faceted_mesh(body: &Body, tolerance: f64) -> Result<Body> {
    crate::guard("convert faceted mesh", || {
        if !tolerance.is_finite() || !(1e-6..=1.).contains(&tolerance) {
            return Err(invalid("tolerance must be between 0.000001 and 1 mm"));
        }
        let input = body.mesh.as_ref().ok_or_else(|| invalid("select a mesh body"))?;
        let welded = crate::mesh_convert::weld_mesh(input, tolerance)?;
        let mesh = welded.mesh.as_ref().ok_or_else(|| invalid("empty mesh"))?;
        if mesh.triangles.len() > crate::mesh_convert::MAX_CONVERT_TRIANGLES {
            return Err(invalid("mesh exceeds the conversion triangle budget"));
        }
        ensure_manifold(mesh)?;
        let (_, normals, _) = triangle_geom(mesh)?;
        let circles = HashMap::new();
        let mut faces = Vec::with_capacity(mesh.triangles.len());
        for ti in 0..mesh.triangles.len() {
            faces.push(planar_face(
                mesh,
                &mesh.positions,
                &[ti],
                &normals,
                tolerance,
                &circles,
                &HashMap::new(),
                &HashSet::new(),
                None,
                &mut HashSet::new(),
            )?);
            if faces.len() > MAX_FACES {
                return Err(invalid("too many faces to convert"));
            }
        }
        if faces.len() < 4 {
            return Err(invalid("not enough faces to close a solid"));
        }
        sew(&faces).map_err(|e| invalid(&format!("faceted sew failed: {e}")))
    })
}

/// Which curved families to look for, and how many triangles may start a region.
/// Planes are always fitted. Cones, tori and threads are recorded for a later fitter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecognizeOptions {
    pub cylinders: bool,
    pub cones: bool,
    pub tori: bool,
    pub spheres: bool,
    pub threads: bool,
    /// 1 uses every triangle as a seed. Lower values keep every `1/fraction` triangle.
    pub mesh_fraction: f64,
}

impl Default for RecognizeOptions {
    fn default() -> Self {
        Self { cylinders: true, cones: true, tori: true, spheres: true, threads: true, mesh_fraction: 1.0 }
    }
}

fn seed_kept(seed: usize, fraction: f64) -> bool {
    if !(fraction.is_finite() && fraction < 1.0) {
        return true;
    }
    let stride = (1.0 / fraction).round().clamp(1.0, 1000.0) as usize;
    seed % stride.max(1) == 0
}

/// Reconstruct a closed mesh as an analytic solid by clustering coplanar / coaxial /
/// spherical triangle sets, fitting surfaces within `tolerance` mm, and sewing them.
/// `angle` is the maximum normal deviation (radians) for growing a planar region.
pub fn convert_analytic_mesh(body: &Body, tolerance: f64, angle: f64) -> Result<Body> {
    convert_analytic_with(body, tolerance, angle, &RecognizeOptions::default())
}

pub(crate) fn convert_analytic_with(body: &Body, tolerance: f64, angle: f64, options: &RecognizeOptions) -> Result<Body> {
    crate::guard("convert analytic mesh", || {
        if !tolerance.is_finite() || !(1e-6..=1.).contains(&tolerance) {
            return Err(invalid("tolerance must be between 0.000001 and 1 mm"));
        }
        if !angle.is_finite() || !(1e-4..=PI / 2.).contains(&angle) {
            return Err(invalid("angle must be between 0.0001 and π/2 radians"));
        }
        let input = body.mesh.as_ref().ok_or_else(|| invalid("select a mesh body"))?;
        let welded = crate::mesh_convert::weld_mesh(input, tolerance)?;
        let mesh = welded.mesh.as_ref().ok_or_else(|| invalid("empty mesh"))?;
        if mesh.triangles.len() > crate::mesh_convert::MAX_CONVERT_TRIANGLES {
            return Err(invalid(&format!(
                "the mesh has {} triangles, more than the {} conversion can take",
                mesh.triangles.len(),
                crate::mesh_convert::MAX_CONVERT_TRIANGLES
            )));
        }
        ensure_manifold(mesh)?;
        let (tris, normals, areas) = triangle_geom(mesh)?;
        let adj = triangle_adjacency(mesh)?;
        let cos_ang = angle.cos();
        // Cones, tori and threads are chosen in the convert dialog. Their triangles stay
        // eligible for planes until a fitter for those families exists.
        let _later = (options.cones, options.tori, options.threads);
        let mut assigned = vec![false; mesh.triangles.len()];
        let mut faces: Vec<FaceSpec> = Vec::new();
        // Triangle cluster behind each planar face, so a pinched wire can be replaced by its triangles.
        let mut origins: Vec<Vec<usize>> = Vec::new();

        let push_face = |faces: &mut Vec<FaceSpec>, origins: &mut Vec<Vec<usize>>, face: FaceSpec, cluster: Vec<usize>| -> Result<()> {
            faces.push(face);
            origins.push(cluster);
            if faces.len() > MAX_FACES {
                return Err(invalid("too many faces to convert"));
            }
            Ok(())
        };

        // Curved families before planes so finely faceted cylinders/spheres are not eaten as flats.
        // Pass A: full-body short-circuits (kernel primitives). Pass B: partial curved FaceSpecs.
        let mut curved: Vec<FaceSpec> = Vec::new();
        // Canonical circles keyed by quantized (axis, centre, radius, height) so planar holes
        // reuse the same EdgeSpecs as cylinder walls.
        let mut circles: HashMap<CircleKey, Vec<EdgeSpec>> = HashMap::new();

        // --- Pass A1: pure cylindrical can → crate::cylinder ---
        for seed in 0..mesh.triangles.len() {
            if !options.cylinders || !seed_kept(seed, options.mesh_fraction) || assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let Some((cluster, cyl)) = grow_cylinder(mesh, &tris, &normals, &adj, &assigned, seed, tolerance, cos_ang)? else {
                continue;
            };
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a cylindrical region exceeds the conversion budget"));
            }
            if let Some((_feature_faces, all_tris)) =
                try_full_cylinder_feature(mesh, &tris, &normals, &assigned, &cluster, &cyl, tolerance, &mut circles)?
            {
                if all_tris.len() * 20 >= mesh.triangles.len() * 19 {
                    let (t_lo, t_hi) = cylinder_height_range(mesh, &cluster, &cyl)?;
                    let base = cyl.origin + cyl.axis * t_lo;
                    let height = t_hi - t_lo;
                    return crate::cylinder(base, cyl.axis, cyl.radius, height).map_err(|e| invalid(&format!("cylinder feature: {e}")));
                }
            }
        }

        // --- Pass A2: pure sphere → crate::sphere ---
        for seed in 0..mesh.triangles.len() {
            if !options.spheres || !seed_kept(seed, options.mesh_fraction) || assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let Some((cluster, sph)) = grow_sphere(mesh, &tris, &normals, &adj, &assigned, seed, tolerance)? else {
                continue;
            };
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a spherical region exceeds the conversion budget"));
            }
            let all_sphere = claim_sphere_tris(&tris, &normals, &assigned, &sph, tolerance);
            if all_sphere.len() * 20 >= mesh.triangles.len() * 19 {
                return crate::sphere(sph.center, sph.radius).map_err(|e| invalid(&format!("sphere feature: {e}")));
            }
            let _ = cluster;
        }

        // Partial spheres stay as facets. Their rims are mesh polylines; emitting an analytic
        // sphere whose boundary does not share those edges makes the whole shell fail to sew,
        // and the fallback then drops every cylinder too. A closed ball still short-circuits above.
        // Tubes are recorded first and built only once their caps lie on the same plane as the
        // circle, so a tessellated face (tenths of a millimetre off a plane) still sews.
        let mut skip_cyl = assigned.clone();
        let mut pending: Vec<PendingTube> = Vec::new();
        for seed in 0..mesh.triangles.len() {
            if !options.cylinders || !seed_kept(seed, options.mesh_fraction) || skip_cyl.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let Some((cluster, cyl)) = grow_cylinder(mesh, &tris, &normals, &adj, &skip_cyl, seed, tolerance, cos_ang)? else {
                continue;
            };
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a cylindrical region exceeds the conversion budget"));
            }
            if let Some(tube) = try_pending_tube(mesh, &cluster, &cyl, tolerance)? {
                for &t in &tube.cluster {
                    if let Some(slot) = skip_cyl.get_mut(t) {
                        *slot = true;
                    }
                    if let Some(slot) = assigned.get_mut(t) {
                        *slot = true;
                    }
                }
                pending.push(tube);
            } else {
                for &t in &cluster {
                    if let Some(slot) = skip_cyl.get_mut(t) {
                        *slot = true;
                    }
                }
            }
        }

        let mut planes: Vec<HeldPlane> = Vec::new();
        for seed in 0..mesh.triangles.len() {
            if !seed_kept(seed, options.mesh_fraction) || assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let Some(cluster) = grow_plane(mesh, &tris, &normals, &areas, &adj, &assigned, seed, tolerance, cos_ang)? else {
                continue;
            };
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a planar region exceeds the conversion budget"));
            }
            for &t in &cluster {
                if let Some(slot) = assigned.get_mut(t) {
                    *slot = true;
                }
            }
            if let Some(fit) = fit_cluster_plane(mesh, &normals, &cluster) {
                planes.push(HeldPlane { cluster, n: fit.n, d: fit.d, locked: false });
            } else {
                for &t in &cluster {
                    if let Some(slot) = assigned.get_mut(t) {
                        *slot = false;
                    }
                }
            }
        }
        let owners = plane_owners(mesh, &planes);
        let mut kept: Vec<PendingTube> = Vec::new();
        for tube in pending {
            let saved: Vec<(Vec3, f64, bool)> = planes.iter().map(|p| (p.n, p.d, p.locked)).collect();
            let locked =
                lock_rim(&mut planes, &owners, &tube.rim0, &tube.cyl, tube.h0) && lock_rim(&mut planes, &owners, &tube.rim1, &tube.cyl, tube.h1);
            if locked {
                kept.push(tube);
            } else {
                for (plane, (n, d, was)) in planes.iter_mut().zip(saved) {
                    plane.n = n;
                    plane.d = d;
                    plane.locked = was;
                }
                for &t in &tube.cluster {
                    if let Some(slot) = assigned.get_mut(t) {
                        *slot = false;
                    }
                }
            }
        }
        // Tessellation splits one face into patches whose planes are a fraction of a degree
        // apart. Snapping a shared vertex onto both leaves it on neither, and truck then
        // refuses the wire. Fold those patches back into one plane first.
        merge_coplanar_planes(mesh, &normals, &mut planes, tolerance);
        let snapped = snapped_positions(mesh, &planes);
        // Caps look circles up by the rim's vertex set. A geometric fit misses once snapping
        // moves a rim point, and the cap then uses lines the wall does not have.
        let mut by_rim: HashMap<Vec<u32>, Vec<EdgeSpec>> = HashMap::new();
        for tube in &kept {
            let bot = register_circle(&mut circles, tube.cyl.axis, tube.cyl.origin + tube.cyl.axis * tube.h0, tube.cyl.radius, 0.3)?;
            let top = register_circle(&mut circles, tube.cyl.axis, tube.cyl.origin + tube.cyl.axis * tube.h1, tube.cyl.radius, 0.3)?;
            let mut k0 = tube.rim0.clone();
            let mut k1 = tube.rim1.clone();
            k0.sort_unstable();
            k1.sort_unstable();
            by_rim.insert(k0, bot);
            by_rim.insert(k1, top);
        }
        // A snapped boundary that repeats a vertex is not a face. Those triangles become
        // facets, and neighbouring planes must keep the vertices the facets still use.
        let empty_keep = HashSet::new();
        for plane in &planes {
            if matches!(
                planar_face(
                    mesh,
                    &snapped,
                    &plane.cluster,
                    &normals,
                    tolerance,
                    &circles,
                    &by_rim,
                    &empty_keep,
                    Some(PlaneFit { n: plane.n, d: plane.d }),
                    &mut HashSet::new(),
                ),
                Err(error) if error.to_string().contains("not simple")
            ) {
                for &t in &plane.cluster {
                    if let Some(slot) = assigned.get_mut(t) {
                        *slot = false;
                    }
                }
            }
        }
        let mut leftover_verts = HashSet::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            if assigned.get(ti).copied().unwrap_or(true) {
                continue;
            }
            leftover_verts.extend(tri.iter().copied());
        }
        let mut used_rims: HashSet<Vec<u32>> = HashSet::new();
        for pass in 0..2 {
            used_rims.clear();
            let mut staged: Vec<(FaceSpec, Vec<usize>)> = Vec::new();
            let mut failed: Vec<usize> = Vec::new();
            for (pi, plane) in planes.iter().enumerate() {
                if plane.cluster.first().is_some_and(|&t| !assigned.get(t).copied().unwrap_or(false)) {
                    continue;
                }
                match planar_face(
                    mesh,
                    &snapped,
                    &plane.cluster,
                    &normals,
                    tolerance,
                    &circles,
                    &by_rim,
                    &leftover_verts,
                    Some(PlaneFit { n: plane.n, d: plane.d }),
                    &mut used_rims,
                ) {
                    Ok(face) => staged.push((face, plane.cluster.clone())),
                    Err(error) if error.to_string().contains("not simple") => failed.push(pi),
                    Err(error) => return Err(error),
                }
            }
            // A circle on only one cap has no wall to meet. Drop that pair and rebuild
            // the cap as lines the faceted wall can share.
            let mut orphan = false;
            for tube in &kept {
                let mut k0 = tube.rim0.clone();
                let mut k1 = tube.rim1.clone();
                k0.sort_unstable();
                k1.sort_unstable();
                let (hit0, hit1) = (used_rims.contains(&k0), used_rims.contains(&k1));
                if hit0 != hit1 {
                    orphan = true;
                    by_rim.remove(&k0);
                    by_rim.remove(&k1);
                }
            }
            if orphan && pass == 0 {
                continue;
            }
            for pi in failed {
                if let Some(plane) = planes.get(pi) {
                    for &t in &plane.cluster {
                        if let Some(slot) = assigned.get_mut(t) {
                            *slot = false;
                        }
                    }
                }
            }
            for tube in &kept {
                let mut k0 = tube.rim0.clone();
                let mut k1 = tube.rim1.clone();
                k0.sort_unstable();
                k1.sort_unstable();
                if !used_rims.contains(&k0) || !used_rims.contains(&k1) {
                    for &t in &tube.cluster {
                        if let Some(slot) = assigned.get_mut(t) {
                            *slot = false;
                        }
                    }
                    continue;
                }
                curved.extend(cylinder_face(mesh, &tube.cluster, &tube.cyl, tolerance, &mut circles)?);
            }
            for (face, cluster) in staged {
                push_face(&mut faces, &mut origins, face, cluster)?;
            }
            break;
        }

        // Leftover triangles as planar facets so the shell can still close.
        for seed in 0..mesh.triangles.len() {
            if assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let face = planar_face(mesh, &snapped, &[seed], &normals, tolerance, &circles, &by_rim, &HashSet::new(), None, &mut HashSet::new())?;
            if let Some(slot) = assigned.get_mut(seed) {
                *slot = true;
            }
            push_face(&mut faces, &mut origins, face, vec![seed])?;
        }

        // A face whose wire revisits a vertex (after the sew weld) is split back into its
        // triangles. One pinched patch must not discard every other analytic face.
        #[cfg(test)]
        let mut pinched_faces = 0usize;
        for _ in 0..32 {
            let mut all = faces.clone();
            all.extend(curved.iter().cloned());
            let mut splittable: Vec<usize> =
                nonsimple_faces(&all).into_iter().filter(|&i| origins.get(i).is_some_and(|cluster| cluster.len() > 1)).collect();
            // A patch truck cannot give a plane (sliver, or points off the plane) becomes its triangles.
            // Circle caps stay whole: splitting one would drop the arcs the wall is built from.
            for (i, face) in faces.iter().enumerate() {
                if origins.get(i).is_some_and(|cluster| cluster.len() > 1) && face_not_a_plane(face) && !splittable.contains(&i) {
                    splittable.push(i);
                }
            }
            if splittable.is_empty() {
                break;
            }
            for fi in splittable.into_iter().rev() {
                let Some(cluster) = origins.get(fi).cloned() else { continue };
                if cluster.len() < 2 {
                    continue;
                }
                origins.remove(fi);
                faces.remove(fi);
                #[cfg(test)]
                {
                    pinched_faces += 1;
                }
                let mut at = fi;
                for ti in cluster {
                    let face =
                        planar_face(mesh, &snapped, &[ti], &normals, tolerance, &circles, &by_rim, &HashSet::new(), None, &mut HashSet::new())?;
                    faces.insert(at, face);
                    origins.insert(at, vec![ti]);
                    at += 1;
                    if faces.len() > MAX_FACES {
                        return Err(invalid("too many faces to convert"));
                    }
                }
            }
        }

        // Try sewing with curved faces; if the shell will not close, demote curved
        // clusters to per-triangle planar facets and sew a faceted solid instead.
        let mut attempt = faces.clone();
        attempt.extend(curved.iter().cloned());
        if attempt.len() < 4 {
            return Err(invalid("not enough faces to close a solid"));
        }
        match sew(&attempt) {
            Ok(body) => Ok(body),
            Err(e) if curved.is_empty() => {
                #[cfg(test)]
                {
                    eprintln!(
                        "diag planar-only faces={} kept={} used_rims={} pinched={} err={e}",
                        faces.len(),
                        kept.len(),
                        used_rims.len(),
                        pinched_faces
                    );
                    let text = e.to_string();
                    if let Some(fi) = text
                        .split("face ")
                        .nth(1)
                        .and_then(|rest| rest.split(|c: char| !c.is_ascii_digit()).next())
                        .and_then(|n| n.parse::<usize>().ok())
                        && let Some(face) = attempt.get(fi)
                    {
                        let (nlen, dev) = face_plane_metrics(face);
                        let edges = face.loops.iter().map(|lp| lp.len()).sum::<usize>();
                        let arcs = face.loops.iter().flatten().filter(|edge| matches!(edge, EdgeSpec::Arc { .. })).count();
                        eprintln!("diag face {fi} loops={} edges={edges} arcs={arcs} nlen={nlen:.3e} dev={dev:.3e}", face.loops.len());
                    }
                    let (once, over, unique) = boundary_uses(&attempt);
                    eprintln!("diag edges once={once} over={over} unique={unique}");
                }
                Err(invalid(&format!("sew failed (planar-only): {e}")))
            }
            Err(first) => {
                #[cfg(test)]
                {
                    eprintln!("diag planar={} curved={} err={first}", faces.len(), curved.len());
                    let q = |p: Vec3| ((p.x * 1e6).round() as i64, (p.y * 1e6).round() as i64, (p.z * 1e6).round() as i64);
                    let mut uses: HashMap<(i64, i64, i64, i64, i64, i64, i64, i64, i64), (usize, bool)> = HashMap::new();
                    let mut all = faces.clone();
                    all.extend(curved.iter().cloned());
                    for face in &all {
                        for edge in face.loops.iter().flatten() {
                            let (a, b, mid, arc) = match edge {
                                EdgeSpec::Line { a, b } => (*a, *b, (*a + *b) * 0.5, false),
                                EdgeSpec::Arc { a, b, mid } => (*a, *b, *mid, true),
                                EdgeSpec::Chain { ctrl, .. } => {
                                    (ctrl.first().copied().unwrap_or(Vec3::ZERO), ctrl.last().copied().unwrap_or(Vec3::ZERO), Vec3::ZERO, false)
                                }
                            };
                            let (mut ka, mut kb, km) = (q(a), q(b), q(mid));
                            if (kb, km) < (ka, km) {
                                std::mem::swap(&mut ka, &mut kb);
                            }
                            let key = (ka.0, ka.1, ka.2, kb.0, kb.1, kb.2, km.0, km.1, km.2);
                            let slot = uses.entry(key).or_insert((0, arc));
                            slot.0 += 1;
                            slot.1 |= arc;
                        }
                    }
                    let once = uses.values().filter(|(c, _)| *c == 1).count();
                    let once_arc = uses.values().filter(|(c, arc)| *c == 1 && *arc).count();
                    let over = uses.values().filter(|(c, _)| *c > 2).count();
                    eprintln!("diag edges once={once} once_arc={once_arc} over={over} unique={}", uses.len());
                }
                // Refuse to "succeed" as a faceted solid here: `convert_mesh` must still
                // try the prismatic path (cylinders in holes) before `convert_faceted_mesh`.
                Err(invalid(&format!("curved faces would not sew: {first}")))
            }
        }
    })
}

fn ensure_manifold(mesh: &TriMesh) -> Result<()> {
    let mut edges: BTreeMap<(u32, u32), (usize, i32)> = BTreeMap::new();
    for triangle in &mesh.triangles {
        let [a, b, c] = *triangle;
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let entry = edges.entry((u.min(v), u.max(v))).or_default();
            entry.0 += 1;
            entry.1 += if u < v { 1 } else { -1 };
        }
    }
    if edges.values().any(|&(count, balance)| count != 2 || balance != 0) {
        return Err(invalid("mesh must be closed and consistently oriented, with two triangles per edge"));
    }
    Ok(())
}

fn triangle_geom(mesh: &TriMesh) -> Result<(Vec<[Vec3; 3]>, Vec<Vec3>, Vec<f64>)> {
    let mut tris = Vec::with_capacity(mesh.triangles.len());
    let mut normals = Vec::with_capacity(mesh.triangles.len());
    let mut areas = Vec::with_capacity(mesh.triangles.len());
    for triangle in &mesh.triangles {
        let [a, b, c] = *triangle;
        let (Some(pa), Some(pb), Some(pc)) = (mesh.positions.get(a as usize), mesh.positions.get(b as usize), mesh.positions.get(c as usize)) else {
            return Err(invalid("triangle refers to a missing vertex"));
        };
        let cross = (*pb - *pa).cross(*pc - *pa);
        let area = cross.len() * 0.5;
        let Some(n) = cross.normalized() else {
            return Err(invalid("degenerate triangle"));
        };
        tris.push([*pa, *pb, *pc]);
        normals.push(n);
        areas.push(area);
    }
    Ok((tris, normals, areas))
}

fn triangle_adjacency(mesh: &TriMesh) -> Result<Vec<Vec<usize>>> {
    let mut edge_tri: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, triangle) in mesh.triangles.iter().enumerate() {
        let [a, b, c] = *triangle;
        for (u, v) in [(a, b), (b, c), (c, a)] {
            edge_tri.entry((u.min(v), u.max(v))).or_default().push(ti);
        }
    }
    let mut adj = vec![Vec::new(); mesh.triangles.len()];
    for pair in edge_tri.values() {
        if pair.len() != 2 {
            continue;
        }
        let (Some(&a), Some(&b)) = (pair.first(), pair.get(1)) else { continue };
        if let Some(slot) = adj.get_mut(a) {
            slot.push(b);
        }
        if let Some(slot) = adj.get_mut(b) {
            slot.push(a);
        }
    }
    Ok(adj)
}

struct PlaneFit {
    n: Vec3,
    d: f64,
}

fn grow_plane(
    _mesh: &TriMesh,
    tris: &[[Vec3; 3]],
    normals: &[Vec3],
    areas: &[f64],
    adj: &[Vec<usize>],
    assigned: &[bool],
    seed: usize,
    tolerance: f64,
    cos_ang: f64,
) -> Result<Option<Vec<usize>>> {
    let Some(n0) = normals.get(seed).copied() else {
        return Ok(None);
    };
    let Some(seed_tri) = tris.get(seed) else {
        return Ok(None);
    };
    let c0 = (seed_tri[0] + seed_tri[1] + seed_tri[2]) * (1. / 3.);
    let d0 = c0.dot(n0);
    let mut cluster = vec![seed];
    let mut in_cluster = HashSet::from([seed]);
    let mut stack = vec![seed];
    let mut steps = 0usize;
    while let Some(ti) = stack.pop() {
        steps += 1;
        if steps > MAX_CLUSTER_TRIS * 4 {
            return Err(invalid("planar region grow exceeds its work budget"));
        }
        for &nb in adj.get(ti).into_iter().flatten() {
            if assigned.get(nb).copied().unwrap_or(true) || in_cluster.contains(&nb) {
                continue;
            }
            let Some(nn) = normals.get(nb).copied() else { continue };
            if nn.dot(n0).abs() < cos_ang {
                continue;
            }
            let Some(tri) = tris.get(nb) else { continue };
            if tri.iter().any(|p| (p.dot(n0) - d0).abs() > tolerance) {
                continue;
            }
            in_cluster.insert(nb);
            cluster.push(nb);
            stack.push(nb);
        }
    }
    // Stable plane from area-weighted normals / centroids.
    let mut sum_n = Vec3::ZERO;
    let mut sum_c = Vec3::ZERO;
    let mut sum_a = 0.;
    for &t in &cluster {
        let Some(nn) = normals.get(t) else { continue };
        let Some(a) = areas.get(t) else { continue };
        let Some(tri) = tris.get(t) else { continue };
        let sign = if nn.dot(n0) >= 0. { 1. } else { -1. };
        sum_n += *nn * sign * *a;
        sum_c += (tri[0] + tri[1] + tri[2]) * (1. / 3.) * *a;
        sum_a += *a;
    }
    let Some(n) = (sum_n * (1. / sum_a.max(1e-18))).normalized() else {
        return Ok(None);
    };
    let d = sum_c.dot(n) / sum_a.max(1e-18);
    let fit = PlaneFit { n, d };
    if !cluster.iter().all(|&t| {
        tris.get(t).is_some_and(|tr| tr.iter().all(|p| (p.dot(fit.n) - fit.d).abs() <= tolerance))
            && normals.get(t).is_some_and(|nn| nn.dot(fit.n).abs() >= cos_ang)
    }) {
        if cluster.len() == 1 {
            return Ok(Some(cluster));
        }
        return Ok(None);
    }
    let n_ref = normals.get(seed).copied().unwrap_or(fit.n);
    if cluster.len() > 1 && !cluster.iter().all(|&t| normals.get(t).is_some_and(|nn| nn.dot(n_ref).abs() >= cos_ang)) {
        return Ok(None);
    }
    Ok(Some(cluster))
}

#[derive(Clone, Copy)]
struct CylFit {
    axis: Vec3,
    origin: Vec3,
    radius: f64,
}

/// True when `seed` and `other` are facets of one cylinder about `n0 × n1`.
/// Rejects a neighbour that only happens to sit in the normal fan-out band.
fn wall_pair_shares_cylinder(tris: &[[Vec3; 3]], seed: usize, other: usize, n0: Vec3, n1: Vec3, tolerance: f64) -> bool {
    let Some(axis) = n0.cross(n1).normalized() else { return false };
    if n0.dot(axis).abs() > 0.08 || n1.dot(axis).abs() > 0.08 {
        return false;
    }
    let (Some(a), Some(b)) = (tris.get(seed), tris.get(other)) else { return false };
    let mut pts = Vec::with_capacity(6);
    pts.extend_from_slice(a);
    pts.extend_from_slice(b);
    let u = axis.any_perp();
    let v = axis.cross(u);
    let pl: Vec<(f64, f64)> = pts.iter().map(|p| (p.dot(u), p.dot(v))).collect();
    let Some((cu, cv, r)) = fit_circle_2d(&pl) else { return false };
    if !r.is_finite() || r < tolerance * 2. {
        return false;
    }
    pl.iter().all(|(x, y)| ((x - cu).hypot(y - cv) - r).abs() <= tolerance * 2.)
}

fn triangle_on_cylinder(tris: &[[Vec3; 3]], normals: &[Vec3], ti: usize, cyl: &CylFit, tolerance: f64, cos_ang: f64) -> bool {
    let Some(nn) = normals.get(ti).copied() else { return false };
    if nn.dot(cyl.axis).abs() > 0.08 {
        return false;
    }
    let Some(tri) = tris.get(ti) else { return false };
    let on_radius = tri.iter().all(|p| {
        let w = *p - cyl.origin;
        let radial = w - cyl.axis * w.dot(cyl.axis);
        (radial.len() - cyl.radius).abs() <= tolerance * 2.
    });
    if !on_radius {
        return false;
    }
    let mid = (tri[0] + tri[1] + tri[2]) * (1. / 3.);
    let w = mid - cyl.origin;
    let radial = w - cyl.axis * w.dot(cyl.axis);
    let Some(rhat) = radial.normalized() else { return false };
    nn.dot(rhat).abs() >= (cos_ang * 0.85).min(0.75)
}

fn expand_cylinder(
    tris: &[[Vec3; 3]],
    normals: &[Vec3],
    adj: &[Vec<usize>],
    assigned: &[bool],
    cluster: &mut Vec<usize>,
    in_cluster: &mut HashSet<usize>,
    cyl: &CylFit,
    tolerance: f64,
    cos_ang: f64,
) -> Result<()> {
    let mut stack = cluster.clone();
    let mut steps = 0usize;
    while let Some(ti) = stack.pop() {
        steps += 1;
        if steps > MAX_CLUSTER_TRIS * 4 {
            return Err(invalid("cylindrical region grow exceeds its work budget"));
        }
        for &nb in adj.get(ti).into_iter().flatten() {
            if assigned.get(nb).copied().unwrap_or(true) || in_cluster.contains(&nb) {
                continue;
            }
            if !triangle_on_cylinder(tris, normals, nb, cyl, tolerance, cos_ang) {
                continue;
            }
            in_cluster.insert(nb);
            cluster.push(nb);
            stack.push(nb);
        }
    }
    Ok(())
}

fn grow_cylinder(
    _mesh: &TriMesh,
    tris: &[[Vec3; 3]],
    normals: &[Vec3],
    adj: &[Vec<usize>],
    assigned: &[bool],
    seed: usize,
    tolerance: f64,
    cos_ang: f64,
) -> Result<Option<(Vec<usize>, CylFit)>> {
    let Some(n0) = normals.get(seed).copied() else {
        return Ok(None);
    };
    // Seed axis from two *wall* normals: n_i × n_j ∥ cylinder axis. Cap normals are nearly
    // orthogonal to wall normals; wall×cap is circumferential and must not seed the axis.
    // Fine facets are only a few degrees apart, so walk a small BFS for a wall neighbour
    // whose |n·n0| sits in (MIN_WALL_DOT, MAX_WALL_DOT].
    const MAX_WALL_DOT: f64 = 0.92; // ≥ ~23° fan-out
    const MIN_WALL_DOT: f64 = 0.2; // reject ~90° (cap) and near-opposite flips
    let Some(seed_tri) = tris.get(seed) else {
        return Ok(None);
    };
    // One wall triangle lies on only two generators, so it does not determine a circle.
    // Collect the wall neighbourhood (never crossing a cap) and fit that.
    let mut hood_pts: Vec<Vec3> = seed_tri.to_vec();
    let mut far = n0;
    let mut far_dot = 1.0_f64;
    let mut seen = HashSet::from([seed]);
    let mut frontier = vec![seed];
    for _depth in 0..16 {
        let mut next = Vec::new();
        for &ti in &frontier {
            for &nb in adj.get(ti).into_iter().flatten() {
                if assigned.get(nb).copied().unwrap_or(true) || !seen.insert(nb) {
                    continue;
                }
                let Some(nn) = normals.get(nb).copied() else { continue };
                let d = nn.dot(n0).abs();
                // Caps and faces across a sharp corner are near-orthogonal to this wall.
                // Walking through them reaches an unrelated wall (the plate's outer side)
                // whose normal also sits in the fan-out band and seeds a false axis.
                if d < MIN_WALL_DOT {
                    continue;
                }
                if hood_pts.len() < 96
                    && let Some(tri) = tris.get(nb)
                {
                    hood_pts.extend_from_slice(tri);
                }
                // Prefer the most rotated neighbour that still shares this cylinder.
                if d <= MAX_WALL_DOT && d < far_dot && wall_pair_shares_cylinder(tris, seed, nb, n0, nn, tolerance) {
                    far_dot = d;
                    far = nn;
                }
                next.push(nb);
            }
        }
        if far_dot <= MAX_WALL_DOT {
            break;
        }
        if next.is_empty() {
            break;
        }
        frontier = next;
    }
    // Need real wall-to-wall fan-out: a nearly planar patch cannot seed a cylinder.
    if far_dot > MAX_WALL_DOT {
        return Ok(None);
    }
    let Some(axis) = n0.cross(far).normalized() else {
        return Ok(None);
    };
    if n0.dot(axis).abs() > 0.05 || far.dot(axis).abs() > 0.05 {
        return Ok(None);
    }
    // Fit the neighbourhood, not the seed triangle: two generators do not determine a radius.
    let Some(cyl0) = fit_cylinder_pts(&hood_pts, &[n0, far], axis, tolerance) else {
        return Ok(None);
    };
    let mut cluster = vec![seed];
    let mut in_cluster = HashSet::from([seed]);
    expand_cylinder(tris, normals, adj, assigned, &mut cluster, &mut in_cluster, &cyl0, tolerance, cos_ang)?;
    if cluster.len() < 4 {
        return Ok(None);
    }
    // Refit on all member vertices, then pick up any triangles the seed fit stopped short of.
    let mut pts = Vec::new();
    let mut nrms = Vec::new();
    for &t in &cluster {
        let Some(tri) = tris.get(t) else { continue };
        let Some(nn) = normals.get(t) else { continue };
        pts.extend_from_slice(tri);
        nrms.push(*nn);
        nrms.push(*nn);
        nrms.push(*nn);
    }
    let Some(cyl) = fit_cylinder_pts(&pts, &nrms, cyl0.axis, tolerance) else {
        return Ok(None);
    };
    expand_cylinder(tris, normals, adj, assigned, &mut cluster, &mut in_cluster, &cyl, tolerance, cos_ang)?;
    pts.clear();
    for &t in &cluster {
        if let Some(tri) = tris.get(t) {
            pts.extend_from_slice(tri);
        }
    }
    // Reject short latitude bands (common false positives on spheres): a real cylinder
    // wall spans a meaningful height compared with its radius.
    let (mut t_lo, mut t_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in &pts {
        let t = (*p - cyl.origin).dot(cyl.axis);
        t_lo = t_lo.min(t);
        t_hi = t_hi.max(t);
    }
    if !(t_hi - t_lo).is_finite() || t_hi - t_lo < cyl.radius * 0.25 {
        return Ok(None);
    }
    Ok(Some((cluster, cyl)))
}

fn fit_cylinder_pts(pts: &[Vec3], nrms: &[Vec3], axis_hint: Vec3, tolerance: f64) -> Option<CylFit> {
    let Some(&n0) = nrms.first() else { return None };
    // Wall–wall fan-out only (same band as grow_cylinder). Cap normals would yield a
    // circumferential “axis” and a nonsense radius.
    const MAX_WALL_DOT: f64 = 0.92;
    const MIN_WALL_DOT: f64 = 0.2;
    let far = nrms
        .iter()
        .copied()
        .filter(|n| {
            let d = n.dot(n0).abs();
            (MIN_WALL_DOT..=MAX_WALL_DOT).contains(&d)
        })
        .min_by(|a, b| a.dot(n0).abs().total_cmp(&b.dot(n0).abs()))
        .unwrap_or(n0);
    if far.dot(n0).abs() > MAX_WALL_DOT {
        return None;
    }
    let axis = n0.cross(far).normalized().unwrap_or(axis_hint);
    let axis = if axis.dot(axis_hint) < 0. { -axis } else { axis };
    if nrms.iter().any(|n| n.dot(axis).abs() > 0.15) {
        return None;
    }
    let u = axis.any_perp();
    let v = axis.cross(u);
    let pl: Vec<(f64, f64)> = pts.iter().map(|p| (p.dot(u), p.dot(v))).collect();
    let (cu, cv, r) = fit_circle_2d(&pl)?;
    if r < tolerance * 2. {
        return None;
    }
    let spread = pts.iter().map(|p| p.dist(pts[0])).fold(0.0_f64, f64::max);
    // Reject near-planar fits: radius must be comparable to the patch, not orders larger.
    if !spread.is_finite() || spread < tolerance || r > spread * 20. {
        return None;
    }
    // Same band the region grow uses, so a triangle the grow accepts cannot make the refit fail.
    if pl.iter().any(|(x, y)| ((x - cu).hypot(y - cv) - r).abs() > tolerance * 2.) {
        return None;
    }
    let origin = u * cu + v * cv;
    let axis = if axis.x + axis.y * 1e-3 + axis.z * 1e-6 < 0. { -axis } else { axis };
    Some(CylFit { axis, origin, radius: r })
}

fn fit_circle_2d(pts: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    let n = pts.len() as f64;
    if pts.len() < 3 {
        return None;
    }
    let (mx, my) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0 / n, a.1 + p.1 / n));
    let (mut suu, mut suv, mut svv, mut suuu, mut svvv, mut suvv, mut svuu) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for (x, y) in pts {
        let (u, v) = (x - mx, y - my);
        suu += u * u;
        suv += u * v;
        svv += v * v;
        suuu += u * u * u;
        svvv += v * v * v;
        suvv += u * v * v;
        svuu += v * u * u;
    }
    let det = suu * svv - suv * suv;
    if det.abs() < 1e-18 {
        return None;
    }
    let b1 = 0.5 * (suuu + suvv);
    let b2 = 0.5 * (svvv + svuu);
    let uc = (b1 * svv - b2 * suv) / det;
    let vc = (suu * b2 - suv * b1) / det;
    let r = (uc * uc + vc * vc + (suu + svv) / n).sqrt();
    if !r.is_finite() || r <= 0. {
        return None;
    }
    Some((uc + mx, vc + my, r))
}

#[derive(Clone, Copy)]
struct SphFit {
    center: Vec3,
    radius: f64,
}

/// Triangles whose vertices, centroid and normal match `sph` (rejects box corners on a
/// circumscribed sphere, where only vertices lie on the sphere).
fn claim_sphere_tris(tris: &[[Vec3; 3]], normals: &[Vec3], assigned: &[bool], sph: &SphFit, tolerance: f64) -> Vec<usize> {
    let mut all = Vec::new();
    for (ti, tri) in tris.iter().enumerate() {
        if assigned.get(ti).copied().unwrap_or(true) {
            continue;
        }
        let Some(nn) = normals.get(ti).copied() else { continue };
        let mid = (tri[0] + tri[1] + tri[2]) * (1. / 3.);
        let on_sphere = tri.iter().all(|p| (p.dist(sph.center) - sph.radius).abs() <= tolerance * 2.)
            && (mid.dist(sph.center) - sph.radius).abs() <= tolerance * 2.;
        let Some(radial) = (mid - sph.center).normalized() else { continue };
        if on_sphere && nn.dot(radial).abs() >= 0.85 {
            all.push(ti);
        }
    }
    all
}

fn grow_sphere(
    _mesh: &TriMesh,
    tris: &[[Vec3; 3]],
    normals: &[Vec3],
    adj: &[Vec<usize>],
    assigned: &[bool],
    seed: usize,
    tolerance: f64,
) -> Result<Option<(Vec<usize>, SphFit)>> {
    let Some(seed_tri) = tris.get(seed) else {
        return Ok(None);
    };
    let Some(n0) = normals.get(seed).copied() else {
        return Ok(None);
    };
    // Fine sphere facets stay nearly parallel over many rings — collect a neighbourhood,
    // fit, then prove sphericity via radial normals (planes fail this).
    let mut seen = HashSet::from([seed]);
    let mut frontier = vec![seed];
    let mut seed_tris = vec![seed];
    let mut pts: Vec<Vec3> = seed_tri.iter().copied().collect();
    for _depth in 0..24 {
        let mut next = Vec::new();
        for &ti in &frontier {
            for &nb in adj.get(ti).into_iter().flatten() {
                if assigned.get(nb).copied().unwrap_or(true) || !seen.insert(nb) {
                    continue;
                }
                if let Some(tri) = tris.get(nb) {
                    pts.extend_from_slice(tri);
                }
                seed_tris.push(nb);
                next.push(nb);
                if seed_tris.len() >= 48 {
                    break;
                }
            }
            if seed_tris.len() >= 48 {
                break;
            }
        }
        if seed_tris.len() >= 48 || next.is_empty() {
            break;
        }
        frontier = next;
    }
    let Some(sph0) = fit_sphere_pts(&pts) else {
        return Ok(None);
    };
    let spread0 = pts.iter().map(|p| p.dist(pts[0])).fold(0.0_f64, f64::max);
    if sph0.radius < tolerance * 2. || sph0.radius > spread0 * 20. {
        return Ok(None);
    }
    // Neighbourhood normals must point at the fitted centre (rejects planes / sharp edges).
    let mut radial_ok = 0usize;
    let mut far_dot = 1.0_f64;
    for &t in &seed_tris {
        let Some(nn) = normals.get(t).copied() else { continue };
        far_dot = far_dot.min(nn.dot(n0).abs());
        let Some(tri) = tris.get(t) else { continue };
        let mid = (tri[0] + tri[1] + tri[2]) * (1. / 3.);
        let Some(radial) = (mid - sph0.center).normalized() else { continue };
        if nn.dot(radial).abs() >= 0.85 {
            radial_ok += 1;
        }
    }
    if radial_ok * 10 < seed_tris.len() * 8 || far_dot > 0.995 {
        return Ok(None);
    }
    let sph = sph0;
    let mut cluster = vec![seed];
    let mut in_cluster = HashSet::from([seed]);
    let mut stack = vec![seed];
    let mut steps = 0usize;
    while let Some(ti) = stack.pop() {
        steps += 1;
        if steps > MAX_CLUSTER_TRIS * 4 {
            return Err(invalid("spherical region grow exceeds its work budget"));
        }
        for &nb in adj.get(ti).into_iter().flatten() {
            if assigned.get(nb).copied().unwrap_or(true) || in_cluster.contains(&nb) {
                continue;
            }
            let Some(nn) = normals.get(nb).copied() else { continue };
            let Some(tri) = tris.get(nb) else { continue };
            let mid = (tri[0] + tri[1] + tri[2]) * (1. / 3.);
            if tri.iter().any(|p| (p.dist(sph.center) - sph.radius).abs() > tolerance * 2.)
                || (mid.dist(sph.center) - sph.radius).abs() > tolerance * 2.
            {
                continue;
            }
            let Some(radial) = (mid - sph.center).normalized() else { continue };
            if nn.dot(radial).abs() < 0.85 {
                continue;
            }
            in_cluster.insert(nb);
            cluster.push(nb);
            stack.push(nb);
        }
    }
    if cluster.len() < 8 {
        return Ok(None);
    }
    // Demand real spherical coverage: normals must fan well beyond a sharp edge (two planes).
    let min_fan = cluster.iter().filter_map(|&t| normals.get(t).copied()).map(|n| n.dot(n0).abs()).fold(1.0_f64, f64::min);
    if min_fan > 0.5 {
        return Ok(None);
    }
    let mut all = Vec::new();
    for &t in &cluster {
        if let Some(tri) = tris.get(t) {
            all.extend_from_slice(tri);
        }
    }
    let Some(sph) = fit_sphere_pts(&all) else {
        return Ok(None);
    };
    if all.iter().any(|p| (p.dist(sph.center) - sph.radius).abs() > tolerance * 2.) {
        return Ok(None);
    }
    // Centroids must sit on the sphere too (rejects box corners on a circumscribed sphere).
    for &t in &cluster {
        let Some(tri) = tris.get(t) else { continue };
        let mid = (tri[0] + tri[1] + tri[2]) * (1. / 3.);
        if (mid.dist(sph.center) - sph.radius).abs() > tolerance * 2. {
            return Ok(None);
        }
    }
    Ok(Some((cluster, sph)))
}

fn fit_sphere_pts(pts: &[Vec3]) -> Option<SphFit> {
    if pts.len() < 4 {
        return None;
    }
    let m = pts.iter().fold(Vec3::ZERO, |a, p| a + *p) / pts.len() as f64;
    let mut a = [[0.0f64; 3]; 3];
    let mut b = [0.0f64; 3];
    for p in pts {
        let q = *p - m;
        let r = q.dot(q);
        let row = [q.x, q.y, q.z];
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] += row[i] * row[j];
            }
            b[i] += row[i] * r;
        }
    }
    let det = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(a);
    if d.abs() < 1e-18 {
        return None;
    }
    let mut c = [0.0; 3];
    for (k, o) in c.iter_mut().enumerate() {
        let mut mk = a;
        for (row, bv) in mk.iter_mut().zip(b) {
            row[k] = bv;
        }
        *o = det(mk) / d;
    }
    let center = m + Vec3::new(c[0], c[1], c[2]) * 0.5;
    if !center.is_finite() {
        return None;
    }
    let radius = pts.iter().map(|p| p.dist(center)).sum::<f64>() / pts.len() as f64;
    if !radius.is_finite() || radius <= 0. {
        return None;
    }
    Some(SphFit { center, radius })
}

/// truck refuses a wire whose points sit about 1e-6 mm off the plane it fits.
const TRUCK_FLAT: f64 = 1e-7;

fn on_plane(p: Vec3, n: Vec3, d: f64) -> bool {
    (p.dot(n) - d).abs() <= TRUCK_FLAT
}

/// Truck refuses a plane when the boundary normal vanishes or a point is more than 1e-6 mm off.
/// Arcs stay: their control points are not the geometric midpoint, and a cap must keep them.
fn face_not_a_plane(face: &FaceSpec) -> bool {
    if face.loops.iter().flatten().any(|edge| matches!(edge, EdgeSpec::Arc { .. })) {
        return false;
    }
    let (nlen, dev) = face_plane_metrics(face);
    nlen < 1e-6 || dev > 1e-6
}

fn face_plane_metrics(face: &FaceSpec) -> (f64, f64) {
    let mut pts = Vec::new();
    for edge in face.loops.iter().flatten() {
        let (a, _) = match edge {
            EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => (*a, *b),
            EdgeSpec::Chain { ctrl, .. } => (ctrl.first().copied().unwrap_or(Vec3::ZERO), ctrl.last().copied().unwrap_or(Vec3::ZERO)),
        };
        pts.push(a);
        if let EdgeSpec::Arc { mid, .. } = edge {
            pts.push(*mid);
        }
    }
    if pts.len() < 3 {
        return (0.0, 0.0);
    }
    let c = pts.iter().fold(Vec3::ZERO, |sum, p| sum + *p) * (1.0 / pts.len() as f64);
    let mut n = Vec3::ZERO;
    for pair in pts.windows(2) {
        let (Some(a), Some(b)) = (pair.first(), pair.get(1)) else { continue };
        n += (*a - c).cross(*b - c);
    }
    if let (Some(a), Some(b)) = (pts.first(), pts.last()) {
        n += (*b - c).cross(*a - c);
    }
    let Some(nn) = n.normalized() else { return (0.0, 0.0) };
    let d = c.dot(nn);
    let dev = pts.iter().fold(0.0_f64, |max, p| max.max((p.dot(nn) - d).abs()));
    (n.len(), dev)
}

#[cfg(test)]
fn boundary_uses(faces: &[FaceSpec]) -> (usize, usize, usize) {
    let q = |p: Vec3| ((p.x * 1e6).round() as i64, (p.y * 1e6).round() as i64, (p.z * 1e6).round() as i64);
    let mut uses: HashMap<(i64, i64, i64, i64, i64, i64), usize> = HashMap::new();
    for face in faces {
        for edge in face.loops.iter().flatten() {
            let (a, b) = match edge {
                EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => (*a, *b),
                EdgeSpec::Chain { ctrl, .. } => (ctrl.first().copied().unwrap_or(Vec3::ZERO), ctrl.last().copied().unwrap_or(Vec3::ZERO)),
            };
            let (mut ka, mut kb) = (q(a), q(b));
            if kb < ka {
                std::mem::swap(&mut ka, &mut kb);
            }
            *uses.entry((ka.0, ka.1, ka.2, kb.0, kb.1, kb.2)).or_default() += 1;
        }
    }
    let once = uses.values().filter(|c| **c == 1).count();
    let over = uses.values().filter(|c| **c > 2).count();
    (once, over, uses.len())
}

fn edges_on_plane(edges: &[EdgeSpec], n: Vec3, d: f64) -> bool {
    edges.iter().all(|e| match e {
        EdgeSpec::Line { a, b } => on_plane(*a, n, d) && on_plane(*b, n, d),
        EdgeSpec::Arc { a, b, mid } => on_plane(*a, n, d) && on_plane(*b, n, d) && on_plane(*mid, n, d),
        EdgeSpec::Chain { ctrl, .. } => ctrl.iter().copied().all(|p| on_plane(p, n, d)),
    })
}

fn planar_face(
    mesh: &TriMesh,
    positions: &[Vec3],
    cluster: &[usize],
    normals: &[Vec3],
    _tolerance: f64,
    _circles: &HashMap<CircleKey, Vec<EdgeSpec>>,
    by_rim: &HashMap<Vec<u32>, Vec<EdgeSpec>>,
    keep_verts: &HashSet<u32>,
    plane: Option<PlaneFit>,
    used_rims: &mut HashSet<Vec<u32>>,
) -> Result<FaceSpec> {
    let at = |i: u32| positions.get(i as usize).copied();
    // Single triangle: keep the mesh winding exactly (no simplify / re-orient) so a closed
    // manifold triangulated shell sews without inventing gaps at shared edges.
    if let [ti] = *cluster {
        let tri = mesh.triangles.get(ti).ok_or_else(|| invalid("missing triangle"))?;
        let (Some(mut a), Some(mut b), Some(mut c)) = (at(tri[0]), at(tri[1]), at(tri[2])) else {
            return Err(invalid("missing vertex"));
        };
        // Snapping a corner onto several planes can flatten a triangle until truck finds no normal.
        if (b - a).cross(c - a).len() < 1e-6 {
            let orig = |i: u32| mesh.positions.get(i as usize).copied();
            if let (Some(oa), Some(ob), Some(oc)) = (orig(tri[0]), orig(tri[1]), orig(tri[2]))
                && (ob - oa).cross(oc - oa).len() >= 1e-6
            {
                a = oa;
                b = ob;
                c = oc;
            }
        }
        let edges = vec![EdgeSpec::Line { a, b }, EdgeSpec::Line { a: b, b: c }, EdgeSpec::Line { a: c, b: a }];
        return Ok(FaceSpec { loops: vec![edges], surface: SurfSpec::Plane, probe: None });
    }
    let loops = boundary_loops(mesh, cluster)?;
    if loops.is_empty() {
        return Err(invalid("planar region has no boundary"));
    }
    let plane = match plane {
        Some(p) => p,
        None => {
            let mut n = Vec3::ZERO;
            for &t in cluster {
                if let Some(nn) = normals.get(t) {
                    n += *nn;
                }
            }
            let Some(n) = n.normalized() else {
                return Err(invalid("planar region has no normal"));
            };
            let mut d = 0.0_f64;
            let mut count = 0.0_f64;
            for &t in cluster {
                let Some(tri) = mesh.triangles.get(t) else { continue };
                for &vi in tri {
                    if let Some(p) = at(vi) {
                        d += p.dot(n);
                        count += 1.;
                    }
                }
            }
            PlaneFit { n, d: d / count.max(1.) }
        }
    };
    let n = plane.n;
    // Snapped vertices are shared by every face that uses them. Projecting each face
    // again onto its own plane pulls a shared corner apart, so the shell does not close.
    // A fitted circle is used only when this loop is exactly a cylinder rim: any other
    // circle sits off the face (truck's plane test is ~1e-6 mm) and does not meet the wall.
    let mut edge_loops = Vec::new();
    let mut hit_rims = Vec::new();
    for lp in loops {
        let pts: Vec<Vec3> = lp.iter().filter_map(|&i| at(i)).collect();
        if pts.iter().any(|p| !on_plane(*p, n, plane.d)) {
            return Err(invalid("wire is not simple"));
        }
        if pts.len() < 3 {
            return Err(invalid("planar loop too short"));
        }
        // A loop that still touches a leftover triangle must keep that mesh edge.
        // Replacing it with a fitted circle would not meet the triangle.
        let mut rim_key = lp.clone();
        rim_key.sort_unstable();
        let touches_leftover = lp.iter().any(|i| keep_verts.contains(i));
        let from_rim = if touches_leftover { None } else { by_rim.get(&rim_key).filter(|edges| edges_on_plane(edges, n, plane.d)).cloned() };
        let edges = if let Some(edges) = from_rim {
            hit_rims.push(rim_key);
            edges
        } else if touches_leftover {
            let keep: Vec<bool> = lp.iter().map(|i| keep_verts.contains(i)).collect();
            polyline_to_lines_keep(&pts, &keep, 0.0)?
        } else {
            // Keep every boundary vertex. Dropping a point within the conversion tolerance
            // opens a gap against a neighbouring triangle that still uses that edge.
            polyline_to_lines(&pts, 0.0)?
        };
        if edges.len() > MAX_LOOP_EDGES {
            return Err(invalid("planar loop exceeds the edge budget"));
        }
        if !loop_is_simple(&edges) || !edges_on_plane(&edges, n, plane.d) {
            return Err(invalid("wire is not simple"));
        }
        edge_loops.push(edges);
    }
    if !boundaries_disjoint(&edge_loops) {
        return Err(invalid("wire is not simple"));
    }
    used_rims.extend(hit_rims);
    // Ensure outer loop is CCW about outward normal; holes CW.
    orient_planar_loops(&mut edge_loops, n);
    Ok(FaceSpec { loops: edge_loops, surface: SurfSpec::Plane, probe: None })
}

fn boundaries_disjoint(loops: &[Vec<EdgeSpec>]) -> bool {
    if loops.len() < 2 {
        return true;
    }
    let mut pts: Vec<Vec<Vec3>> = Vec::new();
    for lp in loops {
        let mut loop_pts = Vec::new();
        let mut seen = HashSet::new();
        for e in lp {
            let a = match e {
                EdgeSpec::Line { a, .. } | EdgeSpec::Arc { a, .. } => *a,
                EdgeSpec::Chain { ctrl, .. } => ctrl.first().copied().unwrap_or(Vec3::ZERO),
            };
            let k = ((a.x * 1e6).round() as i64, (a.y * 1e6).round() as i64, (a.z * 1e6).round() as i64);
            if !seen.insert(k) {
                return false;
            }
            loop_pts.push(a);
        }
        pts.push(loop_pts);
    }
    // truck rejects a face whose wires share a vertex. Welding reaches 1e-6 mm, and a
    // point just across a 1e-6 cell border still joins, so the gap has to clear 2e-6.
    for i in 0..pts.len() {
        for j in (i + 1)..pts.len() {
            for a in &pts[i] {
                for b in &pts[j] {
                    if a.dist(*b) < 2e-6 {
                        return false;
                    }
                }
            }
        }
    }
    true
}

fn loop_is_simple(edges: &[EdgeSpec]) -> bool {
    if edges.len() < 3 {
        return false;
    }
    let mut pts = Vec::with_capacity(edges.len());
    let mut seen = HashSet::new();
    for e in edges {
        let (a, b) = match e {
            EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => (*a, *b),
            EdgeSpec::Chain { ctrl, .. } => (ctrl.first().copied().unwrap_or(Vec3::ZERO), ctrl.last().copied().unwrap_or(Vec3::ZERO)),
        };
        if a.dist(b) < 1e-6 {
            return false;
        }
        let k = ((a.x * 1e6).round() as i64, (a.y * 1e6).round() as i64, (a.z * 1e6).round() as i64);
        if !seen.insert(k) {
            return false;
        }
        pts.push(a);
    }
    let n = pts.len();
    // truck welds ends a hair over 1e-6 mm apart and then rejects a wire that touches itself.
    // A vertex on a non-adjacent edge, or two edges that cross, is that touch.
    for i in 0..n {
        for j in (i + 1)..n {
            let closing = i == 0 && j + 1 == n;
            let adjacent = j == i + 1 || closing;
            if adjacent {
                continue;
            }
            if pts[i].dist(pts[j]) < 2e-6 {
                return false;
            }
        }
    }
    for i in 0..n {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        for j in 0..n {
            if j == i || j == (i + 1) % n {
                continue;
            }
            if point_line_dist(pts[j], a, b) <= 1e-6 {
                return false;
            }
        }
        for j in (i + 2)..n {
            if i == 0 && j + 1 == n {
                continue;
            }
            let (c, d) = (pts[j], pts[(j + 1) % n]);
            if segments_cross(a, b, c, d) {
                return false;
            }
        }
    }
    true
}

fn segments_cross(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> bool {
    let ab_c = (b - a).cross(c - a);
    let ab_d = (b - a).cross(d - a);
    if ab_c.dot(ab_d) >= 0.0 {
        return false;
    }
    let cd_a = (d - c).cross(a - c);
    let cd_b = (d - c).cross(b - c);
    cd_a.dot(cd_b) < 0.0
}

fn polyline_to_lines(pts: &[Vec3], tolerance: f64) -> Result<Vec<EdgeSpec>> {
    polyline_to_lines_keep(pts, &[], tolerance)
}

fn polyline_to_lines_keep(pts: &[Vec3], keep: &[bool], tolerance: f64) -> Result<Vec<EdgeSpec>> {
    let simplified = simplify_polyline_keep(pts, keep, tolerance);
    if simplified.len() < 3 {
        return Err(invalid("loop collapsed while simplifying"));
    }
    let n = simplified.len();
    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let a = simplified[i];
        let b = simplified[(i + 1) % n];
        if a.dist(b) > 1e-12 {
            edges.push(EdgeSpec::Line { a, b });
        }
    }
    if edges.len() < 3 {
        return Err(invalid("planar loop has fewer than 3 edges"));
    }
    Ok(edges)
}

struct HeldPlane {
    cluster: Vec<usize>,
    n: Vec3,
    d: f64,
    locked: bool,
}

struct PendingTube {
    cluster: Vec<usize>,
    cyl: CylFit,
    h0: f64,
    h1: f64,
    rim0: Vec<u32>,
    rim1: Vec<u32>,
}

fn fit_cluster_plane(mesh: &TriMesh, normals: &[Vec3], cluster: &[usize]) -> Option<PlaneFit> {
    let mut n = Vec3::ZERO;
    let mut c = Vec3::ZERO;
    let mut w = 0.0;
    for &t in cluster {
        let nn = normals.get(t).copied()?;
        let tri = mesh.triangles.get(t)?;
        let (pa, pb, pc) = (mesh.positions.get(tri[0] as usize)?, mesh.positions.get(tri[1] as usize)?, mesh.positions.get(tri[2] as usize)?);
        let area = (*pb - *pa).cross(*pc - *pa).len();
        if area <= 0. {
            continue;
        }
        n += nn * area;
        c += (*pa + *pb + *pc) * (area / 3.);
        w += area;
    }
    let n = n.normalized()?;
    Some(PlaneFit { n, d: c.dot(n) / w.max(1e-18) })
}

fn try_pending_tube(mesh: &TriMesh, cluster: &[usize], cyl: &CylFit, tolerance: f64) -> Result<Option<PendingTube>> {
    let loops = boundary_loops(mesh, cluster)?;
    if loops.len() != 2 {
        return Ok(None);
    }
    let mut rims = Vec::new();
    for lp in loops {
        let mut hsum = 0.0;
        let mut count = 0.0;
        for &i in &lp {
            let Some(p) = mesh.positions.get(i as usize) else { continue };
            hsum += (*p - cyl.origin).dot(cyl.axis);
            count += 1.;
        }
        if count < 6. {
            return Ok(None);
        }
        let h = hsum / count;
        let flat = lp.iter().all(|&i| mesh.positions.get(i as usize).is_some_and(|p| ((*p - cyl.origin).dot(cyl.axis) - h).abs() <= tolerance * 4.));
        if !h.is_finite() || !flat {
            return Ok(None);
        }
        rims.push((h, lp));
    }
    rims.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (Some((h0, rim0)), Some((h1, rim1))) = (rims.first().cloned(), rims.get(1).cloned()) else {
        return Ok(None);
    };
    if h1 - h0 <= tolerance {
        return Ok(None);
    }
    Ok(Some(PendingTube { cluster: cluster.to_vec(), cyl: *cyl, h0, h1, rim0, rim1 }))
}

fn plane_owners(mesh: &TriMesh, planes: &[HeldPlane]) -> Vec<Vec<usize>> {
    let mut owners = vec![Vec::new(); mesh.positions.len()];
    for (pi, plane) in planes.iter().enumerate() {
        for &ti in &plane.cluster {
            let Some(tri) = mesh.triangles.get(ti) else { continue };
            for &vi in tri {
                if let Some(slot) = owners.get_mut(vi as usize)
                    && !slot.contains(&pi)
                {
                    slot.push(pi);
                }
            }
        }
    }
    owners
}

/// Put the cap that owns this rim on the cylinder's circle. False when the cap is not
/// perpendicular to the axis, or it was already locked to a different circle.
fn lock_rim(planes: &mut [HeldPlane], owners: &[Vec<usize>], rim: &[u32], cyl: &CylFit, h: f64) -> bool {
    let mut votes: HashMap<usize, usize> = HashMap::new();
    for &vi in rim {
        if let Some(ids) = owners.get(vi as usize) {
            for &pi in ids {
                *votes.entry(pi).or_default() += 1;
            }
        }
    }
    let Some((&pi, &count)) = votes.iter().max_by_key(|(_, c)| *c) else { return false };
    if count * 2 < rim.len() {
        return false;
    }
    let Some(plane) = planes.get_mut(pi) else { return false };
    if plane.n.dot(cyl.axis).abs() < 0.95 {
        return false;
    }
    let n = if plane.n.dot(cyl.axis) >= 0. { cyl.axis } else { -cyl.axis };
    let center = cyl.origin + cyl.axis * h;
    let d = n.dot(center);
    if plane.locked && (plane.n.dot(n) < 0.999 || (plane.d - d).abs() > 1e-4) {
        return false;
    }
    plane.n = n;
    plane.d = d;
    plane.locked = true;
    true
}

fn snapped_positions(mesh: &TriMesh, planes: &[HeldPlane]) -> Vec<Vec3> {
    let owners = plane_owners(mesh, planes);
    mesh.positions
        .iter()
        .enumerate()
        .map(|(vi, p)| {
            let ids = owners.get(vi).map(|s| s.as_slice()).unwrap_or(&[]);
            let mut x = *p;
            if ids.is_empty() {
                return x;
            }
            for _ in 0..32 {
                let mut moved = false;
                for &pi in ids {
                    let Some(plane) = planes.get(pi) else { continue };
                    let delta = x.dot(plane.n) - plane.d;
                    if delta.abs() > 1e-12 {
                        x -= plane.n * delta;
                        moved = true;
                    }
                }
                if !moved {
                    break;
                }
            }
            x
        })
        .collect()
}

fn cluster_on_plane(mesh: &TriMesh, cluster: &[usize], n: Vec3, d: f64, tolerance: f64) -> bool {
    cluster.iter().all(|&ti| {
        mesh.triangles
            .get(ti)
            .is_some_and(|tri| tri.iter().all(|&vi| mesh.positions.get(vi as usize).is_some_and(|p| (p.dot(n) - d).abs() <= tolerance)))
    })
}

/// Join plane patches that share an edge and lie within the conversion tolerance, so snapping
/// has one equation per face. A locked cap absorbs a patch only when that patch already lies
/// inside the conversion tolerance of the cap; two different locked caps stay apart.
fn merge_coplanar_planes(mesh: &TriMesh, normals: &[Vec3], planes: &mut Vec<HeldPlane>, tolerance: f64) {
    let n = planes.len();
    if n < 2 {
        return;
    }
    let owners = plane_owners(mesh, planes);
    let mut shared: HashMap<(usize, usize), u32> = HashMap::new();
    for ids in &owners {
        for a in 0..ids.len() {
            for b in (a + 1)..ids.len() {
                let (Some(&i), Some(&j)) = (ids.get(a), ids.get(b)) else { continue };
                *shared.entry((i.min(j), i.max(j))).or_default() += 1;
            }
        }
    }
    // One shared vertex is a corner, and joining there pinches the wire. An edge is two.
    let shares_edge = |i: usize, j: usize| shared.get(&(i.min(j), i.max(j))).copied().unwrap_or(0) >= 2;
    let aligned = |planes: &[HeldPlane], i: usize, j: usize| planes.get(i).zip(planes.get(j)).is_some_and(|(a, b)| a.n.dot(b.n).abs() >= 0.965);
    // A patch that already lies on a locked cap is absorbed whole. Don't union it into a
    // neighbouring face first, or the cap keeps a shared vertex it can no longer reach.
    let mut pinned = vec![false; n];
    for ids in &owners {
        for a in 0..ids.len() {
            for b in (a + 1)..ids.len() {
                let (Some(&i), Some(&j)) = (ids.get(a), ids.get(b)) else { continue };
                if !shares_edge(i, j) || !aligned(planes, i, j) {
                    continue;
                }
                let (Some(pi), Some(pj)) = (planes.get(i), planes.get(j)) else { continue };
                let (locked, free) = if pi.locked && !pj.locked {
                    (i, j)
                } else if pj.locked && !pi.locked {
                    (j, i)
                } else {
                    continue;
                };
                let on_cap = planes
                    .get(free)
                    .zip(planes.get(locked))
                    .is_some_and(|(free_p, lock)| cluster_on_plane(mesh, &free_p.cluster, lock.n, lock.d, tolerance));
                if on_cap && let Some(flag) = pinned.get_mut(free) {
                    *flag = true;
                }
            }
        }
    }
    let root_of = |parent: &[usize], mut i: usize| -> usize {
        let mut guard = 0usize;
        while parent.get(i).is_some_and(|p| *p != i) && guard < parent.len() {
            i = parent[i];
            guard += 1;
        }
        i
    };
    let mut parent: Vec<usize> = (0..n).collect();
    for ids in &owners {
        for a in 0..ids.len() {
            for b in (a + 1)..ids.len() {
                let (Some(&i), Some(&j)) = (ids.get(a), ids.get(b)) else { continue };
                if !shares_edge(i, j)
                    || pinned.get(i).copied().unwrap_or(false)
                    || pinned.get(j).copied().unwrap_or(false)
                    || planes.get(i).is_some_and(|p| p.locked)
                    || planes.get(j).is_some_and(|p| p.locked)
                    || !aligned(planes, i, j)
                {
                    continue;
                }
                let near = planes.get(i).zip(planes.get(j)).is_some_and(|(pi, pj)| {
                    cluster_on_plane(mesh, &pj.cluster, pi.n, pi.d, tolerance) || cluster_on_plane(mesh, &pi.cluster, pj.n, pj.d, tolerance)
                });
                if !near {
                    continue;
                }
                let (ri, rj) = (root_of(&parent, i), root_of(&parent, j));
                if ri != rj {
                    parent[rj] = ri;
                }
            }
        }
    }
    let mut groups: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in 0..n {
        if planes.get(i).is_some_and(|p| !p.locked) {
            groups[root_of(&parent, i)].push(i);
        }
    }
    // -1 none, -2 two different caps, otherwise the locked plane index.
    let mut absorb_into = vec![-1i32; n];
    for ids in &owners {
        for a in 0..ids.len() {
            for b in (a + 1)..ids.len() {
                let (Some(&i), Some(&j)) = (ids.get(a), ids.get(b)) else { continue };
                if !shares_edge(i, j) || !aligned(planes, i, j) {
                    continue;
                }
                let (Some(pi), Some(pj)) = (planes.get(i), planes.get(j)) else { continue };
                let (locked_i, free_i) = if pi.locked && !pj.locked {
                    (i, j)
                } else if pj.locked && !pi.locked {
                    (j, i)
                } else {
                    continue;
                };
                let root = root_of(&parent, free_i);
                let slot = absorb_into.get_mut(root);
                let Some(slot) = slot else { continue };
                if *slot == -1 {
                    *slot = locked_i as i32;
                } else if *slot != locked_i as i32 {
                    *slot = -2;
                }
            }
        }
    }
    let mut old = std::mem::take(planes);
    let mut skip = vec![false; n];
    let mut extras: Vec<(usize, Vec<usize>)> = Vec::new();
    let mut refit_groups: Vec<Vec<usize>> = Vec::new();
    for root in 0..n {
        let Some(members) = groups.get(root) else { continue };
        if members.is_empty() {
            continue;
        }
        let target = absorb_into.get(root).copied().unwrap_or(-1);
        let can_absorb = target >= 0 && {
            let t = target as usize;
            old.get(t).is_some_and(|lock| {
                members.iter().all(|&m| old.get(m).is_some_and(|p| cluster_on_plane(mesh, &p.cluster, lock.n, lock.d, tolerance)))
            })
        };
        if can_absorb {
            extras.push((target as usize, members.clone()));
        } else if members.len() > 1 {
            refit_groups.push(members.clone());
            for &m in members {
                if let Some(flag) = skip.get_mut(m) {
                    *flag = true;
                }
            }
        }
    }
    for (locked, members) in extras {
        for m in members {
            if let Some(flag) = skip.get_mut(m) {
                *flag = true;
            }
            let cluster = match old.get_mut(m) {
                Some(src) => std::mem::take(&mut src.cluster),
                None => Vec::new(),
            };
            if let Some(dst) = old.get_mut(locked) {
                dst.cluster.extend(cluster);
            }
        }
    }
    let mut merged_clusters: Vec<Vec<usize>> = Vec::new();
    for members in &refit_groups {
        let mut cluster = Vec::new();
        let mut seen = HashSet::new();
        for &m in members {
            if let Some(src) = old.get(m) {
                for &t in &src.cluster {
                    if seen.insert(t) {
                        cluster.push(t);
                    }
                }
            }
        }
        merged_clusters.push(cluster);
    }
    let mut rebuilt = Vec::with_capacity(n);
    for (i, mut plane) in old.into_iter().enumerate() {
        if skip.get(i).copied().unwrap_or(false) {
            continue;
        }
        let mut seen = HashSet::new();
        plane.cluster.retain(|t| seen.insert(*t));
        if plane.cluster.is_empty() {
            continue;
        }
        rebuilt.push(plane);
    }
    for cluster in merged_clusters {
        let Some(fit) = fit_cluster_plane(mesh, normals, &cluster) else { continue };
        rebuilt.push(HeldPlane { cluster, n: fit.n, d: fit.d, locked: false });
    }
    *planes = rebuilt;
}

fn cylinder_face(
    mesh: &TriMesh,
    cluster: &[usize],
    cyl: &CylFit,
    tolerance: f64,
    circles: &mut HashMap<CircleKey, Vec<EdgeSpec>>,
) -> Result<Vec<FaceSpec>> {
    let loops = boundary_loops(mesh, cluster)?;
    if loops.is_empty() {
        return Err(invalid("cylindrical region has no boundary"));
    }
    // Only a wall with two flat circular rims. A partial patch does not share edges with
    // its neighbours; emitting it used to reject every other analytic face as well.
    if loops.len() != 2 {
        return Err(invalid("cylindrical region is not a tube"));
    }
    let mut rims = Vec::new();
    for lp in &loops {
        let pts: Vec<Vec3> = lp.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()).collect();
        if pts.len() < 6 {
            return Err(invalid("cylindrical region is not a tube"));
        }
        let h = pts.iter().map(|p| (*p - cyl.origin).dot(cyl.axis)).sum::<f64>() / pts.len() as f64;
        if !h.is_finite() || pts.iter().any(|p| ((*p - cyl.origin).dot(cyl.axis) - h).abs() > tolerance * 4.) {
            return Err(invalid("cylindrical region is not a tube"));
        }
        rims.push(h);
    }
    rims.sort_by(|a, b| a.total_cmp(b));
    let (Some(h0), Some(h1)) = (rims.first().copied(), rims.get(1).copied()) else {
        return Err(invalid("cylindrical region is not a tube"));
    };
    let height = h1 - h0;
    if height <= tolerance {
        return Err(invalid("cylindrical region is not a tube"));
    }
    let bot = register_circle(circles, cyl.axis, cyl.origin + cyl.axis * h0, cyl.radius, 0.3)?;
    // The far rim is registered so the neighbouring plane reuses these same arcs.
    let _top = register_circle(circles, cyl.axis, cyl.origin + cyl.axis * h1, cyl.radius, 0.3)?;
    half_cylinder_walls(&bot, cyl.origin, cyl.axis * height, material_away_from_axis(mesh, cluster, cyl))
}

/// Quantized key so cap holes and cylinder walls share one circle EdgeSpec list.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct CircleKey {
    ax: i64,
    ay: i64,
    az: i64,
    cx: i64,
    cy: i64,
    cz: i64,
    r: i64,
}

fn circle_key(axis: Vec3, center: Vec3, radius: f64) -> CircleKey {
    let q = |x: f64| (x * 1e5).round() as i64;
    let axis = if axis.x + axis.y * 1e-3 + axis.z * 1e-6 < 0. { -axis } else { axis };
    CircleKey { ax: q(axis.x), ay: q(axis.y), az: q(axis.z), cx: q(center.x), cy: q(center.y), cz: q(center.z), r: q(radius) }
}

/// Full circle about `axis` through `center` as `n_arcs` equal arcs (`n_arcs` ≥ 2).
fn exact_circle_edges_n(center: Vec3, axis: Vec3, radius: f64, phase: f64, n_arcs: usize) -> Option<Vec<EdgeSpec>> {
    let Some(axis) = axis.normalized() else { return None };
    if !(radius.is_finite() && radius > 0.) || n_arcs < 2 || n_arcs > 64 {
        return None;
    }
    let u = axis.any_perp();
    let v = axis.cross(u);
    let p = |a: f64| center + (u * a.cos() + v * a.sin()) * radius;
    let step = TAU / n_arcs as f64;
    let mut edges = Vec::with_capacity(n_arcs);
    for i in 0..n_arcs {
        let a0 = phase + step * i as f64;
        edges.push(EdgeSpec::Arc { a: p(a0), b: p(a0 + step), mid: p(a0 + step * 0.5) });
    }
    Some(edges)
}

fn exact_circle_edges(center: Vec3, axis: Vec3, radius: f64, phase: f64) -> Option<Vec<EdgeSpec>> {
    // Four quarter-arcs keep each Extruded wall well under a semicircle (tsweep + mesh).
    exact_circle_edges_n(center, axis, radius, phase, 4)
}

fn reverse_edges(edges: &[EdgeSpec]) -> Vec<EdgeSpec> {
    let mut out = edges.to_vec();
    reverse_loop(&mut out);
    out
}

fn cylinder_height_range(mesh: &TriMesh, cluster: &[usize], cyl: &CylFit) -> Result<(f64, f64)> {
    let mut t_lo = f64::INFINITY;
    let mut t_hi = f64::NEG_INFINITY;
    for &t in cluster {
        let tri = mesh.triangles.get(t).ok_or_else(|| invalid("missing triangle"))?;
        for &vi in tri {
            let p = mesh.positions.get(vi as usize).ok_or_else(|| invalid("missing vertex"))?;
            let h = (*p - cyl.origin).dot(cyl.axis);
            t_lo = t_lo.min(h);
            t_hi = t_hi.max(h);
        }
    }
    if !(t_hi - t_lo).is_finite() || t_hi - t_lo <= 0. {
        return Err(invalid("cylindrical region has no height"));
    }
    Ok((t_lo, t_hi))
}

/// True when mesh normals point away from the cylinder axis (a boss). A hole's normals
/// point toward the axis.
fn material_away_from_axis(mesh: &TriMesh, cluster: &[usize], cyl: &CylFit) -> bool {
    let mut score = 0.0_f64;
    for &t in cluster {
        let Some(tri) = mesh.triangles.get(t) else { continue };
        let (Some(pa), Some(pb), Some(pc)) =
            (mesh.positions.get(tri[0] as usize), mesh.positions.get(tri[1] as usize), mesh.positions.get(tri[2] as usize))
        else {
            continue;
        };
        let mid = (*pa + *pb + *pc) * (1. / 3.);
        let away = mid - cyl.origin - cyl.axis * (mid - cyl.origin).dot(cyl.axis);
        let Some(away) = away.normalized() else { continue };
        let Some(nn) = (*pb - *pa).cross(*pc - *pa).normalized() else { continue };
        score += nn.dot(away);
    }
    score >= 0.
}

/// A closed cylindrical can: claim every wall/cap triangle that fits `cyl`, then build
/// shared-circle FaceSpecs (or let the caller short-circuit to `crate::cylinder`).
fn try_full_cylinder_feature(
    mesh: &TriMesh,
    tris: &[[Vec3; 3]],
    normals: &[Vec3],
    assigned: &[bool],
    wall: &[usize],
    cyl: &CylFit,
    tolerance: f64,
    circles: &mut HashMap<CircleKey, Vec<EdgeSpec>>,
) -> Result<Option<(Vec<FaceSpec>, Vec<usize>)>> {
    let (t_lo, t_hi) = cylinder_height_range(mesh, wall, cyl)?;
    if t_hi - t_lo < tolerance * 2. {
        return Ok(None);
    }
    let h_lo = t_lo;
    let h_hi = t_hi;
    let away = material_away_from_axis(mesh, wall, cyl);

    // Claim all unassigned triangles that belong on this cylinder or its caps.
    let mut all = Vec::new();
    let mut wall_n = 0usize;
    let mut cap_lo = 0usize;
    let mut cap_hi = 0usize;
    for (ti, tri) in tris.iter().enumerate() {
        if assigned.get(ti).copied().unwrap_or(true) {
            continue;
        }
        let Some(nn) = normals.get(ti).copied() else { continue };
        let on_wall = tri.iter().all(|p| {
            let w = *p - cyl.origin;
            let radial = w - cyl.axis * w.dot(cyl.axis);
            (radial.len() - cyl.radius).abs() <= tolerance * 2.
        }) && nn.dot(cyl.axis).abs() < 0.35;
        let h_mid = tri.iter().map(|p| (*p - cyl.origin).dot(cyl.axis)).sum::<f64>() / 3.;
        let on_cap_lo = (h_mid - h_lo).abs() <= tolerance * 2.
            && nn.dot(-cyl.axis) > 0.85
            && tri.iter().all(|p| {
                let w = *p - cyl.origin;
                let radial = w - cyl.axis * w.dot(cyl.axis);
                radial.len() <= cyl.radius + tolerance * 3.
            });
        let on_cap_hi = (h_mid - h_hi).abs() <= tolerance * 2.
            && nn.dot(cyl.axis) > 0.85
            && tri.iter().all(|p| {
                let w = *p - cyl.origin;
                let radial = w - cyl.axis * w.dot(cyl.axis);
                radial.len() <= cyl.radius + tolerance * 3.
            });
        if on_wall {
            wall_n += 1;
            all.push(ti);
        } else if on_cap_lo {
            cap_lo += 1;
            all.push(ti);
        } else if on_cap_hi {
            cap_hi += 1;
            all.push(ti);
        }
    }
    if wall_n < 8 || cap_lo < 2 || cap_hi < 2 {
        return Ok(None);
    }
    // Refuse if many mesh triangles remain unexplained (not a pure can / boss we own).
    let unexplained = mesh.triangles.len().saturating_sub(all.len() + assigned.iter().filter(|a| **a).count());
    if unexplained > 0 && all.len() + assigned.iter().filter(|a| **a).count() < mesh.triangles.len() {
        // Still OK for a boss on a larger part — only require the can itself to be closed.
    }

    let bot_c = cyl.origin + cyl.axis * h_lo;
    let top_c = cyl.origin + cyl.axis * h_hi;
    let phase = crate::build::SEAM_ANGLE;
    let bot = register_circle(circles, cyl.axis, bot_c, cyl.radius, phase)?;
    let top = register_circle(circles, cyl.axis, top_c, cyl.radius, phase)?;
    let height = h_hi - h_lo;
    let dir = cyl.axis * height;
    let bottom_face = FaceSpec { loops: vec![reverse_edges(&bot)], surface: SurfSpec::Plane, probe: None };
    let top_face = FaceSpec { loops: vec![top.clone()], surface: SurfSpec::Plane, probe: None };
    let wall_faces = half_cylinder_walls(&bot, cyl.origin, dir, away)?;
    let mut feature = vec![bottom_face, top_face];
    feature.extend(wall_faces);
    let _ = mesh;
    Ok(Some((feature, all)))
}

/// Wall faces from each arc of a bottom circle. Each wall is a revolution of the vertical
/// generator through that arc. Extruded arcs lose their circular weights under `tsweep` and
/// measure about two thirds of the cylinder; a revolved generator stays on the true radius.
fn half_cylinder_walls(bot: &[EdgeSpec], axis_origin: Vec3, dir: Vec3, away_from_axis: bool) -> Result<Vec<FaceSpec>> {
    if bot.len() < 2 {
        return Err(invalid("cylinder circle must have at least two arcs"));
    }
    let axis = dir.normalized().unwrap_or(Vec3::Z);
    let angle = TAU / bot.len() as f64;
    let mut walls = Vec::with_capacity(bot.len());
    for arc in bot {
        let EdgeSpec::Arc { a, b, mid } = arc else {
            return Err(invalid("cylinder circle arcs"));
        };
        let (top_a, top_b, top_mid) = (*a + dir, *b + dir, *mid + dir);
        let m = (*mid + top_mid) * 0.5;
        let radial = m - axis_origin - axis * (m - axis_origin).dot(axis);
        let away = radial.normalized().unwrap_or(Vec3::X);
        let outward = if away_from_axis { away } else { -away };
        // Boss walls wind with the caps' outer circles. A hole's caps run those circles
        // the other way (the hole is opposite the plate's outward normal), so the wall
        // loop is reversed or the shell is not consistently oriented.
        let mut boundary = vec![
            EdgeSpec::Arc { a: *a, b: *b, mid: *mid },
            EdgeSpec::Line { a: *b, b: top_b },
            EdgeSpec::Arc { a: top_b, b: top_a, mid: top_mid },
            EdgeSpec::Line { a: top_a, b: *a },
        ];
        if !away_from_axis {
            reverse_loop(&mut boundary);
        }
        walls.push(FaceSpec {
            loops: vec![boundary],
            surface: SurfSpec::Revolved { a: *a, b: top_a, axis_origin, axis, angle },
            probe: Some((m, outward)),
        });
    }
    Ok(walls)
}

fn register_circle(circles: &mut HashMap<CircleKey, Vec<EdgeSpec>>, axis: Vec3, center: Vec3, radius: f64, phase: f64) -> Result<Vec<EdgeSpec>> {
    let key = circle_key(axis, center, radius);
    if let Some(existing) = circles.get(&key) {
        return Ok(existing.clone());
    }
    let edges = exact_circle_edges(center, axis, radius, phase).ok_or_else(|| invalid("bad circle"))?;
    circles.insert(key, edges.clone());
    Ok(edges)
}

#[allow(dead_code)] // geometric circle fit; caps share circles by rim vertex id instead
fn lookup_circle_for_loop(pts: &[Vec3], plane_n: Vec3, tolerance: f64, circles: &HashMap<CircleKey, Vec<EdgeSpec>>) -> Option<Vec<EdgeSpec>> {
    if pts.len() < 6 {
        return None;
    }
    // Prefer a circle the cylinder wall already registered, even when the algebraic
    // fit of these tessellation points drifts past the key quantum.
    let mut best: Option<(f64, Vec<EdgeSpec>)> = None;
    let q = |i: i64| i as f64 * 1e-5;
    for (k, edges) in circles {
        let axis = Vec3::new(q(k.ax), q(k.ay), q(k.az));
        let center = Vec3::new(q(k.cx), q(k.cy), q(k.cz));
        let radius = q(k.r);
        if axis.dot(plane_n).abs() < 0.95 || !(radius.is_finite() && radius > 0.) {
            continue;
        }
        let mut err = 0.0_f64;
        let mut on = true;
        for p in pts {
            let w = *p - center;
            let axial = w.dot(axis).abs();
            let radial = (w - axis * w.dot(axis)).len();
            err = err.max((radial - radius).abs().max(axial));
            if err > tolerance * 4. {
                on = false;
                break;
            }
        }
        if on && best.as_ref().is_none_or(|(e, _)| err < *e) {
            best = Some((err, edges.clone()));
        }
    }
    if let Some((_, edges)) = best {
        return Some(edges);
    }
    let c = pts.iter().fold(Vec3::ZERO, |a, p| a + *p) / pts.len() as f64;
    let u = plane_n.any_perp();
    let v = plane_n.cross(u);
    let pl: Vec<(f64, f64)> = pts.iter().map(|p| ((*p - c).dot(u), (*p - c).dot(v))).collect();
    let (cu, cv, r) = fit_circle_2d(&pl)?;
    let center = c + u * cu + v * cv;
    if pts.iter().any(|p| {
        let w = *p - center;
        let radial = w - plane_n * w.dot(plane_n);
        (radial.len() - r).abs() > tolerance * 2.
    }) {
        return None;
    }
    let key = circle_key(plane_n, center, r);
    if let Some(edges) = circles.get(&key).cloned().or_else(|| {
        let key2 = circle_key(-plane_n, center, r);
        circles.get(&key2).cloned()
    }) {
        return Some(edges);
    }
    // Tessellation shifts the fitted radius by more than the key quantum. Accept a
    // registered circle that still lies on this loop within tolerance.
    let q = |i: i64| i as f64 * 1e-5;
    circles.iter().find_map(|(k, edges)| {
        let axis = Vec3::new(q(k.ax), q(k.ay), q(k.az));
        let stored = Vec3::new(q(k.cx), q(k.cy), q(k.cz));
        let radius = q(k.r);
        if axis.dot(plane_n).abs() < 0.98 || (radius - r).abs() > tolerance * 2. {
            return None;
        }
        if (stored - center).len() > tolerance * 4. {
            return None;
        }
        Some(edges.clone())
    })
}

#[allow(dead_code)] // geometric circle fit; caps share circles by rim vertex id instead
fn exact_rim_from_points(pts: &[Vec3], axis: Vec3, tolerance: f64) -> Result<Vec<EdgeSpec>> {
    if pts.len() < 6 {
        return Err(invalid("not enough rim samples"));
    }
    let c = pts.iter().fold(Vec3::ZERO, |a, p| a + *p) / pts.len() as f64;
    let u = axis.any_perp();
    let v = axis.cross(u);
    let pl: Vec<(f64, f64)> = pts.iter().map(|p| ((*p - c).dot(u), (*p - c).dot(v))).collect();
    let (cu, cv, r) = fit_circle_2d(&pl).ok_or_else(|| invalid("rim is not circular"))?;
    let center = c + u * cu + v * cv;
    if r < tolerance * 2. {
        return Err(invalid("rim radius too small"));
    }
    if pts.iter().any(|p| {
        let w = *p - center;
        let radial = w - axis * w.dot(axis);
        (radial.len() - r).abs() > tolerance * 2.
    }) {
        return Err(invalid("rim leaves its circle"));
    }
    exact_circle_edges(center, axis, r, 0.3).ok_or_else(|| invalid("bad circle"))
}

#[allow(dead_code)] // partial spheres are faceted until their rims share edges with neighbours
fn sphere_face(mesh: &TriMesh, cluster: &[usize], sph: &SphFit, normals: &[Vec3], tolerance: f64) -> Result<FaceSpec> {
    let loops = boundary_loops(mesh, cluster)?;
    if loops.is_empty() {
        return Err(invalid("spherical region has no boundary"));
    }
    let mut edge_loops = Vec::new();
    for lp in loops {
        let pts: Vec<Vec3> = lp.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()).collect();
        let edges = sphere_loop_edges(&pts, sph, tolerance)?;
        if edges.len() > MAX_LOOP_EDGES {
            return Err(invalid("spherical loop exceeds the edge budget"));
        }
        edge_loops.push(edges);
    }
    let mut n = Vec3::ZERO;
    let mut c = Vec3::ZERO;
    let mut w: f64 = 0.;
    for &t in cluster {
        let Some(nn) = normals.get(t) else { continue };
        let Some(tri) = mesh.triangles.get(t) else { continue };
        let (Some(pa), Some(pb), Some(pc)) =
            (mesh.positions.get(tri[0] as usize), mesh.positions.get(tri[1] as usize), mesh.positions.get(tri[2] as usize))
        else {
            continue;
        };
        let mid = (*pa + *pb + *pc) * (1. / 3.);
        n += *nn;
        c += mid;
        w += 1.;
    }
    let probe_p = c * (1. / w.max(1.));
    let probe_n = n.normalized().unwrap_or_else(|| (probe_p - sph.center).normalized().unwrap_or(Vec3::Z));
    Ok(FaceSpec { loops: edge_loops, surface: SurfSpec::Sphere { center: sph.center, radius: sph.radius }, probe: Some((probe_p, probe_n)) })
}

/// Directed boundary loops (vertex indices) of a triangle cluster.
fn boundary_loops(mesh: &TriMesh, cluster: &[usize]) -> Result<Vec<Vec<u32>>> {
    let mut use_count: HashMap<(u32, u32), usize> = HashMap::new();
    let mut directed: Vec<(u32, u32)> = Vec::new();
    for &ti in cluster {
        let tri = mesh.triangles.get(ti).ok_or_else(|| invalid("missing triangle"))?;
        let [a, b, c] = *tri;
        for (u, v) in [(a, b), (b, c), (c, a)] {
            *use_count.entry((u.min(v), u.max(v))).or_insert(0) += 1;
            directed.push((u, v));
        }
    }
    let mut remaining: HashMap<u32, Vec<u32>> = HashMap::new();
    for (u, v) in directed {
        let key = (u.min(v), u.max(v));
        if use_count.get(&key).copied().unwrap_or(0) == 1 {
            remaining.entry(u).or_default().push(v);
        }
    }
    let mut loops = Vec::new();
    let mut steps = 0usize;
    while let Some((&start, outs)) = remaining.iter_mut().find(|(_, v)| !v.is_empty()) {
        let Some(next0) = outs.pop() else { break };
        let mut loop_v = vec![start, next0];
        let mut cur = next0;
        while cur != start {
            steps += 1;
            if steps > MAX_CLUSTER_TRIS * 8 {
                return Err(invalid("boundary walk exceeds its work budget"));
            }
            let outs = remaining.entry(cur).or_default();
            let Some(next) = outs.pop() else {
                return Err(invalid("open boundary in a face region"));
            };
            loop_v.push(next);
            cur = next;
            if loop_v.len() > MAX_LOOP_EDGES + 2 {
                return Err(invalid("boundary loop exceeds the edge budget"));
            }
        }
        loop_v.pop(); // duplicate start
        if loop_v.len() >= 3 {
            loops.push(loop_v);
        }
        if loops.len() > 64 {
            return Err(invalid("too many loops on one face"));
        }
    }
    Ok(loops)
}

fn polyline_to_edges(pts: &[Vec3], tolerance: f64, plane_n: Option<Vec3>) -> Result<Vec<EdgeSpec>> {
    let simplified = simplify_polyline(pts, tolerance);
    if simplified.len() < 3 {
        return Err(invalid("loop collapsed while simplifying"));
    }
    // Try a full circle when coplanar and circular.
    if let Some(n) = plane_n
        && let Some(edges) = try_full_circle(&simplified, n, tolerance)
    {
        return Ok(edges);
    }
    // Greedy: merge runs into arcs when a circle fits, else lines.
    let mut edges = Vec::new();
    let n = simplified.len();
    let mut i = 0;
    while i < n {
        let a = simplified[i];
        // Try longest arc from i.
        let mut best_arc: Option<(usize, Vec3)> = None;
        for len in (3..=n.min(i + n)).rev() {
            if len < 3 {
                break;
            }
            let mut run = Vec::with_capacity(len);
            for k in 0..len {
                run.push(simplified[(i + k) % n]);
            }
            // Do not wrap past start more than once.
            if i + len > n && i != 0 {
                continue;
            }
            if let Some(mid) = fit_arc_mid(&run, tolerance) {
                best_arc = Some((len, mid));
                break;
            }
        }
        if let Some((len, mid)) = best_arc {
            let b = simplified[(i + len - 1) % n];
            edges.push(EdgeSpec::Arc { a, b, mid });
            i += len - 1;
            if i >= n {
                break;
            }
            continue;
        }
        let b = simplified[(i + 1) % n];
        edges.push(EdgeSpec::Line { a, b });
        i += 1;
        if edges.len() > MAX_LOOP_EDGES {
            return Err(invalid("loop exceeds the edge budget"));
        }
    }
    // Closing: last edge already connects to first when we walked all points as segments.
    // When using arcs that skip points, ensure we closed the loop.
    if let (Some(first), Some(last)) = (edges.first(), edges.last()) {
        let start = match first {
            EdgeSpec::Line { a, .. } | EdgeSpec::Arc { a, .. } => *a,
            EdgeSpec::Chain { ctrl, .. } => ctrl.first().copied().unwrap_or(Vec3::ZERO),
        };
        let end = match last {
            EdgeSpec::Line { b, .. } | EdgeSpec::Arc { b, .. } => *b,
            EdgeSpec::Chain { ctrl, .. } => ctrl.last().copied().unwrap_or(Vec3::ZERO),
        };
        if end.dist(start) > tolerance * 10. {
            edges.push(EdgeSpec::Line { a: end, b: start });
        }
    }
    Ok(edges)
}

#[allow(dead_code)]
fn sphere_loop_edges(pts: &[Vec3], sph: &SphFit, tolerance: f64) -> Result<Vec<EdgeSpec>> {
    let simplified = simplify_polyline(pts, tolerance);
    if simplified.len() < 3 {
        return Err(invalid("sphere loop collapsed"));
    }
    // Prefer circular arcs on the sphere (plane ∩ sphere).
    polyline_to_edges(&simplified, tolerance, None).or_else(|_| {
        // Fallback: chordal lines snapped onto the sphere midpoints as arcs when possible.
        let mut edges = Vec::new();
        let n = simplified.len();
        for i in 0..n {
            let a = simplified[i];
            let b = simplified[(i + 1) % n];
            let mid = ((a + b) * 0.5 - sph.center).normalized().map(|d| sph.center + d * sph.radius).unwrap_or((a + b) * 0.5);
            if (mid.dist(sph.center) - sph.radius).abs() <= tolerance * 2. && a.dist(b) > tolerance {
                edges.push(EdgeSpec::Arc { a, b, mid });
            } else {
                edges.push(EdgeSpec::Line { a, b });
            }
        }
        Ok(edges)
    })
}

fn simplify_polyline(pts: &[Vec3], tolerance: f64) -> Vec<Vec3> {
    simplify_polyline_keep(pts, &[], tolerance)
}

fn simplify_polyline_keep(pts: &[Vec3], keep: &[bool], tolerance: f64) -> Vec<Vec3> {
    if pts.len() <= 3 {
        return pts.to_vec();
    }
    let mut out = pts.to_vec();
    let mut hold: Vec<bool> = if keep.len() == pts.len() { keep.to_vec() } else { vec![false; pts.len()] };
    let linear_tol = tolerance.max(1e-9);
    loop {
        let n = out.len();
        if n <= 3 {
            break;
        }
        let remove = (0..n).find(|&i| {
            let (a, b, c) = (out[(i + n - 1) % n], out[i], out[(i + 1) % n]);
            // A snapped duplicate is not an edge, even when a leftover triangle names that vertex.
            if b.dist(a) < 1e-6 || b.dist(c) < 1e-6 {
                return true;
            }
            if hold.get(i).copied().unwrap_or(false) {
                return false;
            }
            point_line_dist(b, a, c) <= linear_tol
        });
        let Some(index) = remove else { break };
        out.remove(index);
        hold.remove(index);
    }
    out
}

fn point_line_dist(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let len2 = ab.len2();
    if len2 < 1e-30 {
        return p.dist(a);
    }
    let t = ((p - a).dot(ab) / len2).clamp(0., 1.);
    (a + ab * t).dist(p)
}

fn try_full_circle(pts: &[Vec3], n: Vec3, tolerance: f64) -> Option<Vec<EdgeSpec>> {
    if pts.len() < 8 {
        return None;
    }
    let c = pts.iter().fold(Vec3::ZERO, |a, p| a + *p) / pts.len() as f64;
    let u = n.any_perp();
    let v = n.cross(u);
    let pl: Vec<(f64, f64)> = pts.iter().map(|p| ((*p - c).dot(u), (*p - c).dot(v))).collect();
    let (cu, cv, r) = fit_circle_2d(&pl)?;
    if r < tolerance * 2. {
        return None;
    }
    let center = c + u * cu + v * cv;
    if pts.iter().any(|p| {
        let w = *p - center;
        let radial = w - n * w.dot(n);
        (radial.len() - r).abs() > tolerance
    }) {
        return None;
    }
    // Two half-arcs.
    let x = (pts[0] - center) - n * (pts[0] - center).dot(n);
    let Some(x) = x.normalized() else { return None };
    let y = n.cross(x);
    let a = center + x * r;
    let b = center - x * r;
    let mid1 = center + y * r;
    let mid2 = center - y * r;
    Some(vec![EdgeSpec::Arc { a, b, mid: mid1 }, EdgeSpec::Arc { a: b, b: a, mid: mid2 }])
}

fn fit_arc_mid(pts: &[Vec3], tolerance: f64) -> Option<Vec3> {
    // Three points always define a circle; demand a real bend and enough samples so a
    // rectangle's corners are not turned into arcs that break sewing against straight walls.
    if pts.len() < 5 {
        return None;
    }
    let a = *pts.first()?;
    let mid_i = pts.len() / 2;
    let mid = *pts.get(mid_i)?;
    let b = *pts.last()?;
    if point_line_dist(mid, a, b) <= tolerance * 2. {
        return None;
    }
    let (centre, radius) = circumcentre_radius(a, mid, b)?;
    if radius < tolerance * 2. {
        return None;
    }
    if pts.iter().any(|p| (p.dist(centre) - radius).abs() > tolerance) {
        return None;
    }
    Some(mid)
}

fn circumcentre_radius(a: Vec3, b: Vec3, c: Vec3) -> Option<(Vec3, f64)> {
    let ab = b - a;
    let ac = c - a;
    let n = ab.cross(ac);
    let n2 = n.len2();
    if n2 < 1e-24 {
        return None;
    }
    let ab2 = ab.len2();
    let ac2 = ac.len2();
    let to = n.cross(ab) * ac2 + ac.cross(n) * ab2;
    let centre = a + to * (1. / (2. * n2));
    if !centre.is_finite() {
        return None;
    }
    let radius = centre.dist(a);
    radius.is_finite().then_some((centre, radius))
}

fn orient_planar_loops(loops: &mut [Vec<EdgeSpec>], outward: Vec3) {
    if loops.is_empty() {
        return;
    }
    // Signed area proxy via Newell on edge endpoints.
    let area_n = |edges: &[EdgeSpec]| -> Vec3 {
        let mut pts = Vec::new();
        for e in edges {
            match e {
                EdgeSpec::Line { a, .. } | EdgeSpec::Arc { a, .. } => pts.push(*a),
                EdgeSpec::Chain { ctrl, .. } => {
                    if let Some(&p) = ctrl.first() {
                        pts.push(p);
                    }
                }
            }
        }
        let mut n = Vec3::ZERO;
        for i in 0..pts.len() {
            let p = pts[i];
            let q = pts[(i + 1) % pts.len()];
            n += p.cross(q);
        }
        n
    };
    // Outer = largest |area|; make it CCW about outward; others opposite.
    let mut areas: Vec<(usize, f64)> = loops.iter().enumerate().map(|(i, l)| (i, area_n(l).len())).collect();
    areas.sort_by(|a, b| b.1.total_cmp(&a.1));
    let Some(&(outer_i, _)) = areas.first() else { return };
    for (i, lp) in loops.iter_mut().enumerate() {
        let n = area_n(lp);
        let want_positive = i == outer_i;
        let positive = n.dot(outward) > 0.;
        if positive != want_positive {
            reverse_loop(lp);
        }
    }
}

fn reverse_loop(edges: &mut Vec<EdgeSpec>) {
    edges.reverse();
    for e in edges.iter_mut() {
        *e = match e {
            EdgeSpec::Line { a, b } => EdgeSpec::Line { a: *b, b: *a },
            EdgeSpec::Arc { a, b, mid } => EdgeSpec::Arc { a: *b, b: *a, mid: *mid },
            EdgeSpec::Chain { degree, ctrl } => {
                let mut c = ctrl.to_vec();
                c.reverse();
                EdgeSpec::Chain { degree: *degree, ctrl: c.into() }
            }
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use solidcraft_geom::{Loop2, Plane, Region2, Vec2};

    fn mesh_of(body: &Body) -> Body {
        let mesh = body.tessellate(0.02).unwrap();
        crate::mesh_body(&mesh.positions, &mesh.triangles).unwrap()
    }

    #[test]
    fn analytic_box_from_tessellation() {
        let source = crate::box_solid(Vec3::ZERO, Vec3::new(10., 20., 30.)).unwrap();
        let mesh = mesh_of(&source);
        let converted = convert_analytic_mesh(&mesh, 0.05, 15f64.to_radians()).unwrap_or_else(|e| panic!("box convert: {e}"));
        assert!(!converted.is_mesh());
        assert!(converted.validity().is_empty());
        let v = crate::measure(&converted).unwrap().volume;
        assert!((v - 6000.).abs() < 1., "{v}");
        assert_eq!(crate::measure(&converted).unwrap().merged.face_types.get("plane"), Some(&6));
    }

    #[test]
    fn cylinder_mesh_converts_via_fallback_chain() {
        let source = crate::cylinder(Vec3::ZERO, Vec3::Z, 5., 12.).unwrap();
        let converted = crate::convert_mesh(&mesh_of(&source), 0.05).unwrap();
        assert!(!converted.is_mesh());
        let expected = std::f64::consts::PI * 25. * 12.;
        let v = crate::measure(&converted).unwrap().volume;
        assert!((v - expected).abs() / expected < 0.05, "{v} vs {expected}");
    }

    #[test]
    fn analytic_cylinder_recovers_cylindrical_surface() {
        let source = crate::cylinder(Vec3::ZERO, Vec3::Z, 5., 12.).unwrap();
        let mesh_body = crate::mesh_convert::weld_mesh(mesh_of(&source).mesh.as_ref().unwrap(), 0.05).unwrap();
        let converted = convert_analytic_mesh(&mesh_body, 0.05, 15f64.to_radians()).expect("analytic cylinder");
        let types = crate::measure(&converted).unwrap().merged.face_types;
        assert!(types.get("cylinder").copied().unwrap_or(0) >= 1, "expected a cylinder face, got {types:?}");
        let expected = std::f64::consts::PI * 25. * 12.;
        let v = crate::measure(&converted).unwrap().volume;
        assert!((v - expected).abs() / expected < 0.02, "{v} vs {expected}");
    }

    #[test]
    fn analytic_sphere_recovers_spherical_surface() {
        let source = crate::sphere(Vec3::ZERO, 5.).unwrap();
        let mesh_body = crate::mesh_convert::weld_mesh(mesh_of(&source).mesh.as_ref().unwrap(), 0.05).unwrap();
        let converted = convert_analytic_mesh(&mesh_body, 0.05, 15f64.to_radians()).expect("analytic sphere");
        let types = crate::measure(&converted).unwrap().merged.face_types;
        assert!(types.get("sphere").copied().unwrap_or(0) >= 1, "expected a sphere face, got {types:?}");
        let expected = 4. / 3. * std::f64::consts::PI * 125.;
        let v = crate::measure(&converted).unwrap().volume;
        assert!((v - expected).abs() / expected < 0.05, "{v} vs {expected}");
    }

    #[test]
    fn exact_circle_cylinder_sews() {
        let r = 5.;
        let h = 12.;
        let bot = exact_circle_edges(Vec3::ZERO, Vec3::Z, r, crate::build::SEAM_ANGLE).unwrap();
        let top = exact_circle_edges(Vec3::new(0., 0., h), Vec3::Z, r, crate::build::SEAM_ANGLE).unwrap();
        let mut faces = vec![
            FaceSpec { loops: vec![reverse_edges(&bot)], surface: SurfSpec::Plane, probe: None },
            FaceSpec { loops: vec![top], surface: SurfSpec::Plane, probe: None },
        ];
        faces.extend(half_cylinder_walls(&bot, Vec3::ZERO, Vec3::Z * h, true).unwrap());
        assert!(faces.len() >= 4);
        let body = sew(&faces).expect("cylinder FaceSpecs must sew");
        let expected = PI * r * r * h;
        let v = crate::measure(&body).unwrap().volume;
        assert!((v - expected).abs() / expected < 0.01, "{v} vs {expected}");
    }

    #[test]
    fn analytic_plate_with_hole_keeps_a_cylinder() {
        let region = Region2 {
            outer: Loop2::polygon(&[Vec2::new(0., 0.), Vec2::new(40., 0.), Vec2::new(40., 30.), Vec2::new(0., 30.)]),
            holes: vec![Loop2::circle(Vec2::new(12., 12.), 4.).reversed()],
        };
        let source = crate::extrude_planar_regions(&Plane::XY, &[region], 0., 10.).unwrap();
        let mesh_body = crate::mesh_convert::weld_mesh(mesh_of(&source).mesh.as_ref().unwrap(), 0.05).unwrap();
        let converted = convert_analytic_mesh(&mesh_body, 0.05, 15f64.to_radians()).expect("plate with hole");
        let types = crate::measure(&converted).unwrap().merged.face_types;
        assert!(types.get("cylinder").copied().unwrap_or(0) >= 1, "expected a cylinder, got {types:?}");
        let expected = (1200. - std::f64::consts::PI * 16.) * 10.;
        let v = crate::measure(&converted).unwrap().volume;
        assert!((v - expected).abs() / expected < 0.02, "{v} vs {expected}");
    }

    #[test]
    fn prismatic_still_handles_profile_with_hole() {
        let region = Region2 {
            outer: Loop2::polygon(&[Vec2::new(0., 0.), Vec2::new(40., 0.), Vec2::new(40., 30.), Vec2::new(0., 30.)]),
            holes: vec![Loop2::circle(Vec2::new(12., 12.), 4.).reversed()],
        };
        let source = crate::extrude_planar_regions(&Plane::XY, &[region], 0., 10.).unwrap();
        // Analytic or prismatic may win; volume must stay within a percent of the analytic solid.
        let converted = crate::convert_mesh(&mesh_of(&source), 0.05).unwrap();
        assert!(!converted.is_mesh());
        let expected = (1200. - std::f64::consts::PI * 16.) * 10.;
        let v = crate::measure(&converted).unwrap().volume;
        assert!((v - expected).abs() / expected < 0.01, "{v} vs {expected}");
    }

    #[test]
    fn psu_mount_keeps_cylinders() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/stl_to_step_a_b_tests/PSU_rear_mount_back_side_1-screw_v2.stl");
        let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        let body = binary_stl_body(&bytes).unwrap_or_else(|e| panic!("stl: {e}"));
        let mesh_vol = crate::measure(&body).map(|m| m.volume).unwrap_or(18_945.0);
        let converted = convert_analytic_mesh(&body, 0.05, 15f64.to_radians()).unwrap_or_else(|e| panic!("analytic: {e}"));
        let measure = crate::measure(&converted).unwrap();
        let cyl = measure.merged.face_types.get("cylinder").copied().unwrap_or(0);
        assert!(cyl >= 1, "expected cylinders, got {:?}", measure.merged.face_types);
        let v = measure.volume;
        assert!((v - mesh_vol).abs() / mesh_vol.max(1.) < 0.02, "{v} vs {mesh_vol} types={:?}", measure.merged.face_types);
    }

    #[test]
    fn lifepo4_cap_keeps_cylinders() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/stl_to_step_a_b_tests/lifepo4-105ah-cap.stl");
        let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        let body = binary_stl_body(&bytes).unwrap_or_else(|e| panic!("stl: {e}"));
        let mesh_vol = crate::measure(&body).map(|m| m.volume).unwrap_or(1.0);
        let converted = convert_analytic_mesh(&body, 0.05, 15f64.to_radians()).unwrap_or_else(|e| panic!("analytic: {e}"));
        let measure = crate::measure(&converted).unwrap();
        let cyl = measure.merged.face_types.get("cylinder").copied().unwrap_or(0);
        assert!(cyl >= 1, "expected cylinders, got {:?}", measure.merged.face_types);
        let v = measure.volume;
        assert!((v - mesh_vol).abs() / mesh_vol.max(1.) < 0.02, "{v} vs {mesh_vol} types={:?}", measure.merged.face_types);
    }

    fn binary_stl_body(data: &[u8]) -> Result<Body> {
        if data.len() < 84 {
            return Err(invalid("stl header"));
        }
        let n = u32::from_le_bytes(data[80..84].try_into().unwrap_or([0; 4])) as usize;
        if n == 0 || n > MAX_CLUSTER_TRIS || data.len() < 84 + n * 50 {
            return Err(invalid("stl truncated"));
        }
        let mut positions = Vec::new();
        let mut triangles = Vec::with_capacity(n);
        let mut index: HashMap<(i64, i64, i64), u32> = HashMap::new();
        for i in 0..n {
            let off = 84 + i * 50;
            let mut tri = [0u32; 3];
            for k in 0..3 {
                let o = off + 12 + k * 12;
                let x = f32::from_le_bytes(data[o..o + 4].try_into().unwrap_or([0; 4])) as f64;
                let y = f32::from_le_bytes(data[o + 4..o + 8].try_into().unwrap_or([0; 4])) as f64;
                let z = f32::from_le_bytes(data[o + 8..o + 12].try_into().unwrap_or([0; 4])) as f64;
                let key = ((x * 1e5).round() as i64, (y * 1e5).round() as i64, (z * 1e5).round() as i64);
                let id = if let Some(id) = index.get(&key) {
                    *id
                } else {
                    let id = u32::try_from(positions.len()).unwrap_or(u32::MAX);
                    positions.push(Vec3::new(x, y, z));
                    index.insert(key, id);
                    id
                };
                tri[k] = id;
            }
            triangles.push(tri);
        }
        crate::mesh_body(&positions, &triangles)
    }
}
