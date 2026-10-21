//! Clean-room mesh → analytic solid: region-grow clusters, fit plane / cylinder / sphere,
//! sew [`FaceSpec`]s. Matches the process stages in `TEMP_REFERENCE/PROCESS_SPEC.md`
//! (cluster → fit → sew), without vendor WASM heuristics.

use std::collections::{BTreeMap, HashMap, HashSet}; // HashSet: cluster membership during grow
use std::f64::consts::{PI, TAU};

use solidcraft_geom::Vec3;

use crate::meshbody::TriMesh;
use crate::sew::{EdgeSpec, FaceSpec, SurfSpec, sew};
use crate::{Body, KernelError, Result};

const MAX_FACES: usize = 50_000;
const MAX_LOOP_EDGES: usize = 8_192;
const MAX_CLUSTER_TRIS: usize = 50_000;

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
            faces.push(planar_face(mesh, &[ti], &normals, tolerance, &circles)?);
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

/// Reconstruct a closed mesh as an analytic solid by clustering coplanar / coaxial /
/// spherical triangle sets, fitting surfaces within `tolerance` mm, and sewing them.
/// `angle` is the maximum normal deviation (radians) for growing a planar region.
pub fn convert_analytic_mesh(body: &Body, tolerance: f64, angle: f64) -> Result<Body> {
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
        let mut assigned = vec![false; mesh.triangles.len()];
        let mut faces: Vec<FaceSpec> = Vec::new();

        let push_face = |faces: &mut Vec<FaceSpec>, face: FaceSpec| -> Result<()> {
            faces.push(face);
            if faces.len() > MAX_FACES {
                return Err(invalid("too many faces to convert"));
            }
            Ok(())
        };

        // Curved families before planes so finely faceted cylinders/spheres are not eaten as flats.
        // Pass A: full-body short-circuits (kernel primitives). Pass B: partial curved FaceSpecs.
        let mut curved: Vec<FaceSpec> = Vec::new();
        let mut curved_tris: Vec<Vec<usize>> = Vec::new();
        // Canonical circles keyed by quantized (axis, centre, radius, height) so planar holes
        // reuse the same EdgeSpecs as cylinder walls.
        let mut circles: HashMap<CircleKey, Vec<EdgeSpec>> = HashMap::new();

        // --- Pass A1: pure cylindrical can → crate::cylinder ---
        for seed in 0..mesh.triangles.len() {
            if assigned.get(seed).copied().unwrap_or(true) {
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
                    return crate::cylinder(base, cyl.axis, cyl.radius, height)
                        .map_err(|e| invalid(&format!("cylinder feature: {e}")));
                }
            }
        }

        // --- Pass A2: pure sphere → crate::sphere ---
        for seed in 0..mesh.triangles.len() {
            if assigned.get(seed).copied().unwrap_or(true) {
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
                return crate::sphere(sph.center, sph.radius)
                    .map_err(|e| invalid(&format!("sphere feature: {e}")));
            }
            let _ = cluster;
        }

        // --- Pass B: partial curved FaceSpecs (may demote to facets if sew fails) ---
        for seed in 0..mesh.triangles.len() {
            if assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let Some((cluster, sph)) = grow_sphere(mesh, &tris, &normals, &adj, &assigned, seed, tolerance)? else {
                continue;
            };
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a spherical region exceeds the conversion budget"));
            }
            let all_sphere = claim_sphere_tris(&tris, &normals, &assigned, &sph, tolerance);
            let cluster = if all_sphere.len() > cluster.len() { all_sphere } else { cluster };
            match sphere_face(mesh, &cluster, &sph, &normals, tolerance) {
                Ok(face) => {
                    for &t in &cluster {
                        if let Some(slot) = assigned.get_mut(t) {
                            *slot = true;
                        }
                    }
                    curved.push(face);
                    curved_tris.push(cluster);
                }
                Err(_) => {}
            }
            if curved.len() + faces.len() > MAX_FACES {
                return Err(invalid("too many faces to convert"));
            }
        }

        for seed in 0..mesh.triangles.len() {
            if assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let Some((cluster, cyl)) = grow_cylinder(mesh, &tris, &normals, &adj, &assigned, seed, tolerance, cos_ang)? else {
                continue;
            };
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a cylindrical region exceeds the conversion budget"));
            }
            match cylinder_face(mesh, &cluster, &cyl, tolerance, &mut circles) {
                Ok(face) => {
                    for &t in &cluster {
                        if let Some(slot) = assigned.get_mut(t) {
                            *slot = true;
                        }
                    }
                    curved.push(face);
                    curved_tris.push(cluster);
                }
                Err(_) => {}
            }
            if curved.len() + faces.len() > MAX_FACES {
                return Err(invalid("too many faces to convert"));
            }
        }

        for seed in 0..mesh.triangles.len() {
            if assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let Some(cluster) = grow_plane(mesh, &tris, &normals, &areas, &adj, &assigned, seed, tolerance, cos_ang)? else {
                continue;
            };
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a planar region exceeds the conversion budget"));
            }
            let face = planar_face(mesh, &cluster, &normals, tolerance, &circles)?;
            for &t in &cluster {
                if let Some(slot) = assigned.get_mut(t) {
                    *slot = true;
                }
            }
            push_face(&mut faces, face)?;
        }

        // Leftover triangles as planar facets so the shell can still close.
        for seed in 0..mesh.triangles.len() {
            if assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let cluster = vec![seed];
            let face = planar_face(mesh, &cluster, &normals, tolerance, &circles)?;
            if let Some(slot) = assigned.get_mut(seed) {
                *slot = true;
            }
            push_face(&mut faces, face)?;
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
            Err(e) if curved.is_empty() => Err(invalid(&format!("sew failed (planar-only): {e}"))),
            Err(first) => {
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
                // Prefer the most rotated still-wall-like normal.
                if (MIN_WALL_DOT..=MAX_WALL_DOT).contains(&d) && d < far_dot {
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
    let Some(seed_tri) = tris.get(seed) else {
        return Ok(None);
    };
    let pts: Vec<Vec3> = seed_tri.iter().copied().collect();
    // Pass both seed and far normals so the planar-patch reject inside the fit does not fire.
    let Some(cyl0) = fit_cylinder_pts(&pts, &[n0, far], axis, tolerance) else {
        return Ok(None);
    };
    let cyl = cyl0;
    let mut cluster = vec![seed];
    let mut in_cluster = HashSet::from([seed]);
    let mut stack = vec![seed];
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
            let Some(nn) = normals.get(nb).copied() else { continue };
            if nn.dot(cyl.axis).abs() > 0.08 {
                continue;
            }
            let Some(tri) = tris.get(nb) else { continue };
            let radial_ok = tri.iter().all(|p| {
                let w = *p - cyl.origin;
                let radial = w - cyl.axis * w.dot(cyl.axis);
                // Facet chord error sits inside tolerance; allow a little more for weld noise.
                (radial.len() - cyl.radius).abs() <= tolerance * 2.
            });
            if !radial_ok {
                continue;
            }
            // Normal should be roughly radial (tessellated facets are not exact).
            let mid = (tri[0] + tri[1] + tri[2]) * (1. / 3.);
            let w = mid - cyl.origin;
            let radial = w - cyl.axis * w.dot(cyl.axis);
            let Some(rhat) = radial.normalized() else { continue };
            if nn.dot(rhat).abs() < (cos_ang * 0.85).min(0.75) {
                continue;
            }
            in_cluster.insert(nb);
            cluster.push(nb);
            stack.push(nb);
        }
    }
    if cluster.len() < 4 {
        return Ok(None);
    }
    // Refit on all member vertices.
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
    let Some(cyl) = fit_cylinder_pts(&pts, &nrms, cyl.axis, tolerance) else {
        return Ok(None);
    };
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
    if pl.iter().any(|(x, y)| ((x - cu).hypot(y - cv) - r).abs() > tolerance) {
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
fn claim_sphere_tris(
    tris: &[[Vec3; 3]],
    normals: &[Vec3],
    assigned: &[bool],
    sph: &SphFit,
    tolerance: f64,
) -> Vec<usize> {
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
    let min_fan = cluster
        .iter()
        .filter_map(|&t| normals.get(t).copied())
        .map(|n| n.dot(n0).abs())
        .fold(1.0_f64, f64::min);
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

fn planar_face(mesh: &TriMesh, cluster: &[usize], normals: &[Vec3], tolerance: f64, circles: &HashMap<CircleKey, Vec<EdgeSpec>>) -> Result<FaceSpec> {
    // Single triangle: keep the mesh winding exactly (no simplify / re-orient) so a closed
    // manifold triangulated shell sews without inventing gaps at shared edges.
    if let [ti] = *cluster {
        let tri = mesh.triangles.get(ti).ok_or_else(|| invalid("missing triangle"))?;
        let (Some(a), Some(b), Some(c)) =
            (mesh.positions.get(tri[0] as usize), mesh.positions.get(tri[1] as usize), mesh.positions.get(tri[2] as usize))
        else {
            return Err(invalid("missing vertex"));
        };
        let edges = vec![EdgeSpec::Line { a: *a, b: *b }, EdgeSpec::Line { a: *b, b: *c }, EdgeSpec::Line { a: *c, b: *a }];
        return Ok(FaceSpec { loops: vec![edges], surface: SurfSpec::Plane, probe: None });
    }
    let loops = boundary_loops(mesh, cluster)?;
    if loops.is_empty() {
        return Err(invalid("planar region has no boundary"));
    }
    let mut n = Vec3::ZERO;
    for &t in cluster {
        if let Some(nn) = normals.get(t) {
            n += *nn;
        }
    }
    let Some(n) = n.normalized() else {
        return Err(invalid("planar region has no normal"));
    };
    let mut edge_loops = Vec::new();
    for lp in loops {
        let pts: Vec<Vec3> = lp.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()).collect();
        if pts.len() < 3 {
            return Err(invalid("planar loop too short"));
        }
        // Prefer a canonical cylinder circle (exact shared arcs), else fit, else lines.
        let edges = match lookup_circle_for_loop(&pts, n, tolerance, circles) {
            Some(e) => e,
            None => match exact_rim_from_points(&pts, n, tolerance) {
                Ok(e) => e,
                Err(_) => polyline_to_lines(&pts, tolerance)?,
            },
        };
        if edges.len() > MAX_LOOP_EDGES {
            return Err(invalid("planar loop exceeds the edge budget"));
        }
        edge_loops.push(edges);
    }
    // Ensure outer loop is CCW about outward normal; holes CW.
    orient_planar_loops(&mut edge_loops, n);
    Ok(FaceSpec { loops: edge_loops, surface: SurfSpec::Plane, probe: None })
}

fn polyline_to_lines(pts: &[Vec3], tolerance: f64) -> Result<Vec<EdgeSpec>> {
    let simplified = simplify_polyline(pts, tolerance);
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

fn cylinder_face(
    mesh: &TriMesh,
    cluster: &[usize],
    cyl: &CylFit,
    tolerance: f64,
    circles: &mut HashMap<CircleKey, Vec<EdgeSpec>>,
) -> Result<FaceSpec> {
    let loops = boundary_loops(mesh, cluster)?;
    if loops.is_empty() {
        return Err(invalid("cylindrical region has no boundary"));
    }
    let (t_lo, t_hi) = cylinder_height_range(mesh, cluster, cyl)?;
    let mut edge_loops = Vec::new();
    for lp in loops {
        let pts: Vec<Vec3> = lp.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()).collect();
        let edges = cylinder_loop_edges(&pts, cyl, tolerance, circles)?;
        if edges.len() > MAX_LOOP_EDGES {
            return Err(invalid("cylindrical loop exceeds the edge budget"));
        }
        edge_loops.push(edges);
    }
    let mid_h = (t_lo + t_hi) * 0.5;
    let radial = cylinder_outward_radial(mesh, cluster, cyl);
    let probe_p = cyl.origin + cyl.axis * mid_h + radial * cyl.radius;
    let a = cyl.origin + radial * cyl.radius + cyl.axis * (t_lo - tolerance);
    let b = cyl.origin + radial * cyl.radius + cyl.axis * (t_hi + tolerance);
    Ok(FaceSpec {
        loops: edge_loops,
        surface: SurfSpec::Revolved {
            a,
            b,
            axis_origin: cyl.origin,
            axis: cyl.axis,
            angle: TAU,
        },
        probe: Some((probe_p, radial)),
    })
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
    CircleKey {
        ax: q(axis.x),
        ay: q(axis.y),
        az: q(axis.z),
        cx: q(center.x),
        cy: q(center.y),
        cz: q(center.z),
        r: q(radius),
    }
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
        edges.push(EdgeSpec::Arc {
            a: p(a0),
            b: p(a0 + step),
            mid: p(a0 + step * 0.5),
        });
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

fn cylinder_outward_radial(mesh: &TriMesh, cluster: &[usize], cyl: &CylFit) -> Vec3 {
    let mut sum_n = Vec3::ZERO;
    let mut sum_r = Vec3::ZERO;
    let mut w = 0.0_f64;
    for &t in cluster {
        let Some(tri) = mesh.triangles.get(t) else { continue };
        let (Some(pa), Some(pb), Some(pc)) =
            (mesh.positions.get(tri[0] as usize), mesh.positions.get(tri[1] as usize), mesh.positions.get(tri[2] as usize))
        else {
            continue;
        };
        let mid = (*pa + *pb + *pc) * (1. / 3.);
        let radial = mid - cyl.origin - cyl.axis * (mid - cyl.origin).dot(cyl.axis);
        let cross = (*pb - *pa).cross(*pc - *pa);
        if let Some(nn) = cross.normalized() {
            sum_n += nn;
        }
        sum_r += radial;
        w += 1.;
    }
    let radial = (sum_r * (1. / w.max(1.0))).normalized().unwrap_or_else(|| cyl.axis.any_perp());
    if sum_n.dot(radial) < 0. {
        -radial
    } else {
        radial
    }
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
    let radial = cylinder_outward_radial(mesh, wall, cyl);

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
    let bottom_face = FaceSpec {
        loops: vec![reverse_edges(&bot)],
        surface: SurfSpec::Plane,
        probe: None,
    };
    let top_face = FaceSpec {
        loops: vec![top.clone()],
        surface: SurfSpec::Plane,
        probe: None,
    };
    let wall_faces = half_cylinder_walls(&bot, cyl.origin, dir, radial)?;
    let mut feature = vec![bottom_face, top_face];
    feature.extend(wall_faces);
    let _ = mesh;
    Ok(Some((feature, all)))
}

/// Extruded wall faces from each arc of a bottom circle. Prefer calling this only for bosses
/// that are not a pure can — pure cans short-circuit to [`crate::cylinder`].
fn half_cylinder_walls(bot: &[EdgeSpec], axis_origin: Vec3, dir: Vec3, radial_hint: Vec3) -> Result<Vec<FaceSpec>> {
    if bot.len() < 2 {
        return Err(invalid("cylinder circle must have at least two arcs"));
    }
    let axis = dir.normalized().unwrap_or(Vec3::Z);
    let mut walls = Vec::with_capacity(bot.len());
    for arc in bot {
        let EdgeSpec::Arc { a, b, mid } = arc else {
            return Err(invalid("cylinder circle arcs"));
        };
        let (top_a, top_b, top_mid) = (*a + dir, *b + dir, *mid + dir);
        let m = (*mid + top_mid) * 0.5;
        let radial = m - axis_origin - axis * (m - axis_origin).dot(axis);
        let outward = radial.normalized().unwrap_or(radial_hint);
        let outward = if outward.dot(radial_hint) < 0. { -outward } else { outward };
        walls.push(FaceSpec {
            loops: vec![vec![
                EdgeSpec::Arc { a: *a, b: *b, mid: *mid },
                EdgeSpec::Line { a: *b, b: top_b },
                EdgeSpec::Arc { a: top_b, b: top_a, mid: top_mid },
                EdgeSpec::Line { a: top_a, b: *a },
            ]],
            surface: SurfSpec::Extruded { a: *a, b: *b, mid: *mid, dir },
            probe: Some((m, outward)),
        });
    }
    Ok(walls)
}

fn register_circle(
    circles: &mut HashMap<CircleKey, Vec<EdgeSpec>>,
    axis: Vec3,
    center: Vec3,
    radius: f64,
    phase: f64,
) -> Result<Vec<EdgeSpec>> {
    let key = circle_key(axis, center, radius);
    if let Some(existing) = circles.get(&key) {
        return Ok(existing.clone());
    }
    let edges = exact_circle_edges(center, axis, radius, phase).ok_or_else(|| invalid("bad circle"))?;
    circles.insert(key, edges.clone());
    Ok(edges)
}

fn lookup_circle_for_loop(
    pts: &[Vec3],
    plane_n: Vec3,
    tolerance: f64,
    circles: &HashMap<CircleKey, Vec<EdgeSpec>>,
) -> Option<Vec<EdgeSpec>> {
    if pts.len() < 6 {
        return None;
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
    circles.get(&key).cloned().or_else(|| {
        // Axis may be flipped in the key.
        let key2 = circle_key(-plane_n, center, r);
        circles.get(&key2).cloned()
    })
}

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
    Ok(FaceSpec {
        loops: edge_loops,
        surface: SurfSpec::Sphere { center: sph.center, radius: sph.radius },
        probe: Some((probe_p, probe_n)),
    })
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

fn cylinder_loop_edges(
    pts: &[Vec3],
    cyl: &CylFit,
    tolerance: f64,
    circles: &mut HashMap<CircleKey, Vec<EdgeSpec>>,
) -> Result<Vec<EdgeSpec>> {
    let heights: Vec<f64> = pts.iter().map(|p| (*p - cyl.origin).dot(cyl.axis)).collect();
    let h0 = heights.first().copied().unwrap_or(0.);
    if heights.iter().all(|h| (*h - h0).abs() <= tolerance * 2.) {
        // Exact circle at this height, registered for planar mates.
        let center = cyl.origin + cyl.axis * h0;
        return register_circle(circles, cyl.axis, center, cyl.radius, 0.3);
    }
    // Seam / generator loops: straight segments through mesh vertices.
    polyline_to_lines(pts, tolerance)
}

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
    if pts.len() <= 3 {
        return pts.to_vec();
    }
    let mut out = pts.to_vec();
    let linear_tol = tolerance.max(1e-9);
    loop {
        let n = out.len();
        if n <= 3 {
            break;
        }
        let remove = (0..n).find(|&i| {
            let (a, b, c) = (out[(i + n - 1) % n], out[i], out[(i + 1) % n]);
            point_line_dist(b, a, c) <= linear_tol
        });
        let Some(index) = remove else { break };
        out.remove(index);
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
    let mut areas: Vec<(usize, f64)> = loops
        .iter()
        .enumerate()
        .map(|(i, l)| (i, area_n(l).len()))
        .collect();
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
                EdgeSpec::Chain {
                    degree: *degree,
                    ctrl: c.into(),
                }
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
        // Caps + extruded half-walls form a closed shell (topology). Volume fidelity for a
        // pure can is covered by `analytic_cylinder_recovers` via `crate::cylinder`.
        let r = 5.;
        let h = 12.;
        let bot = exact_circle_edges(Vec3::ZERO, Vec3::Z, r, crate::build::SEAM_ANGLE).unwrap();
        let top = exact_circle_edges(Vec3::new(0., 0., h), Vec3::Z, r, crate::build::SEAM_ANGLE).unwrap();
        let mut faces = vec![
            FaceSpec { loops: vec![reverse_edges(&bot)], surface: SurfSpec::Plane, probe: None },
            FaceSpec { loops: vec![top], surface: SurfSpec::Plane, probe: None },
        ];
        faces.extend(half_cylinder_walls(&bot, Vec3::ZERO, Vec3::Z * h, Vec3::X).unwrap());
        assert!(faces.len() >= 4);
        let body = sew(&faces).expect("cylinder FaceSpecs must sew");
        assert!(crate::measure(&body).unwrap().volume > 0.);
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
}
