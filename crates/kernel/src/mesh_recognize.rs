//! Clean-room mesh → analytic solid: region-grow clusters, fit plane / cylinder / sphere,
//! sew [`FaceSpec`]s. Matches the process stages in `TEMP_REFERENCE/PROCESS_SPEC.md`
//! (cluster → fit → sew), without vendor WASM heuristics.

use std::collections::{BTreeMap, HashMap, HashSet}; // HashSet: cluster membership during grow
use std::f64::consts::{PI, TAU};

use solidcraft_geom::Vec3;

use crate::meshbody::TriMesh;
use crate::sew::{EdgeSpec, FaceSpec, SurfSpec, drop_degenerate_edges, nonsimple_faces, sew};
use crate::{Body, KernelError, Result};

const MAX_FACES: usize = 50_000;
const MAX_LOOP_EDGES: usize = 8_192;
const MAX_CLUSTER_TRIS: usize = 100_000;

#[cfg(test)]
mod recognize_stats {
    use std::sync::atomic::{AtomicUsize, Ordering};

    static GROWN: AtomicUsize = AtomicUsize::new(0);
    static PENDING: AtomicUsize = AtomicUsize::new(0);
    static KEPT: AtomicUsize = AtomicUsize::new(0);
    static LOOPS: AtomicUsize = AtomicUsize::new(0);
    static FLAT: AtomicUsize = AtomicUsize::new(0);
    static CONE_PATCH: AtomicUsize = AtomicUsize::new(0);
    static CONE_FIT: AtomicUsize = AtomicUsize::new(0);
    static CONE_PENDING: AtomicUsize = AtomicUsize::new(0);
    static CONE_KEPT: AtomicUsize = AtomicUsize::new(0);
    static TRIS: AtomicUsize = AtomicUsize::new(0);
    static HIST: [AtomicUsize; 6] =
        [AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0)];
    static DROPPED: AtomicUsize = AtomicUsize::new(0);
    static EMITTED: AtomicUsize = AtomicUsize::new(0);
    static OVERLAP: AtomicUsize = AtomicUsize::new(0);

    pub fn reset() {
        for n in [&GROWN, &PENDING, &KEPT, &LOOPS, &FLAT, &CONE_PATCH, &CONE_FIT, &CONE_PENDING, &CONE_KEPT, &TRIS, &DROPPED, &EMITTED, &OVERLAP] {
            n.store(0, Ordering::Relaxed);
        }
        for n in &HIST {
            n.store(0, Ordering::Relaxed);
        }
    }

    pub fn add(which: &AtomicUsize) {
        which.fetch_add(1, Ordering::Relaxed);
    }

    pub fn note_overlap(permille: usize) {
        OVERLAP.fetch_max(permille, Ordering::Relaxed);
    }

    pub fn dropped() -> &'static AtomicUsize {
        &DROPPED
    }
    pub fn emitted() -> &'static AtomicUsize {
        &EMITTED
    }

    pub fn note_boundary(loops: usize, tris: usize) {
        let i = loops.min(5);
        HIST[i].fetch_add(1, Ordering::Relaxed);
        TRIS.fetch_add(tris, Ordering::Relaxed);
    }

    pub fn grown() -> &'static AtomicUsize {
        &GROWN
    }
    pub fn pending() -> &'static AtomicUsize {
        &PENDING
    }
    pub fn kept() -> &'static AtomicUsize {
        &KEPT
    }
    pub fn loops() -> &'static AtomicUsize {
        &LOOPS
    }
    pub fn flat() -> &'static AtomicUsize {
        &FLAT
    }
    pub fn cone_patch() -> &'static AtomicUsize {
        &CONE_PATCH
    }
    pub fn cone_fit() -> &'static AtomicUsize {
        &CONE_FIT
    }
    pub fn cone_pending() -> &'static AtomicUsize {
        &CONE_PENDING
    }
    pub fn cone_kept() -> &'static AtomicUsize {
        &CONE_KEPT
    }

    pub fn line() -> String {
        format!(
            "cyl grown {} pending {} kept {} reject-loops {} reject-flat {} | cone patch {} fit {} pending {} kept {} | loop-hist {:?} avg-tris {:.0} | drop {} emit {} overlap {}/1000",
            GROWN.load(Ordering::Relaxed),
            PENDING.load(Ordering::Relaxed),
            KEPT.load(Ordering::Relaxed),
            LOOPS.load(Ordering::Relaxed),
            FLAT.load(Ordering::Relaxed),
            CONE_PATCH.load(Ordering::Relaxed),
            CONE_FIT.load(Ordering::Relaxed),
            CONE_PENDING.load(Ordering::Relaxed),
            CONE_KEPT.load(Ordering::Relaxed),
            HIST.each_ref().map(|n| n.load(Ordering::Relaxed)),
            TRIS.load(Ordering::Relaxed) as f64 / LOOPS.load(Ordering::Relaxed).max(1) as f64,
            DROPPED.load(Ordering::Relaxed),
            EMITTED.load(Ordering::Relaxed),
            OVERLAP.load(Ordering::Relaxed),
        )
    }
}

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
                &HashSet::new(),
                None,
                &mut HashSet::new(),
            )?);
            if faces.len() > MAX_FACES {
                return Err(invalid("too many faces to convert"));
            }
        }
        drop_degenerate_edges(&mut faces);
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
    seed.is_multiple_of(stride.max(1))
}

/// A triangle's recognised family, for the convert preview. Tori and threads are not fitted
/// yet, so their triangles stay a plane, a cylinder, a cone, freeform, or left out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceGroup {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    /// Kept as facets when freeform is on.
    Freeform,
    /// Dropped from the solid when freeform is off.
    LeftOut,
}

/// Welded triangles and the family each one grew into. This is the segmentation the convert
/// dialog draws; it does not sew a solid.
pub struct MeshGroups {
    pub positions: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
    pub groups: Vec<FaceGroup>,
}

/// Label each triangle of `body` with the family a convert at `options` would grow.
pub fn label_mesh(body: &Body, options: &crate::mesh_convert::ConvertOptions) -> Result<MeshGroups> {
    crate::guard("label mesh groups", || {
        let tolerance = options.tolerance;
        let angle = options.angle;
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
            return Err(invalid("mesh exceeds the conversion triangle budget"));
        }
        let (tris, normals, areas) = triangle_geom(mesh)?;
        let adj = triangle_adjacency(mesh)?;
        let cos_ang = angle.cos();
        let fraction = (options.mesh_percent / 100.0).clamp(0.0, 1.0);
        let recognize = RecognizeOptions {
            cylinders: options.cylinders,
            cones: options.cones,
            tori: options.tori,
            spheres: options.spheres,
            threads: options.threads,
            mesh_fraction: fraction,
        };
        let n = mesh.triangles.len();
        let mut assigned = vec![false; n];
        let mut groups = vec![if options.freeform { FaceGroup::Freeform } else { FaceGroup::LeftOut }; n];
        let paint = |groups: &mut [FaceGroup], assigned: &mut [bool], cluster: &[usize], group: FaceGroup| {
            if cluster.len() < 2 {
                return;
            }
            for &ti in cluster {
                if let Some(slot) = assigned.get_mut(ti) {
                    *slot = true;
                }
                if let Some(slot) = groups.get_mut(ti) {
                    *slot = group;
                }
            }
        };
        for seed in 0..n {
            if !recognize.cylinders || !seed_kept(seed, recognize.mesh_fraction) || assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            if let Some((cluster, _)) = grow_cylinder(mesh, &tris, &normals, &adj, &assigned, seed, tolerance, cos_ang)? {
                paint(&mut groups, &mut assigned, &cluster, FaceGroup::Cylinder);
            }
        }
        let mut skip_cone = assigned.clone();
        for seed in 0..n {
            if !recognize.cones || !seed_kept(seed, recognize.mesh_fraction) || skip_cone.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let (cluster, fit) = grow_cone(mesh, &tris, &normals, &adj, &skip_cone, seed, tolerance)?;
            if fit.is_some() {
                paint(&mut groups, &mut assigned, &cluster, FaceGroup::Cone);
                for &t in &cluster {
                    if let Some(slot) = skip_cone.get_mut(t) {
                        *slot = true;
                    }
                }
            } else {
                for &t in &cluster {
                    if let Some(slot) = skip_cone.get_mut(t) {
                        *slot = true;
                    }
                }
            }
        }
        for seed in 0..n {
            if !recognize.spheres || !seed_kept(seed, recognize.mesh_fraction) || assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            if let Some((cluster, _)) = grow_sphere(mesh, &tris, &normals, &adj, &assigned, seed, tolerance)? {
                paint(&mut groups, &mut assigned, &cluster, FaceGroup::Sphere);
            }
        }
        for seed in 0..n {
            if !seed_kept(seed, recognize.mesh_fraction) || assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            if let Some(cluster) = grow_plane(mesh, &tris, &normals, &areas, &adj, &assigned, seed, tolerance, cos_ang)? {
                paint(&mut groups, &mut assigned, &cluster, FaceGroup::Plane);
            }
        }
        Ok(MeshGroups { positions: mesh.positions.clone(), triangles: mesh.triangles.clone(), groups })
    })
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
        // Tori and threads are chosen in the convert dialog. Their triangles stay eligible
        // for planes, cylinders and cones until a fitter for those families exists.
        let _later = (options.tori, options.threads);
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
                && all_tris.len() * 20 >= mesh.triangles.len() * 19
                && !cylinder_opens_wide(mesh, &cluster, &cyl)
            {
                let (t_lo, t_hi) = cylinder_height_range(mesh, &cluster, &cyl)?;
                let base = cyl.origin + cyl.axis * t_lo;
                let height = t_hi - t_lo;
                return crate::cylinder(base, cyl.axis, cyl.radius, height).map_err(|e| invalid(&format!("cylinder feature: {e}")));
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
        // Open walls (a channel, a half-round) are not tubes. They are still one cylinder,
        // and the preview paints them that way. Claimed only after cones, so a frustum
        // that also fits a short cylinder stays a cone.
        let mut partial_candidates: Vec<PartialCyl> = Vec::new();
        for seed in 0..mesh.triangles.len() {
            if !options.cylinders || !seed_kept(seed, options.mesh_fraction) || skip_cyl.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let Some((cluster, cyl)) = grow_cylinder(mesh, &tris, &normals, &adj, &skip_cyl, seed, tolerance, cos_ang)? else {
                continue;
            };
            #[cfg(test)]
            recognize_stats::add(recognize_stats::grown());
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a cylindrical region exceeds the conversion budget"));
            }
            if let Some(tube) = try_pending_tube(mesh, &cluster, &cyl, tolerance)? {
                #[cfg(test)]
                recognize_stats::add(recognize_stats::pending());
                for &t in &tube.cluster {
                    if let Some(slot) = skip_cyl.get_mut(t) {
                        *slot = true;
                    }
                    if let Some(slot) = assigned.get_mut(t) {
                        *slot = true;
                    }
                }
                pending.push(tube);
            } else if cylinder_opens_wide(mesh, &cluster, &cyl) {
                for &t in &cluster {
                    if let Some(slot) = skip_cyl.get_mut(t) {
                        *slot = true;
                    }
                }
                partial_candidates.push(PartialCyl { cluster, cyl });
            } else {
                for &t in &cluster {
                    if let Some(slot) = skip_cyl.get_mut(t) {
                        *slot = true;
                    }
                }
            }
        }

        // A cone is a ring whose radius changes linearly along one axis (a countersink or a
        // frustum). Cylinders are already claimed, so a constant radius is not taken here.
        let mut pending_cones: Vec<PendingCone> = Vec::new();
        let mut skip_cone = assigned.clone();
        for seed in 0..mesh.triangles.len() {
            if !options.cones || !seed_kept(seed, options.mesh_fraction) || skip_cone.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let (cluster, fit) = grow_cone(mesh, &tris, &normals, &adj, &skip_cone, seed, tolerance)?;
            #[cfg(test)]
            recognize_stats::add(recognize_stats::cone_patch());
            let Some(cone) = fit else {
                for &t in &cluster {
                    if let Some(slot) = skip_cone.get_mut(t) {
                        *slot = true;
                    }
                }
                continue;
            };
            #[cfg(test)]
            recognize_stats::add(recognize_stats::cone_fit());
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a conical region exceeds the conversion budget"));
            }
            if let Some(pending_cone) = try_pending_cone(mesh, &cluster, &cone, tolerance)? {
                #[cfg(test)]
                recognize_stats::add(recognize_stats::cone_pending());
                for &t in &pending_cone.cluster {
                    if let Some(slot) = skip_cone.get_mut(t) {
                        *slot = true;
                    }
                    if let Some(slot) = assigned.get_mut(t) {
                        *slot = true;
                    }
                }
                pending_cones.push(pending_cone);
            } else {
                for &t in &cluster {
                    if let Some(slot) = skip_cone.get_mut(t) {
                        *slot = true;
                    }
                }
            }
        }

        let mut partials: Vec<PartialCyl> = Vec::new();
        for partial in partial_candidates {
            if partial.cluster.iter().any(|&t| assigned.get(t).copied().unwrap_or(true)) {
                continue;
            }
            for &t in &partial.cluster {
                if let Some(slot) = assigned.get_mut(t) {
                    *slot = true;
                }
            }
            partials.push(partial);
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
                #[cfg(test)]
                recognize_stats::add(recognize_stats::kept());
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
        let mut kept_cones: Vec<PendingCone> = Vec::new();
        for cone in pending_cones {
            let saved: Vec<(Vec3, f64, bool)> = planes.iter().map(|p| (p.n, p.d, p.locked)).collect();
            let locked = lock_rim(&mut planes, &owners, &cone.rim0, &cone.cyl_at(cone.t0, cone.r0), cone.t0)
                && lock_rim(&mut planes, &owners, &cone.rim1, &cone.cyl_at(cone.t1, cone.r1), cone.t1);
            if locked {
                #[cfg(test)]
                recognize_stats::add(recognize_stats::cone_kept());
                kept_cones.push(cone);
            } else {
                for (plane, (n, d, was)) in planes.iter_mut().zip(saved) {
                    plane.n = n;
                    plane.d = d;
                    plane.locked = was;
                }
                for &t in &cone.cluster {
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
        // A triangle left beside a plane, but still within tolerance of it, becomes part of that
        // plane. Emitted on its own, snapping flattens it and the shell no longer closes.
        absorb_coplanar_triangles(mesh, &adj, &mut planes, &mut assigned, tolerance);
        let snapped = snapped_positions(mesh, &planes);
        let rim_owners = plane_owners(mesh, &planes);
        // Caps look circles up by the rim's vertex set. A geometric fit misses once snapping
        // moves a rim point, and the cap then uses lines the wall does not have.
        let mut by_rim: HashMap<Vec<u32>, Vec<EdgeSpec>> = HashMap::new();
        for tube in &kept {
            let mut bot = register_circle(&mut circles, tube.cyl.axis, tube.cyl.origin + tube.cyl.axis * tube.h0, tube.cyl.radius, 0.3)?;
            let mut top = register_circle(&mut circles, tube.cyl.axis, tube.cyl.origin + tube.cyl.axis * tube.h1, tube.cyl.radius, 0.3)?;
            if let Some((n, d)) = rim_plane(&planes, &rim_owners, &tube.rim0)
                && circle_near_plane(&bot, n, d)
            {
                for edge in &mut bot {
                    snap_edge_plane(edge, n, d);
                }
                circles.insert(circle_key(tube.cyl.axis, tube.cyl.origin + tube.cyl.axis * tube.h0, tube.cyl.radius), bot.clone());
            }
            if let Some((n, d)) = rim_plane(&planes, &rim_owners, &tube.rim1)
                && circle_near_plane(&top, n, d)
            {
                for edge in &mut top {
                    snap_edge_plane(edge, n, d);
                }
                circles.insert(circle_key(tube.cyl.axis, tube.cyl.origin + tube.cyl.axis * tube.h1, tube.cyl.radius), top.clone());
            }
            let mut k0 = tube.rim0.clone();
            let mut k1 = tube.rim1.clone();
            k0.sort_unstable();
            k1.sort_unstable();
            by_rim.insert(k0, bot);
            by_rim.insert(k1, top);
        }
        for cone in &kept_cones {
            let mut bot = register_circle(&mut circles, cone.axis, cone.apex + cone.axis * cone.t0, cone.r0, 0.3)?;
            let mut top = register_circle(&mut circles, cone.axis, cone.apex + cone.axis * cone.t1, cone.r1, 0.3)?;
            if let Some((n, d)) = rim_plane(&planes, &rim_owners, &cone.rim0)
                && circle_near_plane(&bot, n, d)
            {
                for edge in &mut bot {
                    snap_edge_plane(edge, n, d);
                }
                circles.insert(circle_key(cone.axis, cone.apex + cone.axis * cone.t0, cone.r0), bot.clone());
            }
            if let Some((n, d)) = rim_plane(&planes, &rim_owners, &cone.rim1)
                && circle_near_plane(&top, n, d)
            {
                for edge in &mut top {
                    snap_edge_plane(edge, n, d);
                }
                circles.insert(circle_key(cone.axis, cone.apex + cone.axis * cone.t1, cone.r1), top.clone());
            }
            let mut k0 = cone.rim0.clone();
            let mut k1 = cone.rim1.clone();
            k0.sort_unstable();
            k1.sort_unstable();
            by_rim.insert(k0, bot);
            by_rim.insert(k1, top);
        }
        // A snapped boundary that repeats a vertex is not a face. Those triangles become
        // facets, and neighbouring planes must keep the vertices the facets still use.
        let empty_keep = HashSet::new();
        let empty_keep_edges = HashSet::new();
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
                    &empty_keep_edges,
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
        let mut leftover_edges = HashSet::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            if assigned.get(ti).copied().unwrap_or(true) {
                continue;
            }
            leftover_verts.extend(tri.iter().copied());
            let [a, b, c] = *tri;
            for (u, v) in [(a, b), (b, c), (c, a)] {
                leftover_edges.insert((u.min(v), u.max(v)));
            }
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
                    &leftover_edges,
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
            for cone in &kept_cones {
                let mut k0 = cone.rim0.clone();
                let mut k1 = cone.rim1.clone();
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
                    #[cfg(test)]
                    recognize_stats::add(recognize_stats::dropped());
                    for &t in &tube.cluster {
                        if let Some(slot) = assigned.get_mut(t) {
                            *slot = false;
                        }
                    }
                    continue;
                }
                curved.extend(cylinder_face(mesh, &tube.cluster, &tube.cyl, tolerance, &mut circles)?);
                #[cfg(test)]
                recognize_stats::add(recognize_stats::emitted());
            }
            for cone in &kept_cones {
                let mut k0 = cone.rim0.clone();
                let mut k1 = cone.rim1.clone();
                k0.sort_unstable();
                k1.sort_unstable();
                if !used_rims.contains(&k0) || !used_rims.contains(&k1) {
                    for &t in &cone.cluster {
                        if let Some(slot) = assigned.get_mut(t) {
                            *slot = false;
                        }
                    }
                    continue;
                }
                curved.extend(cone_face(mesh, cone, &mut circles)?);
            }
            for (face, cluster) in staged {
                push_face(&mut faces, &mut origins, face, cluster)?;
            }
            break;
        }

        // An open cylinder keeps every mesh edge of its boundary, so the planes and
        // leftover triangles beside it use the same lines. A patch that will not make a
        // simple wire is released and filled as triangles with the rest.
        let mut partial_faces: Vec<(FaceSpec, Vec<usize>)> = Vec::new();
        for partial in partials {
            match partial_cylinder_face(mesh, &snapped, &partial.cluster, &partial.cyl) {
                Ok(face) => partial_faces.push((face, partial.cluster)),
                Err(_) => {
                    for &t in &partial.cluster {
                        if let Some(slot) = assigned.get_mut(t) {
                            *slot = false;
                        }
                    }
                }
            }
        }

        // Leftover triangles as planar facets so the shell can still close.
        for seed in 0..mesh.triangles.len() {
            if assigned.get(seed).copied().unwrap_or(true) {
                continue;
            }
            let face = planar_face(
                mesh,
                &snapped,
                &[seed],
                &normals,
                tolerance,
                &circles,
                &by_rim,
                &HashSet::new(),
                &HashSet::new(),
                None,
                &mut HashSet::new(),
            )?;
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
                for (at, ti) in (fi..).zip(cluster) {
                    let face = planar_face(
                        mesh,
                        &snapped,
                        &[ti],
                        &normals,
                        tolerance,
                        &circles,
                        &by_rim,
                        &HashSet::new(),
                        &HashSet::new(),
                        None,
                        &mut HashSet::new(),
                    )?;
                    faces.insert(at, face);
                    origins.insert(at, vec![ti]);
                    if faces.len() > MAX_FACES {
                        return Err(invalid("too many faces to convert"));
                    }
                }
            }
        }

        // A triangle snapped almost flat has no normal truck will accept, but its edges are
        // already shared. Fold it into the neighbour across the long edge so the shell still closes.
        fold_sliver_triangles(&mut faces, &mut origins);
        // Adjacent facets of one flat become one face. The reference converter does the same
        // merge before it writes a solid; leaving them split is what makes Confirm look like the STL.
        merge_adjacent_coplanar(&mut faces, &mut origins, tolerance, cos_ang);

        // Try sewing with curved faces. An open cylinder that will not close is replaced
        // by its triangles, which share the mesh edges, and the planes are sewn again.
        // Giving up here drops every grouped face and the import looks like the STL.
        let mut attempt = faces.clone();
        attempt.extend(curved.iter().cloned());
        // Snapping onto two planes leaves one shared corner a few hundredths of a micrometre
        // apart. That is enough for the edge not to be shared, so the shell stays open.
        // Open-cylinder wires are appended afterwards: pulling one of their points onto a
        // plane corner lifts that plane off itself, and truck then refuses the wire.
        unify_near_corners(&mut attempt, 3e-5);
        // Unifying corners closer than the sew weld turns a short edge into a
        // zero-length one, which sew refuses. Drop those edges before sewing.
        drop_degenerate_edges(&mut attempt);
        // A patch that was flat enough to grow can collapse once corners weld. Fold it into
        // the neighbour so the shell stays closed and the cylinders are not thrown away.
        fold_flat_faces(&mut attempt);
        drop_degenerate_edges(&mut attempt);
        // Truck rejects a plane whose corners are a few hundredths of a millimetre off, which is
        // still inside the convert tolerance. Lay each corner on the planes that meet there.
        flatten_plane_corners(&mut attempt, tolerance);
        drop_degenerate_edges(&mut attempt);
        let mut partial_only: Vec<FaceSpec> = partial_faces.iter().map(|(face, _)| face.clone()).collect();
        // Cylinder rims use the mesh corners. Follow the plane corner each one was fitted to,
        // and leave an edge alone when both ends would collapse onto the same point.
        snap_partial_corners(&mut partial_only, &attempt, tolerance);
        attempt.extend(partial_only);
        drop_degenerate_edges(&mut attempt);
        #[cfg(test)]
        if !partial_faces.is_empty() {
            eprintln!("diag open-cyl built={}", partial_faces.len());
        }
        if attempt.len() < 4 {
            return Err(invalid("not enough faces to close a solid"));
        }
        match sew(&attempt) {
            Ok(body) => Ok(body),
            Err(first) if !partial_faces.is_empty() => {
                #[cfg(test)]
                {
                    eprintln!(
                        "diag open-cyl sew failed faces={} curved={} partials={} attempt={}: {first}",
                        faces.len(),
                        curved.len(),
                        partial_faces.len(),
                        attempt.len()
                    );
                    if let Some(fi) = first
                        .to_string()
                        .split("face ")
                        .nth(1)
                        .and_then(|rest| rest.split(|c: char| !c.is_ascii_digit()).next())
                        .and_then(|n| n.parse::<usize>().ok())
                        && let Some(face) = attempt.get(fi)
                    {
                        let edges = face.loops.iter().map(|lp| lp.len()).sum::<usize>();
                        let (nlen, dev) = face_plane_metrics(face);
                        eprintln!(
                            "diag bad face {fi} edges={edges} loops={} dev={dev:.3e} nlen={nlen:.3e} revolved={}",
                            face.loops.len(),
                            matches!(face.surface, SurfSpec::Revolved { .. })
                        );
                    }
                }
                // Slivers flattened by the corner weld have no area, and one of them makes
                // truck refuse the whole shell. Drop every empty plane and sew again, cylinders
                // included. If the shell is then open, the triangles below take over.
                let trimmed: Vec<FaceSpec> = attempt
                    .iter()
                    .filter(|face| match face.surface {
                        SurfSpec::Revolved { .. } | SurfSpec::Sphere { .. } => true,
                        _ => {
                            let (nlen, _) = face_plane_metrics(face);
                            nlen >= 1e-6
                        }
                    })
                    .cloned()
                    .collect();
                if trimmed.len() >= 4
                    && trimmed.len() < attempt.len()
                    && let Ok(body) = sew(&trimmed)
                {
                    return Ok(body);
                }
                for (_, cluster) in &partial_faces {
                    for &ti in cluster {
                        let face = planar_face(
                            mesh,
                            &snapped,
                            &[ti],
                            &normals,
                            tolerance,
                            &circles,
                            &by_rim,
                            &HashSet::new(),
                            &HashSet::new(),
                            None,
                            &mut HashSet::new(),
                        )?;
                        push_face(&mut faces, &mut origins, face, vec![ti])?;
                    }
                }
                merge_adjacent_coplanar(&mut faces, &mut origins, tolerance, cos_ang);
                let mut retry = faces.clone();
                retry.extend(curved.iter().cloned());
                unify_near_corners(&mut retry, 3e-5);
                drop_degenerate_edges(&mut retry);
                fold_flat_faces(&mut retry);
                drop_degenerate_edges(&mut retry);
                if retry.len() >= 4
                    && let Ok(body) = sew(&retry)
                {
                    return Ok(body);
                }
                if curved.is_empty() {
                    Err(invalid(&format!("sew failed (planar-only): {first}")))
                } else {
                    Err(invalid(&format!("curved faces would not sew: {first}")))
                }
            }
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
                    for (a, b) in once_edge_samples(&attempt, 12) {
                        eprintln!("diag once ({:.5},{:.5},{:.5})-({:.5},{:.5},{:.5})", a.x, a.y, a.z, b.x, b.y, b.z);
                    }
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

#[derive(Clone, Copy)]
struct ConeFit {
    axis: Vec3,
    apex: Vec3,
    /// How fast the radius grows along `axis`, in millimetres per millimetre. Positive.
    slope: f64,
}

/// Smooth patch around `seed` (stops at a crease), then a cone fit when the radius
/// changes linearly along the normals' axis. The returned cluster is the patch either way,
/// so a patch that is not a cone is not grown again.
fn grow_cone(
    _mesh: &TriMesh,
    tris: &[[Vec3; 3]],
    normals: &[Vec3],
    adj: &[Vec<usize>],
    assigned: &[bool],
    seed: usize,
    tolerance: f64,
) -> Result<(Vec<usize>, Option<ConeFit>)> {
    let Some(n0) = normals.get(seed).copied() else {
        return Ok((Vec::new(), None));
    };
    let mut cluster = vec![seed];
    let mut in_cluster = HashSet::from([seed]);
    let mut fit: Option<ConeFit> = None;
    let mut stack = vec![seed];
    let mut steps = 0usize;
    while let Some(ti) = stack.pop() {
        steps += 1;
        if steps > MAX_CLUSTER_TRIS {
            return Err(invalid("a conical region exceeds the conversion budget"));
        }
        let Some(n_here) = normals.get(ti).copied() else { continue };
        for &nb in adj.get(ti).into_iter().flatten() {
            if assigned.get(nb).copied().unwrap_or(true) || in_cluster.contains(&nb) {
                continue;
            }
            if cluster.len() > MAX_CLUSTER_TRIS {
                return Err(invalid("a conical region exceeds the conversion budget"));
            }
            let Some(nn) = normals.get(nb).copied() else { continue };
            // A right-angle edge is a cap or a side wall, not more of this cone.
            if nn.dot(n_here) < 0.70 {
                continue;
            }
            let take = if let Some(cone) = fit {
                triangle_on_cone(tris, normals, nb, &cone, tolerance)
            } else if nn.dot(n0) > 0.97 || (cluster.len() < 48 && nn.dot(n_here) > 0.90) {
                // Same facet, then the next facet, until the patch is large enough to fit.
                true
            } else {
                false
            };
            if !take {
                continue;
            }
            in_cluster.insert(nb);
            cluster.push(nb);
            stack.push(nb);
            if cluster.len() >= 8 && (fit.is_none() || cluster.len().is_multiple_of(16)) {
                fit = fit_cone(tris, normals, &cluster, tolerance).or(fit);
            }
        }
    }
    if cluster.len() < 8 {
        return Ok((cluster, None));
    }
    let Some(cone) = fit.or_else(|| fit_cone(tris, normals, &cluster, tolerance)) else {
        return Ok((cluster, None));
    };
    // The early triangles were admitted before the fit existed. Keep only those on the cone,
    // then pick up any neighbour the first pass stopped short of.
    cluster.retain(|&t| t == seed || triangle_on_cone(tris, normals, t, &cone, tolerance));
    in_cluster.clear();
    in_cluster.extend(cluster.iter().copied());
    if !in_cluster.contains(&seed) {
        cluster.push(seed);
        in_cluster.insert(seed);
    }
    let mut extra = cluster.clone();
    let mut guard = 0usize;
    while let Some(ti) = extra.pop() {
        guard += 1;
        if guard > MAX_CLUSTER_TRIS {
            break;
        }
        for &nb in adj.get(ti).into_iter().flatten() {
            if assigned.get(nb).copied().unwrap_or(true) || !in_cluster.insert(nb) {
                continue;
            }
            if triangle_on_cone(tris, normals, nb, &cone, tolerance) {
                cluster.push(nb);
                extra.push(nb);
            } else {
                in_cluster.remove(&nb);
            }
        }
    }
    if cluster.len() < 8 {
        return Ok((cluster, None));
    }
    let cone = fit_cone(tris, normals, &cluster, tolerance).unwrap_or(cone);
    Ok((cluster, Some(cone)))
}

fn triangle_on_cone(tris: &[[Vec3; 3]], normals: &[Vec3], ti: usize, cone: &ConeFit, tolerance: f64) -> bool {
    let (Some(tri), Some(nn)) = (tris.get(ti), normals.get(ti).copied()) else { return false };
    let cos = cone.slope / (1.0 + cone.slope * cone.slope).sqrt();
    if (nn.dot(cone.axis).abs() - cos).abs() > 0.15 {
        return false;
    }
    tri.iter().all(|p| {
        let d = *p - cone.apex;
        let t = d.dot(cone.axis);
        let r = (d - cone.axis * t).len();
        t > tolerance && (r - cone.slope * t).abs() <= tolerance * 4.
    })
}

fn fit_cone(tris: &[[Vec3; 3]], normals: &[Vec3], cluster: &[usize], tolerance: f64) -> Option<ConeFit> {
    let mut pts = Vec::new();
    let mut nrms = Vec::new();
    for &t in cluster {
        let Some(tri) = tris.get(t) else { continue };
        let Some(nn) = normals.get(t).copied() else { continue };
        pts.push((tri[0] + tri[1] + tri[2]) * (1. / 3.));
        nrms.push(nn);
    }
    if pts.len() < 8 {
        return None;
    }
    let axis = axis_of_normals(&nrms)?;
    let cos = nrms.iter().map(|n| n.dot(axis)).sum::<f64>() / nrms.len() as f64;
    // A cylinder's normals are perpendicular to its axis, a plane's are parallel to it.
    if cos.abs() < 0.08 || cos.abs() > 0.92 {
        return None;
    }
    if nrms.iter().any(|n| (n.dot(axis) - cos).abs() > 0.15) {
        return None;
    }
    let apex = fit_apex(&pts, &nrms)?;
    let mut ts = Vec::with_capacity(pts.len());
    let mut rs = Vec::with_capacity(pts.len());
    for p in &pts {
        let d = *p - apex;
        let t = d.dot(axis);
        let r = (d - axis * t).len();
        if !t.is_finite() || !r.is_finite() || t.abs() < tolerance {
            return None;
        }
        ts.push(t);
        rs.push(r);
    }
    let (slope, intercept, r2) = linear_fit(&ts, &rs)?;
    if slope.abs() < 0.06 || r2 < 0.82 {
        return None;
    }
    let r_span = rs.iter().copied().fold(0.0_f64, f64::max) - rs.iter().copied().fold(f64::INFINITY, f64::min);
    let r_typ = rs.iter().copied().fold(0.0_f64, f64::max);
    if intercept.abs() > tolerance.max(r_span * 0.15) {
        return None;
    }
    // A faceted cone's normal misses the apex by a fraction of the radius. A sphere's
    // tangent planes do not share an apex, so this still rejects one.
    if pts.iter().zip(&nrms).any(|(p, n)| (*p - apex).dot(*n).abs() > tolerance.max(r_typ * 0.12)) {
        return None;
    }
    let axis = if slope < 0. { -axis } else { axis };
    let slope = slope.abs();
    let mut t_min = f64::INFINITY;
    for p in &pts {
        let t = (*p - apex).dot(axis);
        let r = (*p - apex - axis * t).len();
        if t <= tolerance || (slope * t - r).abs() > tolerance * 4. {
            return None;
        }
        t_min = t_min.min(t);
    }
    if !t_min.is_finite() {
        return None;
    }
    Some(ConeFit { axis, apex, slope })
}

fn axis_of_normals(nrms: &[Vec3]) -> Option<Vec3> {
    let mean = nrms.iter().copied().fold(Vec3::ZERO, |a, n| a + n) / nrms.len() as f64;
    let mut c = [[0.0_f64; 3]; 3];
    for n in nrms {
        let d = *n - mean;
        let v = [d.x, d.y, d.z];
        for i in 0..3 {
            for j in 0..3 {
                c[i][j] += v[i] * v[j];
            }
        }
    }
    let mut x = Vec3::new(0.17, 0.31, 0.93).normalized()?;
    let mut solved = false;
    for _ in 0..32 {
        let Some(v) = solve3(c, [x.x, x.y, x.z]) else { break };
        let next = Vec3::new(v[0], v[1], v[2]);
        if next.len() < 1e-14 {
            break;
        }
        x = next.normalized()?;
        solved = true;
    }
    if !solved {
        x = null_direction(&c)?;
    }
    let spread = nrms.iter().map(|n| (n.dot(x) - nrms[0].dot(x)).abs()).fold(0.0_f64, f64::max);
    (spread < 0.2).then_some(x)
}

/// Direction the covariance does not span, when the normals lie in one plane.
fn null_direction(c: &[[f64; 3]; 3]) -> Option<Vec3> {
    let rows = [Vec3::new(c[0][0], c[0][1], c[0][2]), Vec3::new(c[1][0], c[1][1], c[1][2]), Vec3::new(c[2][0], c[2][1], c[2][2])];
    let mut best = Vec3::ZERO;
    for i in 0..3 {
        let n = rows[i].cross(rows[(i + 1) % 3]);
        if n.len() > best.len() {
            best = n;
        }
    }
    best.normalized()
}

fn fit_apex(pts: &[Vec3], nrms: &[Vec3]) -> Option<Vec3> {
    let mut m = [[0.0_f64; 3]; 3];
    let mut rhs = [0.0_f64; 3];
    for (p, n) in pts.iter().zip(nrms) {
        let nv = [n.x, n.y, n.z];
        let k = n.dot(*p);
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += nv[i] * nv[j];
            }
            rhs[i] += nv[i] * k;
        }
    }
    let v = solve3(m, rhs)?;
    let apex = Vec3::new(v[0], v[1], v[2]);
    (apex.x.is_finite() && apex.y.is_finite() && apex.z.is_finite() && apex.len() < 1e7).then_some(apex)
}

fn linear_fit(xs: &[f64], ys: &[f64]) -> Option<(f64, f64, f64)> {
    let n = xs.len() as f64;
    if xs.len() < 4 || xs.len() != ys.len() {
        return None;
    }
    let (mut sx, mut sy, mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (x, y) in xs.iter().zip(ys) {
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        syy += y * y;
    }
    let den = n * sxx - sx * sx;
    if den.abs() < 1e-12 {
        return None;
    }
    let slope = (n * sxy - sx * sy) / den;
    let intercept = (sy - slope * sx) / n;
    let ss_tot = syy - sy * sy / n;
    let ss_res = xs
        .iter()
        .zip(ys)
        .map(|(x, y)| {
            let e = y - (intercept + slope * x);
            e * e
        })
        .sum::<f64>();
    let r2 = if ss_tot.abs() < 1e-12 { 1.0 } else { 1.0 - ss_res / ss_tot };
    (slope.is_finite() && intercept.is_finite() && r2.is_finite()).then_some((slope, intercept, r2))
}

fn solve3(m: [[f64; 3]; 3], r: [f64; 3]) -> Option<[f64; 3]> {
    let det = |a: [[f64; 3]; 3]| {
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    };
    let d = det(m);
    if d.abs() < 1e-12 {
        return None;
    }
    let mut out = [0.0; 3];
    for (k, o) in out.iter_mut().enumerate() {
        let mut mk = m;
        for (row, rv) in mk.iter_mut().zip(r) {
            row[k] = rv;
        }
        *o = det(mk) / d;
        if !o.is_finite() {
            return None;
        }
    }
    Some(out)
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
        // Wider than the final fit so a short-arc seed can still reach the rest of the hole.
        (radial.len() - cyl.radius).abs() <= (tolerance * 6.).max(0.12)
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
    // Keep walking until the wall has turned about 60°. Stopping at the first 23°
    // of fan-out fits a circle to a short arc, and that radius is too unstable to
    // claim the rest of a fine hole.
    for _depth in 0..48 {
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
                // A countersink meets the wall at a shallow crease. Its points are not on
                // the cylinder; taking them into the fit used to reject the hole. Same-facet
                // triangles (a few degrees apart) are how a fine wall is reached.
                let shares = wall_pair_shares_cylinder(tris, seed, nb, n0, nn, tolerance);
                let same_facet = d > 0.97;
                if !shares && !same_facet {
                    continue;
                }
                if shares
                    && hood_pts.len() < 480
                    && let Some(tri) = tris.get(nb)
                {
                    hood_pts.extend_from_slice(tri);
                }
                if shares && d <= MAX_WALL_DOT && d < far_dot {
                    far_dot = d;
                    far = nn;
                }
                next.push(nb);
            }
        }
        if far_dot <= 0.50 {
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
    let &n0 = nrms.first()?;
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
    let mut pts: Vec<Vec3> = seed_tri.to_vec();
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

fn fold_sliver_triangles(faces: &mut Vec<FaceSpec>, origins: &mut Vec<Vec<usize>>) {
    let mut skip = 0usize;
    for _ in 0..64 {
        let Some(rel) = faces.iter().skip(skip).position(is_sliver_triangle) else { break };
        let si = skip + rel;
        if fold_one_sliver(faces, origins, si) {
            skip = 0;
        } else {
            skip = si + 1;
        }
    }
}

fn fold_one_sliver(faces: &mut Vec<FaceSpec>, origins: &mut Vec<Vec<usize>>, si: usize) -> bool {
    let Some(edges) = faces.get(si).and_then(|face| face.loops.first()).filter(|lp| lp.len() == 3).cloned() else {
        return false;
    };
    let mut long = 0usize;
    let mut long_len = -1.0_f64;
    for (i, edge) in edges.iter().enumerate() {
        let Some((a, b)) = line_ends(edge) else { return false };
        let len = a.dist(b);
        if len > long_len {
            long = i;
            long_len = len;
        }
    }
    let Some((a, b)) = line_ends(&edges[long]) else { return false };
    let (ka, kb) = (quant_pt(a), quant_pt(b));
    let mut host: Option<(usize, usize, usize, bool)> = None;
    for (fi, face) in faces.iter().enumerate() {
        if fi == si {
            continue;
        }
        for (li, lp) in face.loops.iter().enumerate() {
            for (ei, edge) in lp.iter().enumerate() {
                let Some((c, d)) = line_ends(edge) else { continue };
                let (kc, kd) = (quant_pt(c), quant_pt(d));
                if kc == kb && kd == ka {
                    host = Some((fi, li, ei, false));
                } else if kc == ka && kd == kb {
                    host = Some((fi, li, ei, true));
                }
            }
        }
    }
    let Some((fi, li, ei, same_dir)) = host else { return false };
    let n = edges.len();
    let mut bridge: Vec<EdgeSpec> = (1..n).map(|k| edges[(long + k) % n].clone()).collect();
    if same_dir {
        reverse_loop(&mut bridge);
    }
    let Some(lp) = faces.get_mut(fi).and_then(|face| face.loops.get_mut(li)) else { return false };
    if ei >= lp.len() || lp.len() + bridge.len() > MAX_LOOP_EDGES {
        return false;
    }
    lp.splice(ei..=ei, bridge);
    faces.remove(si);
    if si < origins.len() {
        origins.remove(si);
    }
    true
}

fn is_sliver_triangle(face: &FaceSpec) -> bool {
    let Some(lp) = face.loops.first() else { return false };
    if face.loops.len() != 1 || lp.len() != 3 {
        return false;
    }
    let mut pts = [Vec3::ZERO; 3];
    for (i, edge) in lp.iter().enumerate() {
        let Some((a, _)) = line_ends(edge) else { return false };
        pts[i] = a;
    }
    (pts[1] - pts[0]).cross(pts[2] - pts[0]).len() < 1.5e-6
}

fn is_flat_sliver(face: &FaceSpec) -> bool {
    if !matches!(face.surface, SurfSpec::Plane) || face.loops.len() != 1 {
        return false;
    }
    let Some(lp) = face.loops.first() else { return false };
    if lp.len() < 3 || lp.iter().any(|edge| line_ends(edge).is_none()) {
        return false;
    }
    let (nlen, dev) = face_plane_metrics(face);
    nlen < 1e-6 && dev <= 1e-7
}

/// A welded patch with no area still occupies edges. Splice it into the neighbour across
/// its longest shared edge so those edges stay in the shell.
fn fold_flat_faces(faces: &mut Vec<FaceSpec>) {
    let mut skip = 0usize;
    for _ in 0..128 {
        let Some(rel) = faces.iter().skip(skip).position(is_flat_sliver) else { break };
        let si = skip + rel;
        if fold_face_into_neighbor(faces, si) {
            skip = 0;
        } else {
            skip = si + 1;
        }
    }
}

fn fold_face_into_neighbor(faces: &mut Vec<FaceSpec>, si: usize) -> bool {
    let Some(edges) = faces.get(si).and_then(|face| face.loops.first()).cloned() else { return false };
    if edges.len() < 3 {
        return false;
    }
    let mut order: Vec<usize> = (0..edges.len()).collect();
    order.sort_by(|&i, &j| {
        let li = line_ends(&edges[i]).map(|(a, b)| a.dist(b)).unwrap_or(0.0);
        let lj = line_ends(&edges[j]).map(|(a, b)| a.dist(b)).unwrap_or(0.0);
        lj.partial_cmp(&li).unwrap_or(std::cmp::Ordering::Equal)
    });
    for long in order {
        let Some((a, b)) = line_ends(&edges[long]) else { continue };
        let (ka, kb) = (quant_pt(a), quant_pt(b));
        let mut host: Option<(usize, usize, usize, bool)> = None;
        for (fi, face) in faces.iter().enumerate() {
            if fi == si || !matches!(face.surface, SurfSpec::Plane) {
                continue;
            }
            for (li, lp) in face.loops.iter().enumerate() {
                for (ei, edge) in lp.iter().enumerate() {
                    let Some((c, d)) = line_ends(edge) else { continue };
                    let (kc, kd) = (quant_pt(c), quant_pt(d));
                    if kc == kb && kd == ka {
                        host = Some((fi, li, ei, false));
                    } else if kc == ka && kd == kb {
                        host = Some((fi, li, ei, true));
                    }
                }
            }
        }
        let Some((fi, li, ei, same_dir)) = host else { continue };
        let n = edges.len();
        let mut bridge: Vec<EdgeSpec> = (1..n).map(|k| edges[(long + k) % n].clone()).collect();
        if same_dir {
            reverse_loop(&mut bridge);
        }
        let Some(lp) = faces.get(fi).and_then(|face| face.loops.get(li)) else { continue };
        if ei >= lp.len() || lp.len() + bridge.len() > MAX_LOOP_EDGES {
            continue;
        }
        let mut merged = lp.clone();
        merged.splice(ei..=ei, bridge);
        if !loop_is_simple(&merged) {
            continue;
        }
        if let Some(lp) = faces.get_mut(fi).and_then(|face| face.loops.get_mut(li)) {
            *lp = merged;
        }
        faces.remove(si);
        return true;
    }
    false
}

/// Join two planar faces that share one edge when both still lie on one plane.
/// One pass pairs disjoint edges, so a flat of many facets collapses in a few passes.
fn merge_adjacent_coplanar(faces: &mut Vec<FaceSpec>, origins: &mut Vec<Vec<usize>>, tol: f64, cos_ang: f64) {
    for _ in 0..24 {
        let mut located: HashMap<(i64, i64, i64, i64, i64, i64), Vec<(usize, usize, bool)>> = HashMap::new();
        for (fi, face) in faces.iter().enumerate() {
            if !matches!(face.surface, SurfSpec::Plane) || face.loops.len() != 1 {
                continue;
            }
            let Some(lp) = face.loops.first() else { continue };
            if lp.iter().any(|edge| line_ends(edge).is_none()) {
                continue;
            }
            for (ei, edge) in lp.iter().enumerate() {
                let Some((a, b)) = line_ends(edge) else { continue };
                let (ka, kb) = (quant_pt(a), quant_pt(b));
                let (key, flipped) = if (kb.0, kb.1, kb.2) < (ka.0, ka.1, ka.2) {
                    ((kb.0, kb.1, kb.2, ka.0, ka.1, ka.2), true)
                } else {
                    ((ka.0, ka.1, ka.2, kb.0, kb.1, kb.2), false)
                };
                located.entry(key).or_default().push((fi, ei, flipped));
            }
        }
        let mut pairs: Vec<(usize, usize, usize, usize)> = Vec::new();
        let mut used = vec![false; faces.len()];
        for sites in located.values() {
            if sites.len() != 2 {
                continue;
            }
            let (fa, ea, da) = sites[0];
            let (fb, eb, db) = sites[1];
            if fa == fb || used.get(fa).copied().unwrap_or(true) || used.get(fb).copied().unwrap_or(true) || da == db {
                continue;
            }
            let (Some(left), Some(right)) = (faces.get(fa), faces.get(fb)) else { continue };
            if !coplanar_pair(left, right, tol, cos_ang) {
                continue;
            }
            if let Some(slot) = used.get_mut(fa) {
                *slot = true;
            }
            if let Some(slot) = used.get_mut(fb) {
                *slot = true;
            }
            pairs.push((fa, ea, fb, eb));
        }
        if pairs.is_empty() {
            break;
        }
        pairs.sort_by(|a, b| b.0.max(b.2).cmp(&a.0.max(a.2)));
        let mut merged = 0usize;
        for pair_i in 0..pairs.len() {
            let (fa, ea, fb, eb) = pairs[pair_i];
            let Some(gone) = merge_plane_pair(faces, origins, fa, ea, fb, eb) else { continue };
            merged += 1;
            for later in pairs.iter_mut().skip(pair_i + 1) {
                if later.0 > gone {
                    later.0 -= 1;
                }
                if later.2 > gone {
                    later.2 -= 1;
                }
            }
        }
        if merged == 0 {
            break;
        }
    }
}

fn coplanar_pair(left: &FaceSpec, right: &FaceSpec, tol: f64, cos_ang: f64) -> bool {
    let (Some(nl), Some(nr)) = (face_plane_normal(left), face_plane_normal(right)) else { return false };
    if nl.dot(nr) < cos_ang {
        return false;
    }
    let mut pts = Vec::new();
    for face in [left, right] {
        for edge in face.loops.iter().flatten() {
            if let Some((a, _)) = line_ends(edge) {
                pts.push(a);
            }
        }
    }
    if pts.len() < 3 {
        return false;
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
    let Some(nn) = n.normalized() else { return false };
    let d = c.dot(nn);
    pts.iter().all(|p| (p.dot(nn) - d).abs() <= tol)
}

/// Pull each planar corner onto the planes of the faces that use it, moving at most `tol`.
/// Shared corners stay one point, so the shell still meets, and truck can attach the plane.
fn flatten_plane_corners(faces: &mut [FaceSpec], tol: f64) {
    if !(tol.is_finite() && tol > 0.0) {
        return;
    }
    let planes: Vec<Option<(Vec3, f64)>> = faces.iter().map(face_plane_nd).collect();
    let mut groups: HashMap<(i64, i64, i64), Vec<(usize, usize, usize, bool)>> = HashMap::new();
    for (fi, face) in faces.iter().enumerate() {
        if !matches!(face.surface, SurfSpec::Plane) {
            continue;
        }
        for (li, lp) in face.loops.iter().enumerate() {
            for (ei, edge) in lp.iter().enumerate() {
                let EdgeSpec::Line { a, b } = edge else { continue };
                groups.entry(quant_pt(*a)).or_default().push((fi, li, ei, false));
                groups.entry(quant_pt(*b)).or_default().push((fi, li, ei, true));
            }
        }
    }
    let mut updates: Vec<((usize, usize, usize, bool), Vec3)> = Vec::new();
    for uses in groups.values() {
        let Some(&(fi, li, ei, is_b)) = uses.first() else { continue };
        let Some(p0) = line_end_at(faces, fi, li, ei, is_b) else { continue };
        let mut seen: Vec<(Vec3, f64)> = Vec::new();
        for &(fj, _, _, _) in uses {
            let Some(Some((n, d))) = planes.get(fj) else { continue };
            if seen.iter().any(|(nn, dd)| nn.dot(*n) > 0.9999 && (d - dd).abs() < 1e-9) {
                continue;
            }
            seen.push((*n, *d));
        }
        let p = project_onto_planes(p0, &seen);
        if !p.is_finite() || p.dist(p0) > tol {
            continue;
        }
        for &site in uses {
            updates.push((site, p));
        }
    }
    for ((fi, li, ei, is_b), p) in updates {
        set_line_end(faces, fi, li, ei, is_b, p);
    }
}

fn face_plane_nd(face: &FaceSpec) -> Option<(Vec3, f64)> {
    if !matches!(face.surface, SurfSpec::Plane) {
        return None;
    }
    let n = face_plane_normal(face)?;
    let mut c = Vec3::ZERO;
    let mut count = 0.0;
    for edge in face.loops.iter().flatten() {
        if let Some((a, _)) = line_ends(edge) {
            c += a;
            count += 1.0;
        }
    }
    if count < 3.0 {
        return None;
    }
    c = c * (1.0 / count);
    Some((n, c.dot(n)))
}

fn line_end_at(faces: &[FaceSpec], fi: usize, li: usize, ei: usize, is_b: bool) -> Option<Vec3> {
    let edge = faces.get(fi)?.loops.get(li)?.get(ei)?;
    let (a, b) = line_ends(edge)?;
    Some(if is_b { b } else { a })
}

fn set_line_end(faces: &mut [FaceSpec], fi: usize, li: usize, ei: usize, is_b: bool, p: Vec3) {
    let Some(edge) = faces.get_mut(fi).and_then(|face| face.loops.get_mut(li)).and_then(|lp| lp.get_mut(ei)) else { return };
    if let EdgeSpec::Line { a, b } = edge {
        if is_b { *b = p } else { *a = p }
    }
}

fn project_onto_planes(p: Vec3, planes: &[(Vec3, f64)]) -> Vec3 {
    if planes.is_empty() {
        return p;
    }
    if let [(n, d)] = planes {
        return p - *n * (p.dot(*n) - d);
    }
    // Soft constraints: stay near p and on every plane that meets here.
    let w = 1.0e8;
    let mut a = [[0.0; 3]; 3];
    a[0][0] = 1.0;
    a[1][1] = 1.0;
    a[2][2] = 1.0;
    let mut rhs = [p.x, p.y, p.z];
    for (n, d) in planes {
        let nrm = [n.x, n.y, n.z];
        for row in 0..3 {
            for col in 0..3 {
                a[row][col] += w * nrm[row] * nrm[col];
            }
            rhs[row] += w * d * nrm[row];
        }
    }
    solve3(a, rhs).map(|s| Vec3::new(s[0], s[1], s[2])).unwrap_or(p)
}

fn face_plane_normal(face: &FaceSpec) -> Option<Vec3> {
    let mut pts = Vec::new();
    for edge in face.loops.iter().flatten() {
        if let Some((a, _)) = line_ends(edge) {
            pts.push(a);
        }
    }
    if pts.len() < 3 {
        return None;
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
    n.normalized()
}

fn merge_plane_pair(faces: &mut Vec<FaceSpec>, origins: &mut Vec<Vec<usize>>, fa: usize, ea: usize, fb: usize, eb: usize) -> Option<usize> {
    let (host, he, other, oe) = if fa < fb { (fa, ea, fb, eb) } else { (fb, eb, fa, ea) };
    let Some(host_loop) = faces.get(host).and_then(|face| face.loops.first()).cloned() else { return None };
    let Some(other_loop) = faces.get(other).and_then(|face| face.loops.first()).cloned() else { return None };
    if he >= host_loop.len() || oe >= other_loop.len() || host_loop.len() + other_loop.len() > MAX_LOOP_EDGES {
        return None;
    }
    let Some((ha, hb)) = line_ends(&host_loop[he]) else { return None };
    let Some((oa, ob)) = line_ends(&other_loop[oe]) else { return None };
    let mut bridge: Vec<EdgeSpec> = (1..other_loop.len()).map(|k| other_loop[(oe + k) % other_loop.len()].clone()).collect();
    let opposite = quant_pt(oa) == quant_pt(hb) && quant_pt(ob) == quant_pt(ha);
    let same = quant_pt(oa) == quant_pt(ha) && quant_pt(ob) == quant_pt(hb);
    if same {
        reverse_loop(&mut bridge);
    } else if !opposite {
        return None;
    }
    let mut new_loop = Vec::with_capacity(host_loop.len() + bridge.len());
    new_loop.extend(host_loop[..he].iter().cloned());
    new_loop.extend(bridge);
    new_loop.extend(host_loop[he + 1..].iter().cloned());
    if !loop_is_simple(&new_loop) {
        return None;
    }
    if let Some(lp) = faces.get_mut(host).and_then(|face| face.loops.get_mut(0)) {
        *lp = new_loop;
    } else {
        return None;
    }
    if host < origins.len() && other < origins.len() {
        let extra = origins[other].clone();
        origins[host].extend(extra);
    }
    faces.remove(other);
    if other < origins.len() {
        origins.remove(other);
    }
    Some(other)
}

fn line_ends(edge: &EdgeSpec) -> Option<(Vec3, Vec3)> {
    match edge {
        EdgeSpec::Line { a, b } => Some((*a, *b)),
        _ => None,
    }
}

fn unify_near_corners(faces: &mut [FaceSpec], tol: f64) {
    let Some((reps, bins)) = corner_reps(faces, tol) else { return };
    apply_corner_reps(faces, &reps, &bins, tol);
}

/// Move `loose` corners onto corners already chosen on `anchors`. Plane wires stay put;
/// an open-cylinder end that sits a few hundredths of a micrometre off then shares the edge.
fn snap_partial_corners(loose: &mut [FaceSpec], anchors: &[FaceSpec], tol: f64) {
    if !(tol.is_finite() && tol > 0.0) {
        return;
    }
    let mut reps: Vec<Vec3> = Vec::new();
    for face in anchors {
        if !matches!(face.surface, SurfSpec::Plane) {
            continue;
        }
        for edge in face.loops.iter().flatten() {
            let Some((a, b)) = line_ends(edge) else { continue };
            for p in [a, b] {
                if reps.iter().any(|r| r.dist(p) <= 1e-6) {
                    continue;
                }
                reps.push(p);
            }
        }
    }
    if reps.is_empty() {
        return;
    }
    let nearest = |p: Vec3| -> Option<Vec3> {
        let mut best: Option<(f64, Vec3)> = None;
        let mut second = f64::INFINITY;
        for r in &reps {
            let d = r.dist(p);
            if best.is_some_and(|(bd, _)| d < bd) || best.is_none() {
                second = best.map(|(bd, _)| bd).unwrap_or(f64::INFINITY);
                best = Some((d, *r));
            } else if d < second {
                second = d;
            }
        }
        let (d, r) = best?;
        if d <= tol && second > d * 3.0 && second > 1e-4 { Some(r) } else { None }
    };
    for face in loose.iter_mut() {
        for edge in face.loops.iter_mut().flatten() {
            let EdgeSpec::Line { a, b } = edge else { continue };
            let na = nearest(*a);
            let nb = nearest(*b);
            if na.is_some_and(|p| nb.is_some_and(|q| p.dist(q) < 1e-6)) {
                continue;
            }
            if let Some(p) = na {
                *a = p;
            }
            if let Some(p) = nb {
                *b = p;
            }
        }
    }
}

fn corner_reps(faces: &[FaceSpec], tol: f64) -> Option<(Vec<Vec3>, HashMap<(i64, i64, i64), Vec<usize>>)> {
    if !(tol.is_finite() && tol > 0.0) {
        return None;
    }
    let mut reps: Vec<Vec3> = Vec::new();
    let mut bins: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    let key = |p: Vec3| ((p.x / tol).round() as i64, (p.y / tol).round() as i64, (p.z / tol).round() as i64);
    let mut remember = |p: Vec3| {
        let k = key(p);
        for dz in -1i64..=1 {
            for dy in -1i64..=1 {
                for dx in -1i64..=1 {
                    let Some(ids) = bins.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) else { continue };
                    for &id in ids {
                        if reps.get(id).is_some_and(|r| r.dist(p) <= tol) {
                            return;
                        }
                    }
                }
            }
        }
        let id = reps.len();
        reps.push(p);
        bins.entry(k).or_default().push(id);
    };
    for face in faces {
        for edge in face.loops.iter().flatten() {
            match edge {
                EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => {
                    remember(*a);
                    remember(*b);
                }
                EdgeSpec::Chain { ctrl, .. } => {
                    if let Some(a) = ctrl.first() {
                        remember(*a);
                    }
                    if let Some(b) = ctrl.last() {
                        remember(*b);
                    }
                }
            }
        }
    }
    Some((reps, bins))
}

fn apply_corner_reps(faces: &mut [FaceSpec], reps: &[Vec3], bins: &HashMap<(i64, i64, i64), Vec<usize>>, tol: f64) {
    let key = |p: Vec3| ((p.x / tol).round() as i64, (p.y / tol).round() as i64, (p.z / tol).round() as i64);
    let canon = |p: Vec3| -> Vec3 {
        let k = key(p);
        let mut best: Option<Vec3> = None;
        let mut best_d = tol;
        for dz in -1i64..=1 {
            for dy in -1i64..=1 {
                for dx in -1i64..=1 {
                    let Some(ids) = bins.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) else { continue };
                    for &id in ids {
                        let Some(r) = reps.get(id) else { continue };
                        let d = r.dist(p);
                        if d <= best_d {
                            best = Some(*r);
                            best_d = d;
                        }
                    }
                }
            }
        }
        best.unwrap_or(p)
    };
    for face in faces.iter_mut() {
        for edge in face.loops.iter_mut().flatten() {
            match edge {
                EdgeSpec::Line { a, b } => {
                    *a = canon(*a);
                    *b = canon(*b);
                }
                EdgeSpec::Arc { a, b, .. } => {
                    *a = canon(*a);
                    *b = canon(*b);
                }
                EdgeSpec::Chain { ctrl, .. } => {
                    let mut c = ctrl.to_vec();
                    if let Some(a) = c.first_mut() {
                        *a = canon(*a);
                    }
                    if let Some(b) = c.last_mut() {
                        *b = canon(*b);
                    }
                    *ctrl = c.into();
                }
            }
        }
    }
}

fn quant_pt(p: Vec3) -> (i64, i64, i64) {
    ((p.x * 1e6).round() as i64, (p.y * 1e6).round() as i64, (p.z * 1e6).round() as i64)
}

/// Truck refuses a plane when the boundary normal vanishes or a point is more than 1e-6 mm off.
/// Arcs stay: their control points are not the geometric midpoint, and a cap must keep them.
fn face_not_a_plane(face: &FaceSpec) -> bool {
    if face.loops.iter().flatten().any(|edge| matches!(edge, EdgeSpec::Arc { .. })) {
        return false;
    }
    let (nlen, dev) = face_plane_metrics(face);
    nlen < 1e-7 || dev > 1e-7
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

#[cfg(test)]
fn once_edge_samples(faces: &[FaceSpec], limit: usize) -> Vec<(Vec3, Vec3)> {
    let q = |p: Vec3| ((p.x * 1e6).round() as i64, (p.y * 1e6).round() as i64, (p.z * 1e6).round() as i64);
    let mut uses: HashMap<(i64, i64, i64, i64, i64, i64), (usize, Vec3, Vec3)> = HashMap::new();
    for face in faces {
        for edge in face.loops.iter().flatten() {
            let (a, b) = match edge {
                EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => (*a, *b),
                EdgeSpec::Chain { ctrl, .. } => (ctrl.first().copied().unwrap_or(Vec3::ZERO), ctrl.last().copied().unwrap_or(Vec3::ZERO)),
            };
            let (mut ka, mut kb, mut pa, mut pb) = (q(a), q(b), a, b);
            if kb < ka {
                std::mem::swap(&mut ka, &mut kb);
                std::mem::swap(&mut pa, &mut pb);
            }
            let slot = uses.entry((ka.0, ka.1, ka.2, kb.0, kb.1, kb.2)).or_insert((0, pa, pb));
            slot.0 += 1;
        }
    }
    uses.values().filter(|(c, _, _)| *c == 1).take(limit).map(|(_, a, b)| (*a, *b)).collect()
}

fn edges_on_plane(edges: &[EdgeSpec], n: Vec3, d: f64) -> bool {
    edges.iter().all(|e| match e {
        EdgeSpec::Line { a, b } => on_plane(*a, n, d) && on_plane(*b, n, d),
        EdgeSpec::Arc { a, b, mid } => on_plane(*a, n, d) && on_plane(*b, n, d) && on_plane(*mid, n, d),
        EdgeSpec::Chain { ctrl, .. } => ctrl.iter().copied().all(|p| on_plane(p, n, d)),
    })
}

#[cfg(test)]
fn rim_plane_gap(edges: &[EdgeSpec], n: Vec3, d: f64) -> f64 {
    let mut gap = 0.0_f64;
    for edge in edges {
        let pts = match edge {
            EdgeSpec::Line { a, b } => [*a, *b, *b],
            EdgeSpec::Arc { a, b, mid } => [*a, *b, *mid],
            EdgeSpec::Chain { ctrl, .. } => [*ctrl.first().unwrap_or(&Vec3::ZERO), *ctrl.last().unwrap_or(&Vec3::ZERO), Vec3::ZERO],
        };
        for p in pts {
            gap = gap.max((p.dot(n) - d).abs());
        }
    }
    gap
}

fn circle_near_plane(edges: &[EdgeSpec], n: Vec3, d: f64) -> bool {
    edges.iter().all(|edge| match edge {
        EdgeSpec::Arc { a, b, mid } => [a, b, mid].iter().all(|p| (p.dot(n) - d).abs() < 0.05),
        EdgeSpec::Line { a, b } => (a.dot(n) - d).abs() < 0.05 && (b.dot(n) - d).abs() < 0.05,
        EdgeSpec::Chain { ctrl, .. } => ctrl.iter().all(|p| (p.dot(n) - d).abs() < 0.05),
    })
}

fn snap_edge_plane(edge: &mut EdgeSpec, n: Vec3, d: f64) {
    let snap = |p: &mut Vec3| {
        let k = p.dot(n) - d;
        *p -= n * k;
    };
    match edge {
        EdgeSpec::Line { a, b } => {
            snap(a);
            snap(b);
        }
        EdgeSpec::Arc { a, b, mid } => {
            snap(a);
            snap(b);
            snap(mid);
        }
        EdgeSpec::Chain { ctrl, .. } => {
            let mut pts: Vec<Vec3> = ctrl.iter().copied().collect();
            for p in &mut pts {
                snap(p);
            }
            *ctrl = pts.into();
        }
    }
}

/// The cap locked to this rim, if one plane owns it.
fn rim_plane(planes: &[HeldPlane], owners: &[Vec<usize>], rim: &[u32]) -> Option<(Vec3, f64)> {
    let mut votes: HashMap<usize, usize> = HashMap::new();
    for &vi in rim {
        if let Some(ids) = owners.get(vi as usize) {
            for &pi in ids {
                *votes.entry(pi).or_default() += 1;
            }
        }
    }
    let (&pi, _) = votes.iter().max_by_key(|(_, c)| *c)?;
    let plane = planes.get(pi)?;
    Some((plane.n, plane.d))
}

fn loop_meets_leftover(lp: &[u32], edges: &HashSet<(u32, u32)>) -> bool {
    if lp.len() < 2 {
        return false;
    }
    let closes = lp.windows(2).any(|w| edges.contains(&(w[0].min(w[1]), w[0].max(w[1]))));
    let Some(&last) = lp.last() else { return closes };
    closes || edges.contains(&(lp[0].min(last), lp[0].max(last)))
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
    leftover_edges: &HashSet<(u32, u32)>,
    plane: Option<PlaneFit>,
    used_rims: &mut HashSet<Vec<u32>>,
) -> Result<FaceSpec> {
    let at = |i: u32| positions.get(i as usize).copied();
    // Single triangle: keep the mesh winding exactly (no simplify / re-orient) so a closed
    // manifold triangulated shell sews without inventing gaps at shared edges.
    if let [ti] = *cluster {
        let tri = mesh.triangles.get(ti).ok_or_else(|| invalid("missing triangle"))?;
        let (Some(a), Some(b), Some(c)) = (at(tri[0]), at(tri[1]), at(tri[2])) else {
            return Err(invalid("missing vertex"));
        };
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
        let edge_blocked = loop_meets_leftover(&lp, leftover_edges);
        #[cfg(test)]
        if !by_rim.is_empty() {
            let here: HashSet<u32> = lp.iter().copied().collect();
            let mut best = 0usize;
            for key in by_rim.keys() {
                let hit = key.iter().filter(|v| here.contains(v)).count();
                best = best.max(hit.saturating_mul(1000) / key.len().max(1));
            }
            if let Some(edges) = by_rim.get(&rim_key) {
                best = if edge_blocked {
                    3000
                } else if !edges_on_plane(edges, n, plane.d) {
                    let dev = rim_plane_gap(edges, n, plane.d);
                    10_000 + (dev * 1.0e6).min(1.0e6) as usize
                } else {
                    2000
                };
            }
            recognize_stats::note_overlap(best);
        }
        let touches_leftover = lp.iter().any(|i| keep_verts.contains(i));
        // A leftover triangle that only shares a rim vertex still has its own edges.
        // The circle is refused only when a leftover triangle owns one of this loop's edges.
        let from_rim = if edge_blocked { None } else { by_rim.get(&rim_key).filter(|edges| edges_on_plane(edges, n, plane.d)).cloned() };
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
        for (j, point) in pts.iter().enumerate().take(n) {
            if j == i || j == (i + 1) % n {
                continue;
            }
            if point_line_dist(*point, a, b) <= 1e-6 {
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

/// A cylinder wall that does not close into a tube: a channel or a half-round.
/// Its boundary is the mesh polyline, so neighbouring faces share those edges.
struct PartialCyl {
    cluster: Vec<usize>,
    cyl: CylFit,
}

struct PendingCone {
    cluster: Vec<usize>,
    axis: Vec3,
    apex: Vec3,
    t0: f64,
    t1: f64,
    r0: f64,
    r1: f64,
    rim0: Vec<u32>,
    rim1: Vec<u32>,
}

impl PendingCone {
    /// A cylinder record so the rim lock can place the cap on this circle.
    fn cyl_at(&self, _t: f64, radius: f64) -> CylFit {
        CylFit { axis: self.axis, origin: self.apex, radius }
    }
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
    let loops: Vec<_> = loops.into_iter().filter(|lp| lp.len() >= 6).collect();
    if loops.len() != 2 {
        #[cfg(test)]
        {
            recognize_stats::add(recognize_stats::loops());
            recognize_stats::note_boundary(loops.len(), cluster.len());
        }
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
            #[cfg(test)]
            recognize_stats::add(recognize_stats::flat());
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
    // A semicircular rim has the same vertex count as a cap. Treating it as a full
    // circle closes the missing half and the solid is a complete cylinder.
    if !rim_covers_the_turn(mesh, &rim0, cyl) || !rim_covers_the_turn(mesh, &rim1, cyl) {
        return Ok(None);
    }
    Ok(Some(PendingTube { cluster: cluster.to_vec(), cyl: *cyl, h0, h1, rim0, rim1 }))
}

/// The rim runs the whole way around the axis. One facet may be missing; a half-round may not.
fn rim_covers_the_turn(mesh: &TriMesh, rim: &[u32], cyl: &CylFit) -> bool {
    let Some(axis) = cyl.axis.normalized() else { return false };
    let u = axis.any_perp();
    let v = axis.cross(u);
    let mut angles = Vec::new();
    for &i in rim {
        let Some(p) = mesh.positions.get(i as usize) else { continue };
        let d = *p - cyl.origin;
        let radial = d - axis * d.dot(axis);
        if radial.len() < cyl.radius * 0.5 {
            continue;
        }
        angles.push(radial.dot(v).atan2(radial.dot(u)));
    }
    let Some((_, gap, span)) = angular_opening(&angles) else { return false };
    gap < 0.55 && span > TAU - 0.55
}

fn try_pending_cone(mesh: &TriMesh, cluster: &[usize], cone: &ConeFit, tolerance: f64) -> Result<Option<PendingCone>> {
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
            hsum += (*p - cone.apex).dot(cone.axis);
            count += 1.;
        }
        if count < 6. {
            return Ok(None);
        }
        let t = hsum / count;
        let flat = lp.iter().all(|&i| mesh.positions.get(i as usize).is_some_and(|p| ((*p - cone.apex).dot(cone.axis) - t).abs() <= tolerance * 4.));
        if !t.is_finite() || t <= tolerance || !flat {
            return Ok(None);
        }
        let radius = cone.slope * t;
        let round = lp.iter().all(|&i| {
            mesh.positions.get(i as usize).is_some_and(|p| {
                let d = *p - cone.apex;
                let h = d.dot(cone.axis);
                (d - cone.axis * h).len() - radius < tolerance * 4. && radius - (d - cone.axis * h).len() < tolerance * 4.
            })
        });
        if !round || radius <= tolerance {
            return Ok(None);
        }
        rims.push((t, radius, lp));
    }
    rims.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (Some((t0, r0, rim0)), Some((t1, r1, rim1))) = (rims.first().cloned(), rims.get(1).cloned()) else {
        return Ok(None);
    };
    if t1 - t0 <= tolerance || (r0 - r1).abs() <= tolerance * 2. {
        return Ok(None);
    }
    Ok(Some(PendingCone { cluster: cluster.to_vec(), axis: cone.axis, apex: cone.apex, t0, t1, r0, r1, rim0, rim1 }))
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

/// Pull unassigned triangles that share an edge with a plane, and lie within `tolerance` of it,
/// into that plane. Several rounds, so a strip of slivers can follow the first one in.
fn absorb_coplanar_triangles(mesh: &TriMesh, adj: &[Vec<usize>], planes: &mut [HeldPlane], assigned: &mut [bool], tolerance: f64) {
    let mut owner = vec![None; mesh.triangles.len()];
    for (pi, plane) in planes.iter().enumerate() {
        for &t in &plane.cluster {
            if let Some(slot) = owner.get_mut(t) {
                *slot = Some(pi);
            }
        }
    }
    for _ in 0..8 {
        let mut added: Vec<(usize, usize)> = Vec::new();
        for ti in 0..mesh.triangles.len() {
            if assigned.get(ti).copied().unwrap_or(true) {
                continue;
            }
            let Some(tri) = mesh.triangles.get(ti) else { continue };
            let mut votes: Vec<(usize, usize)> = Vec::new();
            for &nb in adj.get(ti).into_iter().flatten() {
                let Some(Some(pi)) = owner.get(nb).copied() else { continue };
                if let Some((_, count)) = votes.iter_mut().find(|(plane, _)| *plane == pi) {
                    *count += 1;
                } else {
                    votes.push((pi, 1));
                }
            }
            // Most shared edges, then the earlier plane, so the choice does not depend on hash order.
            let Some(&(pi, _)) = votes.iter().max_by(|a, b| a.1.cmp(&b.1).then(b.0.cmp(&a.0))) else { continue };
            let Some(plane) = planes.get(pi) else { continue };
            let on_plane = tri.iter().all(|&vi| mesh.positions.get(vi as usize).is_some_and(|p| (p.dot(plane.n) - plane.d).abs() <= tolerance));
            if on_plane {
                added.push((pi, ti));
            }
        }
        if added.is_empty() {
            break;
        }
        for (pi, ti) in added {
            if assigned.get(ti).copied().unwrap_or(true) {
                continue;
            }
            if let Some(plane) = planes.get_mut(pi) {
                plane.cluster.push(ti);
            }
            if let Some(slot) = assigned.get_mut(ti) {
                *slot = true;
            }
            if let Some(slot) = owner.get_mut(ti) {
                *slot = Some(pi);
            }
        }
    }
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

/// Largest empty angle around the axis, and the span of the wall on the other side.
/// `gap_at` is the start of the empty sector. Fewer than four distinct angles is not a wall.
fn angular_opening(angles: &[f64]) -> Option<(f64, f64, f64)> {
    if angles.len() < 4 {
        return None;
    }
    let mut uniq: Vec<f64> = angles.iter().copied().filter(|a| a.is_finite()).collect();
    if uniq.len() < 4 {
        return None;
    }
    uniq.sort_by(|a, b| a.total_cmp(b));
    let mut collapsed = Vec::with_capacity(uniq.len());
    collapsed.push(uniq[0]);
    for a in uniq.into_iter().skip(1) {
        if let Some(prev) = collapsed.last().copied()
            && a - prev > 1e-3
        {
            collapsed.push(a);
        }
    }
    if collapsed.len() < 4 {
        return None;
    }
    let mut best_gap = 0.0;
    let mut gap_at = collapsed[0];
    for pair in collapsed.windows(2) {
        let (Some(left), Some(right)) = (pair.first().copied(), pair.get(1).copied()) else { continue };
        let g = right - left;
        if g > best_gap {
            best_gap = g;
            gap_at = left;
        }
    }
    let (Some(first), Some(last)) = (collapsed.first().copied(), collapsed.last().copied()) else {
        return None;
    };
    let wrap = first + TAU - last;
    if wrap > best_gap {
        best_gap = wrap;
        gap_at = last;
    }
    let span = TAU - best_gap;
    (span.is_finite() && best_gap.is_finite()).then_some((gap_at, best_gap, span))
}

/// The wall leaves a real opening around its axis (a channel or a half-round), so it can be
/// one face without wrapping the parameter seam. A complete tube's only gap is one facet.
fn cylinder_opens_wide(mesh: &TriMesh, cluster: &[usize], cyl: &CylFit) -> bool {
    let Ok(loops) = boundary_loops(mesh, cluster) else { return false };
    if loops.is_empty() || loops.len() > 32 {
        return false;
    }
    let Some(axis) = cyl.axis.normalized() else { return false };
    let u = axis.any_perp();
    let v = axis.cross(u);
    let mut angles = Vec::new();
    for lp in &loops {
        if lp.len() > MAX_LOOP_EDGES {
            return false;
        }
        for &i in lp {
            let Some(p) = mesh.positions.get(i as usize) else { continue };
            let d = *p - cyl.origin;
            let radial = d - axis * d.dot(axis);
            if radial.len() < cyl.radius * 0.5 {
                continue;
            }
            angles.push(radial.dot(v).atan2(radial.dot(u)));
        }
    }
    let Some((_, gap, span)) = angular_opening(&angles) else { return false };
    gap >= 0.45 && span >= 0.4 && span < TAU - 0.2
}

/// One revolved face for an open cylinder. The boundary is every mesh edge, so a
/// neighbouring plane or triangle meets it on the same line.
fn partial_cylinder_face(mesh: &TriMesh, positions: &[Vec3], cluster: &[usize], cyl: &CylFit) -> Result<FaceSpec> {
    let loops = boundary_loops(mesh, cluster)?;
    if loops.is_empty() || loops.len() > 32 {
        return Err(invalid("cylindrical region is not an open patch"));
    }
    let Some(axis) = cyl.axis.normalized() else {
        return Err(invalid("cylindrical region is not an open patch"));
    };
    let u = axis.any_perp();
    let v = axis.cross(u);
    let mut angles = Vec::new();
    let mut t_lo = f64::INFINITY;
    let mut t_hi = f64::NEG_INFINITY;
    let mut edge_loops = Vec::new();
    for lp in &loops {
        let mut pts = Vec::with_capacity(lp.len());
        for &i in lp {
            let Some(p) = positions.get(i as usize).copied() else {
                return Err(invalid("missing vertex"));
            };
            let d = p - cyl.origin;
            let t = d.dot(axis);
            t_lo = t_lo.min(t);
            t_hi = t_hi.max(t);
            let radial = d - axis * t;
            if radial.len() >= cyl.radius * 0.5 {
                angles.push(radial.dot(v).atan2(radial.dot(u)));
            }
            pts.push(p);
        }
        edge_loops.push(polyline_to_lines(&pts, 0.0)?);
    }
    let Some((gap_at, gap, span)) = angular_opening(&angles) else {
        return Err(invalid("cylindrical region is not an open patch"));
    };
    if gap < 0.45 || span < 0.4 || !(t_hi - t_lo).is_finite() || t_hi - t_lo < 1e-4 {
        return Err(invalid("cylindrical region is not an open patch"));
    }
    if !boundaries_disjoint(&edge_loops) {
        return Err(invalid("wire is not simple"));
    }
    for edges in &edge_loops {
        if !loop_is_simple(edges) {
            return Err(invalid("wire is not simple"));
        }
    }
    let start = gap_at + gap;
    let sweep = (span + 1e-3).min(TAU - 1e-3);
    let dir = u * start.cos() + v * start.sin();
    let a = cyl.origin + axis * t_lo + dir * cyl.radius;
    let b = cyl.origin + axis * t_hi + dir * cyl.radius;
    let mid_ang = start + span * 0.5;
    let mid_dir = u * mid_ang.cos() + v * mid_ang.sin();
    let probe = cyl.origin + axis * ((t_lo + t_hi) * 0.5) + mid_dir * cyl.radius;
    let outward = if material_away_from_axis(mesh, cluster, cyl) { mid_dir } else { -mid_dir };
    Ok(FaceSpec {
        loops: edge_loops,
        surface: SurfSpec::Revolved { a, b, axis_origin: cyl.origin, axis, angle: sweep },
        probe: Some((probe, outward)),
    })
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

fn cone_face(mesh: &TriMesh, cone: &PendingCone, circles: &mut HashMap<CircleKey, Vec<EdgeSpec>>) -> Result<Vec<FaceSpec>> {
    let bot = register_circle(circles, cone.axis, cone.apex + cone.axis * cone.t0, cone.r0, 0.3)?;
    let top = register_circle(circles, cone.axis, cone.apex + cone.axis * cone.t1, cone.r1, 0.3)?;
    let cyl = CylFit { axis: cone.axis, origin: cone.apex, radius: cone.r0.max(cone.r1) };
    cone_walls(&bot, &top, cone.apex, cone.axis, material_away_from_axis(mesh, &cone.cluster, &cyl))
}

/// Wall faces of a frustum. Each wall revolves the slanted generator between one arc of the
/// small circle and the matching arc of the large circle.
fn cone_walls(bot: &[EdgeSpec], top: &[EdgeSpec], apex: Vec3, axis: Vec3, away_from_axis: bool) -> Result<Vec<FaceSpec>> {
    if bot.len() != top.len() || bot.len() < 2 {
        return Err(invalid("cone circle must have at least two arcs"));
    }
    let axis = axis.normalized().ok_or_else(|| invalid("cone axis"))?;
    let angle = TAU / bot.len() as f64;
    let mut walls = Vec::with_capacity(bot.len());
    for (arc, top_arc) in bot.iter().zip(top) {
        let EdgeSpec::Arc { a, b, mid } = arc else {
            return Err(invalid("cone circle arcs"));
        };
        let EdgeSpec::Arc { a: ta, b: tb, mid: tm } = top_arc else {
            return Err(invalid("cone circle arcs"));
        };
        let m = (*mid + *tm) * 0.5;
        let radial = m - apex - axis * (m - apex).dot(axis);
        let away = radial.normalized().unwrap_or(Vec3::X);
        let outward = if away_from_axis { away } else { -away };
        let mut boundary = vec![
            EdgeSpec::Arc { a: *a, b: *b, mid: *mid },
            EdgeSpec::Line { a: *b, b: *tb },
            EdgeSpec::Arc { a: *tb, b: *ta, mid: *tm },
            EdgeSpec::Line { a: *ta, b: *a },
        ];
        if !away_from_axis {
            reverse_loop(&mut boundary);
        }
        walls.push(FaceSpec {
            loops: vec![boundary],
            surface: SurfSpec::Revolved { a: *a, b: *ta, axis_origin: apex, axis, angle },
            probe: Some((m, outward)),
        });
    }
    Ok(walls)
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
    let axis = axis.normalized()?;
    if !(radius.is_finite() && radius > 0.) || !(2..=64).contains(&n_arcs) {
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
    let x = x.normalized()?;
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

fn reverse_loop(edges: &mut [EdgeSpec]) {
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
    fn a_box_mesh_labels_its_faces_as_planes() {
        let source = crate::box_solid(Vec3::ZERO, Vec3::new(10., 20., 30.)).unwrap();
        let mesh = mesh_of(&source);
        let groups = label_mesh(&mesh, &crate::mesh_convert::ConvertOptions::from_tolerance(0.05)).unwrap();
        assert_eq!(groups.groups.len(), groups.triangles.len());
        assert!(groups.groups.iter().any(|g| *g == FaceGroup::Plane));
        assert!(groups.groups.iter().all(|g| *g == FaceGroup::Plane));
    }

    /// A closed mesh the folder converter sews, except for edges that collapse under
    /// the corner weld. Absent on a machine without the file, so CI skips it.
    #[test]
    fn probe_holder_stl_converts_to_a_solid() {
        let path = r"C:\Users\aidenleefaulconer\Downloads\tst\probe-holder.stl";
        let Ok(bytes) = std::fs::read(path) else { return };
        recognize_stats::reset();
        let mesh = stl_body(&bytes);
        let solid = crate::convert_mesh(&mesh, 0.02).unwrap_or_else(|e| panic!("{e}"));
        let stats = recognize_stats::line();
        assert!(!solid.is_mesh());
        let measured = crate::measure(&solid).unwrap();
        let volume = measured.volume;
        eprintln!("probe-holder faces {:?} {stats}", measured.merged.face_types);
        assert!((volume - 2953.874).abs() < 150.0, "volume {volume}, faces {:?}", measured.merged.face_types);
    }

    #[test]
    fn analytic_frustum_recovers_a_cone() {
        let (n, r0, r1, h) = (48usize, 10.0_f64, 4.0, 12.0);
        let mut positions = vec![Vec3::new(0., 0., 0.), Vec3::new(0., 0., h)];
        for i in 0..n {
            let a = TAU * i as f64 / n as f64;
            positions.push(Vec3::new(r0 * a.cos(), r0 * a.sin(), 0.));
            positions.push(Vec3::new(r1 * a.cos(), r1 * a.sin(), h));
        }
        let mut triangles = Vec::new();
        for i in 0..n {
            let b0 = (2 + i * 2) as u32;
            let t0 = b0 + 1;
            let b1 = (2 + ((i + 1) % n) * 2) as u32;
            let t1 = b1 + 1;
            triangles.push([b0, b1, t1]);
            triangles.push([b0, t1, t0]);
            triangles.push([0, b1, b0]);
            triangles.push([1, t0, t1]);
        }
        let mesh = crate::mesh_body(&positions, &triangles).unwrap();
        let solid = crate::convert_mesh(&mesh, 0.05).unwrap_or_else(|e| panic!("{e}"));
        assert!(!solid.is_mesh());
        let measured = crate::measure(&solid).unwrap();
        let cones = measured.merged.face_types.get("cone").copied().unwrap_or(0);
        assert!(cones >= 1, "expected a cone, got {:?}", measured.merged.face_types);
        let expected = std::f64::consts::PI * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
        assert!((measured.volume - expected).abs() / expected < 0.05, "{} vs {expected}", measured.volume);
    }

    fn stl_body(bytes: &[u8]) -> Body {
        assert!(bytes.len() >= 84, "not a binary STL");
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        let mut positions = Vec::with_capacity(count.saturating_mul(3));
        let mut triangles = Vec::with_capacity(count);
        let mut at = 84usize;
        for _ in 0..count {
            at += 12;
            let base = u32::try_from(positions.len()).unwrap();
            for _ in 0..3 {
                let x = f32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()) as f64;
                let y = f32::from_le_bytes(bytes[at + 4..at + 8].try_into().unwrap()) as f64;
                let z = f32::from_le_bytes(bytes[at + 8..at + 12].try_into().unwrap()) as f64;
                positions.push(Vec3::new(x, y, z));
                at += 12;
            }
            at += 2;
            triangles.push([base, base + 1, base + 2]);
        }
        crate::mesh_body(&positions, &triangles).unwrap()
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

    /// A half-round is not a tube. Confirm must still sew it as one cylinder, the way the
    /// convert preview paints it, rather than one face per triangle.
    #[test]
    fn analytic_half_cylinder_is_one_cylindrical_face() {
        let (n, r, h) = (24usize, 10.0_f64, 12.0);
        let mut positions = Vec::new();
        for i in 0..=n {
            let a = PI * i as f64 / n as f64;
            positions.push(Vec3::new(r * a.cos(), r * a.sin(), 0.));
        }
        for i in 0..=n {
            let a = PI * i as f64 / n as f64;
            positions.push(Vec3::new(r * a.cos(), r * a.sin(), h));
        }
        let b = |i: usize| i as u32;
        let t = |i: usize| (n + 1 + i) as u32;
        let mut triangles = Vec::new();
        for i in 0..n {
            let (b0, b1, t0, t1) = (b(i), b(i + 1), t(i), t(i + 1));
            triangles.push([b0, b1, t1]);
            triangles.push([b0, t1, t0]);
        }
        let bc = positions.len() as u32;
        positions.push(Vec3::ZERO);
        let tc = positions.len() as u32;
        positions.push(Vec3::new(0., 0., h));
        for i in 0..n {
            triangles.push([bc, b(i + 1), b(i)]);
            triangles.push([tc, t(i), t(i + 1)]);
        }
        // The cut is the plane y = 0, through the two centres. Outward is -Y, and each
        // edge is the opposite of the cap or wall edge it meets.
        triangles.push([b(0), t(0), tc]);
        triangles.push([b(0), tc, bc]);
        triangles.push([bc, tc, t(n)]);
        triangles.push([bc, t(n), b(n)]);
        let mesh_body = crate::mesh_body(&positions, &triangles).expect("half mesh");
        let expected = 0.5 * PI * r * r * h;
        let converted = convert_analytic_mesh(&mesh_body, 0.05, 15f64.to_radians()).expect("analytic half cylinder");
        let measured = crate::measure(&converted).expect("measure");
        let cyl = measured.merged.face_types.get("cylinder").copied().unwrap_or(0);
        assert!(cyl >= 1, "expected a cylinder, got {:?}", measured.merged.face_types);
        assert!((measured.volume - expected).abs() / expected < 0.08, "{} vs {expected} {:?}", measured.volume, measured.merged.face_types);
        let faces: usize = measured.merged.face_types.values().sum();
        assert!(faces < 40, "still a triangle mesh: {:?}", measured.merged.face_types);
    }

    /// The feeder the convert dialog paints as planes and cylinders. Absent on a machine
    /// without the file, so CI skips it. The committed solid must not be one face per triangle.
    #[test]
    fn ball_feeder_import_is_not_the_stl() {
        let path = r"C:\Users\aidenleefaulconer\Downloads\ball-feeder-to-rod - Copy.stl";
        let Ok(bytes) = std::fs::read(path) else { return };
        let mesh = stl_body(&bytes);
        let triangles = mesh.mesh.as_ref().map(|m| m.triangles.len()).unwrap_or(0);
        let solid = crate::convert_mesh(&mesh, 0.05).unwrap_or_else(|e| panic!("{e}"));
        assert!(!solid.is_mesh());
        let measured = crate::measure(&solid).expect("measure");
        let faces: usize = measured.merged.face_types.values().sum();
        eprintln!("ball-feeder tris={triangles} faces={faces} {:?}", measured.merged.face_types);
        assert!(faces * 4 < triangles.max(1), "still one face per triangle: {faces} from {triangles} {:?}", measured.merged.face_types);
        assert!(measured.merged.face_types.get("cylinder").copied().unwrap_or(0) >= 1, "{:?}", measured.merged.face_types);
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
