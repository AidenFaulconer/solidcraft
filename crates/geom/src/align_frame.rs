//! Alignment frames: a point and a right-handed triad, placed from what the cursor is nearest
//! on screen.
//!
//! On a face the point is the face centre or a corner, whichever lies closer to the cursor after
//! perspective, and the primary axis is the face normal toward the camera. The secondary axis
//! follows the cursor around the edge that looks nearest: the view ray's closest approach to
//! that edge, flattened into the face. On an edge the point snaps to an end or the middle, and
//! the primary axis itself swings around the edge toward the cursor. A circle keeps its centre
//! and points an axis at the cursor in the circle's plane.

use crate::{Vec2, Vec3};

const MAX_POINTS: usize = 256;
const MAX_SEGMENTS: usize = 2048;

/// Where the cursor is, in the camera's own projection, so nearer-on-screen follows perspective.
#[derive(Clone, Copy, Debug)]
pub struct Screen {
    pub eye: Vec3,
    /// Unit direction from the eye into the scene.
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    /// Cursor in the same space as [`Screen::project`].
    pub cursor: Vec2,
    /// Perspective divide. Orthographic projection ignores depth.
    pub perspective: bool,
}

impl Screen {
    /// The cursor's ray: origin and unit direction.
    pub fn ray(&self) -> Option<(Vec3, Vec3)> {
        if !self.cursor.is_finite() || !self.forward.is_finite() {
            return None;
        }
        if self.perspective {
            (self.forward + self.right * self.cursor.x + self.up * self.cursor.y).normalized().map(|d| (self.eye, d))
        } else {
            Some((self.eye + self.right * self.cursor.x + self.up * self.cursor.y, self.forward))
        }
    }

    /// Cursor-space position of a world point. `None` behind the camera or not finite.
    pub fn project(&self, p: Vec3) -> Option<Vec2> {
        if !p.is_finite() {
            return None;
        }
        let d = p - self.eye;
        let z = d.dot(self.forward);
        if !z.is_finite() || z < 1e-8 {
            return None;
        }
        let q = if self.perspective { Vec2::new(d.dot(self.right) / z, d.dot(self.up) / z) } else { Vec2::new(d.dot(self.right), d.dot(self.up)) };
        q.is_finite().then_some(q)
    }
}

/// Right-handed frame. `y` is `z` × `x`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AlignFrame {
    pub origin: Vec3,
    pub x: Vec3,
    pub z: Vec3,
}

/// A frame, and the snap points the cursor was choosing among (so the view can show them).
#[derive(Clone, Debug)]
pub struct Inferred {
    pub frame: AlignFrame,
    pub marks: Vec<Vec3>,
}

/// Rotation about `axis` through the world origin, then `translate`. Matches a rigid move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    pub translate: Vec3,
    pub axis: Vec3,
    pub angle: f64,
}

impl AlignFrame {
    pub fn y(self) -> Vec3 {
        self.z.cross(self.x)
    }

    /// Orthonormalise. `None` when the axes are parallel, zero or not finite.
    pub fn orthonormal(origin: Vec3, x: Vec3, z: Vec3) -> Option<Self> {
        if !origin.is_finite() {
            return None;
        }
        let z = z.normalized()?;
        let x = (x - z * x.dot(z)).normalized()?;
        Some(AlignFrame { origin, x, z })
    }

    /// Flip the primary axis, spin about it, then move the origin along the resulting axes.
    /// The offset is in the frame the triad shows, so each component follows one drawn axis.
    pub fn adjusted(self, offset: Vec3, spin: f64, flip: bool) -> Option<Self> {
        if !offset.is_finite() || !spin.is_finite() {
            return None;
        }
        let z = if flip { -self.z } else { self.z };
        let (s, c) = spin.sin_cos();
        let x = self.x * c + z.cross(self.x) * s;
        let frame = Self::orthonormal(self.origin, x, z)?;
        let moved = frame.origin + frame.x * offset.x + frame.y() * offset.y + frame.z * offset.z;
        moved.is_finite().then_some(AlignFrame { origin: moved, x: frame.x, z: frame.z })
    }

    /// The rigid move that lays this frame onto `to` (same local coordinates).
    pub fn map_onto(self, to: Self) -> Option<Placement> {
        let m = map_matrix(self, to);
        if m.iter().flatten().any(|v| !v.is_finite()) {
            return None;
        }
        let (axis, angle) = axis_angle(&m);
        let translate = to.origin - apply(&m, self.origin);
        (translate.is_finite() && angle.is_finite()).then_some(Placement { translate, axis, angle })
    }
}

impl Placement {
    pub fn apply(self, p: Vec3) -> Vec3 {
        apply(&rotation(self.axis, self.angle), p) + self.translate
    }
}

/// Circle through a polyline, if the points lie on one: centre and unit normal.
pub fn circle_through(pts: &[Vec3]) -> Option<(Vec3, Vec3)> {
    let n = pts.len();
    if n < 4 {
        return None;
    }
    let (a, b, c) = (*pts.first()?, *pts.get(n / 3)?, *pts.get(2 * n / 3)?);
    let (ab, ac) = (b - a, c - a);
    let nn = ab.cross(ac);
    let l2 = nn.len2();
    if l2 < 1e-18 {
        return None;
    }
    let center = a + (nn.cross(ab) * ac.len2() + ac.cross(nn) * ab.len2()) / (2.0 * l2);
    let r = center.dist(a);
    if !r.is_finite() || r > 1e6 {
        return None;
    }
    let tol = (r * 2e-3).max(1e-6);
    let normal = nn.normalized()?;
    pts.iter().all(|p| (p.dist(center) - r).abs() < tol && (*p - center).dot(normal).abs() < tol).then_some((center, normal))
}

/// Face (or a single vertex of one): `points` are the centre, corners and circle centres.
pub fn on_face(points: &[Vec3], segments: &[(Vec3, Vec3)], normal: Vec3, screen: &Screen) -> Option<Inferred> {
    let hint = *points.first()?;
    let origin = nearest(screen, points, hint)?;
    let frame = orient_on_face(origin, normal, segments, screen)?;
    Some(Inferred { frame, marks: points.iter().copied().take(64).filter(|p| p.is_finite()).collect() })
}

/// Edge polyline. A circle stays at its centre; a straight edge snaps to an end or the middle.
pub fn on_edge(polyline: &[Vec3], screen: &Screen) -> Option<Inferred> {
    if let Some((center, axis)) = circle_through(polyline) {
        return circle_frame(center, axis, screen);
    }
    let (a, b) = (*polyline.first()?, *polyline.last()?);
    let dir = (b - a).normalized()?;
    let span = a.dist(b).max(1.0);
    let straight = polyline.iter().all(|p| (*p - a).cross(dir).len() < 1e-4 * span);
    let mid = (a + b) * 0.5;
    let candidates: [Vec3; 3] = [a, b, mid];
    let used = if straight { 3 } else { 2 };
    let origin = nearest(screen, &candidates[..used], mid)?;
    let frame = orient_around_edge(origin, a, b, screen)?;
    Some(Inferred { frame, marks: candidates[..used].to_vec() })
}

fn circle_frame(center: Vec3, axis: Vec3, screen: &Screen) -> Option<Inferred> {
    let mut z = axis.normalized()?;
    if z.dot(screen.eye - center) < 0.0 {
        z = -z;
    }
    let radial = screen.ray().and_then(|(ro, rd)| plane_hit(ro, rd, center, z)).map(|hit| hit - center).unwrap_or(screen.right);
    let x = (radial - z * radial.dot(z)).normalized().unwrap_or_else(|| z.any_perp());
    let frame = AlignFrame::orthonormal(center, x, z)?;
    Some(Inferred { frame, marks: vec![center] })
}

fn orient_on_face(origin: Vec3, normal: Vec3, segments: &[(Vec3, Vec3)], screen: &Screen) -> Option<AlignFrame> {
    let mut z = normal.normalized()?;
    if z.dot(screen.eye - origin) < 0.0 {
        z = -z;
    }
    let (a, b) = nearest_segment(screen, segments).unwrap_or((origin, origin + z.any_perp()));
    let edge = (b - a).normalized().unwrap_or_else(|| z.any_perp());
    let x = in_plane_x(z, a, b, edge, screen);
    AlignFrame::orthonormal(origin, x, z)
}

/// Secondary axis, square to the direction from the nearest edge toward the cursor.
fn in_plane_x(z: Vec3, a: Vec3, b: Vec3, edge: Vec3, screen: &Screen) -> Vec3 {
    let Some((ro, rd)) = screen.ray() else {
        return reject(edge, z).normalized().unwrap_or_else(|| z.any_perp());
    };
    let (on_edge, on_ray) = closest_approach(ro, rd, a, b);
    let radial = reject(on_ray - on_edge, z);
    z.cross(radial).normalized().or_else(|| reject(edge, z).normalized()).unwrap_or_else(|| z.any_perp())
}

fn orient_around_edge(origin: Vec3, a: Vec3, b: Vec3, screen: &Screen) -> Option<AlignFrame> {
    let edge = (b - a).normalized()?;
    let (ro, rd) = screen.ray()?;
    let (on_edge, on_ray) = closest_approach(ro, rd, a, b);
    let radial = reject(on_ray - on_edge, edge);
    let z = radial.normalized().or_else(|| reject(screen.eye - origin, edge).normalized())?;
    let x = reject(edge, z).normalized()?;
    AlignFrame::orthonormal(origin, x, z)
}

fn nearest(screen: &Screen, points: &[Vec3], hint: Vec3) -> Option<Vec3> {
    let mut best: Option<(f64, Vec3)> = None;
    for p in points.iter().copied().take(MAX_POINTS) {
        let Some(s) = screen.project(p) else { continue };
        let d = (s - screen.cursor).len2();
        if best.as_ref().is_none_or(|(bd, _)| d < *bd) {
            best = Some((d, p));
        }
    }
    if let Some((_, p)) = best {
        return Some(p);
    }
    points.iter().copied().take(MAX_POINTS).filter(|p| p.is_finite()).min_by(|a, b| a.dist(hint).total_cmp(&b.dist(hint)))
}

fn nearest_segment(screen: &Screen, segments: &[(Vec3, Vec3)]) -> Option<(Vec3, Vec3)> {
    let mut best: Option<(f64, (Vec3, Vec3))> = None;
    for seg in segments.iter().copied().take(MAX_SEGMENTS) {
        let Some(d) = screen_dist2(screen, seg.0, seg.1) else { continue };
        if best.as_ref().is_none_or(|(bd, _)| d < *bd) {
            best = Some((d, seg));
        }
    }
    best.map(|(_, seg)| seg)
}

fn screen_dist2(screen: &Screen, a: Vec3, b: Vec3) -> Option<f64> {
    let (pa, pb) = (screen.project(a)?, screen.project(b)?);
    let ab = pb - pa;
    let l2 = ab.len2();
    let t = if l2 > 1e-18 { ((screen.cursor - pa).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    Some((screen.cursor - (pa + ab * t)).len2())
}

/// Closest points on the ray `ro + s * rd` and the segment `a`–`b`. `rd` need not be a unit.
fn closest_approach(ro: Vec3, rd: Vec3, a: Vec3, b: Vec3) -> (Vec3, Vec3) {
    let u = b - a;
    let w = a - ro;
    let aa = rd.dot(rd);
    let bb = rd.dot(u);
    let cc = u.dot(u);
    let dd = rd.dot(w);
    let ee = u.dot(w);
    let den = aa * cc - bb * bb;
    let t = if den.abs() < 1e-12 || cc < 1e-18 { 0.0 } else { ((bb * dd - aa * ee) / den).clamp(0.0, 1.0) };
    let on_edge = a + u * t;
    let s = if aa < 1e-18 { 0.0 } else { (rd.dot(on_edge - ro) / aa).max(0.0) };
    (on_edge, ro + rd * s)
}

fn plane_hit(ro: Vec3, rd: Vec3, origin: Vec3, normal: Vec3) -> Option<Vec3> {
    let den = rd.dot(normal);
    if den.abs() < 1e-9 {
        return None;
    }
    let t = (origin - ro).dot(normal) / den;
    (t >= 0.0).then_some(ro + rd * t).filter(|p| p.is_finite())
}

fn reject(v: Vec3, n: Vec3) -> Vec3 {
    v - n * v.dot(n)
}

fn map_matrix(from: AlignFrame, to: AlignFrame) -> [[f64; 3]; 3] {
    let from_axes = [from.x, from.y(), from.z];
    let to_axes = [to.x, to.y(), to.z];
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, value) in row.iter_mut().enumerate() {
            *value = (0..3).map(|k| component(to_axes[k], i) * component(from_axes[k], j)).sum();
        }
    }
    m
}

fn component(v: Vec3, i: usize) -> f64 {
    match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

fn apply(m: &[[f64; 3]; 3], p: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z,
        m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z,
        m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z,
    )
}

/// Column-vector rotation, the same sense as a move about `axis` (right-handed).
fn rotation(axis: Vec3, angle: f64) -> [[f64; 3]; 3] {
    let Some(axis) = axis.normalized() else {
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    };
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    let (x, y, z) = (axis.x, axis.y, axis.z);
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

fn axis_angle(m: &[[f64; 3]; 3]) -> (Vec3, f64) {
    let tr = m[0][0] + m[1][1] + m[2][2];
    let angle = ((tr - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
    if angle < 1e-12 {
        return (Vec3::Z, 0.0);
    }
    let v = Vec3::new(m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]);
    if let Some(a) = v.normalized().filter(|_| v.len() > 1e-6) {
        return (a, angle);
    }
    let d = [((m[0][0] + 1.0) / 2.0).max(0.0).sqrt(), ((m[1][1] + 1.0) / 2.0).max(0.0).sqrt(), ((m[2][2] + 1.0) / 2.0).max(0.0).sqrt()];
    let (x, mut y, mut z) = (d[0], d[1], d[2]);
    if m[0][1] + m[1][0] < 0.0 && x > 1e-6 {
        y = -y;
    }
    if m[0][2] + m[2][0] < 0.0 && x > 1e-6 {
        z = -z;
    }
    if x <= 1e-6 && m[1][2] + m[2][1] < 0.0 {
        z = -z;
    }
    (Vec3::new(x, y, z).normalized().unwrap_or(Vec3::Z), angle)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn above() -> Screen {
        Screen {
            eye: Vec3::new(0.0, 0.0, 100.0),
            forward: Vec3::new(0.0, 0.0, -1.0),
            right: Vec3::X,
            up: Vec3::Y,
            cursor: Vec2::ZERO,
            perspective: false,
        }
    }

    fn square() -> (Vec<Vec3>, Vec<(Vec3, Vec3)>) {
        let corners = [Vec3::new(10.0, 10.0, 0.0), Vec3::new(10.0, -10.0, 0.0), Vec3::new(-10.0, 10.0, 0.0), Vec3::new(-10.0, -10.0, 0.0)];
        let points = vec![Vec3::ZERO, corners[0], corners[1], corners[2], corners[3]];
        let segments = vec![
            (Vec3::new(10.0, -10.0, 0.0), Vec3::new(10.0, 10.0, 0.0)),
            (Vec3::new(-10.0, 10.0, 0.0), Vec3::new(10.0, 10.0, 0.0)),
            (Vec3::new(-10.0, -10.0, 0.0), Vec3::new(10.0, -10.0, 0.0)),
            (Vec3::new(-10.0, -10.0, 0.0), Vec3::new(-10.0, 10.0, 0.0)),
        ];
        (points, segments)
    }

    #[test]
    fn a_face_uses_its_centre_until_the_cursor_is_nearer_a_corner() {
        let (points, segments) = square();
        let mid = on_face(&points, &segments, Vec3::Z, &above()).unwrap();
        assert!(mid.frame.origin.len() < 1e-6, "{:?}", mid.frame.origin);
        assert!(mid.frame.z.dot(Vec3::Z) > 0.9);

        let mut at_corner = above();
        at_corner.cursor = Vec2::new(10.0, 10.0);
        let corner = on_face(&points, &segments, Vec3::Z, &at_corner).unwrap();
        assert!(corner.frame.origin.dist(Vec3::new(10.0, 10.0, 0.0)) < 1e-6, "{:?}", corner.frame.origin);
    }

    #[test]
    fn moving_the_cursor_across_an_edge_reverses_the_secondary_axis() {
        let (points, segments) = square();
        let mut inside = above();
        inside.cursor = Vec2::new(9.0, 2.0);
        let mut outside = above();
        outside.cursor = Vec2::new(11.0, 2.0);
        let a = on_face(&points, &segments, Vec3::Z, &inside).unwrap();
        let b = on_face(&points, &segments, Vec3::Z, &outside).unwrap();
        assert!(a.frame.origin.dist(b.frame.origin) < 1e-6, "same corner, {:?} vs {:?}", a.frame.origin, b.frame.origin);
        assert!(a.frame.x.dot(b.frame.x) < -0.9, "across the edge {:?} vs {:?}", a.frame.x, b.frame.x);
    }

    #[test]
    fn perspective_prefers_the_point_that_looks_nearest() {
        let screen = Screen { eye: Vec3::ZERO, forward: Vec3::Y, right: Vec3::X, up: Vec3::Z, cursor: Vec2::new(0.15, 0.0), perspective: true };
        // The far point sits on the view axis. In 3D the other point is closer to the eye.
        let far = Vec3::new(0.0, 10.0, 0.0);
        let near = Vec3::new(2.0, 2.0, 0.0);
        let points = [far, near];
        let got = nearest(&screen, &points, near).unwrap();
        assert!(got.dist(far) < 1e-9, "screen distance wins, got {got:?}");
        let mut beside = screen;
        beside.cursor = Vec2::new(0.9, 0.0);
        let got = nearest(&beside, &points, far).unwrap();
        assert!(got.dist(near) < 1e-9, "got {got:?}");
    }

    #[test]
    fn an_edge_swings_its_primary_axis_around_itself_toward_the_cursor() {
        let edge = [Vec3::ZERO, Vec3::new(0.0, 0.0, 10.0)];
        let mut screen = Screen {
            eye: Vec3::new(10.0, 0.0, 5.0),
            forward: Vec3::new(-1.0, 0.0, 0.0),
            right: Vec3::Y,
            up: Vec3::Z,
            cursor: Vec2::new(2.0, 0.0),
            perspective: false,
        };
        let pos = on_edge(&edge, &screen).unwrap();
        assert!(pos.frame.origin.dist(Vec3::new(0.0, 0.0, 5.0)) < 1e-6, "{:?}", pos.frame.origin);
        assert!(pos.frame.z.dot(Vec3::Y) > 0.9, "{:?}", pos.frame.z);
        screen.cursor = Vec2::new(-2.0, 0.0);
        let neg = on_edge(&edge, &screen).unwrap();
        assert!(neg.frame.z.dot(Vec3::Y) < -0.9, "{:?}", neg.frame.z);
    }

    #[test]
    fn a_frame_laid_on_another_keeps_local_coordinates_and_a_turn_does_not_move_a_zero_offset() {
        let from = AlignFrame::orthonormal(Vec3::new(1.0, 2.0, 3.0), Vec3::X, Vec3::Z).unwrap();
        let to = AlignFrame::orthonormal(Vec3::new(4.0, 0.0, 1.0), Vec3::Y, Vec3::Z).unwrap();
        let place = from.map_onto(to).unwrap();
        let local = Vec3::new(2.0, -1.0, 0.5);
        let p = from.origin + from.x * local.x + from.y() * local.y + from.z * local.z;
        let q = to.origin + to.x * local.x + to.y() * local.y + to.z * local.z;
        assert!(place.apply(p).dist(q) < 1e-9, "{:?} vs {q:?}", place.apply(p));

        let spun = from.adjusted(Vec3::ZERO, std::f64::consts::FRAC_PI_2, false).unwrap();
        assert!(spun.origin.dist(from.origin) < 1e-12);
        assert!(spun.x.dot(Vec3::Y) > 0.999, "{:?}", spun.x);
        let shifted = from.adjusted(Vec3::new(5.0, 0.0, 0.0), 0.0, false).unwrap();
        assert!(shifted.origin.dist(from.origin + Vec3::X * 5.0) < 1e-9);
        let flipped = from.adjusted(Vec3::ZERO, 0.0, true).unwrap();
        assert!(flipped.z.dot(Vec3::Z) < -0.999);
    }

    #[test]
    fn a_quarter_turn_about_z_matches_the_right_hand_rule() {
        let place = Placement { translate: Vec3::new(10.0, 10.0, 30.0), axis: Vec3::Z, angle: std::f64::consts::FRAC_PI_2 };
        let got = place.apply(Vec3::X);
        assert!(got.dist(Vec3::new(10.0, 11.0, 30.0)) < 1e-9, "{got:?}");
    }
}
