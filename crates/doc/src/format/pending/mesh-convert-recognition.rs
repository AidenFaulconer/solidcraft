//! A Convert Mesh feature can record which families to look for (`cylinders`, `cones`, `tori`,
//! `spheres`, `threads`, `freeform`) and `mesh`, the percent of triangles that start a fit
//! (1–100). Version designs without these fields fit every family and use the whole mesh, and
//! still save that way; a design that turns one off is a newer format, so the last release asks
//! for an update instead of ignoring the choice.
