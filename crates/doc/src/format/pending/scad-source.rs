//! An OpenSCAD import stores its script path, parameter overrides and curve settings on
//! the mesh solid (`solid.scad`). An older import without that field stays the solid it
//! already is; editing it reopens the script from the path recorded in `solid.files`.
//! A design that sets `solid.scad` is a newer format, so the last release asks for an update.
