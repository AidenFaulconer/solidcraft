//! A mesh import can store `solid`: the recognition settings and an origin used to rebuild
//! the embedded meshes as solids in the design. An import without the field is still mesh
//! bodies. A design that sets `solid` is a newer format, so the last release asks for an
//! update instead of dropping the solids back to meshes.
