//! A Convert Mesh feature can record `angle`, how far a triangle normal may turn while a planar
//! face grows. Version designs without the field use 15 degrees, and still save that way; a
//! design that sets another angle is a newer format, so the last release asks for an update
//! instead of ignoring it.
