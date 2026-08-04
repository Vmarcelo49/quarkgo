package physics

// GetRigidBody returns the *RigidBody that embeds the given *Body, or nil.
// This is the exported version of asRigidBody, for use by external packages
// (e.g., the examples' mouse drag handler).
func GetRigidBody(b *Body) *RigidBody {
	return asRigidBody(b)
}
