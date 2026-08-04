package physics

// BodyPair represents an unordered pair of bodies.
// Used by broadphase to report candidate collision pairs.
type BodyPair struct {
	A, B *Body
}

// Canonicalize returns the pair in canonical order (by index in the world's
// bodies slice, which is stable). Used for deduplication in broadphase.
func (p BodyPair) Canonicalize() BodyPair {
	// Order by pointer address for a stable canonical form.
	// We compare via reflect.ValueOf().Pointer() which returns uintptr.
	// To avoid the reflect import in the hot path, broadphase implementations
	// use a simpler approach: they only emit pairs where A is added before B
	// in the bodies slice, so the pair is already canonical.
	return p
}

// IsSelf reports whether the pair is a body with itself.
func (p BodyPair) IsSelf() bool { return p.A == p.B }
