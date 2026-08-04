package physics

// CollisionInfo is passed to OnCollision event listeners. Matches
// QBody::CollisionInfo in qbody.h:146-156.
type CollisionInfo struct {
	// Position is the world-space contact position.
	Position Vec2

	// Body is the other body involved in the collision.
	Body *Body

	// Normal is the collision normal.
	Normal Vec2

	// Penetration is the overlap depth.
	Penetration float32
}
