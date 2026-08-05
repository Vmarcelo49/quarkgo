// Package platformer provides a character controller for platformer games.
//
// QPlatformerBody extends RigidBody with behaviors such as gravity, walking
// on slopes, jumping (including multi-jump and wall jumps), and moving
// platform snapping. It provides helper methods for directional collision
// probes (GetFloor, GetCeiling, GetRightWall, GetLeftWall).
//
// Reference: QPlatformerBody in qplatformerbody.h, qplatformerbody.cpp
package platformer

import (
	"github.com/Vmarcelo49/quarkgo/physics"
	"github.com/chewxy/math32"
)

// JumpMode enumerates the jump state machine states.
type JumpMode int

const (
	JumpReleased JumpMode = iota
	JumpPressed
	JumpPressing
)

// CollisionTestInfo holds the result of a directional collision probe.
type CollisionTestInfo struct {
	Body        *physics.Body
	Position    physics.Vec2
	Penetration float32
	Normal      physics.Vec2
}

// HasBody reports whether the collision test found a body.
func (c CollisionTestInfo) HasBody() bool { return c.Body != nil }

// PlatformerBody is a character controller for platformer games.
// Embeds physics.RigidBody and overrides PostUpdate to apply character physics.
type PlatformerBody struct {
	physics.RigidBody

	// Floor state
	onFloor   bool
	onCeiling bool

	// Platform layer filtering (0 = use all collision layers)
	platformLayersBit int

	// Floor angle limit (radians)
	maxFloorAngle float32

	// Moving floor tracking
	movingFloorSnapOffset float32
	lastMovableFloor      *physics.Body

	// Gravity
	gravity           physics.Vec2
	gravityMultiplier float32

	// Directions derived from gravity
	upDirection    physics.Vec2
	rightDirection physics.Vec2

	// Walk
	walkSpeed            float32
	horizontalVelocity   physics.Vec2
	verticalVelocity     physics.Vec2
	isFalling            bool
	isRising             bool
	walkSide             int
	walkAccelerationRate float32
	walkDecelerationRate float32

	// Jump
	jumpMode                  JumpMode
	jumpForce                 float32
	maxJumpCount              int
	currentJumpCount          int
	jumpDurationFrameCount    int
	jumpFrameCountDown        int
	jumpGravityMultiplier     float32
	jumpFallGravityMultiplier float32
}

// New creates a PlatformerBody with default values.
// Matches QPlatformerBody::QPlatformerBody in qplatformerbody.cpp:37-45.
func New() *PlatformerBody {
	pb := &PlatformerBody{
		maxFloorAngle:             math32.Pi * 0.25,
		movingFloorSnapOffset:     10.0,
		gravity:                   physics.Vec2{X: 0, Y: 0.3},
		gravityMultiplier:         1.0,
		walkSpeed:                 3.0,
		walkAccelerationRate:      0.1,
		walkDecelerationRate:      0.1,
		jumpForce:                 5.0,
		maxJumpCount:              2,
		jumpDurationFrameCount:    30,
		jumpGravityMultiplier:     0.4,
		jumpFallGravityMultiplier: 1.0,
		upDirection:               physics.Vec2Up(),
		rightDirection:            physics.Vec2Right(),
	}

	// Configure the embedded RigidBody
	pb.RigidBody = *physics.NewRigidBody()
	pb.SetIntegratedVelocitiesEnabled(false)
	pb.SetKinematicCollisionsEnabled(true)
	pb.SetFixedRotationEnabled(true)
	pb.SetFriction(0.0)
	pb.SetStaticFriction(0.5)

	// Set up directions from default gravity
	pb.updateDirections()

	return pb
}

// updateDirections recomputes upDirection and rightDirection from gravity.
func (pb *PlatformerBody) updateDirections() {
	pb.upDirection = pb.gravity.Neg().Normalized()
	pb.rightDirection = pb.upDirection.Perpendicular().Neg()
}

// AsBody returns the *physics.Body for this platformer body.
func (pb *PlatformerBody) AsBody() *physics.Body { return &pb.RigidBody.Body }

// RegisterPostUpdate registers this platformer body's PostUpdate with the
// physics engine. MUST be called after adding the body to the world:
//
//	world.AddRigidBody(&pb.RigidBody)
//	pb.RegisterPostUpdate()
//
// This replaces C++ virtual method dispatch — Go has no virtual methods,
// so the World calls PostUpdate via a function registry.
func (pb *PlatformerBody) RegisterPostUpdate() {
	physics.RegisterPostUpdater(pb.AsBody(), pb.PostUpdate)
}

// --- Getters ---

func (pb *PlatformerBody) IsOnFloor() bool      { return pb.onFloor }
func (pb *PlatformerBody) IsOnCeiling() bool    { return pb.onCeiling }
func (pb *PlatformerBody) IsFalling() bool      { return pb.isFalling }
func (pb *PlatformerBody) IsRising() bool       { return pb.isRising }
func (pb *PlatformerBody) IsJumping() bool      { return pb.jumpMode != JumpReleased }
func (pb *PlatformerBody) IsJumpReleased() bool { return pb.jumpMode == JumpReleased }

func (pb *PlatformerBody) MovingFloorSnapOffset() float32             { return pb.movingFloorSnapOffset }
func (pb *PlatformerBody) FloorMaxAngle() float32                     { return pb.maxFloorAngle }
func (pb *PlatformerBody) FloorMaxAngleDegree() float32               { return pb.maxFloorAngle / (math32.Pi / 180) }
func (pb *PlatformerBody) Gravity() physics.Vec2                      { return pb.gravity }
func (pb *PlatformerBody) GravityMultiplier() float32                 { return pb.gravityMultiplier }
func (pb *PlatformerBody) WalkSpeed() float32                         { return pb.walkSpeed }
func (pb *PlatformerBody) WalkAccelerationRate() float32              { return pb.walkAccelerationRate }
func (pb *PlatformerBody) WalkDecelerationRate() float32              { return pb.walkDecelerationRate }
func (pb *PlatformerBody) ControllerHorizontalVelocity() physics.Vec2 { return pb.horizontalVelocity }
func (pb *PlatformerBody) ControllerVerticalVelocity() physics.Vec2   { return pb.verticalVelocity }
func (pb *PlatformerBody) JumpDurationFrameCount() int                { return pb.jumpDurationFrameCount }
func (pb *PlatformerBody) JumpGravityMultiplier() float32             { return pb.jumpGravityMultiplier }
func (pb *PlatformerBody) JumpFallGravityMultiplier() float32         { return pb.jumpFallGravityMultiplier }
func (pb *PlatformerBody) MaxJumpCount() int                          { return pb.maxJumpCount }
func (pb *PlatformerBody) SpecificPlatformLayers() int                { return pb.platformLayersBit }

// --- Setters ---

func (pb *PlatformerBody) SetMovingFloorSnapOffset(v float32) *PlatformerBody {
	pb.movingFloorSnapOffset = v
	return pb
}

func (pb *PlatformerBody) SetFloorMaxAngle(v float32) *PlatformerBody {
	pb.maxFloorAngle = v
	return pb
}

func (pb *PlatformerBody) SetFloorMaxAngleDegree(v float32) *PlatformerBody {
	return pb.SetFloorMaxAngle(v * (math32.Pi / 180))
}

func (pb *PlatformerBody) SetGravity(v physics.Vec2) *PlatformerBody {
	pb.gravity = v
	pb.updateDirections()
	return pb
}

func (pb *PlatformerBody) SetGravityMultiplier(v float32) *PlatformerBody {
	pb.gravityMultiplier = v
	return pb
}

func (pb *PlatformerBody) SetWalkSpeed(v float32) *PlatformerBody {
	pb.walkSpeed = v
	return pb
}

func (pb *PlatformerBody) SetWalkAccelerationRate(v float32) *PlatformerBody {
	pb.walkAccelerationRate = v
	return pb
}

func (pb *PlatformerBody) SetWalkDecelerationRate(v float32) *PlatformerBody {
	pb.walkDecelerationRate = v
	return pb
}

func (pb *PlatformerBody) SetControllerHorizontalVelocity(v physics.Vec2) *PlatformerBody {
	pb.horizontalVelocity = v
	return pb
}

func (pb *PlatformerBody) SetControllerVerticalVelocity(v physics.Vec2) *PlatformerBody {
	pb.verticalVelocity = v
	return pb
}

func (pb *PlatformerBody) SetJumpDurationFrameCount(v int) *PlatformerBody {
	pb.jumpDurationFrameCount = v
	return pb
}

func (pb *PlatformerBody) SetJumpGravityMultiplier(v float32) *PlatformerBody {
	pb.jumpGravityMultiplier = v
	return pb
}

func (pb *PlatformerBody) SetJumpFallGravityMultiplier(v float32) *PlatformerBody {
	pb.jumpFallGravityMultiplier = v
	return pb
}

func (pb *PlatformerBody) SetMaxJumpCount(v int) *PlatformerBody {
	pb.maxJumpCount = v
	return pb
}

func (pb *PlatformerBody) SetSpecificPlatformLayers(v int) *PlatformerBody {
	pb.platformLayersBit = v
	return pb
}

// --- Actions ---

// Walk sets the walk direction (-1 = left, 1 = right, 0 = stop).
func (pb *PlatformerBody) Walk(side int) *PlatformerBody {
	pb.walkSide = side
	return pb
}

// Jump initiates a jump with the given force. If unconditional is true,
// the jump executes regardless of current state.
// Matches QPlatformerBody::Jump in qplatformerbody.cpp:349-376.
func (pb *PlatformerBody) Jump(force float32, unconditional bool) *PlatformerBody {
	if pb.jumpMode == JumpReleased {
		cond1 := unconditional
		cond2 := pb.onFloor && pb.jumpFrameCountDown > pb.jumpDurationFrameCount
		cond3 := !pb.onFloor && pb.currentJumpCount+1 < pb.maxJumpCount

		if cond1 || cond2 || cond3 {
			pb.jumpMode = JumpPressed
			pb.jumpForce = force
			pb.jumpFrameCountDown = 0
		}

		if cond2 || cond3 {
			pb.currentJumpCount++
		}
	} else if pb.jumpMode == JumpPressed {
		pb.jumpMode = JumpPressing
	}

	return pb
}

// ReleaseJump ends the jump press.
func (pb *PlatformerBody) ReleaseJump() *PlatformerBody {
	pb.jumpMode = JumpReleased
	return pb
}

// ApplyForce applies a force, decomposed into horizontal and vertical components.
// Matches QPlatformerBody::ApplyForce in qplatformerbody.cpp:296-313.
func (pb *PlatformerBody) ApplyForce(force physics.Vec2) *PlatformerBody {
	if force == physics.Vec2Zero() {
		return pb
	}

	horizontalForce := pb.rightDirection.Mul(force.Dot(pb.rightDirection))
	verticalForce := pb.upDirection.Mul(force.Dot(pb.upDirection))

	pb.horizontalVelocity = pb.horizontalVelocity.Add(horizontalForce)
	pb.verticalVelocity = pb.verticalVelocity.Add(verticalForce)

	pb.SetPositionAndCollide(pb.Position().Add(force), true)
	pb.WakeUp()

	return pb
}

// --- Collision probes ---

// GetPlatformCollisions tests for collisions at a position and returns
// the nearest one. Matches QPlatformerBody::GetPlatformCollisions.
func (pb *PlatformerBody) GetPlatformCollisions(testPosition physics.Vec2, filterByMovingDirection bool) CollisionTestInfo {
	var result CollisionTestInfo
	world := pb.World()
	if world == nil {
		return result
	}

	tempPosition := pb.Position()
	pb.SetPosition(testPosition, false)
	testMovementDirection := testPosition.Sub(tempPosition).Normalized()
	manifolds := world.TestCollisionWithWorld(pb.AsBody())
	pb.SetPosition(tempPosition, false)

	minDistance := physics.MaxWorldSize
	for _, m := range manifolds {
		var collidedBody *physics.Body
		if m.BodyA() == pb.AsBody() {
			collidedBody = m.BodyB()
		} else {
			collidedBody = m.BodyA()
		}

		if collidedBody.BodyType() == physics.BodyTypeArea {
			continue
		}

		if pb.platformLayersBit != 0 {
			if (pb.platformLayersBit & collidedBody.LayersBit()) == 0 {
				continue
			}
		}

		for _, contact := range m.Contacts() {
			if filterByMovingDirection {
				collisionNormal := contact.Normal
				// Flip normal if the contact particle is NOT owned by this body
				if contact.Particle != nil && contact.Particle.OwnerMesh() != nil &&
					contact.Particle.OwnerMesh().OwnerBody() != pb.AsBody() {
					collisionNormal = collisionNormal.Neg()
				}
				if collisionNormal.Dot(testMovementDirection) >= 0 {
					continue
				}
			}

			distance := contact.Position.Dot(testMovementDirection)
			if distance < minDistance {
				result.Body = collidedBody
				result.Position = contact.Position
				// Normal pointing away from the collided surface toward this body
				if contact.Particle != nil && contact.Particle.OwnerMesh() != nil &&
					contact.Particle.OwnerMesh().OwnerBody() == pb.AsBody() {
					result.Normal = contact.Normal
				} else {
					result.Normal = contact.Normal.Neg()
				}
				result.Penetration = contact.Penetration
				minDistance = distance
			}
		}
	}

	return result
}

// GetRightWall probes for a wall to the right.
func (pb *PlatformerBody) GetRightWall(offset float32) CollisionTestInfo {
	offsetVec := pb.rightDirection.Mul(offset)
	test := pb.GetPlatformCollisions(pb.Position().Add(offsetVec), true)

	if test.HasBody() {
		normalAngle := physics.AngleBetweenTwoVectors(test.Normal, pb.upDirection)
		if math32.Abs(normalAngle) > pb.maxFloorAngle &&
			math32.Abs(normalAngle) < (math32.Pi-pb.maxFloorAngle) {
			return test
		}
	}
	return CollisionTestInfo{}
}

// GetLeftWall probes for a wall to the left.
func (pb *PlatformerBody) GetLeftWall(offset float32) CollisionTestInfo {
	return pb.GetRightWall(-offset)
}

// GetFloor probes for floor below.
func (pb *PlatformerBody) GetFloor(offset float32) CollisionTestInfo {
	offsetVec := pb.upDirection.Mul(offset)
	test := pb.GetPlatformCollisions(pb.Position().Add(offsetVec), true)

	if test.HasBody() {
		normalAngle := physics.AngleBetweenTwoVectors(test.Normal, pb.upDirection)
		if math32.Abs(normalAngle) <= pb.maxFloorAngle {
			return test
		}
	}
	return CollisionTestInfo{}
}

// GetCeiling probes for ceiling above.
func (pb *PlatformerBody) GetCeiling(offset float32) CollisionTestInfo {
	offsetVec := pb.upDirection.Neg().Mul(offset)
	test := pb.GetPlatformCollisions(pb.Position().Add(offsetVec), true)

	if test.HasBody() {
		normalAngle := physics.AngleBetweenTwoVectors(test.Normal, pb.upDirection)
		if math32.Abs(normalAngle) > (math32.Pi - pb.maxFloorAngle) {
			return test
		}
	}
	return CollisionTestInfo{}
}

// --- PostUpdate (the main character controller) ---

// PostUpdate applies character physics. Called by World.Update after all
// bodies have completed their Verlet integration.
// Matches QPlatformerBody::PostUpdate in qplatformerbody.cpp:402-648.
func (pb *PlatformerBody) PostUpdate() {
	dirCeiling := pb.upDirection
	dirFloor := pb.upDirection.Neg()

	gravityAmount := pb.gravity.Mul(pb.gravityMultiplier)
	if pb.IgnoreGravity() {
		gravityAmount = physics.Vec2Zero()
	}

	// Save current position before tests
	tempPosition := pb.Position()

	// --- Moving platform forces ---
	if pb.lastMovableFloor != nil {
		floorVelocity := pb.lastMovableFloor.Position().Sub(pb.lastMovableFloor.PreviousPosition())
		// Sync prevPosition to zero implicit velocity (matches qplatformerbody.cpp:426
		// which uses the default withPreviousPosition=true). Without this, the
		// platformer's per-frame movement leaks into velRef on the first solver
		// iteration and corrupts static-friction forces on contacted dynamic bodies.
		pb.AddPosition(floorVelocity, true)

		// Check if still on the moving floor
		tempPosition = pb.Position()
		pb.AddPosition(dirFloor.Mul(pb.movingFloorSnapOffset), false)
		world := pb.World()
		if world != nil {
			// Re-test collision with the moving floor
			manifolds := world.TestCollisionWithWorld(pb.AsBody())
			pb.SetPosition(tempPosition, false)

			stillFloor := false
			for _, m := range manifolds {
				var collidedBody *physics.Body
				if m.BodyA() == pb.AsBody() {
					collidedBody = m.BodyB()
				} else {
					collidedBody = m.BodyA()
				}
				if collidedBody != pb.lastMovableFloor {
					continue
				}
				for _, contact := range m.Contacts() {
					normal := contact.Normal
					if contact.Particle != nil && contact.Particle.OwnerMesh() != nil &&
						contact.Particle.OwnerMesh().OwnerBody() != pb.AsBody() {
						normal = normal.Neg()
					}
					floorAngle := physics.AngleBetweenTwoVectors(normal, pb.upDirection)
					if math32.Abs(floorAngle) < pb.maxFloorAngle {
						stillFloor = true
						break
					}
				}
				if stillFloor {
					break
				}
			}
			if !stillFloor {
				pb.lastMovableFloor = nil
			}
		} else {
			pb.SetPosition(tempPosition, false)
		}
	}

	// --- Jump velocities ---
	switch pb.jumpMode {
	case JumpPressed:
		pb.verticalVelocity = pb.upDirection.Mul(pb.jumpForce)
	case JumpPressing:
		if pb.verticalVelocity.Dot(pb.upDirection) > 0 {
			gravityAmount = gravityAmount.Mul(pb.jumpGravityMultiplier)
		}
	case JumpReleased:
		if pb.verticalVelocity.Dot(pb.upDirection) > 0 {
			gravityAmount = gravityAmount.Mul(pb.jumpFallGravityMultiplier)
		}
	}

	pb.jumpFrameCountDown++

	// --- Vertical velocities ---
	tempPosition = pb.Position()

	// Check floor
	pb.onFloor = false
	pb.AddPosition(dirFloor.Mul(1.0), false)
	world := pb.World()
	if world != nil {
		floorManifolds := world.TestCollisionWithWorld(pb.AsBody())
		pb.SetPosition(tempPosition, false)

		for _, m := range floorManifolds {
			var collidedBody *physics.Body
			if m.BodyA() == pb.AsBody() {
				collidedBody = m.BodyB()
			} else {
				collidedBody = m.BodyA()
			}
			if collidedBody.BodyType() == physics.BodyTypeArea {
				continue
			}
			if pb.platformLayersBit != 0 {
				if (pb.platformLayersBit & collidedBody.LayersBit()) == 0 {
					continue
				}
			}
			for _, contact := range m.Contacts() {
				normal := contact.Normal
				if contact.Particle != nil && contact.Particle.OwnerMesh() != nil &&
					contact.Particle.OwnerMesh().OwnerBody() != pb.AsBody() {
					normal = normal.Neg()
				}
				floorAngle := physics.AngleBetweenTwoVectors(normal, pb.upDirection)
				if math32.Abs(floorAngle) < pb.maxFloorAngle {
					pb.onFloor = true
					if collidedBody.BodyType() == physics.BodyTypeRigid {
						pb.lastMovableFloor = collidedBody
					}
					break
				}
			}
			if pb.onFloor {
				break
			}
		}
	} else {
		pb.SetPosition(tempPosition, false)
	}

	// Check ceiling
	pb.onCeiling = false
	tempPosition = pb.Position()
	pb.AddPosition(dirCeiling.Mul(1.0), false)
	if world != nil {
		ceilingManifolds := world.TestCollisionWithWorld(pb.AsBody())
		pb.SetPosition(tempPosition, false)

		for _, m := range ceilingManifolds {
			var collidedBody *physics.Body
			if m.BodyA() == pb.AsBody() {
				collidedBody = m.BodyB()
			} else {
				collidedBody = m.BodyA()
			}
			if collidedBody.BodyType() == physics.BodyTypeArea {
				continue
			}
			if pb.platformLayersBit != 0 {
				if (pb.platformLayersBit & collidedBody.LayersBit()) == 0 {
					continue
				}
			}
			for _, contact := range m.Contacts() {
				normal := contact.Normal
				if contact.Particle != nil && contact.Particle.OwnerMesh() != nil &&
					contact.Particle.OwnerMesh().OwnerBody() != pb.AsBody() {
					normal = normal.Neg()
				}
				floorAngle := physics.AngleBetweenTwoVectors(normal, pb.upDirection)
				if math32.Abs(floorAngle) > (math32.Pi - pb.maxFloorAngle) {
					pb.onCeiling = true
					break
				}
			}
			if pb.onCeiling {
				break
			}
		}
	} else {
		pb.SetPosition(tempPosition, false)
	}

	// Update rising/falling state
	if pb.onFloor {
		pb.isRising = false
		pb.isFalling = false
		pb.currentJumpCount = 0
	} else if pb.verticalVelocity.Dot(pb.upDirection) < 0 {
		pb.isFalling = true
		pb.isRising = false
	} else if pb.verticalVelocity.Dot(pb.upDirection) > 0 {
		pb.isFalling = false
		pb.isRising = true
	}

	// Velocity limit
	if pb.VelocityLimit() > 0.0 && pb.verticalVelocity.Length() > pb.VelocityLimit() {
		pb.verticalVelocity = pb.verticalVelocity.Normalized().Mul(pb.VelocityLimit())
	}

	// Apply vertical velocity
	if pb.onFloor && pb.verticalVelocity.Dot(pb.upDirection) < 0 {
		pb.verticalVelocity = physics.Vec2Zero()
	} else if pb.onCeiling && pb.verticalVelocity.Dot(pb.upDirection) > 0 {
		pb.verticalVelocity = physics.Vec2Zero()
	} else {
		// Sync prevPosition (matches qplatformerbody.cpp:579 default true).
		pb.AddPosition(pb.verticalVelocity, true)
		pb.verticalVelocity = pb.verticalVelocity.Add(gravityAmount)
	}

	// --- Horizontal velocities ---
	if pb.walkSide == 0 {
		// Decelerate
		if math32.Abs(pb.horizontalVelocity.X) < 0.001 && math32.Abs(pb.horizontalVelocity.Y) < 0.001 {
			pb.horizontalVelocity = physics.Vec2Zero()
		} else {
			pb.horizontalVelocity = pb.horizontalVelocity.Add(pb.horizontalVelocity.Neg().Mul(pb.walkDecelerationRate))
		}
	} else {
		// Accelerate toward target speed
		target := pb.rightDirection.Mul(pb.walkSpeed * float32(pb.walkSide))
		diff := target.Sub(pb.horizontalVelocity).Mul(pb.walkAccelerationRate)
		pb.horizontalVelocity = pb.horizontalVelocity.Add(diff)
	}

	if pb.VelocityLimit() > 0.0 && pb.horizontalVelocity.Length() > pb.VelocityLimit() {
		pb.horizontalVelocity = pb.horizontalVelocity.Normalized().Mul(pb.VelocityLimit())
	}

	// Apply horizontal velocity with slope handling
	if pb.horizontalVelocity != physics.Vec2Zero() {
		walkVector := pb.horizontalVelocity

		// Check for sloped floor and project walk vector onto floor tangent
		slopedFloor := pb.GetPlatformCollisions(pb.Position().Add(dirFloor.Mul(5.0)), true)
		if slopedFloor.HasBody() {
			floorAngle := physics.AngleBetweenTwoVectors(slopedFloor.Normal, pb.upDirection)
			if math32.Abs(floorAngle) <= pb.maxFloorAngle {
				floorUnit := slopedFloor.Normal.Perpendicular().Neg()
				walkVector = floorUnit.Mul(walkVector.Dot(floorUnit))
			}
		}

		// Sync prevPosition (matches qplatformerbody.cpp:621 default true).
		pb.AddPosition(walkVector, true)
	}

	// Wall collision tests
	rightWall := pb.GetRightWall(0.0)
	if rightWall.HasBody() {
		if pb.horizontalVelocity.Dot(pb.rightDirection) > 0 && rightWall.Body.Mode() == physics.BodyModeStatic {
			pb.horizontalVelocity = physics.Vec2Zero()
			defaultResponse := rightWall.Normal.Mul(rightWall.Penetration)
			onAxisResponse := pb.rightDirection.Mul(defaultResponse.Dot(pb.rightDirection))
			pb.SetPosition(pb.Position().Add(onAxisResponse), true)
		}
	}

	leftWall := pb.GetLeftWall(0.0)
	if leftWall.HasBody() {
		if pb.horizontalVelocity.Dot(pb.rightDirection) < 0 && leftWall.Body.Mode() == physics.BodyModeStatic {
			pb.horizontalVelocity = physics.Vec2Zero()
			defaultResponse := leftWall.Normal.Mul(leftWall.Penetration)
			onAxisResponse := pb.rightDirection.Neg().Mul(defaultResponse.Dot(pb.rightDirection.Neg()))
			pb.SetPosition(pb.Position().Add(onAxisResponse), true)
		}
	}
}
