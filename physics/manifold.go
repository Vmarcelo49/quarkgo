package physics

// Manifold holds collision data between two bodies and resolves it.
// Matches QManifold in qmanifold.h, qmanifold.cpp.
//
// A Manifold is created for each colliding body pair, per solver iteration.
// It owns a list of Contact pointers (from the ContactPool).
//
// Resolution happens in two phases:
//   1. Solve() — position correction (push bodies apart along contact normals)
//   2. SolveFrictionAndVelocities() — apply restitution impulse and friction
type Manifold struct {
        bodyA    *Body
        bodyB    *Body
        contacts []*Contact
        world    *World

        // One-time computed properties (set in init)
        restitution      float32
        invMass          float32
        massA            float32 // effective mass of A (0 if static/sleeping)
        massB            float32 // effective mass of B (0 if static/sleeping)
        invMassA         float32 // 1/massA (0 if static/sleeping)
        invMassB         float32 // 1/massB (0 if static/sleeping)
        isCollisionOneSide bool
}

// init computes one-time properties. Matches QManifold::QManifold constructor.
func (m *Manifold) init() {
        // restitution = min(bodyA.restitution, bodyB.restitution)
        m.restitution = m.bodyA.restitution
        if m.bodyB.restitution < m.restitution {
                m.restitution = m.bodyB.restitution
        }

        // For mass weighting: static bodies have effectively infinite mass.
        // invMass for a static body is 0 (it doesn't move).
        // For dynamic bodies, invMass = 1/mass.
        m.massA = m.bodyA.mass
        m.massB = m.bodyB.mass
        invMassA := float32(0)
        invMassB := float32(0)
        if m.bodyA.mode != BodyModeStatic && !m.bodyA.isSleeping && m.massA > 0 {
                invMassA = 1.0 / m.massA
        }
        if m.bodyB.mode != BodyModeStatic && !m.bodyB.isSleeping && m.massB > 0 {
                invMassB = 1.0 / m.massB
        }
        totalInvMass := invMassA + invMassB
        if totalInvMass > 0 {
                m.invMass = totalInvMass
        } else {
                m.invMass = 0
        }
        m.invMassA = invMassA
        m.invMassB = invMassB

        // Check if either body refuses collision response entirely.
        // In the C++ engine, isCollisionOneSide defaults to false and is only
        // set true in specific cases (e.g., kinematic-kinematic without
        // allowKinematicCollisions). A static body does NOT make the collision
        // one-sided — the dynamic body still gets pushed away.
        m.isCollisionOneSide = false
        // Special case: both bodies are kinematic without kinematic collisions
        if m.bodyA.isKinematic && m.bodyB.isKinematic {
                if !m.bodyA.allowKinematicCollisions || !m.bodyB.allowKinematicCollisions {
                        m.isCollisionOneSide = true
                }
        }
}

// BodyA returns bodyA.
func (m *Manifold) BodyA() *Body { return m.bodyA }

// BodyB returns bodyB.
func (m *Manifold) BodyB() *Body { return m.bodyB }

// Contacts returns the contacts.
func (m *Manifold) Contacts() []*Contact { return m.contacts }

// Solve applies position correction to resolve overlaps.
// Matches QManifold::Solve in qmanifold.cpp:108-343.
//
// C++ convention:
//   - referenceBody = owner of contact.referenceParticles (the polygon that was hit)
//   - incidentBody = owner of contact.particle (the penetrating particle)
//   - normal = points from referenceBody toward incidentBody
//   - refResponseForce = -responseForce (applied to referenceBody)
//   - incResponseForce = +responseForce (applied to incidentBody)
func (m *Manifold) Solve() {
	// Always dispatch area body events (enter/exit tracking)
	m.dispatchAreaBodyEvents()

	if m.isCollisionOneSide {
		return
	}

	// Determine body types
	betweenRigidBodies := m.bodyA.bodyType == BodyTypeRigid && m.bodyB.bodyType == BodyTypeRigid
	betweenPressuredSoftBodies := false
	if m.bodyA.bodyType == BodyTypeSoft && m.bodyB.bodyType == BodyTypeSoft {
		// Check if both have area preserving enabled
		sbA := asSoftBody(m.bodyA)
		sbB := asSoftBody(m.bodyB)
		if sbA != nil && sbB != nil && sbA.enableAreaPreserving && sbB.enableAreaPreserving {
			betweenPressuredSoftBodies = true
		}
	}

	for _, contact := range m.contacts {
		if contact.Solved {
			continue
		}

		penetration := contact.Penetration
		if betweenRigidBodies {
			penetration *= 0.9
		} else if betweenPressuredSoftBodies {
			penetration *= 0.5
		}
		if penetration < 0 {
			penetration = 0
		}

		responseForce := contact.Normal.Mul(penetration)
		if m.restitution > 0 {
			responseForce = responseForce.Mul(2.0)
		}
		if betweenRigidBodies {
			responseForce = responseForce.Div(float32(len(m.contacts)))
		}

		// Identify reference and incident bodies (C++ convention)
		// referenceBody = owner of contact.referenceParticles
		// incidentBody = owner of contact.particle
		var referenceBody, incidentBody *Body
		if len(contact.ReferenceParticles) > 0 && contact.ReferenceParticles[0] != nil {
			if contact.ReferenceParticles[0].OwnerMesh() != nil {
				referenceBody = contact.ReferenceParticles[0].OwnerMesh().OwnerBody()
			}
		}
		if contact.Particle != nil && contact.Particle.OwnerMesh() != nil {
			incidentBody = contact.Particle.OwnerMesh().OwnerBody()
		}

		// Fallback: if we can't determine from particles, use bodyA/bodyB
		if referenceBody == nil {
			referenceBody = m.bodyA
		}
		if incidentBody == nil {
			incidentBody = m.bodyB
		}

		// Compute contact position relative to each body
		rRef := contact.Position.Sub(referenceBody.position)
		rInc := contact.Position.Sub(incidentBody.position)

		// Dispatch collision events
		info := CollisionInfo{
			Position:    contact.Position,
			Body:        incidentBody,
			Normal:      contact.Normal.Neg(),
			Penetration: contact.Penetration,
		}
		applyResponse := true
		if referenceBody.OnCollision != nil {
			if !referenceBody.OnCollision(referenceBody, info) {
				applyResponse = false
			}
		}
		info.Body = referenceBody
		info.Normal = contact.Normal
		if incidentBody.OnCollision != nil {
			if !incidentBody.OnCollision(incidentBody, info) {
				applyResponse = false
			}
		}
		if !applyResponse {
			contact.Solved = true
			continue
		}

		// Check if particles are enabled
		if contact.Particle != nil && !contact.Particle.enabled {
			continue
		}
		refEnabled := true
		for _, rp := range contact.ReferenceParticles {
			if rp != nil && !rp.enabled {
				refEnabled = false
				break
			}
		}
		if !refEnabled {
			continue
		}

		// Compute response forces (C++ convention)
		refResponseForce := responseForce.Neg()
		incResponseForce := responseForce

		// Mass-weight the forces (C++: refResponseForce *= incidentBody->GetMass() * invMass)
		refResponseForce = refResponseForce.Mul(incidentBody.Mass() * m.invMass)
		incResponseForce = incResponseForce.Mul(referenceBody.Mass() * m.invMass)

		// Apply force to reference body (the polygon that was hit)
		if incidentBody.CanGiveCollisionResponseTo(referenceBody) {
			contact.Solved = true
			if referenceBody.bodyType == BodyTypeRigid {
				if rb := asRigidBody(referenceBody); rb != nil {
					rb.ApplyForceAt(refResponseForce, rRef, true)
				}
			} else if referenceBody.bodyType == BodyTypeSoft {
				// Apply to reference particles (the polygon edge)
				if len(contact.ReferenceParticles) == 2 {
					ApplyForceToParticleSegment(
						contact.ReferenceParticles[0],
						contact.ReferenceParticles[1],
						refResponseForce,
						contact.Position,
					)
				} else if len(contact.ReferenceParticles) == 1 {
					contact.ReferenceParticles[0].ApplyForce(refResponseForce)
				}
			}
		}

		// Apply force to incident body (the penetrating particle)
		if referenceBody.CanGiveCollisionResponseTo(incidentBody) {
			contact.Solved = true
			if incidentBody.bodyType == BodyTypeRigid {
				if rb := asRigidBody(incidentBody); rb != nil {
					rb.ApplyForceAt(incResponseForce, rInc, true)
				}
			} else if incidentBody.bodyType == BodyTypeSoft {
				// Apply to the incident particle directly
				if contact.Particle != nil {
					contact.Particle.ApplyForce(incResponseForce)
				}
			}
		}

		// Debug gizmo
		if m.world != nil && m.world.debugGizmos {
			m.world.AddGizmo(NewGizmoLine(
				contact.Position,
				contact.Position.Add(contact.Normal.Mul(contact.Penetration)),
				true,
			))
		}
	}
}

// SolveFrictionAndVelocities applies restitution and friction.
// Matches QManifold::SolveFrictionAndVelocities in qmanifold.cpp:345-470.
func (m *Manifold) SolveFrictionAndVelocities() {
        if m.isCollisionOneSide {
                return
        }
        // Skip if both bodies are static/kinematic
        if (m.bodyA.mode == BodyModeStatic || m.bodyA.isKinematic) &&
                (m.bodyB.mode == BodyModeStatic || m.bodyB.isKinematic) {
                return
        }

        for _, contact := range m.contacts {
                // Compute relative velocity
                relVel := m.relativeVelocity(contact)
                velAlongNormal := relVel.Dot(contact.Normal)

                // Restitution (only on first contact, if moving toward each other)
                if m.restitution > 0 && velAlongNormal > 0 {
                        j := -(1 + m.restitution) * velAlongNormal
                        j *= m.invMass
                        impulse := contact.Normal.Mul(j)
                        // Apply impulse (Verlet-style: modify prevPosition)
                        m.applyImpulse(m.bodyA, impulse.Neg(), contact)
                        m.applyImpulse(m.bodyB, impulse, contact)
                }

                // Friction
                tangent := relVel.Sub(contact.Normal.Mul(velAlongNormal))
                if tangent.LengthSquared() > 1e-6 {
                        tangent = tangent.Normalized()
                        frictionForce := ComputeFriction(m.bodyA, m.bodyB, contact.Normal, contact.Penetration, relVel)
                        if rb := asRigidBody(m.bodyA); rb != nil { r := contact.Position.Sub(m.bodyA.position); rb.ApplyForceAt(frictionForce.Neg(), r, true) } else if m.bodyA.bodyType == BodyTypeSoft && contact.Particle != nil { contact.Particle.ApplyForce(frictionForce.Neg()) }
                        if rb := asRigidBody(m.bodyB); rb != nil { r := contact.Position.Sub(m.bodyB.position); rb.ApplyForceAt(frictionForce, r, true) } else if m.bodyB.bodyType == BodyTypeSoft && contact.Particle != nil { contact.Particle.ApplyForce(frictionForce) }
                }
        }
}

// relativeVelocity computes the relative velocity at the contact point.
// Matches QManifold::GetRelativeVelocity in qmanifold.cpp:66-102.
func (m *Manifold) relativeVelocity(contact *Contact) Vec2 {
        // For rigid bodies: vel = bodyVel + angVel × r
        // For soft bodies: vel = particle.vel (averaged over reference particles)
        if m.bodyA.bodyType == BodyTypeRigid {
                velA := m.bodyA.position.Sub(m.bodyA.prevPosition)
                rA := contact.Position.Sub(m.bodyA.position)
                // angVel contributes: angVel * r.Perpendicular()
                angVelA := m.bodyA.rotation - m.bodyA.prevRotation
                velA = velA.Add(rA.Perpendicular().Mul(angVelA))

                if m.bodyB.bodyType == BodyTypeRigid {
                        velB := m.bodyB.position.Sub(m.bodyB.prevPosition)
                        rB := contact.Position.Sub(m.bodyB.position)
                        angVelB := m.bodyB.rotation - m.bodyB.prevRotation
                        velB = velB.Add(rB.Perpendicular().Mul(angVelB))
                        return velA.Sub(velB)
                }
                // Rigid vs soft
                if contact.Particle != nil {
                        velB := contact.Particle.GlobalPosition().Sub(contact.Particle.PreviousGlobalPosition())
                        return velA.Sub(velB)
                }
                return velA
        }

        // Soft body fallback
        if contact.Particle != nil {
                return contact.Particle.GlobalPosition().Sub(contact.Particle.PreviousGlobalPosition())
        }
        return Vec2Zero()
}

// applyImpulse applies a Verlet-style impulse to a body (modifies prevPosition).
func (m *Manifold) applyImpulse(body *Body, impulse Vec2, contact *Contact) {
        if body.bodyType == BodyTypeRigid {
                rb := asRigidBody(body)
                if rb == nil {
                        return
                }
                r := contact.Position.Sub(body.position)
                rb.ApplyImpulse(impulse, r)
        } else if body.bodyType == BodyTypeSoft && contact.Particle != nil {
                contact.Particle.AddPreviousGlobalPosition(impulse.Neg())
        }
}

// dispatchAreaBodyEvents notifies area bodies of enter/exit events.
// For area bodies, we register the collision in the area body's "bodies" set.
// CheckBodies (called by World.Update after the solver) will dispatch
// OnCollisionEnter/Exit based on this set.
func (m *Manifold) dispatchAreaBodyEvents() {
        if m.bodyA.bodyType == BodyTypeArea {
                if ab := asAreaBody(m.bodyA); ab != nil {
                        ab.addCollidedBody(m.bodyB)
                }
        }
        if m.bodyB.bodyType == BodyTypeArea {
                if ab := asAreaBody(m.bodyB); ab != nil {
                        ab.addCollidedBody(m.bodyA)
                }
        }
}
