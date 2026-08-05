package physics

// Manifold holds collision data between two bodies and resolves it.
// Faithful port of QManifold in qmanifold.h, qmanifold.cpp.
//
// Key C++ conventions:
//   - bodyA/bodyB are ordered by pointer (smaller first) — NOT reference/incident
//   - referenceBody = owner of contact.referenceParticles (the polygon that was hit)
//   - incidentBody = owner of contact.particle (the penetrating particle)
//   - normal = points from referenceBody toward incidentBody
//   - refResponseForce = -responseForce (applied to referenceBody via ApplyForce/ApplyForceToParticleSegment)
//   - incResponseForce = +responseForce (applied to incidentBody via ApplyForce/ApplyForceAt)
//   - invMass = 1 / (bodyA.mass + bodyB.mass) — uses RAW masses, no zeroing for static
//   - isCollisionOneSide = true if either body can't give response (includes static!)
type Manifold struct {
	bodyA    *Body
	bodyB    *Body
	contacts []*Contact
	world    *World

	// One-time computed properties (set in init)
	restitution        float32
	invMass            float32
	isCollisionOneSide bool
	linearRelVel       Vec2 // cached relative velocity from first contact
}

// init computes one-time properties. Matches QManifold constructor.
func (m *Manifold) init() {
	// restitution = min(bodyA.restitution, bodyB.restitution)
	m.restitution = m.bodyA.restitution
	if m.bodyB.restitution < m.restitution {
		m.restitution = m.bodyB.restitution
	}

	// invMass = 1 / (bodyA.mass + bodyB.mass) — C++ uses raw masses, no zeroing
	totalMass := m.bodyA.mass + m.bodyB.mass
	if totalMass > 0 {
		m.invMass = 1.0 / totalMass
	} else {
		m.invMass = 0
	}

	// isCollisionOneSide: true if either body can't give response to the other.
	// CanGiveCollisionResponseTo returns false if the OTHER body is static.
	// So for dynamic-vs-static, this is TRUE (one-sided = only the dynamic body moves).
	m.isCollisionOneSide = false
	if !m.bodyA.CanGiveCollisionResponseTo(m.bodyB) || !m.bodyB.CanGiveCollisionResponseTo(m.bodyA) {
		m.isCollisionOneSide = true
	}
}

func (m *Manifold) BodyA() *Body         { return m.bodyA }
func (m *Manifold) BodyB() *Body         { return m.bodyB }
func (m *Manifold) Contacts() []*Contact { return m.contacts }

// getRelativeVelocity computes relative velocity at contact point.
// Matches QManifold::GetRelativeVelocity in qmanifold.cpp:66-102.
//
// Returns (velRef + angVelRef * -rRef.Perpendicular()) - (velInc + angVelInc * -rInc.Perpendicular())
func (m *Manifold) getRelativeVelocity(contact *Contact, rRef, rInc Vec2) Vec2 {
	// Derive reference and incident bodies from contact particles
	var bodyRef, bodyInc *Body
	if len(contact.ReferenceParticles) > 0 && contact.ReferenceParticles[0] != nil {
		if mesh := contact.ReferenceParticles[0].OwnerMesh(); mesh != nil {
			bodyRef = mesh.OwnerBody()
		}
	}
	if contact.Particle != nil {
		if mesh := contact.Particle.OwnerMesh(); mesh != nil {
			bodyInc = mesh.OwnerBody()
		}
	}
	if bodyRef == nil {
		bodyRef = m.bodyA
	}
	if bodyInc == nil {
		bodyInc = m.bodyB
	}

	var velRef, velInc Vec2
	var angVelRef, angVelInc float32

	if bodyRef.bodyType == BodyTypeRigid {
		// Rigid body: vel = position - prevPosition, angVel = rotation - prevRotation
		angVelRef = bodyRef.rotation - bodyRef.prevRotation
		velRef = bodyRef.position.Sub(bodyRef.prevPosition)
	} else {
		// Soft body: average velocity of reference particles
		for _, p := range contact.ReferenceParticles {
			if p != nil {
				velRef = velRef.Add(p.GlobalPosition().Sub(p.PreviousGlobalPosition()))
			}
		}
		if len(contact.ReferenceParticles) > 0 {
			velRef = velRef.Div(float32(len(contact.ReferenceParticles)))
		}
	}

	if bodyInc.bodyType == BodyTypeRigid {
		angVelInc = bodyInc.rotation - bodyInc.prevRotation
		velInc = bodyInc.position.Sub(bodyInc.prevPosition)
	} else {
		// Soft body: particle velocity
		if contact.Particle != nil {
			velInc = contact.Particle.GlobalPosition().Sub(contact.Particle.PreviousGlobalPosition())
		}
	}

	// return (velRef + angVelRef * -rRef.Perpendicular()) - (velInc + angVelInc * -rInc.Perpendicular())
	refVel := velRef.Add(rRef.Perpendicular().Mul(-angVelRef))
	incVel := velInc.Add(rInc.Perpendicular().Mul(-angVelInc))
	return refVel.Sub(incVel)
}

// Solve applies position correction to resolve overlaps.
// Faithful port of QManifold::Solve in qmanifold.cpp:108-343.
func (m *Manifold) Solve() {
	// Area-body events are dispatched per-contact inside the loop (matches
	// qmanifold.cpp:187-199 + 229-230). The pre-loop dispatchAreaBodyEvents()
	// call has been removed because it lacked the cancelSolving/continue guard.

	// NOTE: C++ does NOT return early when isCollisionOneSide is true.
	// It continues solving but skips mass-weighting. The CanGiveCollisionResponseTo
	// checks prevent forces from being applied to bodies that can't receive them
	// (e.g., static bodies). So isCollisionOneSide only affects the mass-weighting.

	// Determine body type flags
	betweenRigidBodies := m.bodyA.bodyType == BodyTypeRigid && m.bodyB.bodyType == BodyTypeRigid
	betweenPressuredSoftBodies := false
	if m.bodyA.bodyType == BodyTypeSoft && m.bodyB.bodyType == BodyTypeSoft {
		sbA := asSoftBody(m.bodyA)
		sbB := asSoftBody(m.bodyB)
		if sbA != nil && sbB != nil && sbA.enableAreaPreserving && sbB.enableAreaPreserving {
			betweenPressuredSoftBodies = true
		}
	}

	for i, contact := range m.contacts {
		if contact.Solved {
			continue
		}

		// Scale penetration (mutates contact.Penetration to match C++ qmanifold.cpp:132-138).
		// The mutated value is later read by SolveFrictionAndVelocities -> ComputeFriction
		// as the Coulomb threshold `|jt| < penetration * staticFriction`.
		penetration := contact.Penetration
		if betweenRigidBodies {
			penetration *= 0.9
		} else if betweenPressuredSoftBodies {
			penetration *= 0.5
		}
		if penetration < 0 {
			penetration = 0
		}
		contact.Penetration = penetration

		// Response force
		responseForce := contact.Normal.Mul(penetration)
		if m.restitution > 0 {
			responseForce = responseForce.Mul(2.0)
		}
		if betweenRigidBodies {
			responseForce = responseForce.Div(float32(len(m.contacts)))
		}

		// Identify reference and incident bodies from contact particles
		var referenceBody, incidentBody *Body
		if len(contact.ReferenceParticles) > 0 && contact.ReferenceParticles[0] != nil {
			if mesh := contact.ReferenceParticles[0].OwnerMesh(); mesh != nil {
				referenceBody = mesh.OwnerBody()
			}
		}
		if contact.Particle != nil {
			if mesh := contact.Particle.OwnerMesh(); mesh != nil {
				incidentBody = mesh.OwnerBody()
			}
		}
		if referenceBody == nil {
			referenceBody = m.bodyA
		}
		if incidentBody == nil {
			incidentBody = m.bodyB
		}

		rRef := contact.Position.Sub(referenceBody.position)
		rInc := contact.Position.Sub(incidentBody.position)

		// Cache linear relative velocity for friction (first contact only)
		if i == 0 {
			m.linearRelVel = m.getRelativeVelocity(contact, rRef, rInc)
		}

		// Area-body sensors: register the overlap and skip ALL force application.
		// Matches qmanifold.cpp:187-199 + 229-230 — area bodies are pure sensors,
		// they must not apply position correction or friction.
		if referenceBody.bodyType == BodyTypeArea {
			if ab := asAreaBody(referenceBody); ab != nil {
				ab.addCollidedBody(incidentBody)
			}
			// Solved=false ensures SolveFrictionAndVelocities
			// SKIPS this contact (no friction on sensor bodies).
			// Matches C++ cancelSolving=true; continue which leaves
			// contact->solved at its initial false value.
			contact.Solved = false
			continue
		}
		if incidentBody.bodyType == BodyTypeArea {
			if ab := asAreaBody(incidentBody); ab != nil {
				ab.addCollidedBody(referenceBody)
			}
			contact.Solved = false
			continue
		}

		// Dispatch collision events
		infoRef := CollisionInfo{
			Position:    contact.Position,
			Body:        incidentBody,
			Normal:      contact.Normal.Neg(),
			Penetration: contact.Penetration,
		}
		infoInc := CollisionInfo{
			Position:    contact.Position,
			Body:        referenceBody,
			Normal:      contact.Normal,
			Penetration: contact.Penetration,
		}
		cancelSolving := false
		if !referenceBody.onCollision(infoRef) {
			cancelSolving = true
		}
		if !incidentBody.onCollision(infoInc) {
			cancelSolving = true
		}
		if cancelSolving {
			contact.Solved = true
			continue
		}

		// Check disabled particles
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

		// Lazy particle handling (one-way platforms / pass-through).
		// Faithful port of qmanifold.cpp:254-287, 307, 322, 333-334.
		//
		// A lazy particle only collides ONCE per body-pair. The first
		// contact registers the body in oneTimeCollidedBodies; subsequent
		// contacts with the same body are skipped. This implements
		// one-way platforms: a body falling onto a lazy particle from
		// above collides once, then passes through if it tries to come
		// back up.
		incidentParticleIsLazy := false
		referenceParticlesAreLazy := false
		if contact.Particle != nil && contact.Particle.IsLazy() {
			contact.Particle.addOneTimeCollision(referenceBody)
			incidentParticleIsLazy = true
			m.isCollisionOneSide = true
		}
		for _, rp := range contact.ReferenceParticles {
			if rp != nil && rp.IsLazy() {
				rp.addOneTimeCollision(incidentBody)
				m.isCollisionOneSide = true
				referenceParticlesAreLazy = true
			}
		}
		// Skip if the incident particle has already collided with this reference body.
		if contact.Particle != nil && contact.Particle.IsLazy() {
			if contact.Particle.oneTimeCollidedBodies != nil {
				if _, found := contact.Particle.oneTimeCollidedBodies[referenceBody]; found {
					// Already collided before — skip this contact.
					// (C++ uses `continue` here, but we need to mark
					// the contact as not-solved so friction skips it too.)
					contact.Solved = false
					continue
				}
			}
		}
		// Skip if any lazy reference particle has already collided with the incident body.
		for _, rp := range contact.ReferenceParticles {
			if rp != nil && rp.IsLazy() {
				if rp.oneTimeCollidedBodies != nil {
					if _, found := rp.oneTimeCollidedBodies[incidentBody]; found {
						contact.Solved = false
						referenceParticlesAreLazy = true // ensure skip-below
						break
					}
				}
			}
		}
		if referenceParticlesAreLazy {
			continue
		}

		// Compute response forces
		refResponseForce := responseForce.Neg()
		incResponseForce := responseForce

		// Mass weighting (C++: if isCollisionOneSide==false, scale by mass)
		// When isCollisionOneSide is true (e.g., dynamic vs static),
		// forces are NOT mass-weighted — the dynamic body gets the FULL force.
		if !m.isCollisionOneSide {
			refResponseForce = refResponseForce.Mul(incidentBody.Mass() * m.invMass)
			incResponseForce = incResponseForce.Mul(referenceBody.Mass() * m.invMass)
		}

		// Apply force to reference body (the polygon that was hit).
		// Gate: skip if the incident particle is lazy (one-way: incident
		// passes through, only reference gets force if it's the lazy side
		// — wait, C++ gates reference-force by !incidentParticleIsLazy).
		if !incidentParticleIsLazy && incidentBody.CanGiveCollisionResponseTo(referenceBody) {
			contact.Solved = true
			if referenceBody.bodyType == BodyTypeRigid {
				if rb := asRigidBody(referenceBody); rb != nil {
					rb.ApplyForceAt(refResponseForce, rRef, true)
				}
			} else if referenceBody.bodyType == BodyTypeSoft {
				if len(contact.ReferenceParticles) == 2 {
					ApplyForceToParticleSegment(
						contact.ReferenceParticles[0],
						contact.ReferenceParticles[1],
						refResponseForce,
						contact.Position,
					)
				} else if len(contact.ReferenceParticles) >= 1 && contact.ReferenceParticles[0] != nil {
					contact.ReferenceParticles[0].ApplyForce(refResponseForce)
				}
			}
		}

		// Apply force to incident body (the penetrating particle).
		// Gate: skip if any reference particle is lazy.
		if !referenceParticlesAreLazy && referenceBody.CanGiveCollisionResponseTo(incidentBody) {
			contact.Solved = true
			if incidentBody.bodyType == BodyTypeRigid {
				if rb := asRigidBody(incidentBody); rb != nil {
					rb.ApplyForceAt(incResponseForce, rInc, true)
				}
			} else if incidentBody.bodyType == BodyTypeSoft {
				if contact.Particle != nil {
					contact.Particle.ApplyForce(incResponseForce)
				}
			}
		}

		// If either side is lazy, force Solved=false so friction skips it.
		// Matches qmanifold.cpp:333-334.
		if incidentParticleIsLazy || referenceParticlesAreLazy {
			contact.Solved = false
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
// Faithful port of QManifold::SolveFrictionAndVelocities in qmanifold.cpp:345-470.
func (m *Manifold) SolveFrictionAndVelocities() {
	// Don't apply friction to kinematic and static body pairs
	isBodyADynamic := !m.bodyA.isKinematic && m.bodyA.mode != BodyModeStatic
	isBodyBDynamic := !m.bodyB.isKinematic && m.bodyB.mode != BodyModeStatic
	if !isBodyADynamic && !isBodyBDynamic {
		return
	}

	for i, contact := range m.contacts {
		if !contact.Solved {
			continue
		}

		// Identify reference and incident bodies
		var referenceBody, incidentBody *Body
		if len(contact.ReferenceParticles) > 0 && contact.ReferenceParticles[0] != nil {
			if mesh := contact.ReferenceParticles[0].OwnerMesh(); mesh != nil {
				referenceBody = mesh.OwnerBody()
			}
		}
		if contact.Particle != nil {
			if mesh := contact.Particle.OwnerMesh(); mesh != nil {
				incidentBody = mesh.OwnerBody()
			}
		}
		if referenceBody == nil {
			referenceBody = m.bodyA
		}
		if incidentBody == nil {
			incidentBody = m.bodyB
		}

		var refRigidBody *RigidBody
		var incRigidBody *RigidBody
		if referenceBody.bodyType == BodyTypeRigid {
			refRigidBody = asRigidBody(referenceBody)
		}
		if incidentBody.bodyType == BodyTypeRigid {
			incRigidBody = asRigidBody(incidentBody)
		}

		rRef := contact.Position.Sub(referenceBody.position)
		rInc := contact.Position.Sub(incidentBody.position)

		// Velocity correction (restitution) — only on first contact
		if i == 0 && m.restitution > 0 {
			j := m.linearRelVel.Dot(contact.Normal)
			if j > m.restitution*2.0 {
				relVel := m.getRelativeVelocity(contact, Vec2Zero(), Vec2Zero())
				tangent := relVel.Sub(contact.Normal.Mul(relVel.Dot(contact.Normal)))
				jn := contact.Normal.Mul(j).Mul(m.restitution).Sub(tangent)

				refImpulse := jn.Neg()
				incImpulse := jn

				if !m.isCollisionOneSide {
					refImpulse = refImpulse.Mul(incidentBody.Mass() * m.invMass)
					incImpulse = incImpulse.Mul(referenceBody.Mass() * m.invMass)
				}

				// Apply impulses (Verlet-style: modify prevPosition)
				if refRigidBody != nil {
					if incidentBody.CanGiveCollisionResponseTo(referenceBody) && !referenceBody.isKinematic {
						refRigidBody.SetPreviousPosition(referenceBody.position.Sub(refImpulse))
					}
				}
				if incRigidBody != nil {
					if referenceBody.CanGiveCollisionResponseTo(incidentBody) && !incidentBody.isKinematic {
						incRigidBody.SetPreviousPosition(incidentBody.position.Sub(incImpulse))
					}
				}
			}
		}

		// Friction
		relVel := m.getRelativeVelocity(contact, rRef, rInc)
		frictionForce := ComputeFriction(incidentBody, referenceBody, contact.Normal, contact.Penetration, relVel)

		// C++ convention: refResponseForce = frictionForce, incResponseForce = -frictionForce
		refResponseForce := frictionForce
		incResponseForce := frictionForce.Neg()

		if !m.isCollisionOneSide {
			refResponseForce = refResponseForce.Mul(incidentBody.Mass() * m.invMass)
			incResponseForce = incResponseForce.Mul(referenceBody.Mass() * m.invMass)
		}

		// Apply friction to reference body
		if incidentBody.CanGiveCollisionResponseTo(referenceBody) && !referenceBody.isKinematic {
			if refRigidBody != nil {
				refRigidBody.ApplyForceAt(refResponseForce, rRef, true)
			} else if referenceBody.bodyType == BodyTypeSoft {
				if len(contact.ReferenceParticles) == 2 {
					ApplyForceToParticleSegment(
						contact.ReferenceParticles[0],
						contact.ReferenceParticles[1],
						refResponseForce,
						contact.Particle.GlobalPosition(),
					)
				} else if len(contact.ReferenceParticles) >= 1 && contact.ReferenceParticles[0] != nil {
					contact.ReferenceParticles[0].ApplyForce(refResponseForce)
				}
			}
		}

		// Apply friction to incident body
		if referenceBody.CanGiveCollisionResponseTo(incidentBody) && !incidentBody.isKinematic {
			if incRigidBody != nil {
				incRigidBody.ApplyForceAt(incResponseForce, rInc, true)
			} else if incidentBody.bodyType == BodyTypeSoft && contact.Particle != nil {
				contact.Particle.ApplyForce(incResponseForce)
			}
		}
	}
}

// onCollision dispatches the OnCollision virtual event and the CollisionEventListener.
func (b *Body) onCollision(info CollisionInfo) bool {
	result := true
	if b.OnCollision != nil {
		result = b.OnCollision(b, info)
	}
	return result
}

// dispatchAreaBodyEvents notifies area bodies of enter/exit events.
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
