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
// For each contact:
//   - Scale penetration (0.9× rigid-rigid, 0.5× soft-soft)
//   - Compute response force = normal * penetration
//   - Dispatch OnCollision events (can cancel solving)
//   - Apply mass-weighted forces to both bodies
func (m *Manifold) Solve() {
        // Always dispatch area body events (enter/exit tracking)
        m.dispatchAreaBodyEvents()

        if m.isCollisionOneSide {
                return
        }

        for _, contact := range m.contacts {
                if contact.Solved {
                        continue
                }

                // Scale penetration based on body types
                penetration := contact.Penetration
                if m.bodyA.bodyType == BodyTypeRigid && m.bodyB.bodyType == BodyTypeRigid {
                        penetration *= 0.9
                } else if m.bodyA.bodyType == BodyTypeSoft || m.bodyB.bodyType == BodyTypeSoft {
                        penetration *= 0.5
                }
                if penetration < 0 {
                        penetration = 0
                }

                // Response force
                responseForce := contact.Normal.Mul(penetration)
                if m.restitution > 0 {
                        responseForce = responseForce.Mul(2.0)
                }
                // Divide by contact count for rigid-rigid pairs (distributes force)
                if m.bodyA.bodyType == BodyTypeRigid && m.bodyB.bodyType == BodyTypeRigid && len(m.contacts) > 0 {
                        responseForce = responseForce.Div(float32(len(m.contacts)))
                }

                // Dispatch OnCollision events
                info := CollisionInfo{
                        Position:    contact.Position,
                        Body:        m.bodyB,
                        Normal:      contact.Normal,
                        Penetration: contact.Penetration,
                }
                applyResponse := true
                if m.bodyA.OnCollision != nil {
                        if !m.bodyA.OnCollision(m.bodyA, info) {
                                applyResponse = false
                        }
                }
                info.Body = m.bodyA
                if m.bodyB.OnCollision != nil {
                        if !m.bodyB.OnCollision(m.bodyB, info) {
                                applyResponse = false
                        }
                }
                if !applyResponse {
                        contact.Solved = true
                        continue
                }

                // Area body short-circuit: don't apply forces to area bodies
                if m.bodyA.bodyType == BodyTypeArea || m.bodyB.bodyType == BodyTypeArea {
                        m.dispatchAreaBodyEvents()
                        contact.Solved = true
                        continue
                }

                // Mass-weighted response using INVERSE mass.
                // Normal points from bodyB toward bodyA.
                // bodyA should be pushed in +normal direction, bodyB in -normal.
                // Each body's share = its invMass / total invMass.
                // Static body (invMass=0) doesn't move; dynamic body gets full push.
                var responseA, responseB Vec2
                if m.invMass > 0 {
                        fracA := m.invMassA / m.invMass // A's share
                        fracB := m.invMassB / m.invMass // B's share
                        responseA = responseForce.Mul(fracA)
                        responseB = responseForce.Neg().Mul(fracB)
                }

                // Apply to bodies
                m.applyForceToBody(m.bodyA, responseA, contact)
                m.applyForceToBody(m.bodyB, responseB, contact)

                // Debug gizmo
                if m.world != nil && m.world.debugGizmos {
                        m.world.AddGizmo(NewGizmoLine(
                                contact.Position,
                                contact.Position.Add(contact.Normal.Mul(contact.Penetration)),
                                true,
                        ))
                }

                contact.Solved = true
        }

        // Return contacts to pool after solving
        // (The C++ engine does FreeAll at the start of the next iteration;
        // we rely on sync.Pool's GC-friendly behavior, so we return here.)
        // NOTE: We don't return here because SolveFrictionAndVelocities still
        // needs the contacts. The pool is reset implicitly on next iteration.
}

// applyForceToBody applies a response force to a body, handling both rigid
// bodies (via ApplyForceAt with torque) and soft bodies (via particle forces).
func (m *Manifold) applyForceToBody(body *Body, force Vec2, contact *Contact) {
        if body.bodyType == BodyTypeRigid {
                rb := asRigidBody(body)
                if rb == nil {
                        return
                }
                // Compute contact point relative to body center
                r := contact.Position.Sub(body.position)
                rb.ApplyForceAt(force, r, true)
        } else if body.bodyType == BodyTypeSoft {
                // Soft body: apply force to the contact particle directly,
                // distributed across reference particles if there are 2.
                if len(contact.ReferenceParticles) == 2 {
                        ApplyForceToParticleSegment(
                                contact.ReferenceParticles[0],
                                contact.ReferenceParticles[1],
                                force,
                                contact.Position,
                        )
                } else if contact.Particle != nil {
                        contact.Particle.ApplyForce(force)
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
                        m.applyForceToBody(m.bodyA, frictionForce.Neg(), contact)
                        m.applyForceToBody(m.bodyB, frictionForce, contact)
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
