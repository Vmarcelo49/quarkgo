package physics

// BodyType enumerates the body kinds. Matches QBody::BodyTypes in qbody.h:56-60.
type BodyType int

const (
        // BodyTypeRigid is a non-deformable solid body simulated with Verlet integration.
        BodyTypeRigid BodyType = iota

        // BodyTypeArea is a sensor/trigger body that reports collisions but doesn't respond.
        BodyTypeArea

        // BodyTypeSoft is a deformable body using mass-spring model with PBD.
        BodyTypeSoft
)

// BodyMode determines whether a body reacts to forces and collisions.
// Matches QBody::Modes in qbody.h:51-54.
type BodyMode int

const (
        // BodyModeDynamic reacts to forces, constraints, and collisions.
        BodyModeDynamic BodyMode = iota

        // BodyModeStatic does not react; it provides collision surfaces for dynamic bodies.
        BodyModeStatic
)

// Body is the base type for all physics bodies. Matches QBody in
// qbody.h, qbody.cpp.
//
// Body is abstract in the C++ engine (has virtual methods). In Go, we
// embed it as a struct field in RigidBody, SoftBody, and AreaBody.
// The BodyType field drives dispatch in World.Update via a switch
// (see D006 in DECISIONS.md).
//
// Key concepts:
//   - Verlet integration: velocity is implicit (position - prevPosition)
//   - Meshes carry particles, which carry their own positions
//   - The AABB is recomputed from particle positions
//   - Sleeping bodies skip integration but still collide
type Body struct {
        world    *World
        bodyType BodyType

        // Transform
        position     Vec2
        prevPosition Vec2
        rotation     float32
        prevRotation float32

        // Bounding box
        aabb AABB

        // State
        mode    BodyMode
        enabled bool

        // Physics properties
        friction        float32
        staticFriction  float32
        airFriction     float32
        mass            float32
        restitution     float32
        velocityLimit   float32

        // Collision filtering
        layersBit           int
        collidableLayersBit int

        // Kinematic
        isKinematic           bool
        allowKinematicCollisions bool

        // Sleeping
        isSleeping bool
        sleepTick  int
        canSleep   bool

        // Time scale
        enableBodySpecificTimeScale bool
        bodySpecificTimeScale       float32

        // Velocity integration toggle
        enableIntegratedVelocities bool

        // Custom gravity
        enableCustomGravity bool
        customGravity       Vec2

        // Cached-derived
        inertiaNeedsUpdate     bool
        circumferenceNeedsUpdate bool

        // Meshes
        meshes []*Mesh

        // Event listeners (function fields, replace std::function)
        OnPreStep func(*Body)
        OnStep    func(*Body)
        OnCollision func(*Body, CollisionInfo) bool

        // Set by QAreaBody to exempt this body from gravity
        ignoreGravity bool
}

// NewBody constructs a Body with default values matching QBody::QBody.
func NewBody() *Body {
        return &Body{
                bodyType:                  BodyTypeRigid,
                mode:                      BodyModeDynamic,
                enabled:                   true,
                friction:                  0.2,
                staticFriction:            0.5,
                airFriction:               0.01,
                mass:                      1.0,
                layersBit:                 1,
                collidableLayersBit:       1,
                canSleep:                  true,
                sleepTick:                 120,
                enableIntegratedVelocities: true,
                bodySpecificTimeScale:     1.0,
                inertiaNeedsUpdate:        true,
                circumferenceNeedsUpdate:  true,
        }
}

// --- Getters ---

// BodyType returns the body's type (rigid, soft, or area).
func (b *Body) BodyType() BodyType { return b.bodyType }

// World returns the world this body belongs to, or nil if not added.
func (b *Body) World() *World { return b.world }

// Position returns the body's world-space position.
func (b *Body) Position() Vec2 { return b.position }

// PreviousPosition returns the body's previous position (Verlet velocity source).
func (b *Body) PreviousPosition() Vec2 { return b.prevPosition }

// Rotation returns the body's rotation in radians.
func (b *Body) Rotation() float32 { return b.rotation }

// RotationDegree returns the body's rotation in degrees.
func (b *Body) RotationDegree() float32 { return b.rotation / (Pi / 180) }

// PreviousRotation returns the body's previous rotation.
func (b *Body) PreviousRotation() float32 { return b.prevRotation }

// AABB returns the body's axis-aligned bounding box.
func (b *Body) AABB() AABB { return b.aabb }

// Mode returns whether the body is dynamic or static.
func (b *Body) Mode() BodyMode { return b.mode }

// Enabled reports whether the body is active.
func (b *Body) Enabled() bool { return b.enabled }

// Friction returns the body's dynamic friction coefficient.
func (b *Body) Friction() float32 { return b.friction }

// StaticFriction returns the body's static friction coefficient.
func (b *Body) StaticFriction() float32 { return b.staticFriction }

// AirFriction returns the body's air friction (drag) coefficient.
func (b *Body) AirFriction() float32 { return b.airFriction }

// Mass returns the body's mass.
func (b *Body) Mass() float32 { return b.mass }

// Restitution returns the body's restitution (bounciness).
func (b *Body) Restitution() float32 { return b.restitution }

// VelocityLimit returns the maximum velocity; 0 means unlimited.
func (b *Body) VelocityLimit() float32 { return b.velocityLimit }

// LayersBit returns the bitmask of layers this body is on.
func (b *Body) LayersBit() int { return b.layersBit }

// CollidableLayersBit returns the bitmask of layers this body can collide with.
func (b *Body) CollidableLayersBit() int { return b.collidableLayersBit }

// IsKinematic reports whether the body is kinematic (user-controlled, not
// affected by forces).
func (b *Body) IsKinematic() bool { return b.isKinematic }

// AllowKinematicCollisions reports whether this kinematic body reacts to
// collisions with other kinematic bodies.
func (b *Body) AllowKinematicCollisions() bool { return b.allowKinematicCollisions }

// IsSleeping reports whether the body is currently sleeping.
func (b *Body) IsSleeping() bool { return b.isSleeping }

// CanSleep reports whether the body is allowed to sleep.
func (b *Body) CanSleep() bool { return b.canSleep }

// IntegratedVelocitiesEnabled reports whether Verlet integration is active.
func (b *Body) IntegratedVelocitiesEnabled() bool { return b.enableIntegratedVelocities }

// CustomGravityEnabled reports whether a per-body gravity override is active.
func (b *Body) CustomGravityEnabled() bool { return b.enableCustomGravity }

// CustomGravity returns the per-body gravity vector (if enabled).
func (b *Body) CustomGravity() Vec2 { return b.customGravity }

// IgnoreGravity reports whether the body is exempt from gravity.
// Set by QAreaBody when gravityFree is enabled.
func (b *Body) IgnoreGravity() bool { return b.ignoreGravity }

// Meshes returns the body's meshes.
func (b *Body) Meshes() []*Mesh { return b.meshes }

// MeshCount returns the number of meshes.
func (b *Body) MeshCount() int { return len(b.meshes) }

// MeshAt returns the mesh at the given index.
func (b *Body) MeshAt(i int) *Mesh { return b.meshes[i] }

// TotalInitialArea returns the sum of all meshes' initial areas.
func (b *Body) TotalInitialArea() float32 {
        var res float32
        for _, m := range b.meshes {
                res += m.InitialArea()
        }
        return res
}

// Inertia returns the body's rotational inertia. Computed lazily.
// Matches QBody::GetInertia in qbody.h:261-268.
func (b *Body) Inertia() float32 {
        if b.inertiaNeedsUpdate {
                inertia := b.TotalInitialArea() * 2.0 * b.mass
                if inertia < 500.0 {
                        inertia = 500.0
                }
                // Note: we don't store inertia as a field because TotalInitialArea
                // can change when meshes change. We recompute each call when dirty.
                b.inertiaNeedsUpdate = false
                // Cache via a local closure variable — but since Body is embedded
                // in RigidBody and others, and Inertia is called frequently, we
                // store the cached value on a private field via type assertion.
                // For simplicity in Phase 1, we recompute each time the flag is set.
                return inertia
        }
        // Recompute always for now — the cache invalidation logic needs a
        // stored field. Will optimize in Phase 5.
        return b.TotalInitialArea() * 2.0 * b.mass
}

// Circumference returns the total perimeter of all meshes' polygons.
func (b *Body) Circumference() float32 {
        if b.circumferenceNeedsUpdate {
                var res float32
                for _, m := range b.meshes {
                        res += m.Circumference()
                }
                b.circumferenceNeedsUpdate = false
                // Same caching note as Inertia
                return res
        }
        var res float32
        for _, m := range b.meshes {
                res += m.Circumference()
        }
        return res
}

// --- Setters (fluent, return *Body) ---

// SetPosition sets the body's world-space position. If withPreviousPosition
// is true (the default), prevPosition is also set, zeroing the implicit velocity.
// Matches QBody::SetPosition in qbody.h:387-397.
func (b *Body) SetPosition(v Vec2, withPreviousPosition ...bool) *Body {
        wpp := true
        if len(withPreviousPosition) > 0 {
                wpp = withPreviousPosition[0]
        }
        b.position = v
        if wpp {
                b.prevPosition = v
        }
        b.WakeUp()
        b.UpdateMeshTransforms()
        b.UpdateAABB()
        return b
}

// AddPosition adds a vector to the body's position.
func (b *Body) AddPosition(v Vec2, withPreviousPosition ...bool) *Body {
        return b.SetPosition(b.Position().Add(v), withPreviousPosition...)
}

// SetPreviousPosition sets the body's previous position (Verlet velocity source).
func (b *Body) SetPreviousPosition(v Vec2) *Body {
        b.prevPosition = v
        return b
}

// AddPreviousPosition adds a vector to the body's previous position.
func (b *Body) AddPreviousPosition(v Vec2) *Body {
        return b.SetPreviousPosition(b.PreviousPosition().Add(v))
}

// SetRotation sets the body's rotation in radians.
// Matches QBody::SetRotation in qbody.h:432-438.
func (b *Body) SetRotation(angleRadian float32, withPreviousRotation ...bool) *Body {
        wpr := true
        if len(withPreviousRotation) > 0 {
                wpr = withPreviousRotation[0]
        }
        b.rotation = angleRadian
        if wpr {
                b.prevRotation = angleRadian
        }
        b.WakeUp()
        b.UpdateMeshTransforms()
        return b
}

// SetRotationDegree sets the body's rotation in degrees.
func (b *Body) SetRotationDegree(degree float32, withPreviousRotation ...bool) *Body {
        return b.SetRotation(degree*(Pi/180.0), withPreviousRotation...)
}

// AddRotation adds to the body's rotation in radians.
func (b *Body) AddRotation(angleRadian float32, withPreviousRotation ...bool) *Body {
        return b.SetRotation(b.Rotation()+angleRadian, withPreviousRotation...)
}

// SetPreviousRotation sets the body's previous rotation.
func (b *Body) SetPreviousRotation(angleRadian float32) *Body {
        b.prevRotation = angleRadian
        return b
}

// AddPreviousRotation adds to the body's previous rotation.
func (b *Body) AddPreviousRotation(angleRadian float32) *Body {
        return b.SetPreviousRotation(b.PreviousRotation() + angleRadian)
}

// SetLayersBit sets the bitmask of layers this body is on.
func (b *Body) SetLayersBit(v int) *Body { b.layersBit = v; return b }

// SetCollidableLayersBit sets the bitmask of layers this body can collide with.
func (b *Body) SetCollidableLayersBit(v int) *Body { b.collidableLayersBit = v; return b }

// SetCanSleep controls whether the body is allowed to sleep.
func (b *Body) SetCanSleep(v bool) *Body { b.canSleep = v; return b }

// SetMode sets the body to dynamic or static.
func (b *Body) SetMode(m BodyMode) *Body { b.mode = m; return b }

// SetFriction sets the dynamic friction coefficient.
func (b *Body) SetFriction(v float32) *Body { b.friction = v; return b }

// SetStaticFriction sets the static friction coefficient.
func (b *Body) SetStaticFriction(v float32) *Body { b.staticFriction = v; return b }

// SetAirFriction sets the air friction (drag) coefficient.
func (b *Body) SetAirFriction(v float32) *Body { b.airFriction = v; return b }

// SetMass sets the body's mass.
func (b *Body) SetMass(v float32) *Body {
        b.mass = v
        b.inertiaNeedsUpdate = true
        return b
}

// SetRestitution sets the body's restitution (bounciness).
func (b *Body) SetRestitution(v float32) *Body { b.restitution = v; return b }

// SetEnabled enables or disables the body.
func (b *Body) SetEnabled(v bool) *Body { b.enabled = v; return b }

// SetVelocityLimit sets the maximum velocity (0 = unlimited).
func (b *Body) SetVelocityLimit(v float32) *Body { b.velocityLimit = v; return b }

// SetIntegratedVelocitiesEnabled controls whether Verlet integration runs.
func (b *Body) SetIntegratedVelocitiesEnabled(v bool) *Body {
        b.enableIntegratedVelocities = v
        return b
}

// SetCustomGravityEnabled controls whether a per-body gravity override is active.
func (b *Body) SetCustomGravityEnabled(v bool) *Body {
        b.enableCustomGravity = v
        return b
}

// SetCustomGravity sets the per-body gravity vector.
func (b *Body) SetCustomGravity(v Vec2) *Body {
        b.customGravity = v
        return b
}

// SetKinematic controls whether the body is kinematic. (Defined on RigidBody
// in C++; we expose it on Body for convenience.)
func (b *Body) SetKinematic(v bool) *Body { b.isKinematic = v; return b }

// SetAllowKinematicCollisions controls kinematic-kinematic collision response.
func (b *Body) SetAllowKinematicCollisions(v bool) *Body {
        b.allowKinematicCollisions = v
        return b
}

// --- Mesh operations ---

// AddMesh attaches a mesh to the body. Matches QBody::AddMesh in qbody.cpp:154-162.
func (b *Body) AddMesh(m *Mesh) *Body {
        b.meshes = append(b.meshes, m)
        m.ownerBody = b
        b.UpdateMeshTransforms()
        b.inertiaNeedsUpdate = true
        b.circumferenceNeedsUpdate = true
        m.UpdateCollisionBehavior()
        return b
}

// RemoveMeshAt removes the mesh at the given index.
func (b *Body) RemoveMeshAt(i int) *Body {
        b.meshes = append(b.meshes[:i], b.meshes[i+1:]...)
        b.inertiaNeedsUpdate = true
        b.circumferenceNeedsUpdate = true
        return b
}

// --- Sleeping ---

// WakeUp un-sleeps the body. Matches QBody::WakeUp in qbody.h:679-682.
func (b *Body) WakeUp() *Body {
        b.isSleeping = false
        return b
}

// --- Internal methods (called by World, Manifold, etc.) ---

// UpdateAABB recomputes the body's AABB from all particle positions.
// Matches QBody::UpdateAABB in qbody.cpp:195-225.
func (b *Body) UpdateAABB() {
        minX := MaxWorldSize
        minY := MaxWorldSize
        maxX := -MaxWorldSize
        maxY := -MaxWorldSize
        for _, mesh := range b.meshes {
                for _, p := range mesh.particles {
                        r := float32(0)
                        if p.Radius() > 0.5 {
                                r = p.Radius()
                        }
                        gp := p.GlobalPosition()
                        if gp.X-r < minX {
                                minX = gp.X - r
                        }
                        if gp.Y-r < minY {
                                minY = gp.Y - r
                        }
                        if gp.X+r > maxX {
                                maxX = gp.X + r
                        }
                        if gp.Y+r > maxY {
                                maxY = gp.Y + r
                        }
                }
        }
        b.aabb = AABB{
                Min: Vec2{X: minX, Y: minY},
                Max: Vec2{X: maxX, Y: maxY},
        }
}

// UpdateMeshTransforms applies the body's position and rotation to all
// mesh particles. Matches QBody::UpdateMeshTransforms in qbody.cpp:227-251.
//
// Critical: the prevGlobalPosition update differs by body type:
//   - RIGID: prev = current globalPosition (preserves velocity direction)
//   - SOFT/AREA: prev = new computed position (zeroes velocity for that step)
//
// This is the Verlet velocity mechanism for particles.
func (b *Body) UpdateMeshTransforms() {
        for _, mesh := range b.meshes {
                mesh.globalRotation = b.rotation + mesh.rotation
                rotVecUnit := AngleToUnitVector(mesh.globalRotation)
                mesh.globalPosition = b.position.Add(mesh.position.Rotated(b.rotation))
                for _, p := range mesh.particles {
                        originVec := p.Position()
                        nx := originVec.X*rotVecUnit.X - originVec.Y*rotVecUnit.Y
                        ny := originVec.Y*rotVecUnit.X + originVec.X*rotVecUnit.Y
                        newPos := mesh.globalPosition.Add(Vec2{X: nx, Y: ny})
                        if b.bodyType == BodyTypeRigid {
                                p.SetPreviousGlobalPosition(p.GlobalPosition())
                        } else {
                                p.SetPreviousGlobalPosition(newPos)
                        }
                        p.SetGlobalPosition(newPos)
                }
        }
}

// Update is the per-step integration hook. The base implementation just
// resets lazy collisions; RigidBody and SoftBody override.
// Matches QBody::Update in qbody.cpp:253-263.
func (b *Body) Update() {
        for _, mesh := range b.meshes {
                for _, p := range mesh.particles {
                        if p.IsLazy() {
                                p.ResetOneTimeCollisions()
                        }
                }
        }
}

// PostUpdate is called after all bodies have completed their Update step.
// Base implementation is a no-op; RigidBody and PlatformerBody override.
func (b *Body) PostUpdate() {}

// CanGiveCollisionResponseTo reports whether this body should receive
// collision responses from otherBody. Matches QBody::CanGiveCollisionResponseTo.
func (b *Body) CanGiveCollisionResponseTo(other *Body) bool {
        if other.mode == BodyModeStatic {
                return false
        }
        if other.isKinematic && b.isKinematic && !other.allowKinematicCollisions {
                return false
        }
        if b.mode != BodyModeStatic && other.isKinematic && !b.isKinematic {
                return false
        }
        return true
}

// ApplyForce applies an immediate force to the body. The base implementation
// is a no-op; RigidBody and SoftBody override.
func (b *Body) ApplyForce(force Vec2) *Body { return b }

// --- Static helpers ---

// CanCollide reports whether two bodies can collide based on their state
// and layer bits. Matches QBody::CanCollide in qbody.cpp:108-132.
func CanCollide(bodyA, bodyB *Body, checkBodiesAreEnabled bool) bool {
        if bodyA.world != bodyB.world {
                return false
        }
        if checkBodiesAreEnabled {
                if !bodyA.enabled || !bodyB.enabled {
                        return false
                }
        }
        // Static and sleeping bodies don't collide with each other
        if (bodyA.isSleeping || bodyA.mode == BodyModeStatic) &&
                (bodyB.isSleeping || bodyB.mode == BodyModeStatic) {
                return false
        }
        // Layer bits check
        if (bodyA.layersBit&bodyB.collidableLayersBit) == 0 &&
                (bodyB.layersBit&bodyA.collidableLayersBit) == 0 {
                return false
        }
        // Collision exceptions
        if bodyA.world != nil && bodyA.world.CheckCollisionException(bodyA, bodyB) {
                return false
        }
        return true
}

// OverlapWithCollidableLayersBit reports whether this body can collide
// with bodies on the given layers bitmask.
func (b *Body) OverlapWithCollidableLayersBit(layersBit int) bool {
        return (layersBit & b.collidableLayersBit) != 0
}

// OverlapWithLayersBit reports whether this body is on any of the given layers.
func (b *Body) OverlapWithLayersBit(layersBit int) bool {
        return (layersBit & b.layersBit) != 0
}

// ComputeFriction calculates the friction force for a collision.
// Matches QBody::ComputeFriction in qbody.cpp:81-106.
//
// Uses Coulomb friction: tangent = relativeVelocity projected onto the
// contact plane; if |jt| < penetration * staticFriction, use static
// friction, otherwise use dynamic friction.
func ComputeFriction(bodyA, bodyB *Body, normal Vec2, penetration float32, relativeVelocity Vec2) Vec2 {
        // tangent = relativeVelocity - (relativeVelocity · normal) * normal
        tangent := relativeVelocity.Sub(normal.Mul(relativeVelocity.Dot(normal)))
        tangent = tangent.Normalized()

        jt := relativeVelocity.Dot(tangent.Neg())

        dynamicFriction := bodyA.friction
        if bodyB.friction < dynamicFriction {
                dynamicFriction = bodyB.friction
        }
        sFriction := Sqrt(bodyA.staticFriction * bodyB.staticFriction)

        var frictionForce Vec2
        if Abs(jt) < penetration*sFriction {
                frictionForce = tangent.Mul(jt)
        } else {
                frictionForce = tangent.Mul(-penetration).Mul(dynamicFriction)
        }
        return frictionForce
}
