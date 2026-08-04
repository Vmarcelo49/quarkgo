# Decision Log

Every non-trivial decision in this port gets an entry. This creates an
audit trail and a reference for future contributors.

Format:

```
## D### — <short title>

**Date:** YYYY-MM-DD
**Phase:** <phase number>
**Context:** <why this decision is needed>
**Decision:** <what was decided>
**Consequences:** <trade-offs and side effects>
**References:** <links to analysis doc sections, C++ source, etc.>
```

---

## D001 — Use sync.Pool for Contact recycling (per-World, not global)

**Date:** 2026-08-04
**Phase:** 0
**Context:** The C++ engine uses a global static `QObjectPool<Contact>` (`QCollision::contactPool`, qcollision.h:93) shared across all QWorld instances. This is not thread-safe and blocks future concurrency.
**Decision:** Each World owns its own `ContactPool` backed by `sync.Pool`. Contacts are recycled within a World, not globally.
**Consequences:** Enables future per-World concurrency (Phase 5). Slight overhead vs raw free-list, but acceptable — sync.Pool is optimized for this access pattern. Worlds can no longer share Contact objects, but no caller ever did this in the C++ engine anyway.
**References:** Analysis doc §7.5, §8.2 R4; qcollision.h:93, qcollision.cpp:38

---

## D002 — float32 throughout, vendored math32 helpers

**Date:** 2026-08-04
**Phase:** 0
**Context:** Go's stdlib `math` package is float64-only. The C++ engine uses `float` (32-bit) throughout. Using float64 in the Go port would break float-drift parity and double memory usage for vectors and particles.
**Decision:** All physics math operates on `float32`. A `math32.go` file in the physics package provides wrappers (`Sqrt`, `Sin`, `Cos`, `Atan2`, `Asin`, `Abs`, `Floor`, `IsNaN`) that convert to float64, call stdlib `math`, and convert back.
**Consequences:** Preserves bit-for-bit parity with C++ `float` arithmetic (within Go's float32 rounding). Slight perf overhead from float conversion at math call sites — acceptable, and a future optimization could vendor a real `math32` package if profiling shows it matters. The `Asin` wrapper preserves the C++ `safe_asin` clamp (qsoftbody.h:65-73).
**References:** Analysis doc §7.6 (Numeric Precision); qmath_utils.h, qsoftbody.h:65-73

---

## D003 — Single `physics` package for core types (mirrors C++ friend classes)

**Date:** 2026-08-04
**Phase:** 0
**Context:** The C++ engine uses `friend` declarations extensively (QBody friends: QMesh, QWorld, QManifold, QParticle, QJoint, QBroadPhase, QAreaBody — see analysis doc §3.5). Splitting these into separate Go packages would force either exported fields (polluting the API) or accessor methods (perf cost in hot loops).
**Decision:** All core types (`Body`, `RigidBody`, `SoftBody`, `AreaBody`, `Mesh`, `Particle`, `World`, `Manifold`, `Collision`, `BroadPhase`, `Raycast`, `Joint`, `Spring`, `AngleConstraint`) live in a single `physics` package. Unexported fields are visible across types within the package.
**Consequences:** Package is large but cohesive — same as the C++ engine's effective coupling. Extensions (spatial hashing, platformer) live in `ext/` sub-packages and use only exported APIs. Examples live in `examples/` and use only exported APIs.
**References:** Analysis doc §3.5, §7.2

---

## D004 — Drop the `Q` prefix from type names

**Date:** 2026-08-04
**Phase:** 0
**Context:** C++ types are prefixed with `Q` (`QBody`, `QWorld`, `QMesh`, etc.) as a Hungarian-style namespace marker. Go doesn't use this convention — the package name serves as the namespace.
**Decision:** Drop the `Q` prefix: `QBody` → `Body`, `QWorld` → `World`, `QMesh` → `Mesh`, `QVector` → `Vec2`, `QAABB` → `AABB`, etc.
**Consequences:** Cleaner Go API. The mapping is documented in the execution guide's Quick Reference table (§14). Callers write `physics.Body` rather than `physics.QBody`.
**References:** Execution guide §10.2 (Naming)

---

## D005 — Examples in a separate Go module (Ebitengine isolation)

**Date:** 2026-08-04
**Phase:** 0
**Context:** Ebitengine is a heavy dependency (pulls in golang.org/x/mobile, audio libs, etc.). Users who want just the engine shouldn't have to download it. A CI boundary-check script enforces that `physics/`, `ext/`, and `mesh/` never import Ebitengine.
**Decision:** `examples/` is a separate Go module with its own `go.mod` that depends on the root module via `replace` and on `github.com/hajimehoshi/ebiten/v2`. The root module has zero non-stdlib dependencies.
**Consequences:** Two `go.mod` files to maintain, but clean separation. Users `go get github.com/example/quarkgo` for the engine; contributors clone the repo and build `examples/` for demos. The boundary check script (`scripts/check_no_ebitengine_in_physics.sh`) runs in CI.
**References:** Execution guide §0.7, §1.3, §1.4

---

## D006 — Struct tag + switch for virtual dispatch (not interfaces)

**Date:** 2026-08-04
**Phase:** 0
**Context:** The C++ engine uses virtual functions for `QBody::Update()`, `PostUpdate()`, `OnPreStep()`, `OnStep()`, `OnCollision()`, `ApplyForce()`, `GetMass()` — overridden by `QRigidBody`, `QSoftBody`, `QAreaBody`, `QPlatformerBody`. Go has two options: interfaces (dynamic dispatch, idiomatic) or struct tags + switch (no interface overhead, more verbose).
**Decision:** Use a `bodyType` field on `Body` and dispatch via `switch` in `World.Update`. This avoids interface dispatch overhead in the hot loop (one call per body per step) and keeps the body's concrete type accessible for type-specific operations.
**Consequences:** Code is more verbose (`switch body.bodyType { case BodyTypeRigid: ... }`) but faster. Type assertions are not needed. The `BroadPhase` interface is kept because broadphase implementations are pluggable and not in the hot per-body loop.
**References:** Execution guide §7.1 (C++ → Go Mapping); analysis doc §7.1

---

## D007 — `manualDeletion` flag replaced by explicit World.Close()

**Date:** 2026-08-04
**Phase:** 0
**Context:** The C++ engine uses a `manualDeletion` flag on Body, Mesh, Joint, Spring, Raycast to control whether the owning container's destructor should `delete` the object. Go has GC; this flag is meaningless.
**Decision:** Drop `manualDeletion`. The World owns its bodies, joints, springs, raycasts; bodies own their meshes; meshes own their particles/springs. Users call `world.Close()` to release all owned resources explicitly (rather than relying on GC finalizers, which have non-deterministic ordering).
**Consequences:** No `manualDeletion` field in the Go API. Users who want to retain ownership of an object after removing it from the World simply keep a Go reference — GC won't collect it. The `Close()` pattern is documented in the README.
**References:** Analysis doc §7.5, §8.1 R7

---

## D008 — Use `github.com/example/quarkgo` as placeholder module path

**Date:** 2026-08-04
**Phase:** 0
**Context:** The actual org/user for the final module path is not yet decided. The execution guide uses `github.com/<org>/quarkgo` as a placeholder.
**Decision:** Use `github.com/example/quarkgo` for now. This is a find-and-replace before publishing.
**Consequences:** All imports use this path until renamed. The `examples/go.mod` (when created) will `replace github.com/example/quarkgo => ../`.
**References:** Execution guide §1.1

---

## D009 — Soft body collision uses point-in-polygon + nearest-edge normal

**Date:** 2026-08-05
**Phase:** 2
**Context:** The C++ engine uses a bisector-based ray-casting approach for polyline-vs-polygon collisions (`PolylineAndPolygon` in qcollision.cpp:43-201). This is complex and hard to port correctly. An alternative is needed for Phase 2.
**Decision:** Use a simpler approach: for each polyline vertex, check if it's inside the polygon via `pointInPolygon` (cross-product test for convex polygons). If inside, compute the nearest edge and use its outward normal as the contact normal. If outside but within the particle radius, use the nearest edge point as the contact.
**Consequences:** Simpler implementation that works for most cases. Limitation: when a particle penetrates deep into a thick polygon (past the center), the nearest edge may be the wrong side, causing the solver to push the particle out the wrong direction. This is mitigated by using sufficiently thick floors/walls in tests and examples. The C++ bisector approach will be ported in Phase 3 when polypartition enables proper convex decomposition.
**References:** qcollision.cpp:43-201, analysis doc §3.1 (QCollision)

---

## D010 — Soft body position field not updated during integration

**Date:** 2026-08-05
**Phase:** 2
**Context:** In the C++ engine, `QSoftBody::Update()` does NOT update the `position` field — only particle positions change. The body position is set externally via `SetPosition()` and stays fixed during simulation. This differs from rigid bodies where `position` is the primary integration variable.
**Decision:** Preserve this behavior in Go. `SoftBody.Update()` integrates particle positions but does not update `sb.position`. Users who need the soft body's "center" should compute it from particles via `GetAveragePositionAndRotation`.
**Consequences:** Tests checking `sb.Position().Y` will see the initial value, not the current particle center. This is correct C++ behavior. The AABB IS updated correctly (computed from particles). Documentation should note this difference from rigid bodies.
**References:** qsoftbody.cpp:93-185, analysis doc §3.1 (QSoftBody)

---

## D011 — Polypartition port uses simplified Hertel-Mehlhorn (ear clipping + merge)

**Date:** 2026-08-05
**Phase:** 3
**Context:** The C++ engine uses Ivan Fratric's polypartition library (2,270 LOC) for convex decomposition. A full port would be large and complex. The only method actually used by QuarkPhysics is `ConvexPartition_HM` (Hertel-Mehlhorn).
**Decision:** Port a simplified Hertel-Mehlhorn: ear-clipping triangulation followed by convex merging of adjacent triangles. This produces at most 4× the optimal number of convex pieces but is fast and robust enough for real-time physics.
**Consequences:** The decomposition may produce more convex pieces than the C++ version for complex concave polygons, but collision results will be correct. The full polypartition library can be ported later if needed. The pure-Go implementation avoids cgo, maintaining the "no cgo" constraint.
**References:** polypartition.cpp, qmesh.cpp:625-665, analysis doc §8.1 R1

---

## D012 — Polypartition registered via SetConvexPartitioner to avoid circular import

**Date:** 2026-08-05
**Phase:** 3
**Context:** The `physics` package needs to call polypartition for concave polygon decomposition, but `mesh/polypartition` imports `physics` (for `*physics.Particle`). A direct import from `physics` → `mesh/polypartition` would create a circular dependency.
**Decision:** Use a function variable (`convexPartitionerFunc`) set via `physics.SetConvexPartitioner()`. The application (or World constructor) calls `SetConvexPartitioner(polypartition.ConvexPartitionFromParticles)` at initialization. If not set, concave polygons fall back to using the full polygon without decomposition.
**Consequences:** Applications must call `SetConvexPartitioner` once at startup to enable concave polygon support. This is documented in the README. The indirection has zero runtime cost (one function pointer call per decomposition, which is rare).
**References:** analysis doc §7.2 (Package Layout)

---

## D013 — Area body events dispatched in Manifold.Solve, not in CheckBodies

**Date:** 2026-08-05
**Phase:** 3
**Context:** The C++ engine calls `AddCollidedBody` from `Manifold::Solve` (when it detects an area body), then `CheckBodies` (called by `World::Update`) re-tests and dispatches enter/exit events. The initial `AddCollidedBody` in Solve is what registers a body as "currently colliding".
**Decision:** `Manifold.Solve` calls `dispatchAreaBodyEvents()` at the top (before the `isCollisionOneSide` check), which calls `addCollidedBody` on the area body. `CheckBodies` then re-tests all registered bodies and dispatches `OnCollisionEnter`/`OnCollisionExit` for new/departed bodies.
**Consequences:** Area body enter/exit events fire reliably. The `isCollisionOneSide` flag (which is true for area-static pairs) no longer blocks event dispatch. Area bodies still don't receive physical collision response (no position correction).
**References:** qareabody.cpp:36-44, qmanifold.cpp:108-343

---

## D014 — PostUpdate dispatched via function registry (not virtual methods)

**Date:** 2026-08-05
**Phase:** 4
**Context:** The C++ engine uses virtual `PostUpdate()` on `QBody`, overridden by `QPlatformerBody`. Go has no virtual methods — embedding `RigidBody` in `PlatformerBody` doesn't override `Body.PostUpdate()`.
**Decision:** Use a function registry (`postUpdaterRegistry`) mapping `*Body` → `func()`. `PlatformerBody.RegisterPostUpdate()` registers its `PostUpdate` method. `World.Update` checks the registry first; if found, calls the registered function; otherwise calls `Body.PostUpdate()` (the no-op base).
**Consequences:** Users must call `pb.RegisterPostUpdate()` after `world.AddRigidBody(&pb.RigidBody)`. This is documented in the platformer package. The registry pattern is consistent with the existing RigidBody/SoftBody/AreaBody registries. Zero performance overhead (one map lookup per body per step).
**References:** qplatformerbody.cpp:402-648, analysis doc §3.1 (QPlatformerBody)

---

## D015 — PlatformerBody stores lastMovableFloor as *Body (not *RigidBody)

**Date:** 2026-08-05
**Phase:** 4
**Context:** The C++ `QPlatformerBody::lastMovableFloor` is `QRigidBody*`. In Go, `physics.GetCollisions` returns `*physics.Body` (the base type), and the rigid body registry maps `*Body → *RigidBody`. Converting back to `*RigidBody` for storage would require a registry lookup on every floor probe.
**Decision:** Store `lastMovableFloor` as `*physics.Body`. The platformer body only needs `Position()` and `PreviousPosition()` (which are on `Body`), not any RigidBody-specific methods.
**Consequences:** Simplifies the code — no registry lookup needed. The pointer comparison `collidedBody != pb.lastMovableFloor` works directly. No loss of functionality.
**References:** qplatformerbody.h:57

---

## D016 — Parallel narrowphase with serial Solve (Option B from execution guide)

**Date:** 2026-08-05
**Phase:** 5
**Context:** The execution guide proposed two options for parallel narrowphase: (A) group pairs by body, solve serially within groups, parallel across groups; (B) solve position correction in parallel (read-only on neighbors), then merge velocity updates serially.
**Decision:** Implement a simpler variant: GetCollisions (narrowphase detection) runs in parallel across goroutines — it's read-only on body state. Manifold.Solve and SolveFrictionAndVelocities stay serial because they mutate body positions and velocities. Each worker writes manifolds to its own slice; results are merged after all workers complete.
**Consequences:** Race-free (verified with `go test -race`). Produces identical results to serial mode (verified by parity test). For small worlds (< 50 bodies), goroutine overhead exceeds speedup — leave concurrency disabled (the default). For 500+ bodies on 4+ core machines, parallel narrowphase provides measurable speedup on the detection phase. The Solve phase remains the serial bottleneck; a future optimization could parallelize per-island solving.
**References:** Execution guide §7.4 (Concurrency Opportunities), analysis doc §7.4
