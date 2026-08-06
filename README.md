# quarkgo

A Go port of [QuarkPhysics](https://github.com/erayzesen/QuarkPhysics), a 2D physics engine for games.

## Status

Some logic still does not match the original C++ logic, we are still in active development.

## Features

- **Rigid bodies** — convex polygons, circles, rectangles; Verlet integration; kinematic mode; collision response with friction and restitution
- **Soft bodies** — mass-spring model with PBD; area-preserving; shape matching; self-collisions; internal springs
- **Area bodies** — sensor/trigger volumes; `OnCollisionEnter`/`OnCollisionExit` events; gravity-free zones; linear force application
- **Joints** — distance constraints with balance, groove mode (pull-only), pin joints, world-space anchors
- **Springs** — particle-level distance constraints with distance limits and accumulated-force pipeline
- **Angle constraints** — 3-particle angle limits with wrap-around handling
- **Raycasting** — instance-based auto-update and static one-shot queries; AABB broadphase filter; layer masks
- **Broadphase** — built-in Sweep-and-Prune; pluggable interface; spatial hashing extension
- **Platformer body** — walk, jump (variable height, multi-jump, wall jump), slope walking, moving-platform snap
- **Serialization** — `.qmesh` JSON format for mesh loading/saving
- **Concave decomposition** — pure-Go Hertel-Mehlhorn algorithm (ear clipping + convex merge)
- **Parallel narrowphase** — optional goroutine-based collision detection (Phase 5)

## Quick Start

```bash
# Build and test the core engine
go test ./...

# Run benchmarks
go test -bench=. -run=^$ ./physics/

# Run with race detector
go test -race ./...
```

## Usage

### Creating a World

```go
import "github.com/Vmarcelo49/quarkgo/physics"

world := physics.NewWorld(
    physics.WithGravity(physics.Vec2{X: 0, Y: 0.2}),
    physics.WithIterations(4),
)
```

### Rigid Body

```go
box := physics.NewRigidBody()
box.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
box.SetPosition(physics.Vec2{X: 100, Y: 0})
world.AddRigidBody(box)
```

### Static Body (Floor)

```go
floor := physics.NewRigidBody()
floor.AddMesh(physics.NewRectMesh(physics.Vec2{X: 500, Y: 20}, physics.Vec2Zero(), physics.Vec2Zero()))
floor.SetPosition(physics.Vec2{X: 250, Y: 400})
floor.SetMode(physics.BodyModeStatic)
world.AddRigidBody(floor)
```

### Soft Body

```go
sb := physics.NewSoftBody()
sb.AddMesh(physics.NewPolygonMesh(16, 6, physics.Vec2Zero(), -1))
sb.SetPosition(physics.Vec2{X: 100, Y: 0})
sb.SetAreaPreservingEnabled(true)
sb.SetShapeMatchingEnabled(true, false)
world.AddSoftBody(sb)
```

### Area Body (Sensor)

```go
area := physics.NewAreaBody()
area.AddMesh(physics.NewCircleMesh(30, physics.Vec2Zero()))
area.SetPosition(physics.Vec2{X: 100, Y: 100})
area.OnCollisionEnter = func(ab *physics.AreaBody, b *physics.Body) {
    fmt.Println("Body entered area!")
}
world.AddAreaBody(area)
```

### Joint

```go
joint := physics.NewJoint(bodyA, anchorA, anchorB, bodyB)
joint.SetLength(50)
joint.SetRigidity(0.8)
world.AddJoint(joint)
```

### Platformer Body

```go
import "github.com/Vmarcelo49/quarkgo/ext/platformer"

player := platformer.New()
player.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
player.SetPosition(physics.Vec2{X: 100, Y: 300})
world.AddRigidBody(&player.RigidBody)
player.RegisterPostUpdate() // Required for PostUpdate dispatch

// In your game loop:
player.Walk(1)  // walk right
player.Jump(5.0, false)
player.ReleaseJump()
```

### Parallel Narrowphase

```go
world := physics.NewWorld(
    physics.WithGravity(physics.Vec2{X: 0, Y: 0.2}),
    physics.WithConcurrency(physics.ConcurrencyConfig{
        Enabled:    true,
        NumWorkers: 0, // 0 = runtime.NumCPU()
    }),
)
```

### Loading .qmesh Files

```go
import "github.com/Vmarcelo49/quarkgo/mesh/qmesh"

meshes, err := qmesh.LoadFile("path/to/mesh.qmesh")
if err != nil {
    panic(err)
}
for _, md := range meshes {
    body.AddMesh(physics.NewMeshFromData(md, true, true))
}
```

### Concave Polygon Decomposition

```go
import "github.com/Vmarcelo49/quarkgo/mesh/polypartition"

// Register once at startup
physics.SetConvexPartitioner(polypartition.ConvexPartitionFromParticles)
```

### Spatial Hashing Broadphase

```go
import "github.com/Vmarcelo49/quarkgo/ext/spatialhash"

world := physics.NewWorld(
    physics.WithGravity(physics.Vec2{X: 0, Y: 0.2}),
    physics.WithBroadphaseImpl(spatialhash.New(128.0)),
)
```

### Event Listeners

```go
body.OnPreStep = func(b *physics.Body) {
    // Called before each physics step
}

body.OnStep = func(b *physics.Body) {
    // Called after each physics step
}

body.OnCollision = func(b *physics.Body, info physics.CollisionInfo) bool {
    // Return false to ignore this collision
    return true
}
```

### Raycasting

```go
// One-shot raycast
contacts := physics.RaycastTo(world, rayPos, rayVec, 1, false)
for _, c := range contacts {
    fmt.Printf("Hit %v at (%.1f, %.1f)\n", c.Body, c.Position.X, c.Position.Y)
}

// Registered raycast (auto-updated each step)
ray := physics.NewRaycast(rayPos, rayVec, false)
world.AddRaycast(ray)
// After world.Update():
for _, c := range ray.Contacts() {
    // ...
}
```

## Simulation Step

```go
// Advance the simulation by one step
world.Update()
```

Call `world.Update()` once per frame in your game loop.

## Package Layout

- `physics/` — core engine (one package, mirrors C++ friend-class encapsulation)
- `ext/spatialhash/` — uniform-grid broadphase
- `ext/platformer/` — character controller for platformer games
- `mesh/qmesh/` — `.qmesh` JSON I/O
- `mesh/polypartition/` — pure-Go convex decomposition (Hertel-Mehlhorn)
- `tests/parity/` — golden-file parity test harness
- `tests/benchmark/` — performance benchmarks

## Design Decisions

Key porting decisions are documented in [DECISIONS.md](DECISIONS.md). Notable choices:

- **float32 throughout** — preserves bit-for-bit parity with C++ `float` arithmetic
- **Per-World ContactPool** — replaces the C++ global static pool (enables future concurrency)
- **Single `physics` package** — mirrors C++ friend-class encapsulation without exported fields
- **No cgo** — pure Go, including the polypartition port
- **Struct tag + switch** for virtual dispatch — avoids interface overhead in the hot loop

## Performance

Benchmark results (Intel Xeon, 2 cores):

| Scenario | Bodies | ns/op |
|---|---|---|
| Rigid bodies stacking | 10 | ~5,500 |
| Rigid bodies stacking | 50 | ~69,000 |
| Rigid bodies stacking | 100 | ~345,000 |
| Rigid bodies stacking | 500 | ~668,000 |
| Soft bodies | 5 | ~7,200 |
| Soft bodies | 20 | ~40,000 |
| Soft bodies | 50 | ~153,000 |

The engine targets 60 FPS for 500+ rigid bodies on modern hardware.

## License

MIT, matching the upstream QuarkPhysics license. See [LICENSE](LICENSE).
