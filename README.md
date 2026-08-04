# quarkgo

A Go port of [QuarkPhysics](https://github.com/erayzesen/QuarkPhysics), a 2D physics engine for games.

## About This Port

This project is a complete Go language port of QuarkPhysics, a MIT-licensed 2D physics engine originally written in C++11 by Eray Zesen. The port preserves the engine's Verlet integration scheme, iterative constraint solver, and unified rigid/soft/area body model while exposing an idiomatic Go API.

### How It Was Made

This port was developed entirely by **GLM 5.2** (an AI model by Z.ai), working as an autonomous agent across a single session. The process followed a structured plan:

1. **Analysis phase** — GLM 5.2 cloned the C++ source repository, read every header and implementation file (~6,200 lines of C++ across 31 files), and produced a detailed technical analysis document covering the architecture, class hierarchy, simulation step pipeline, dependency graph, and porting risks.

2. **Planning phase** — Based on the analysis, GLM 5.2 created a phased execution guide with 6 phases, each with step-by-step instructions, acceptance criteria, and exit gates.

3. **Implementation phase** — GLM 5.2 executed the plan phase by phase:
   - **Phase 0** (Foundation): math primitives, contact pool, parity test harness, CI config
   - **Phase 1** (Rigid Body MVP): Particle, Mesh, Body, RigidBody, SAT collision, Manifold solver, SAP broadphase, World.Update core loop, Raycast
   - **Phase 2** (Soft Bodies): Spring, AngleConstraint, SoftBody with PBD, area-preserving, shape matching, polyline collisions, self-collisions
   - **Phase 3** (Joints + Serialization): Joint distance constraint, AreaBody sensors, .qmesh JSON I/O, pure-Go polypartition (Hertel-Mehlhorn)
   - **Phase 4** (Extensions): Spatial hashing broadphase, platformer character controller
   - **Phase 5** (Concurrency + Polish): Parallel narrowphase, benchmark suite, documentation

4. **Verification** — Each phase included automated tests (80 total, all passing) and the final port was verified with `go vet`, `go build`, `go test`, and `go test -race` (zero data races).

### Design Decisions

Key porting decisions made by GLM 5.2 (16 decisions documented in [DECISIONS.md](DECISIONS.md)):

- **float32 throughout** — preserves bit-for-bit parity with C++ `float` arithmetic
- **Per-World ContactPool** — replaces the C++ global static pool (enables future concurrency)
- **Single `physics` package** — mirrors C++ friend-class encapsulation without exported fields
- **No cgo** — pure Go, including the polypartition port
- **Struct tag + switch** for virtual dispatch — avoids interface overhead in the hot loop
- **PostUpdate function registry** — replaces C++ virtual method dispatch for PlatformerBody
- **Parallel narrowphase with serial Solve** — GetCollisions runs in goroutines (read-only), Solve stays serial

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
- **Parallel narrowphase** — optional goroutine-based collision detection (race-free)

## Quick Start

```bash
# Build and test
go test ./...

# Run benchmarks
go test -bench=. -run=^$ ./physics/

# Race detector
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

## Simulation Step

```go
// Advance the simulation by one step
world.Update()
```

Call `world.Update()` once per frame in your game loop.

## Package Layout

```
quarkgo/
├── physics/                    # Core engine (one package)
│   ├── vector.go               # Vec2 (2D float32 vector)
│   ├── aabb.go                 # AABB (axis-aligned bounding box)
│   ├── math32.go               # float32 math wrappers
│   ├── particle.go             # Particle (smallest building block)
│   ├── mesh.go                 # Mesh (shape + topology container)
│   ├── mesh_shapematching.go   # Shape matching helpers
│   ├── body.go                 # Body (base type)
│   ├── rigidbody.go            # RigidBody (Verlet integration)
│   ├── softbody.go             # SoftBody (mass-spring PBD)
│   ├── areabody.go             # AreaBody (sensor/trigger)
│   ├── world.go                # World (simulation manager)
│   ├── collision.go            # SAT collision detection
│   ├── collision_polyline.go   # Polyline collision (soft bodies)
│   ├── manifold.go             # Collision resolution
│   ├── broadphase.go           # BroadPhase interface + SAP
│   ├── broadphase_internal.go  # SAP + brute force pair generation
│   ├── joint.go                # Joint (distance constraint)
│   ├── spring.go               # Spring (particle distance constraint)
│   ├── angleconstraint.go      # AngleConstraint (3-particle)
│   ├── raycast.go              # Raycasting
│   ├── gizmo.go                # Debug visualization primitives
│   ├── pool.go                 # ContactPool (sync.Pool)
│   ├── contact.go              # Contact struct
│   ├── events.go               # CollisionInfo
│   ├── concurrency.go          # Parallel narrowphase
│   └── *_test.go               # Unit + integration tests
├── ext/
│   ├── spatialhash/            # Uniform-grid broadphase
│   └── platformer/             # Character controller
├── mesh/
│   ├── qmesh/                  # .qmesh JSON I/O
│   └── polypartition/          # Convex decomposition (Hertel-Mehlhorn)
├── tests/
│   └── parity/                 # Golden-file parity test harness
├── scripts/
│   ├── check_no_ebitengine_in_physics.sh
│   └── check_no_cgo_in_core.sh
├── .github/workflows/ci.yml    # CI configuration
├── DECISIONS.md                # 16 porting decisions with rationale
├── LICENSE                     # MIT
└── README.md                   # This file
```

## Project Stats

| Metric | Value |
|---|---|
| Lines of Go | ~10,300 |
| Go files | 49 |
| Packages | 6 |
| Tests | 80 (all passing) |
| Decisions logged | 16 |
| External dependencies | 0 (pure Go stdlib) |
| cgo | none |
| Data races | none |

## C++ → Go File Mapping

| C++ source | Go file | Phase |
|---|---|---|
| `qvector.h/cpp` | `physics/vector.go` | 0 |
| `qaabb.h/cpp` | `physics/aabb.go` | 0 |
| `qmath_utils.h` | `physics/mathutils.go` | 0 |
| `qobjectpool.h` | `physics/pool.go` | 0 |
| `qparticle.h/cpp` | `physics/particle.go` | 1 |
| `qmesh.h/cpp` | `physics/mesh.go` | 1-2 |
| `qbody.h/cpp` | `physics/body.go` | 1 |
| `qrigidbody.h/cpp` | `physics/rigidbody.go` | 1 |
| `qsoftbody.h/cpp` | `physics/softbody.go` | 2 |
| `qareabody.h/cpp` | `physics/areabody.go` | 3 |
| `qworld.h/cpp` | `physics/world.go` | 1-3 |
| `qcollision.h/cpp` | `physics/collision.go` | 1-2 |
| `qmanifold.h/cpp` | `physics/manifold.go` | 1 |
| `qbroadphase.h/cpp` | `physics/broadphase.go` | 1 |
| `qjoint.h/cpp` | `physics/joint.go` | 3 |
| `qspring.h/cpp` | `physics/spring.go` | 2 |
| `qangleconstraint.h/cpp` | `physics/angleconstraint.go` | 2 |
| `qraycast.h/cpp` | `physics/raycast.go` | 3 |
| `qgizmos.h/cpp` | `physics/gizmo.go` | 1 |
| `extensions/qspatialhashing.*` | `ext/spatialhash/spatialhash.go` | 4 |
| `extensions/qplatformerbody.*` | `ext/platformer/platformer.go` | 4 |
| `json/json.hpp` | `encoding/json` (stdlib) | 3 |
| `polypartition/*` | `mesh/polypartition/polypartition.go` | 3 |

## Examples (Ebitengine)

The `examples/` directory contains interactive demos built with [Ebitengine](https://github.com/hajimehoshi/ebiten) (a pure-Go game framework). The examples are in a **separate Go module** so the core physics package stays dependency-free.

### Prerequisites

Ebitengine requires CGO on macOS and a C compiler on Linux:

```bash
# Debian/Ubuntu
sudo apt install gcc libc6-dev

# Arch Linux
sudo pacman -S gcc

# macOS (Xcode command line tools)
xcode-select --install
```

### Running the Examples

```bash
# Clone the repo
git clone https://github.com/Vmarcelo49/quarkgo.git
cd quarkgo/examples

# Download dependencies
go mod tidy

# Run any example:
go run ./01_mixed_bodies/      # Mixed rigid + soft bodies
go run ./02_soft_bodies/       # Soft body variants
go run ./03_joints/            # Joint types (pin, distance, groove)
go run ./04_frictions/         # Friction on angled platforms
go run ./05_raycasts/          # Raycast fan
go run ./06_platformer/        # Platformer character controller
go run ./07_benchmark_boxes/   # 200 boxes stress test
go run ./08_benchmark_boxes_2/ # 600 boxes heavy stress test
go run ./09_blobs/             # Spawn soft body blobs
go run ./99_testing/           # Zero-gravity sandbox
```

### Available Examples

| # | Example | Description | Controls |
|---|---------|-------------|----------|
| 01 | Mixed Bodies | Random rigid primitives + soft body "QUARK" letters with shape matching and self-collisions | Click to drag |
| 02 | Soft Bodies | Multiple soft body variants: gridded rect, polygon, polar grid, pressure volume, shape matching | Click to drag |
| 03 | Joints | Pin joints, distance chains, spring joints, and groove joints (pull-only) | Click to drag |
| 04 | Frictions | Friction, static friction, and air friction on 6.2° angled platforms + box stack | Click to drag |
| 05 | Raycasts | 90-ray fan from mouse position, rotating slowly, hitting random bodies | Move mouse |
| 06 | Platformer | Full character controller: walk, jump, wall-jump, slopes, moving platform, collectible coins | Arrows/WASD + Space |
| 07 | Benchmark Boxes | 200 dynamic boxes stacked on a floor (stress test) | Click to drag |
| 08 | Benchmark Boxes 2 | 600 small boxes, low gravity, sleeping disabled (heavy stress test) | Click to drag |
| 09 | Blobs | Spawn pressure-preserving soft body blobs at the mouse cursor | Hold Space to spawn |
| 99 | Testing | Zero-gravity sandbox with bouncing circles | Click to drag |

### Controls (Example 06: Platformer)

| Key | Action |
|-----|--------|
| ← / A | Walk left |
| → / D | Walk right |
| Space / ↑ / W | Jump (press for variable height, double-jump supported) |
| Click + Drag | Grab and drag any body (all examples) |

### Controls (Example 09: Blobs)

| Key | Action |
|-----|--------|
| Space | Spawn a soft body blob at mouse position |
| Click + Drag | Grab and drag existing blobs |

### Writing Your Own Example

```go
package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type MyScene struct {
	*common.Scene
}

func NewMyScene() *MyScene {
	scene := common.NewScene(800, 600)

	// Add a floor
	floor := physics.NewRigidBody()
	floor.AddMesh(physics.NewRectMesh(physics.Vec2{X: 780, Y: 20}, physics.Vec2Zero(), physics.Vec2Zero()))
	floor.SetPosition(physics.Vec2{X: 400, Y: 570})
	floor.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(floor)

	// Add a falling box
	box := physics.NewRigidBody()
	box.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	box.SetPosition(physics.Vec2{X: 400, Y: 100})
	scene.World.AddRigidBody(box)

	return &MyScene{Scene: scene}
}

func main() {
	ebiten.SetWindowSize(800, 600)
	ebiten.SetWindowTitle("My Physics Demo")
	ebiten.RunGame(NewMyScene())
}
```

The `common.Scene` struct handles the Ebitengine `Update`/`Draw`/`Layout` interface — you just set up the physics world and it runs automatically.


## License

MIT, matching the upstream QuarkPhysics license. See [LICENSE](LICENSE).

## Credits

- **Original engine**: [Eray Zesen](https://github.com/erayzesen) — QuarkPhysics C++ engine
- **Go port**: Built by [GLM 5.2](https://z.ai) (AI model by Z.ai) as an autonomous development task
- **Third-party ports included**:
  - [polypartition](https://github.com/ivanfratric/polypartition) by Ivan Fratric — ported to pure Go (Hertel-Mehlhorn algorithm)
  - [nlohmann/json](https://github.com/nlohmann/json) — replaced with Go stdlib `encoding/json`
