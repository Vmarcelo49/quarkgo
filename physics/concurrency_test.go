package physics

import (
	"testing"

	"github.com/chewxy/math32"
)

// TestParallelNarrowphaseParity verifies that parallel narrowphase produces
// the same simulation results as serial narrowphase.
func TestParallelNarrowphaseParity(t *testing.T) {
	// Run the same simulation twice: once serial, once parallel.
	// Compare final body positions.

	runSim := func(parallel bool, steps int) []Vec2 {
		world := NewWorld(
			WithGravity(Vec2{X: 0, Y: 0.2}),
			WithIterations(4),
		)
		if parallel {
			world.concurrency = ConcurrencyConfig{Enabled: true, NumWorkers: 4}
		}

		// Floor
		floor := NewRigidBody()
		floor.AddMesh(NewRectMesh(Vec2{X: 500, Y: 20}, Vec2Zero(), Vec2Zero()))
		floor.SetPosition(Vec2{X: 250, Y: 400})
		floor.SetMode(BodyModeStatic)
		world.AddRigidBody(floor)

		// 20 falling boxes
		for i := range 20 {
			box := NewRigidBody()
			box.AddMesh(NewRectMesh(Vec2{X: 16, Y: 16}, Vec2Zero(), Vec2Zero()))
			box.SetPosition(Vec2{X: float32(i*25 + 10), Y: float32(i % 3 * 50)})
			world.AddRigidBody(box)
		}

		for range steps {
			world.Update()
		}

		positions := make([]Vec2, 20)
		for i := 1; i <= 20; i++ {
			positions[i-1] = world.bodies[i].Position()
		}
		return positions
	}

	serial := runSim(false, 50)
	parallel := runSim(true, 50)

	// Compare — allow small float differences due to goroutine scheduling
	// (shouldn't affect deterministic Verlet integration, but be lenient)
	tol := float32(0.1)
	for i := range serial {
		if math32.Abs(serial[i].X-parallel[i].X) > tol || math32.Abs(serial[i].Y-parallel[i].Y) > tol {
			t.Errorf("body %d position mismatch:\n  serial:   (%.4f, %.4f)\n  parallel: (%.4f, %.4f)",
				i, serial[i].X, serial[i].Y, parallel[i].X, parallel[i].Y)
		}
	}
}

// TestConcurrencyConfigDefaults verifies default worker count.
func TestConcurrencyConfigDefaults(t *testing.T) {
	cfg := ConcurrencyConfig{Enabled: true}
	workers := cfg.numWorkers()
	if workers < 1 {
		t.Errorf("default workers = %d, should be >= 1", workers)
	}

	cfg2 := ConcurrencyConfig{Enabled: true, NumWorkers: 8}
	if cfg2.numWorkers() != 8 {
		t.Errorf("workers = %d, want 8", cfg2.numWorkers())
	}
}

// TestParallelNarrowphaseWithSoftBodies verifies parallel narrowphase works
// with mixed rigid + soft bodies.
func TestParallelNarrowphaseWithSoftBodies(t *testing.T) {
	world := NewWorld(
		WithGravity(Vec2{X: 0, Y: 0.2}),
		WithIterations(4),
		WithConcurrency(ConcurrencyConfig{Enabled: true, NumWorkers: 2}),
	)

	// Floor
	floor := NewRigidBody()
	floor.AddMesh(NewRectMesh(Vec2{X: 500, Y: 100}, Vec2Zero(), Vec2Zero()))
	floor.SetPosition(Vec2{X: 250, Y: 450})
	floor.SetMode(BodyModeStatic)
	world.AddRigidBody(floor)

	// Rigid body
	box := NewRigidBody()
	box.AddMesh(NewRectMesh(Vec2{X: 32, Y: 32}, Vec2Zero(), Vec2Zero()))
	box.SetPosition(Vec2{X: 100, Y: 100})
	world.AddRigidBody(box)

	// Soft body
	sb := NewSoftBody()
	sb.AddMesh(NewPolygonMesh(16, 6, Vec2Zero(), -1))
	sb.SetPosition(Vec2{X: 200, Y: 100})
	world.AddSoftBody(sb)

	// Step — should not crash or race
	for range 50 {
		world.Update()
	}

	// Both bodies should have fallen
	if box.Position().Y < 200 {
		t.Errorf("rigid body didn't fall: Y=%f", box.Position().Y)
	}
	mesh := sb.Meshes()[0]
	if mesh.ParticleAt(0).GlobalPosition().Y < 200 {
		t.Errorf("soft body didn't fall: Y=%f", mesh.ParticleAt(0).GlobalPosition().Y)
	}
	t.Logf("rigid Y=%.1f, soft particle Y=%.1f", box.Position().Y, mesh.ParticleAt(0).GlobalPosition().Y)
}
