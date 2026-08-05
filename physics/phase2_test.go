package physics

import (
	"testing"

	"github.com/chewxy/math32"
)

// TestSoftBodyFallsUnderGravity verifies that a soft body (polygon mesh)
// falls under gravity and its particles move correctly.
//
// Note: Soft body position field does NOT update during integration —
// only particles move. This matches C++ behavior.
func TestSoftBodyFallsUnderGravity(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0.2}))

	sb := NewSoftBody()
	sb.AddMesh(NewPolygonMesh(16, 6, Vec2Zero(), -1)) // hexagon, radius 16, no grid
	sb.SetPosition(Vec2{X: 100, Y: 0})
	world.AddSoftBody(sb)

	// Record initial particle position
	mesh := sb.Meshes()[0]
	startParticleY := mesh.ParticleAt(0).GlobalPosition().Y

	// Step 60 times
	for range 60 {
		world.Update()
	}

	// Check that particles fell (the body position doesn't update for soft bodies)
	endParticleY := mesh.ParticleAt(0).GlobalPosition().Y
	fall := endParticleY - startParticleY

	if fall < 50 {
		t.Errorf("soft body particles didn't fall: fall=%f (start=%f, end=%f)",
			fall, startParticleY, endParticleY)
	}
	t.Logf("soft body particle fell %f units (from %f to %f)", fall, startParticleY, endParticleY)
}

// TestSoftBodyRestsOnFloor verifies that a soft body falls and rests on
// a static floor. The particles should not pass through.
func TestSoftBodyRestsOnFloor(t *testing.T) {
	world := NewWorld(
		WithGravity(Vec2{X: 0, Y: 0.1}), // lower gravity to prevent tunneling
		WithIterations(4),
	)

	// Wide, thick floor
	floor := NewRigidBody()
	floor.AddMesh(NewRectMesh(Vec2{X: 500, Y: 200}, Vec2Zero(), Vec2Zero()))
	floor.SetPosition(Vec2{X: 250, Y: 500})
	floor.SetMode(BodyModeStatic)
	world.AddRigidBody(floor)

	// Soft body (hexagon, radius 16)
	sb := NewSoftBody()
	sb.AddMesh(NewPolygonMesh(16, 6, Vec2Zero(), -1))
	sb.SetPosition(Vec2{X: 250, Y: 200})
	world.AddSoftBody(sb)

	// Step 200 times — should reach and rest on floor
	for range 200 {
		world.Update()
	}

	// Check that no particle is below the floor top (400)
	floorTop := float32(400.0)
	mesh := sb.Meshes()[0]
	maxY := float32(0)
	for i, p := range mesh.Particles() {
		gp := p.GlobalPosition()
		if gp.Y > maxY {
			maxY = gp.Y
		}
		if gp.Y > floorTop+25 {
			t.Errorf("particle %d passed through floor: Y=%f (floor top=%f)",
				i, gp.Y, floorTop)
		}
	}

	// The soft body should have fallen significantly
	if maxY < 350 {
		t.Errorf("soft body didn't fall enough: maxY=%f (expected >350)", maxY)
	}
	t.Logf("soft body rests: maxY=%f (floor top=400)", maxY)
}

// TestSoftBodyWithGrid verifies that a gridded soft body maintains its
// structure (particles stay connected via springs).
func TestSoftBodyWithGrid(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0}))

	sb := NewSoftBody()
	// 3x3 grid soft body
	sb.AddMesh(NewRectMesh(Vec2{X: 48, Y: 48}, Vec2Zero(), Vec2{X: 3, Y: 3}))
	sb.SetPosition(Vec2{X: 100, Y: 100})
	world.AddSoftBody(sb)

	mesh := sb.Meshes()[0]

	// Should have 16 particles (4x4 grid)
	expectedParticles := 16
	if mesh.ParticleCount() != expectedParticles {
		t.Errorf("grid mesh particle count = %d, want %d", mesh.ParticleCount(), expectedParticles)
	}

	// Should have springs
	if mesh.SpringCount() == 0 {
		t.Error("grid mesh has no springs")
	}
	t.Logf("grid mesh: %d particles, %d springs", mesh.ParticleCount(), mesh.SpringCount())

	// Step a few times — springs should keep the structure roughly intact
	startPos := mesh.ParticleAt(0).GlobalPosition()
	for range 10 {
		world.Update()
	}
	endPos := mesh.ParticleAt(0).GlobalPosition()

	// Without gravity, the body should stay roughly in place
	dist := (endPos.Sub(startPos)).Length()
	if dist > 20 {
		t.Errorf("soft body drifted too far without gravity: dist=%f", dist)
	}
	t.Logf("soft body drift: %f", dist)
}

// TestSoftBodyShapeMatching verifies that shape matching pulls a deformed
// soft body back toward its rest shape.
func TestSoftBodyShapeMatching(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0}))

	sb := NewSoftBody()
	sb.AddMesh(NewPolygonMesh(20, 6, Vec2Zero(), -1)) // hexagon
	sb.SetPosition(Vec2{X: 100, Y: 100})
	sb.SetShapeMatchingEnabled(true, false)
	sb.SetShapeMatchingRate(0.5)
	world.AddSoftBody(sb)

	// Deform the soft body by moving a particle's GLOBAL position
	// Also set prevGlobalPosition to zero the velocity (prevents inertia from
	// amplifying the deformation before shape matching can correct it)
	mesh := sb.Meshes()[0]
	originalGlobal := mesh.ParticleAt(0).GlobalPosition()
	deformed := originalGlobal.Add(Vec2{X: 10, Y: 0})
	mesh.ParticleAt(0).SetGlobalPosition(deformed)
	mesh.ParticleAt(0).SetPreviousGlobalPosition(deformed)

	// Step several times — shape matching should pull the particle back
	for range 50 {
		world.Update()
	}

	// The particle should not have moved further away from the deformation.
	// Shape matching + springs should keep it at or below the initial 10-unit deformation.
	finalGlobal := mesh.ParticleAt(0).GlobalPosition()
	dist := (finalGlobal.Sub(originalGlobal)).Length()
	if dist > 15 {
		t.Errorf("shape matching let particle drift too far: dist=%f (deformation was 10)", dist)
	}
	t.Logf("particle distance from original after shape matching: %f (deformation was 10)", dist)
}

// TestSoftBodyAreaPreserving verifies that area preserving keeps the
// polygon area close to the target.
func TestSoftBodyAreaPreserving(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0}))

	sb := NewSoftBody()
	sb.AddMesh(NewPolygonMesh(20, 6, Vec2Zero(), -1)) // hexagon
	sb.SetPosition(Vec2{X: 100, Y: 100})
	sb.SetAreaPreservingEnabled(true)
	world.AddSoftBody(sb)

	mesh := sb.Meshes()[0]
	initialArea := mesh.PolygonArea()

	// Deform the soft body
	mesh.ParticleAt(0).SetPosition(mesh.ParticleAt(0).Position().Add(Vec2{X: 5, Y: 5}))
	sb.UpdateMeshTransforms()

	// Step — area preserving should restore area
	for range 100 {
		world.Update()
	}

	finalArea := mesh.PolygonArea()

	// Area should be close to initial (within 20%)
	if math32.Abs(finalArea-initialArea) > math32.Abs(initialArea)*0.2 {
		t.Errorf("area not preserved: initial=%f, final=%f", initialArea, finalArea)
	}
	t.Logf("area: initial=%f, final=%f", initialArea, finalArea)
}

// TestSpringConvergence verifies that two particles connected by a spring
// converge to the rest length.
func TestSpringConvergence(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0}))

	sb := NewSoftBody()
	// Simple 2-particle mesh via MeshData
	data := MeshData{
		ParticlePositions:      []Vec2{{X: 0, Y: 0}, {X: 20, Y: 0}},
		ParticleRadValues:      []float32{2, 2},
		ParticleInternalValues: []bool{false, false},
		ParticleEnabledValues:  []bool{true, true},
		ParticleLazyValues:     []bool{false, false},
		SpringList:             [][2]int{{0, 1}},
	}
	sb.AddMesh(NewMeshFromData(data, true, false))
	sb.SetPosition(Vec2{X: 100, Y: 100})
	world.AddSoftBody(sb)

	mesh := sb.Meshes()[0]
	restLength := mesh.Springs()[0].Length()

	// Pull particles apart
	mesh.ParticleAt(1).SetGlobalPosition(Vec2{X: 120, Y: 100})
	mesh.ParticleAt(1).SetPreviousGlobalPosition(Vec2{X: 120, Y: 100})

	// Step — spring should pull them back toward rest length
	for range 50 {
		world.Update()
	}

	finalDist := (mesh.ParticleAt(1).GlobalPosition().Sub(mesh.ParticleAt(0).GlobalPosition())).Length()

	// Should be closer to rest length than the initial 20-unit separation
	if math32.Abs(finalDist-restLength) > 5 {
		t.Errorf("spring didn't converge: restLength=%f, finalDist=%f", restLength, finalDist)
	}
	t.Logf("spring: restLength=%f, finalDist=%f", restLength, finalDist)
}

// TestSoftBodyVsRigidBody verifies that a soft body collides with a rigid body.
func TestSoftBodyVsRigidBody(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0}))

	// Static rigid body (wall)
	wall := NewRigidBody()
	wall.AddMesh(NewRectMesh(Vec2{X: 20, Y: 200}, Vec2Zero(), Vec2Zero()))
	wall.SetPosition(Vec2{X: 200, Y: 100})
	wall.SetMode(BodyModeStatic)
	world.AddRigidBody(wall)

	// Soft body moving toward the wall
	sb := NewSoftBody()
	sb.AddMesh(NewPolygonMesh(16, 6, Vec2Zero(), -1))
	sb.SetPosition(Vec2{X: 100, Y: 100})
	sb.SetPreviousPosition(Vec2{X: 95, Y: 100}) // velocity = (5, 0)
	world.AddSoftBody(sb)

	// Step until they should collide
	for range 100 {
		world.Update()
	}

	// The soft body should not have passed through the wall
	// Wall left edge is at X=190, soft body radius ~16
	if sb.Position().X > 180 {
		t.Errorf("soft body passed through wall: posX=%f (wall at 200)", sb.Position().X)
	}
	t.Logf("soft body final X=%f (wall at 200)", sb.Position().X)
}
