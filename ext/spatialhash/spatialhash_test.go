package spatialhash

import (
	"testing"

	"github.com/example/quarkgo/physics"
)

// TestSpatialHashInsertRemove verifies basic insert and remove operations.
func TestSpatialHashInsertRemove(t *testing.T) {
	sh := New(64.0)

	// Create two bodies in the same world
	world := physics.NewWorld()
	b1 := physics.NewRigidBody()
	b1.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	b1.SetPosition(physics.Vec2{X: 100, Y: 100})
	world.AddRigidBody(b1)

	b2 := physics.NewRigidBody()
	b2.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	b2.SetPosition(physics.Vec2{X: 110, Y: 100})
	world.AddRigidBody(b2)

	// Insert both
	sh.Insert(b1.AsBody())
	sh.Insert(b2.AsBody())

	// They should overlap (both in the same cell region)
	pairs := sh.Pairs()
	if len(pairs) == 0 {
		t.Error("expected at least 1 pair from overlapping bodies")
	}
	t.Logf("found %d pairs for overlapping bodies", len(pairs))

	// Remove b2 — should produce no pairs
	sh.Remove(b2.AsBody())
	pairs = sh.Pairs()
	if len(pairs) != 0 {
		t.Errorf("expected 0 pairs after removing b2, got %d", len(pairs))
	}
}

// TestSpatialHashNoOverlap verifies that non-overlapping bodies don't generate pairs.
func TestSpatialHashNoOverlap(t *testing.T) {
	sh := New(64.0)

	world := physics.NewWorld()
	b1 := physics.NewRigidBody()
	b1.AddMesh(physics.NewRectMesh(physics.Vec2{X: 16, Y: 16}, physics.Vec2Zero(), physics.Vec2Zero()))
	b1.SetPosition(physics.Vec2{X: 0, Y: 0})
	world.AddRigidBody(b1)

	b2 := physics.NewRigidBody()
	b2.AddMesh(physics.NewRectMesh(physics.Vec2{X: 16, Y: 16}, physics.Vec2Zero(), physics.Vec2Zero()))
	b2.SetPosition(physics.Vec2{X: 500, Y: 500})
	world.AddRigidBody(b2)

	sh.Insert(b1.AsBody())
	sh.Insert(b2.AsBody())

	pairs := sh.Pairs()
	if len(pairs) != 0 {
		t.Errorf("expected 0 pairs for non-overlapping bodies, got %d", len(pairs))
	}
}

// TestSpatialHashCellSizeChange verifies SetCellSize clears the grid.
func TestSpatialHashCellSizeChange(t *testing.T) {
	sh := New(64.0)

	world := physics.NewWorld()
	b := physics.NewRigidBody()
	b.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	b.SetPosition(physics.Vec2{X: 100, Y: 100})
	world.AddRigidBody(b)

	sh.Insert(b.AsBody())

	// Change cell size — should clear all data
	sh.SetCellSize(128.0)

	pairs := sh.Pairs()
	if len(pairs) != 0 {
		t.Errorf("expected 0 pairs after cell size change, got %d", len(pairs))
	}
}

// TestSpatialHashImplementsBroadPhase verifies the interface.
func TestSpatialHashImplementsBroadPhase(t *testing.T) {
	var _ physics.BroadPhase = (*SpatialHashing)(nil)
}
