package platformer

import (
	"testing"

	"github.com/Vmarcelo49/quarkgo/physics"
)

// TestPlatformerWalk verifies that Walk sets the walk direction.
func TestPlatformerWalk(t *testing.T) {
	pb := New()
	pb.Walk(1) // right
	if pb.walkSide != 1 {
		t.Errorf("walkSide = %d, want 1", pb.walkSide)
	}

	pb.Walk(-1) // left
	if pb.walkSide != -1 {
		t.Errorf("walkSide = %d, want -1", pb.walkSide)
	}

	pb.Walk(0) // stop
	if pb.walkSide != 0 {
		t.Errorf("walkSide = %d, want 0", pb.walkSide)
	}
}

// TestPlatformerJump verifies the jump state machine.
func TestPlatformerJump(t *testing.T) {
	pb := New()

	// Initial state
	if pb.IsJumping() {
		t.Error("should not be jumping initially")
	}

	// Jump
	pb.Jump(5.0, false)
	if !pb.IsJumping() {
		t.Error("should be jumping after Jump()")
	}

	// Release
	pb.ReleaseJump()
	if pb.IsJumping() {
		t.Error("should not be jumping after ReleaseJump()")
	}
}

// TestPlatformerRestsOnFloor verifies that a platformer body falls and rests on a floor.
func TestPlatformerRestsOnFloor(t *testing.T) {
	world := physics.NewWorld(
		physics.WithGravity(physics.Vec2{X: 0, Y: 0}),
		physics.WithIterations(4),
	)

	// Floor
	floor := physics.NewRigidBody()
	floor.AddMesh(physics.NewRectMesh(physics.Vec2{X: 300, Y: 20}, physics.Vec2Zero(), physics.Vec2Zero()))
	floor.SetPosition(physics.Vec2{X: 150, Y: 400})
	floor.SetMode(physics.BodyModeStatic)
	world.AddRigidBody(floor)

	// Platformer body
	pb := New()
	pb.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	pb.SetPosition(physics.Vec2{X: 150, Y: 200})
	world.AddRigidBody(&pb.RigidBody)
	pb.RegisterPostUpdate()

	// Step — body should fall and land on floor
	for i := 0; i < 200; i++ {
		world.Update()
	}

	// Should be on floor
	if !pb.IsOnFloor() {
		t.Error("platformer body should be on floor after falling")
	}

	// Should not have fallen through
	if pb.Position().Y > 390 {
		t.Errorf("platformer body fell through floor: Y=%f", pb.Position().Y)
	}
	t.Logf("platformer body resting at Y=%f, onFloor=%v", pb.Position().Y, pb.IsOnFloor())
}

// TestPlatformerWalkOnFloor verifies horizontal movement.
func TestPlatformerWalkOnFloor(t *testing.T) {
	world := physics.NewWorld(
		physics.WithGravity(physics.Vec2{X: 0, Y: 0}),
		physics.WithIterations(4),
	)

	// Wide floor
	floor := physics.NewRigidBody()
	floor.AddMesh(physics.NewRectMesh(physics.Vec2{X: 500, Y: 20}, physics.Vec2Zero(), physics.Vec2Zero()))
	floor.SetPosition(physics.Vec2{X: 250, Y: 400})
	floor.SetMode(physics.BodyModeStatic)
	world.AddRigidBody(floor)

	// Platformer body
	pb := New()
	pb.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	pb.SetPosition(physics.Vec2{X: 100, Y: 370}) // start on floor
	world.AddRigidBody(&pb.RigidBody)
	pb.RegisterPostUpdate()

	// Step to settle
	for i := 0; i < 10; i++ {
		world.Update()
	}

	// Walk right
	pb.Walk(1)
	startX := pb.Position().X
	for i := 0; i < 30; i++ {
		world.Update()
	}

	endX := pb.Position().X
	moved := endX - startX
	if moved < 10 {
		t.Errorf("platformer body didn't walk: moved %f units (expected >10)", moved)
	}
	t.Logf("platformer walked %f units right", moved)
}

// TestPlatformerJumpAndLand verifies jumping and landing.
func TestPlatformerJumpAndLand(t *testing.T) {
	world := physics.NewWorld(
		physics.WithGravity(physics.Vec2{X: 0, Y: 0}),
		physics.WithIterations(4),
	)

	// Floor
	floor := physics.NewRigidBody()
	floor.AddMesh(physics.NewRectMesh(physics.Vec2{X: 300, Y: 20}, physics.Vec2Zero(), physics.Vec2Zero()))
	floor.SetPosition(physics.Vec2{X: 150, Y: 400})
	floor.SetMode(physics.BodyModeStatic)
	world.AddRigidBody(floor)

	// Platformer body
	pb := New()
	pb.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	pb.SetPosition(physics.Vec2{X: 150, Y: 370})
	world.AddRigidBody(&pb.RigidBody)
	pb.RegisterPostUpdate()

	// Step to settle on floor
	for i := 0; i < 10; i++ {
		world.Update()
	}

	if !pb.IsOnFloor() {
		t.Fatal("platformer body should be on floor before jumping")
	}

	// Jump
	pb.Jump(5.0, true)

	// Step — body should rise
	for i := 0; i < 5; i++ {
		world.Update()
	}

	if !pb.IsRising() {
		t.Error("platformer body should be rising after jump")
	}

	// Release jump (variable height)
	pb.ReleaseJump()

	// Step until landing
	for i := 0; i < 200; i++ {
		world.Update()
	}

	if !pb.IsOnFloor() {
		t.Error("platformer body should be on floor after landing")
	}
	t.Logf("jump and land: onFloor=%v, isFalling=%v, isRising=%v",
		pb.IsOnFloor(), pb.IsFalling(), pb.IsRising())
}

// TestPlatformerGetters verifies the getter methods.
func TestPlatformerGetters(t *testing.T) {
	pb := New()

	// Default gravity
	if pb.Gravity() != (physics.Vec2{X: 0, Y: 0.3}) {
		t.Errorf("default gravity = %v, want (0, 0.3)", pb.Gravity())
	}

	// Set gravity
	pb.SetGravity(physics.Vec2{X: 0, Y: 0.5})
	if pb.Gravity().Y != 0.5 {
		t.Errorf("gravity Y = %f, want 0.5", pb.Gravity().Y)
	}

	// Up direction should be opposite of gravity
	if pb.upDirection.Y >= 0 {
		t.Errorf("upDirection.Y = %f, should be negative (opposite of gravity)", pb.upDirection.Y)
	}

	// Walk speed
	pb.SetWalkSpeed(5.0)
	if pb.WalkSpeed() != 5.0 {
		t.Errorf("walkSpeed = %f, want 5.0", pb.WalkSpeed())
	}

	// Max jump count
	if pb.MaxJumpCount() != 2 {
		t.Errorf("maxJumpCount = %d, want 2", pb.MaxJumpCount())
	}
	pb.SetMaxJumpCount(3)
	if pb.MaxJumpCount() != 3 {
		t.Errorf("maxJumpCount = %d, want 3", pb.MaxJumpCount())
	}

	// Floor max angle
	pb.SetFloorMaxAngleDegree(30)
	expected := float32(30.0) * (physics.Pi / 180.0)
	if physics.Abs(pb.FloorMaxAngle()-expected) > 1e-4 {
		t.Errorf("floorMaxAngle = %f, want %f", pb.FloorMaxAngle(), expected)
	}
}
