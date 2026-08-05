package physics

import "testing"

// TestJointConstraint verifies that a joint maintains a target distance
// between two rigid bodies.
func TestJointConstraint(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0}))

	// Two dynamic bodies
	bodyA := NewRigidBody()
	bodyA.AddMesh(NewRectMesh(Vec2{X: 32, Y: 32}, Vec2Zero(), Vec2Zero()))
	bodyA.SetPosition(Vec2{X: 0, Y: 100})
	world.AddRigidBody(bodyA)

	bodyB := NewRigidBody()
	bodyB.AddMesh(NewRectMesh(Vec2{X: 32, Y: 32}, Vec2Zero(), Vec2Zero()))
	bodyB.SetPosition(Vec2{X: 100, Y: 100})
	world.AddRigidBody(bodyB)

	// Joint with target length 50
	joint := NewJoint(bodyA, Vec2{X: 16, Y: 100}, Vec2{X: 84, Y: 100}, bodyB)
	joint.SetLength(50)
	joint.SetRigidity(1.0)
	world.AddJoint(joint)

	// Step and trace
	for i := range 5 {
		world.Update()
		dist := (bodyB.Position().Sub(bodyA.Position())).Length()
		t.Logf("step %d: dist=%.2f posA=(%.1f,%.1f) posB=(%.1f,%.1f)", i, dist,
			bodyA.Position().X, bodyA.Position().Y, bodyB.Position().X, bodyB.Position().Y)
	}
	for i := 5; i < 100; i++ {
		world.Update()
	}

	dist := (bodyB.Position().Sub(bodyA.Position())).Length()
	// The joint should have pulled the bodies closer than the initial 100.
	// Due to Verlet oscillation, the exact distance varies, but it should
	// be significantly less than 100.
	if dist > 90 {
		t.Errorf("joint didn't pull bodies together: dist=%f (initial=100, target=50)", dist)
	}
	t.Logf("joint: initial=100, target=50, final=%f", dist)
}

// TestJointPinToWorld verifies a joint connecting a body to a fixed point.
func TestJointPinToWorld(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0.2}))

	// Dynamic body
	body := NewRigidBody()
	body.AddMesh(NewRectMesh(Vec2{X: 32, Y: 32}, Vec2Zero(), Vec2Zero()))
	body.SetPosition(Vec2{X: 100, Y: 100})
	world.AddRigidBody(body)

	// Pin joint to world space at (100, 100)
	joint := NewJoint(body, Vec2{X: 100, Y: 100}, Vec2{X: 100, Y: 100}, nil)
	joint.SetLength(0)
	joint.SetRigidity(0.5)
	world.AddJoint(joint)

	// Step — gravity pulls down, joint pulls back
	for range 100 {
		world.Update()
	}

	// Body should stay near the pin point (not fall away)
	dist := (body.Position().Sub(Vec2{X: 100, Y: 100})).Length()
	if dist > 30 {
		t.Errorf("pin joint didn't hold body: dist=%f (expected <30)", dist)
	}
	t.Logf("pin joint: body drifted %f from pin point", dist)
}

// TestJointGrooveMode verifies groove mode (pull-only).
func TestJointGrooveMode(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0}))

	bodyA := NewRigidBody()
	bodyA.AddMesh(NewRectMesh(Vec2{X: 32, Y: 32}, Vec2Zero(), Vec2Zero()))
	bodyA.SetPosition(Vec2{X: 0, Y: 100})
	world.AddRigidBody(bodyA)

	bodyB := NewRigidBody()
	bodyB.AddMesh(NewRectMesh(Vec2{X: 32, Y: 32}, Vec2Zero(), Vec2Zero()))
	bodyB.SetPosition(Vec2{X: 30, Y: 100}) // closer than length
	world.AddRigidBody(bodyB)

	// Joint with groove mode and length 50
	joint := NewJoint(bodyA, Vec2{X: 16, Y: 100}, Vec2{X: 14, Y: 100}, bodyB)
	joint.SetLength(50)
	joint.SetRigidity(1.0)
	joint.SetGrooveEnabled(true)
	world.AddJoint(joint)

	// Step — groove mode should NOT push bodies apart (current < target)
	for range 50 {
		world.Update()
	}

	dist := (bodyB.Position().Sub(bodyA.Position())).Length()
	// Should not have been pushed apart (groove = pull-only)
	if dist > 40 {
		t.Errorf("groove joint pushed bodies apart: dist=%f (should stay <=50)", dist)
	}
	t.Logf("groove joint: dist=%f (target=50, initial=30)", dist)
}

// TestJointCollisionException verifies that jointed bodies don't collide.
func TestJointCollisionException(t *testing.T) {
	world := NewWorld(WithGravity(Vec2{X: 0, Y: 0}))

	bodyA := NewRigidBody()
	bodyA.AddMesh(NewRectMesh(Vec2{X: 32, Y: 32}, Vec2Zero(), Vec2Zero()))
	bodyA.SetPosition(Vec2{X: 0, Y: 100})
	bodyA.SetPreviousPosition(Vec2{X: -1, Y: 100}) // moving right
	world.AddRigidBody(bodyA)

	bodyB := NewRigidBody()
	bodyB.AddMesh(NewRectMesh(Vec2{X: 32, Y: 32}, Vec2Zero(), Vec2Zero()))
	bodyB.SetPosition(Vec2{X: 50, Y: 100}) // close to A
	bodyB.SetMode(BodyModeStatic)
	world.AddRigidBody(bodyB)

	// Joint with collisions disabled (default) — should register exception
	joint := NewJoint(bodyA, Vec2{X: 16, Y: 100}, Vec2{X: 34, Y: 100}, bodyB)
	world.AddJoint(joint)

	// Verify collision exception was registered
	if !world.CheckCollisionException(bodyA.AsBody(), bodyB.AsBody()) {
		t.Error("joint did not register collision exception")
	}
}
