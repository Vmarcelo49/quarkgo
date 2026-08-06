// Example 06: Platformer — character controller with walk, jump, wall-jump.
// Ported from examplesceneplatformer.cpp
package main

import (
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/ext/platformer"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type PlatformerScene struct {
	*common.Scene
	player                               *platformer.PlatformerBody
	mBlock                               *physics.RigidBody
	mBlockStart, mBlockEnd, mBlockTarget physics.Vec2
	mBlockMoveSpeed                      float32
}

func NewPlatformerScene() *PlatformerScene {
	scene := common.NewScene(1024, 600)
	scene.World.SetSleepingEnabled(false)

	// Custom sloped floor (concave polygon)
	customData := physics.MeshData{
		ParticlePositions: []physics.Vec2{
			{X: 203, Y: 480}, {X: 380, Y: 480}, {X: 500, Y: 410},
			{X: 690, Y: 410}, {X: 750, Y: 480}, {X: 900, Y: 480},
			// Bottom (reversed)
			{X: 900, Y: 550}, {X: 750, Y: 550}, {X: 690, Y: 550},
			{X: 500, Y: 550}, {X: 380, Y: 550}, {X: 203, Y: 550},
		},
		ParticleRadValues:      []float32{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
		ParticleInternalValues: []bool{false, false, false, false, false, false, false, false, false, false, false, false},
		ParticleEnabledValues:  []bool{true, true, true, true, true, true, true, true, true, true, true, true},
		ParticleLazyValues:     []bool{false, false, false, false, false, false, false, false, false, false, false, false},
		Polygon:                []int{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
	}
	floor := physics.NewRigidBody()
	floor.AddMesh(physics.NewMeshFromData(customData, false, true))
	floor.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(floor)

	// Rectangle platforms
	scene.AddStaticRect(99, 400, 128, 64)
	scene.AddStaticRect(960, 270, 64, 300)
	scene.AddStaticRect(810, 275, 64, 200)

	// Moving platform
	ps := &PlatformerScene{Scene: scene, mBlockMoveSpeed: 1.0}
	ps.mBlock = physics.NewRigidBody()
	ps.mBlock.AddMesh(physics.NewRectMesh(physics.Vec2{X: 128, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	ps.mBlock.SetPosition(physics.Vec2{X: 630, Y: 190})
	ps.mBlock.SetKinematicEnabled(true)
	ps.mBlock.SetFixedRotationEnabled(true)
	scene.World.AddRigidBody(ps.mBlock)
	ps.mBlockStart = physics.Vec2{X: 630, Y: 190}
	ps.mBlockEnd = ps.mBlockStart.Add(physics.Vec2{X: -320, Y: 150})
	ps.mBlockTarget = ps.mBlockEnd

	// Player
	ps.player = platformer.New()
	ps.player.AddMesh(physics.NewRectMesh(physics.Vec2{X: 32, Y: 32}, physics.Vec2Zero(), physics.Vec2Zero()))
	ps.player.SetPosition(physics.Vec2{X: 600, Y: 200})
	scene.World.AddRigidBody(&ps.player.RigidBody)
	ps.player.RegisterPostUpdate()

	// Coins (area bodies)
	coinPositions := []physics.Vec2{
		{X: 570, Y: 320}, {X: 606, Y: 320}, {X: 640, Y: 320},
		{X: 84, Y: 350}, {X: 120, Y: 350},
	}
	for _, pos := range coinPositions {
		coin := physics.NewAreaBody()
		coin.AddMesh(physics.NewCircleMesh(8, physics.Vec2Zero()))
		coin.SetPosition(pos)
		coin.OnCollisionEnter = func(ab *physics.AreaBody, b *physics.Body) {
			scene.World.RemoveBody(ab.AsBody())
		}
		scene.World.AddAreaBody(coin)
	}

	return ps
}

func (s *PlatformerScene) Update() error {

	// teleport with click
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButton0) {
		s.player.SetPosition(s.MousePosition(), true)
	}

	// Player input
	walkSide := 0
	if ebiten.IsKeyPressed(ebiten.KeyRight) || ebiten.IsKeyPressed(ebiten.KeyD) {
		walkSide = 1
	}
	if ebiten.IsKeyPressed(ebiten.KeyLeft) || ebiten.IsKeyPressed(ebiten.KeyA) {
		walkSide = -1
	}
	s.player.Walk(walkSide)

	// Wall mode
	wallOffset := float32(3.0)
	wallSide := 0
	if s.player.GetRightWall(wallOffset).HasBody() {
		wallSide = 1
	}
	if s.player.GetLeftWall(wallOffset).HasBody() {
		wallSide = -1
	}
	wallMode := !s.player.IsOnFloor() && wallSide != 0
	if wallMode {
		if s.player.IsFalling() {
			s.player.SetGravityMultiplier(0.3)
		} else {
			s.player.SetGravityMultiplier(1.0)
		}
	} else {
		s.player.SetGravityMultiplier(1.0)
	}

	// Jump
	if ebiten.IsKeyPressed(ebiten.KeyUp) || ebiten.IsKeyPressed(ebiten.KeySpace) || ebiten.IsKeyPressed(ebiten.KeyW) {
		if wallMode {
			if s.player.IsJumpReleased() {
				s.player.Jump(5.0, true)
				s.player.SetControllerHorizontalVelocity(physics.Vec2Right().Mul(10.0 * float32(-wallSide)))
			}
		} else {
			s.player.Jump(5.0, false)
		}
	} else {
		s.player.ReleaseJump()
	}

	// Moving platform logic
	diff := s.mBlockTarget.Sub(s.mBlock.Position())
	diffNormal := diff.Normalized()
	if diff.Length() <= s.mBlockMoveSpeed {
		s.mBlock.AddForce(diff)
		if s.mBlockTarget == s.mBlockEnd {
			s.mBlockTarget = s.mBlockStart
		} else {
			s.mBlockTarget = s.mBlockEnd
		}
	} else {
		s.mBlock.AddForce(diffNormal.Mul(s.mBlockMoveSpeed))
	}

	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 06: Platformer (Arrows/WASD + Space)")
	scene := NewPlatformerScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
