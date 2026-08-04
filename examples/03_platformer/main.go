package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/ext/platformer"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type PlatformerScene struct {
	*common.Scene
	player *platformer.PlatformerBody
}

func NewPlatformerScene() *PlatformerScene {
	scene := common.NewScene(800, 600)

	// Floor
	floor := physics.NewRigidBody()
	floor.AddMesh(physics.NewRectMesh(physics.Vec2{X: 800, Y: 40}, physics.Vec2Zero(), physics.Vec2Zero()))
	floor.SetPosition(physics.Vec2{X: 400, Y: 580})
	floor.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(floor)

	// Platforms
	platform := physics.NewRigidBody()
	platform.AddMesh(physics.NewRectMesh(physics.Vec2{X: 150, Y: 20}, physics.Vec2Zero(), physics.Vec2Zero()))
	platform.SetPosition(physics.Vec2{X: 300, Y: 450})
	platform.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(platform)

	platform2 := physics.NewRigidBody()
	platform2.AddMesh(physics.NewRectMesh(physics.Vec2{X: 150, Y: 20}, physics.Vec2Zero(), physics.Vec2Zero()))
	platform2.SetPosition(physics.Vec2{X: 550, Y: 350})
	platform2.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(platform2)

	// Player
	player := platformer.New()
	player.AddMesh(physics.NewRectMesh(physics.Vec2{X: 24, Y: 24}, physics.Vec2Zero(), physics.Vec2Zero()))
	player.SetPosition(physics.Vec2{X: 100, Y: 500})
	scene.World.AddRigidBody(&player.RigidBody)
	player.RegisterPostUpdate()

	return &PlatformerScene{Scene: scene, player: player}
}

func (s *PlatformerScene) Update() error {
	// Input
	walkSide := 0
	if ebiten.IsKeyPressed(ebiten.KeyRight) || ebiten.IsKeyPressed(ebiten.KeyD) {
		walkSide = 1
	}
	if ebiten.IsKeyPressed(ebiten.KeyLeft) || ebiten.IsKeyPressed(ebiten.KeyA) {
		walkSide = -1
	}
	s.player.Walk(walkSide)

	if ebiten.IsKeyPressed(ebiten.KeySpace) || ebiten.IsKeyPressed(ebiten.KeyUp) || ebiten.IsKeyPressed(ebiten.KeyW) {
		s.player.Jump(5.0, false)
	} else {
		s.player.ReleaseJump()
	}

	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(800, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 03: Platformer (Arrow Keys + Space)")
	scene := NewPlatformerScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
