package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type MixedBodiesScene struct {
	*common.Scene
	spawnTimer int
}

func NewMixedBodiesScene() *MixedBodiesScene {
	scene := common.NewScene(800, 600)

	// Floor
	floor := physics.NewRigidBody()
	floor.AddMesh(physics.NewRectMesh(physics.Vec2{X: 780, Y: 20}, physics.Vec2Zero(), physics.Vec2Zero()))
	floor.SetPosition(physics.Vec2{X: 400, Y: 570})
	floor.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(floor)

	// Walls
	leftWall := physics.NewRigidBody()
	leftWall.AddMesh(physics.NewRectMesh(physics.Vec2{X: 20, Y: 600}, physics.Vec2Zero(), physics.Vec2Zero()))
	leftWall.SetPosition(physics.Vec2{X: 10, Y: 300})
	leftWall.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(leftWall)

	rightWall := physics.NewRigidBody()
	rightWall.AddMesh(physics.NewRectMesh(physics.Vec2{X: 20, Y: 600}, physics.Vec2Zero(), physics.Vec2Zero()))
	rightWall.SetPosition(physics.Vec2{X: 790, Y: 300})
	rightWall.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(rightWall)

	return &MixedBodiesScene{Scene: scene}
}

func (s *MixedBodiesScene) Update() error {
	s.spawnTimer++
	if s.spawnTimer >= 30 && s.World.BodyCount() < 50 {
		s.spawnTimer = 0
		box := physics.NewRigidBody()
		box.AddMesh(physics.NewRectMesh(physics.Vec2{X: 24, Y: 24}, physics.Vec2Zero(), physics.Vec2Zero()))
		box.SetPosition(physics.Vec2{X: 400, Y: 50})
		box.SetRestitution(0.3)
		s.World.AddRigidBody(box)
	}
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(800, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 01: Mixed Bodies")
	scene := NewMixedBodiesScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
