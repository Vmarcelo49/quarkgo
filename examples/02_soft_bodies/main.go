package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type SoftBodiesScene struct {
	*common.Scene
	spawnTimer int
}

func NewSoftBodiesScene() *SoftBodiesScene {
	scene := common.NewScene(800, 600)

	// Floor
	floor := physics.NewRigidBody()
	floor.AddMesh(physics.NewRectMesh(physics.Vec2{X: 780, Y: 100}, physics.Vec2Zero(), physics.Vec2Zero()))
	floor.SetPosition(physics.Vec2{X: 400, Y: 550})
	floor.SetMode(physics.BodyModeStatic)
	scene.World.AddRigidBody(floor)

	// Walls
	for _, x := range []float32{10, 790} {
		wall := physics.NewRigidBody()
		wall.AddMesh(physics.NewRectMesh(physics.Vec2{X: 20, Y: 600}, physics.Vec2Zero(), physics.Vec2Zero()))
		wall.SetPosition(physics.Vec2{X: x, Y: 300})
		wall.SetMode(physics.BodyModeStatic)
		scene.World.AddRigidBody(wall)
	}

	return &SoftBodiesScene{Scene: scene}
}

func (s *SoftBodiesScene) Update() error {
	s.spawnTimer++
	if s.spawnTimer >= 60 && s.World.BodyCount() < 10 {
		s.spawnTimer = 0
		sb := physics.NewSoftBody()
		sb.AddMesh(physics.NewPolygonMesh(20, 8, physics.Vec2Zero(), -1))
		sb.SetPosition(physics.Vec2{X: 300 + float32(s.World.BodyCount()*40), Y: 50})
		sb.SetAreaPreservingEnabled(true)
		sb.SetShapeMatchingEnabled(true, false)
		s.World.AddSoftBody(sb)
	}
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(800, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 02: Soft Bodies")
	scene := NewSoftBodiesScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
