// Example 99: Testing — zero-gravity sandbox with bouncing circles.
// Ported from examplescenetesting.cpp
package main

import (
	"math/rand"

	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type TestingScene struct {
	*common.Scene
}

func NewTestingScene() *TestingScene {
	scene := common.NewScene(1024, 600)
	scene.World.SetGravity(physics.Vec2{X: 0, Y: 0})
	scene.CreateSceneBorders()

	// 5 bouncing circles
	for range 5 {
		x := float32(rand.Intn(800) + 100)
		y := float32(rand.Intn(400) + 100)
		ball := scene.AddCircleBodyR(x, y, 24)
		ball.SetRestitution(0.9)
		// Give random velocity
		ball.SetPreviousPosition(physics.Vec2{
			X: x - float32(rand.Intn(10)-5),
			Y: y - float32(rand.Intn(10)-5),
		})
	}

	return &TestingScene{Scene: scene}
}

func (s *TestingScene) Update() error {
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 99: Testing Sandbox (Click to drag)")
	scene := NewTestingScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
