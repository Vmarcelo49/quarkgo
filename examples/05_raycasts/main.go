// Example 05: Raycasts — 360° radial raycast fan following the mouse.
// Ported from examplesceneraycasts.cpp
package main

import (
	"math/rand"

	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type RaycastsScene struct {
	*common.Scene
	raycasts []*physics.Raycast
}

func NewRaycastsScene() *RaycastsScene {
	scene := common.NewScene(1024, 600)
	scene.World.SetGravity(physics.Vec2{X: 0, Y: 0})
	scene.CreateSceneBorders()

	// 15 random bodies
	for i := 0; i < 15; i++ {
		x := float32(rand.Intn(800) + 100)
		y := float32(rand.Intn(400) + 100)
		switch rand.Intn(3) {
		case 0:
			w := float32(rand.Intn(96) + 32)
			h := float32(rand.Intn(96) + 32)
			scene.AddRectBodySized(x, y, w, h)
		case 1:
			r := float32(rand.Intn(48) + 16)
			scene.AddPolygonBodyR(x, y, rand.Intn(6)+6, r)
		case 2:
			r := float32(rand.Intn(48) + 16)
			scene.AddCircleBodyR(x, y, r)
		}
	}

	r := &RaycastsScene{Scene: scene}

	// 90 raycasts from center, length 1000
	center := physics.Vec2{X: 400, Y: 400}
	numRays := 90
	for i := 0; i < numRays; i++ {
		angle := float32(i) / float32(numRays) * physics.Pi * 2
		dir := physics.Vec2{X: physics.Cos(angle) * 1000, Y: physics.Sin(angle) * 1000}
		ray := physics.NewRaycast(center, dir, true)
		scene.World.AddRaycast(ray)
		r.raycasts = append(r.raycasts, ray)
	}

	return r
}

func (s *RaycastsScene) Update() error {
	// Raycasts follow mouse
	mouseX, mouseY := ebiten.CursorPosition()
	mousePos := physics.Vec2{X: float32(mouseX), Y: float32(mouseY)}
	for _, ray := range s.raycasts {
		ray.SetPosition(mousePos)
		// Slow rotation
		ray.SetRotation(ray.Rotation() + 0.001)
	}
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 05: Raycasts (Move mouse)")
	scene := NewRaycastsScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
