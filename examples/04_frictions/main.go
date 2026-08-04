// Example 04: Frictions — friction, static friction, air friction on angled platforms.
// Ported from examplescenefrictions.cpp
package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type FrictionsScene struct {
	*common.Scene
}

func NewFrictionsScene() *FrictionsScene {
	scene := common.NewScene(1024, 600)
	scene.World.SetSleepingEnabled(false)

	// Floor
	floor := scene.AddStaticRect(512, 550, 3000, 64)
	floor.SetFriction(0.5)
	floor.SetStaticFriction(1.0)

	// 3 angled platforms
	platformAngle := float32(6.2 * (physics.Pi / 180.0))
	for row := 0; row < 3; row++ {
		px := float32(200)
		py := float32(176 + row*120)

		// Angled platform
		plat := scene.AddStaticRectRot(px, py, 600, 32, platformAngle)
		plat.SetFriction(0.5)

		// Box on platform
		boxOffset := physics.Vec2{X: 0, Y: -41}.Rotated(platformAngle)
		box := scene.AddRectBodySized(px+boxOffset.X, py+boxOffset.Y, 32, 32)
		box.SetRotation(platformAngle)
		switch row {
		case 0:
			box.SetFriction(0.01)
			box.SetAirFriction(0.0)
		case 1:
			box.SetFriction(0.05)
		case 2:
			box.SetFriction(0.05)
			box.SetStaticFriction(0.01)
		}

		// Ball on platform (rows 0-1)
		if row < 2 {
			ballOffset := physics.Vec2{X: -130, Y: -51}.Rotated(platformAngle)
			ball := scene.AddCircleBodyR(px+ballOffset.X, py+ballOffset.Y, 16)
			switch row {
			case 0:
				ball.SetFriction(0.1)
				ball.SetStaticFriction(0.0)
			case 1:
				ball.SetFriction(0.2)
				ball.SetStaticFriction(0.0)
			}
		}
	}

	// Box stack on floor
	for i := 0; i < 10; i++ {
		scene.AddRectBodySized(512+100, 550-48-float32(i)*32, 32, 32)
	}

	return &FrictionsScene{Scene: scene}
}

func (s *FrictionsScene) Update() error {
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 04: Frictions (Click to drag)")
	scene := NewFrictionsScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
