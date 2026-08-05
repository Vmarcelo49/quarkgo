// Example 04: Frictions — friction, static friction, air friction on angled platforms.
// Ported from examplescenefrictions.cpp
package main

import (
	"github.com/chewxy/math32"
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
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
	platformAngle := float32(6.2 * (math32.Pi / 180.0))
	for row := range 3 {
		px := float32(200)
		py := float32(176 + row*120)

		// Angled platform
		plat := scene.AddStaticRectRot(px, py, 600, 32, platformAngle)
		plat.SetFriction(0.5)

		// Box on platform.
		// Matches C++ examplescenefrictions.cpp: uses a WORLD-space offset
		// (not rotated by platformAngle) for the box position, then sets
		// the box rotation to platformAngle so its bottom edge is parallel
		// to the platform's top edge.
		box := scene.AddRectBodySized(px, py-41, 32, 32)
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

		// Ball on platform (rows 0-1).
		// Same convention as the box: world-space offset (-130, -51).
		if row < 2 {
			ball := scene.AddCircleBodyR(px-130, py-51, 16)
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
	for i := range 10 {
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
