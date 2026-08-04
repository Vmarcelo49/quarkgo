// Example 09: Blobs — spawn soft body blobs at the mouse position.
// Ported from examplesceneblobs.cpp
package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type BlobsScene struct {
	*common.Scene
}

func NewBlobsScene() *BlobsScene {
	scene := common.NewScene(1024, 600)
	scene.World.SetIterationCount(2)

	// Floor
	scene.AddStaticRect(512, 550, 960, 64)
	// Walls
	scene.AddStaticRect(512-960/2+32, 550+1500/2, 64, 1500)
	scene.AddStaticRect(512+960/2-32, 550-1500/2, 64, 1500)

	return &BlobsScene{Scene: scene}
}

func (s *BlobsScene) Update() error {
	// Spawn blob on Space
	if ebiten.IsKeyPressed(ebiten.KeySpace) {
		mouseX, mouseY := ebiten.CursorPosition()
		mousePos := physics.Vec2{X: float32(mouseX), Y: float32(mouseY)}

		sb := physics.NewSoftBody()
		sb.AddMesh(physics.NewPolygonMesh(64, 12, physics.Vec2Zero(), -1))
		sb.SetPosition(mousePos)
		sb.SetRigidity(0.5)
		sb.SetMass(0.5)
		sb.SetAreaPreservingEnabled(true)
		sb.SetAreaPreservingRate(0.7)
		s.World.AddSoftBody(sb)
	}

	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 09: Blobs (Hold Space to spawn)")
	scene := NewBlobsScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
