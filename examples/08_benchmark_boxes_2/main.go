// Example 08: Benchmark Boxes 2 — 600 small boxes, low gravity, sleeping disabled.
// Ported from examplescenebenchmarkboxes2.cpp
package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type BenchmarkBoxes2Scene struct {
	*common.Scene
}

func NewBenchmarkBoxes2Scene() *BenchmarkBoxes2Scene {
	scene := common.NewScene(1024, 600)
	scene.World.SetGravity(physics.Vec2{X: 0, Y: 0.1})
	scene.World.SetSleepingEnabled(false)
	scene.SpawnRectSize = 16
	scene.Renderer.ShowBoundingBoxes = false
	scene.Renderer.ShowColliders = false
	scene.Renderer.ShowJoints = false
	scene.Renderer.ShowSprings = false
	scene.Renderer.ShowVertices = false
	// Wide floor
	scene.AddStaticRect(512, 550, 5000, 64)

	// 600 boxes (30 columns × 20 rows)
	boxGroupCount := 30
	boxHeapCount := 20
	startX := float32(512) - float32(boxGroupCount*16)/2
	startY := float32(550) - 40
	for row := range boxHeapCount {
		for col := range boxGroupCount {
			x := startX + float32(col)*16
			y := startY - float32(row)*16
			scene.AddRectBodySized(x, y, 16, 16)
		}
	}

	return &BenchmarkBoxes2Scene{Scene: scene}
}

func (s *BenchmarkBoxes2Scene) Update() error {
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 08: Benchmark Boxes 2 (600 bodies)")
	scene := NewBenchmarkBoxes2Scene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
