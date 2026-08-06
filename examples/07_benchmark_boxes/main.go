// Example 07: Benchmark Boxes — 200 dynamic boxes stacked on a floor.
// Ported from examplescenebenchmarkboxes.cpp
package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
)

type BenchmarkBoxesScene struct {
	*common.Scene
}

func NewBenchmarkBoxesScene() *BenchmarkBoxesScene {
	scene := common.NewScene(1024, 600)
	scene.Renderer.Antialias = false
	scene.Renderer.ShowVertices = false

	// Floor
	scene.AddStaticRect(512, 550, 960, 64)
	// Walls
	scene.AddStaticRect(512-960/2+32, 550+1500/2, 64, 1500)
	scene.AddStaticRect(512+960/2-32, 550-1500/2, 64, 1500)

	// 200 boxes (20 columns × 10 rows)
	boxGroupCount := 20
	boxHeapCount := 10
	startX := float32(512) - float32(boxGroupCount*32)/2
	startY := float32(550) - 48
	for row := range boxHeapCount {
		for col := range boxGroupCount {
			x := startX + float32(col)*32
			y := startY - float32(row)*32
			scene.AddRectBodySized(x, y, 32, 32)
		}
	}

	return &BenchmarkBoxesScene{Scene: scene}
}

func (s *BenchmarkBoxesScene) Update() error {
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 07: Benchmark Boxes (200 bodies)")
	scene := NewBenchmarkBoxesScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
