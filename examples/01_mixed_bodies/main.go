// Example 01: Mixed Bodies — random rigid primitives + soft body "QUARK" letters.
// Ported from examplescenemixedbodies.cpp
package main

import (
	"fmt"
	"math/rand"
	"path/filepath"

	"github.com/hajimehoshi/ebiten/v2"

	"github.com/Vmarcelo49/quarkgo/examples/common"
	"github.com/Vmarcelo49/quarkgo/mesh/qmesh"
	"github.com/Vmarcelo49/quarkgo/physics"
)

type MixedBodiesScene struct {
	*common.Scene
}

func NewMixedBodiesScene() *MixedBodiesScene {
	scene := common.NewScene(1024, 600)

	// Floor
	floor := scene.AddStaticRect(512, 550, 960, 64)
	floor.SetRestitution(0.3)

	// Side walls
	scene.AddStaticRect(512-960/2+32, 550+1500/2, 64, 1500) // left
	scene.AddStaticRect(512+960/2-32, 550-1500/2, 64, 1500) // right

	// Random primitive grid: 3 rows × 7 cols
	startX, startY := float32(128), float32(100)
	for row := range 3 {
		for col := range 7 {
			x := startX + float32(col)*96
			y := startY - float32(row)*64
			r := float32(rand.Intn(32) + 16) // 16..47
			if rand.Intn(2) == 0 {
				scene.AddCircleBodyR(x, y, r)
			} else {
				scene.AddPolygonBodyR(x, y, rand.Intn(8)+3, r)
			}
		}
	}

	// Soft body "QUARK" letters
	letterFiles := []string{"word_q.qmesh", "word_u.qmesh", "word_a.qmesh", "word_r.qmesh", "word_k.qmesh"}
	for i, file := range letterFiles {
		meshes, err := qmesh.LoadFile(filepath.Join("examples", "01_mixed_bodies", file))
		if err != nil {
			// Try relative to executable
			meshes, err = qmesh.LoadFile(file)
			if err != nil {
				fmt.Printf("Warning: could not load %s: %v\n", file, err)
				continue
			}
		}
		sb := physics.NewSoftBody()
		for _, md := range meshes {
			sb.AddMesh(physics.NewMeshFromData(md, true, true))
		}
		sb.SetPosition(physics.Vec2{X: float32(180 + i*170), Y: 425})
		sb.SetRigidity(0.3)
		sb.SetShapeMatchingEnabled(true, false)
		sb.SetShapeMatchingRate(0.35)
		sb.SetSelfCollisionsEnabled(true)
		scene.World.AddSoftBody(sb)
	}

	return &MixedBodiesScene{Scene: scene}
}

func (s *MixedBodiesScene) Update() error {
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 01: Mixed Bodies (Click to drag)")
	scene := NewMixedBodiesScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}
