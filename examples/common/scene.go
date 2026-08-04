package common

import (
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/Vmarcelo49/quarkgo/physics"
)

// Scene is a base structure for example scenes.
type Scene struct {
	World    *physics.World
	Size     physics.Vec2
	Renderer *Renderer
}

// NewScene creates a new Scene with default settings.
func NewScene(w, h float32) *Scene {
	world := physics.NewWorld(
		physics.WithGravity(physics.Vec2{X: 0, Y: 0.2}),
		physics.WithIterations(4),
	)
	return &Scene{
		World:    world,
		Size:     physics.Vec2{X: w, Y: h},
		Renderer: NewRenderer(),
	}
}

// Update advances the physics simulation by one step.
func (s *Scene) Update() error {
	s.World.Update()
	return nil
}

// Draw renders the physics state to the screen.
func (s *Scene) Draw(screen *ebiten.Image) {
	s.Renderer.Draw(screen, s.World)
}

// Layout returns the screen dimensions.
func (s *Scene) Layout(outsideW, outsideH int) (int, int) {
	return int(s.Size.X), int(s.Size.Y)
}
