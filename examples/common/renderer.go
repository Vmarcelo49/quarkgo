package common

import (
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
	"github.com/Vmarcelo49/quarkgo/physics"
)

// Renderer draws physics bodies to an Ebitengine image.
type Renderer struct {
	ShowColliders     bool
	ShowBoundingBoxes bool
}

// NewRenderer creates a Renderer with default settings.
func NewRenderer() *Renderer {
	return &Renderer{
		ShowColliders: true,
	}
}

var (
	colorDynamic = color.RGBA{R: 100, G: 180, B: 255, A: 255}
	colorStatic  = color.RGBA{R: 180, G: 180, B: 180, A: 255}
	colorSoft    = color.RGBA{R: 255, G: 150, B: 100, A: 255}
	colorArea    = color.RGBA{R: 100, G: 255, B: 150, A: 100}
	colorBg      = color.RGBA{R: 30, G: 30, B: 35, A: 255}
)

// Draw renders all bodies in the world to the screen.
func (r *Renderer) Draw(screen *ebiten.Image, world *physics.World) {
	screen.Fill(colorBg)

	for _, body := range world.Bodies() {
		if !body.Enabled() {
			continue
		}

		var c color.Color
		switch body.BodyType() {
		case physics.BodyTypeRigid:
			if body.Mode() == physics.BodyModeStatic {
				c = colorStatic
			} else {
				c = colorDynamic
			}
		case physics.BodyTypeSoft:
			c = colorSoft
		case physics.BodyTypeArea:
			c = colorArea
		}

		for _, mesh := range body.Meshes() {
			// Draw particles
			for _, p := range mesh.Particles() {
				pos := p.GlobalPosition()
				vector.DrawFilledCircle(screen, pos.X, pos.Y, p.Radius(), c, true)
			}

			// Draw polygon edges
			poly := mesh.Polygon()
			if len(poly) >= 2 {
				for i := 0; i < len(poly); i++ {
					a := poly[i].GlobalPosition()
					b := poly[(i+1)%len(poly)].GlobalPosition()
					vector.StrokeLine(screen, a.X, a.Y, b.X, b.Y, 1.5, c, true)
				}
			}

			// Draw springs for soft bodies
			if body.BodyType() == physics.BodyTypeSoft {
				for _, spring := range mesh.Springs() {
					a := spring.ParticleA().GlobalPosition()
					b := spring.ParticleB().GlobalPosition()
					vector.StrokeLine(screen, a.X, a.Y, b.X, b.Y, 0.5, color.RGBA{R: 80, G: 80, B: 80, A: 200}, true)
				}
			}
		}

		if r.ShowBoundingBoxes {
			aabb := body.AABB()
			vector.StrokeRect(screen, aabb.Min.X, aabb.Min.Y, aabb.Max.X-aabb.Min.X, aabb.Max.Y-aabb.Min.Y, 1, color.RGBA{R: 50, G: 50, B: 50, A: 150}, true)
		}
	}
}
