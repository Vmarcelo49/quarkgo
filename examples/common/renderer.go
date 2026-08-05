package common

import (
	"fmt"
	"image/color"

	"github.com/Vmarcelo49/quarkgo/physics"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// Renderer draws physics bodies to an Ebitengine image.
// Ported from QPhysicsRenderer.
type Renderer struct {
	ShowColliders     bool
	ShowBoundingBoxes bool
	ShowSprings       bool
	ShowJoints        bool
	ShowRaycasts      bool
	ShowParticles     bool
	path              vector.Path
	so                vector.StrokeOptions
	dpo               vector.DrawPathOptions
	fo                vector.FillOptions
}

// NewRenderer creates a Renderer with default settings.
func NewRenderer() *Renderer {
	return &Renderer{
		ShowColliders: true,
		ShowSprings:   true,
		ShowJoints:    true,
		ShowRaycasts:  true,
		ShowParticles: true,
		path:          vector.Path{},
		so:            vector.StrokeOptions{},
		dpo:           vector.DrawPathOptions{},
		fo:            vector.FillOptions{},
	}
}

var (
	colorParticle = rgb(202, 158, 219)
	colorDynamic  = rgb(48, 182, 3)
	colorStatic   = rgb(141, 141, 141)
	colorSoft     = color.RGBA{R: 255, G: 150, B: 100, A: 255}
	colorArea     = color.RGBA{R: 100, G: 255, B: 150, A: 80}
	colorBg       = color.RGBA{R: 25, G: 25, B: 30, A: 255}
	colorSpring   = color.RGBA{R: 60, G: 60, B: 70, A: 200}
	colorJoint    = color.RGBA{R: 255, G: 255, B: 100, A: 200}
	colorRay      = rgb(102, 102, 102)
	colorRayHit   = rgb(255, 0, 0)
	colorDrag     = color.RGBA{R: 255, G: 255, B: 0, A: 200}
	colorAABB     = color.RGBA{R: 40, G: 40, B: 50, A: 100}
)

func rgb(r, g, b uint8) color.RGBA {
	return color.RGBA{r, g, b, 255}
}

// Draw renders all bodies in the world to the screen.
func (r *Renderer) Draw(screen *ebiten.Image, world *physics.World) {
	screen.Fill(colorBg)

	for _, body := range world.Bodies() {
		if !body.Enabled() {
			continue
		}
		r.drawBody(screen, body)
	}

	if r.ShowJoints {
		for _, joint := range world.Joints() {
			r.drawJoint(screen, joint)
		}
	}

	if r.ShowSprings {
		for _, spring := range world.Springs() {
			r.drawSpring(screen, spring)
		}
	}

	if r.ShowRaycasts {
		for _, ray := range world.Raycasts() {
			r.drawRaycast(screen, ray)
		}
	}

	ebitenutil.DebugPrint(
		screen,
		fmt.Sprintf("FPS: %v TPS: %v", ebiten.ActualFPS(), ebiten.ActualTPS()),
	)
}

func (r *Renderer) drawBody(screen *ebiten.Image, body *physics.Body) {

	var clr color.Color
	switch body.BodyType() {
	case physics.BodyTypeRigid:
		if body.Mode() == physics.BodyModeStatic {
			clr = colorStatic
		} else {
			clr = colorDynamic
		}
	case physics.BodyTypeSoft:
		clr = colorSoft
	case physics.BodyTypeArea:
		clr = colorArea
	default:
		clr = colorDynamic
	}

	for _, mesh := range body.Meshes() {

		// Draw polygon edges
		poly := mesh.Polygon()
		r.path.Reset()
		if len(poly) >= 2 {
			for i := range poly {
				p := poly[i].GlobalPosition()
				if i == 0 {
					r.path.MoveTo(p.X, p.Y)
				} else {
					r.path.LineTo(p.X, p.Y)
				}
			}
			r.path.Close()
		}
		r.so.Width = 2
		r.dpo.ColorScale.Reset()
		r.dpo.ColorScale.ScaleWithColor(clr)
		vector.FillPath(screen, &r.path, &r.fo, &r.dpo)

		if r.ShowParticles {
			// // Draw particles
			for _, p := range mesh.Particles() {
				pos := p.GlobalPosition()
				rad := p.Radius()
				if rad < 1 {
					rad = 2
				}
				vector.FillCircle(screen, pos.X, pos.Y, rad, colorParticle, false)
			}
		}

		// Draw springs for soft bodies
		if body.BodyType() == physics.BodyTypeSoft && r.ShowSprings {
			for _, spring := range mesh.Springs() {
				a := spring.ParticleA().GlobalPosition()
				b := spring.ParticleB().GlobalPosition()
				vector.StrokeLine(screen, a.X, a.Y, b.X, b.Y, 0.5, colorSpring, false)
			}
		}
	}

	if r.ShowBoundingBoxes {
		aabb := body.AABB()
		vector.StrokeRect(screen, aabb.Min.X, aabb.Min.Y,
			aabb.Max.X-aabb.Min.X, aabb.Max.Y-aabb.Min.Y,
			1, colorAABB, true)
	}
}

func (r *Renderer) drawJoint(screen *ebiten.Image, joint *physics.Joint) {
	a := joint.AnchorAGlobalPosition()
	// b := joint.AnchorBGlobalPosition()
	// vector.StrokeLine(screen, a.X, a.Y, b.X, b.Y, 1.5, colorJoint, true)
	// Draw anchor points
	vector.FillCircle(screen, a.X, a.Y, 3, colorJoint, false)
}

func (r *Renderer) drawSpring(screen *ebiten.Image, spring *physics.Spring) {
	a := spring.ParticleA().GlobalPosition()
	b := spring.ParticleB().GlobalPosition()
	vector.StrokeLine(screen, a.X, a.Y, b.X, b.Y, 1.0, colorSpring, false)
}

func (r *Renderer) drawRaycast(screen *ebiten.Image, ray *physics.Raycast) {
	pos := ray.Position()
	vec := ray.RayVector()
	endX := pos.X + vec.X
	endY := pos.Y + vec.Y
	vector.StrokeLine(screen, pos.X, pos.Y, endX, endY, 1, colorRay, false)

	for _, c := range ray.Contacts() {
		vector.FillCircle(screen, c.Position.X, c.Position.Y, 3, colorRayHit, false)
	}
}

// DrawDragLine draws a line from body to mouse cursor during drag.
func (r *Renderer) DrawDragLine(screen *ebiten.Image, from, to physics.Vec2) {
	vector.StrokeLine(screen, from.X, from.Y, to.X, to.Y, 2, colorDrag, false)
	vector.FillCircle(screen, to.X, to.Y, 4, colorDrag, false)
}
