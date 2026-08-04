package common

import (
	"image/color"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
	"github.com/Vmarcelo49/quarkgo/physics"
)

// Renderer draws physics bodies to an Ebitengine image.
// Ported from QPhysicsRenderer.
type Renderer struct {
	ShowColliders     bool
	ShowBoundingBoxes bool
	ShowSprings       bool
	ShowJoints        bool
	ShowRaycasts      bool
}

// NewRenderer creates a Renderer with default settings.
func NewRenderer() *Renderer {
	return &Renderer{
		ShowColliders: true,
		ShowSprings:   true,
		ShowJoints:    true,
		ShowRaycasts:  true,
	}
}

var (
	colorDynamic = color.RGBA{R: 100, G: 180, B: 255, A: 255}
	colorStatic  = color.RGBA{R: 120, G: 120, B: 130, A: 255}
	colorSoft    = color.RGBA{R: 255, G: 150, B: 100, A: 255}
	colorArea    = color.RGBA{R: 100, G: 255, B: 150, A: 80}
	colorBg      = color.RGBA{R: 25, G: 25, B: 30, A: 255}
	colorSpring  = color.RGBA{R: 60, G: 60, B: 70, A: 200}
	colorJoint   = color.RGBA{R: 255, G: 255, B: 100, A: 200}
	colorRay     = color.RGBA{R: 100, G: 255, B: 255, A: 150}
	colorRayHit  = color.RGBA{R: 255, G: 100, B: 100, A: 255}
	colorDrag    = color.RGBA{R: 255, G: 255, B: 0, A: 200}
	colorAABB    = color.RGBA{R: 40, G: 40, B: 50, A: 100}
)

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
}

func (r *Renderer) drawBody(screen *ebiten.Image, body *physics.Body) {
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
	default:
		c = colorDynamic
	}

	for _, mesh := range body.Meshes() {
		// Draw particles
		for _, p := range mesh.Particles() {
			pos := p.GlobalPosition()
			rad := p.Radius()
			if rad < 1 {
				rad = 2
			}
			vector.DrawFilledCircle(screen, pos.X, pos.Y, rad, c, true)
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
		if body.BodyType() == physics.BodyTypeSoft && r.ShowSprings {
			for _, spring := range mesh.Springs() {
				a := spring.ParticleA().GlobalPosition()
				b := spring.ParticleB().GlobalPosition()
				vector.StrokeLine(screen, a.X, a.Y, b.X, b.Y, 0.5, colorSpring, true)
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
	b := joint.AnchorBGlobalPosition()
	vector.StrokeLine(screen, a.X, a.Y, b.X, b.Y, 1.5, colorJoint, true)
	// Draw anchor points
	vector.DrawFilledCircle(screen, a.X, a.Y, 3, colorJoint, true)
	vector.DrawFilledCircle(screen, b.X, b.Y, 3, colorJoint, true)
}

func (r *Renderer) drawSpring(screen *ebiten.Image, spring *physics.Spring) {
	a := spring.ParticleA().GlobalPosition()
	b := spring.ParticleB().GlobalPosition()
	vector.StrokeLine(screen, a.X, a.Y, b.X, b.Y, 1.0, colorSpring, true)
}

func (r *Renderer) drawRaycast(screen *ebiten.Image, ray *physics.Raycast) {
	pos := ray.Position()
	vec := ray.RayVector()
	endX := pos.X + vec.X
	endY := pos.Y + vec.Y
	vector.StrokeLine(screen, pos.X, pos.Y, endX, endY, 1, colorRay, true)

	for _, c := range ray.Contacts() {
		vector.DrawFilledCircle(screen, c.Position.X, c.Position.Y, 4, colorRayHit, true)
		// Draw normal
		nx := c.Position.X + c.Normal.X*15
		ny := c.Position.Y + c.Normal.Y*15
		vector.StrokeLine(screen, c.Position.X, c.Position.Y, nx, ny, 1, colorRayHit, true)
	}
}

// DrawDragLine draws a line from body to mouse cursor during drag.
func (r *Renderer) DrawDragLine(screen *ebiten.Image, from, to physics.Vec2) {
	vector.StrokeLine(screen, from.X, from.Y, to.X, to.Y, 2, colorDrag, true)
	vector.DrawFilledCircle(screen, to.X, to.Y, 4, colorDrag, true)
}
