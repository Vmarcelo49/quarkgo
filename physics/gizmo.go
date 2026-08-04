package physics

// Gizmo is a debug visualization primitive. Matches QGizmo in qgizmos.h.
//
// Gizmos are recorded by the engine during collision solving when
// World.DebugGizmos() is true. Renderers (e.g., the Ebitengine example
// renderer) consume them for visual debugging.
type Gizmo struct {
	Type GizmoType
	// Circle fields
	Position Vec2
	Radius   float32
	// Line fields
	PointA Vec2
	PointB Vec2
	IsArrow bool
	// Rect field
	Rect AABB
}

// GizmoType enumerates gizmo kinds.
type GizmoType int

const (
	GizmoCircle GizmoType = iota
	GizmoLine
	GizmoRectangle
)

// NewGizmoCircle creates a circle gizmo.
func NewGizmoCircle(pos Vec2, radius float32) *Gizmo {
	return &Gizmo{Type: GizmoCircle, Position: pos, Radius: radius}
}

// NewGizmoLine creates a line gizmo.
func NewGizmoLine(from, to Vec2, arrow bool) *Gizmo {
	return &Gizmo{Type: GizmoLine, PointA: from, PointB: to, IsArrow: arrow}
}

// NewGizmoRect creates a rectangle gizmo.
func NewGizmoRect(rect AABB) *Gizmo {
	return &Gizmo{Type: GizmoRectangle, Rect: rect}
}
