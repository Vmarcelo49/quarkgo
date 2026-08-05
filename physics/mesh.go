package physics

import (
	"slices"

	"github.com/chewxy/math32"
)

// Mesh is the shape + topology container for a body. Matches QMesh in
// qmesh.h, qmesh.cpp.
//
// A Mesh carries:
//   - Particles (the smallest building blocks; positioned by the body)
//   - Polygon (the subset of particles forming the collision boundary)
//   - Springs (distance constraints between particles — used by soft bodies)
//   - AngleConstraints (3-particle angle limits — used by soft bodies)
//   - SubConvexPolygons (concave polygon decomposed into convex pieces for SAT)
//   - PolygonBisectors (cached bisector vectors for collision response)
//   - UVMaps (triangle index lists for rendering)
//
// The CollisionBehavior field determines which collision algorithm runs:
//   - CIRCLES  — particles treated as circles (no polygon)
//   - POLYGONS — rigid body with a polygon (uses SAT + clipping)
//   - POLYLINE — soft body with a polygon (treated as a deformable rope)
type Mesh struct {
	particles         []*Particle
	polygon           []*Particle
	subConvexPolygons [][]*Particle
	polygonBisectors  []Vec2
	springs           []*Spring
	angleConstraints  []*AngleConstraint

	position       Vec2
	globalPosition Vec2
	rotation       float32
	globalRotation float32

	ownerBody *Body

	collisionBehavior            CollisionBehavior
	collisionBehaviorNeedsUpdate bool

	uvMaps [][]int

	// Polygon state
	circumference                float32
	lastPolygonCornerAngles      []float32
	minAngleConstraintOfPolygon  float32
	subConvexPolygonsNeedsUpdate bool
	polygonBisectorsNeedsUpdate  bool
	isPolygonSelfIntersected     bool
	disablePolygonForCollisions  bool
}

// CollisionBehavior enumerates how a mesh participates in collision detection.
// Matches QMesh::CollisionBehaviors in qmesh.h:51-55.
type CollisionBehavior int

const (
	// CollisionCircles treats particles as circles. Used when no polygon
	// is defined (e.g., a single-particle circle mesh).
	CollisionCircles CollisionBehavior = iota

	// CollisionPolygons treats the polygon as a solid convex/concave shape.
	// Used by rigid bodies.
	CollisionPolygons

	// CollisionPolyline treats the polygon as a deformable rope. Used by
	// soft bodies, where the polygon may self-intersect.
	CollisionPolyline
)

// MeshData is the serializable description of a mesh. Used as input to
// CreateWithMeshData and as the on-disk .qmesh format (Phase 3).
// Matches QMesh::MeshData in qmesh.h:104-145.
type MeshData struct {
	ParticlePositions      []Vec2
	ParticleRadValues      []float32
	ParticleInternalValues []bool
	ParticleEnabledValues  []bool
	ParticleLazyValues     []bool
	SpringList             [][2]int
	InternalSpringList     [][2]int
	Polygon                []int
	UVMaps                 [][]int
	Position               Vec2
	Rotation               float32
}

// NewMesh creates an empty mesh.
func NewMesh() *Mesh {
	return &Mesh{
		minAngleConstraintOfPolygon:  math32.Pi * 0.3,
		polygonBisectorsNeedsUpdate:  true,
		subConvexPolygonsNeedsUpdate: true,
	}
}

// --- Getters ---

// Position returns the mesh's local position (relative to the owning body).
func (m *Mesh) Position() Vec2 { return m.position }

// GlobalPosition returns the mesh's world-space position.
func (m *Mesh) GlobalPosition() Vec2 { return m.globalPosition }

// Rotation returns the mesh's local rotation (radians).
func (m *Mesh) Rotation() float32 { return m.rotation }

// GlobalRotation returns the mesh's world-space rotation.
func (m *Mesh) GlobalRotation() float32 { return m.globalRotation }

// OwnerBody returns the body that owns this mesh.
func (m *Mesh) OwnerBody() *Body { return m.ownerBody }

// Particles returns the slice of particles owned by this mesh.
func (m *Mesh) Particles() []*Particle { return m.particles }

// Polygon returns the particles forming the collision boundary.
func (m *Mesh) Polygon() []*Particle { return m.polygon }

// ParticleCount returns the number of particles in the mesh.
func (m *Mesh) ParticleCount() int { return len(m.particles) }

// ParticleAt returns the particle at the given index.
func (m *Mesh) ParticleAt(i int) *Particle { return m.particles[i] }

// Springs returns the springs owned by this mesh.
func (m *Mesh) Springs() []*Spring { return m.springs }

// SpringCount returns the number of springs.
func (m *Mesh) SpringCount() int { return len(m.springs) }

// AngleConstraints returns the angle constraints owned by this mesh.
func (m *Mesh) AngleConstraints() []*AngleConstraint { return m.angleConstraints }

// AddAngleConstraint attaches an angle constraint to the mesh.
func (m *Mesh) AddAngleConstraint(ac *AngleConstraint) *Mesh {
	m.angleConstraints = append(m.angleConstraints, ac)
	return m
}

// AddSpring attaches a spring to the mesh.
func (m *Mesh) AddSpring(s *Spring) *Mesh {
	m.springs = append(m.springs, s)
	return m
}

// CollisionBehavior returns the collision behavior, computing it lazily
// if a recomputation is pending.
func (m *Mesh) CollisionBehavior() CollisionBehavior {
	if m.collisionBehaviorNeedsUpdate {
		m.UpdateCollisionBehavior()
		m.collisionBehaviorNeedsUpdate = false
	}
	return m.collisionBehavior
}

// Circumference returns the total perimeter of the polygon (using local
// particle positions). Matches QMesh::GetCircumference.
func (m *Mesh) Circumference() float32 {
	var res float32
	n := len(m.polygon)
	for i := range n {
		p := m.polygon[i]
		np := m.polygon[(i+1)%n]
		res += (np.Position().Sub(p.Position())).Length()
	}
	return res
}

// InitialArea returns the area computed from local particle positions,
// including both polygon area and circle areas (for particles with r > 0.5).
// Matches QMesh::GetInitialArea.
func (m *Mesh) InitialArea() float32 {
	res := m.polygonArea(true)
	for _, p := range m.particles {
		if p.Radius() > 0.5 {
			res += p.Radius() * p.Radius()
		}
	}
	return res
}

// Area returns the area computed from global particle positions.
// Matches QMesh::GetArea.
func (m *Mesh) Area() float32 {
	res := m.polygonArea(false)
	for _, p := range m.particles {
		if p.Radius() > 0.5 {
			res += p.Radius() * p.Radius()
		}
	}
	return res
}

// PolygonArea returns the polygon area (local or global).
func (m *Mesh) PolygonArea() float32 { return m.polygonArea(false) }

// polygonArea computes the signed area of the polygon using the shoelace
// formula. If useLocal is true, uses local particle positions; otherwise
// uses global positions.
func (m *Mesh) polygonArea(useLocal bool) float32 {
	if len(m.polygon) < 3 {
		return 0
	}
	var area float32
	n := len(m.polygon)
	for i := range n {
		p := m.polygon[i]
		np := m.polygon[(i+1)%n]
		var pp, npv Vec2
		if useLocal {
			pp = p.Position()
			npv = np.Position()
		} else {
			pp = p.GlobalPosition()
			npv = np.GlobalPosition()
		}
		area += pp.X*npv.Y - npv.X*pp.Y
	}
	return area * 0.5
}

// --- Setters ---

// SetPosition sets the mesh's local position.
func (m *Mesh) SetPosition(v Vec2) *Mesh { m.position = v; return m }

// SetGlobalPosition sets the mesh's world-space position.
func (m *Mesh) SetGlobalPosition(v Vec2) *Mesh { m.globalPosition = v; return m }

// SetRotation sets the mesh's local rotation.
func (m *Mesh) SetRotation(r float32) *Mesh { m.rotation = r; return m }

// SetPolygonForCollisionsDisabled disables the polygon for collisions,
// forcing the mesh to use circle-based collision on its particles.
func (m *Mesh) SetPolygonForCollisionsDisabled(b bool) *Mesh {
	m.disablePolygonForCollisions = b
	m.collisionBehaviorNeedsUpdate = true
	return m
}

// --- Particle operations ---

// AddParticle appends a particle to the mesh and sets its owner.
// Matches QMesh::AddParticle.
func (m *Mesh) AddParticle(p *Particle) *Mesh {
	p.SetOwnerMesh(m)
	m.particles = append(m.particles, p)
	return m
}

// RemoveParticleAt removes the particle at the given index and cascades the
// removal to polygon, springs, UV maps, and angle constraints.
// Matches QMesh::RemoveParticleAt (qmesh.cpp:112-128).
//
// C++ calls RemoveParticleFromPolygon, RemoveMatchingSprings,
// RemoveMatchingUVMaps, RemoveMatchingAngleConstraints before erasing the
// particle from the vector. Then sets dirty flags (collisionBehaviorNeedsUpdate,
// polygonBisectorsNeedsUpdate, inertiaNeedsUpdate, circumferenceNeedsUpdate)
// and updates static body transforms if applicable.
func (m *Mesh) RemoveParticleAt(i int) *Mesh {
	if i < 0 || i >= len(m.particles) {
		return m
	}
	particle := m.particles[i]

	// Remove from polygon (if present).
	m.removeParticleFromPolygon(particle)

	// Remove springs that reference this particle.
	m.removeMatchingSprings(particle)

	// Remove/trim UV maps referencing this index.
	m.removeMatchingUVMaps(i)

	// Remove angle constraints referencing this particle.
	m.removeMatchingAngleConstraints(particle)

	// Erase the particle from the slice.
	m.particles = append(m.particles[:i], m.particles[i+1:]...)

	// Cascade dirty flags to owner body.
	if m.ownerBody != nil {
		if m.ownerBody.mode == BodyModeStatic {
			m.ownerBody.UpdateMeshTransforms()
		}
		m.ownerBody.inertiaNeedsUpdate = true
		m.ownerBody.circumferenceNeedsUpdate = true
	}
	m.collisionBehaviorNeedsUpdate = true
	m.polygonBisectorsNeedsUpdate = true
	return m
}

// removeParticleFromPolygon removes the particle from the polygon slice if present.
func (m *Mesh) removeParticleFromPolygon(p *Particle) {
	for i, pp := range m.polygon {
		if pp == p {
			m.polygon = append(m.polygon[:i], m.polygon[i+1:]...)
			return
		}
	}
}

// removeMatchingSprings removes all springs that reference the given particle.
// Note: Go's Mesh has a single `springs` slice (internal springs are
// identified by their `isInternal` flag, not a separate slice). Matches C++
// RemoveMatchingSprings which removes from both boundary and internal lists.
func (m *Mesh) removeMatchingSprings(p *Particle) {
	newSprings := m.springs[:0]
	for _, s := range m.springs {
		if s.pA != p && s.pB != p {
			newSprings = append(newSprings, s)
		}
	}
	m.springs = newSprings
}

// removeMatchingUVMaps removes the index from all UV maps and trims maps that
// become too short. Matches C++ RemoveMatchingUVMaps which shifts indices
// down by 1 for indices > i, and removes maps containing the deleted index.
func (m *Mesh) removeMatchingUVMaps(removedIndex int) {
	newUVMaps := m.uvMaps[:0]
	for _, uv := range m.uvMaps {
		// Drop the UV map if it contains the removed index.
		contains := slices.Contains(uv, removedIndex)
		if contains {
			continue
		}
		// Shift indices > removedIndex down by 1.
		for j := range uv {
			if uv[j] > removedIndex {
				uv[j]--
			}
		}
		newUVMaps = append(newUVMaps, uv)
	}
	m.uvMaps = newUVMaps
}

// removeMatchingAngleConstraints removes all angle constraints that reference
// the given particle (as pA, pB, or pC).
func (m *Mesh) removeMatchingAngleConstraints(p *Particle) {
	newAC := m.angleConstraints[:0]
	for _, ac := range m.angleConstraints {
		if ac.pA != p && ac.pB != p && ac.pC != p {
			newAC = append(newAC, ac)
		}
	}
	m.angleConstraints = newAC
}

// RemoveParticle removes the given particle from the mesh.
func (m *Mesh) RemoveParticle(p *Particle) *Mesh {
	for i, pp := range m.particles {
		if pp == p {
			return m.RemoveParticleAt(i)
		}
	}
	return m
}

// ParticleIndex returns the index of the given particle, or -1 if not found.
func (m *Mesh) ParticleIndex(p *Particle) int {
	for i, pp := range m.particles {
		if pp == p {
			return i
		}
	}
	return -1
}

// --- Collision behavior ---

// UpdateCollisionBehavior recomputes the collision behavior based on
// the owning body type and polygon presence. Matches QMesh::UpdateCollisionBehavior.
func (m *Mesh) UpdateCollisionBehavior() {
	if m.disablePolygonForCollisions || len(m.polygon) == 0 {
		m.collisionBehavior = CollisionCircles
		return
	}
	switch m.ownerBody.bodyType {
	case BodyTypeRigid, BodyTypeArea:
		// Rigid and area bodies use solid polygon collision (SAT + clipping).
		// Area bodies don't respond to collisions but still detect them.
		m.collisionBehavior = CollisionPolygons
	case BodyTypeSoft:
		m.collisionBehavior = CollisionPolyline
	default:
		m.collisionBehavior = CollisionPolygons
	}
}

// --- Factory methods ---

// NewCircleMesh creates a single-particle mesh representing a circle.
// Matches QMesh::CreateWithCircle in qmesh.cpp.
func NewCircleMesh(radius float32, centerPosition Vec2) *Mesh {
	m := NewMesh()
	p := NewParticle(centerPosition.X, centerPosition.Y, radius)
	m.AddParticle(p)
	return m
}

// NewRectMesh creates a 4-corner rectangle mesh. If grid is non-zero,
// generates an internal grid of particles with cross-diagonal springs
// (used for soft bodies). For rigid bodies, pass Vec2Zero for grid.
// Matches QMesh::CreateWithRect.
func NewRectMesh(size, centerPosition, grid Vec2, opts ...MeshFactoryOption) *Mesh {
	cfg := defaultMeshFactoryConfig()
	for _, opt := range opts {
		opt(&cfg)
	}
	data := GenerateRectangleMeshData(size, centerPosition, grid, cfg.particleRadius)
	return NewMeshFromData(data, cfg.enableSprings, cfg.enablePolygons)
}

// NewPolygonMesh creates a regular N-gon mesh. If polarGrid > 0, generates
// concentric rings of internal particles connected by springs.
// Matches QMesh::CreateWithPolygon.
func NewPolygonMesh(radius float32, sideCount int, centerPosition Vec2, polarGrid int, opts ...MeshFactoryOption) *Mesh {
	cfg := defaultMeshFactoryConfig()
	for _, opt := range opts {
		opt(&cfg)
	}
	data := GeneratePolygonMeshData(radius, sideCount, centerPosition, polarGrid, cfg.particleRadius)
	return NewMeshFromData(data, cfg.enableSprings, cfg.enablePolygons)
}

// MeshFactoryConfig holds optional parameters for mesh factory methods.
type MeshFactoryConfig struct {
	enableSprings  bool
	enablePolygons bool
	particleRadius float32
}

func defaultMeshFactoryConfig() MeshFactoryConfig {
	return MeshFactoryConfig{
		enableSprings:  true,
		enablePolygons: true,
		particleRadius: 0.5,
	}
}

// MeshFactoryOption configures a mesh factory.
type MeshFactoryOption func(*MeshFactoryConfig)

// WithSprings enables/disables spring generation in mesh factories.
func WithSprings(b bool) MeshFactoryOption {
	return func(c *MeshFactoryConfig) { c.enableSprings = b }
}

// WithPolygons enables/disables polygon generation in mesh factories.
func WithPolygons(b bool) MeshFactoryOption {
	return func(c *MeshFactoryConfig) { c.enablePolygons = b }
}

// WithParticleRadius sets the particle radius used by mesh factories.
func WithParticleRadius(r float32) MeshFactoryOption {
	return func(c *MeshFactoryConfig) { c.particleRadius = r }
}

// NewMeshFromData constructs a mesh from a MeshData struct. This is the
// universal factory — all other factories (rect, polygon) build a MeshData
// and delegate here. Matches QMesh::CreateWithMeshData in qmesh.cpp.
func NewMeshFromData(data MeshData, enableSprings, enablePolygons bool) *Mesh {
	m := NewMesh()

	// Add particles
	for i, pos := range data.ParticlePositions {
		p := NewParticle(pos.X, pos.Y, 0.5)
		if i < len(data.ParticleRadValues) {
			p.SetRadius(data.ParticleRadValues[i])
		}
		if i < len(data.ParticleInternalValues) {
			p.SetIsInternal(data.ParticleInternalValues[i])
		}
		if i < len(data.ParticleEnabledValues) {
			p.SetEnabled(data.ParticleEnabledValues[i])
		}
		if i < len(data.ParticleLazyValues) {
			p.SetIsLazy(data.ParticleLazyValues[i])
		}
		m.AddParticle(p)
	}

	// Build polygon from indices
	if enablePolygons {
		for _, idx := range data.Polygon {
			if idx >= 0 && idx < len(m.particles) {
				m.polygon = append(m.polygon, m.particles[idx])
			}
		}
	}

	// Add springs (boundary + internal)
	if enableSprings {
		for _, spr := range data.SpringList {
			if spr[0] >= 0 && spr[0] < len(m.particles) && spr[1] >= 0 && spr[1] < len(m.particles) {
				s := NewSpring(m.particles[spr[0]], m.particles[spr[1]], false)
				m.springs = append(m.springs, s)
				m.particles[spr[0]].registerSpringConnection(m.particles[spr[1]])
				m.particles[spr[1]].registerSpringConnection(m.particles[spr[0]])
			}
		}
		for _, spr := range data.InternalSpringList {
			if spr[0] >= 0 && spr[0] < len(m.particles) && spr[1] >= 0 && spr[1] < len(m.particles) {
				s := NewSpring(m.particles[spr[0]], m.particles[spr[1]], true)
				m.springs = append(m.springs, s)
				m.particles[spr[0]].registerSpringConnection(m.particles[spr[1]])
				m.particles[spr[1]].registerSpringConnection(m.particles[spr[0]])
			}
		}
	}

	// Build angle constraints for the polygon (one per polygon vertex)
	// This matches QMesh's behavior where polygon meshes get angle constraints
	// automatically. The rigidity is set from minAngleConstraintOfPolygon.
	if enablePolygons && len(m.polygon) >= 3 {
		// Skip angle constraint creation by default — the C++ engine creates
		// them via ApplyAngleConstraintsToPolygon which uses a different
		// mechanism (per-polygon-vertex angle limits). For Phase 2 we rely
		// on springs for structural rigidity. Angle constraints can be added
		// explicitly by users via AddAngleConstraint.
	}

	// Copy UV maps
	m.uvMaps = make([][]int, len(data.UVMaps))
	for i, uv := range data.UVMaps {
		m.uvMaps[i] = append([]int(nil), uv...)
	}

	m.position = data.Position
	m.rotation = data.Rotation

	return m
}

// --- Mesh data generators ---

// GenerateRectangleMeshData produces a MeshData for a rectangle of the
// given size, centered at centerPosition. If grid.X or grid.Y > 1, an
// internal grid of particles is generated with cross-diagonal springs.
// Matches QMesh::GenerateRectangleMeshData in qmesh.cpp.
func GenerateRectangleMeshData(size, centerPosition, grid Vec2, particleRadius float32) MeshData {
	res := MeshData{}
	halfSize := size.Mul(0.5)

	if grid.X <= 1 && grid.Y <= 1 {
		// 4-corner rectangle
		res.ParticlePositions = []Vec2{
			{X: -halfSize.X + centerPosition.X, Y: -halfSize.Y + centerPosition.Y},
			{X: halfSize.X + centerPosition.X, Y: -halfSize.Y + centerPosition.Y},
			{X: halfSize.X + centerPosition.X, Y: halfSize.Y + centerPosition.Y},
			{X: -halfSize.X + centerPosition.X, Y: halfSize.Y + centerPosition.Y},
		}
		res.ParticleRadValues = []float32{particleRadius, particleRadius, particleRadius, particleRadius}
		res.ParticleInternalValues = []bool{false, false, false, false}
		res.ParticleEnabledValues = []bool{true, true, true, true}
		res.ParticleLazyValues = []bool{false, false, false, false}
		for i := range res.ParticlePositions {
			res.Polygon = append(res.Polygon, i)
		}
		res.SpringList = [][2]int{{0, 1}, {1, 2}, {2, 3}, {3, 0}}
		res.InternalSpringList = [][2]int{{0, 2}, {1, 3}}
		res.UVMaps = [][]int{{0, 1, 3}, {1, 2, 3}}
	} else {
		cellSize := size.DivVec(grid)
		// Add all particle coordinates
		for iy := 0; iy < int(grid.Y)+1; iy++ {
			for ix := 0; ix < int(grid.X)+1; ix++ {
				nPos := Vec2{X: float32(ix) * cellSize.X, Y: float32(iy) * cellSize.Y}
				res.ParticlePositions = append(res.ParticlePositions, centerPosition.Sub(halfSize).Add(nPos))
				res.ParticleRadValues = append(res.ParticleRadValues, particleRadius)
				isBoundary := ix == 0 || ix == int(grid.X) || iy == 0 || iy == int(grid.Y)
				res.ParticleInternalValues = append(res.ParticleInternalValues, !isBoundary)
				res.ParticleEnabledValues = append(res.ParticleEnabledValues, true)
				res.ParticleLazyValues = append(res.ParticleLazyValues, false)
			}
		}
		// Add spring pairs (matches the C++ logic in GenerateRectangleMeshData)
		gx := int(grid.X)
		gy := int(grid.Y)
		for i := range res.ParticlePositions {
			gridX := i % (gx + 1)
			gridY := (i - gridX) / (gx + 1)
			// To right
			if gridX != gx {
				var spr [2]int
				if gridY == 0 {
					spr = [2]int{i, i + 1}
				} else {
					spr = [2]int{i + 1, i}
				}
				if gridY == 0 || gridY == gy {
					res.SpringList = append(res.SpringList, spr)
				} else {
					res.InternalSpringList = append(res.InternalSpringList, spr)
				}
			}
			// To right cross down
			if gridX != gx && gridY != gy {
				res.InternalSpringList = append(res.InternalSpringList, [2]int{i, i + gx + 2})
			}
			// To left cross down
			if gridX != 0 && gridY != gy {
				res.InternalSpringList = append(res.InternalSpringList, [2]int{i, i + gx})
			}
			// To down
			if gridY != gy {
				var spr [2]int
				if gridX == 0 {
					spr = [2]int{i + gx + 1, i}
				} else {
					spr = [2]int{i, i + gx + 1}
				}
				if gridX == 0 || gridX == gx {
					res.SpringList = append(res.SpringList, spr)
				} else {
					res.InternalSpringList = append(res.InternalSpringList, spr)
				}
			}
		}
		// Build polygon from boundary particles (those on the edge of the grid)
		// Top edge (left to right)
		for ix := 0; ix <= gx; ix++ {
			res.Polygon = append(res.Polygon, ix)
		}
		// Right edge (top+1 to bottom)
		for iy := 1; iy <= gy; iy++ {
			res.Polygon = append(res.Polygon, iy*(gx+1)+gx)
		}
		// Bottom edge (right-1 to left)
		for ix := gx - 1; ix >= 0; ix-- {
			res.Polygon = append(res.Polygon, gy*(gx+1)+ix)
		}
		// Left edge (bottom-1 to top+1)
		for iy := gy - 1; iy >= 1; iy-- {
			res.Polygon = append(res.Polygon, iy*(gx+1))
		}
	}
	return res
}

// GeneratePolygonMeshData produces a MeshData for a regular N-gon of the
// given radius and side count. If polarGrid > 0, generates concentric
// rings of internal particles. Matches QMesh::GeneratePolygonMeshData.
func GeneratePolygonMeshData(radius float32, sideCount int, centerPosition Vec2, polarGrid int, particleRadius float32) MeshData {
	res := MeshData{}
	anglePart := (math32.Pi * 2) / float32(sideCount)

	// Boundary particles, polygon, and springs
	for i := range sideCount {
		curAng := anglePart * float32(i)
		curNorm := Vec2{X: math32.Cos(curAng), Y: math32.Sin(curAng)}
		nPos := centerPosition.Add(curNorm.Mul(radius))
		res.ParticlePositions = append(res.ParticlePositions, nPos)
		res.ParticleRadValues = append(res.ParticleRadValues, particleRadius)
		res.ParticleInternalValues = append(res.ParticleInternalValues, false)
		res.ParticleEnabledValues = append(res.ParticleEnabledValues, true)
		res.ParticleLazyValues = append(res.ParticleLazyValues, false)
		res.Polygon = append(res.Polygon, i)
		res.SpringList = append(res.SpringList, [2]int{i, (i + 1) % sideCount})
	}

	// Internal particles and springs (polar grid)
	if polarGrid > 0 {
		radiusPart := radius / float32(polarGrid)
		for i := polarGrid - 1; i > 0; i-- {
			curRadius := radiusPart * float32(i)
			for n := range sideCount {
				curAng := anglePart * float32(n)
				curNorm := Vec2{X: math32.Cos(curAng), Y: math32.Sin(curAng)}
				nPos := centerPosition.Add(curNorm.Mul(curRadius))
				res.ParticlePositions = append(res.ParticlePositions, nPos)
				res.ParticleRadValues = append(res.ParticleRadValues, particleRadius)
				res.ParticleInternalValues = append(res.ParticleInternalValues, true)
				res.ParticleEnabledValues = append(res.ParticleEnabledValues, true)
				res.ParticleLazyValues = append(res.ParticleLazyValues, false)

				if n != 0 {
					res.InternalSpringList = append(res.InternalSpringList,
						[2]int{len(res.ParticlePositions) - 2, len(res.ParticlePositions) - 1})
				}
			}
			res.InternalSpringList = append(res.InternalSpringList,
				[2]int{len(res.ParticlePositions) - 1, len(res.ParticlePositions) - sideCount})

			// Add cross springs (matches the C++ logic)
			startIdx := len(res.ParticlePositions) - sideCount
			endIdx := len(res.ParticlePositions)
			for n := startIdx; n < endIdx; n++ {
				a := n - sideCount
				var b, c int
				if n == endIdx-1 {
					b = len(res.ParticlePositions) - sideCount*2
					c = len(res.ParticlePositions) - sideCount
				} else {
					b = n - (sideCount - 1)
					c = n + 1
				}
				d := n
				res.InternalSpringList = append(res.InternalSpringList,
					[2]int{d, a},
					[2]int{d, b},
					[2]int{c, a},
					[2]int{c, b},
				)
			}
		}

		// Adding a Center Particle (matches qmesh.cpp:1185-1198).
		// For polarGrid > 0, add a center particle and radial springs from
		// the center to the innermost ring. Without this, the innermost ring
		// has no inward anchor and the soft body collapses inward.
		centerParticleFactor := 0
		if polarGrid > 0 {
			centerParticleFactor = 1
			res.ParticlePositions = append(res.ParticlePositions, centerPosition)
			res.ParticleRadValues = append(res.ParticleRadValues, particleRadius)
			res.ParticleInternalValues = append(res.ParticleInternalValues, true)
			res.ParticleEnabledValues = append(res.ParticleEnabledValues, true)
			res.ParticleLazyValues = append(res.ParticleLazyValues, false)
			// Radial springs from center to innermost ring.
			centerIdx := len(res.ParticlePositions) - 1
			for i := centerIdx - sideCount; i < centerIdx; i++ {
				res.InternalSpringList = append(res.InternalSpringList,
					[2]int{centerIdx, i})
			}
		}

		// Adding construction springs (matches qmesh.cpp:1202-1214).
		// For polarGrid >= 0, add ±2 boundary-neighbor springs to provide
		// shear resistance. Without these, soft body polygons shear too easily.
		if polarGrid >= 0 {
			pc := len(res.ParticlePositions)
			startIndex := pc - sideCount - centerParticleFactor
			for i := range sideCount {
				prevParticle := startIndex + ((i - 2 + sideCount) % sideCount)
				particle := startIndex + i
				nextParticle := startIndex + ((i + 2) % sideCount)
				res.InternalSpringList = append(res.InternalSpringList,
					[2]int{prevParticle, particle},
					[2]int{particle, nextParticle})
			}
		}

		// Adding UV Maps (matches qmesh.cpp:1219-1266).
		if polarGrid <= 0 {
			// Fan: single UV map listing all polygon vertices.
			var map1 []int
			for i := range res.Polygon {
				map1 = append(map1, i)
			}
			res.UVMaps = append(res.UVMaps, map1)
		} else if polarGrid == 1 {
			// Per-edge triangle to center.
			polySize := len(res.Polygon)
			centerIdx := len(res.ParticlePositions) - 1
			for i := range polySize {
				tri := []int{i, (i + 1) % polySize, centerIdx}
				res.UVMaps = append(res.UVMaps, tri)
			}
		} else {
			// Grid of 2 triangles per quad + innermost ring fan.
			polySize := len(res.Polygon)
			// Quad triangles (between each pair of rings).
			for i := 0; i < len(res.ParticlePositions)-polySize-1; i++ {
				a := i
				var b int
				if (i+1)%polySize == 0 {
					b = (i + 1) - polySize
				} else {
					b = i + 1
				}
				c := b + polySize
				d := i + polySize
				res.UVMaps = append(res.UVMaps, []int{a, b, d})
				res.UVMaps = append(res.UVMaps, []int{b, c, d})
			}
			// Innermost ring fan (to center).
			centerIdx := len(res.ParticlePositions) - 1
			for i := len(res.ParticlePositions) - polySize - 1; i < centerIdx; i++ {
				var b int
				if (i+1)%polySize == 0 {
					b = (i + 1) - polySize
				} else {
					b = i + 1
				}
				tri := []int{i, b, centerIdx}
				res.UVMaps = append(res.UVMaps, tri)
			}
		}
	}
	return res
}

// --- Internal helpers used by Body ---

// markSubConvexPolygonsDirty flags that the convex decomposition cache
// is stale. Called when the polygon changes.
func (m *Mesh) markSubConvexPolygonsDirty() {
	m.subConvexPolygonsNeedsUpdate = true
}

// UpdateSubConvexPolygons recomputes the convex decomposition of the
// mesh's polygon. If the polygon is convex, the decomposition is just
// the polygon itself. If concave, it's decomposed via polypartition.
//
// Matches QMesh::UpdateSubConvexPolygons in qmesh.cpp:239-268.
//
// NOTE: The C++ engine uses the vendored polypartition library directly.
// The Go port uses the mesh/polypartition sub-package, but to avoid a
// circular import (polypartition imports physics, physics can't import
// polypartition), the decomposition is performed by an external function
// set via SetConvexPartitioner. The World sets this up at initialization.
func (m *Mesh) UpdateSubConvexPolygons(majorUpdate bool) {
	if !m.subConvexPolygonsNeedsUpdate && !majorUpdate {
		return
	}

	if len(m.polygon) < 3 {
		m.subConvexPolygons = nil
		m.subConvexPolygonsNeedsUpdate = false
		return
	}

	// Check if the polygon is convex
	if isPolygonConvex(m.polygon) {
		// Convex — the polygon is its own decomposition
		m.subConvexPolygons = [][]*Particle{m.polygon}
	} else {
		// Concave — use the external partitioner if available
		if convexPartitionerFunc != nil {
			m.subConvexPolygons = convexPartitionerFunc(m.polygon)
		} else {
			// No partitioner registered — fall back to the full polygon.
			// This means concave bodies won't be properly decomposed, but
			// the simulation will still run (with possible collision artifacts).
			m.subConvexPolygons = [][]*Particle{m.polygon}
		}
	}

	m.subConvexPolygonsNeedsUpdate = false
}

// SubConvexPolygons returns the cached convex decomposition.
func (m *Mesh) SubConvexPolygons() [][]*Particle {
	if m.subConvexPolygonsNeedsUpdate {
		m.UpdateSubConvexPolygons(false)
	}
	return m.subConvexPolygons
}

// isPolygonConvex reports whether a polygon (as a slice of particles) is convex.
// Uses the cross product test: all cross products must have the same sign.
func isPolygonConvex(polygon []*Particle) bool {
	n := len(polygon)
	if n < 3 {
		return false
	}

	var prevSign float32 = 0
	for i := range n {
		prev := polygon[(i-1+n)%n].GlobalPosition()
		curr := polygon[i].GlobalPosition()
		next := polygon[(i+1)%n].GlobalPosition()

		v1x := curr.X - prev.X
		v1y := curr.Y - prev.Y
		v2x := next.X - curr.X
		v2y := next.Y - curr.Y
		cross := v1x*v2y - v1y*v2x

		if cross != 0 {
			sign := float32(1)
			if cross < 0 {
				sign = -1
			}
			if prevSign == 0 {
				prevSign = sign
			} else if sign != prevSign {
				return false
			}
		}
	}
	return true
}

// convexPartitionerFunc is the external function used to decompose concave
// polygons into convex sub-polygons. Set by the World (or the user) at
// initialization. This indirection avoids a circular import between
// physics and mesh/polypartition.
var convexPartitionerFunc func(polygon []*Particle) [][]*Particle

// SetConvexPartitioner registers the function used for concave polygon
// decomposition. Pass nil to disable (concave polygons will use the full
// polygon without decomposition, which may cause collision artifacts).
func SetConvexPartitioner(f func(polygon []*Particle) [][]*Particle) {
	convexPartitionerFunc = f
}
