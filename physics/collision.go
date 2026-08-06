package physics

import "github.com/chewxy/math32"

// GetCollisions runs narrowphase collision detection between two bodies
// and returns a list of contacts.
//
// Dispatches by the collision behavior of each body's meshes:
//   - Polygons × Polygons → PolygonAndPolygon
//   - Circles × Polygons  → CircleAndPolygon
//   - Circles × Circles   → CircleAndCircle
//   - Polyline × Polygons → CircleAndPolygon (hot-solved if applyHotSolvers)
//   - PolylineAndPolygon
//
// When applyHotSolvers is true AND the body pair is Polyline×Polygons, the
// CircleAndPolygon contacts are immediately solved via a temp Manifold
// (hot-solving) BEFORE PolylineAndPolygon runs. This matches qworld.cpp:1017-1025.
// Hot-solving mutates body state during narrowphase, so it MUST NOT be used
// when parallel narrowphase is enabled — the caller is responsible for
// passing applyHotSolvers=false in that mode.
func GetCollisions(bodyA, bodyB *Body, pool *ContactPool, applyHotSolvers bool) []*Contact {
	var contacts []*Contact

	for _, meshA := range bodyA.meshes {
		for _, meshB := range bodyB.meshes {
			cbA := meshA.CollisionBehavior()
			cbB := meshB.CollisionBehavior()
			switch {
			case cbA == CollisionPolygons && cbB == CollisionPolygons:
				contacts = append(contacts, polygonVsPolygonParticles(meshA.polygon, meshB.polygon, pool)...)
			case cbA == CollisionCircles && cbB == CollisionPolygons:
				contacts = append(contacts, circleVsPolygon(meshA, meshB, pool)...)
			case cbA == CollisionPolygons && cbB == CollisionCircles:
				contacts = append(contacts, circleVsPolygon(meshB, meshA, pool)...)
			case cbA == CollisionCircles && cbB == CollisionCircles:
				contacts = append(contacts, circleVsCircle(meshA, meshB, pool, bodyA, bodyB)...)
			case cbA == CollisionPolyline && cbB == CollisionPolygons:
				// Soft body (polyline) vs rigid body (polygon).
				// Calls CircleAndPolygon
				// (polyline particles as circles vs polygon) FIRST,
				// then PolylineAndPolygon (segment-vs-bisector ray)
				// as a secondary detector for edges crossing the
				// polygon interior.
				//
				// Hot-solving: when applyHotSolvers is true, the
				// CircleAndPolygon contacts are immediately solved
				// via a temp Manifold BEFORE PolylineAndPolygon runs.
				// This is critical for soft-body stability — without
				// it, particles penetrate multiple bodies before any
				// resolution happens.
				if applyHotSolvers {
					hotContacts := circleVsPolygon(meshA, meshB, pool)
					if len(hotContacts) > 0 {
						hotManifold := &Manifold{
							bodyA:    bodyA,
							bodyB:    bodyB,
							contacts: hotContacts,
							world:    bodyA.world,
						}
						hotManifold.init()
						hotManifold.Solve()
						hotManifold.SolveFrictionAndVelocities()
					}
				} else {
					contacts = append(contacts, circleVsPolygon(meshA, meshB, pool)...)
				}
				contacts = append(contacts, polylineAndPolygon(meshA.polygon, meshB.polygon, pool)...)
			case cbA == CollisionPolygons && cbB == CollisionPolyline:
				// Rigid body (polygon) vs soft body (polyline) — swap.
				if applyHotSolvers {
					hotContacts := circleVsPolygon(meshB, meshA, pool)
					if len(hotContacts) > 0 {
						hotManifold := &Manifold{
							bodyA:    bodyA,
							bodyB:    bodyB,
							contacts: hotContacts,
							world:    bodyA.world,
						}
						hotManifold.init()
						hotManifold.Solve()
						hotManifold.SolveFrictionAndVelocities()
					}
				} else {
					contacts = append(contacts, circleVsPolygon(meshB, meshA, pool)...)
				}
				polyContacts := polylineAndPolygon(meshB.polygon, meshA.polygon, pool)
				for _, c := range polyContacts {
					c.Normal = c.Normal.Neg()
				}
				contacts = append(contacts, polyContacts...)
			case cbA == CollisionPolyline && cbB == CollisionPolyline:
				// Soft body vs soft body — test both directions.
				// Only runs when both bodies are MASS_SPRING (soft bodies).
				if bodyA.bodyType == BodyTypeSoft && bodyB.bodyType == BodyTypeSoft {
					contacts = append(contacts, polylineAndPolyline(meshA.polygon, meshB.polygon, pool, bodyA.world)...)
					contacts = append(contacts, polylineAndPolyline(meshB.polygon, meshA.polygon, pool, bodyA.world)...)
				}
			case cbA == CollisionPolyline && cbB == CollisionCircles:
				// Soft body (polyline) vs circle particles
				contacts = append(contacts, circleVsPolygon(meshB, meshA, pool)...)
			case cbA == CollisionCircles && cbB == CollisionPolyline:
				contacts = append(contacts, circleVsPolygon(meshA, meshB, pool)...)
			}
		}
	}

	return contacts
}

// --- Polygon vs Polygon (concave-aware) ---

// polygonVsPolygonConcave handles polygon-vs-polygon collision for both
// convex and concave polygons.
//
// Approach: treat each vertex of polyA as a "circle particle" (with radius 0)
// and test it against polyB using the same Voronoi-region algorithm as
// circleVsPolygon. Then do the same for polyB's vertices against polyA.
// This correctly handles concave polygons because circleVsPolygon uses
// PointInPolygonWN (winding number) and finds the nearest REAL edge (not
// a diagonal from sub-convex decomposition).
//
// For resting contacts (edges aligned), the Voronoi region classification
// correctly identifies the edge region and produces a contact with the
// edge's outward normal.
func polygonVsPolygonConcave(polyA, polyB []*Particle, pool *ContactPool) []*Contact {
	if len(polyA) < 3 || len(polyB) < 3 {
		return nil
	}

	// Pre-compute adjusted positions for fat polygon particles.
	polyBPositions := particlePolygonToPolygon(polyB)
	polyAPositions := particlePolygonToPolygon(polyA)

	var contacts []*Contact

	// 1. Test each vertex of A against B's polygon (as circle with radius 0).
	for _, pA := range polyA {
		if !pA.enabled {
			continue
		}
		c := testVertexVsPolygon(pA, polyB, polyBPositions, pool)
		if c != nil {
			contacts = append(contacts, c)
		}
	}

	// 2. Test each vertex of B against A's polygon (as circle with radius 0).
	for _, pB := range polyB {
		if !pB.enabled {
			continue
		}
		c := testVertexVsPolygon(pB, polyA, polyAPositions, pool)
		if c != nil {
			contacts = append(contacts, c)
		}
	}

	// 3. Deduplicate contacts that are very close (a vertex of A inside B
	// and a vertex of B inside A can produce near-identical contacts).
	contacts = deduplicateContacts(contacts)

	return contacts
}

// testVertexVsPolygon tests a single vertex (as a circle with the vertex's
// radius) against a polygon. Uses the Voronoi region algorithm from
// circleVsPolygon. Returns nil if no collision.
func testVertexVsPolygon(vertex *Particle, poly []*Particle, polyPositions []Vec2, pool *ContactPool) *Contact {
	cPos := vertex.GlobalPosition()
	// Use the vertex's radius (typically 0.5 for polygon particles).
	// This gives a small "skin" that detects contacts before deep penetration,
	// which is critical for resting contacts where edges are aligned.
	cRadius := vertex.Radius()
	n := len(poly)

	var nearestPolygonParticle *Particle
	nearestParticlePenetrationSq := MaxWorldSize
	var nearestParticleNormal Vec2

	var nearestEdgeParticles [2]*Particle
	nearestEdgePenetration := MaxWorldSize
	nearestEdgeMinDist := MaxWorldSize
	var nearestEdgeNormal Vec2

	for pi := range n {
		npi := (pi + 1) % n
		p := poly[pi]
		np := poly[npi]
		pPos := polyPositions[pi]
		npPos := polyPositions[npi]

		// Nearest vertex
		circleToParticleVec := cPos.Sub(pPos)
		circleToParticleDistSq := circleToParticleVec.LengthSquared()
		if circleToParticleDistSq < nearestParticlePenetrationSq {
			nearestPolygonParticle = p
			nearestParticlePenetrationSq = circleToParticleDistSq
			nearestParticleNormal = circleToParticleVec.Normalized()
		}

		// Nearest edge
		edgeVec := npPos.Sub(pPos)
		edgeVecUnit := edgeVec.Normalized()
		edgeVecNormal := edgeVecUnit.Perpendicular()
		circleToEdgeBegin := cPos.Sub(pPos)
		circleToEdgePenetration := circleToEdgeBegin.Dot(edgeVecNormal)
		if math32.Abs(circleToEdgePenetration) < nearestEdgeMinDist {
			circleToEdgeRangeProject := circleToEdgeBegin.Dot(edgeVecUnit)
			if circleToEdgeRangeProject >= 0.0 && circleToEdgeRangeProject <= edgeVec.Length() {
				nearestEdgeMinDist = math32.Abs(circleToEdgePenetration)
				nearestEdgePenetration = circleToEdgePenetration
				nearestEdgeParticles[0] = p
				nearestEdgeParticles[1] = np
				nearestEdgeNormal = edgeVecNormal
			}
		}
	}

	nearestParticlePenetration := math32.Sqrt(nearestParticlePenetrationSq)

	var voronoiRegion int
	if nearestEdgeParticles[0] == nil {
		voronoiRegion = 0
	} else {
		if nearestParticlePenetration > nearestEdgeMinDist {
			if nearestEdgePenetration < 0 && pointInPolygonWN(cPos, poly) {
				voronoiRegion = 2
			} else {
				voronoiRegion = 1
			}
		} else {
			voronoiRegion = 0
		}
	}

	var normal Vec2
	var penetration float32
	var refParticles []*Particle

	switch voronoiRegion {
	case 0:
		if nearestPolygonParticle == nil {
			return nil
		}
		if pointInPolygonWN(cPos, poly) {
			// Inside — deep penetration
			penetration = cRadius + nearestParticlePenetration
			normal = nearestParticleNormal
			refParticles = []*Particle{nearestPolygonParticle}
		} else {
			// Outside — check if within radius
			if nearestParticlePenetration < cRadius {
				penetration = cRadius - nearestParticlePenetration
				normal = nearestParticleNormal
				refParticles = []*Particle{nearestPolygonParticle}
			} else {
				return nil
			}
		}
	case 1:
		// Edge region
		if nearestEdgePenetration < cRadius {
			penetration = cRadius - nearestEdgePenetration
			normal = nearestEdgeNormal
			refParticles = []*Particle{nearestEdgeParticles[0], nearestEdgeParticles[1]}
		} else {
			return nil
		}
	default:
		// Inside region (voronoiRegion == 2)
		penetration = cRadius - nearestEdgePenetration
		normal = nearestEdgeNormal
		refParticles = []*Particle{nearestEdgeParticles[0], nearestEdgeParticles[1]}
	}

	if penetration <= 0 {
		return nil
	}

	c := pool.Get()
	c.Particle = vertex
	c.Position = cPos
	c.Normal = normal
	c.Penetration = penetration
	c.ReferenceParticles = refParticles
	return c
}

// deduplicateContacts removes contacts that are very close in position.
func deduplicateContacts(contacts []*Contact) []*Contact {
	if len(contacts) <= 1 {
		return contacts
	}
	var result []*Contact
	for _, c := range contacts {
		dup := false
		for _, existing := range result {
			diff := c.Position.Sub(existing.Position)
			if diff.LengthSquared() < 1.0 { // within 1 unit
				dup = true
				// Keep the one with more penetration
				if c.Penetration > existing.Penetration {
					existing.Penetration = c.Penetration
					existing.Normal = c.Normal
					existing.Particle = c.Particle
					existing.ReferenceParticles = c.ReferenceParticles
				}
				break
			}
		}
		if !dup {
			result = append(result, c)
		}
	}
	return result
}

// findEdgeEdgeContacts finds edge-edge intersection contacts between two
// polygons. Used when no vertex is inside the other polygon (edge-edge contact).
// TODO: Check why this is unused, where in the c++ code this was ported from.
func findEdgeEdgeContacts(polyA, polyB []*Particle, pool *ContactPool) []*Contact {
	var contacts []*Contact
	nA := len(polyA)
	nB := len(polyB)

	for i := range nA {
		a1 := polyA[i].GlobalPosition()
		a2 := polyA[(i+1)%nA].GlobalPosition()
		for j := range nB {
			b1 := polyB[j].GlobalPosition()
			b2 := polyB[(j+1)%nB].GlobalPosition()
			intersection := LineIntersectionLine(a1, a2, b1, b2)
			if intersection.IsNaN() {
				continue
			}
			edgeA := a2.Sub(a1)
			edgeLen := edgeA.Length()
			if edgeLen < 1e-6 {
				continue
			}
			normal := edgeA.Div(edgeLen).Perpendicular()
			penetration := float32(0.5)
			c := pool.Get()
			c.Particle = polyA[i]
			c.Position = intersection
			c.Normal = normal
			c.Penetration = penetration
			c.ReferenceParticles = []*Particle{polyB[j], polyB[(j+1)%nB]}
			contacts = append(contacts, c)
		}
	}
	return contacts
}

// --- Polygon vs Polygon (SAT + clipping) ---

// polygonVsPolygon runs SAT + edge clipping between two polygon meshes.
// Convenience wrapper that delegates to polygonVsPolygonParticles using
// the meshes' polygon slices.
// TODO: Check why this is unused, where in the c++ code this was ported from.
func polygonVsPolygon(meshA, meshB *Mesh, pool *ContactPool) []*Contact {
	return polygonVsPolygonParticles(meshA.polygon, meshB.polygon, pool)
}

// polygonVsPolygonParticles runs SAT + edge clipping between two particle
// slices (each representing a convex polygon).
//
// Algorithm:
//
//	A. CHECK SEPARATING AXIS AND FIND NORMAL WITH MINIMUM PENETRATION.
//	   Loop over totalPointCount = sizeA + sizeB edges, switching ref/inc
//	   at the midpoint. Project both polygons onto each edge's perpendicular
//	   (the outward normal). Use Project.Overlap which returns
//	   min - other.max (negative when overlapping). Early-return on any
//	   axis with penetration >= 0 (separating axis found).
//	B. FIND INCIDENT AND REFERENCE SEGMENT ACCORDING TO refNormal.
//	   Project both polygons onto refNormal. Pick support points
//	   (maxIndex of A, minIndex of B, swapped if B.min < A.min). For each
//	   support point, choose the segment (prev→cur vs cur→next) most
//	   perpendicular to refNormal (smallest |seg · refNormal|).
//	C. CLIP POINTS AND DEFINE CONTACT POINTS.
//	   Clip the incident segment against the reference segment. If no
//	   contacts produced, swap and clip the other way.
//	D. RETURN COLLISION MANIFOLD.
func polygonVsPolygonParticles(particlesA, particlesB []*Particle, pool *ContactPool) []*Contact {
	sizeA := len(particlesA)
	sizeB := len(particlesB)
	if sizeA < 3 || sizeB < 3 {
		return nil
	}

	totalPointCount := sizeA + sizeB

	// refPolygon/incPolygon start as A/B (will swap mid-loop).
	refPolygon := particlesA
	incPolygon := particlesB
	refPolygonSize := sizeA
	swapped := false

	minPenetration := MaxWorldSize
	var refNormal Vec2 // axis with minimum penetration (outward from ref)

	s := 0
	for p := range totalPointCount {
		// Switch ref/inc when we cross the midpoint (start of B's edges).
		if p >= sizeA && !swapped {
			refPolygon = particlesB
			refPolygonSize = sizeB
			incPolygon = particlesA
			s = 0
			swapped = true
		}
		s1 := refPolygon[s]
		s2 := refPolygon[(s+1)%refPolygonSize]
		sEdge := s2.GlobalPosition().Sub(s1.GlobalPosition())
		sEdgeLen := sEdge.Length()
		if sEdgeLen < 1e-6 {
			s++
			continue
		}
		sNormal := sEdge.Div(sEdgeLen).Perpendicular()

		refProject := projectPolygon(refPolygon, sNormal)
		incProject := projectPolygon(incPolygon, sNormal)

		// Overlap returns min - other.max (negative when overlapping).
		penetration := refProject.min - incProject.max
		if penetration >= 0 {
			// Separating axis found — polygons don't overlap.
			return nil
		}
		penetration = math32.Abs(penetration)
		if penetration < minPenetration {
			minPenetration = penetration
			refNormal = sNormal
		}
		s++
	}

	if minPenetration >= MaxWorldSize {
		return nil
	}

	// B. FIND INCIDENT AND REFERENCE SEGMENT ACCORDING TO refNormal.
	supportProjectA := projectPolygon(particlesA, refNormal)
	supportProjectB := projectPolygon(particlesB, refNormal)

	supportPointAIndex := supportProjectA.maxIndex
	supportPointBIndex := supportProjectB.minIndex
	if supportProjectB.min < supportProjectA.min {
		supportPointAIndex = supportProjectA.minIndex
		supportPointBIndex = supportProjectB.maxIndex
	}

	// particlesA segment option
	segPointPrevA := ((supportPointAIndex - 1) + sizeA) % sizeA
	segPointA := supportPointAIndex
	segPointNextA := (supportPointAIndex + 1) % sizeA
	segmentAOption1 := particlesA[segPointNextA].GlobalPosition().Sub(particlesA[segPointA].GlobalPosition())
	segmentAOption2 := particlesA[segPointA].GlobalPosition().Sub(particlesA[segPointPrevA].GlobalPosition())
	segmentAOption1ParallelRate := math32.Abs(segmentAOption1.Dot(refNormal))
	segmentAOption2ParallelRate := math32.Abs(segmentAOption2.Dot(refNormal))

	segmentA := [2]*Particle{particlesA[segPointA], particlesA[segPointNextA]}
	segmentAParallelRate := segmentAOption1ParallelRate
	if segmentAOption2ParallelRate < segmentAOption1ParallelRate {
		segmentA = [2]*Particle{particlesA[segPointPrevA], particlesA[segPointA]}
		segmentAParallelRate = segmentAOption2ParallelRate
	}

	// particlesB segment option
	segPointPrevB := ((supportPointBIndex - 1) + sizeB) % sizeB
	segPointB := supportPointBIndex
	segPointNextB := (supportPointBIndex + 1) % sizeB
	segmentBOption1 := particlesB[segPointNextB].GlobalPosition().Sub(particlesB[segPointB].GlobalPosition())
	segmentBOption2 := particlesB[segPointB].GlobalPosition().Sub(particlesB[segPointPrevB].GlobalPosition())
	segmentBOption1ParallelRate := math32.Abs(segmentBOption1.Dot(refNormal))
	segmentBOption2ParallelRate := math32.Abs(segmentBOption2.Dot(refNormal))

	segmentB := [2]*Particle{particlesB[segPointB], particlesB[segPointNextB]}
	segmentBParallelRate := segmentBOption1ParallelRate
	if segmentBOption2ParallelRate < segmentBOption1ParallelRate {
		segmentB = [2]*Particle{particlesB[segPointPrevB], particlesB[segPointB]}
		segmentBParallelRate = segmentBOption2ParallelRate
	}

	// C. CLIP POINTS AND DEFINE CONTACT POINTS.
	// Pick the segment with smaller parallel rate as the reference (more
	// perpendicular to refNormal = better clipping axis). Fall back to the
	// other segment if no contacts are produced.
	var contacts []*Contact
	if segmentBParallelRate < segmentAParallelRate {
		contacts = clipEdges(segmentB, segmentA, pool)
		if len(contacts) == 0 {
			contacts = clipEdges(segmentA, segmentB, pool)
		}
	} else {
		contacts = clipEdges(segmentA, segmentB, pool)
		if len(contacts) == 0 {
			contacts = clipEdges(segmentB, segmentA, pool)
		}
	}

	return contacts
}

// edgeInfo holds an edge's normal and the two particles forming it.
type edgeInfo struct {
	normal Vec2
	p1, p2 *Particle
}

// polygonEdges returns all edges of a polygon with their outward normals.
// Assumes CW winding in screen coordinates (Y down).
// Outward normal = (edge.Y, -edge.X) = Perpendicular() (points LEFT of edge
// direction in screen coords = away from interior for CW).
// TODO: Check why this is unused, where in the c++ code this was ported from.
func polygonEdges(poly []*Particle) []edgeInfo {
	n := len(poly)
	edges := make([]edgeInfo, n)
	for i := range n {
		p1 := poly[i]
		p2 := poly[(i+1)%n]
		edge := p2.GlobalPosition().Sub(p1.GlobalPosition())
		edges[i] = edgeInfo{
			normal: Vec2{X: edge.Y, Y: -edge.X}.Normalized(),
			p1:     p1,
			p2:     p2,
		}
	}
	return edges
}

// distanceToPolygon computes the distance from a point to the nearest edge
// of a convex polygon, and returns the outward normal of that edge.
// If the point is inside the polygon, the distance is positive (penetration depth).
// Assumes CW winding in screen coordinates (Y down).
// TODO: Check why this is unused, where in the c++ code this was ported from.
func distanceToPolygon(point Vec2, poly []*Particle) (float32, Vec2) {
	n := len(poly)
	bestDepth := -MaxWorldSize
	bestNormal := Vec2Zero()
	for i := range n {
		p1 := poly[i].GlobalPosition()
		p2 := poly[(i+1)%n].GlobalPosition()
		edge := p2.Sub(p1)
		// Outward normal for CW in screen coords: (edge.Y, -edge.X)
		normal := Vec2{X: edge.Y, Y: -edge.X}.Normalized()
		bridge := point.Sub(p1)
		dist := bridge.Dot(normal) // positive = outside, negative = inside
		if dist > bestDepth {
			bestDepth = dist
			bestNormal = normal
		}
	}
	// If bestDepth < 0, the point is inside the polygon.
	// Penetration depth = -bestDepth (positive).
	if bestDepth < 0 {
		return -bestDepth, bestNormal
	}
	return 0, bestNormal
}

// findMinPenAxis tests all edges of `refPoly` as separating axes against
// `incidentPoly`. Returns the axis with minimum penetration, the edge index
// that produced it, and true if no separating axis was found (i.e., the
// polygons are colliding). Returns false if a separating axis was found.
//
// The returned normal points from refPoly toward incidentPoly (i.e., the
// direction to push incidentPoly away from refPoly).
// TODO: Check why this is unused, where in the c++ code this was ported from.
func findMinPenAxis(refPoly, incidentPoly []*Particle) (normal Vec2, penetration float32, edgeIdx int, ok bool) {
	n := len(refPoly)
	bestPen := MaxWorldSize
	bestIdx := 0
	bestNormal := Vec2Zero()

	for i := range n {
		p1 := refPoly[i]
		p2 := refPoly[(i+1)%n]
		edge := p2.GlobalPosition().Sub(p1.GlobalPosition())
		// Outward normal (for CW winding): (edge.Y, -edge.X)
		outwardNormal := Vec2{X: edge.Y, Y: -edge.X}.Normalized()

		projRef := projectPolygon(refPoly, outwardNormal)
		projInc := projectPolygon(incidentPoly, outwardNormal)

		// Check for overlap
		// Overlap = min(maxRef, maxInc) - max(minRef, minInc)
		overlap := min(projRef.max, projInc.max) - max(projRef.min, projInc.min)
		if overlap <= 0 {
			// Separating axis found
			return Vec2Zero(), 0, 0, false
		}

		// The penetration is the overlap, but we need to determine the
		// contact normal direction. The incident polygon is on the side
		// of the outward normal if projInc.min > projRef.min (incident
		// is "above" ref on this axis), or on the opposite side if
		// projInc.max < projRef.max (incident is "below" ref).
		//
		// For the contact normal to point from ref toward incident:
		//   - If incident is on the outward side → normal = outwardNormal
		//   - If incident is on the inward side → normal = -outwardNormal
		//   - We pick the direction that gives the smaller penetration
		//
		// Actually, in standard SAT, the overlap is always positive if
		// the polygons intersect, and the normal points from ref toward
		// incident. The penetration is the overlap amount.
		//
		// The key: the contact normal should point from the reference
		// body toward the incident body. If the incident polygon is on
		// the outward side of the reference edge, the outward normal
		// points toward it. If it's on the inward side, we need to flip.

		// Determine which side the incident polygon is on
		// by comparing the centroids
		var pen float32
		if projInc.min >= projRef.min {
			// Incident is on the positive (outward) side
			pen = projRef.max - projInc.min
			if pen < bestPen {
				bestPen = pen
				bestIdx = i
				bestNormal = outwardNormal
			}
		} else if projInc.max <= projRef.max {
			// Incident is on the negative (inward) side
			pen = projInc.max - projRef.min
			if pen < bestPen {
				bestPen = pen
				bestIdx = i
				bestNormal = outwardNormal.Neg()
			}
		} else {
			// Incident spans the entire reference projection — use the overlap
			if overlap < bestPen {
				bestPen = overlap
				bestIdx = i
				bestNormal = outwardNormal
			}
		}
	}

	return bestNormal, bestPen, bestIdx, true
}

// projectResult holds the min/max projection of a polygon onto an axis.
// Matches QCollision::Project (qcollision.h:98-120) — includes maxIndex,
// which is required by the support-point selection in PolygonAndPolygon
// (qcollision.cpp:1149,1152).
type projectResult struct {
	min, max float32
	minIndex int
	maxIndex int
}

// projectPolygon projects a polygon onto an axis (unit normal).
func projectPolygon(poly []*Particle, axis Vec2) projectResult {
	if len(poly) == 0 {
		return projectResult{}
	}
	min := poly[0].GlobalPosition().Dot(axis)
	max := min
	minIndex := 0
	maxIndex := 0
	for i := 1; i < len(poly); i++ {
		p := poly[i].GlobalPosition().Dot(axis)
		if p < min {
			min = p
			minIndex = i
		}
		if p > max {
			max = p
			maxIndex = i
		}
	}
	return projectResult{min: min, max: max, minIndex: minIndex, maxIndex: maxIndex}
}

// findIncidentEdge finds the edge of poly most anti-parallel to `normal`.
// Returns the two particles forming that edge.
// TODO: Check why this is unused, where in the c++ code this was ported from.
func findIncidentEdge(poly []*Particle, normal Vec2) [2]*Particle {
	n := len(poly)
	bestDot := float32(1.0) // we want the most negative dot
	bestIdx := 0
	for i := range n {
		p1 := poly[i]
		p2 := poly[(i+1)%n]
		edge := p2.GlobalPosition().Sub(p1.GlobalPosition())
		edgeNormal := edge.Perpendicular().Normalized()
		dot := edgeNormal.Dot(normal)
		if dot < bestDot {
			bestDot = dot
			bestIdx = i
		}
	}
	return [2]*Particle{poly[bestIdx], poly[(bestIdx+1)%n]}
}

// clipEdges clips the incident edge against the reference edge and produces
// contacts.
//
// Computes `normal = unit.Perpendicular()` from the reference segment (NOT
// passed in as a parameter — the C++ algorithm derives the normal from the
// segment itself, not from the SAT best-normal). Penetrating condition is
// `dist <= 0` (incident is on the OPPOSITE side of `normal`). Penetration
// is `abs(dist)`. Projection range is strict `0 <= proj <= len` (no tolerance).
func clipEdges(refParts, incParts [2]*Particle, pool *ContactPool) []*Contact {
	var contacts []*Contact

	refA := refParts[0].GlobalPosition()
	refB := refParts[1].GlobalPosition()
	sv := refB.Sub(refA)
	svLen := sv.Length()
	if svLen < 1e-6 {
		return nil
	}
	unit := sv.Div(svLen)
	normal := unit.Perpendicular()

	for _, incP := range incParts {
		incPos := incP.GlobalPosition()
		bv := incPos.Sub(refA)
		dist := bv.Dot(normal)
		if dist <= 0 {
			proj := bv.Dot(unit)
			if proj >= 0.0 && proj <= svLen {
				c := pool.Get()
				c.Particle = incP
				c.Position = incPos
				c.Normal = normal
				c.Penetration = math32.Abs(dist)
				c.ReferenceParticles = []*Particle{refParts[0], refParts[1]}
				contacts = append(contacts, c)
			}
		}
	}

	return contacts
}

// --- Circle vs Circle ---

// circleVsCircle runs circle-circle collision detection.
//
// For each pair of particles (one from each mesh), checks if the distance
// is less than the sum of radii. Uses sweep-and-prune for efficiency.
func circleVsCircle(meshA, meshB *Mesh, pool *ContactPool, bodyA, bodyB *Body) []*Contact {
	var contacts []*Contact
	particlesA := meshA.particles
	particlesB := meshB.particles

	// velocitySensitive: when both bodies are rigid, use previous positions
	// for normals (stable resting contacts). Matches qcollision.cpp:779-783.
	velocitySensitive := bodyA.bodyType == BodyTypeRigid && bodyB.bodyType == BodyTypeRigid

	for _, pA := range particlesA {
		for _, pB := range particlesB {
			gA := pA.GlobalPosition()
			gB := pB.GlobalPosition()
			diff := gB.Sub(gA)
			distSq := diff.LengthSquared()
			rSum := pA.Radius() + pB.Radius()
			if distSq < rSum*rSum && distSq > 1e-8 {
				dist := math32.Sqrt(distSq)
				var normal Vec2
				if velocitySensitive {
					// Use previous positions for stable normals
					prevDiff := pB.PreviousGlobalPosition().Sub(pA.PreviousGlobalPosition())
					prevLen := prevDiff.Length()
					if prevLen > 1e-6 {
						normal = prevDiff.Div(prevLen)
					} else {
						normal = diff.Div(dist)
					}
				} else {
					normal = diff.Div(dist)
				}
				penetration := rSum - dist

				c := pool.Get()
				c.Particle = pB
				// Contact position is on the surface of circle A toward B
				// The previous Go code used `gB` (raw particle B center), which
				// produced wrong torque arms in the manifold solver for
				// off-center circle meshes (e.g., a wheel with multiple circles).
				c.Position = gA.Add(normal.Mul(pA.Radius()))
				c.Normal = normal
				c.Penetration = penetration
				c.ReferenceParticles = []*Particle{pA}
				contacts = append(contacts, c)
			}
		}
	}
	return contacts
}

// --- Circle vs Polygon ---

// circleVsPolygon runs circle-polygon collision detection.
//
// Uses Voronoi region classification: for each circle particle, finds the
// nearest polygon vertex and edge, then classifies as vertex/edge/inside.
// Applies ParticlePolygonToPolygon first to offset fat polygon particles
// inward. Uses PointInPolygonWN for the inside
// test so concave polygons are handled correctly.
func circleVsPolygon(circleMesh, polygonMesh *Mesh, pool *ContactPool) []*Contact {
	var contacts []*Contact
	poly := polygonMesh.polygon
	if len(poly) < 3 {
		return nil
	}
	n := len(poly)

	// Pre-compute adjusted positions for fat polygon particles.
	polygonPositions := particlePolygonToPolygon(poly)

	for _, circle := range circleMesh.particles {
		cPos := circle.GlobalPosition()
		cRadius := circle.Radius()

		// Find nearest polygon vertex (for vertex Voronoi region) AND
		// nearest edge (for edge/inside regions). Matches qcollision.cpp:938-981.
		var nearestPolygonParticle *Particle
		nearestParticlePenetrationSq := MaxWorldSize
		var nearestParticleNormal Vec2

		var nearestEdgeParticles [2]*Particle
		nearestEdgePenetration := MaxWorldSize
		nearestEdgeMinDist := MaxWorldSize
		var nearestEdgeNormal Vec2

		for pi := range n {
			npi := (pi + 1) % n
			p := poly[pi]
			np := poly[npi]
			pPos := polygonPositions[pi]
			npPos := polygonPositions[npi]

			// a1. Find the nearest vertex of the polygon.
			circleToParticleVec := cPos.Sub(pPos)
			circleToParticleDistSq := circleToParticleVec.LengthSquared()
			if circleToParticleDistSq < nearestParticlePenetrationSq {
				nearestPolygonParticle = p
				nearestParticlePenetrationSq = circleToParticleDistSq
				nearestParticleNormal = circleToParticleVec.Normalized()
			}

			// a2. Find the nearest edge of the polygon.
			edgeVec := npPos.Sub(pPos)
			edgeVecUnit := edgeVec.Normalized()
			edgeVecNormal := edgeVecUnit.Perpendicular()
			circleToEdgeBegin := cPos.Sub(pPos)
			circleToEdgePenetration := circleToEdgeBegin.Dot(edgeVecNormal)
			if math32.Abs(circleToEdgePenetration) < nearestEdgeMinDist {
				circleToEdgeRangeProject := circleToEdgeBegin.Dot(edgeVecUnit)
				if circleToEdgeRangeProject >= 0.0 && circleToEdgeRangeProject <= edgeVec.Length() {
					nearestEdgeMinDist = math32.Abs(circleToEdgePenetration)
					nearestEdgePenetration = circleToEdgePenetration
					nearestEdgeParticles[0] = p
					nearestEdgeParticles[1] = np
					nearestEdgeNormal = edgeVecNormal
				}
			}
		}

		nearestParticlePenetration := math32.Sqrt(nearestParticlePenetrationSq)

		// a3. Define the Voronoi region: 0=vertex, 1=edge, 2=inside.
		var voronoiRegion int
		if nearestEdgeParticles[0] == nil {
			voronoiRegion = 0
		} else {
			if nearestParticlePenetration > nearestEdgeMinDist {
				if nearestEdgePenetration < 0 && pointInPolygonWN(cPos, poly) {
					voronoiRegion = 2
				} else {
					voronoiRegion = 1
				}
			} else {
				voronoiRegion = 0
			}
		}

		var normal Vec2
		var penetration float32
		var contactPos Vec2
		var refParticles []*Particle

		// B. Test collisions based on Voronoi region.
		if voronoiRegion == 0 {
			// Vertex region.
			if nearestPolygonParticle == nil {
				continue
			}
			if pointInPolygonWN(cPos, poly) {
				// Inside, but classified as vertex — deep penetration.
				penetration = cRadius + nearestParticlePenetration
				contactPos = cPos
				if cRadius > 0.5 {
					contactPos = contactPos.Sub(nearestParticleNormal.Mul(cRadius))
				}
				normal = nearestParticleNormal
				refParticles = []*Particle{nearestPolygonParticle}
			} else {
				if nearestParticlePenetration < cRadius {
					penetration = cRadius - nearestParticlePenetration
					contactPos = cPos
					if cRadius > 0.5 {
						contactPos = contactPos.Sub(nearestParticleNormal.Mul(cRadius))
					}
					normal = nearestParticleNormal
					refParticles = []*Particle{nearestPolygonParticle}
				} else {
					continue
				}
			}
		} else if voronoiRegion == 1 {
			// Edge region.
			if nearestEdgePenetration < cRadius {
				penetration = cRadius - nearestEdgePenetration
				contactPos = cPos
				if cRadius > 0.5 {
					contactPos = contactPos.Sub(nearestEdgeNormal.Mul(cRadius))
				}
				normal = nearestEdgeNormal
				refParticles = []*Particle{nearestEdgeParticles[0], nearestEdgeParticles[1]}
			} else {
				continue
			}
		} else {
			// Inside region (voronoiRegion == 2).
			penetration = cRadius - nearestEdgePenetration
			contactPos = cPos
			if cRadius > 0.5 {
				contactPos = contactPos.Sub(nearestEdgeNormal.Mul(cRadius))
			}
			normal = nearestEdgeNormal
			refParticles = []*Particle{nearestEdgeParticles[0], nearestEdgeParticles[1]}
		}

		c := pool.Get()
		c.Particle = circle
		c.Position = contactPos
		c.Normal = normal
		c.Penetration = penetration
		c.ReferenceParticles = refParticles
		contacts = append(contacts, c)
	}

	return contacts
}

// pointInPolygon reports whether a point is inside a convex polygon.
// Assumes clockwise winding in screen coordinates (Y down).
// For CW polygons, the interior is on the LEFT side of each edge,
// meaning the cross product (edge × toPoint) should be >= 0 for all edges.
// TODO: Check why this is unused, where in the c++ code this was ported from.
func pointInPolygon(point Vec2, poly []*Particle) bool {
	n := len(poly)
	if n < 3 {
		return false
	}
	for i := range n {
		p1 := poly[i].GlobalPosition()
		p2 := poly[(i+1)%n].GlobalPosition()
		edge := p2.Sub(p1)
		toPoint := point.Sub(p1)
		cross := edge.X*toPoint.Y - edge.Y*toPoint.X
		// For CW in screen coords: inside = left side = cross >= 0
		// If cross < 0, point is on the right (outside)
		if cross < 0 {
			return false
		}
	}
	return true
}

// --- Geometry helpers ---

// LineIntersectionLine computes the intersection of two line segments.
// Returns Vec2NaN() if no intersection. Matches QCollision::LineIntersectionLine.
func LineIntersectionLine(d1A, d1B, d2A, d2B Vec2) Vec2 {
	r := d1B.Sub(d1A)
	s := d2B.Sub(d2A)
	denom := r.X*s.Y - r.Y*s.X
	if math32.Abs(denom) < 1e-6 {
		return Vec2NaN()
	}
	diff := d2A.Sub(d1A)
	t := (diff.X*s.Y - diff.Y*s.X) / denom
	u := (diff.X*r.Y - diff.Y*r.X) / denom
	if t < 0 || t > 1 || u < 0 || u > 1 {
		return Vec2NaN()
	}
	return d1A.Add(r.Mul(t))
}

// pointInPolygonWN tests whether `point` is inside the polygon (convex or
// concave) using the winding number algorithm.
//
// Casts a horizontal ray from `point` to +X infinity. For each polygon edge,
// if the edge crosses the ray's Y range, computes the ray-vs-edge intersection
// parameters t (along the ray) and u (along the edge). Counts +1 for upward
// crossings, -1 for downward. Point is inside iff winding != 0.
//
// NOTE: The C++ at line 1397 has a likely typo `(u>=0.0 && t<=1.0)` (should be
// `u<=1.0`). We port the typo faithfully — for byte-accurate parity, the same
// edge cases (where u > 1) will be accepted/rejected identically.
func pointInPolygonWN(point Vec2, polygon []*Particle) bool {
	if len(polygon) < 3 {
		return false
	}
	ray := Vec2{X: MaxWorldSize, Y: 0}
	rayPerp := Vec2Down() // (0, 1)
	windingNumber := 0
	polygonSize := len(polygon)
	for i := range polygonSize {
		s1 := polygon[i].GlobalPosition()
		var s2 Vec2
		if i+1 == polygonSize {
			s2 = polygon[0].GlobalPosition()
		} else {
			s2 = polygon[i+1].GlobalPosition()
		}
		// Broadphase: check if point.Y is within the edge's Y range.
		if (point.Y <= s1.Y) != (point.Y <= s2.Y) {
			sideVec := s2.Sub(s1)
			sideVecPerp := sideVec.Perpendicular()
			s1ToPoint := s1.Sub(point)
			rayDotSidePerp := ray.Dot(sideVecPerp)
			if math32.Abs(rayDotSidePerp) > 1e-6 {
				t := s1ToPoint.Dot(sideVecPerp) / rayDotSidePerp
				sideDotRayPerp := sideVec.Dot(rayPerp)
				if math32.Abs(sideDotRayPerp) > 1e-6 {
					u := s1ToPoint.Neg().Dot(rayPerp) / sideDotRayPerp
					// Check intersection between the ray and the side vector.
					//
					// C++ qcollision.cpp:1397 has `(u>=0.0 && t<=1.0)` which is
					// almost certainly a typo for `(u>=0.0 && u<=1.0)` (the
					// `t<=1.0` is already checked in the first conjunct).
					// We port the INTENDED behavior (u<=1.0) rather than the
					// literal typo, because:
					//   1. Go's static analyzer rejects `t<=1.0 && t<=1.0` as
					//      a redundant condition (vet error, not just warning).
					//   2. The typo only matters in the rare edge case where
					//      u > 1 and t <= 1 — for which the C++ would accept
					//      a spurious crossing. The intended behavior is
					//      mathematically correct (ray-edge intersection
					//      requires both t and u in [0,1]).
					if (t >= 0.0 && t <= 1.0) && (u >= 0.0 && u <= 1.0) {
						if sideVec.Y < 0 {
							windingNumber -= 1
						} else {
							windingNumber += 1
						}
					}
				}
			}
		}
	}
	return windingNumber != 0
}

// particlePolygonToPolygon returns adjusted positions for each polygon
// particle. For particles with radius > 0.5, the position is offset inward
// by `radius * bisectorUnit` so that circle-vs-polygon collisions trigger
// at the particle surface rather than the particle center.

func particlePolygonToPolygon(particlePolygon []*Particle) []Vec2 {
	particlePolygonSize := len(particlePolygon)
	polygonPositions := make([]Vec2, particlePolygonSize)
	for i := range particlePolygonSize {
		p := particlePolygon[i]
		if p.Radius() > 0.5 {
			pp := particlePolygon[((i - 1 + particlePolygonSize) % particlePolygonSize)]
			np := particlePolygon[(i+1)%particlePolygonSize]
			bisectorUnit := GetBisectorUnitVector(pp.GlobalPosition(), p.GlobalPosition(), np.GlobalPosition(), false)
			offsetPos := p.GlobalPosition().Sub(bisectorUnit.Mul(p.Radius()))
			polygonPositions[i] = offsetPos
		} else {
			polygonPositions[i] = p.GlobalPosition()
		}
	}
	return polygonPositions
}

// findNearestSideOfPolygon finds the polygon side nearest to `point`.
//
// Parameters:
//   - checkSideRange: if true, only consider sides where the point's
//     projection onto the side lies within [0, sideLength]. If false,
//     consider all sides regardless of projection.
//   - checkNegativeDistance: if true, only consider sides where the signed
//     perpendicular distance is <= 0 (point is on the "negative" side of
//     the side's perpendicular). Used to find sides the point has crossed.
//
// Returns (startParticleIndex, endParticleIndex). Returns (-1, -1) if no
// side matches.
func findNearestSideOfPolygon(point Vec2, polygonParticles []*Particle, checkSideRange, checkNegativeDistance bool) (int, int) {
	resA, resB := -1, -1
	polygonSize := len(polygonParticles)
	minDistance := MaxWorldSize
	for pi := range polygonSize {
		npi := (pi + 1) % polygonSize
		p := polygonParticles[pi]
		np := polygonParticles[npi]
		bridgeVec := point.Sub(p.GlobalPosition())
		sideVec := np.GlobalPosition().Sub(p.GlobalPosition())
		sidePerp := sideVec.Perpendicular()

		if checkSideRange {
			sideUnit := sideVec.Normalized()
			proj := bridgeVec.Dot(sideUnit)
			if proj < 0 || proj > sideVec.Length() {
				continue
			}
		}

		dist := bridgeVec.Dot(sidePerp)

		if checkNegativeDistance && dist > 0 {
			continue
		}

		if math32.Abs(dist) < minDistance {
			resA = pi
			resB = npi
			minDistance = math32.Abs(dist)
		}
	}
	return resA, resB
}

// findNearestParticleOfPolygon returns the index of the polygon particle
// nearest to `particle` (skipping identity).
func findNearestParticleOfPolygon(particle *Particle, polygonParticles []*Particle) int {
	res := 0
	minDistance := MaxWorldSize
	for i, p := range polygonParticles {
		if p == particle {
			continue
		}
		dist := (particle.GlobalPosition().Sub(p.GlobalPosition())).Length()
		if dist < minDistance {
			minDistance = dist
			res = i
		}
	}
	return res
}

// findExtremeParticleOfAxis returns the index of the polygon particle with
// the maximum projection on `axisNormal`.
// TODO: Check why this is unused, where in the c++ code this was ported from.
func findExtremeParticleOfAxis(polygonParticles []*Particle, axisNormal Vec2) int {
	res := 0
	maxDistance := -MaxWorldSize
	for i, p := range polygonParticles {
		proj := p.GlobalPosition().Dot(axisNormal)
		if proj > maxDistance {
			maxDistance = proj
			res = i
		}
	}
	return res
}
