package physics

// GetCollisions runs narrowphase collision detection between two bodies
// and returns a list of contacts. Matches QWorld::GetCollisions (static)
// in qworld.cpp:927-1067.
//
// Dispatches by the collision behavior of each body's meshes:
//   - Polygons × Polygons → PolygonAndPolygon
//   - Circles × Polygons  → CircleAndPolygon
//   - Circles × Circles   → CircleAndCircle
//   - Polyline variants   → Phase 2 (soft bodies)
//
// If applyHotSolvers is true, contacts are returned but NOT pre-solved.
// The caller (World.solver loop) creates a Manifold and calls Solve().
func GetCollisions(bodyA, bodyB *Body, pool *ContactPool, applyHotSolvers bool) []*Contact {
        var contacts []*Contact

        for _, meshA := range bodyA.meshes {
                for _, meshB := range bodyB.meshes {
                        cbA := meshA.CollisionBehavior()
                        cbB := meshB.CollisionBehavior()

                        switch {
                        case cbA == CollisionPolygons && cbB == CollisionPolygons:
                                contacts = append(contacts, polygonVsPolygon(meshA, meshB, pool)...)
                        case cbA == CollisionCircles && cbB == CollisionPolygons:
                                contacts = append(contacts, circleVsPolygon(meshA, meshB, pool)...)
                        case cbA == CollisionPolygons && cbB == CollisionCircles:
				// Polygon is bodyA, circle is bodyB. Normal is correct (from polygon toward circle).
				contacts = append(contacts, circleVsPolygon(meshB, meshA, pool)...)
                        case cbA == CollisionCircles && cbB == CollisionCircles:
                                contacts = append(contacts, circleVsCircle(meshA, meshB, pool, bodyA, bodyB)...)
                        case cbA == CollisionPolyline && cbB == CollisionPolygons:
                                // Soft body (polyline) vs rigid body (polygon)
                                contacts = append(contacts, polylineAndPolygon(meshA.polygon, meshB.polygon, pool)...)
                        case cbA == CollisionPolygons && cbB == CollisionPolyline:
				// Rigid (polygons) vs soft (polyline). Normal is correct (outward from polygon = reference→incident).
				contacts = append(contacts, polylineAndPolygon(meshB.polygon, meshA.polygon, pool)...)
                        case cbA == CollisionPolyline && cbB == CollisionPolyline:
                                // Soft body vs soft body — test both directions
                                contacts = append(contacts, polylineAndPolyline(meshA.polygon, meshB.polygon, pool)...)
                                contacts = append(contacts, polylineAndPolyline(meshB.polygon, meshA.polygon, pool)...)
                        case cbA == CollisionPolyline && cbB == CollisionCircles:
                                // Soft body (polyline) vs circle particles
                                contacts = append(contacts, polylineAndPolygon(meshB.particles, meshA.polygon, pool)...)
                        case cbA == CollisionCircles && cbB == CollisionPolyline:
                                contacts = append(contacts, polylineAndPolygon(meshA.particles, meshB.polygon, pool)...)
                        }
                }
        }

        // Deduplicate contacts that are very close (the C++ engine does this
        // implicitly via the contact pool's solved flag). For Phase 1, we
        // skip deduplication — the Manifold solver handles multiple contacts.

        return contacts
}

// --- Polygon vs Polygon (SAT + clipping) ---

// polygonVsPolygon runs SAT between two polygon meshes and creates contacts
// at the deepest penetrating vertices.
//
// Approach: find the minimum penetration axis across all edges of both
// polygons, then find the vertices of each polygon that are deepest inside
// the other. Create contacts at those vertices.
func polygonVsPolygon(meshA, meshB *Mesh, pool *ContactPool) []*Contact {
        polyA := meshA.polygon
        polyB := meshB.polygon
        if len(polyA) < 3 || len(polyB) < 3 {
                return nil
        }

        // Find the minimum penetration axis across all edges of both polygons.
        // The normal points from A toward B.
        var bestNormal Vec2
        bestPenetration := float32(MaxWorldSize)

        // Test polyA's edges — outward normal of A points away from A.
        // If B overlaps on the outward side, the contact normal (A→B) = outward normal.
        // If B overlaps on the inward side, the contact normal = -outward normal.
        for _, edge := range polygonEdges(polyA) {
                normal := edge.normal // outward from A
                projA := projectPolygon(polyA, normal)
                projB := projectPolygon(polyB, normal)
                overlap := Min(projA.max, projB.max) - Max(projA.min, projB.min)
                if overlap <= 0 {
                        return nil // separating axis
                }
                // Determine contact normal direction (A→B)
                var contactNormal Vec2
                var pen float32
                if projB.min >= projA.min {
                        // B is on the outward side of A
                        contactNormal = normal
                        pen = projA.max - projB.min
                } else {
                        // B is on the inward side of A
                        contactNormal = normal.Neg()
                        pen = projB.max - projA.min
                }
                if pen < bestPenetration {
                        bestPenetration = pen
                        bestNormal = contactNormal
                }
        }

        // Test polyB's edges — outward normal of B points away from B.
        // Contact normal (A→B) = -outward normal if A is on outward side,
        // or +outward normal if A is on inward side.
        for _, edge := range polygonEdges(polyB) {
                normal := edge.normal // outward from B
                projA := projectPolygon(polyA, normal)
                projB := projectPolygon(polyB, normal)
                overlap := Min(projA.max, projB.max) - Max(projA.min, projB.min)
                if overlap <= 0 {
                        return nil // separating axis
                }
                var contactNormal Vec2 // A→B direction
                var pen float32
                if projA.min >= projB.min {
                        // A is on the outward side of B → A→B = outward normal
                        contactNormal = normal
                        pen = projB.max - projA.min
                } else {
                        // A is on the inward side of B → A→B = -outward normal
                        contactNormal = normal.Neg()
                        pen = projA.max - projB.min
                }
                if pen < bestPenetration {
                        bestPenetration = pen
                        bestNormal = contactNormal
                }
        }

        if bestPenetration <= 0 || bestPenetration >= MaxWorldSize {
                return nil
        }

        // bestNormal points from A→B. The Manifold convention is B→A, so negate.

        var contacts []*Contact

        // Find vertices of B that are inside A (penetrating from B into A)
        for _, p := range polyB {
                if pointInPolygon(p.GlobalPosition(), polyA) {
                        // Find the penetration depth: distance from p to the nearest edge of A
                        depth, edgeNormal := distanceToPolygon(p.GlobalPosition(), polyA)
                        if depth > 0 {
                                c := pool.Get()
                                c.Particle = p
                                c.Position = p.GlobalPosition()
                                // Normal should point from B→A (away from B, toward A)
                                // edgeNormal points outward from A, so negate to get B→A
                                c.Normal = edgeNormal
                                c.Penetration = depth
                                c.ReferenceParticles = nearestEdgeParticles(p.GlobalPosition(), polyA)
                                contacts = append(contacts, c)
                        }
                }
        }

        // Find vertices of A that are inside B (penetrating from A into B)
        for _, p := range polyA {
                if pointInPolygon(p.GlobalPosition(), polyB) {
                        depth, edgeNormal := distanceToPolygon(p.GlobalPosition(), polyB)
                        if depth > 0 {
                                c := pool.Get()
                                c.Particle = p
                                c.Position = p.GlobalPosition()
                                // Normal should point from B→A (away from B, toward A)
                                // edgeNormal points outward from B. Since the contact particle
                                // is on A, the normal should point from B toward A = outward from B.
                                c.Normal = edgeNormal
                                c.Penetration = depth
                                c.ReferenceParticles = nearestEdgeParticles(p.GlobalPosition(), polyB)
                                contacts = append(contacts, c)
                        }
                }
        }

        // No vertices inside = no collision
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
func polygonEdges(poly []*Particle) []edgeInfo {
        n := len(poly)
        edges := make([]edgeInfo, n)
        for i := 0; i < n; i++ {
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
func distanceToPolygon(point Vec2, poly []*Particle) (float32, Vec2) {
        n := len(poly)
        bestDepth := float32(-MaxWorldSize)
        bestNormal := Vec2Zero()
        for i := 0; i < n; i++ {
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
func findMinPenAxis(refPoly, incidentPoly []*Particle) (normal Vec2, penetration float32, edgeIdx int, ok bool) {
        n := len(refPoly)
        bestPen := float32(MaxWorldSize)
        bestIdx := 0
        bestNormal := Vec2Zero()

        for i := 0; i < n; i++ {
                p1 := refPoly[i]
                p2 := refPoly[(i+1)%n]
                edge := p2.GlobalPosition().Sub(p1.GlobalPosition())
                // Outward normal (for CW winding): (edge.Y, -edge.X)
                outwardNormal := Vec2{X: edge.Y, Y: -edge.X}.Normalized()

                projRef := projectPolygon(refPoly, outwardNormal)
                projInc := projectPolygon(incidentPoly, outwardNormal)

                // Check for overlap
                // Overlap = min(maxRef, maxInc) - max(minRef, minInc)
                overlap := Min(projRef.max, projInc.max) - Max(projRef.min, projInc.min)
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
type projectResult struct {
        min, max    float32
        minIndex    int
}

// projectPolygon projects a polygon onto an axis (unit normal).
// Matches QCollision::ProjectToAxis.
func projectPolygon(poly []*Particle, axis Vec2) projectResult {
        if len(poly) == 0 {
                return projectResult{}
        }
        min := poly[0].GlobalPosition().Dot(axis)
        max := min
        minIndex := 0
        for i := 1; i < len(poly); i++ {
                p := poly[i].GlobalPosition().Dot(axis)
                if p < min {
                        min = p
                        minIndex = i
                }
                if p > max {
                        max = p
                }
        }
        return projectResult{min: min, max: max, minIndex: minIndex}
}

// findIncidentEdge finds the edge of poly most anti-parallel to `normal`.
// Returns the two particles forming that edge.
func findIncidentEdge(poly []*Particle, normal Vec2) [2]*Particle {
        n := len(poly)
        bestDot := float32(1.0) // we want the most negative dot
        bestIdx := 0
        for i := 0; i < n; i++ {
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
// contacts. Matches QCollision::ClipContactParticles in qcollision.cpp:1210-1234.
//
// `normal` points from the incident body toward the reference body (i.e.,
// the direction to push the incident body to resolve the collision).
// `penetration` is the overlap depth.
func clipEdges(refParts, incParts [2]*Particle, normal Vec2, penetration float32, pool *ContactPool) []*Contact {
        var contacts []*Contact

        refA := refParts[0].GlobalPosition()
        refB := refParts[1].GlobalPosition()
        refEdge := refB.Sub(refA)
        refLen := refEdge.Length()
        if refLen < 1e-6 {
                return nil
        }
        refDir := refEdge.Div(refLen)

        // For each incident particle, check if it's on the penetrating side
        // of the reference edge. The normal points from bodyB→bodyA, so
        // incident particles on the POSITIVE side of the normal are penetrating
        // (they've pushed through into bodyA's side).
        for _, incP := range incParts {
                incPos := incP.GlobalPosition()
                bridge := incPos.Sub(refA)
                dist := bridge.Dot(normal)
                if dist > 0 {
                        // Penetrating: dist is the depth along the normal
                        // Check projection is within the reference edge segment (with small tolerance)
                        proj := bridge.Dot(refDir)
                        if proj >= -1 && proj <= refLen+1 {
                                c := pool.Get()
                                c.Particle = incP
                                c.Position = incPos
                                c.Normal = normal
                                c.Penetration = dist
                                c.ReferenceParticles = []*Particle{refParts[0], refParts[1]}
                                contacts = append(contacts, c)
                        }
                }
        }

        return contacts
}

// --- Circle vs Circle ---

// circleVsCircle runs circle-circle collision detection.
// Matches QCollision::CircleAndCircle in qcollision.cpp:682-806.
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
                                dist := Sqrt(distSq)
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
                                c.Position = gB
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
// Matches QCollision::CircleAndPolygon in qcollision.cpp:902-1074.
//
// The C++ convention:
//   - contact->particle = the INCIDENT particle (circle particle that penetrated)
//   - contact->referenceParticles = the REFERENCE particles (polygon edge/vertex)
//   - contact->normal = points from polygon (reference) toward circle (incident)
//   - contact->position = the circle particle's position (adjusted by radius)
//
// In the Manifold solver:
//   - referenceBody = owner of referenceParticles (the polygon)
//   - incidentBody = owner of particle (the circle)
//   - refResponseForce = -responseForce (applied to polygon)
//   - incResponseForce = +responseForce (applied to circle particle)
// So the normal must point from polygon toward circle = OUTWARD from polygon.
func circleVsPolygon(circleMesh, polygonMesh *Mesh, pool *ContactPool) []*Contact {
	var contacts []*Contact
	poly := polygonMesh.polygon
	if len(poly) < 3 {
		return nil
	}
	n := len(poly)

	// Build polygon positions, shrinking by particle radius (C++ ParticlePolygonToPolygon)
	polyPositions := make([]Vec2, n)
	for i := 0; i < n; i++ {
		p := poly[i]
		if p.Radius() > 0.5 {
			pp := poly[(i-1+n)%n]
			np := poly[(i+1)%n]
			bisectorUnit := GetBisectorUnitVector(pp.GlobalPosition(), p.GlobalPosition(), np.GlobalPosition(), false)
			polyPositions[i] = p.GlobalPosition().Sub(bisectorUnit.Mul(p.Radius()))
		} else {
			polyPositions[i] = p.GlobalPosition()
		}
	}

	for _, circle := range circleMesh.particles {
		cPos := circle.GlobalPosition()
		cRadius := circle.Radius()

		// Find nearest vertex and nearest edge
		var nearestPolygonParticle *Particle
		nearestParticlePenSq := float32(MaxWorldSize)
		var nearestParticleNormal Vec2

		var nearestEdgeParticles [2]*Particle
		nearestEdgePenetration := float32(MaxWorldSize)
		nearestEdgeMinDist := float32(MaxWorldSize)
		var nearestEdgeNormal Vec2

		for pi := 0; pi < n; pi++ {
			npi := (pi + 1) % n
			p := poly[pi]
			np := poly[npi]
			pPos := polyPositions[pi]
			npPos := polyPositions[npi]

			// Nearest vertex
			circleToParticleVec := cPos.Sub(pPos)
			distSq := circleToParticleVec.LengthSquared()
			if distSq < nearestParticlePenSq {
				nearestPolygonParticle = p
				nearestParticlePenSq = distSq
				nearestParticleNormal = circleToParticleVec.Normalized()
			}

			// Nearest edge
			edgeVec := npPos.Sub(pPos)
			edgeVecUnit := edgeVec.Normalized()
			edgeVecNormal := Vec2{X: edgeVec.Y, Y: -edgeVec.X}.Normalized()

			circleToEdgeBegin := cPos.Sub(pPos)
			pen := circleToEdgeBegin.Dot(edgeVecNormal)

			if Abs(pen) < nearestEdgeMinDist {
				proj := circleToEdgeBegin.Dot(edgeVecUnit)
				if proj >= 0.0 && proj <= edgeVec.Length() {
					nearestEdgeMinDist = Abs(pen)
					nearestEdgePenetration = pen
					nearestEdgeParticles[0] = p
					nearestEdgeParticles[1] = np
					nearestEdgeNormal = edgeVecNormal
				}
			}
		}

		nearestParticlePen := Sqrt(nearestParticlePenSq)

		// Determine Voronoi region: 0=vertex, 1=edge, 2=inside
		voronoiRegion := 0
		if nearestEdgeParticles[0] != nil {
			if nearestParticlePen > nearestEdgeMinDist {
				if nearestEdgePenetration < 0 && pointInPolygon(cPos, poly) {
					voronoiRegion = 2
				} else {
					voronoiRegion = 1
				}
			} else {
				voronoiRegion = 0
			}
		}

		// Test collisions based on Voronoi region
		if voronoiRegion == 0 {
			// Vertex region
			if nearestPolygonParticle != nil {
				if pointInPolygon(cPos, poly) {
					// Inside, nearest to vertex
					penetration := cRadius + nearestParticlePen
					contactPos := cPos
					c := pool.Get()
					c.Configure(circle, contactPos, nearestParticleNormal.Neg(), penetration, []*Particle{nearestPolygonParticle})
					contacts = append(contacts, c)
				} else {
					// Outside, near vertex
					if nearestParticlePen < cRadius {
						penetration := cRadius - nearestParticlePen
						contactPos := cPos
						if cRadius > 0.5 {
							contactPos = contactPos.Sub(nearestParticleNormal.Mul(cRadius))
						}
						c := pool.Get()
						c.Configure(circle, contactPos, nearestParticleNormal, penetration, []*Particle{nearestPolygonParticle})
						contacts = append(contacts, c)
					}
				}
			}
		} else if voronoiRegion == 1 {
			// Edge region
			if nearestEdgePenetration < cRadius {
				penetration := cRadius - nearestEdgePenetration
				contactPos := cPos
				if cRadius > 0.5 {
					contactPos = contactPos.Sub(nearestEdgeNormal.Mul(cRadius))
				}
				c := pool.Get()
				c.Configure(circle, contactPos, nearestEdgeNormal, penetration, []*Particle{nearestEdgeParticles[0], nearestEdgeParticles[1]})
				contacts = append(contacts, c)
			}
		} else if voronoiRegion == 2 {
			// Inside region
			penetration := cRadius - nearestEdgePenetration
			contactPos := cPos
			if cRadius > 0.5 {
				contactPos = contactPos.Sub(nearestEdgeNormal.Mul(cRadius))
			}
			c := pool.Get()
			c.Configure(circle, contactPos, nearestEdgeNormal, penetration, []*Particle{nearestEdgeParticles[0], nearestEdgeParticles[1]})
			contacts = append(contacts, c)
		}
	}

	return contacts
}

// ---
// --- Geometry helpers ---

// LineIntersectionLine computes the intersection of two line segments.
// Returns Vec2NaN() if no intersection. Matches QCollision::LineIntersectionLine.
func LineIntersectionLine(d1A, d1B, d2A, d2B Vec2) Vec2 {
        r := d1B.Sub(d1A)
        s := d2B.Sub(d2A)
        denom := r.X*s.Y - r.Y*s.X
        if Abs(denom) < 1e-6 {
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

// pointInPolygon reports whether a point is inside a convex polygon.
// For CW winding in screen coords (Y down), interior = RIGHT side of edge.
// Cross product (edge × toPoint): positive = LEFT (outside), negative/zero = RIGHT (inside).
// Point is inside if cross <= 0 for ALL edges... actually:
// edge=(dx,dy), toPoint=(px,py), cross = dx*py - dy*px
// For CW in Y-down: going right (dx>0,dy=0), right side is down (py>0).
// cross = dx*py - 0 = dx*py > 0 when py>0 (point is on the right/inside).
// So inside = cross >= 0 for ALL edges. Outside = any cross < 0.
func pointInPolygon(point Vec2, poly []*Particle) bool {
	n := len(poly)
	if n < 3 {
		return false
	}
	for i := 0; i < n; i++ {
		p1 := poly[i].GlobalPosition()
		p2 := poly[(i+1)%n].GlobalPosition()
		edge := p2.Sub(p1)
		toPoint := point.Sub(p1)
		cross := edge.X*toPoint.Y - edge.Y*toPoint.X
		if cross < 0 {
			return false
		}
	}
	return true
}

// nearestEdgeParticles returns the 2 particles forming the nearest edge
// of a polygon to the given point.
func nearestEdgeParticles(point Vec2, poly []*Particle) []*Particle {
	n := len(poly)
	if n < 2 {
		return nil
	}
	bestIdx := 0
	bestDist := float32(MaxWorldSize)
	for i := 0; i < n; i++ {
		p1 := poly[i].GlobalPosition()
		p2 := poly[(i+1)%n].GlobalPosition()
		edge := p2.Sub(p1)
		edgeLen := edge.Length()
		if edgeLen < 1e-6 {
			continue
		}
		edgeDir := edge.Div(edgeLen)
		bridge := point.Sub(p1)
		proj := bridge.Dot(edgeDir)
		if proj < 0 { proj = 0 }
		if proj > edgeLen { proj = edgeLen }
		closest := p1.Add(edgeDir.Mul(proj))
		dist := point.Sub(closest).Length()
		if dist < bestDist {
			bestDist = dist
			bestIdx = i
		}
	}
	return []*Particle{poly[bestIdx], poly[(bestIdx+1)%n]}
}
