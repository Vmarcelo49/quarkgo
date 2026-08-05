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

// polygonVsPolygon runs SAT + edge clipping between two convex polygons.
// Faithful port of QCollision::PolygonAndPolygon in qcollision.cpp:1087-1208.
//
// Algorithm:
//   A. SAT: test all edges of both polygons, find minimum penetration axis
//   B. Find reference and incident edges using support points
//   C. Clip incident edge against reference edge
//   D. Create contacts at clipped points inside the reference edge
func polygonVsPolygon(meshA, meshB *Mesh, pool *ContactPool) []*Contact {
	polyA := meshA.polygon
	polyB := meshB.polygon
	sizeA := len(polyA)
	sizeB := len(polyB)
	if sizeA < 3 || sizeB < 3 {
		return nil
	}

	// A. SAT: find minimum penetration axis
	// The C++ uses Perpendicular() (not .Perpendicular().Normalized()) but
	// QVector::Perpendicular returns (y, -x) which is not normalized.
	// The C++ ProjectToAxis uses Dot which works with unnormalized normals.
	// We match the C++ exactly: use Perpendicular() (unnormalized).
	minPenetration := float32(MaxWorldSize)
	var refNormal Vec2

	// Test all edges of both polygons
	for polyIdx := 0; polyIdx < 2; polyIdx++ {
		var refPoly []*Particle
		var incPoly []*Particle
		var refSize int
		if polyIdx == 0 {
			refPoly = polyA
			incPoly = polyB
			refSize = sizeA
		} else {
			refPoly = polyB
			incPoly = polyA
			refSize = sizeB
		}

		for s := 0; s < refSize; s++ {
			s1 := refPoly[s]
			s2 := refPoly[(s+1)%refSize]
			edge := s2.GlobalPosition().Sub(s1.GlobalPosition())
			// C++: (s2-s1).Normalized().Perpendicular()
			// Perpendicular() = (y, -x)
			sNormal := edge.Normalized().Perpendicular()

			// Project both polygons onto sNormal
			refProj := projectToAxis(sNormal, refPoly)
			incProj := projectToAxis(sNormal, incPoly)

			// C++ Project::Overlap: returns negative if overlapping
			// if other.min < min: penetration = min - other.max
			// else: penetration = other.min - max
			var penetration float32
			if incProj.min < refProj.min {
				penetration = refProj.min - incProj.max
			} else {
				penetration = incProj.min - refProj.max
			}

			// C++: if penetration >= 0, return (separating axis found)
			if penetration >= 0 {
				return nil
			}

			penetration = Abs(penetration)
			if penetration < minPenetration {
				minPenetration = penetration
				refNormal = sNormal
			}
		}
	}

	// B. Find reference and incident edges using support points
	// C++: project both polygons onto refNormal, find support points
	supportProjA := projectToAxis(refNormal, polyA)
	supportProjB := projectToAxis(refNormal, polyB)

	supportAIdx := supportProjA.maxIndex
	supportBIdx := supportProjB.minIndex
	if supportProjB.min < supportProjA.min {
		supportAIdx = supportProjA.minIndex
		supportBIdx = supportProjB.maxIndex
	}

	// Find the reference edge: the edge most perpendicular to refNormal
	// (least parallel to refNormal = most parallel to the contact surface)
	// C++: choose the edge with the smallest |dot(edge, refNormal)|

	// polyA segment options
	segPrevA := (supportAIdx - 1 + sizeA) % sizeA
	segNextA := (supportAIdx + 1) % sizeA

	segAOption1 := polyA[segNextA].GlobalPosition().Sub(polyA[supportAIdx].GlobalPosition())
	segAOption2 := polyA[supportAIdx].GlobalPosition().Sub(polyA[segPrevA].GlobalPosition())

	segAOption1Par := Abs(segAOption1.Dot(refNormal))
	segAOption2Par := Abs(segAOption2.Dot(refNormal))

	segmentA := [2]*Particle{polyA[supportAIdx], polyA[segNextA]}
	segAPar := segAOption1Par
	if segAOption2Par < segAOption1Par {
		segmentA = [2]*Particle{polyA[segPrevA], polyA[supportAIdx]}
		segAPar = segAOption2Par
	}

	// polyB segment options
	segPrevB := (supportBIdx - 1 + sizeB) % sizeB
	segNextB := (supportBIdx + 1) % sizeB

	segBOption1 := polyB[segNextB].GlobalPosition().Sub(polyB[supportBIdx].GlobalPosition())
	segBOption2 := polyB[supportBIdx].GlobalPosition().Sub(polyB[segPrevB].GlobalPosition())

	segBOption1Par := Abs(segBOption1.Dot(refNormal))
	segBOption2Par := Abs(segBOption2.Dot(refNormal))

	segmentB := [2]*Particle{polyB[supportBIdx], polyB[segNextB]}
	segBPar := segBOption1Par
	if segBOption2Par < segBOption1Par {
		segmentB = [2]*Particle{polyB[segPrevB], polyB[supportBIdx]}
		segBPar = segBOption2Par
	}

	// C. Clip: the reference segment is the one most perpendicular to refNormal
	var contacts []*Contact
	if segBPar < segAPar {
		// Reference is segmentB, incident is segmentA
		clipContactParticles(segmentB, segmentA, pool, &contacts)
		if len(contacts) == 0 {
			clipContactParticles(segmentA, segmentB, pool, &contacts)
		}
	} else {
		// Reference is segmentA, incident is segmentB
		clipContactParticles(segmentA, segmentB, pool, &contacts)
		if len(contacts) == 0 {
			clipContactParticles(segmentB, segmentA, pool, &contacts)
		}
	}

	return contacts
}

// projectToAxis projects a polygon onto an axis and returns min/max + indices.
// Matches QCollision::ProjectToAxis in qcollision.cpp:1237-1258.
func projectToAxis(normal Vec2, poly []*Particle) struct {
	min, max float32
	minIndex, maxIndex int
} {
	result := struct {
		min, max float32
		minIndex, maxIndex int
	}{
		min: MaxWorldSize,
		max: -MaxWorldSize,
	}
	for i, p := range poly {
		dist := p.GlobalPosition().Dot(normal)
		if dist < result.min {
			result.min = dist
			result.minIndex = i
		}
		if dist > result.max {
			result.max = dist
			result.maxIndex = i
		}
	}
	return result
}

// clipContactParticles clips incident edge against reference edge.
// Matches QCollision::ClipContactParticles in qcollision.cpp:1210-1234.
//
// Creates contacts for incident particles that are on the negative side
// of the reference edge normal (inside the reference polygon).
func clipContactParticles(refParticles, incParticles [2]*Particle, pool *ContactPool, contacts *[]*Contact) {
	sv := refParticles[1].GlobalPosition().Sub(refParticles[0].GlobalPosition())
	length := sv.Length()
	if length < 1e-6 {
		return
	}
	unit := sv.Div(length)
	normal := unit.Perpendicular()

	for i := 0; i < 2; i++ {
		p := incParticles[i]
		bv := p.GlobalPosition().Sub(refParticles[0].GlobalPosition())
		dist := bv.Dot(normal)
		if dist <= 0 {
			proj := bv.Dot(unit)
			if proj >= 0.0 && proj <= length {
				c := pool.Get()
				c.Configure(p, p.GlobalPosition(), normal, Abs(dist), []*Particle{refParticles[0], refParticles[1]})
				*contacts = append(*contacts, c)
			}
		}
	}
}

// ---
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
