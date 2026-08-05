package physics

// Polyline collision methods for soft bodies.
// Matches PolylineAndPolygon, PolylineAndPolyline, CircleAndPolyline2,
// and CircleAndCircleSelf in qcollision.cpp.

// polylineAndPolygon checks collisions between a polyline (deformable rope)
// and a solid polygon. Matches QCollision::PolylineAndPolygon.
//
// Uses nearest-edge detection (like C++ CircleAndPolygon Voronoi regions):
// finds the polygon edge with the smallest perpendicular distance to
// each polyline vertex. If the vertex is inside the polygon and on the
// interior side of that edge, creates a contact pushing it out.
func polylineAndPolygon(polylineParticles, polygonParticles []*Particle, pool *ContactPool) []*Contact {
	var contacts []*Contact
	n := len(polygonParticles)
	if n < 3 {
		return nil
	}

	for _, p := range polylineParticles {
		if !p.enabled {
			continue
		}
		pos := p.GlobalPosition()

		// Find the nearest edge by perpendicular distance
		bestEdgeIdx := 0
		bestEdgeDist := float32(MaxWorldSize)
		bestEdgePen := float32(0) // signed: negative = inside
		var bestEdgeNormal Vec2

		for i := 0; i < n; i++ {
			p1 := polygonParticles[i].GlobalPosition()
			p2 := polygonParticles[(i+1)%n].GlobalPosition()
			edge := p2.Sub(p1)
			edgeLen := edge.Length()
			if edgeLen < 1e-6 {
				continue
			}
			edgeUnit := edge.Div(edgeLen)
			edgeNormal := Vec2{X: edge.Y, Y: -edge.X}.Normalized()

			bridge := pos.Sub(p1)
			pen := bridge.Dot(edgeNormal)
			absPen := pen
			if absPen < 0 {
				absPen = -absPen
			}

			proj := bridge.Dot(edgeUnit)
			if proj >= 0 && proj <= edgeLen {
				if absPen < bestEdgeDist {
					bestEdgeDist = absPen
					bestEdgePen = pen
					bestEdgeIdx = i
					bestEdgeNormal = edgeNormal
				}
			}
		}

		inside := pointInPolygon(pos, polygonParticles)

		if inside {
			// Point is inside the polygon. Find the edge to push out through.
			// Prefer edges with bestEdgePen < 0 (point is on the interior side).
			// If no such edge exists (deep penetration, all edges have pen > 0),
			// use the edge with the smallest pen (closest to interior = nearest exit).
			if bestEdgePen < 0 {
				// Nearest edge is on the interior side — push out through it.
				depth := -bestEdgePen
				c := pool.Get()
				c.Particle = p
				c.Position = pos
				c.Normal = bestEdgeNormal
				c.Penetration = depth
				c.ReferenceParticles = []*Particle{polygonParticles[bestEdgeIdx], polygonParticles[(bestEdgeIdx+1)%n]}
				contacts = append(contacts, c)
			} else {
				// Deep penetration — all edges have pen > 0.
				// Use velocity direction to determine the entry edge.
				vel := pos.Sub(p.PreviousGlobalPosition())
				velLen := vel.Length()
				if velLen > 1e-4 {
					velUnit := vel.Div(velLen)
					// Find edge whose outward normal is most anti-parallel to velocity
					bestVelDot := float32(MaxWorldSize)
					entryIdx := bestEdgeIdx
					entryNormal := bestEdgeNormal
					for i := 0; i < n; i++ {
						p1 := polygonParticles[i].GlobalPosition()
						p2 := polygonParticles[(i+1)%n].GlobalPosition()
						eedge := p2.Sub(p1)
						enormal := Vec2{X: eedge.Y, Y: -eedge.X}.Normalized()
						dot := enormal.Dot(velUnit)
						if dot < bestVelDot {
							bestVelDot = dot
							entryIdx = i
							entryNormal = enormal
						}
					}
					// Depth = distance to that edge
					ep1 := polygonParticles[entryIdx].GlobalPosition()
					ebridge := pos.Sub(ep1)
					edepth := ebridge.Dot(entryNormal)
					if edepth < 0 { edepth = -edepth }
					c := pool.Get()
					c.Particle = p
					c.Position = pos
					c.Normal = entryNormal
					c.Penetration = edepth
					c.ReferenceParticles = []*Particle{polygonParticles[entryIdx], polygonParticles[(entryIdx+1)%n]}
					contacts = append(contacts, c)
				} else {
					// No velocity — use nearest edge by abs distance
					depth := bestEdgeDist
					c := pool.Get()
					c.Particle = p
					c.Position = pos
					c.Normal = bestEdgeNormal
					c.Penetration = depth
					c.ReferenceParticles = []*Particle{polygonParticles[bestEdgeIdx], polygonParticles[(bestEdgeIdx+1)%n]}
					contacts = append(contacts, c)
				}
			}
		} else if !inside && bestEdgeDist < p.Radius() {
			r := p.Radius()
			if r > 0.5 {
				depth := r - bestEdgeDist
				c := pool.Get()
				c.Particle = p
				c.Position = pos
				c.Normal = bestEdgeNormal
				c.Penetration = depth
				c.ReferenceParticles = []*Particle{polygonParticles[bestEdgeIdx], polygonParticles[(bestEdgeIdx+1)%n]}
				contacts = append(contacts, c)
			}
		}
	}

	return contacts
}

// polylineAndPolyline checks collisions between two polylines.
func polylineAndPolyline(testPolyline, targetPolyline []*Particle, pool *ContactPool) []*Contact {
	var contacts []*Contact
	if len(targetPolyline) < 2 {
		return nil
	}

	for _, p := range testPolyline {
		if !p.enabled {
			continue
		}
		pos := p.GlobalPosition()
		r := p.Radius()
		if r < 0.5 {
			r = 0.5
		}

		nearestEdge, nearestDist, contactPos, found := nearestEdgeToPoint(pos, targetPolyline)
		if found && nearestDist < r {
			c := pool.Get()
			c.Particle = p
			c.Position = contactPos
			edge := nearestEdge[1].GlobalPosition().Sub(nearestEdge[0].GlobalPosition())
			normal := edge.Perpendicular().Normalized()
			toVertex := pos.Sub(contactPos)
			if normal.Dot(toVertex) < 0 {
				normal = normal.Neg()
			}
			c.Normal = normal
			c.Penetration = r - nearestDist
			c.ReferenceParticles = []*Particle{nearestEdge[0], nearestEdge[1]}
			contacts = append(contacts, c)
		}
	}

	return contacts
}

// circleAndPolyline2 checks collisions between circles and a polyline.
func circleAndPolyline2(circleParticles, polylineParticles []*Particle, pool *ContactPool) []*Contact {
	return polylineAndPolygon(circleParticles, polylineParticles, pool)
}

// circleAndCircleSelf checks self-collisions among particles in a single mesh.
func circleAndCircleSelf(particles []*Particle, pool *ContactPool, specifiedRadius float32) []*Contact {
	var contacts []*Contact
	n := len(particles)
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			pA := particles[i]
			pB := particles[j]
			if !pA.enabled || !pB.enabled {
				continue
			}
			gA := pA.GlobalPosition()
			gB := pB.GlobalPosition()
			diff := gB.Sub(gA)
			distSq := diff.LengthSquared()

			rA := pA.Radius()
			rB := pB.Radius()
			if specifiedRadius > 0 {
				rA = specifiedRadius
				rB = specifiedRadius
			}
			rSum := rA + rB

			if distSq < rSum*rSum && distSq > 1e-8 {
				dist := Sqrt(distSq)
				normal := diff.Div(dist)
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

// nearestEdgeToPoint finds the nearest edge of a polygon to a point.
func nearestEdgeToPoint(point Vec2, poly []*Particle) ([2]*Particle, float32, Vec2, bool) {
	n := len(poly)
	if n < 2 {
		return [2]*Particle{nil, nil}, MaxWorldSize, Vec2Zero(), false
	}

	bestDist := float32(MaxWorldSize)
	bestEdge := [2]*Particle{nil, nil}
	bestPos := Vec2Zero()

	for i := 0; i < n; i++ {
		p1 := poly[i].GlobalPosition()
		p2 := poly[(i+1)%n].GlobalPosition()
		edgeVec := p2.Sub(p1)
		edgeLen := edgeVec.Length()
		if edgeLen < 1e-6 {
			continue
		}
		edgeDir := edgeVec.Div(edgeLen)
		bridge := point.Sub(p1)
		proj := bridge.Dot(edgeDir)
		if proj < 0 {
			proj = 0
		} else if proj > edgeLen {
			proj = edgeLen
		}
		c := p1.Add(edgeDir.Mul(proj))
		d := (point.Sub(c)).Length()
		if d < bestDist {
			bestDist = d
			bestEdge = [2]*Particle{poly[i], poly[(i+1)%n]}
			bestPos = c
		}
	}

	if bestEdge[0] == nil {
		return [2]*Particle{nil, nil}, MaxWorldSize, Vec2Zero(), false
	}
	return bestEdge, bestDist, bestPos, true
}
