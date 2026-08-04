package physics

// Polyline collision methods for soft bodies.
// Matches PolylineAndPolygon, PolylineAndPolyline, CircleAndPolyline2,
// and CircleAndCircleSelf in qcollision.cpp.

// polylineAndPolygon checks collisions between a polyline (deformable rope)
// and a solid polygon. Matches QCollision::PolylineAndPolygon.
//
// For each polyline vertex, finds the nearest polygon edge and computes
// penetration. Produces contacts at penetrating vertices.
func polylineAndPolygon(polylineParticles, polygonParticles []*Particle, pool *ContactPool) []*Contact {
        var contacts []*Contact
        if len(polygonParticles) < 3 {
                return nil
        }

        for _, p := range polylineParticles {
                if !p.enabled {
                        continue
                }
                pos := p.GlobalPosition()

                // Check if the polyline vertex is inside the polygon
                if pointInPolygon(pos, polygonParticles) {
                        depth, edgeNormal := distanceToPolygon(pos, polygonParticles)
                        if depth > 0 {
                                c := pool.Get()
                                c.Particle = p
                                c.Position = pos
                                c.Normal = edgeNormal
                                c.Penetration = depth
                                c.ReferenceParticles = []*Particle{}
                                contacts = append(contacts, c)
                        }
                } else {
                        // Check distance to nearest edge — if within particle radius, collide
                        r := p.Radius()
                        if r > 0.5 {
                                nearestEdge, nearestDist, contactPos, found := nearestEdgeToPoint(pos, polygonParticles)
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
                }
        }

        return contacts
}

// polylineAndPolyline checks collisions between two polylines.
// Matches QCollision::PolylineAndPolyline.
//
// For each vertex of the test polyline, checks if it's near any edge of
// the target polyline. Produces contacts at penetrating vertices.
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
// Matches QCollision::CircleAndPolyline2.
func circleAndPolyline2(circleParticles, polylineParticles []*Particle, pool *ContactPool) []*Contact {
        return polylineAndPolygon(circleParticles, polylineParticles, pool)
}

// circleAndCircleSelf checks self-collisions among particles in a single mesh.
// Matches QCollision::CircleAndCircleSelf.
//
// For each pair (i, j) with i < j, checks if the distance is less than the
// sum of radii. Produces contacts and immediately solves them (hot solving).
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
// Returns the edge (2 particles), the distance, and the closest point on the edge.
// Returns found=false if no valid edge exists.
func nearestEdgeToPoint(point Vec2, poly []*Particle) (edge [2]*Particle, dist float32, closest Vec2, found bool) {
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
