package physics

// Polyline collision methods for soft bodies.
// Faithful port of PolylineAndPolygon, PolylineAndPolyline, CircleAndPolyline2,
// and CircleAndCircleSelf in qcollision.cpp.

// polylineAndPolygon checks collisions between a polyline (deformable rope)
// and a solid polygon. Faithful port of QCollision::PolylineAndPolygon
// (qcollision.cpp:43-201).
//
// Algorithm:
//  A. Compute a bisector vector for each polygon particle — a ray from the
//     particle toward the polygon interior, length = half the distance to
//     the nearest intersecting polygon edge (or half the prev-edge length
//     for self-collisions).
//  B. Loop over polyline SEGMENTS (not particles). For each segment, apply
//     radius offsets to s1 and s2 if r > 0.5.
//  C. For each polygon particle, test if its bisector ray intersects the
//     polyline segment via LineIntersectionLine.
//  D. On intersection: penetration = bridgeVec.Dot(-normal) where
//     bridgeVec = pPos - s1Pos and normal = (s2Pos-s1Pos).Normalized().Perpendicular().
//     Contact: particle = polygon particle, reference = polyline segment {s1, s2}.
func polylineAndPolygon(polylineParticles, polygonParticles []*Particle, pool *ContactPool) []*Contact {
        var contacts []*Contact
        polySize := len(polygonParticles)
        if polySize < 3 {
                return nil
        }

        // A. Compute bisector list for each polygon particle.
        isSelfCollision := false
        // In C++ this is `polylineParticles == polygonParticles` (pointer equality).
        // In Go we compare slice headers via unsafe.Pointer — but the engine only
        // calls this with self-collision via world.go's polylineAndPolyline(meshA.polygon, meshB.polygon)
        // where meshA==meshB implies pointer equality. Use a safer check: identity
        // of the first particle pointer.
        if polySize > 0 && len(polylineParticles) > 0 && polySize == len(polylineParticles) {
                same := true
                for i := 0; i < polySize; i++ {
                        if polygonParticles[i] != polylineParticles[i] {
                                same = false
                                break
                        }
                }
                isSelfCollision = same
        }

        bisectorList := make([]Vec2, polySize)
        for i := 0; i < polySize; i++ {
                pi := ((i - 1) + polySize) % polySize
                ni := (i + 1) % polySize
                pp := polygonParticles[pi]
                p := polygonParticles[i]
                np := polygonParticles[ni]

                bisectorUnit := GetBisectorUnitVector(pp.GlobalPosition(), p.GlobalPosition(), np.GlobalPosition(), true)
                bisectorRay := bisectorUnit.Mul(MaxWorldSize)

                var bisectorVector Vec2
                if isSelfCollision {
                        rayLength := Abs((p.GlobalPosition().Sub(pp.GlobalPosition())).Dot(bisectorUnit)) * 0.5
                        bisectorVector = bisectorUnit.Mul(rayLength)
                } else {
                        // Find the nearest intersecting polygon edge (other than the
                        // two edges adjacent to particle p, since those always share p).
                        minDistance := float32(MaxWorldSize)
                        sia := ni
                        for sia != pi {
                                sib := (sia + 1) % polySize
                                intersection := LineIntersectionLine(
                                        p.GlobalPosition(),
                                        p.GlobalPosition().Add(bisectorRay),
                                        polygonParticles[sia].GlobalPosition(),
                                        polygonParticles[sib].GlobalPosition(),
                                )
                                if !intersection.IsNaN() {
                                        findedVec := intersection.Sub(p.GlobalPosition())
                                        distance := findedVec.Length()
                                        if distance < minDistance {
                                                bisectorVector = findedVec.Mul(0.5)
                                                minDistance = distance
                                        }
                                }
                                sia = sib
                        }
                }
                bisectorList[i] = bisectorVector
        }

        // B. Loop over polyline segments.
        polylineSize := len(polylineParticles)
        for i := 0; i < polylineSize; i++ {
                s1 := polylineParticles[i]
                s2 := polylineParticles[(i+1)%polylineSize]

                s1Pos := s1.GlobalPosition()
                s2Pos := s2.GlobalPosition()
                if s1Pos == s2Pos {
                        continue
                }
                edgeVec := s2Pos.Sub(s1Pos)
                edgeLen := edgeVec.Length()
                if edgeLen < 1e-6 {
                        continue
                }
                normal := edgeVec.Div(edgeLen).Perpendicular()

                // Apply radius factor to segment endpoints.
                if s1.Radius() > 0.5 {
                        s1Pos = s1Pos.Add(normal.Mul(s1.Radius()))
                }
                if s2.Radius() > 0.5 {
                        s2Pos = s2Pos.Add(normal.Mul(s2.Radius()))
                }
                // Recompute normal after radius offsets.
                edgeVec = s2Pos.Sub(s1Pos)
                edgeLen = edgeVec.Length()
                if edgeLen < 1e-6 {
                        continue
                }
                normal = edgeVec.Div(edgeLen).Perpendicular()

                // C. Loop over polygon particles, test bisector ray vs segment.
                for n := 0; n < polySize; n++ {
                        if bisectorList[n] == (Vec2{}) {
                                continue
                        }
                        p := polygonParticles[n]

                        // Self-collision: skip particles connected by spring.
                        if isSelfCollision {
                                if p.IsConnectedWithSpring(s1) || p.IsConnectedWithSpring(s2) {
                                        continue
                                }
                        }

                        pPos := p.GlobalPosition()
                        if p.Radius() > 0.5 {
                                pPos = pPos.Sub(normal.Mul(p.Radius()))
                        }

                        // Bisector ray vs polyline segment intersection.
                        intersection := LineIntersectionLine(
                                pPos,
                                pPos.Add(bisectorList[n]),
                                s1Pos, s2Pos,
                        )
                        if intersection.IsNaN() {
                                continue
                        }

                        // D. Compute penetration.
                        bridgeVec := pPos.Sub(s1Pos)
                        penetration := bridgeVec.Dot(normal.Neg())

                        c := pool.Get()
                        c.Particle = p
                        c.Position = p.GlobalPosition()
                        c.Normal = normal
                        c.Penetration = penetration
                        c.ReferenceParticles = []*Particle{s1, s2}
                        contacts = append(contacts, c)
                }
        }

        return contacts
}

// polylineAndPolyline checks collisions between two polylines.
// Faithful port of QCollision::PolylineAndPolyline (qcollision.cpp:203-593).
//
// Algorithm:
//  A. For each test particle, AABB-cull against the target polyline AABB.
//  B. Determine if the test particle is INSIDE the target polyline:
//     - Use PointInPolygonWN for the primary inside test.
//     - If WN says outside but both polylines have > 3 particles, do an
//       intersection test: check if the test particle's adjacent edges
//       (pp→p and p→np) both intersect a target edge. If both intersect,
//       the particle is "logically" inside (edge passes through).
//  C. If INSIDE:
//     - Compute a bisector ray from the test particle (using its prev/next).
//     - Find the nearest target particle via FindNearestParticleOfPolygon.
//     - Build nearestSides = {prev→nearest, nearest→next}.
//     - Check if the nearest particle is on the "wrong side" of the ray
//       (sidePerp · rayUnit > 0 for at least one side). If wrong side,
//       rebuild nearestSides from ALL target sides where sidePerp·rayUnit > 0,
//       and set useMiniResponse=true (applies hysteresis scaling).
//     - For each candidate side: if test polyline >= 3 particles, do a
//       ray-vs-segment intersection test (rayEndPoint → pA vs side). If
//       intersection found and dist < 0 and dist > minDistance, record.
//       If test polyline < 3 particles, use vertical projection instead.
//     - If no side found, fallback: find side with min |dist|.
//     - Apply hysteresis if useMiniResponse.
//     - Configure contact with -penetration sign flip.
//  D. If OUTSIDE and radius > 0.5:
//     - Find nearest side via FindNearestSideOfPolygon(checkSideRange=true).
//     - Find nearest particle on that side.
//     - Build nearestSides from that particle's adjacent edges.
//     - For each side: apply radius offsets, compute perpProj, and if
//       |perpProj| < radius and proj in [0, len], record contact.
func polylineAndPolyline(testPolyline, targetPolyline []*Particle, pool *ContactPool, world *World) []*Contact {
        var contacts []*Contact
        targetSize := len(targetPolyline)
        if targetSize < 2 {
                return nil
        }

        // Compute the target polyline's AABB for broadphase culling.
        targetAABB := computePolylineAABB(targetPolyline)

        testSize := len(testPolyline)
        isSelfCollision := slicesEqual(testPolyline, targetPolyline)

        // A. Loop over each test particle.
        for ia := 0; ia < testSize; ia++ {
                pA := testPolyline[ia]
                if !pA.enabled {
                        continue
                }
                pAPos := pA.GlobalPosition()
                pARadius := pA.Radius()

                // AABB cull.
                particleAABB := AABB{
                        Min: Vec2{X: pAPos.X - pARadius, Y: pAPos.Y - pARadius},
                        Max: Vec2{X: pAPos.X + pARadius, Y: pAPos.Y + pARadius},
                }
                if !particleAABB.IsCollidingWith(targetAABB) {
                        continue
                }

                collidedSideIndex := -1

                // B. Determine if the test particle is inside the target polyline.
                circleCenterInsidePolyline := false
                if !isSelfCollision {
                        if pointInPolygonWN(pAPos, targetPolyline) {
                                circleCenterInsidePolyline = true
                        } else {
                                // Intersection tests between target sides and test polyline edges.
                                if testSize > 3 && targetSize > 3 {
                                        ppA := testPolyline[((ia-1+testSize)%testSize)]
                                        npA := testPolyline[(ia+1)%testSize]
                                        for j := 0; j < targetSize; j++ {
                                                pJ := targetPolyline[j]
                                                npJ := targetPolyline[(j+1)%targetSize]
                                                sideIntersectionA := !LineIntersectionLine(
                                                        ppA.GlobalPosition(), pAPos,
                                                        pJ.GlobalPosition(), npJ.GlobalPosition(),
                                                ).IsNaN()
                                                if sideIntersectionA {
                                                        sideIntersectionB := !LineIntersectionLine(
                                                                pAPos, npA.GlobalPosition(),
                                                                pJ.GlobalPosition(), npJ.GlobalPosition(),
                                                        ).IsNaN()
                                                        if sideIntersectionB {
                                                                circleCenterInsidePolyline = true
                                                        }
                                                }
                                        }
                                }
                        }
                }

                if circleCenterInsidePolyline {
                        // C. Inside case.
                        var nearestSides [][2]*Particle
                        rayEndPoint := Vec2Zero()
                        var rayVector Vec2
                        var rayUnit Vec2

                        if testSize >= 3 {
                                prevParticle := testPolyline[((ia-1+testSize)%testSize)]
                                nextParticle := testPolyline[(ia+1)%testSize]
                                cornerLen := nextParticle.Position().Sub(prevParticle.Position()).Length()

                                // Note: C++ uses GetPolygonBisectorVectorAt(ia) when ownerMesh
                                // is set. We don't maintain polygonBisectors cache, so we
                                // always compute the bisector unit vector directly.
                                rayUnit = GetBisectorUnitVector(prevParticle.GlobalPosition(), pAPos, nextParticle.GlobalPosition(), true)
                                rayVector = rayUnit.Mul(cornerLen)
                                // If the cached bisector would be shorter than cornerLen,
                                // extend it to cornerLen (matches qcollision.cpp:293-295).
                                if rayVector.LengthSquared() < cornerLen*cornerLen {
                                        rayVector = rayUnit.Mul(cornerLen)
                                }
                                rayEndPoint = pAPos.Add(rayVector)
                        }

                        // Find the nearest target particle.
                        ni := findNearestParticleOfPolygon(pA, targetPolyline)
                        pB := targetPolyline[ni]
                        nearestSides = append(nearestSides, [2]*Particle{
                                targetPolyline[((ni-1+targetSize)%targetSize)], pB,
                        })
                        nearestSides = append(nearestSides, [2]*Particle{
                                pB, targetPolyline[(ni+1)%targetSize],
                        })

                        // Check if the nearest particle is on the "wrong side" of the ray.
                        useMiniResponse := false
                        isNearestParticleOnWrongSide := true
                        for _, side := range nearestSides {
                                sideVec := side[1].GlobalPosition().Sub(side[0].GlobalPosition())
                                sidePerp := sideVec.Perpendicular()
                                if sidePerp.Dot(rayUnit) > 0 {
                                        isNearestParticleOnWrongSide = false
                                }
                        }
                        if isNearestParticleOnWrongSide {
                                nearestSides = nearestSides[:0]
                                for j := 0; j < targetSize; j++ {
                                        nj := (j + 1) % targetSize
                                        sideVec := targetPolyline[nj].GlobalPosition().Sub(targetPolyline[j].GlobalPosition())
                                        sidePerp := sideVec.Perpendicular()
                                        if sidePerp.Dot(rayUnit) > 0 {
                                                nearestSides = append(nearestSides, [2]*Particle{
                                                        targetPolyline[j], targetPolyline[nj],
                                                })
                                        }
                                }
                                useMiniResponse = true
                        }

                        penetration := float32(0)
                        var normal Vec2
                        minDistance := -float32(MaxWorldSize)

                        for n, side := range nearestSides {
                                sA := side[0]
                                sB := side[1]
                                sideVec := sB.GlobalPosition().Sub(sA.GlobalPosition())
                                sideNormal := sideVec.Normalized().Perpendicular()
                                sAPos := sA.GlobalPosition()
                                sBPos := sB.GlobalPosition()
                                if sA.Radius()+sB.Radius() > 1.0 {
                                        if sA.Radius() > 0.5 {
                                                sAPos = sAPos.Add(sideNormal.Mul(sA.Radius()))
                                        }
                                        if sB.Radius() > 0.5 {
                                                sBPos = sBPos.Add(sideNormal.Mul(sB.Radius()))
                                        }
                                        sideVec = sBPos.Sub(sAPos)
                                        sideNormal = sideVec.Normalized().Perpendicular()
                                }

                                if testSize >= 3 {
                                        // Ray-vs-segment intersection test.
                                        intersection := LineIntersectionLine(rayEndPoint, pAPos, sAPos, sBPos)
                                        if !intersection.IsNaN() {
                                                radius := pARadius
                                                bridgeVec := pAPos.Sub(sAPos)
                                                dist := bridgeVec.Dot(sideNormal)
                                                if dist < 0 && dist > minDistance {
                                                        minDistance = dist
                                                        normal = sideNormal
                                                        penetration = dist - radius
                                                        collidedSideIndex = n
                                                }
                                        }
                                } else {
                                        // Vertical projection.
                                        bridgeVec := pAPos.Sub(sAPos)
                                        dist := bridgeVec.Dot(sideNormal)
                                        radius := pARadius
                                        if dist > minDistance && dist < radius {
                                                minDistance = dist
                                                normal = sideNormal
                                                penetration = dist - radius
                                                collidedSideIndex = n
                                        }
                                }
                        }

                        // Fallback if no side found.
                        if collidedSideIndex == -1 {
                                nearestSides = nearestSides[:0]
                                minDistanceFallback := float32(MaxWorldSize)
                                var findedSide [2]*Particle
                                for n := 0; n < targetSize; n++ {
                                        sA := targetPolyline[n]
                                        sB := targetPolyline[(n+1)%targetSize]
                                        sideVec := sB.GlobalPosition().Sub(sA.GlobalPosition())
                                        sideNormal := sideVec.Normalized().Perpendicular()
                                        sAPos := sA.GlobalPosition()
                                        bridgeVec := pAPos.Sub(sAPos)
                                        dist := bridgeVec.Dot(sideNormal)
                                        if Abs(dist) < minDistanceFallback {
                                                minDistanceFallback = Abs(dist)
                                                normal = sideNormal
                                                penetration = dist
                                                findedSide = [2]*Particle{sA, sB}
                                        }
                                }
                                nearestSides = append(nearestSides, findedSide)
                                collidedSideIndex = 0
                        }

                        if useMiniResponse && world != nil {
                                penetration *= world.softBodyCollisionHysteresis
                        }

                        // Contact: configure with -penetration sign flip (matches qcollision.cpp:465).
                        c := pool.Get()
                        c.Particle = pA
                        c.Position = pAPos
                        c.Normal = normal
                        c.Penetration = -penetration
                        c.ReferenceParticles = []*Particle{
                                nearestSides[collidedSideIndex][0],
                                nearestSides[collidedSideIndex][1],
                        }
                        contacts = append(contacts, c)
                } else {
                        // D. Outside case — only if radius > 0.5.
                        if pARadius > 0.5 {
                                nsA, nsB := findNearestSideOfPolygon(pAPos, targetPolyline, true, false)
                                if nsA == -1 && nsB == -1 {
                                        continue
                                }
                                // Find which of the two side particles is nearest to pA.
                                ni := findNearestParticleOfPolygon(pA, []*Particle{targetPolyline[nsA], targetPolyline[nsB]})
                                var targetNi int
                                if ni == 0 {
                                        targetNi = nsA
                                } else {
                                        targetNi = nsB
                                }
                                pB := targetPolyline[targetNi]

                                var nearestSides [][2]*Particle
                                nearestSides = append(nearestSides, [2]*Particle{
                                        targetPolyline[((targetNi-1+targetSize)%targetSize)], pB,
                                })
                                nearestSides = append(nearestSides, [2]*Particle{
                                        pB, targetPolyline[(targetNi+1)%targetSize],
                                })

                                nearestSideIndex := -1
                                var contactPenetration float32
                                var contactNormal Vec2
                                var contactPosition Vec2
                                minDistance := float32(MaxWorldSize)

                                for is, side := range nearestSides {
                                        s1 := side[0]
                                        s2 := side[1]
                                        s1Pos := s1.GlobalPosition()
                                        s2Pos := s2.GlobalPosition()
                                        segVec := s2Pos.Sub(s1Pos)
                                        unit := segVec.Normalized()
                                        normal := unit.Perpendicular()
                                        if s1.Radius() > 0.5 || s2.Radius() > 0.5 {
                                                if s1.Radius() > 0.5 {
                                                        s1Pos = s1Pos.Add(normal.Mul(s1.Radius()))
                                                }
                                                if s2.Radius() > 0.5 {
                                                        s2Pos = s2Pos.Add(normal.Mul(s2.Radius()))
                                                }
                                                segVec = s2Pos.Sub(s1Pos)
                                                unit = segVec.Normalized()
                                                normal = unit.Perpendicular()
                                        }
                                        segLen := segVec.Length()
                                        bridgeVec := pAPos.Sub(s1Pos)
                                        testBridgeVec := pAPos.Sub(s1Pos)
                                        perpProj := testBridgeVec.Dot(normal)
                                        if Abs(perpProj) < minDistance {
                                                if Abs(perpProj) < pARadius {
                                                        proj := bridgeVec.Dot(unit)
                                                        if proj >= 0 && proj <= segLen {
                                                                projSign := float32(1)
                                                                if perpProj < 0 {
                                                                        projSign = -1
                                                                }
                                                                contactPenetration = Abs(pARadius*projSign - perpProj)
                                                                contactPosition = pAPos.Sub(normal.Mul(pARadius * projSign))
                                                                contactNormal = normal
                                                                nearestSideIndex = is
                                                                minDistance = Abs(perpProj)
                                                        }
                                                }
                                        }
                                }

                                if nearestSideIndex != -1 {
                                        s1 := nearestSides[nearestSideIndex][0]
                                        s2 := nearestSides[nearestSideIndex][1]
                                        c := pool.Get()
                                        c.Particle = pA
                                        c.Position = contactPosition
                                        c.Normal = contactNormal
                                        c.Penetration = contactPenetration
                                        c.ReferenceParticles = []*Particle{s1, s2}
                                        contacts = append(contacts, c)
                                }
                        }
                }
        }

        return contacts
}

// slicesEqual reports whether two []*Particle slices are the same (same
// length, same particle pointers in same order). Used to detect self-collision
// in polylineAndPolyline.
func slicesEqual(a, b []*Particle) bool {
        if len(a) != len(b) {
                return false
        }
        for i := range a {
                if a[i] != b[i] {
                        return false
                }
        }
        return true
}

// computePolylineAABB returns the AABB enclosing all particles in the slice.
func computePolylineAABB(polyline []*Particle) AABB {
        if len(polyline) == 0 {
                return AABB{}
        }
        min := polyline[0].GlobalPosition()
        max := min
        for _, p := range polyline[1:] {
                pos := p.GlobalPosition()
                r := p.Radius()
                if pos.X-r < min.X {
                        min.X = pos.X - r
                }
                if pos.Y-r < min.Y {
                        min.Y = pos.Y - r
                }
                if pos.X+r > max.X {
                        max.X = pos.X + r
                }
                if pos.Y+r > max.Y {
                        max.Y = pos.Y + r
                }
        }
        return AABB{Min: min, Max: max}
}

// circleAndPolyline2 checks collisions between circle particles and a
// polyline (soft body). Faithful port of QCollision::CircleAndPolyline2
// (qcollision.cpp:595-680).
//
// For each circle particle:
//  - If INSIDE the polyline (via PointInPolygonWN):
//    * Compute a bisector ray from the particle's prev/next (length 32).
//    * For each polyline edge, test ray-vs-edge intersection.
//    * Track the nearest intersection (min distance from particle to edge).
//    * Configure contact: particle = circle, reference = {sideA, sideB},
//      normal = sideVecNormal, penetration = abs(bVec · sideVecNormal).
//      Special cases when proj < 0 or proj > sideLen (vertex regions):
//      use the particle-to-vertex distance and direction instead.
//  - If OUTSIDE: no contact (this method only handles inside case; outside
//    is handled by the regular circleVsPolygon).
func circleAndPolyline2(circleParticles, polylineParticles []*Particle, pool *ContactPool) []*Contact {
        var contacts []*Contact
        particlesSize := len(circleParticles)
        polygonParticlesSize := len(polylineParticles)
        if particlesSize < 3 || polygonParticlesSize < 3 {
                // C++ requires prev/next particles for the bisector — skip if < 3.
                return nil
        }
        for i := 0; i < particlesSize; i++ {
                particle := circleParticles[i]
                if !particle.enabled {
                        continue
                }
                pPos := particle.GlobalPosition()

                checkingSide := 1 // 0:inside 1:outside
                if pointInPolygonWN(pPos, polylineParticles) {
                        checkingSide = 0
                }

                if checkingSide != 0 {
                        continue
                }

                // Inside case.
                prevParticle := circleParticles[((i-1+particlesSize)%particlesSize)]
                nextParticle := circleParticles[(i+1)%particlesSize]
                bisectorRayUnit := GetBisectorUnitVector(prevParticle.GlobalPosition(), pPos, nextParticle.GlobalPosition(), false)
                bisectorRay := bisectorRayUnit.Mul(32.0)

                minIntersectionSideDistance := float32(MaxWorldSize)
                minIntersectionSideIndex := -1
                for j := 0; j < polygonParticlesSize; j++ {
                        sideParticleA := polylineParticles[j]
                        sideParticleB := polylineParticles[(j+1)%polygonParticlesSize]
                        intersection := LineIntersectionLine(
                                pPos, pPos.Add(bisectorRay),
                                sideParticleA.GlobalPosition(), sideParticleB.GlobalPosition(),
                        )
                        if !intersection.IsNaN() {
                                distance := (intersection.Sub(pPos)).Length()
                                if distance < minIntersectionSideDistance {
                                        // NOTE: C++ qcollision.cpp:632 has a dead-code assignment
                                        // `minIntersectionSideIndex=distance;` (immediately overwritten
                                        // by `minIntersectionSideIndex=j;`). We port only the live one.
                                        minIntersectionSideDistance = distance
                                        minIntersectionSideIndex = j
                                }
                        }
                }

                if minIntersectionSideIndex == -1 {
                        continue
                }

                sideParticleA := polylineParticles[minIntersectionSideIndex]
                sideParticleB := polylineParticles[(minIntersectionSideIndex+1)%polygonParticlesSize]
                sideVec := sideParticleB.GlobalPosition().Sub(sideParticleA.GlobalPosition())
                sideVecUnit := sideVec.Normalized()
                sideVecNormal := sideVecUnit.Perpendicular()
                bVec := pPos.Sub(sideParticleA.GlobalPosition())

                penetration := bVec.Dot(sideVecNormal)
                normal := sideVecNormal

                proj := bVec.Dot(sideVecUnit)
                if proj < 0.0 {
                        // Vertex region A: use distance to sideParticleA.
                        penetration = (sideParticleA.GlobalPosition().Sub(pPos)).Length()
                        normal = (sideParticleA.GlobalPosition().Sub(pPos)).Normalized()
                }
                if proj > sideVec.Length() {
                        // Vertex region B: use distance to sideParticleB.
                        penetration = (sideParticleB.GlobalPosition().Sub(pPos)).Length()
                        normal = (sideParticleB.GlobalPosition().Sub(pPos)).Normalized()
                }

                c := pool.Get()
                c.Particle = particle
                c.Position = pPos
                c.Normal = normal
                c.Penetration = Abs(penetration)
                c.ReferenceParticles = []*Particle{sideParticleA, sideParticleB}
                contacts = append(contacts, c)
        }
        return contacts
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
