package pair

import (
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the face-local shallow-penetration patch of
// docs/multibody-dynamics-design.md §9.6: a convex solid M whose corner or
// edge pokes through one flat face h of any planar solid S, convex or not.
// The patch is published only under four exact tests, each sound on its own
// terms:
//
//  1. One crossed face. Every crossing ClassifyPlanar recorded lies in a
//     triangle of h or in an edge of S whose two triangles both belong to h.
//     A crossing in two faces withholds the patch with AmbiguousFeature.
//  2. One deepest feature. M's vertices at the least height below h's plane
//     are one vertex or the two ends of one edge (shallowSupport).
//  3. The sunk part lies over the face. The projection along h's normal of
//     every vertex of M on or behind the plane and of every point where an
//     edge of M crosses the plane lies strictly inside h's region, no edge of
//     their convex hull meets a loop of h, and no loop vertex of h lies in
//     that hull. M is convex, so its part behind the plane is the hull of
//     those points, which therefore lies over h's material.
//  4. The column is clear. Every triangle of S with a vertex strictly in
//     front of the plane projects strictly apart from M's vertex box
//     (PlanarColumnClear with the zero horizon), so S's material in front of
//     the plane meets no part of M.
//
// With all four, M ∩ S lies behind h's plane inside M's sunk part, and
// moving M along h's normal by the depth d of its deepest feature clears it.

// PlanarFacePenetration publishes the face-local patch of an Overlapping
// pair from the crossings ClassifyPlanar recorded. Each convex solid in turn
// is tried as M against the other as S; the patch is published when exactly
// one order publishes, and two publishing orders withhold it with
// AmbiguousFeature. The points are §9.3's shallow row: each deepest vertex
// paired with its exact foot on h's plane, h's outward normal oriented A to
// B, and a Separation enclosing −d. Supports names h, whose lifted set
// (PlanarSupportSet with overlap) a positive band appends. A withheld patch
// carries AmbiguousFeature when the overlap crossed two faces of S, and no
// reason otherwise. Both snapshots must carry Faces. poll is charged
// throughout; its error is returned unchanged.
func PlanarFacePenetration(a, b *PlanarSolid, crossings []PlanarCrossing, convexA, convexB bool,
	poll func() error) (PlanarManifold, error) {
	if len(a.Faces) != len(a.Tris) || len(b.Faces) != len(b.Tris) || len(crossings) == 0 {
		return PlanarManifold{}, nil
	}
	var published []PlanarManifold
	reason := NoReason
	for _, mIsA := range []bool{true, false} {
		if mIsA && !convexA || !mIsA && !convexB {
			continue
		}
		got, err := facePenetration(a, b, crossings, mIsA, poll)
		if err != nil {
			return PlanarManifold{}, err
		}
		if got.Points != nil {
			published = append(published, got)
		} else if got.Reason != NoReason {
			reason = got.Reason
		}
	}
	switch len(published) {
	case 1:
		return published[0], nil
	case 0:
		return PlanarManifold{Reason: reason}, nil
	default:
		return PlanarManifold{Reason: AmbiguousFeature}, nil
	}
}

// facePenetration runs the four tests with M the A solid when mIsA.
func facePenetration(a, b *PlanarSolid, crossings []PlanarCrossing, mIsA bool,
	poll func() error) (PlanarManifold, error) {
	mSolid, sSolid := b, a
	if mIsA {
		mSolid, sSolid = a, b
	}
	host := newPatchSide(sSolid, false, !mIsA)
	guest := newPatchSide(mSolid, true, mIsA)

	// 1. Every crossing's S part lies in one face h.
	h := -1
	for _, crossing := range crossings {
		if err := poll(); err != nil {
			return PlanarManifold{}, err
		}
		part := crossing.B
		if !mIsA {
			part = crossing.A
		}
		for _, t := range part.Facets {
			switch face := sSolid.Faces[t]; {
			case h < 0:
				h = face
			case face != h:
				return PlanarManifold{Reason: AmbiguousFeature}, nil
			}
		}
	}
	if h < 0 || !host.isFlat(h) {
		return PlanarManifold{}, nil
	}

	n, q := host.faceNormal(h), host.faceOrigin(h)
	plane := proof.DvDot(n, q)
	heights := make([]proof.Dyadic, len(mSolid.Verts))
	level := proof.DyZero()
	for v, at := range mSolid.Verts {
		if err := poll(); err != nil {
			return PlanarManifold{}, err
		}
		heights[v] = proof.DvDot(n, at)
		if v == 0 || proof.DyCmp(heights[v], level) < 0 {
			level = heights[v]
		}
	}
	if proof.DyCmp(level, plane) >= 0 {
		return PlanarManifold{}, nil
	}

	// 3. The sunk part's projection lies strictly inside h's region.
	frame := NewPlaneFrame(n, q)
	outer, holes, ok := host.frameLoops(h, frame, Point3{})
	if !ok {
		return PlanarManifold{}, nil
	}
	region := append([][]Point2{outer}, holes...)
	sunk, err := sunkOutline(guest, heights, plane, frame, poll)
	if err != nil {
		return PlanarManifold{}, err
	}
	inside, err := hullInsideRegion(sunk, region, poll)
	if err != nil || !inside {
		return PlanarManifold{}, err
	}

	// 4. S's material in front of the plane stays clear of M's column.
	lo, hi := vertexBox(mSolid.Verts)
	_, columnClear, err := PlanarColumnClear(sSolid, n, q, lo, hi, poll)
	if err != nil || !columnClear {
		return PlanarManifold{}, err
	}

	// 2. One deepest vertex or edge, published as §9.3's shallow row.
	depth, ok := canonicalSqrt(frac{num: proof.DyMul(proof.DySubScalar(plane, level), proof.DySubScalar(plane, level)),
		den: proof.DvDot(n, n)})
	if !ok {
		return PlanarManifold{}, nil
	}
	separation := ScalarReading{ValueMM: -depth.ValueMM, BoundMM: depth.BoundMM}
	points, err := shallowSupport(host, h, guest, n, level, plane, separation, poll)
	if err != nil || points == nil {
		return PlanarManifold{}, err
	}
	return PlanarManifold{Points: points, Supports: []SupportPlane{{HostIsA: host.isA, Face: h}}}, nil
}

// sunkOutline projects M's part on or behind the plane n·x = plane: every
// vertex at or below it and the exact point where each edge crosses it.
func sunkOutline(m *patchSide, heights []proof.Dyadic, plane proof.Dyadic, frame PlaneFrame,
	poll func() error) ([]Point2, error) {
	verts := m.prep.s.Verts
	var out []Point2
	for v, height := range heights {
		if proof.DyCmp(height, plane) <= 0 {
			out = append(out, frame.Project(ratPoint3(verts[v])))
		}
	}
	for _, edge := range m.prep.edges {
		if err := poll(); err != nil {
			return nil, err
		}
		hp := proof.DySubScalar(heights[edge[0]], plane)
		hq := proof.DySubScalar(heights[edge[1]], plane)
		if hp.Sign()*hq.Sign() >= 0 {
			continue
		}
		// p + t·(q − p) with t = hp / (hp − hq) lies on the plane.
		t := new(big.Rat).Quo(hp.Rat(), proof.DySubScalar(hp, hq).Rat())
		out = append(out, frame.Project(lerp3(ratPoint3(verts[edge[0]]), ratPoint3(verts[edge[1]]), t)))
	}
	return out, nil
}

// hullInsideRegion reports whether the convex hull of points lies strictly
// inside the region the loops bound: every point strictly inside, no hull
// edge meeting a loop edge, and no loop vertex in the closed hull. With the
// first two, the hull's boundary lies in the region's interior, so a loop can
// reach the hull only by lying wholly inside it, which the third refuses.
func hullInsideRegion(points []Point2, loops [][]Point2, poll func() error) (bool, error) {
	if len(points) == 0 {
		return false, nil
	}
	for _, p := range points {
		if err := poll(); err != nil {
			return false, err
		}
		if locate(p, loops) <= 0 {
			return false, nil
		}
	}
	hull := convexHull2(points)
	for i, u := range hull {
		w := hull[(i+1)%len(hull)]
		for _, loop := range loops {
			for j, a := range loop {
				if err := poll(); err != nil {
					return false, err
				}
				if segmentsMeet(u, w, a, loop[(j+1)%len(loop)]) {
					return false, nil
				}
			}
		}
	}
	for _, loop := range loops {
		for _, a := range loop {
			if err := poll(); err != nil {
				return false, err
			}
			if inConvexHull(a, hull) {
				return false, nil
			}
		}
	}
	return true, nil
}

// convexHull2 returns the corners of the points' convex hull counterclockwise
// (Andrew's monotone chain), dropping collinear points. A degenerate hull is
// one point or the two ends of a segment.
func convexHull2(points []Point2) []Point2 {
	sorted := slices.Clone(points)
	sort.Slice(sorted, func(i, j int) bool {
		if c := sorted[i].X.Cmp(sorted[j].X); c != 0 {
			return c < 0
		}
		return sorted[i].Y.Cmp(sorted[j].Y) < 0
	})
	sorted = uniquePoints(sorted)
	if len(sorted) < 3 {
		return sorted
	}
	turn := func(o, a, b Point2) int { return cross2(sub2(a, o), sub2(b, o)).Sign() }
	var hull []Point2
	for pass := range 2 {
		start := len(hull)
		for i := range sorted {
			p := sorted[i]
			if pass == 1 {
				p = sorted[len(sorted)-1-i]
			}
			for len(hull) >= start+2 && turn(hull[len(hull)-2], hull[len(hull)-1], p) <= 0 {
				hull = hull[:len(hull)-1]
			}
			hull = append(hull, p)
		}
		hull = hull[:len(hull)-1]
	}
	if len(hull) < 3 {
		return []Point2{sorted[0], sorted[len(sorted)-1]}
	}
	return hull
}

// inConvexHull reports whether x lies in the closed convex hull, given as
// convexHull2 returns it.
func inConvexHull(x Point2, hull []Point2) bool {
	switch len(hull) {
	case 1:
		return equal2(x, hull[0])
	case 2:
		return onSegment2(x, hull[0], hull[1])
	}
	for i, u := range hull {
		if cross2(sub2(hull[(i+1)%len(hull)], u), sub2(x, u)).Sign() < 0 {
			return false
		}
	}
	return true
}

// onSegment2 reports whether x lies on the closed segment uw.
func onSegment2(x, u, w Point2) bool {
	e, rel := sub2(w, u), sub2(x, u)
	return cross2(e, rel).Sign() == 0 && dot2(e, rel).Sign() >= 0 && dot2(e, rel).Cmp(dot2(e, e)) <= 0
}

// segmentsMeet reports whether the closed segments pq and uw share a point.
func segmentsMeet(p, q, u, w Point2) bool {
	d1 := cross2(sub2(q, p), sub2(u, p)).Sign()
	d2 := cross2(sub2(q, p), sub2(w, p)).Sign()
	d3 := cross2(sub2(w, u), sub2(p, u)).Sign()
	d4 := cross2(sub2(w, u), sub2(q, u)).Sign()
	if d1*d2 < 0 && d3*d4 < 0 {
		return true
	}
	return onSegment2(u, p, q) || onSegment2(w, p, q) || onSegment2(p, u, w) || onSegment2(q, u, w)
}

// vertexBox is the exact coordinate box of the vertices.
func vertexBox(verts []proof.DyV3) ([3]*big.Rat, [3]*big.Rat) {
	indices := make([]int, len(verts))
	for i := range indices {
		indices[i] = i
	}
	dlo, dhi := pointBox(verts, indices)
	var lo, hi [3]*big.Rat
	for axis := range 3 {
		lo[axis], hi[axis] = dlo[axis].Rat(), dhi[axis].Rat()
	}
	return lo, hi
}
