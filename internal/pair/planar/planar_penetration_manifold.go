package planar

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// PlanarPenetrationManifold is the shallow-penetration patch of two convex
// solids (§9.3): over every face normal and edge cross product, the directed
// translation that moves B clear of A must have one strictly smallest length,
// its direction must be the outward normal of a face of A or the inward
// normal of a face of B, and the two bodies must cross along it rather than
// one holding the other. When it is both, the faces' projected overlap along
// that direction must have positive area: the patch is clipped on A's face
// plane, and each corner pairs that point with its translate on B's face.
// When only one body has the face, the other pokes through it with an edge or
// a vertex, its deepest along the direction (shallowSupport). The separation
// is minus the translation length. A nil result withholds the manifold. poll
// is charged once per axis and inside the clip.
func PlanarPenetrationManifold(a, b *PlanarSolid, poll func() error) ([]PatchPoint, error) {
	points, _, err := PlanarPenetrationSupport(a, b, poll)
	return points, err
}

// PlanarPenetrationSupport is PlanarPenetrationManifold that also names the
// host face a shallow edge or vertex pokes through (shallowSupport), or nil
// when the patch is a face pair or is withheld.
func PlanarPenetrationSupport(a, b *PlanarSolid, poll func() error) ([]PatchPoint, *SupportPlane, error) {
	if len(a.Faces) != len(a.Tris) || len(b.Faces) != len(b.Tris) {
		return nil, nil, nil
	}
	sa, sb := newPatchSide(a, true, true), newPatchSide(b, true, false)
	edgesA := uniqueEdgeDirections(a.Verts, sa.prep.edges)
	edgesB := uniqueEdgeDirections(b.Verts, sb.prep.edges)
	axes := make([]proof.DyV3, 0, len(a.Tris)+len(b.Tris)+len(edgesA)*len(edgesB))
	axes = append(axes, sa.prep.normal...)
	axes = append(axes, sb.prep.normal...)
	for _, da := range edgesA {
		for _, db := range edgesB {
			axes = append(axes, proof.DvCross(da, db))
		}
	}
	var best frac
	var bestDir proof.DyV3
	var bestT proof.Dyadic
	found, tied := false, false
	for _, u := range axes {
		if err := poll(); err != nil {
			return nil, nil, err
		}
		if proof.DvIsZero(u) {
			continue
		}
		minA, maxA := projectSpan(a.Verts, u)
		minB, maxB := projectSpan(b.Verts, u)
		norm := proof.DvDot(u, u)
		for _, candidate := range []struct {
			t   proof.Dyadic
			dir proof.DyV3
		}{{proof.DySubScalar(maxA, minB), u}, {proof.DySubScalar(maxB, minA), dvNeg(u)}} {
			if candidate.t.Sign() <= 0 {
				return nil, nil, nil
			}
			value := frac{num: proof.DyMul(candidate.t, candidate.t), den: norm}
			switch c := fracCmp(value, best); {
			case !found || c < 0:
				best, bestDir, bestT, found, tied = value, candidate.dir, candidate.t, true, false
			case c == 0 && !sameDirection(candidate.dir, bestDir):
				tied = true
			}
		}
	}
	if !found || tied {
		return nil, nil, nil
	}
	faceA, okA := supportFace(sa, bestDir)
	faceB, okB := supportFace(sb, dvNeg(bestDir))
	minA, maxA := projectSpan(a.Verts, bestDir)
	minB, maxB := projectSpan(b.Verts, bestDir)
	if proof.DyCmp(maxB, maxA) <= 0 || proof.DyCmp(minA, minB) >= 0 {
		return nil, nil, nil
	}
	if okA != okB {
		depth, ok := canonicalSqrt(best)
		if !ok {
			return nil, nil, nil
		}
		separation := pair.ScalarReading{ValueMM: -depth.ValueMM, BoundMM: depth.BoundMM}
		if okA {
			points, err := shallowSupport(sa, faceA, sb, bestDir, minB, maxA, separation, poll)
			return points, &SupportPlane{HostIsA: true, Face: faceA}, err
		}
		points, err := shallowSupport(sb, faceB, sa, dvNeg(bestDir), dvNeg1(maxA), dvNeg1(minB), separation, poll)
		return points, &SupportPlane{Face: faceB}, err
	}
	if !okA || !sa.isFlat(faceA) || !sb.isFlat(faceB) {
		return nil, nil, nil
	}
	// B moves by shift = t·d/(d·d) onto A's face plane.
	norm := proof.DvDot(bestDir, bestDir).Rat()
	var shift, back Point3
	for axis := range 3 {
		shift[axis] = new(big.Rat).Quo(new(big.Rat).Mul(bestT.Rat(), bestDir[axis].Rat()), norm)
		back[axis] = new(big.Rat).Neg(shift[axis])
	}
	frame := NewPlaneFrame(bestDir, sa.faceOrigin(faceA))
	clipOuter, clipHoles, okClip := sa.frameLoops(faceA, frame, Point3{})
	outer, holes, okSubject := sb.frameLoops(faceB, frame, shift)
	if !okClip || !okSubject || len(clipHoles) > 0 || len(holes) > 0 {
		return nil, nil, nil
	}
	clip := clipOuter
	if !IsConvex(clip) {
		return nil, nil, nil
	}
	polygon, err := ClipConvex(outer, clip, poll)
	if err != nil {
		return nil, nil, err
	}
	if len(polygon) < 3 || DoubleArea(polygon).Sign() == 0 {
		return nil, nil, nil
	}
	depth, ok := canonicalSqrt(best)
	if !ok {
		return nil, nil, nil
	}
	separation := pair.ScalarReading{ValueMM: -depth.ValueMM, BoundMM: depth.BoundMM}
	featureA := PatchFeature{Kind: FeatureFacet, Faces: []int{faceA}}
	featureB := PatchFeature{Kind: FeatureFacet, Faces: []int{faceB}}
	var points []PatchPoint
	for _, corner := range extremalVertices(polygon) {
		onA := frame.Lift(corner)
		var onB Point3
		for axis := range 3 {
			onB[axis] = new(big.Rat).Add(onA[axis], back[axis])
		}
		points = append(points, PatchPoint{OnA: onA, OnB: onB, A: featureA, B: featureB,
			Normal: bestDir, Separation: separation})
	}
	return points, nil, nil
}

// uniqueEdgeDirections retains the first edge in each parallel direction.
// Opposite edges share an axis: the penetration scan checks both translations
// along each axis, so their cross products give the same candidates.
func uniqueEdgeDirections(verts []proof.DyV3, edges [][2]int) []proof.DyV3 {
	directions := make([]proof.DyV3, 0, len(edges))
	for _, edge := range edges {
		direction := proof.DvSub(verts[edge[1]], verts[edge[0]])
		if proof.DvIsZero(direction) {
			continue
		}
		duplicate := false
		for _, held := range directions {
			if proof.DvIsZero(proof.DvCross(direction, held)) {
				duplicate = true
				break
			}
		}
		if !duplicate {
			directions = append(directions, direction)
		}
	}
	return directions
}

// shallowSupport publishes a convex guest poking through face h of the host
// along n, the face's outward normal: the guest's deepest vertices along n,
// at n·v = level, must be one vertex or one edge, and each foot on the face
// plane n·x = plane must lie in the face's interior, as §9.3's support row
// requires of a touch. Each point pairs the guest vertex with its foot, and
// carries the separation.
func shallowSupport(host *patchSide, h int, guest *patchSide, n proof.DyV3, level, plane proof.Dyadic,
	separation pair.ScalarReading, poll func() error) ([]PatchPoint, error) {
	if !host.isFlat(h) {
		return nil, nil
	}
	verts := guest.prep.s.Verts
	var deepest []int
	for v, at := range verts {
		if err := poll(); err != nil {
			return nil, err
		}
		if proof.DyCmp(proof.DvDot(n, at), level) == 0 {
			deepest = append(deepest, v)
		}
	}
	if len(deepest) == 0 {
		return nil, nil
	}
	// The foot moves each deepest vertex along n onto the face plane.
	norm := proof.DvDot(n, n).Rat()
	lift := new(big.Rat).Quo(proof.DySubScalar(plane, level).Rat(), norm)
	foot := func(v int) Point3 {
		at := ratPoint3(verts[v])
		for axis := range 3 {
			at[axis] = new(big.Rat).Add(at[axis], new(big.Rat).Mul(lift, n[axis].Rat()))
		}
		return at
	}
	frame := NewPlaneFrame(n, host.faceOrigin(h))
	outer, holes, ok := host.frameLoops(h, frame, Point3{})
	if !ok {
		return nil, nil
	}
	region := append([][]Point2{outer}, holes...)
	hostFeature := PatchFeature{Kind: FeatureFacet, Faces: []int{h}}
	point := func(at Point3, guestAt Point3, feature PatchFeature) PatchPoint {
		p := PatchPoint{OnA: at, OnB: guestAt, A: hostFeature, B: feature, Normal: n, Separation: separation}
		if !host.isA {
			p.OnA, p.OnB, p.A, p.B, p.Normal = guestAt, at, feature, hostFeature, dvNeg(n)
		}
		return p
	}
	unlift := func(at Point3) Point3 {
		var out Point3
		for axis := range 3 {
			out[axis] = new(big.Rat).Sub(at[axis], new(big.Rat).Mul(lift, n[axis].Rat()))
		}
		return out
	}
	if len(deepest) == 1 {
		at := foot(deepest[0])
		if locate(frame.Project(at), region) <= 0 {
			return nil, nil
		}
		feature := PatchFeature{Kind: FeatureVertex, Faces: guest.faceIDs(guest.vertTris[deepest[0]])}
		return []PatchPoint{point(at, ratPoint3(verts[deepest[0]]), feature)}, nil
	}
	// A convex guest's deepest set with no triangle in it is one edge, so its
	// vertices are collinear.
	for _, tri := range guest.prep.s.Tris {
		if slices.Contains(deepest, tri[0]) && slices.Contains(deepest, tri[1]) && slices.Contains(deepest, tri[2]) {
			return nil, nil
		}
	}
	p := verts[deepest[0]]
	d := proof.DvSub(verts[deepest[1]], p)
	lo, hi := deepest[0], deepest[0]
	for _, v := range deepest {
		rel := proof.DvSub(verts[v], p)
		if !proof.DvIsZero(proof.DvCross(rel, d)) {
			return nil, nil
		}
		if proof.DyCmp(proof.DvDot(rel, d), proof.DvDot(proof.DvSub(verts[lo], p), d)) < 0 {
			lo = v
		}
		if proof.DyCmp(proof.DvDot(rel, d), proof.DvDot(proof.DvSub(verts[hi], p), d)) > 0 {
			hi = v
		}
	}
	start, end := foot(lo), foot(hi)
	spans, err := segmentSpans(frame.Project(start), frame.Project(end), region, poll)
	if err != nil || len(spans) == 0 {
		return nil, err
	}
	mid := hpoint{x: proof.DvAdd(verts[lo], verts[hi]), w: proof.DyInt(2)}
	var holding []int
	for t := range guest.prep.s.Tris {
		if pointInFacet(guest.prep, t, mid) {
			holding = append(holding, t)
		}
	}
	feature := PatchFeature{Kind: FeatureEdge, Faces: guest.faceIDs(holding)}
	if len(feature.Faces) != 2 {
		return nil, nil
	}
	var points []PatchPoint
	for _, s := range spans {
		if !s.interior {
			return nil, nil
		}
		for _, t := range []*big.Rat{s.lo, s.hi} {
			at := lerp3(start, end, t)
			points = append(points, point(at, unlift(at), feature))
		}
	}
	return points, nil
}

func dvNeg1(x proof.Dyadic) proof.Dyadic { return proof.DyNeg(x) }

func projectSpan(verts []proof.DyV3, u proof.DyV3) (proof.Dyadic, proof.Dyadic) {
	lo := proof.DvDot(verts[0], u)
	hi := lo
	for _, v := range verts[1:] {
		value := proof.DvDot(v, u)
		lo, hi = dyMin(lo, value), dyMax(hi, value)
	}
	return lo, hi
}

func sameDirection(u, v proof.DyV3) bool {
	return proof.DvIsZero(proof.DvCross(u, v)) && proof.DvDot(u, v).Sign() > 0
}

// supportFace returns the one face whose triangles have outward normals
// along dir.
func supportFace(side *patchSide, dir proof.DyV3) (int, bool) {
	var tris []int
	for t, n := range side.prep.normal {
		if sameDirection(n, dir) {
			tris = append(tris, t)
		}
	}
	faces := side.faceIDs(tris)
	if len(faces) != 1 {
		return 0, false
	}
	return faces[0], true
}

// canonicalSqrt encloses the square root of a nonnegative rational after
// reducing it to lowest terms, so equal values give identical readings.
func canonicalSqrt(x frac) (pair.ScalarReading, bool) {
	value := new(big.Rat).Quo(x.num.Rat(), x.den.Rat())
	num, okNum := proof.DyOfRat(new(big.Rat).SetInt(value.Num()))
	den, okDen := proof.DyOfRat(new(big.Rat).SetInt(value.Denom()))
	if !okNum || !okDen {
		return pair.ScalarReading{}, false
	}
	return fracSqrtReading(frac{num: num, den: den})
}
