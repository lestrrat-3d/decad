package box

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// OrientedBox is a parallelotope whose corners are exact dyadic readings of
// the source box under its recorded placement and query pose.
type OrientedBox struct {
	Corner [8]proofarith.DyV3
	Edge   [3]proofarith.DyV3
}

// OrientedBoxRelation applies the complete separating-axis test. Every axis
// and projection is a polynomial of held float entries, hence exact dyadic.
func OrientedBoxRelation(a, b OrientedBox) (pair.Relation, proofarith.Dyadic, proofarith.Dyadic) {
	faceAxes := func(box OrientedBox) [3]proofarith.DyV3 {
		return [3]proofarith.DyV3{proofarith.DvCross(box.Edge[1], box.Edge[2]),
			proofarith.DvCross(box.Edge[2], box.Edge[0]), proofarith.DvCross(box.Edge[0], box.Edge[1])}
	}
	axisA, axisB := faceAxes(a), faceAxes(b)
	axes := make([]proofarith.DyV3, 0, 15)
	axes = append(axes, axisA[:]...)
	axes = append(axes, axisB[:]...)
	for _, ea := range a.Edge {
		for _, eb := range b.Edge {
			axes = append(axes, proofarith.DvCross(ea, eb))
		}
	}
	touch := false
	bestGap, bestNormSquared := proofarith.DyZero(), proofarith.DyZero()
	for _, axis := range axes {
		if proofarith.DvIsZero(axis) {
			continue
		}
		alo, ahi := OrientedProjection(a, axis)
		blo, bhi := OrientedProjection(b, axis)
		gap := proofarith.DyZero()
		if proofarith.DyCmp(ahi, blo) < 0 {
			gap = proofarith.DySubScalar(blo, ahi)
		} else if proofarith.DyCmp(bhi, alo) < 0 {
			gap = proofarith.DySubScalar(alo, bhi)
		} else if proofarith.DyCmp(ahi, blo) == 0 || proofarith.DyCmp(bhi, alo) == 0 {
			touch = true
		}
		if gap.Sign() <= 0 {
			continue
		}
		normSquared := proofarith.DvDot(axis, axis)
		if bestGap.Sign() == 0 ||
			new(big.Rat).Quo(proofarith.DyMul(gap, gap).Rat(), normSquared.Rat()).Cmp(
				new(big.Rat).Quo(proofarith.DyMul(bestGap, bestGap).Rat(), bestNormSquared.Rat())) > 0 {
			bestGap, bestNormSquared = gap, normSquared
		}
	}
	if bestGap.Sign() > 0 {
		return pair.Separated, bestGap, bestNormSquared
	}
	if touch {
		return pair.Touching, proofarith.DyZero(), proofarith.DyZero()
	}
	return pair.Overlapping, proofarith.DyZero(), proofarith.DyZero()
}

func OrientedProjection(box OrientedBox, axis proofarith.DyV3) (proofarith.Dyadic, proofarith.Dyadic) {
	lo, hi := proofarith.DvDot(box.Corner[0], axis), proofarith.DvDot(box.Corner[0], axis)
	for _, point := range box.Corner[1:] {
		value := proofarith.DvDot(point, axis)
		lo, hi = dyMin(lo, value), dyMax(hi, value)
	}
	return lo, hi
}

// OrientedBoxGap encloses the true minimum distance. SAT supplies a lower
// bound; actual vertex/face point pairs supply upper bounds.
func OrientedBoxGap(a, b OrientedBox, gap, normSquared proofarith.Dyadic) (pair.ScalarReading, bool) {
	normUp := proofbound.RatSqrtUp(normSquared.Rat())
	if !finite(normUp) || normUp <= 0 {
		return pair.ScalarReading{}, false
	}
	lower := new(big.Rat).Quo(gap.Rat(), proofarith.FloatRat(normUp))
	if lower.Sign() <= 0 {
		return pair.ScalarReading{}, false
	}
	var upperSquared *big.Rat
	consider := func(candidate *big.Rat) {
		if candidate != nil && (upperSquared == nil || candidate.Cmp(upperSquared) < 0) {
			upperSquared = candidate
		}
	}
	facesA, validA := prepareOrientedFaceProjectors(a)
	facesB, validB := prepareOrientedFaceProjectors(b)
	for _, va := range a.Corner {
		for _, vb := range b.Corner {
			delta := proofarith.DvSub(va, vb)
			consider(proofarith.DvDot(delta, delta).Rat())
		}
		for index := range facesB {
			if validB[index] {
				if foot, ok := facesB[index].project(va); ok {
					consider(foot.distanceSquared())
				}
			}
		}
	}
	for _, vb := range b.Corner {
		for index := range facesA {
			if validA[index] {
				if foot, ok := facesA[index].project(vb); ok {
					consider(foot.distanceSquared())
				}
			}
		}
	}
	if upperSquared == nil {
		return pair.ScalarReading{}, false
	}
	upperFloat := proofbound.RatSqrtUp(upperSquared)
	if !finite(upperFloat) {
		return pair.ScalarReading{}, false
	}
	upper := proofarith.FloatRat(upperFloat)
	if upper.Cmp(lower) < 0 {
		return pair.ScalarReading{}, false
	}
	mid := new(big.Rat).Quo(new(big.Rat).Add(lower, upper), big.NewRat(2, 1))
	value, _ := mid.Float64()
	if !finite(value) {
		return pair.ScalarReading{}, false
	}
	held := proofarith.FloatRat(value)
	deviation := new(big.Rat).Sub(held, lower)
	deviation.Abs(deviation)
	other := new(big.Rat).Sub(upper, held)
	other.Abs(other)
	if other.Cmp(deviation) > 0 {
		deviation = other
	}
	bound := proofbound.RatFloatUp(deviation)
	if !finite(bound) || new(big.Rat).Sub(held, proofarith.FloatRat(bound)).Sign() <= 0 {
		return pair.ScalarReading{}, false
	}
	return pair.ScalarReading{ValueMM: value, BoundMM: bound}, true
}

func OrientedVertexFaceDistanceSquared(vertex proofarith.DyV3, box OrientedBox, axis, side int) *big.Rat {
	foot, ok := orientedVertexFaceProjection(vertex, box, axis, side)
	if !ok {
		return nil
	}
	return foot.distanceSquared()
}

func OrientedVertexFaceFoot(vertex proofarith.DyV3, box OrientedBox, axis, side int) ([3]*big.Rat, *big.Rat) {
	foot, ok := orientedVertexFaceProjection(vertex, box, axis, side)
	if !ok {
		return [3]*big.Rat{}, nil
	}
	var at [3]*big.Rat
	normSquared := foot.normSquared.Rat()
	for k := range 3 {
		at[k] = new(big.Rat).Quo(foot.scaled[k].Rat(), normSquared)
	}
	return at, foot.distanceSquared()
}

// orientedFaceProjection is a vertex's orthogonal projection onto a box face's
// plane, held exactly as Dyadic numerators over the face normal's squared
// length: the foot is scaled/normSquared and the squared distance to it
// distance²/normSquared.
type orientedFaceProjection struct {
	scaled      proofarith.DyV3
	distance    proofarith.Dyadic
	normSquared proofarith.Dyadic
}

func (f orientedFaceProjection) distanceSquared() *big.Rat {
	return new(big.Rat).Quo(proofarith.DyMul(f.distance, f.distance).Rat(), f.normSquared.Rat())
}

// orientedVertexFaceProjection projects vertex onto the face of box on the
// given axis and side, ok only when the foot lies in the closed face. With
// a, b the face's edges, n = a × b, N = n·n and w the vertex less the face
// origin, the foot is the origin plus P/N, P = N·w − (w·n)·n, and its face
// coordinates are u = U/(N·det) and v = V/(N·det), det = (a·a)(b·b) − (a·b)²,
// U = (P·a)(b·b) − (P·b)(a·b), V = (P·b)(a·a) − (P·a)(a·b). Every quantity is
// a polynomial in held coordinates, so it is Dyadic, and the face test
// 0 ≤ u, v ≤ 1 compares U and V against 0 and N·det exactly.
func orientedVertexFaceProjection(vertex proofarith.DyV3, box OrientedBox, axis, side int) (orientedFaceProjection, bool) {
	projector, ok := prepareOrientedFaceProjector(box, axis, side)
	if !ok {
		return orientedFaceProjection{}, false
	}
	return projector.project(vertex)
}

type orientedFaceProjector struct {
	face, a, b, normal  proofarith.DyV3
	normSquared, aa, bb proofarith.Dyadic
	ab, whole           proofarith.Dyadic
}

func prepareOrientedFaceProjectors(box OrientedBox) ([6]orientedFaceProjector, [6]bool) {
	var projectors [6]orientedFaceProjector
	var usable [6]bool
	for axis := range 3 {
		for side := range 2 {
			projectors[axis*2+side], usable[axis*2+side] = prepareOrientedFaceProjector(box, axis, side)
		}
	}
	return projectors, usable
}

func prepareOrientedFaceProjector(box OrientedBox, axis, side int) (orientedFaceProjector, bool) {
	i, j := (axis+1)%3, (axis+2)%3
	face := box.Corner[0]
	if side == 1 {
		face = proofarith.DvAdd(face, box.Edge[axis])
	}
	a, b := box.Edge[i], box.Edge[j]
	normal := proofarith.DvCross(a, b)
	normSquared := proofarith.DvDot(normal, normal)
	if normSquared.Sign() == 0 {
		return orientedFaceProjector{}, false
	}
	aa, bb, ab := proofarith.DvDot(a, a), proofarith.DvDot(b, b), proofarith.DvDot(a, b)
	det := proofarith.DySubScalar(proofarith.DyMul(aa, bb), proofarith.DyMul(ab, ab))
	if det.Sign() <= 0 {
		return orientedFaceProjector{}, false
	}
	return orientedFaceProjector{face: face, a: a, b: b, normal: normal, normSquared: normSquared,
		aa: aa, bb: bb, ab: ab, whole: proofarith.DyMul(normSquared, det)}, true
}

func (p *orientedFaceProjector) project(vertex proofarith.DyV3) (orientedFaceProjection, bool) {
	w := proofarith.DvSub(vertex, p.face)
	distance := proofarith.DvDot(w, p.normal)
	var point proofarith.DyV3
	for k := range 3 {
		point[k] = proofarith.DySubScalar(proofarith.DyMul(w[k], p.normSquared),
			proofarith.DyMul(p.normal[k], distance))
	}
	pa, pb := proofarith.DvDot(point, p.a), proofarith.DvDot(point, p.b)
	u := proofarith.DySubScalar(proofarith.DyMul(pa, p.bb), proofarith.DyMul(pb, p.ab))
	v := proofarith.DySubScalar(proofarith.DyMul(pb, p.aa), proofarith.DyMul(pa, p.ab))
	if u.Sign() < 0 || v.Sign() < 0 || proofarith.DyCmp(u, p.whole) > 0 || proofarith.DyCmp(v, p.whole) > 0 {
		return orientedFaceProjection{}, false
	}
	var foot proofarith.DyV3
	for k := range 3 {
		foot[k] = proofarith.DyAdd(proofarith.DyMul(p.face[k], p.normSquared), point[k])
	}
	return orientedFaceProjection{scaled: foot, distance: distance, normSquared: p.normSquared}, true
}

// An actual point strictly inside both η-eroded read boxes is also inside
// both ideal boxes: every support plane moves by at most its body's η.
func OrientedInteriorWitness(a, b OrientedBox, etaA, etaB *big.Rat) bool {
	insideA, okA := prepareOrientedInside(a, etaA)
	insideB, okB := prepareOrientedInside(b, etaB)
	if !okA || !okB {
		return false
	}
	try := func(point [3]*big.Rat) bool {
		return insideA.contains(point) && insideB.contains(point)
	}
	center := func(box OrientedBox) [3]*big.Rat {
		var result [3]*big.Rat
		for k := range 3 {
			edges := proofarith.DyAdd(box.Edge[0][k], proofarith.DyAdd(box.Edge[1][k], box.Edge[2][k]))
			result[k] = proofarith.DyAdd(box.Corner[0][k], proofarith.DyShift(edges, -1)).Rat()
		}
		return result
	}
	ca, cb := center(a), center(b)
	if try(ca) || try(cb) {
		return true
	}
	for _, pair := range [][2]OrientedBox{{a, b}, {b, a}} {
		projectors, usable := prepareOrientedFaceProjectors(pair[1])
		for _, vertex := range OrientedWitnessSamples(pair[0]) {
			for index := range projectors {
				if !usable[index] {
					continue
				}
				projection, ok := projectors[index].project(vertex)
				if !ok {
					continue
				}
				var candidate [3]*big.Rat
				normSquared := projection.normSquared.Rat()
				for k := range 3 {
					foot := new(big.Rat).Quo(projection.scaled[k].Rat(), normSquared)
					candidate[k] = new(big.Rat).Quo(new(big.Rat).Add(vertex[k].Rat(), foot), big.NewRat(2, 1))
				}
				if try(candidate) {
					return true
				}
			}
		}
	}
	return false
}

func OrientedWitnessSamples(box OrientedBox) []proofarith.DyV3 {
	samples := make([]proofarith.DyV3, 0, 26)
	samples = append(samples, box.Corner[:]...)
	average := func(shift int, indices ...int) proofarith.DyV3 {
		var point proofarith.DyV3
		for axis := range 3 {
			sum := proofarith.DyZero()
			for _, index := range indices {
				sum = proofarith.DyAdd(sum, box.Corner[index][axis])
			}
			point[axis] = proofarith.DyShift(sum, shift)
		}
		return point
	}
	for axis := range 3 {
		for index := range 8 {
			if index&(1<<axis) == 0 {
				samples = append(samples, average(-1, index, index|(1<<axis)))
			}
		}
		for side := range 2 {
			indices := make([]int, 0, 4)
			for index := range 8 {
				if (index>>axis)&1 == side {
					indices = append(indices, index)
				}
			}
			samples = append(samples, average(-2, indices...))
		}
	}
	return samples
}

type orientedInsidePlane struct {
	normal [3]*big.Rat
	limit  *big.Rat
}

type orientedInsidePlanes [6]orientedInsidePlane

// prepareOrientedInside fixes each eroded face plane once for all witness
// candidates of the same box.
func prepareOrientedInside(box OrientedBox, eta *big.Rat) (orientedInsidePlanes, bool) {
	var planes orientedInsidePlanes
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		normal := proofarith.DvCross(box.Edge[i], box.Edge[j])
		if proofarith.DvDot(normal, box.Edge[axis]).Sign() < 0 {
			for k := range 3 {
				normal[k] = proofarith.DyNeg(normal[k])
			}
		}
		normUp := proofbound.RatSqrtUp(proofarith.DvDot(normal, normal).Rat())
		if !finite(normUp) || normUp <= 0 {
			return orientedInsidePlanes{}, false
		}
		margin := new(big.Rat).Mul(eta, proofarith.FloatRat(normUp))
		for side := range 2 {
			face := box.Corner[0]
			signed := normal
			if side == 1 {
				face = proofarith.DvAdd(face, box.Edge[axis])
			} else {
				for k := range 3 {
					signed[k] = proofarith.DyNeg(signed[k])
				}
			}
			plane := &planes[axis*2+side]
			for k := range 3 {
				plane.normal[k] = signed[k].Rat()
			}
			plane.limit = new(big.Rat).Sub(proofarith.DvDot(signed, face).Rat(), margin)
		}
	}
	return planes, true
}

func (planes *orientedInsidePlanes) contains(point [3]*big.Rat) bool {
	for i := range planes {
		plane := &planes[i]
		projected := new(big.Rat)
		for k := range 3 {
			projected.Add(projected, new(big.Rat).Mul(plane.normal[k], point[k]))
		}
		if projected.Cmp(plane.limit) >= 0 {
			return false
		}
	}
	return true
}

// OrientedFace is a face's origin and two spanning edges in exact coordinates.
type OrientedFace struct {
	Origin, U, V proofarith.DyV3
}

func OrientedAxisFace(box *OrientedBox, axis, side int, face *OrientedFace) bool {
	for edgeAxis := range box.Edge {
		edge := &box.Edge[edgeAxis]
		if edge[axis].Sign() == 0 {
			continue
		}
		for other := range 3 {
			if other != axis && edge[other].Sign() != 0 {
				return false
			}
		}
		others := [2]int{}
		count := 0
		for i := range 3 {
			if i != edgeAxis {
				if box.Edge[i][axis].Sign() != 0 {
					return false
				}
				others[count] = i
				count++
			}
		}
		shift := (side == 1 && edge[axis].Sign() > 0) ||
			(side == 0 && edge[axis].Sign() < 0)
		for coordinate := range 3 {
			face.Origin[coordinate] = box.Corner[0][coordinate]
			if shift {
				face.Origin[coordinate] = proofarith.DyAdd(face.Origin[coordinate], edge[coordinate])
			}
			face.U[coordinate] = box.Edge[others[0]][coordinate]
			face.V[coordinate] = box.Edge[others[1]][coordinate]
		}
		return true
	}
	return false
}

func OrientedFaceCenter(face *OrientedFace, point *proofarith.DyV3) {
	for i := range 3 {
		point[i] = proofarith.DyAdd(face.Origin[i], proofarith.DyShift(proofarith.DyAdd(face.U[i], face.V[i]), -1))
	}
}

func OrientedFaceContainsProjection(face *OrientedFace, point *proofarith.DyV3, axis int) bool {
	i, j := (axis+1)%3, (axis+2)%3
	det := proofarith.DySubScalar(proofarith.DyMul(face.U[i], face.V[j]), proofarith.DyMul(face.U[j], face.V[i]))
	if det.Sign() == 0 {
		return false
	}
	pi, pj := proofarith.DySubScalar(point[i], face.Origin[i]), proofarith.DySubScalar(point[j], face.Origin[j])
	u := proofarith.DySubScalar(proofarith.DyMul(pi, face.V[j]), proofarith.DyMul(pj, face.V[i]))
	v := proofarith.DySubScalar(proofarith.DyMul(face.U[i], pj), proofarith.DyMul(face.U[j], pi))
	if det.Sign() < 0 {
		det, u, v = proofarith.DyNeg(det), proofarith.DyNeg(u), proofarith.DyNeg(v)
	}
	return u.Sign() > 0 && v.Sign() > 0 && proofarith.DyCmp(u, det) < 0 && proofarith.DyCmp(v, det) < 0
}
