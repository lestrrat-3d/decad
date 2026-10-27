package pointquery

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type ratVec [3]*big.Rat

func exactVec(v r3.Vec) ratVec {
	return ratVec{proof.FloatRat(v.X), proof.FloatRat(v.Y), proof.FloatRat(v.Z)}
}

func subtract(a, b ratVec) ratVec {
	return ratVec{
		new(big.Rat).Sub(a[0], b[0]),
		new(big.Rat).Sub(a[1], b[1]),
		new(big.Rat).Sub(a[2], b[2]),
	}
}

func dot(a, b ratVec) *big.Rat {
	out := new(big.Rat)
	for i := range a {
		out.Add(out, new(big.Rat).Mul(a[i], b[i]))
	}
	return out
}

func cross(a, b ratVec) ratVec {
	return ratVec{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

// dyFraction is the exact value num/den, den positive.
type dyFraction struct{ num, den proof.Dyadic }

// less reports whether f < g, by cross-multiplying the positive denominators.
func (f dyFraction) less(g dyFraction) bool {
	return proof.DyCmp(proof.DyMul(f.num, g.den), proof.DyMul(g.num, f.den)) < 0
}

// rat is the fraction as a reduced big.Rat, the one normalisation a triangle
// distance pays.
func (f dyFraction) rat() *big.Rat {
	return new(big.Rat).Quo(f.num.Rat(), f.den.Rat())
}

var dyOne = proof.MustDyOf(1)

// segmentDistanceSquared is the exact squared distance from p to the closed
// segment ab, as a fraction of dyadics.
func segmentDistanceSquared(p, a, b proof.DyV3) dyFraction {
	axis := proof.DvSub(b, a)
	lengthSquared := proof.DvDot(axis, axis)
	fromStart := proof.DvSub(p, a)
	startSquared := proof.DvDot(fromStart, fromStart)
	if lengthSquared.IsZero() {
		return dyFraction{startSquared, dyOne}
	}
	along := proof.DvDot(fromStart, axis)
	if along.Sign() <= 0 {
		return dyFraction{startSquared, dyOne}
	}
	if proof.DyCmp(along, lengthSquared) >= 0 {
		fromEnd := proof.DvSub(p, b)
		return dyFraction{proof.DvDot(fromEnd, fromEnd), dyOne}
	}
	// |p−a|² − along²/|b−a|², over the positive |b−a|².
	num := proof.DySubScalar(proof.DyMul(startSquared, lengthSquared), proof.DyMul(along, along))
	return dyFraction{num, lengthSquared}
}

// triangleNormal is the exact normal (b−a)×(c−a).
func triangleNormal(a, b, c proof.DyV3) proof.DyV3 {
	return proof.DvCross(proof.DvSub(b, a), proof.DvSub(c, a))
}

// triangleDistanceSquared is the exact squared distance from p to the closed
// triangle abc. Every step is exact dyadic arithmetic; the result divides one
// dyadic numerator by one positive dyadic denominator. A zero normal is an
// error.
func triangleDistanceSquared(p, a, b, c proof.DyV3) (*big.Rat, error) {
	normal := triangleNormal(a, b, c)
	if proof.DvIsZero(normal) {
		return nil, fmt.Errorf(`%w: point query mesh contains a degenerate triangle`, decaderr.ErrUnsupported)
	}
	triangle := [3]proof.DyV3{a, b, c}
	projectionInside := true
	for i := range triangle {
		start, end := triangle[i], triangle[(i+1)%3]
		side := proof.DvDot(proof.DvCross(proof.DvSub(end, start), proof.DvSub(p, start)), normal)
		if side.Sign() < 0 {
			projectionInside = false
			break
		}
	}
	if projectionInside {
		offset := proof.DvDot(proof.DvSub(p, a), normal)
		return dyFraction{proof.DyMul(offset, offset), proof.DvDot(normal, normal)}.rat(), nil
	}
	minimum := segmentDistanceSquared(p, a, b)
	for i := 1; i < 3; i++ {
		if distance := segmentDistanceSquared(p, triangle[i], triangle[(i+1)%3]); distance.less(minimum) {
			minimum = distance
		}
	}
	return minimum.rat(), nil
}

// boxDistanceSquared is a lower bound used only to skip a triangle whose
// closed axis-aligned box cannot beat the current exact minimum.
func boxDistanceSquared(p, a, b, c ratVec) *big.Rat {
	result := new(big.Rat)
	for i := range p {
		lo, hi := a[i], a[i]
		for _, v := range [2]*big.Rat{b[i], c[i]} {
			if v.Cmp(lo) < 0 {
				lo = v
			}
			if v.Cmp(hi) > 0 {
				hi = v
			}
		}
		gap := new(big.Rat)
		switch {
		case p[i].Cmp(lo) < 0:
			gap.Sub(lo, p[i])
		case p[i].Cmp(hi) > 0:
			gap.Sub(p[i], hi)
		}
		result.Add(result, new(big.Rat).Mul(gap, gap))
	}
	return result
}

// boxDistanceSquaredLower rounds every positive gap, square and partial sum
// downward. It bounds the exact box distance even when an operation
// overflows: NextDown then gives MaxFloat64, below the exact result.
// Comparing this lower bound with an upward-rounded minimum can only skip
// triangles the exact box test would skip. Unresolved cases keep that test.
func boxDistanceSquaredLower(p, a, b, c r3.Vec) float64 {
	var result float64
	for _, axis := range [3][4]float64{
		{p.X, a.X, b.X, c.X},
		{p.Y, a.Y, b.Y, c.Y},
		{p.Z, a.Z, b.Z, c.Z},
	} {
		lo, hi := min(axis[1], axis[2], axis[3]), max(axis[1], axis[2], axis[3])
		var gap float64
		switch {
		case axis[0] < lo:
			gap = proof.NextDown(lo - axis[0])
		case axis[0] > hi:
			gap = proof.NextDown(axis[0] - hi)
		default:
			continue
		}
		gap = max(0, gap)
		squared := max(0, proof.NextDown(float64(gap*gap)))
		result = max(0, proof.NextDown(result+squared))
	}
	return result
}

// checkFiniteVertices spends one budget step per vertex, in order, and
// rejects the first non-finite vertex.
func checkFiniteVertices(budget *proofbound.WorkBudget, verts []r3.Vec) error {
	for _, v := range verts {
		if err := budget.Step(); err != nil {
			return err
		}
		if !proofbound.FiniteVec(v) {
			return fmt.Errorf(`%w: point query mesh has a non-finite vertex`, decaderr.ErrUnsupported)
		}
	}
	return nil
}

// lazyVertices converts a vertex to its exact rational form the first time a
// triangle needs it. The float filter rejects most triangles before then.
type lazyVertices struct {
	verts []r3.Vec
	exact []ratVec
}

func newLazyVertices(verts []r3.Vec) *lazyVertices {
	return &lazyVertices{verts: verts, exact: make([]ratVec, len(verts))}
}

// at returns vertex i, which must be in range and finite, as exact rationals.
func (l *lazyVertices) at(i int) ratVec {
	if l.exact[i][0] == nil {
		l.exact[i] = exactVec(l.verts[i])
	}
	return l.exact[i]
}

// normalNonzeroFloat reports whether a float evaluation proves the exact
// normal (b−a)×(c−a) nonzero. False leaves the triangle undecided, never
// degenerate. Every nonzero difference must lie in [2^-300, 2^250]; a zero
// float difference of finite floats is exact. Each component's float value
// then carries at most three roundings on each product and one on their
// difference, so its error is below 4.001*2^-53*(|l|+|r|), less than half
// the 1e-15*(|l|+|r|) the test requires it to exceed. Products of admitted
// differences are normal or exact zero, and a nonzero sum |l|+|r| is at
// least 2^-600, so the bound itself stays normal.
func normalNonzeroFloat(a, b, c r3.Vec) bool {
	u := [3]float64{b.X - a.X, b.Y - a.Y, b.Z - a.Z}
	v := [3]float64{c.X - a.X, c.Y - a.Y, c.Z - a.Z}
	for _, x := range [6]float64{u[0], u[1], u[2], v[0], v[1], v[2]} {
		x = math.Abs(x)
		if x != 0 && (x < 0x1p-300 || x > 0x1p250) {
			return false
		}
	}
	for k := range 3 {
		i, j := (k+1)%3, (k+2)%3
		left, right := u[i]*v[j], u[j]*v[i]
		if math.Abs(left-right) > 1e-15*(math.Abs(left)+math.Abs(right)) {
			return true
		}
	}
	return false
}

// degenerateTriangle reports whether the exact normal of finite vertices is
// zero, the test triangleDistanceSquared rejects.
func degenerateTriangle(a, b, c r3.Vec) bool {
	return proof.DvIsZero(triangleNormal(proof.DyVec(a), proof.DyVec(b), proof.DyVec(c)))
}

// meshCandidate is one triangle and a downward-rounded lower bound on its
// exact squared distance.
type meshCandidate struct {
	lower float64
	index int
}

// MeshDistanceSquared returns the exact squared distance from p to the held
// triangle set, which may be a selected face patch. p and every vertex are
// finite float64 values interpreted as exact dyadics; a non-finite p panics.
// The caller must provide a verified embedded mesh or a subset of its
// triangles.
//
// The result is the exact minimum of triangleDistanceSquared over every
// triangle. A degenerate triangle makes that distance an error, so the result
// then depends on whether the index-order scan in scanMeshInOrder reaches it.
// The first pass spends the budget steps, checks indices and computes each
// triangle's downward-rounded float box bound in index order. When it proves
// every triangle non-degenerate, no visiting order can error, and the exact
// distances are visited nearest box bound first. The triangle with the
// smallest bound seeds the exact minimum. Only triangles whose bound lies
// below the seed's upward-rounded minimum are kept, sorted by bound, and the
// first bound at or above the current upward-rounded minimum ends the visit.
// A skipped triangle's exact distance is at least its bound, so it cannot
// lower the minimum: a float bound only ever skips a triangle, and the
// minimum is always exact. The first exactly degenerate triangle hands the
// rest of the work to scanMeshInOrder, which keeps the index-order result and
// error. The visit after the first pass spends no budget steps, so every step
// and every error arrives in the same order as the index-order scan.
func MeshDistanceSquared(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int) (*big.Rat, error) {
	if len(tris) == 0 {
		return nil, fmt.Errorf(`%w: point query mesh has no triangles`, decaderr.ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := checkFiniteVertices(budget, verts); err != nil {
		return nil, err
	}
	lower := make([]float64, len(tris))
	seed := 0
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if err := checkTriangleIndices(tri, len(verts)); err != nil {
			return nil, err
		}
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		if !normalNonzeroFloat(a, b, c) && degenerateTriangle(a, b, c) {
			return scanMeshInOrder(budget, p, verts, tris, i)
		}
		lower[i] = boxDistanceSquaredLower(p, a, b, c)
		if lower[i] < lower[seed] {
			seed = i
		}
	}
	point := proof.DyVec(p)
	distanceTo := func(tri [3]int) (*big.Rat, error) {
		return triangleDistanceSquared(point, proof.DyVec(verts[tri[0]]), proof.DyVec(verts[tri[1]]), proof.DyVec(verts[tri[2]]))
	}
	minimum, err := distanceTo(tris[seed])
	if err != nil {
		return nil, err
	}
	minimumUpper := proofbound.RatFloatUp(minimum)
	var candidates []meshCandidate
	for i, bound := range lower {
		if i != seed && bound < minimumUpper {
			candidates = append(candidates, meshCandidate{lower: bound, index: i})
		}
	}
	slices.SortFunc(candidates, func(x, y meshCandidate) int {
		return cmp.Or(cmp.Compare(x.lower, y.lower), cmp.Compare(x.index, y.index))
	})
	for _, candidate := range candidates {
		if candidate.lower >= minimumUpper {
			break
		}
		distance, err := distanceTo(tris[candidate.index])
		if err != nil {
			return nil, err
		}
		if distance.Cmp(minimum) < 0 {
			minimum = distance
			minimumUpper = proofbound.RatFloatUp(minimum)
		}
	}
	return minimum, nil
}

func checkTriangleIndices(tri [3]int, count int) error {
	for _, index := range tri {
		if index < 0 || index >= count {
			return fmt.Errorf(`%w: point query mesh has an invalid triangle index`, decaderr.ErrUnsupported)
		}
	}
	return nil
}

// scanMeshInOrder visits the triangles in index order. Triangles up to and
// including checked have already spent their budget step and passed the
// index check; each later one spends its step and is checked before its
// distance. A triangle is skipped only when its float or exact box distance
// is at least the current exact minimum, so the first degenerate triangle
// the scan reaches is the error.
func scanMeshInOrder(budget *proofbound.WorkBudget, p r3.Vec, verts []r3.Vec, tris [][3]int, checked int) (*big.Rat, error) {
	point, dyPoint := exactVec(p), proof.DyVec(p)
	exact := newLazyVertices(verts)
	var minimum *big.Rat
	var minimumUpper float64
	for i, tri := range tris {
		if i > checked {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if err := checkTriangleIndices(tri, len(verts)); err != nil {
				return nil, err
			}
		}
		if minimum != nil && boxDistanceSquaredLower(p, verts[tri[0]], verts[tri[1]], verts[tri[2]]) >= minimumUpper {
			continue
		}
		if minimum != nil && boxDistanceSquared(point, exact.at(tri[0]), exact.at(tri[1]), exact.at(tri[2])).Cmp(minimum) >= 0 {
			continue
		}
		distance, err := triangleDistanceSquared(dyPoint, proof.DyVec(verts[tri[0]]), proof.DyVec(verts[tri[1]]), proof.DyVec(verts[tri[2]]))
		if err != nil {
			return nil, err
		}
		if minimum == nil || distance.Cmp(minimum) < 0 {
			minimum = distance
			minimumUpper = proofbound.RatFloatUp(minimum)
		}
	}
	return minimum, nil
}
