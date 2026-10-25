// Package coneaxis holds the exact axis vector of a Revolve about a line
// through the world origin, the two predicates that select the +X branch, and
// the exact A-scaled coordinates every predicate of the axis-general sector
// class is written in (agent-docs/revolve-axis-design.md §3).
//
// The axis A is a world vector with float components, a positive multiple of
// the axis direction. It need not be unit length and is never normalized. Every
// reading here is an exact Dyadic or big.Rat of the stored floats.
package coneaxis

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/r3"
)

// Axis is the exact axis vector. The zero Axis names no axis; build one with
// New, X or OfRevolve. Axis values compare with ==, which is the bitwise
// comparison of §3.3.
type Axis struct{ v r3.Vec }

// X returns the canonical world +X axis.
func X() Axis { return Axis{v: r3.Vec{X: 1}} }

// New returns the axis with vector v. It reports false for a non-finite or
// zero vector.
func New(v r3.Vec) (Axis, bool) {
	if !proofbound.FiniteVec(v) || v == (r3.Vec{}) {
		return Axis{}, false
	}
	return Axis{v: v}, true
}

// Vec returns the axis vector.
func (a Axis) Vec() r3.Vec { return a.v }

// Canonical is the axis predicate: it reports whether A equals exactly
// r3.Vec{X: 1}, the case every canonical arithmetic branch is written for.
func (a Axis) Canonical() bool { return a.v == (r3.Vec{X: 1}) }

// FrameCanonical is the frame predicate of §3.3. It reports whether a Revolve
// record's frame origin is 0, its U is X and its V is Y, and its axis anchor
// is (0, 0) and direction (1, 0), all four proven bounds zero. It is the gate
// the +X record-reading code applies to a record today.
func FrameCanonical(frame r3.Frame, ax revolveaxis.Frame) bool {
	return frame.Origin() == (r3.Vec{}) && frame.U() == (r3.Vec{X: 1}) && frame.V() == (r3.Vec{Y: 1}) &&
		ax.AU == 0 && ax.AV == 0 && ax.DU == 1 && ax.DV == 0 &&
		ax.AUBound == 0 && ax.AVBound == 0 && ax.DUBound == 0 && ax.DVBound == 0
}

// OfRevolve builds the axis value of a Revolve record from its frame, resolved
// axis and recorded AxisVec (revolveplan.Payload.AxisVec). A record that
// passes the frame predicate gets the constant +X axis and its axisVec is
// never read. Any other record gets axisVec, and it reports false unless the
// record satisfies §3.6: the frame origin is the world origin, U and V are
// distinct signed world basis vectors, the axis anchor is the plane origin
// with zero bounds, and axisVec is a finite nonzero vector in the plane.
func OfRevolve(frame r3.Frame, ax revolveaxis.Frame, axisVec r3.Vec) (Axis, bool) {
	if FrameCanonical(frame, ax) {
		return X(), true
	}
	if frame.Origin() != (r3.Vec{}) || ax.AU != 0 || ax.AV != 0 || ax.AUBound != 0 || ax.AVBound != 0 {
		return Axis{}, false
	}
	iu, ok := signedBasis(frame.U())
	if !ok {
		return Axis{}, false
	}
	iv, ok := signedBasis(frame.V())
	if !ok || iu == iv {
		return Axis{}, false
	}
	if component(axisVec, 3-iu-iv) != 0 {
		return Axis{}, false
	}
	return New(axisVec)
}

// signedBasis reports the coordinate index of v when v is exactly a signed
// world basis vector.
func signedBasis(v r3.Vec) (int, bool) {
	c := [3]float64{v.X, v.Y, v.Z}
	index := -1
	for i, x := range c {
		switch {
		case x == 0:
		case (x == 1 || x == -1) && index < 0:
			index = i
		default:
			return 0, false
		}
	}
	return index, index >= 0
}

func component(v r3.Vec, i int) float64 { return [3]float64{v.X, v.Y, v.Z}[i] }

func dyVec(v r3.Vec) [3]proofarith.Dyadic {
	return [3]proofarith.Dyadic{proofarith.MustDyOf(v.X), proofarith.MustDyOf(v.Y), proofarith.MustDyOf(v.Z)}
}

// Axial returns s(p) = p·A exactly. p must be finite.
func (a Axis) Axial(p r3.Vec) proofarith.Dyadic {
	dp, da := dyVec(p), dyVec(a.v)
	return proofarith.DyAdd(proofarith.DyAdd(
		proofarith.DyMul(dp[0], da[0]), proofarith.DyMul(dp[1], da[1])), proofarith.DyMul(dp[2], da[2]))
}

// RadialSq returns σ(p)² = |p×A|² exactly. p must be finite.
func (a Axis) RadialSq(p r3.Vec) proofarith.Dyadic {
	dp, da := dyVec(p), dyVec(a.v)
	cross := func(i, j proofarith.Dyadic, k, l proofarith.Dyadic) proofarith.Dyadic {
		return proofarith.DySubScalar(proofarith.DyMul(i, j), proofarith.DyMul(k, l))
	}
	cx := cross(dp[1], da[2], dp[2], da[1])
	cy := cross(dp[2], da[0], dp[0], da[2])
	cz := cross(dp[0], da[1], dp[1], da[0])
	return proofarith.DyAdd(proofarith.DyAdd(proofarith.DyMul(cx, cx), proofarith.DyMul(cy, cy)),
		proofarith.DyMul(cz, cz))
}

// NormSq returns A·A exactly.
func (a Axis) NormSq() proofarith.Dyadic {
	da := dyVec(a.v)
	return proofarith.DyAdd(proofarith.DyAdd(
		proofarith.DyMul(da[0], da[0]), proofarith.DyMul(da[1], da[1])), proofarith.DyMul(da[2], da[2]))
}

// NormBracket returns floats lo ≤ |A| ≤ hi, each proven by an exact
// comparison against A·A. A bound that divides a scaled value by |A| takes the
// side that widens it (§3.4).
func (a Axis) NormBracket() (lo, hi float64) {
	q := a.NormSq().Rat()
	return proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
}

// RadialLower returns a float no larger than σ(q) for every point q of the
// triangle (§3.5), and reports false when A lies outside every coordinate
// plane. A canonical axis keeps the current rule, the larger of the smallest
// |y| and the smallest |z| over the three vertices.
func (a Axis) RadialLower(tri [3]r3.Vec) (float64, bool) {
	return a.RadialLowerOf(tri[:])
}

// RadialLowerOf is RadialLower over the convex hull of a non-empty point set.
// Both functionals of §3.5 are linear, so their ranges over the hull come from
// the points themselves.
func (a Axis) RadialLowerOf(points []r3.Vec) (float64, bool) {
	if a.Canonical() {
		minAbs := func(lo, hi float64) float64 {
			if lo <= 0 && hi >= 0 {
				return 0
			}
			return min(math.Abs(lo), math.Abs(hi))
		}
		yLo, yHi, zLo, zHi := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
		for _, p := range points {
			yLo, yHi = min(yLo, p.Y), max(yHi, p.Y)
			zLo, zHi = min(zLo, p.Z), max(zHi, p.Z)
		}
		return max(minAbs(yLo, yHi), minAbs(zLo, zHi)), true
	}
	n := -1
	for i := range 3 {
		if component(a.v, i) == 0 {
			n = i
			break
		}
	}
	if n < 0 {
		return 0, false
	}
	// N_A = A × e_n lies in the plane with normal e_n, orthogonal to A and e_n
	// (§3.5), so σ² = (p·N_A)² + p_n²·(A·A) with both terms exact.
	en := r3.Vec{}
	switch n {
	case 0:
		en.X = 1
	case 1:
		en.Y = 1
	default:
		en.Z = 1
	}
	na := a.v.Cross(en)
	inPlane, normal := ratRange(), ratRange()
	dna := dyVec(na)
	for _, p := range points {
		dp := dyVec(p)
		d := proofarith.DyAdd(proofarith.DyAdd(
			proofarith.DyMul(dp[0], dna[0]), proofarith.DyMul(dp[1], dna[1])), proofarith.DyMul(dp[2], dna[2]))
		inPlane.add(d.Rat())
		normal.add(proofarith.FloatRat(component(p, n)))
	}
	sum := new(big.Rat).Mul(inPlane.minAbs(), inPlane.minAbs())
	nm := normal.minAbs()
	sum.Add(sum, new(big.Rat).Mul(a.NormSq().Rat(), new(big.Rat).Mul(nm, nm)))
	return proofbound.RatSqrtDown(sum), true
}

// TriangleBeyond reports whether every point of the closed triangle lies
// farther than distance millimetres from the axis line. The test is exact:
// q = p×A maps the triangle linearly onto a triangle in the plane normal to
// A with |q| = σ(p), so the triangle clears the axis exactly when the image
// misses the origin and each image edge keeps |q|² above distance²·(A·A).
// It reports false for a non-finite vertex or distance.
func (a Axis) TriangleBeyond(tri [3]r3.Vec, distance float64) bool {
	if !proofbound.FiniteVec(a.v) || proofbound.IsNonFinite(distance) || distance < 0 {
		return false
	}
	for _, p := range tri {
		if !proofbound.FiniteVec(p) {
			return false
		}
	}
	da := proofarith.DyVec(a.v)
	var q [3]proofarith.DyV3
	for i, p := range tri {
		q[i] = proofarith.DvCross(proofarith.DyVec(p), da)
	}
	// The origin lies inside the image when it is strictly on one side of
	// every directed edge, read about A.
	var signs [3]int
	for i := range 3 {
		signs[i] = proofarith.DvDot(proofarith.DvCross(q[i], q[(i+1)%3]), da).Sign()
	}
	if (signs[0] > 0 && signs[1] > 0 && signs[2] > 0) || (signs[0] < 0 && signs[1] < 0 && signs[2] < 0) {
		return false
	}
	dd := proofarith.MustDyOf(distance)
	limit := proofarith.DyMul(proofarith.DyMul(dd, dd), a.NormSq())
	beyond := func(v proofarith.DyV3) bool { return proofarith.DyCmp(proofarith.DvDot(v, v), limit) > 0 }
	for i := range 3 {
		from, to := q[i], q[(i+1)%3]
		edge := proofarith.DvSub(to, from)
		along := proofarith.DvDot(edge, edge)
		start := proofarith.DvDot(from, edge)
		switch {
		case along.IsZero() || start.Sign() >= 0:
			if !beyond(from) {
				return false
			}
		case proofarith.DvDot(to, edge).Sign() <= 0:
			if !beyond(to) {
				return false
			}
		default:
			// The nearest point is interior: |from|² − (from·edge)²/(edge·edge),
			// compared with the limit after multiplying through by edge·edge > 0.
			gap := proofarith.DySubScalar(proofarith.DyMul(proofarith.DvDot(from, from), along),
				proofarith.DyMul(start, start))
			if proofarith.DyCmp(gap, proofarith.DyMul(limit, along)) <= 0 {
				return false
			}
		}
	}
	return true
}

// ScaledDown converts an A-scaled linear quantity q to millimetres as a lower
// bound (§3.4): it returns a float no larger than q/|A|, dividing a
// nonnegative q by |A|↑ and a negative q by |A|↓. It reports false when the
// norm bracket or the result is not finite and positive.
func (a Axis) ScaledDown(q *big.Rat) (float64, bool) {
	lo, hi := a.NormBracket()
	div := hi
	if q.Sign() < 0 {
		div = lo
	}
	return scaled(q, div, lo, hi, proofbound.RatFloatDown)
}

// ScaledUp is ScaledDown's upper bound: a float no smaller than q/|A|,
// dividing a nonnegative q by |A|↓ and a negative q by |A|↑.
func (a Axis) ScaledUp(q *big.Rat) (float64, bool) {
	lo, hi := a.NormBracket()
	div := lo
	if q.Sign() < 0 {
		div = hi
	}
	return scaled(q, div, lo, hi, proofbound.RatFloatUp)
}

func scaled(q *big.Rat, div, lo, hi float64, round func(*big.Rat) float64) (float64, bool) {
	if !(lo > 0) || proofbound.IsNonFinite(hi) {
		return 0, false
	}
	out := round(new(big.Rat).Quo(q, proofarith.FloatRat(div)))
	if proofbound.IsNonFinite(out) {
		return 0, false
	}
	return out, true
}

type ratSpan struct{ lo, hi *big.Rat }

func ratRange() *ratSpan { return &ratSpan{} }

func (s *ratSpan) add(v *big.Rat) {
	if s.lo == nil || v.Cmp(s.lo) < 0 {
		s.lo = v
	}
	if s.hi == nil || v.Cmp(s.hi) > 0 {
		s.hi = v
	}
}

// minAbs returns the smallest |x| over the span.
func (s *ratSpan) minAbs() *big.Rat {
	if s.lo.Sign() <= 0 && s.hi.Sign() >= 0 {
		return new(big.Rat)
	}
	return new(big.Rat).Abs(minRat(new(big.Rat).Abs(s.lo), new(big.Rat).Abs(s.hi)))
}

func minRat(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

// PlaneScaled returns the exact plane-scaled coordinates of the plane-local
// point (u, v): z = p·A_plane and r = cross2D(A_plane, p), where A_plane is A
// read in the frame's U and V axes (§4.1). The frame must satisfy OfRevolve's
// §3.6 conditions for A, which makes A_plane an exact pair. For the canonical
// frame it returns (u, v).
func (a Axis) PlaneScaled(frame r3.Frame, u, v float64) (z, r proofarith.Dyadic) {
	au := proofarith.MustDyOf(a.v.Dot(frame.U()))
	av := proofarith.MustDyOf(a.v.Dot(frame.V()))
	du, dv := proofarith.MustDyOf(u), proofarith.MustDyOf(v)
	z = proofarith.DyAdd(proofarith.DyMul(du, au), proofarith.DyMul(dv, av))
	r = proofarith.DySubScalar(proofarith.DyMul(au, dv), proofarith.DyMul(av, du))
	return z, r
}

// RecordedVec derives the AxisVec a Revolve records (§3.6). It is the zero
// vector, which refuses the general class, unless all of the following hold:
//
//   - the frame origin is the world origin;
//   - U and V are distinct signed world basis vectors, so the map from plane
//     coordinates to world is an exact signed permutation;
//   - the resolved axis anchor is the plane origin, with zero bounds;
//   - the plane-local direction is a stored float pair read without
//     subtraction: a SketchLine must start at (0, 0), and its direction is the
//     end point; a ConstructionAxis must have the world origin as its origin,
//     and its direction is (Dir·U, Dir·V), exact for a signed basis.
//
// The result is side·(du·U + dv·V), where side is the sign ResolveSide applied
// to the direction. Negation and a signed permutation are exact. A component
// that would be -0 is returned as +0, so two records of one axis compare
// bitwise equal.
func RecordedVec(frame r3.Frame, input revolveaxis.AxisInput, ax revolveaxis.Frame, side float64) r3.Vec {
	if (side != 1 && side != -1) || frame.Origin() != (r3.Vec{}) ||
		ax.AU != 0 || ax.AV != 0 || ax.AUBound != 0 || ax.AVBound != 0 {
		return r3.Vec{}
	}
	iu, ok := signedBasis(frame.U())
	if !ok {
		return r3.Vec{}
	}
	iv, ok := signedBasis(frame.V())
	if !ok || iu == iv {
		return r3.Vec{}
	}
	var du, dv float64
	switch in := input.(type) {
	case revolveaxis.SketchLine:
		if in.StartU != 0 || in.StartV != 0 {
			return r3.Vec{}
		}
		du, dv = in.EndU, in.EndV
	case revolveaxis.ConstructionAxis:
		if in.Origin != (r3.Vec{}) {
			return r3.Vec{}
		}
		du, dv = in.Dir.Dot(frame.U()), in.Dir.Dot(frame.V())
	default:
		return r3.Vec{}
	}
	if (du == 0 && dv == 0) || !proofbound.FiniteVec(r3.Vec{X: du, Y: dv}) {
		return r3.Vec{}
	}
	v := frame.ToWorldUV(du, dv).Scale(side)
	return r3.Vec{X: v.X + 0, Y: v.Y + 0, Z: v.Z + 0}
}
