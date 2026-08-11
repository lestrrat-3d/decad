package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/loft-design.md §8's mass-property engine: the exact
// rational tetrahedron-sum accumulator over the assembled wall-and-cap
// triangle set T. Volume and Centroid are polynomial in the vertex
// coordinates, so — following moments.go's own anchor-then-publish-once
// discipline — every vertex coordinate is a float, taken exactly as a
// math/big.Rat, accumulated into one rational sum anchored at the first
// profile's own plane origin, and rounded to float64 exactly ONCE at
// publication. Area has no such closed form (a triangle's own area is a
// square root of a rational, generically irrational) and is never Exact,
// for the same reason spline design §3 gives arc length. It is still PROVEN:
// each triangle's area is bracketed from its own exact rational cross-norm by
// spline_length.go's outward-rounded ratSqrtDown/ratSqrtUp, and the published
// bound sums those per-triangle widths beside the summation loop's own slop —
// or is +Inf where either of those two terms has itself saturated, since a
// saturated term states no scale for the bound to be proven at (wallBound).
//
// A PLACEMENT adds one further term to all four readings: the payload's own
// proven displacement delta and the allowances it feeds (§12 PR 2a), which is
// why none of the four is Exact on a placed body however exactly its own
// arithmetic comes out. Every one of those terms is gated on delta > 0, so an
// unplaced body's published readings are bit-identical to PR 1's.

// loftMassAccumulator is docs/loft-design.md §8's tetrahedron-sum kernel. It
// streams one outward-oriented triangle of T at a time — a wall or a cap
// triangulation triangle — anchored at anchor (p0's own PlaneRecord.Origin).
// Volume, Centroid and Bounds fold every triangle handed to add; Area's wall
// contribution folds only the triangles whose wall flag is true, since a
// cap's own contribution is its 2-D region's exact rational area
// (moments.go), never the sum of its triangulation's own float areas.
type loftMassAccumulator struct {
	anchor  xpt
	anchorF r3.Vec

	// delta is the placement's own proven displacement of every held vertex
	// from the exact placed image of the recorded sections (loft_build.go's
	// loftPayload.delta, docs/loft-design.md §5/§12 PR 2a) — zero for an
	// unplaced body. Every extra term below is gated on delta > 0, so an
	// unplaced loft's published measurements stay bit-identical to PR 1's.
	delta float64

	// vol6 is Σ (A-anchor)·((B-anchor)×(C-anchor)) over every triangle of T:
	// six times the signed volume (docs/loft-design.md §8).
	vol6 *big.Rat
	// momX/momY/momZ are Σ vol6_tri · Σ(A-anchor, B-anchor, C-anchor), the
	// first-moment accumulator the centroid divides by 4·vol6.
	momX, momY, momZ *big.Rat

	haveBounds bool
	lo, hi     r3.Vec // componentwise extremes over every held vertex

	// coordUpper is a proven upper bound on |u|, |v| and |z| over the body's
	// own material relative to anchor: the max |v-anchor|_inf over every held
	// vertex (bounds.go's sweptMomentAllow reads it widened by delta at the
	// point of use, since the true vertex may sit up to delta further out).
	coordUpper float64

	// perturbAreaSum is Σ perturbedTriangleAreaAllow(...) over EVERY triangle
	// of T — walls and caps alike — the extra area a placement's delta can
	// add on top of the held triangle areas area() already sums. It stays
	// exactly 0 when delta is 0.
	perturbAreaSum float64

	// wallAreaSum is the naive float sum of the per-triangle PROVEN LOWER
	// bounds wallTriangleArea returns; wallAreaAbs is an upper bound on
	// Σ|term|, the scale sumSlop's summation proof reads; wallAreaSlack is an
	// upper bound on Σ (upper − lower), the enclosure width each triangle's
	// own area contributes. The three are what make the published bound a
	// proof rather than an estimate.
	//
	// Each of the two proof terms is an UPPER bound and is nudged outward once
	// per term, so each diverges upward from the value it speaks for and can
	// SATURATE at +Inf while wallAreaSum is still finite. A saturated term has
	// stopped being a proven scale, and area() publishes an infinite bound
	// rather than the zero sumSlop reports for a non-finite absSum.
	wallAreaSum   float64
	wallAreaAbs   float64
	wallAreaSlack float64
	wallTerms     int // term count sumSlop needs to bound that sum
}

// newLoftMassAccumulator opens a fresh accumulator anchored at the loft's
// first profile plane origin (docs/loft-design.md §8), carrying the
// payload's own proven placement displacement delta (§12 PR 2a) — zero for
// an unplaced body.
func newLoftMassAccumulator(anchor r3.Vec, delta float64) *loftMassAccumulator {
	return &loftMassAccumulator{
		anchor:  xptOf(anchor),
		anchorF: anchor,
		delta:   delta,
		vol6:    new(big.Rat),
		momX:    new(big.Rat),
		momY:    new(big.Rat),
		momZ:    new(big.Rat),
	}
}

// add folds one outward-oriented triangle (A, B, C) of T into the volume,
// centroid and bounds accumulators, and — when wall is true — into the area
// accumulator's float sum and that sum's two proof terms. Every vertex
// coordinate is a float64, hence an exact rational (clearance_poly.go's
// take-the-floats-exactly discipline); the volume and centroid sums round
// nothing until publication, and the area sum's own terms are the endpoints
// of a proven per-triangle enclosure rather than a float evaluation.
func (m *loftMassAccumulator) add(a, b, c r3.Vec, wall bool) {
	sa := xsub(xptOf(a), m.anchor)
	sb := xsub(xptOf(b), m.anchor)
	sc := xsub(xptOf(c), m.anchor)

	triVol6 := xdotRat(sa, xcross(sb, sc))
	m.vol6.Add(m.vol6, triVol6)

	saX, saY, saZ := xhpRat(xhp(sa))
	sbX, sbY, sbZ := xhpRat(xhp(sb))
	scX, scY, scZ := xhpRat(xhp(sc))
	sumX := ratAdd(saX, sbX, scX)
	sumY := ratAdd(saY, sbY, scY)
	sumZ := ratAdd(saZ, sbZ, scZ)
	m.momX.Add(m.momX, new(big.Rat).Mul(triVol6, sumX))
	m.momY.Add(m.momY, new(big.Rat).Mul(triVol6, sumY))
	m.momZ.Add(m.momZ, new(big.Rat).Mul(triVol6, sumZ))

	m.foldBounds(a)
	m.foldBounds(b)
	m.foldBounds(c)
	m.foldCoordUpper(a)
	m.foldCoordUpper(b)
	m.foldCoordUpper(c)

	if m.delta > 0 {
		m.perturbAreaSum = upRound(m.perturbAreaSum + perturbedTriangleAreaAllow(a, b, c, m.delta))
	}

	if !wall {
		return
	}
	// sb-sa and sc-sa are b-a and c-a exactly: the anchor cancels over
	// rationals, so the already-lifted vertices serve the area bracket too.
	lo, hi := wallTriangleArea(xsub(sb, sa), xsub(sc, sa))
	m.wallAreaSum += lo
	m.wallAreaAbs = upRound(m.wallAreaAbs + lo)
	m.wallAreaSlack = upRound(m.wallAreaSlack + upRound(hi-lo))
	m.wallTerms++
}

// wallTriangleArea brackets one wall triangle's own area between two floats,
// both PROVEN. u and v are the triangle's exact rational edge vectors, so
// |u×v|² is an exact rational and the area is the square root of |u×v|²/4;
// ratSqrtDown/ratSqrtUp (spline_length.go) bracket that rational root with
// OUTWARD rounding decided by exact comparison, so lo ≤ area ≤ hi holds
// whatever the platform's own sqrt does.
//
// The cross product is taken over rationals and NEVER in float64.
// r3.Vec.Cross is the naive difference-of-products form, whose forward error
// scales with the PRODUCTS rather than with the result, so a thin triangle's
// float area carries an error larger than the held sum's own summation slop by
// roughly one over the triangle's aspect ratio — and a bound read off that
// held sum would not enclose it. The wall of a short loft over long recorded
// LineSegs is exactly that shape (docs/loft-design.md Table B splits every
// wall quad along a diagonal), so this is the ordinary case, not an edge one.
func wallTriangleArea(u, v xpt) (float64, float64) {
	w := xcross(u, v)
	q := xdotRat(w, w)
	q.Quo(q, big.NewRat(4, 1))
	return ratSqrtDown(q), ratSqrtUp(q)
}

// foldBounds extends the componentwise extreme box over one held vertex.
// Comparing held coordinates introduces no rounding of its own, so the box is
// exactly as good as the vertex set it is taken over: exact on an unplaced
// body (docs/loft-design.md §5) and within delta of the true extreme on a
// placed one, which is what bounds() publishes.
func (m *loftMassAccumulator) foldBounds(p r3.Vec) {
	if !m.haveBounds {
		m.lo, m.hi = p, p
		m.haveBounds = true
		return
	}
	m.lo = r3.Vec{X: math.Min(m.lo.X, p.X), Y: math.Min(m.lo.Y, p.Y), Z: math.Min(m.lo.Z, p.Z)}
	m.hi = r3.Vec{X: math.Max(m.hi.X, p.X), Y: math.Max(m.hi.Y, p.Y), Z: math.Max(m.hi.Z, p.Z)}
}

// foldCoordUpper extends coordUpper over one held vertex's own inf-norm
// distance from anchor.
func (m *loftMassAccumulator) foldCoordUpper(p r3.Vec) {
	d := p.Sub(m.anchorF)
	m.coordUpper = max(m.coordUpper, math.Abs(d.X), math.Abs(d.Y), math.Abs(d.Z))
}

// volume publishes Σvol6/6, rounded to float64 exactly once. Its Exactness
// is exactnessOf the single rounding's proven error — Exact exactly when the
// published rational is representable in cubic millimetres, never
// unconditionally (docs/loft-design.md §8, spline design §3's Tier A rule).
//
// A placement (delta > 0, §12 PR 2a) widens that bound by
// bounds.go's sweptVolumeAllow(delta, areaUpper), areaUpper the SAME
// whole-mesh perturbedAreaUpper the identity fast path never reaches — this
// is the term that closes the measured 1.82e-12 gap a naive re-lift-and-round
// implementation misses: every held vertex is exact ONLY under the identity
// transform, and a general rigid motion rounds inside its own products and
// sums.
func (m *loftMassAccumulator) volume(verts []r3.Vec, tris [][3]int) Measurement {
	vol := new(big.Rat).Quo(m.vol6, big.NewRat(6, 1))
	value, _ := vol.Float64()
	bound := rationalFloatError(vol, value)
	if m.delta > 0 {
		areaUpper := perturbedAreaUpper(verts, tris, m.delta)
		bound = absSumUpper(bound, sweptVolumeAllow(m.delta, areaUpper))
	}
	return Measurement{
		Value:     units.CubicMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}
}

// centroid publishes anchor + Σmoment/(4·Σvol6) as a VecMeasurement, each of
// the three coordinates rounded to float64 exactly once. Bound is radius3D
// of the largest per-coordinate rounding error, Exact only when all three
// round exactly AND the payload carries no placement displacement
// (docs/loft-design.md §8, §12 PR 2a). A loft with zero net volume has no
// centroid.
//
// A placement (delta > 0) widens each coordinate's bound by
// placedCentroidAllow's own quotient composition — moments.go's
// boundedQuotient formula, specialized to the proven volume/moment
// allowances sweptVolumeAllow/sweptMomentAllow already state rather than a
// fresh division. A non-positive clearance (the volume allowance is not
// smaller than the held volume) leaves the quotient's denominator with
// nothing left to divide by, so the centroid is unstateable — refused
// ErrUnsupported (Table S, S12) rather than published with a bound nobody
// could use.
func (m *loftMassAccumulator) centroid(verts []r3.Vec, tris [][3]int) (VecMeasurement, error) {
	if m.vol6.Sign() == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a loft with zero net volume has no centroid`, ErrDegenerate)
	}
	denom := new(big.Rat).Mul(big.NewRat(4, 1), m.vol6)
	anchorX, anchorY, anchorZ := xhpRat(xhp(m.anchor))
	cx := new(big.Rat).Add(anchorX, new(big.Rat).Quo(m.momX, denom))
	cy := new(big.Rat).Add(anchorY, new(big.Rat).Quo(m.momY, denom))
	cz := new(big.Rat).Add(anchorZ, new(big.Rat).Quo(m.momZ, denom))

	fx, _ := cx.Float64()
	fy, _ := cy.Float64()
	fz, _ := cz.Float64()
	bx := rationalFloatError(cx, fx)
	by := rationalFloatError(cy, fy)
	bz := rationalFloatError(cz, fz)

	if m.delta > 0 {
		vol := new(big.Rat).Quo(m.vol6, big.NewRat(6, 1))
		volValue, _ := vol.Float64()
		areaUpper := perturbedAreaUpper(verts, tris, m.delta)
		epsV := sweptVolumeAllow(m.delta, areaUpper)
		epsM := sweptMomentAllow(m.delta, areaUpper, m.coordUpper+m.delta)

		clearance := math.Nextafter(math.Abs(volValue)-epsV, math.Inf(-1))
		if clearance <= 0 {
			return VecMeasurement{}, fmt.Errorf(`%w: the placement's proven volume allowance is not smaller than the held volume; this evaluator cannot state the placed centroid`, ErrUnsupported)
		}
		bx = absSumUpper(bx, placedCentroidAllow(fx-m.anchorF.X, epsM, epsV, clearance))
		by = absSumUpper(by, placedCentroidAllow(fy-m.anchorF.Y, epsM, epsV, clearance))
		bz = absSumUpper(bz, placedCentroidAllow(fz-m.anchorF.Z, epsM, epsV, clearance))
	}

	bound := radius3D(math.Max(bx, math.Max(by, bz)))

	return VecMeasurement{
		Value:     r3.NewVec(fx, fy, fz),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// placedCentroidAllow bounds how far one already-computed centroid
// coordinate can move under a placement's proven volume and first-moment
// allowances, mirroring moments.go's boundedQuotient formula (§12 PR 2a):
// coordRel is the coordinate's own value relative to the accumulator's
// anchor, epsM the proven first-moment allowance (sweptMomentAllow), epsV
// the proven volume allowance (sweptVolumeAllow), and clearance the
// caller's own proven positive gap between the held volume and epsV (S12's
// own test, checked once by the caller since it does not depend on
// coordRel).
func placedCentroidAllow(coordRel, epsM, epsV, clearance float64) float64 {
	numerator := absSumUpper(epsM, productUpper(math.Abs(coordRel), epsV))
	return upRound(numerator / clearance)
}

// bounds publishes the componentwise min/max over every held vertex. For an
// unplaced body it is always Exact: every vertex is already exact by
// construction, so an extreme over an already-exact set introduces no
// rounding of its own (docs/loft-design.md §8). A placed body's box carries
// the payload's own delta (§12 PR 2a): the held vertex set is no longer
// provably exact, so the box's Bound is delta and its Exactness is Exact
// only when delta is zero. The second return is false only when add has
// never been called.
func (m *loftMassAccumulator) bounds() (Box, bool) {
	if !m.haveBounds {
		return Box{}, false
	}
	return Box{
		Min:       m.lo,
		Max:       m.hi,
		Exactness: exactnessOf(m.delta),
		Bound:     units.Millimeters(m.delta),
	}, true
}

// area publishes the two caps' own exact rational areas (moments.go's
// ProfileRecord.Area, already-exact rationals — never the sum of their own
// triangulations' float areas) plus the wall triangles' float sum.
//
// The Exactness is the CONSTANT Approximate — docs/loft-design.md §8's
// "Area is never Exact", spline design §3's arc-length asymmetry — and is
// never derived from the published bound. A triangle's own area is a square
// root of a rational and is generically irrational, so no arithmetic on the
// bound can make the reading exactly representable; a bound that reaches zero
// says only that the bound arithmetic ran out of scale to state (sumSlop
// underflowing on a subnormal wall triangle, a saturated wallAreaAbs), which
// is a fact about the proof term and not about the value.
//
// Bound is proven independently, and every one of its four terms is charged
// at the magnitude where its own rounding happens:
//
//   - wallAreaSlack — Σ over the triangles of the exact-rational cross-norm
//     bracket's own width, so each triangle's area is charged at ITS OWN
//     scale, never at the held total's;
//   - sumSlop over wallAreaAbs — the summation loop that added those terms;
//   - capBound — the caps' exact rational rounding once into float64;
//   - addBound — the final wall+cap addition's own rounding, exact.
//
// wallBound owns the first two and answers +Inf where either has saturated,
// since neither is a proven scale any more.
func (m *loftMassAccumulator) area(capAreas ...*big.Rat) Measurement {
	capTotal := new(big.Rat)
	for _, ca := range capAreas {
		if ca != nil {
			capTotal.Add(capTotal, ca)
		}
	}
	capFloat, _ := capTotal.Float64()
	capBound := rationalFloatError(capTotal, capFloat)

	value := m.wallAreaSum + capFloat
	addBound := addRoundError(m.wallAreaSum, capFloat, value)
	bound := absSumUpper(m.wallBound(), capBound, addBound)
	// A placement's own per-triangle allowance covers both directions at
	// once: the wall sum is over held triangles and the cap term is the
	// denoted region's exact rational, and the two differ by at most this
	// sum (docs/loft-design.md §12 PR 2a). Gated on delta > 0 so an
	// unplaced loft's Area stays bit-identical to PR 1's.
	if m.delta > 0 {
		bound = absSumUpper(bound, m.perturbAreaSum)
	}

	return Measurement{
		Value:     units.SquareMillimeters(value),
		Exactness: Approximate,
		Bound:     units.SquareMillimeters(bound),
	}
}

// wallBound is the wall summation's own share of area's proven bound: the
// per-triangle enclosure widths beside the summation loop's slop.
//
// Both held terms are upper bounds nudged outward once per triangle, so either
// can SATURATE at +Inf on a wall set whose areas approach float64's own
// ceiling, while the plain sum they speak for stays finite by rounding whole
// triangles away. sumSlop answers a non-finite absSum with 0 — correct for a
// helper that cannot invent a scale, and fatal here, because that term is the
// ONLY cover the wall loop's rounding has: the slack is exactly 0 whenever
// every triangle's own area is representable, so the two together would leave
// a saturated sum publishing a ZERO bound over mass it has already swallowed.
//
// A saturated term is not a small bound, it is the absence of a proven scale,
// and the error it stands for here runs past MaxFloat64 — so the honest answer
// is +Inf. Any finite substitute would be a guess, which is what this kernel's
// proven-bound discipline exists to prevent (docs/loft-design.md §8).
func (m *loftMassAccumulator) wallBound() float64 {
	if isNonFinite(m.wallAreaAbs) || isNonFinite(m.wallAreaSlack) {
		return math.Inf(1)
	}
	return absSumUpper(m.wallAreaSlack, sumSlop(m.wallTerms, m.wallAreaAbs))
}
