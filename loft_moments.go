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

// loftMassAccumulator is docs/loft-design.md §8's tetrahedron-sum kernel. It
// streams one outward-oriented triangle of T at a time — a wall or a cap
// triangulation triangle — anchored at anchor (p0's own PlaneRecord.Origin).
// Volume, Centroid and Bounds fold every triangle handed to add; Area's wall
// contribution folds only the triangles whose wall flag is true, since a
// cap's own contribution is its 2-D region's exact rational area
// (moments.go), never the sum of its triangulation's own float areas.
type loftMassAccumulator struct {
	anchor xpt

	// vol6 is Σ (A-anchor)·((B-anchor)×(C-anchor)) over every triangle of T:
	// six times the signed volume (docs/loft-design.md §8).
	vol6 *big.Rat
	// momX/momY/momZ are Σ vol6_tri · Σ(A-anchor, B-anchor, C-anchor), the
	// first-moment accumulator the centroid divides by 4·vol6.
	momX, momY, momZ *big.Rat

	haveBounds bool
	lo, hi     r3.Vec // componentwise extremes over every held vertex

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
// first profile plane origin (docs/loft-design.md §8).
func newLoftMassAccumulator(anchor r3.Vec) *loftMassAccumulator {
	return &loftMassAccumulator{
		anchor: xptOf(anchor),
		vol6:   new(big.Rat),
		momX:   new(big.Rat),
		momY:   new(big.Rat),
		momZ:   new(big.Rat),
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

	triVol6 := xdot(sa, xcross(sb, sc))
	m.vol6.Add(m.vol6, triVol6)

	sumX := ratAdd(sa.x, sb.x, sc.x)
	sumY := ratAdd(sa.y, sb.y, sc.y)
	sumZ := ratAdd(sa.z, sb.z, sc.z)
	m.momX.Add(m.momX, new(big.Rat).Mul(triVol6, sumX))
	m.momY.Add(m.momY, new(big.Rat).Mul(triVol6, sumY))
	m.momZ.Add(m.momZ, new(big.Rat).Mul(triVol6, sumZ))

	m.foldBounds(a)
	m.foldBounds(b)
	m.foldBounds(c)

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
	q := xdot(w, w)
	q.Quo(q, big.NewRat(4, 1))
	return ratSqrtDown(q), ratSqrtUp(q)
}

// foldBounds extends the componentwise extreme box over one held vertex.
// Every vertex is already exact (docs/loft-design.md §5), so comparing them
// introduces no rounding of its own.
func (m *loftMassAccumulator) foldBounds(p r3.Vec) {
	if !m.haveBounds {
		m.lo, m.hi = p, p
		m.haveBounds = true
		return
	}
	m.lo = r3.Vec{X: math.Min(m.lo.X, p.X), Y: math.Min(m.lo.Y, p.Y), Z: math.Min(m.lo.Z, p.Z)}
	m.hi = r3.Vec{X: math.Max(m.hi.X, p.X), Y: math.Max(m.hi.Y, p.Y), Z: math.Max(m.hi.Z, p.Z)}
}

// volume publishes Σvol6/6, rounded to float64 exactly once. Its Exactness
// is exactnessOf the single rounding's proven error — Exact exactly when the
// published rational is representable in cubic millimetres, never
// unconditionally (docs/loft-design.md §8, spline design §3's Tier A rule).
func (m *loftMassAccumulator) volume() Measurement {
	vol := new(big.Rat).Quo(m.vol6, big.NewRat(6, 1))
	value, _ := vol.Float64()
	bound := rationalFloatError(vol, value)
	return Measurement{
		Value:     units.CubicMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}
}

// centroid publishes anchor + Σmoment/(4·Σvol6) as a VecMeasurement, each of
// the three coordinates rounded to float64 exactly once. Bound is radius3D
// of the largest per-coordinate rounding error, Exact only when all three
// round exactly (docs/loft-design.md §8). A loft with zero net volume has no
// centroid.
func (m *loftMassAccumulator) centroid() (VecMeasurement, error) {
	if m.vol6.Sign() == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a loft with zero net volume has no centroid`, ErrDegenerate)
	}
	denom := new(big.Rat).Mul(big.NewRat(4, 1), m.vol6)
	cx := new(big.Rat).Add(m.anchor.x, new(big.Rat).Quo(m.momX, denom))
	cy := new(big.Rat).Add(m.anchor.y, new(big.Rat).Quo(m.momY, denom))
	cz := new(big.Rat).Add(m.anchor.z, new(big.Rat).Quo(m.momZ, denom))

	fx, _ := cx.Float64()
	fy, _ := cy.Float64()
	fz, _ := cz.Float64()
	bx := rationalFloatError(cx, fx)
	by := rationalFloatError(cy, fy)
	bz := rationalFloatError(cz, fz)
	bound := radius3D(math.Max(bx, math.Max(by, bz)))

	return VecMeasurement{
		Value:     r3.NewVec(fx, fy, fz),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// bounds publishes the componentwise min/max over every held vertex. It is
// always Exact: every vertex is already exact by construction, so an extreme
// over an already-exact set introduces no rounding of its own
// (docs/loft-design.md §8). The second return is false only when add has
// never been called.
func (m *loftMassAccumulator) bounds() (Box, bool) {
	if !m.haveBounds {
		return Box{}, false
	}
	return Box{
		Min:       m.lo,
		Max:       m.hi,
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}, true
}

// area publishes the two caps' own exact rational areas (moments.go's
// ProfileRecord.Area, already-exact rationals — never the sum of their own
// triangulations' float areas) plus the wall triangles' float sum. Four proven
// terms bound the total, and every one of them is charged at the magnitude
// where its own rounding happens:
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
//
// Area is never Exact whenever any wall triangle has nonzero area
// (docs/loft-design.md §8, spline design §3's arc-length asymmetry).
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

	return Measurement{
		Value:     units.SquareMillimeters(value),
		Exactness: exactnessOf(bound),
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
// a saturated sum claiming Exact over mass it has already swallowed.
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
