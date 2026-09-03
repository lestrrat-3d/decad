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
// cap's own contribution is the exact rational shoelace area of the SAME
// polygon its triangulation was built from (loft_build.go's
// capPolygonAreaRat), never the sum of its triangulation's own float areas.
type loftMassAccumulator struct {
	anchor  xpt
	anchorF r3.Vec

	// delta is the placement's own proven displacement of every held vertex
	// from the exact placed image of the recorded sections (loft_build.go's
	// loftPayload.delta, docs/loft-design.md §5/§12 PR 2a) — zero for an
	// unplaced LineSeg-only body. Every extra term below is gated on
	// delta > 0, so an unplaced LineSeg-only loft's published measurements
	// stay bit-identical to PR 1's.
	delta float64

	// sectionDelta is loft_build.go's loftPayload.sectionDelta: the proven
	// upper bound, as a MAX over cells, on how far a BUILT CHORD point sits
	// from the recorded curve it chords, AS A SET (a10-plan.md Part 3 PR 6) —
	// zero for a LineSeg-only pairing. It is delta's independent twin
	// (loftPayload's own doc comment), never composed as if it were delta.
	// It is ALSO never bounds.go's cellChordCurveAreaUpper (or
	// chordedBoundaryVolumeResidualAllow/chordedBoundaryMomentResidualAllow/
	// chordedBoundarySeamAllow's) own matchedDeltaUpper obligation — a
	// STRONGER, PARAMETER-MATCHED quantity sectionMatchedDelta below carries
	// instead — and that includes the cap-area tube
	// (sectionDisplacementArea), whose own §5.2 row names the matched term
	// because a held cap polygon's vertices are displaced as well as chorded.
	// sectionDelta's OWN remaining spend is Bounds.Bound, a SET-distance
	// reading, and the terms below that read it are gated on sectionDelta > 0
	// exactly as the delta-driven terms are gated on delta > 0.
	sectionDelta float64

	// sectionMatchedDelta is docs/loft-design.md §5.2's own matchedDelta term,
	// composed by evalLoft as absSumUpper(sectionDelta, delta) over that
	// table's sectionDelta and delta rows: how far one point of a HELD chord
	// sits from the point the recorded curve denotes at the SAME arc-length
	// parameter. It is a DIFFERENT, strictly stronger quantity than
	// sectionDelta's own SET-distance sagitta and never interchangeable with
	// it, and it is not the sagitta with a new name either — the delta leg is
	// what charges the computed station's own displacement, which the sagitta
	// leaves out (§5.2's matchedDelta paragraph). Every composed reading that
	// needs "the SAME parameter-matched displacement leg (a)'s own
	// obligation" — bounds.go's own phrase, repeated verbatim on
	// chordedBoundaryVolumeResidualAllow, chordedBoundaryMomentResidualAllow and
	// chordedBoundarySeamAllow's own doc comments — reads this field, never
	// sectionDelta. It stays exactly 0 on a build with no chorded cell at all
	// (a LineSeg-only pairing), whose held triangle pair IS the boundary §5
	// gives it and whose vertex displacement the delta-keyed legs above
	// already charge.
	sectionMatchedDelta float64

	// chorded holds the corrections and residuals computeLoftChordedAllow derives from the
	// composed sectionMatchedDelta and the delta above (loft_build.go): every
	// field stays
	// its zero value unless evalLoft calls computeLoftChordedAllow, which it
	// does only when sectionDelta > 0 or sectionMatchedDelta > 0.
	chorded loftChordedAllow

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
	// distUpper is coordUpper's Euclidean twin (a10-plan.md Part 3 PR 6): the
	// max |v-anchor| (3D distance, not inf-norm) over every held vertex —
	// computeLoftChordedAllow's own posUpper reading, tighter than
	// radius3D(coordUpper) since it never assumes the three per-axis
	// extremes land on one vertex at once.
	distUpper float64

	// perturbAreaSum is Σ perturbedTriangleAreaAllow(...) over EVERY triangle
	// of T — walls and caps alike — the extra area the payload's own delta
	// can add on top of the held triangle areas area() already sums. That
	// per-TRIANGLE role is all of it: a chorded wall cell's held-to-denoted
	// SURFACE step is cellStationShiftAreaAllow's leg of areaExcess instead,
	// area()'s own composition site and cellChordCurveAreaAllow's (bounds.go)
	// owning the split. It stays exactly 0 when delta is 0.
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
// payload's own proven placement displacement delta (§12 PR 2a), the
// SET-distance sagitta sectionDelta, and §5.2's own composed PARAMETER-MATCHED
// term sectionMatchedDelta (a10-plan.md Part 3 PR 6/PR 9) — all three zero for
// an unplaced LineSeg-only body.
func newLoftMassAccumulator(anchor r3.Vec, delta, sectionDelta, sectionMatchedDelta float64) *loftMassAccumulator {
	return &loftMassAccumulator{
		anchor:              xptOf(anchor),
		anchorF:             anchor,
		delta:               delta,
		sectionDelta:        sectionDelta,
		sectionMatchedDelta: sectionMatchedDelta,
		vol6:                new(big.Rat),
		momX:                new(big.Rat),
		momY:                new(big.Rat),
		momZ:                new(big.Rat),
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
// distance from anchor, and distUpper (a10-plan.md Part 3 PR 6) over its
// own EUCLIDEAN distance from anchor — a tighter reading than
// radius3D(coordUpper) by up to sqrt(3), since the inf-norm bound assumes
// the per-axis extremes are simultaneously achieved at one vertex, which a
// real point set rarely does. computeLoftChordedAllow reads distUpper for
// chordedBoundarySeamAllow's own posUpper obligation.
//
// distUpper is PROVEN by exact rational arithmetic, the SAME mechanism
// computeLoftChordedAllow's own h1Upper reading already uses
// (ratSquaredDistance3/ratSqrtUp): p and anchorF are both float64, hence
// both exact rationals, so ratSquaredDistance3 is the true squared distance
// with no rounding of its own, and ratSqrtUp brackets its root by exact
// comparison, proven whatever the platform's own sqrt does. An earlier
// version of this function instead nudged r3.Vec.Len()'s own float64
// result outward by a single upRound — one ulp — which does not cover
// Len()'s own composed rounding (Sub, two nested Hypot calls each with
// their own error) and so was not actually proven to enclose the true
// distance; the exact-rational route replaces that single-ulp guess with a
// derivation this function's own callers can trust the same way h1Upper's
// already is. A vertex or anchor coordinate ratSquaredDistance3 cannot read
// as an exact rational (non-finite) answers +Inf here rather than silently
// dropping the widening — the same "absent bound must never read as small"
// rule this file's other terms already follow.
func (m *loftMassAccumulator) foldCoordUpper(p r3.Vec) {
	d := p.Sub(m.anchorF)
	m.coordUpper = max(m.coordUpper, math.Abs(d.X), math.Abs(d.Y), math.Abs(d.Z))
	dist := math.Inf(1)
	if d2 := ratSquaredDistance3(m.anchorF.X, m.anchorF.Y, m.anchorF.Z, p.X, p.Y, p.Z); d2 != nil {
		dist = ratSqrtUp(d2)
	}
	m.distUpper = max(m.distUpper, dist)
}

// volume publishes Σvol6/6 plus the exact bilinear-patch correction, rounded
// to float64 exactly once. Its Exactness is exactnessOf the single rounding's
// proven error — Exact exactly when the published rational is representable
// in cubic millimetres, never unconditionally (docs/loft-design.md §8,
// spline design §3's Tier A rule).
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
	if m.chorded.twistVolumeCorrection != nil {
		vol.Add(vol, m.chorded.twistVolumeCorrection)
	}
	value, _ := vol.Float64()
	bound := rationalFloatError(vol, value)
	if m.delta > 0 {
		areaUpper := perturbedAreaUpper(verts, tris, m.delta)
		bound = absSumUpper(bound, sweptVolumeAllow(m.delta, areaUpper))
	}
	if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		bound = absSumUpper(bound, chordedBoundaryVolumeResidualAllow(
			m.sectionMatchedDelta, m.chorded.wallAreaUpper,
			m.chorded.capVolumeUpper, m.chorded.seamAllow,
		))
	}
	return Measurement{
		Value:     units.CubicMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}
}

// centroid publishes anchor + Σmoment/(4·Σvol6) as a VecMeasurement after
// applying the exact bilinear-patch corrections to both the volume and first
// moments. Each coordinate rounds once. Its bound is radius3D of the largest
// per-coordinate rounding error, and a loft with zero corrected volume has no
// centroid.
//
// A placement (delta > 0) and a curved pairing (sectionDelta > 0 or
// sectionMatchedDelta > 0) each widen one COMBINED volume allowance epsV and
// one COMBINED first-moment allowance epsM before either is spent: delta's
// own leg is sweptVolumeAllow/sweptMomentAllow, and the curved pairing's leg
// is chordedBoundaryVolumeResidualAllow/chordedBoundaryMomentResidualAllow,
// both composed
// from sectionMatchedDelta — the PARAMETER-MATCHED quantity those two
// helpers' own doc comments oblige, never sectionDelta, whose SET-distance
// sagitta is spent on Bounds instead. The two legs are mechanically distinct (a vertex displaced versus
// a boundary replaced by a nearby non-mesh surface, docs/loft-design.md §5),
// but each publishes a volume and a first-moment allowance over the SAME
// anchored accumulator, so ONE clearance test and ONE placedCentroidAllow
// quotient composition — moments.go's boundedQuotient formula, specialized to
// whichever allowances are active — cover both. A non-positive clearance (the
// combined volume allowance is not smaller than the held volume) leaves the
// quotient's denominator with nothing left to divide by, so the centroid is
// unstateable — refused ErrUnsupported (Table S, S12) rather than published
// with a bound nobody could use. This gate is reachable on an UNPLACED body
// under a curved pairing alone (a10-plan.md Part 3 PR 6): delta's own fast
// path (delta == 0) does not imply epsV == 0, and EITHER section quantity on
// its own reaches it, since a free-form cell can carry a positive
// matchedDelta at an exactly-zero sagitta (spline_sagitta.go's own
// counterexample).
func (m *loftMassAccumulator) centroid(verts []r3.Vec, tris [][3]int) (VecMeasurement, error) {
	vol6 := new(big.Rat).Set(m.vol6)
	momX := new(big.Rat).Set(m.momX)
	momY := new(big.Rat).Set(m.momY)
	momZ := new(big.Rat).Set(m.momZ)
	if m.chorded.twistVolumeCorrection != nil {
		vol6.Add(vol6, new(big.Rat).Mul(big.NewRat(6, 1), m.chorded.twistVolumeCorrection))
		momX.Add(momX, m.chorded.twistMomentCorrection[0])
		momY.Add(momY, m.chorded.twistMomentCorrection[1])
		momZ.Add(momZ, m.chorded.twistMomentCorrection[2])
	}
	if vol6.Sign() == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a loft with zero net volume has no centroid`, ErrDegenerate)
	}
	denom := new(big.Rat).Mul(big.NewRat(4, 1), vol6)
	anchorX, anchorY, anchorZ := xhpRat(xhp(m.anchor))
	cx := new(big.Rat).Add(anchorX, new(big.Rat).Quo(momX, denom))
	cy := new(big.Rat).Add(anchorY, new(big.Rat).Quo(momY, denom))
	cz := new(big.Rat).Add(anchorZ, new(big.Rat).Quo(momZ, denom))

	fx, _ := cx.Float64()
	fy, _ := cy.Float64()
	fz, _ := cz.Float64()
	bx := rationalFloatError(cx, fx)
	by := rationalFloatError(cy, fy)
	bz := rationalFloatError(cz, fz)

	if m.delta > 0 || m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		vol := new(big.Rat).Quo(vol6, big.NewRat(6, 1))
		volValue, _ := vol.Float64()

		epsV, epsM := 0.0, 0.0
		if m.delta > 0 {
			areaUpper := perturbedAreaUpper(verts, tris, m.delta)
			epsV = absSumUpper(epsV, sweptVolumeAllow(m.delta, areaUpper))
			epsM = absSumUpper(epsM, sweptMomentAllow(m.delta, areaUpper, m.coordUpper+m.delta))
		}
		if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
			epsV = absSumUpper(epsV, chordedBoundaryVolumeResidualAllow(
				m.sectionMatchedDelta, m.chorded.wallAreaUpper,
				m.chorded.capVolumeUpper, m.chorded.seamAllow,
			))
			epsM = absSumUpper(epsM, chordedBoundaryMomentResidualAllow(
				m.sectionMatchedDelta, m.chorded.wallAreaUpper,
				m.chorded.capVolumeUpper, m.chorded.seamAllow, m.chorded.maxTwistOffsetUpper, m.coordUpper,
			))
		}

		clearance := math.Nextafter(math.Abs(volValue)-epsV, math.Inf(-1))
		if clearance <= 0 {
			return VecMeasurement{}, fmt.Errorf(`%w: the placement and section proven volume allowance is not smaller than the held volume; this evaluator cannot state the placed centroid`, ErrUnsupported)
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

// bounds publishes the componentwise min/max over every held vertex. Bound
// is absSumUpper(delta, sectionDelta) — NON-NEGOTIABLE (a10-plan.md Part 3
// PR 6): a chorded curved section's TRUE curve bulges OUTSIDE the station
// polygon this box is taken over, so a box carrying only delta UNDERSTATES
// the true box, and Verify's box-disjointness (Table D row D3) reads Bounds
// to prove pairs disjoint — understating it is unsound in the one direction
// that matters. For an unplaced LineSeg-only body both terms are exactly
// zero, absSumUpper(0, 0) is exactly 0.0 (upRound never nudges a
// non-positive value), and Exactness stays Exact — bit-identical to before
// this field existed. The second return is false only when add has never
// been called.
func (m *loftMassAccumulator) bounds() (Box, bool) {
	if !m.haveBounds {
		return Box{}, false
	}
	bound := absSumUpper(m.delta, m.sectionDelta)
	return Box{
		Min:       m.lo,
		Max:       m.hi,
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, true
}

// area publishes the two caps' exact rational shoelace areas plus a mixed wall
// reading: held triangle areas for LineSeg cells and certified bilinear-patch
// midpoints for chorded cells.
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
// Bound is proven independently, and every one of its base four terms is
// charged at the magnitude where its own rounding happens:
//
//   - wallAreaSlack — Σ over the triangles of the exact-rational cross-norm
//     bracket's own width, so each triangle's area is charged at ITS OWN
//     scale, never at the held total's;
//   - sumSlop over wallAreaAbs — the summation loop that added those terms;
//   - capBound — the caps' exact rational rounding once into float64;
//   - addBound — the final wall+cap addition's own rounding, exact.
//
// wallBound owns the first two and answers +Inf where either has saturated,
// since neither is a proven scale any more. A curved pairing (sectionDelta >
// 0) adds the bilinear integration enclosure, computeLoftChordedAllow's own
// two-leg wall residual, and capAreaExcess (the SAME cap
// chord-versus-curve gap capVolumeUpper folds into Volume, spent here as an
// area rather than a volume) — both documented at the composition below. A
// displaced build (delta > 0) adds perturbAreaSum, the held triangles' and the
// caps' own per-triangle placement allowance; the wall's own held-to-denoted
// SURFACE step is a leg of areaExcess, not of that sum, and the composition
// site below owns the split.
func (m *loftMassAccumulator) area(capAreas ...*big.Rat) Measurement {
	capTotal := new(big.Rat)
	for _, ca := range capAreas {
		if ca != nil {
			capTotal.Add(capTotal, ca)
		}
	}
	capFloat, _ := capTotal.Float64()
	capBound := rationalFloatError(capTotal, capFloat)

	wallValue := m.wallAreaSum
	wallBound := m.wallBound()
	if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		corrected := wallValue + m.chorded.areaCorrection
		wallBound = absSumUpper(
			wallBound,
			m.chorded.areaCorrectionBound,
			m.chorded.bilinearAreaBound,
			addRoundError(wallValue, m.chorded.areaCorrection, corrected),
		)
		wallValue = corrected
	}
	value := wallValue + capFloat
	addBound := addRoundError(wallValue, capFloat, value)
	bound := absSumUpper(wallBound, capBound, addBound)
	// The per-triangle allowance covers both directions at once: the base wall
	// accumulator is over held triangles and the cap term is the denoted region's
	// exact rational, and the two differ by at most this sum
	// (docs/loft-design.md §12 PR 2a). That TRIANGLE-level role is the whole
	// of what it is proven for. The wall's own SURFACE step from the held
	// corners to the stations they denote is a different quantity of the same
	// shape, charged by cellStationShiftAreaAllow inside areaExcess below;
	// cellChordCurveAreaAllow's own composition section owns that
	// split. The gate stays delta > 0, and an unplaced LineSeg-only loft's
	// Area stays bit-identical to PR 1's.
	if m.delta > 0 {
		bound = absSumUpper(bound, m.perturbAreaSum)
	}
	// A curved pairing's own two-leg wall residual PLUS its own cap
	// chord-versus-curve excess (a10-plan.md Part 3 PR 6,
	// computeLoftChordedAllow's own doc comment): the corrected wall value above
	// uses bilinear patches, and the true wall surface a circular cell denotes
	// differs by at most areaExcess; capFloat
	// above is capPolygonAreaRat, the built polygon's own exact rational, and
	// the region the loft's construction actually denotes is the CURVED
	// region sectionDisplacementArea bounds the gap to on EITHER cap
	// (capAreaExcess) — the identical gap capVolumeUpper folds into the
	// Volume leg via a plane-offset division that Area, having no such
	// offset, spends unfolded. Both are gated on sectionDelta > 0 OR
	// sectionMatchedDelta > 0 — never sectionDelta alone — so a free-form
	// cell whose matchedDelta is positive at an exactly-zero sagitta
	// (spline_sagitta.go's own counterexample) still has its wall and cap
	// excess charged; an unplaced LineSeg-only loft, where both are exactly
	// 0, stays bit-identical to PR 1's.
	if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		bound = absSumUpper(bound, m.chorded.areaExcess, m.chorded.capAreaExcess)
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

// loftChordedAllow bundles the exact volume and first-moment corrections,
// the unsigned twist measure retained for tessellation's occupied-volume
// proof, the three residual volume terms, and the wall's two-leg area residual
// (docs/loft-design.md §5/§8, a10-plan.md
// Part 3 PR 6's integration task). Every field of the zero value is 0, the
// correct standing for a LineSeg-only loft that never calls
// computeLoftChordedAllow at all. It is also what a REFUSING call returns
// beside its error, where the zero stands for nothing at all and no consumer
// ever sees it: evalLoft propagates that error and publishes no measurement.
type loftChordedAllow struct {
	wallAreaUpper         float64
	twistVolumeUpper      float64
	twistVolumeCorrection *big.Rat
	twistMomentCorrection ratV3
	maxTwistOffsetUpper   float64
	capVolumeUpper        float64
	seamAllow             float64
	// areaCorrection moves Area.Value from the held chord facets to the
	// bilinear wall patches. Its bound covers the correction's one rounding;
	// bilinearAreaBound covers those patches' certified integration intervals.
	areaCorrection      float64
	areaCorrectionBound float64
	bilinearAreaBound   float64
	// areaExcess is the wall's ruled and station-shift residuals summed over
	// chorded cells. The held-to-bilinear step is in Area.Value above.
	areaExcess float64
	// twistAreaAllow is that SAME held-to-bilinear step read as a BOUND
	// rather than as a correction: cellTwistAreaAllow summed over the same
	// chorded cells the loop below walks
	// (docs/tessellation-reach-design.md §4). area() never reads it, because
	// areaCorrection has already MOVED Area.Value onto the bilinear patches
	// and charging the gap again there would double-count it. The
	// tessellation does read it: the mesh holds the UNCORRECTED held
	// triangles, so the step areaCorrection performs is, for that triangle
	// set, an outstanding area gap and the mesh's own areaSlack must carry it
	// (docs/tessellation-design.md §2's loftPayload row). It stays exactly 0
	// on a build with no chorded cell, the zero value's own standing.
	twistAreaAllow float64
	// capAreaExcess is capAreaAllow0 and capAreaAllow1 (each
	// sectionDisplacementArea over its own cap's boundary) composed by
	// absSumUpper: the SAME two per-cap area allowances capVolumeUpper folds
	// into a volume via capAreaVolumeAllow's own |h|/3 identity, spent here
	// UNFOLDED as an area instead — area()'s own AREA reading, unlike
	// volume(), has no plane offset h to divide by, since a cap's own
	// published area IS the built polygon's area and the true denoted area
	// differs from it by exactly this much (docs/loft-design.md §5/§8).
	capAreaExcess float64
}

// computeLoftChordedAllow derives loftChordedAllow's corrections and bounds by walking
// every wall cell of every loop pairs holds, over the SAME cell corner
// convention assembleLoft's own Table B split uses (vLo, vHi = section-0's
// two corners; wLo, wHi = section-1's) — verts/vIdx/wIdx are evalLoft's own
// assembled loftAssembly fields, read after the whole mass-accumulator add
// loop so coordUpper is complete. anchor is the accumulator's own anchor
// (p0's placed plane origin).
//
// matchedDelta is docs/loft-design.md §5.2's own matchedDelta term, already
// COMPOSED by evalLoft as absSumUpper(sectionDelta, delta) over that table's
// sectionDelta and delta rows — the build-wide PARAMETER-MATCHED displacement,
// spent on every leg that table keys on it: the cap-area tube
// (sectionDisplacementArea, below, whose capAreaAllow row names this term and
// not the sagitta, because a held cap polygon's own vertices are displaced as
// well as chorded) and chordedBoundarySeamAllow's own matchedDelta/posUpper
// obligations. delta is that same table's delta row on its own — the held
// vertex displacement — passed alongside so each cell can compose its OWN
// tighter matched reading through chordCellDeltaUpper (loft_build.go) from the
// cell's own chord-to-curve half rather than the build-wide maximum. Neither is
// ever the sagitta alone: reading matchedDelta as sectionDelta leaves the
// computed station's own displacement uncharged on every chorded leg (§5.2's
// matchedDelta paragraph).
//
// A cell contributes to wallAreaUpper (cellChordCurveAreaUpper), the exact
// volume/first-moment corrections, the unsigned twistVolumeUpper retained for
// tessellation's occupied-volume proof, maxTwistOffsetUpper (the MAX,
// never a sum, of cellTwistOffsetUpper), twistAreaAllow (the SUM of
// cellTwistAreaAllow, retained for the tessellation's own area slack) and
// seamPerimeterUpper's own running
// total ONLY when its own CHORD-TO-CURVE departure is positive
// (pairs[i].matchedDelta[j] > 0, the cell's own half of §5.2's matchedDelta
// composition) — NEVER keyed on a segment kind (a10-plan.md Part 3 PR 9 Task
// 1a): an exact LineSeg cell's true curve already IS its chord, so that half
// is exactly 0 and the cell has nothing to contribute to any of those four
// legs — its held triangle pair IS the boundary §5 gives it, and its own
// vertex displacement is charged by the accumulator's delta-keyed legs
// instead, never twice. The gate therefore reads the cell's chord-to-curve
// half and never the composed matched value, which a placed build makes
// positive on every cell including the straight ones.
// Skipping such a cell also keeps the bound tighter than passing a zero
// displacement through the same machinery would. Any OTHER cell with a
// positive chord-to-curve half — circular today, a same-kind Tier A free-form cell once that
// arm lands — is charged, regardless of which arm produced it: gating on the
// PROVEN quantity itself, rather than on an enum naming which arm ran, is
// what keeps a future arm from being silently exempted from this whole
// charge the way an earlier version of this evaluator's kind-keyed gate
// would have exempted it.
//
// perimeterUpperV/perimeterUpperW and walksV/walksW sum only the POSITIVE-
// matchedDelta cells of their own cap, never every cell regardless of kind:
// under the fixed-station chord-to-curve homotopy this whole function
// reasons about, a LineSeg boundary never moves at all (its chord IS the
// curve it denotes), so the symmetric difference between the held cap
// polygon and the true denoted region is the union of the curved cells' own
// lenses alone and already lies inside the tube sectionDisplacementArea
// takes over their own perimeter — including the straight cells would only
// widen an already-sound bound, never repair an unsound one, so they are
// left out.
//
// h0 (cap0's own offset from anchor) is always exactly zero because anchor
// IS a point on cap0's own plane (evalLoft's own anchor := xform.Apply(
// plane0.Origin)) — capAreaVolumeAllow(0, ...) answers 0 without reading its
// second argument, so capAreaAllow0 is computed but never spent BY THE VOLUME
// LEG. h1 (cap1's own offset) is bounded by the proven exact-rational
// distance from anchor to ANY one held cap1 vertex — valid because a plane's
// own perpendicular offset from a point is never more than the distance to
// any single point ON that plane, and every cap1 vertex lies on cap1's own
// plane exactly. Where the assembly states no such distance this function
// RETURNS errLoftCapOffsetUnderivable and publishes nothing, which is the one
// error it raises and the reason it returns one at all; the site itself owns
// why a zero would not do.
//
// capAreaAllow0 and capAreaAllow1 are ALSO area()'s own AREA reading of the
// identical cap gap capVolumeUpper folds into a volume: h0 being zero sinks
// capAreaAllow0 out of capVolumeUpper (an area gap in a plane THROUGH the
// anchor sweeps no volume), but it never sinks the AREA those two allowances
// bound in the first place — a cap's published Area is capPolygonAreaRat, the
// built polygon's own exact rational, and the region the loft's construction
// actually denotes is the curved region sectionDisplacementArea(matchedDelta,
// walks, perimeterUpper) bounds the gap to, for EITHER cap, offset or not —
// the MATCHED term, never the sagitta, because that cap polygon's own held
// vertices are displaced as well as chorded (§5.2's capAreaAllow row).
// capAreaExcess is absSumUpper(capAreaAllow0, capAreaAllow1), unfolded by no
// plane-offset division at all, and area() charges it beside the wall term
// below — omitting it (an earlier version of this function did) understates
// Area on every curved pairing, caught on the shipped A10a wedge fixture
// itself (TestLoftArcWedgeAreaMatchesExtrudeOracle).
//
// areaExcess is the two residual legs of the wall's own per-cell gap after
// cellBilinearArea has moved the nominal reading from held facets to the
// bilinear patch through the cell's four held corners:
//
//   - the RULED leg, cellChordCurveAreaAllow (bounds.go): |bilinear − true|,
//     how far that bilinear patch's own area sits from the ruled patch through
//     the same four corners. It reads the cell's own held corners, its two
//     per-side arc-length bounds, its PARAMETER-MATCHED matchedDelta
//     and its two per-side tangent-deviation ENERGIES
//     (p.tangentEnergyV/tangentEnergyW, perCellTangentEnergy's own per-arm
//     reading), and that helper's own doc comment carries the derivation. It
//     replaces an arc-minus-chord LENGTH excess times a rung length, a shape
//     third order in the cell's own sweep where the gap it stood for is
//     SECOND order wherever the ruling runs anything but square across the
//     section tangent — an understatement without bound on a twisted
//     pairing.
//   - the STATION-SHIFT leg, cellStationShiftAreaAllow (bounds.go):
//     |ruled at the held corners − ruled at the denoted stations|, the step the
//     ruled leg stops short of, since it pins its patch at the corners
//     the build HOLDS. It reads the cell's own held corners for the same eB
//     convexity rung the ruled leg forms, its two per-side arc-length bounds,
//     the SAME composed cellMatched those legs take, and the payload's own
//     station displacement delta; that helper's own doc comment carries the
//     derivation, one dimension up from the |e x v| + |u x f| + |e x f|
//     expansion perturbedTriangleAreaAllow states for a single triangle. It is
//     exactly zero on a build that holds the stations it denotes.
//
// cellChordCurveAreaAllow's own composition section owns the split and why the
// ruled reading is not widened by delta to stand in for the station shift.
//
// This is the wall term; capAreaExcess above is Area's own cap term, and the
// two together are what area() charges beside the mixed wall reading.
func computeLoftChordedAllow(pairs []loftLoopPair, vIdx, wIdx [][]int, verts []r3.Vec, anchor r3.Vec, matchedDelta, delta, distUpper float64, reversed bool) (loftChordedAllow, error) {
	// Derive cap1's offset before lifting any wall cell into exact rationals.
	// Production has already refused non-finite vertices at S13, while direct
	// internal callers still receive S14's existing derivation refusal instead
	// of reaching mustRatOf with a NaN.
	h1Upper := math.Inf(1)
	for _, row := range wIdx {
		for _, idx := range row {
			v := verts[idx]
			d2 := ratSquaredDistance3(anchor.X, anchor.Y, anchor.Z, v.X, v.Y, v.Z)
			if d2 == nil {
				return loftChordedAllow{}, errLoftCapOffsetUnderivable
			}
			h1Upper = math.Min(h1Upper, ratSqrtUp(d2))
		}
	}
	if isNonFinite(h1Upper) {
		return loftChordedAllow{}, errLoftCapOffsetUnderivable
	}

	var wallAreaUpper, twistVolumeUpper, maxTwistOffsetUpper, seamPerimeterUpper float64
	var perimeterUpperV, perimeterUpperW, areaExcess, bilinearAreaBound, twistAreaAllow float64
	var walksV, walksW int
	twistVolumeCorrection := new(big.Rat)
	var twistMomentCorrection ratV3
	for axis := range twistMomentCorrection {
		twistMomentCorrection[axis] = new(big.Rat)
	}
	areaCorrection := new(big.Rat)

	for i, p := range pairs {
		n := len(p.v)
		for j := range n {
			jn := (j + 1) % n
			vLo, vHi := verts[vIdx[i][j]], verts[vIdx[i][jn]]
			wLo, wHi := verts[wIdx[i][j]], verts[wIdx[i][jn]]
			if p.matchedDelta[j] <= 0 {
				// An exact LineSeg cell's own recorded chord IS the curve it
				// denotes, so its true departure is exactly zero: excluding
				// it from the cap's own perimeter/walks tally
				// (sectionDisplacementArea's own tube-plus-joints argument)
				// is sound, not merely convenient — a zero-width segment of
				// the tube contributes nothing to widen, and a joint whose
				// own incident boundary never moves contributes no disk
				// either. Gated on the proven matchedDelta itself, never on a
				// segment kind (this function's own doc comment).
				continue
			}
			walksV++
			walksW++
			perimeterUpperV = absSumUpper(perimeterUpperV, p.arcUpperV[j])
			perimeterUpperW = absSumUpper(perimeterUpperW, p.arcUpperW[j])

			// docs/loft-design.md §5.2's matchedDelta row, at THIS cell:
			// the cell's own chord-to-curve half composed with the held
			// vertex displacement delta, which is what makes the bound a
			// claim about the chord the build actually DREW rather than the
			// ideal chord between the two points the record denotes. The
			// per-cell composition is at most the build-wide matchedDelta
			// above (that term takes the same sum at the MAX cell), so it is
			// the tighter of the two readings of the same row.
			cellMatched := chordCellDeltaUpper(p.matchedDelta[j], delta)

			cellWallUpper := cellChordCurveAreaUpper(vLo, vHi, wLo, wHi, p.arcUpperV[j], p.arcUpperW[j], cellMatched)
			wallAreaUpper = absSumUpper(wallAreaUpper, cellWallUpper)
			twistVolumeUpper = absSumUpper(twistVolumeUpper, cellTwistVolumeAllow(vLo, vHi, wLo, wHi))
			cellTwist := cellTwistVolume(vLo, vHi, wLo, wHi)
			cellMoment := cellTwistMoment(vLo, vHi, wLo, wHi, anchor)
			if reversed {
				cellTwist.Neg(cellTwist)
				for axis := range cellMoment {
					cellMoment[axis].Neg(cellMoment[axis])
				}
			}
			twistVolumeCorrection.Add(twistVolumeCorrection, cellTwist)
			for axis := range cellMoment {
				twistMomentCorrection[axis].Add(twistMomentCorrection[axis], cellMoment[axis])
			}
			maxTwistOffsetUpper = math.Max(maxTwistOffsetUpper, cellTwistOffsetUpper(vLo, vHi, wLo, wHi))
			seamPerimeterUpper = absSumUpper(seamPerimeterUpper, p.arcUpperV[j], p.arcUpperW[j])

			ruledLeg := cellChordCurveAreaAllow(
				vLo, vHi, wLo, wHi,
				p.arcUpperV[j], p.arcUpperW[j], cellMatched,
				p.tangentEnergyV[j], p.tangentEnergyW[j],
			)
			bilinearValue, bilinearBound := cellBilinearArea(vLo, vHi, wLo, wHi)
			bilinearAreaBound = absSumUpper(bilinearAreaBound, bilinearBound)
			vLoX := xptOf(vLo)
			lowerLo, _ := wallTriangleArea(xsub(xptOf(vHi), vLoX), xsub(xptOf(wHi), vLoX))
			upperLo, _ := wallTriangleArea(xsub(xptOf(wHi), vLoX), xsub(xptOf(wLo), vLoX))
			areaCorrection.Add(areaCorrection, mustRatOf(bilinearValue))
			areaCorrection.Sub(areaCorrection, mustRatOf(lowerLo))
			areaCorrection.Sub(areaCorrection, mustRatOf(upperLo))
			stationLeg := cellStationShiftAreaAllow(
				vLo, vHi, wLo, wHi,
				p.arcUpperV[j], p.arcUpperW[j], cellMatched, delta,
			)
			areaExcess = absSumUpper(areaExcess, ruledLeg, stationLeg)
			twistAreaAllow = absSumUpper(twistAreaAllow, cellTwistAreaAllow(vLo, vHi, wLo, wHi))
		}
	}

	capAreaAllow0 := sectionDisplacementArea(matchedDelta, walksV, perimeterUpperV)
	capAreaAllow1 := sectionDisplacementArea(matchedDelta, walksW, perimeterUpperW)
	// h1Upper (cap1's own offset from anchor) is bounded by the distance to
	// the CLOSEST held cap1 vertex to anchor, never an arbitrary one: a
	// plane's own perpendicular offset from a point is at most the distance
	// to ANY point on that plane, so the minimum over every held vertex is
	// the tightest such bound this evaluator can read off the assembly
	// without a fresh plane-distance computation of its own.
	//
	// Either half of that reading failing is §5.2's cap planeOffsetUpper row
	// answering +Inf, and this function REFUSES on it rather than publishing
	// a number: a vertex whose coordinates ratSquaredDistance3 cannot read as
	// exact rationals (a non-finite coordinate) states no distance at all, and
	// an assembly whose every cap1 vertex overflows ratSqrtUp leaves the
	// minimum at +Inf, as does an assembly stating no cap1 vertex. §5.2's own
	// closing rule — an enclosure the record cannot state answers +Inf and the
	// build refuses at Table S row S14, "never a finite substitute and never a
	// published zero" — is what forbids the obvious alternative of assigning
	// h1Upper = 0 here. That zero is not a bound: capAreaVolumeAllow takes its
	// planeOffsetUpper <= 0 arm on it and publishes capVolumeUpper = 0, the
	// SMALLEST possible number standing in for a quantity this evaluator could
	// not derive, in a term every consumer reads as an upper bound. Nothing
	// about the surrounding legs is allowed to excuse it: whether a sibling leg
	// happens to saturate on the same assembly is that leg's own business, and
	// a bound may not rest on another term's value to stay sound.
	capVolumeUpper := absSumUpper(
		capAreaVolumeAllow(0, capAreaAllow0),
		capAreaVolumeAllow(h1Upper, capAreaAllow1),
	)

	// posUpper (chordedBoundarySeamAllow's own obligation): the held
	// material's own max EUCLIDEAN distance from anchor (distUpper, tighter
	// than radius3D(coordUpper) — this file's own foldCoordUpper doc
	// comment), widened by matchedDelta — chordedBoundarySeamAllow's
	// own doc comment requires "the SAME parameter-matched displacement leg
	// (a)'s own obligation", never the sagitta alone, so this widening and
	// the matchedDelta argument beside it both read §5.2's own COMPOSED
	// matched term, never either half of it alone.
	posUpper := absSumUpper(distUpper, matchedDelta)
	seamAllow := chordedBoundarySeamAllow(matchedDelta, posUpper, seamPerimeterUpper)
	areaCorrectionValue, _ := areaCorrection.Float64()

	return loftChordedAllow{
		wallAreaUpper:         wallAreaUpper,
		twistVolumeUpper:      twistVolumeUpper,
		twistVolumeCorrection: twistVolumeCorrection,
		twistMomentCorrection: twistMomentCorrection,
		maxTwistOffsetUpper:   maxTwistOffsetUpper,
		capVolumeUpper:        capVolumeUpper,
		seamAllow:             seamAllow,
		areaCorrection:        areaCorrectionValue,
		areaCorrectionBound:   rationalFloatError(areaCorrection, areaCorrectionValue),
		bilinearAreaBound:     bilinearAreaBound,
		areaExcess:            areaExcess,
		twistAreaAllow:        twistAreaAllow,
		capAreaExcess:         absSumUpper(capAreaAllow0, capAreaAllow1),
	}, nil
}

// errLoftCapOffsetUnderivable is the sentinel docs/loft-design.md Table S row
// S14 carries for the cap planeOffsetUpper term (§5.2), raised in the
// CONSTRUCTION arm §4's gate-order paragraph assigns that term — it reads the
// held vertex table, which the record-only arm does not have. Like its
// certified-sagitta and station-displacement twins the shape itself is fine and
// the chord set is buildable; only one of the proven displacement terms the
// published cap volume allowance is composed from cannot be stated, so the
// sentinel is ErrUnsupported and no finite value — least of all a zero — is
// published in its place.
var errLoftCapOffsetUnderivable = fmt.Errorf(
	`%w: a chorded loft cap's plane offset from the mass anchor has no derivation from this assembly`, ErrUnsupported,
)
