package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
// internal/freeform/spline_length.go's outward-rounded proofbound.RatSqrtDown/proofbound.RatSqrtUp, and the published
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
	anchor  proofbound.Xpt
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
	// It is ALSO never internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper (or
	// proofbound.ChordedBoundaryVolumeResidualAllow/proofbound.ChordedBoundaryMomentResidualAllow/
	// proofbound.ChordedBoundarySeamAllow's) own matchedDeltaUpper obligation — a
	// STRONGER, PARAMETER-MATCHED quantity sectionMatchedDelta below carries
	// instead — and that includes the cap-area tube
	// (proofbound.SectionDisplacementArea), whose own §5.2 row names the matched term
	// because a held cap polygon's vertices are displaced as well as chorded.
	// sectionDelta's OWN remaining spend is Bounds.Bound, a SET-distance
	// reading, and the terms below that read it are gated on sectionDelta > 0
	// exactly as the delta-driven terms are gated on delta > 0.
	sectionDelta float64

	// sectionMatchedDelta is docs/loft-design.md §5.2's own matchedDelta term,
	// composed by evalLoft as proofbound.AbsSumUpper(sectionDelta, delta) over that
	// table's sectionDelta and delta rows: how far one point of a HELD chord
	// sits from the point the recorded curve denotes at the SAME arc-length
	// parameter. It is a DIFFERENT, strictly stronger quantity than
	// sectionDelta's own SET-distance sagitta and never interchangeable with
	// it, and it is not the sagitta with a new name either — the delta leg is
	// what charges the computed station's own displacement, which the sagitta
	// leaves out (§5.2's matchedDelta paragraph). Every composed reading that
	// needs "the SAME parameter-matched displacement leg (a)'s own
	// obligation" — internal/proofbound/bounds.go's own phrase, repeated verbatim on
	// proofbound.ChordedBoundaryVolumeResidualAllow, proofbound.ChordedBoundaryMomentResidualAllow and
	// proofbound.ChordedBoundarySeamAllow's own doc comments — reads this field, never
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
	chorded loftmesh.LoftChordedAllow

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
	// vertex (internal/proofbound/bounds.go's proofbound.SweptMomentAllow reads it widened by delta at the
	// point of use, since the true vertex may sit up to delta further out).
	coordUpper float64
	// distUpper is coordUpper's Euclidean twin (a10-plan.md Part 3 PR 6): the
	// max |v-anchor| (3D distance, not inf-norm) over every held vertex —
	// computeLoftChordedAllow's own posUpper reading, tighter than
	// proofbound.Radius3D(coordUpper) since it never assumes the three per-axis
	// extremes land on one vertex at once.
	distUpper float64

	// perturbAreaSum is Σ proofbound.PerturbedTriangleAreaAllow(...) over EVERY triangle
	// of T — walls and caps alike — the extra area the payload's own delta
	// can add on top of the held triangle areas area() already sums. That
	// per-TRIANGLE role is all of it: a chorded wall cell's held-to-denoted
	// SURFACE step is proofbound.CellStationShiftAreaAllow's leg of areaExcess instead,
	// area()'s own composition site and proofbound.CellChordCurveAreaAllow's (internal/proofbound/bounds.go)
	// owning the split. It stays exactly 0 when delta is 0.
	perturbAreaSum float64

	// wallAreaSum is the naive float sum of the per-triangle PROVEN LOWER
	// bounds loftmesh.WallTriangleArea returns; wallAreaAbs is an upper bound on
	// Σ|term|, the scale proofbound.SumSlop's summation proof reads; wallAreaSlack is an
	// upper bound on Σ (upper − lower), the enclosure width each triangle's
	// own area contributes. The three are what make the published bound a
	// proof rather than an estimate.
	//
	// Each of the two proof terms is an UPPER bound and is nudged outward once
	// per term, so each diverges upward from the value it speaks for and can
	// SATURATE at +Inf while wallAreaSum is still finite. A saturated term has
	// stopped being a proven scale, and area() publishes an infinite bound
	// rather than the zero proofbound.SumSlop reports for a non-finite absSum.
	wallAreaSum   float64
	wallAreaAbs   float64
	wallAreaSlack float64
	wallTerms     int // term count proofbound.SumSlop needs to bound that sum
}

// newLoftMassAccumulator opens a fresh accumulator anchored at the loft's
// first profile plane origin (docs/loft-design.md §8), carrying the
// payload's own proven placement displacement delta (§12 PR 2a), the
// SET-distance sagitta sectionDelta, and §5.2's own composed PARAMETER-MATCHED
// term sectionMatchedDelta (a10-plan.md Part 3 PR 6/PR 9) — all three zero for
// an unplaced LineSeg-only body.
func newLoftMassAccumulator(anchor r3.Vec, delta, sectionDelta, sectionMatchedDelta float64) *loftMassAccumulator {
	return &loftMassAccumulator{
		anchor:              proofbound.XptOf(anchor),
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
// coordinate is a float64, hence an exact rational (internal/freeform/clearance_poly.go's
// take-the-floats-exactly discipline); the volume and centroid sums round
// nothing until publication, and the area sum's own terms are the endpoints
// of a proven per-triangle enclosure rather than a float evaluation.
func (m *loftMassAccumulator) add(a, b, c r3.Vec, wall bool) {
	m.addTriangle(a, b, c, wall, [3]int{}, nil)
}

// addTriangle keeps add's triangle fold unchanged. Only evalLoft supplies
// indices and a cache, so repeated references to one assembled vertex reuse
// its exact Euclidean upper distance.
func (m *loftMassAccumulator) addTriangle(a, b, c r3.Vec, wall bool, indices [3]int, distances []loftmesh.LoftVertexDistance) {
	sa := proofbound.Xsub(proofbound.XptOf(a), m.anchor)
	sb := proofbound.Xsub(proofbound.XptOf(b), m.anchor)
	sc := proofbound.Xsub(proofbound.XptOf(c), m.anchor)

	triVol6 := proofbound.XdotRat(sa, meshbool.Xcross(sb, sc))
	m.vol6.Add(m.vol6, triVol6)

	saX, saY, saZ := meshbool.XhpRat(proofbound.Xhp(sa))
	sbX, sbY, sbZ := meshbool.XhpRat(proofbound.Xhp(sb))
	scX, scY, scZ := meshbool.XhpRat(proofbound.Xhp(sc))
	sumX := proofbound.RatAdd(saX, sbX, scX)
	sumY := proofbound.RatAdd(saY, sbY, scY)
	sumZ := proofbound.RatAdd(saZ, sbZ, scZ)
	m.momX.Add(m.momX, new(big.Rat).Mul(triVol6, sumX))
	m.momY.Add(m.momY, new(big.Rat).Mul(triVol6, sumY))
	m.momZ.Add(m.momZ, new(big.Rat).Mul(triVol6, sumZ))

	m.foldBounds(a)
	m.foldBounds(b)
	m.foldBounds(c)
	if distances == nil {
		m.foldCoordUpper(a)
		m.foldCoordUpper(b)
		m.foldCoordUpper(c)
	} else {
		m.foldCoordUpperCached(a, &distances[indices[0]])
		m.foldCoordUpperCached(b, &distances[indices[1]])
		m.foldCoordUpperCached(c, &distances[indices[2]])
	}

	if m.delta > 0 {
		m.perturbAreaSum = proofbound.UpRound(m.perturbAreaSum + proofbound.PerturbedTriangleAreaAllow(a, b, c, m.delta))
	}

	if !wall {
		return
	}
	// sb-sa and sc-sa are b-a and c-a exactly: the anchor cancels over
	// rationals, so the already-lifted vertices serve the area bracket too.
	lo, hi := loftmesh.WallTriangleArea(proofbound.Xsub(sb, sa), proofbound.Xsub(sc, sa))
	m.wallAreaSum += lo
	m.wallAreaAbs = proofbound.UpRound(m.wallAreaAbs + lo)
	m.wallAreaSlack = proofbound.UpRound(m.wallAreaSlack + proofbound.UpRound(hi-lo))
	m.wallTerms++
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
// proofbound.Radius3D(coordUpper) by up to sqrt(3), since the inf-norm bound assumes
// the per-axis extremes are simultaneously achieved at one vertex, which a
// real point set rarely does. computeLoftChordedAllow reads distUpper for
// proofbound.ChordedBoundarySeamAllow's own posUpper obligation.
//
// distUpper is PROVEN by exact rational arithmetic, the SAME mechanism
// computeLoftChordedAllow's own h1Upper reading already uses
// (ratSquaredDistance3/proofbound.RatSqrtUp): p and anchorF are both float64, hence
// both exact rationals, so ratSquaredDistance3 is the true squared distance
// with no rounding of its own, and proofbound.RatSqrtUp brackets its root by exact
// comparison, proven whatever the platform's own sqrt does. An earlier
// version of this function instead nudged r3.Vec.Len()'s own float64
// result outward by a single proofbound.UpRound — one ulp — which does not cover
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
		dist = proofbound.RatSqrtUp(d2)
	}
	m.distUpper = max(m.distUpper, dist)
}

// foldCoordUpperCached performs the same per-reference maxima as
// foldCoordUpper. The distance is computed when this assembled vertex index
// is first referenced, so unused vertices never affect the measurements.
func (m *loftMassAccumulator) foldCoordUpperCached(p r3.Vec, entry *loftmesh.LoftVertexDistance) {
	d := p.Sub(m.anchorF)
	m.coordUpper = max(m.coordUpper, math.Abs(d.X), math.Abs(d.Y), math.Abs(d.Z))
	if !entry.Ready {
		entry.Upper = math.Inf(1)
		if d2 := ratSquaredDistance3(m.anchorF.X, m.anchorF.Y, m.anchorF.Z, p.X, p.Y, p.Z); d2 != nil {
			entry.Upper = proofbound.RatSqrtUp(d2)
		}
		entry.Ready = true
	}
	m.distUpper = max(m.distUpper, entry.Upper)
}

// volume publishes Σvol6/6 plus the exact bilinear-patch correction, rounded
// to float64 exactly once. Its Exactness is exactnessOf the single rounding's
// proven error — Exact exactly when the published rational is representable
// in cubic millimetres, never unconditionally (docs/loft-design.md §8,
// spline design §3's Tier A rule).
//
// A placement (delta > 0, §12 PR 2a) widens that bound by
// internal/proofbound/bounds.go's proofbound.SweptVolumeAllow(delta, areaUpper), areaUpper the SAME
// whole-mesh proofbound.PerturbedAreaUpper the identity fast path never reaches — this
// is the term that closes the measured 1.82e-12 gap a naive re-lift-and-round
// implementation misses: every held vertex is exact ONLY under the identity
// transform, and a general rigid motion rounds inside its own products and
// sums.
func (m *loftMassAccumulator) volume(verts []r3.Vec, tris [][3]int) Measurement {
	vol := new(big.Rat).Quo(m.vol6, big.NewRat(6, 1))
	if m.chorded.TwistVolumeCorrection != nil {
		vol.Add(vol, m.chorded.TwistVolumeCorrection)
	}
	value, _ := vol.Float64()
	bound := proofarith.RationalFloatError(vol, value)
	if m.delta > 0 {
		areaUpper := proofbound.PerturbedAreaUpper(verts, tris, m.delta)
		bound = proofbound.AbsSumUpper(bound, proofbound.SweptVolumeAllow(m.delta, areaUpper))
	}
	if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		bound = proofbound.AbsSumUpper(bound, proofbound.ChordedBoundaryVolumeResidualAllow(
			m.sectionMatchedDelta, m.chorded.WallAreaUpper,
			m.chorded.CapVolumeUpper, m.chorded.SeamAllow,
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
// moments. Each coordinate rounds once. Its bound is proofbound.Radius3D of the largest
// per-coordinate rounding error, and a loft with zero corrected volume has no
// centroid.
//
// A placement (delta > 0) and a curved pairing (sectionDelta > 0 or
// sectionMatchedDelta > 0) each widen one COMBINED volume allowance epsV and
// one COMBINED first-moment allowance epsM before either is spent: delta's
// own leg is proofbound.SweptVolumeAllow/proofbound.SweptMomentAllow, and the curved pairing's leg
// is proofbound.ChordedBoundaryVolumeResidualAllow/proofbound.ChordedBoundaryMomentResidualAllow,
// both composed
// from sectionMatchedDelta — the PARAMETER-MATCHED quantity those two
// helpers' own doc comments oblige, never sectionDelta, whose SET-distance
// sagitta is spent on Bounds instead. The two legs are mechanically distinct (a vertex displaced versus
// a boundary replaced by a nearby non-mesh surface, docs/loft-design.md §5),
// but each publishes a volume and a first-moment allowance over the SAME
// anchored accumulator, so ONE clearance test and ONE loftmesh.PlacedCentroidAllow
// quotient composition — moments.go's proofbound.BoundedQuotient formula, specialized to
// whichever allowances are active — cover both. A non-positive clearance (the
// combined volume allowance is not smaller than the held volume) leaves the
// quotient's denominator with nothing left to divide by, so the centroid is
// unstateable — refused ErrUnsupported (Table S, S12) rather than published
// with a bound nobody could use. This gate is reachable on an UNPLACED body
// under a curved pairing alone (a10-plan.md Part 3 PR 6): delta's own fast
// path (delta == 0) does not imply epsV == 0, and EITHER section quantity on
// its own reaches it, since a free-form cell can carry a positive
// matchedDelta at an exactly-zero sagitta (internal/freeform/spline_sagitta.go's own
// counterexample).
func (m *loftMassAccumulator) centroid(verts []r3.Vec, tris [][3]int) (VecMeasurement, error) {
	vol6 := new(big.Rat).Set(m.vol6)
	momX := new(big.Rat).Set(m.momX)
	momY := new(big.Rat).Set(m.momY)
	momZ := new(big.Rat).Set(m.momZ)
	if m.chorded.TwistVolumeCorrection != nil {
		vol6.Add(vol6, new(big.Rat).Mul(big.NewRat(6, 1), m.chorded.TwistVolumeCorrection))
		momX.Add(momX, m.chorded.TwistMomentCorrection[0])
		momY.Add(momY, m.chorded.TwistMomentCorrection[1])
		momZ.Add(momZ, m.chorded.TwistMomentCorrection[2])
	}
	if vol6.Sign() == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a loft with zero net volume has no centroid`, ErrDegenerate)
	}
	denom := new(big.Rat).Mul(big.NewRat(4, 1), vol6)
	anchorX, anchorY, anchorZ := meshbool.XhpRat(proofbound.Xhp(m.anchor))
	cx := new(big.Rat).Add(anchorX, new(big.Rat).Quo(momX, denom))
	cy := new(big.Rat).Add(anchorY, new(big.Rat).Quo(momY, denom))
	cz := new(big.Rat).Add(anchorZ, new(big.Rat).Quo(momZ, denom))

	fx, _ := cx.Float64()
	fy, _ := cy.Float64()
	fz, _ := cz.Float64()
	bx := proofarith.RationalFloatError(cx, fx)
	by := proofarith.RationalFloatError(cy, fy)
	bz := proofarith.RationalFloatError(cz, fz)

	if m.delta > 0 || m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		vol := new(big.Rat).Quo(vol6, big.NewRat(6, 1))
		volValue, _ := vol.Float64()

		epsV, epsM := 0.0, 0.0
		if m.delta > 0 {
			areaUpper := proofbound.PerturbedAreaUpper(verts, tris, m.delta)
			epsV = proofbound.AbsSumUpper(epsV, proofbound.SweptVolumeAllow(m.delta, areaUpper))
			epsM = proofbound.AbsSumUpper(epsM, proofbound.SweptMomentAllow(m.delta, areaUpper, m.coordUpper+m.delta))
		}
		if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
			epsV = proofbound.AbsSumUpper(epsV, proofbound.ChordedBoundaryVolumeResidualAllow(
				m.sectionMatchedDelta, m.chorded.WallAreaUpper,
				m.chorded.CapVolumeUpper, m.chorded.SeamAllow,
			))
			epsM = proofbound.AbsSumUpper(epsM, proofbound.ChordedBoundaryMomentResidualAllow(
				m.sectionMatchedDelta, m.chorded.WallAreaUpper,
				m.chorded.CapVolumeUpper, m.chorded.SeamAllow, m.chorded.MaxTwistOffsetUpper, m.coordUpper,
			))
		}

		clearance := math.Nextafter(math.Abs(volValue)-epsV, math.Inf(-1))
		if clearance <= 0 {
			return VecMeasurement{}, fmt.Errorf(`%w: the placement and section proven volume allowance is not smaller than the held volume; this evaluator cannot state the placed centroid`, ErrUnsupported)
		}
		bx = proofbound.AbsSumUpper(bx, loftmesh.PlacedCentroidAllow(fx-m.anchorF.X, epsM, epsV, clearance))
		by = proofbound.AbsSumUpper(by, loftmesh.PlacedCentroidAllow(fy-m.anchorF.Y, epsM, epsV, clearance))
		bz = proofbound.AbsSumUpper(bz, loftmesh.PlacedCentroidAllow(fz-m.anchorF.Z, epsM, epsV, clearance))
	}

	bound := proofbound.Radius3D(math.Max(bx, math.Max(by, bz)))

	return VecMeasurement{
		Value:     r3.NewVec(fx, fy, fz),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// bounds publishes the componentwise min/max over every held vertex. Bound
// is proofbound.AbsSumUpper(delta, sectionDelta) — NON-NEGOTIABLE (a10-plan.md Part 3
// PR 6): a chorded curved section's TRUE curve bulges OUTSIDE the station
// polygon this box is taken over, so a box carrying only delta UNDERSTATES
// the true box, and Verify's box-disjointness (Table D row D3) reads Bounds
// to prove pairs disjoint — understating it is unsound in the one direction
// that matters. For an unplaced LineSeg-only body both terms are exactly
// zero, proofbound.AbsSumUpper(0, 0) is exactly 0.0 (proofbound.UpRound never nudges a
// non-positive value), and Exactness stays Exact — bit-identical to before
// this field existed. The second return is false only when add has never
// been called.
func (m *loftMassAccumulator) bounds() (Box, bool) {
	if !m.haveBounds {
		return Box{}, false
	}
	bound := proofbound.AbsSumUpper(m.delta, m.sectionDelta)
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
// says only that the bound arithmetic ran out of scale to state (proofbound.SumSlop
// underflowing on a subnormal wall triangle, a saturated wallAreaAbs), which
// is a fact about the proof term and not about the value.
//
// Bound is proven independently, and every one of its base four terms is
// charged at the magnitude where its own rounding happens:
//
//   - wallAreaSlack — Σ over the triangles of the exact-rational cross-norm
//     bracket's own width, so each triangle's area is charged at ITS OWN
//     scale, never at the held total's;
//   - proofbound.SumSlop over wallAreaAbs — the summation loop that added those terms;
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
	capBound := proofarith.RationalFloatError(capTotal, capFloat)

	wallValue := m.wallAreaSum
	wallBound := m.wallBound()
	if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		corrected := wallValue + m.chorded.AreaCorrection
		wallBound = proofbound.AbsSumUpper(
			wallBound,
			m.chorded.AreaCorrectionBound,
			m.chorded.BilinearAreaBound,
			proofarith.AddRoundError(wallValue, m.chorded.AreaCorrection, corrected),
		)
		wallValue = corrected
	}
	value := wallValue + capFloat
	addBound := proofarith.AddRoundError(wallValue, capFloat, value)
	bound := proofbound.AbsSumUpper(wallBound, capBound, addBound)
	// The per-triangle allowance covers both directions at once: the base wall
	// accumulator is over held triangles and the cap term is the denoted region's
	// exact rational, and the two differ by at most this sum
	// (docs/loft-design.md §12 PR 2a). That TRIANGLE-level role is the whole
	// of what it is proven for. The wall's own SURFACE step from the held
	// corners to the stations they denote is a different quantity of the same
	// shape, charged by proofbound.CellStationShiftAreaAllow inside areaExcess below;
	// proofbound.CellChordCurveAreaAllow's own composition section owns that
	// split. The gate stays delta > 0, and an unplaced LineSeg-only loft's
	// Area stays bit-identical to PR 1's.
	if m.delta > 0 {
		bound = proofbound.AbsSumUpper(bound, m.perturbAreaSum)
	}
	// A curved pairing's own two-leg wall residual PLUS its own cap
	// chord-versus-curve excess (a10-plan.md Part 3 PR 6,
	// computeLoftChordedAllow's own doc comment): the corrected wall value above
	// uses bilinear patches, and the true wall surface a circular cell denotes
	// differs by at most areaExcess; capFloat
	// above is capPolygonAreaRat, the built polygon's own exact rational, and
	// the region the loft's construction actually denotes is the CURVED
	// region proofbound.SectionDisplacementArea bounds the gap to on EITHER cap
	// (capAreaExcess) — the identical gap capVolumeUpper folds into the
	// Volume leg via a plane-offset division that Area, having no such
	// offset, spends unfolded. Both are gated on sectionDelta > 0 OR
	// sectionMatchedDelta > 0 — never sectionDelta alone — so a free-form
	// cell whose matchedDelta is positive at an exactly-zero sagitta
	// (internal/freeform/spline_sagitta.go's own counterexample) still has its wall and cap
	// excess charged; an unplaced LineSeg-only loft, where both are exactly
	// 0, stays bit-identical to PR 1's.
	if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		bound = proofbound.AbsSumUpper(bound, m.chorded.AreaExcess, m.chorded.CapAreaExcess)
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
// triangles away. proofbound.SumSlop answers a non-finite absSum with 0 — correct for a
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
	if proofbound.IsNonFinite(m.wallAreaAbs) || proofbound.IsNonFinite(m.wallAreaSlack) {
		return math.Inf(1)
	}
	return proofbound.AbsSumUpper(m.wallAreaSlack, proofbound.SumSlop(m.wallTerms, m.wallAreaAbs))
}

// computeLoftChordedAllow maps built loft cells to the neutral chord proof.
func computeLoftChordedAllow(pairs []loftLoopPair, vIdx, wIdx [][]int, verts []r3.Vec, anchor r3.Vec, matchedDelta, delta, distUpper float64, reversed bool) (loftmesh.LoftChordedAllow, error) {
	// Derive cap1's offset before lifting any wall cell into exact rationals.
	// Production has already refused non-finite vertices at S13, while direct
	// internal callers still receive S14's existing derivation refusal instead
	// of reaching freeform.MustRatOf with a NaN.
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
	// an assembly whose every cap1 vertex overflows proofbound.RatSqrtUp leaves the
	// minimum at +Inf, as does an assembly stating no cap1 vertex. §5.2's own
	// closing rule — an enclosure the record cannot state answers +Inf and the
	// build refuses at Table S row S14, "never a finite substitute and never a
	// published zero" — is what forbids the obvious alternative of assigning
	// h1Upper = 0 here. That zero is not a bound: proofbound.CapAreaVolumeAllow takes its
	// planeOffsetUpper <= 0 arm on it and publishes capVolumeUpper = 0, the
	// SMALLEST possible number standing in for a quantity this evaluator could
	// not derive, in a term every consumer reads as an upper bound. Nothing
	// about the surrounding legs is allowed to excuse it: whether a sibling leg
	// happens to saturate on the same assembly is that leg's own business, and
	// a bound may not rest on another term's value to stay sound.
	h1Upper := math.Inf(1)
	for _, row := range wIdx {
		for _, idx := range row {
			v := verts[idx]
			d2 := ratSquaredDistance3(anchor.X, anchor.Y, anchor.Z, v.X, v.Y, v.Z)
			if d2 == nil {
				return loftmesh.LoftChordedAllow{}, loftmesh.ErrLoftCapOffsetUnderivable
			}
			h1Upper = math.Min(h1Upper, proofbound.RatSqrtUp(d2))
		}
	}
	if proofbound.IsNonFinite(h1Upper) {
		return loftmesh.LoftChordedAllow{}, loftmesh.ErrLoftCapOffsetUnderivable
	}

	neutral := make([]loftmesh.LoftChordPair, len(pairs))
	for i, p := range pairs {
		neutral[i] = loftmesh.LoftChordPair{
			Cells: len(p.v), ArcUpperV: p.arcUpperV, ArcUpperW: p.arcUpperW,
			MatchedDelta: p.matchedDelta, TangentEnergyV: p.tangentEnergyV,
			TangentEnergyW: p.tangentEnergyW,
		}
	}
	return loftmesh.ComputeLoftChordedAllow(
		neutral, vIdx, wIdx, verts, anchor, h1Upper, matchedDelta, delta, distUpper, reversed,
	), nil
}
