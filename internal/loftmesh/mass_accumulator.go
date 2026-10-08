package loftmesh

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// MassAccumulator is docs/loft-design.md §8's tetrahedron-sum kernel. It
// streams one outward-oriented triangle of T at a time — a wall or a cap
// triangulation triangle — anchored at anchor (p0's own PlaneRecord.Origin).
// Volume, Centroid and Bounds fold every triangle handed to Add; Area's wall
// contribution folds only the triangles whose wall flag is true, since a
// cap's own contribution is the exact rational shoelace area of the SAME
// polygon its triangulation was built from (loft_build.go's
// capPolygonAreaRat), never the sum of its triangulation's own float areas.
type MassAccumulator struct {
	anchor  proofarith.Xpt
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
	// proofbound.ChordedBoundaryVolumeResidualAllow/proofbound.ChordedBoundarySeamAllow's)
	// own matchedDeltaUpper obligation, nor CentroidRadius's reach — a
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
	// proofbound.ChordedBoundaryVolumeResidualAllow and
	// proofbound.ChordedBoundarySeamAllow's own doc comments — reads this field, never
	// sectionDelta, and so does CentroidRadius. It stays exactly 0 on a build with no chorded cell at all
	// (a LineSeg-only pairing), whose held triangle pair IS the boundary §5
	// gives it and whose vertex displacement the delta-keyed legs above
	// already charge.
	sectionMatchedDelta float64

	// chorded holds the corrections and residuals computeLoftChordedAllow derives from the
	// composed sectionMatchedDelta and the delta above (loft_build.go): every
	// field stays its zero value unless evalLoft calls computeLoftChordedAllow,
	// which it does exactly when the build holds a cell that is not faceted
	// (a circular or free-form cell) or a positive section term. A degree-1
	// free-form build reaches it with both section terms zero, so the readings
	// below key the bilinear corrections on the computed correction itself.
	Chorded LoftChordedAllow

	// vol6 is Σ (A-anchor)·((B-anchor)×(C-anchor)) over every triangle of T:
	// six times the signed volume (docs/loft-design.md §8).
	Vol6 *big.Rat
	// momX/momY/momZ are Σ vol6_tri · Σ(A-anchor, B-anchor, C-anchor), the
	// first-moment accumulator the centroid divides by 4·vol6.
	MomX, MomY, MomZ *big.Rat

	haveBounds bool
	lo, hi     r3.Vec // componentwise extremes over every held vertex

	// DistUpper is the max |v-anchor| (3D distance) over every held vertex
	// (a10-plan.md Part 3 PR 6), computeLoftChordedAllow's own posUpper
	// reading.
	DistUpper float64

	// perturbAreaSum is Σ proofbound.PerturbedTriangleAreaAllow(...) over EVERY triangle
	// of T — walls and caps alike — the extra area the payload's own delta
	// can add on top of the held triangle areas area() already sums. That
	// per-TRIANGLE role is all of it: a chorded wall cell's held-to-denoted
	// SURFACE step is proofbound.CellStationShiftAreaAllow's leg of areaExcess instead,
	// area()'s own composition site and proofbound.CellChordCurveAreaAllow's (internal/proofbound/bounds.go)
	// owning the split. It stays exactly 0 when delta is 0.
	PerturbAreaSum float64

	// wallAreaSum is the naive float sum of the per-triangle PROVEN LOWER
	// bounds WallTriangleArea returns; wallAreaAbs is an upper bound on
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
	WallAreaSum   float64
	WallAreaAbs   float64
	WallAreaSlack float64
	WallTerms     int // term count proofbound.SumSlop needs to bound that sum
}

// NewMassAccumulator opens a fresh accumulator anchored at the loft's
// first profile plane origin (docs/loft-design.md §8), carrying the
// payload's own proven placement displacement delta (§12 PR 2a), the
// SET-distance sagitta sectionDelta, and §5.2's own composed PARAMETER-MATCHED
// term sectionMatchedDelta (a10-plan.md Part 3 PR 6/PR 9) — all three zero for
// an unplaced LineSeg-only body.
func NewMassAccumulator(anchor r3.Vec, delta, sectionDelta, sectionMatchedDelta float64) *MassAccumulator {
	return &MassAccumulator{
		anchor:              proofarith.XptOf(anchor),
		anchorF:             anchor,
		delta:               delta,
		sectionDelta:        sectionDelta,
		sectionMatchedDelta: sectionMatchedDelta,
		Vol6:                new(big.Rat),
		MomX:                new(big.Rat),
		MomY:                new(big.Rat),
		MomZ:                new(big.Rat),
	}
}

// Add folds one outward-oriented triangle (A, B, C) of T into the volume,
// centroid and bounds accumulators, and — when wall is true — into the area
// accumulator's float sum and that sum's two proof terms. Every vertex
// coordinate is a float64, hence an exact rational (internal/polynomial/rat_poly.go's
// take-the-floats-exactly discipline); the volume and centroid sums round
// nothing until publication, and the area sum's own terms are the endpoints
// of a proven per-triangle enclosure rather than a float evaluation.
func (m *MassAccumulator) Add(a, b, c r3.Vec, wall bool) {
	m.AddTriangle(a, b, c, wall, [3]int{}, nil)
}

// AddTriangle keeps Add's triangle fold unchanged. Only evalLoft supplies
// indices and a cache, so repeated references to one assembled vertex reuse
// its exact Euclidean upper distance.
func (m *MassAccumulator) AddTriangle(a, b, c r3.Vec, wall bool, indices [3]int, distances []LoftVertexDistance) {
	sa := proofarith.Xsub(proofarith.XptOf(a), m.anchor)
	sb := proofarith.Xsub(proofarith.XptOf(b), m.anchor)
	sc := proofarith.Xsub(proofarith.XptOf(c), m.anchor)

	triVol6 := proofarith.XdotRat(sa, proofarith.Xcross(sb, sc))
	m.Vol6.Add(m.Vol6, triVol6)

	saX, saY, saZ := proofarith.XhpRat(proofarith.Xhp(sa))
	sbX, sbY, sbZ := proofarith.XhpRat(proofarith.Xhp(sb))
	scX, scY, scZ := proofarith.XhpRat(proofarith.Xhp(sc))
	sumX := proofbound.RatAdd(saX, sbX, scX)
	sumY := proofbound.RatAdd(saY, sbY, scY)
	sumZ := proofbound.RatAdd(saZ, sbZ, scZ)
	m.MomX.Add(m.MomX, new(big.Rat).Mul(triVol6, sumX))
	m.MomY.Add(m.MomY, new(big.Rat).Mul(triVol6, sumY))
	m.MomZ.Add(m.MomZ, new(big.Rat).Mul(triVol6, sumZ))

	m.foldBounds(a)
	m.foldBounds(b)
	m.foldBounds(c)
	if distances == nil {
		m.FoldCoordUpper(a)
		m.FoldCoordUpper(b)
		m.FoldCoordUpper(c)
	} else {
		m.FoldCoordUpperCached(a, &distances[indices[0]])
		m.FoldCoordUpperCached(b, &distances[indices[1]])
		m.FoldCoordUpperCached(c, &distances[indices[2]])
	}

	if m.delta > 0 {
		m.PerturbAreaSum = proofbound.UpRound(m.PerturbAreaSum + proofbound.PerturbedTriangleAreaAllow(a, b, c, m.delta))
	}

	if !wall {
		return
	}
	// sb-sa and sc-sa are b-a and c-a exactly: the anchor cancels over
	// rationals, so the already-lifted vertices serve the area bracket too.
	lo, hi := WallTriangleArea(proofarith.Xsub(sb, sa), proofarith.Xsub(sc, sa))
	m.WallAreaSum += lo
	m.WallAreaAbs = proofbound.UpRound(m.WallAreaAbs + lo)
	m.WallAreaSlack = proofbound.UpRound(m.WallAreaSlack + proofbound.UpRound(hi-lo))
	m.WallTerms++
}

// foldBounds extends the componentwise extreme box over one held vertex.
// Comparing held coordinates introduces no rounding of its own, so the box is
// exactly as good as the vertex set it is taken over: exact on an unplaced
// body (docs/loft-design.md §5) and within delta of the true extreme on a
// placed one, which is what bounds() publishes.
func (m *MassAccumulator) foldBounds(p r3.Vec) {
	if !m.haveBounds {
		m.lo, m.hi = p, p
		m.haveBounds = true
		return
	}
	m.lo = r3.Vec{X: math.Min(m.lo.X, p.X), Y: math.Min(m.lo.Y, p.Y), Z: math.Min(m.lo.Z, p.Z)}
	m.hi = r3.Vec{X: math.Max(m.hi.X, p.X), Y: math.Max(m.hi.Y, p.Y), Z: math.Max(m.hi.Z, p.Z)}
}

// FoldCoordUpper extends DistUpper (a10-plan.md Part 3 PR 6) over one held
// vertex's own EUCLIDEAN distance from anchor. computeLoftChordedAllow reads
// DistUpper for proofbound.ChordedBoundarySeamAllow's own posUpper obligation.
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
func (m *MassAccumulator) FoldCoordUpper(p r3.Vec) {
	dist := math.Inf(1)
	if d2 := proofarith.RatSquaredDistance3(m.anchorF.X, m.anchorF.Y, m.anchorF.Z, p.X, p.Y, p.Z); d2 != nil {
		dist = proofbound.RatSqrtUp(d2)
	}
	m.DistUpper = max(m.DistUpper, dist)
}

// FoldCoordUpperCached performs the same per-reference maxima as
// FoldCoordUpper. The distance is computed when this assembled vertex index
// is first referenced, so unused vertices never affect the measurements.
func (m *MassAccumulator) FoldCoordUpperCached(p r3.Vec, entry *LoftVertexDistance) {
	if !entry.Ready {
		entry.Upper = math.Inf(1)
		if d2 := proofarith.RatSquaredDistance3(m.anchorF.X, m.anchorF.Y, m.anchorF.Z, p.X, p.Y, p.Z); d2 != nil {
			entry.Upper = proofbound.RatSqrtUp(d2)
		}
		entry.Ready = true
	}
	m.DistUpper = max(m.DistUpper, entry.Upper)
}

// Volume returns Σvol6/6 plus the exact bilinear-patch correction, rounded
// to float64 exactly once. Its caller derives Exactness from the single rounding's
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
func (m *MassAccumulator) Volume(verts []r3.Vec, tris [][3]int) (float64, float64) {
	vol := m.correctedVolume()
	value, _ := vol.Float64()
	bound := proofarith.RationalFloatError(vol, value)
	// An unplaced LineSeg-only body has no allowance, and its bound stays the
	// single rounding exactly; a NaN allowance still reaches the sum.
	if allow := m.VolumeAllow(verts, tris); allow != 0 {
		bound = proofbound.AbsSumUpper(bound, allow)
	}
	return value, bound
}

// VolumeAllow is Volume's proven allowance for the gap between the exact
// corrected volume and the true one, without the value's own rounding: the
// placement's swept term at delta > 0 and the chorded residual at a positive
// sectionDelta or sectionMatchedDelta. Centroid's clearance subtracts it from
// the exact rational volume.
func (m *MassAccumulator) VolumeAllow(verts []r3.Vec, tris [][3]int) float64 {
	allow := 0.0
	if m.delta > 0 {
		areaUpper := proofbound.PerturbedAreaUpper(verts, tris, m.delta)
		allow = proofbound.SweptVolumeAllow(m.delta, areaUpper)
	}
	if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 {
		allow = proofbound.AbsSumUpper(allow, proofbound.ChordedBoundaryVolumeResidualAllow(
			m.sectionMatchedDelta, m.Chorded.WallAreaUpper,
			m.Chorded.CapVolumeUpper, m.Chorded.SeamAllow,
		))
	}
	return allow
}

// correctedVolume is the exact rational volume Volume rounds: the held
// tetrahedron sum plus the exact twist correction.
func (m *MassAccumulator) correctedVolume() *big.Rat {
	vol := new(big.Rat).Quo(m.Vol6, big.NewRat(6, 1))
	if m.Chorded.TwistVolumeCorrection != nil {
		vol.Add(vol, m.Chorded.TwistVolumeCorrection)
	}
	return vol
}

// CentroidClearance is a proven lower bound on the true volume's magnitude:
// |vol| − allow evaluated over exact rationals and rounded DOWN once. A
// float subtraction from the rounded volume would carry that rounding into
// the result, and where allow is most of |vol| the volume's half-ulp is many
// ulps of the small difference, so no single outward step covers it. A
// non-finite allow states no bound and answers 0, which Centroid refuses.
func CentroidClearance(vol *big.Rat, allow float64) float64 {
	if proofbound.IsNonFinite(allow) || allow < 0 {
		return 0
	}
	gap := new(big.Rat).Abs(vol)
	gap.Sub(gap, new(big.Rat).SetFloat64(allow))
	if gap.Sign() <= 0 {
		return 0
	}
	f, _ := gap.Float64()
	if new(big.Rat).SetFloat64(f).Cmp(gap) > 0 {
		f = math.Nextafter(f, math.Inf(-1))
	}
	return f
}

// Centroid returns anchor + Σmoment/(4·Σvol6) after
// applying the exact bilinear-patch corrections to both the volume and first
// moments. Each coordinate rounds once. Its bound is proofbound.Radius3D of the largest
// per-coordinate rounding error, and a loft with zero corrected volume has no
// centroid.
//
// A placement (delta > 0) or a curved pairing (sectionDelta > 0 or
// sectionMatchedDelta > 0) widens that bound by one SHIFT of the held
// centroid, docs/loft-gear-bounds-design.md §3's
//
//	shift = epsV · R_c / clearance
//
// epsV (CentroidMeasureAllow) bounds the measure of the region where the
// true solid and the corrected held body differ, and R_c (CentroidRadius)
// bounds how far any point of that region lies from the published centroid.
// The true centroid is the held one plus the first moment of that difference
// about it divided by the true volume, so it moves at most epsV · R_c over a
// proven lower bound on that volume. clearance is that lower bound
// (CentroidClearance): the exact corrected volume minus VolumeAllow, the
// allowance Volume already proves for the true volume, rounded down once.
// R_c is measured from the published centroid, so
// the shift scales with the body's own size and never with its distance from
// the mass anchor.
//
// A non-positive clearance (Volume's allowance is not smaller than the held
// volume) leaves the shift nothing to divide by, so the centroid is
// unstateable — refused decaderr.ErrUnsupported (Table S, S12) rather than
// published with a bound nobody could use. This gate is reachable on an
// UNPLACED body under a curved pairing alone (a10-plan.md Part 3 PR 6): either
// section quantity on its own reaches it, since a free-form cell can carry a
// positive matchedDelta at an exactly-zero sagitta
// (internal/freeform/spline_sagitta.go's own counterexample).
func (m *MassAccumulator) Centroid(verts []r3.Vec, tris [][3]int) (r3.Vec, float64, error) {
	vol6 := new(big.Rat).Set(m.Vol6)
	momX := new(big.Rat).Set(m.MomX)
	momY := new(big.Rat).Set(m.MomY)
	momZ := new(big.Rat).Set(m.MomZ)
	if m.Chorded.TwistVolumeCorrection != nil {
		vol6.Add(vol6, new(big.Rat).Mul(big.NewRat(6, 1), m.Chorded.TwistVolumeCorrection))
		momX.Add(momX, m.Chorded.TwistMomentCorrection[0])
		momY.Add(momY, m.Chorded.TwistMomentCorrection[1])
		momZ.Add(momZ, m.Chorded.TwistMomentCorrection[2])
	}
	if vol6.Sign() == 0 {
		return r3.Vec{}, 0, fmt.Errorf(`%w: a loft with zero net volume has no centroid`, decaderr.ErrDegenerate)
	}
	denom := new(big.Rat).Mul(big.NewRat(4, 1), vol6)
	anchorX, anchorY, anchorZ := proofarith.XhpRat(proofarith.Xhp(m.anchor))
	cx := new(big.Rat).Add(anchorX, new(big.Rat).Quo(momX, denom))
	cy := new(big.Rat).Add(anchorY, new(big.Rat).Quo(momY, denom))
	cz := new(big.Rat).Add(anchorZ, new(big.Rat).Quo(momZ, denom))

	fx, _ := cx.Float64()
	fy, _ := cy.Float64()
	fz, _ := cz.Float64()
	bx := proofarith.RationalFloatError(cx, fx)
	by := proofarith.RationalFloatError(cy, fy)
	bz := proofarith.RationalFloatError(cz, fz)
	published := r3.NewVec(fx, fy, fz)
	rounding := proofbound.Radius3D(math.Max(bx, math.Max(by, bz)))

	if m.delta <= 0 && m.sectionDelta <= 0 && m.sectionMatchedDelta <= 0 {
		return published, rounding, nil
	}

	// The clearance test (S12) runs before any shift term is read, so a body
	// whose volume enclosure reaches zero refuses whatever its other terms are.
	clearance := CentroidClearance(m.correctedVolume(), m.VolumeAllow(verts, tris))
	if !(clearance > 0) {
		return r3.Vec{}, 0, fmt.Errorf(`%w: the placement and section proven volume allowance is not smaller than the held volume; this evaluator cannot state the placed centroid`, decaderr.ErrUnsupported)
	}
	epsV := m.CentroidMeasureAllow(verts, tris)
	if epsV == 0 {
		return published, rounding, nil
	}
	radius := m.CentroidRadius(verts, tris, published, rounding)
	shift := proofbound.DivUpper(proofbound.ProductUpper(epsV, radius), clearance)
	return published, proofbound.AbsSumUpper(rounding, shift), nil
}

// CentroidMeasureAllow is Centroid's epsV (docs/loft-gear-bounds-design.md
// §3): a proven upper bound on the measure of the region where the true solid
// and the corrected held body differ. That region is swept in three steps, and
// each has its charge:
//
//   - the held caps, each within delta of its cap plane, project onto the
//     plane, and every cell whose chord departure is zero moves to its exact
//     place, both at speed at most delta over held triangles:
//     proofbound.SweptVolumeAllow over the perturbed area, zero at delta == 0;
//   - every chorded cell moves from its ruled patch to the true surface:
//     LoftChordedAllow.WallLeg, each cell at its own matched departure;
//   - the skirt joining every held seam cell to the cap plane moves with it:
//     LoftChordedAllow.SkirtLeg, zero at delta == 0.
//
// The cap and seam volume legs are signed identities that sweep no material,
// so neither enters it.
func (m *MassAccumulator) CentroidMeasureAllow(verts []r3.Vec, tris [][3]int) float64 {
	epsV := 0.0
	if m.delta > 0 {
		epsV = proofbound.SweptVolumeAllow(m.delta, proofbound.PerturbedAreaUpper(verts, tris, m.delta))
	}
	return proofbound.AbsSumUpper(epsV, m.Chorded.WallLeg, m.Chorded.SkirtLeg)
}

// CentroidRadius is Centroid's R_c (docs/loft-gear-bounds-design.md §3): a
// proven upper bound on the distance from the published centroid to any point
// of the region CentroidMeasureAllow measures. A wall-sweep point lies within
// its cell's matched departure of a bilinear patch, which lies in the convex
// hull of its four held corners; a vertex-sweep or skirt point lies within
// delta of a held triangle or seam. So every such point lies within
// max(sectionMatchedDelta, delta) of the held vertex hull, and the farthest
// vertex tris references, measured from the published centroid, bounds that
// hull's reach; a vertex no triangle references is no part of the body.
// rounding is the published centroid's own distance bound from the exact one,
// which the derivation measures from.
//
// Each vertex distance is exact: both points are float64, so the squared
// distance is an exact dyadic and proofarith.DySqrtUp brackets its root by
// exact comparison. A coordinate the dyadic lift cannot read (non-finite)
// answers +Inf rather than dropping the vertex.
func (m *MassAccumulator) CentroidRadius(verts []r3.Vec, tris [][3]int, centroid r3.Vec, rounding float64) float64 {
	farthest := 0.0
	seen := make([]bool, len(verts))
	for _, tri := range tris {
		for _, idx := range tri {
			if seen[idx] {
				continue
			}
			seen[idx] = true
			v := verts[idx]
			d2, ok := proofarith.DySquaredDistance3(centroid.X, centroid.Y, centroid.Z, v.X, v.Y, v.Z)
			if !ok {
				return math.Inf(1)
			}
			farthest = math.Max(farthest, proofarith.DySqrtUp(d2))
		}
	}
	return proofbound.AbsSumUpper(farthest, rounding, math.Max(m.sectionMatchedDelta, m.delta))
}

// Bounds returns the componentwise min/max over every held vertex. Bound
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
func (m *MassAccumulator) Bounds() (r3.Vec, r3.Vec, float64, bool) {
	if !m.haveBounds {
		return r3.Vec{}, r3.Vec{}, 0, false
	}
	bound := proofbound.AbsSumUpper(m.delta, m.sectionDelta)
	return m.lo, m.hi, bound, true
}

// Area returns the two caps' exact rational shoelace areas plus a mixed wall
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
// since neither is a proven scale any more. A chorded build (a computed
// correction, or a positive section term) adds the bilinear integration
// enclosure, and a positive section term adds computeLoftChordedAllow's own
// two-leg wall residual and capAreaExcess (the SAME cap
// chord-versus-curve gap capVolumeUpper folds into Volume, spent here as an
// area rather than a volume) — both documented at the composition below. A
// displaced build (delta > 0) adds perturbAreaSum, the held triangles' and the
// caps' own per-triangle placement allowance; the wall's own held-to-denoted
// SURFACE step is a leg of areaExcess, not of that sum, and the composition
// site below owns the split.
func (m *MassAccumulator) Area(capAreas ...*big.Rat) (float64, float64) {
	capTotal := new(big.Rat)
	for _, ca := range capAreas {
		if ca != nil {
			capTotal.Add(capTotal, ca)
		}
	}
	capFloat, _ := capTotal.Float64()
	capBound := proofarith.RationalFloatError(capTotal, capFloat)

	wallValue := m.WallAreaSum
	wallBound := m.wallBound()
	if m.sectionDelta > 0 || m.sectionMatchedDelta > 0 || m.Chorded.TwistVolumeCorrection != nil {
		corrected := wallValue + m.Chorded.AreaCorrection
		wallBound = proofbound.AbsSumUpper(
			wallBound,
			m.Chorded.AreaCorrectionBound,
			m.Chorded.BilinearAreaBound,
			proofarith.AddRoundError(wallValue, m.Chorded.AreaCorrection, corrected),
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
		bound = proofbound.AbsSumUpper(bound, m.PerturbAreaSum)
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
		bound = proofbound.AbsSumUpper(bound, m.Chorded.AreaExcess, m.Chorded.CapAreaExcess)
	}

	return value, bound
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
func (m *MassAccumulator) wallBound() float64 {
	if proofbound.IsNonFinite(m.WallAreaAbs) || proofbound.IsNonFinite(m.WallAreaSlack) {
		return math.Inf(1)
	}
	return proofbound.AbsSumUpper(m.WallAreaSlack, proofbound.SumSlop(m.WallTerms, m.WallAreaAbs))
}
