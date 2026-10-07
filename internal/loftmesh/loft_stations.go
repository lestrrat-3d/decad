package loftmesh

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// LoftStation is a plane-local station held by a loft chord chain.
type LoftStation struct{ U, V float64 }

// LoftPointBounder encloses the recorded point denoted by a station parameter.
type LoftPointBounder interface {
	BoundAt(t *big.Rat, u, v float64) proofbound.WalkEndBound
}

// LoftCircularSide carries record-certified data and the resolved walk for one
// circular side. The root evaluator obtains the enclosures from the record.
type LoftCircularSide struct {
	Walk           survey2d.SegmentWalk
	Radius, Sweep  proofbound.RatInterval
	Enclosed       bool
	TStart, DT     *big.Rat
	RangeOK        bool
	EndRadialUpper float64
	Bounder        LoftPointBounder
}

// ErrLoftSagittaUnderivable is the S14 refusal for a circular chord whose
// recorded radius or sweep has no certified enclosure.
var ErrLoftSagittaUnderivable = fmt.Errorf(
	`%w: a chorded circular pair's certified per-cell sagitta has no derivation from this record`, decaderr.ErrUnsupported,
)

// ErrLoftStationDisplacementUnderivable is S14's station-displacement refusal.
var ErrLoftStationDisplacementUnderivable = fmt.Errorf(
	`%w: a loft pair's generated stations have no proven displacement from the recorded curve`, decaderr.ErrUnsupported,
)

// WalkEndPlaneDelta reads a walk endpoint's two componentwise bounds as one
// in-section-plane distance: the two components are along the section frame's
// own orthogonal U and V, so proofbound.Radius2D's √2 factor over the wider of them is an
// upper bound on the displacement's own length. An underivable component
// answers +Inf, never a small number spent in its place.
func WalkEndPlaneDelta(bound proofbound.WalkEndBound) float64 {
	if !bound.Derivable() {
		return math.Inf(1)
	}
	return proofbound.Radius2D(math.Abs(bound.U), math.Abs(bound.V))
}

// LoftLineCellStations is the LineSeg arm: one station per side, at the
// segment's own recorded start, with zero sagitta and zero matchedDelta — a
// straight wall's own chord IS the recorded segment, so there is no curve
// for it to depart from. m is fixed at 1, so this arm's STATION SET is
// bit-identical to every LineSeg pairing this evaluator built before the
// station generator existed.
//
// Its stationRoundUpper is NOT unconditionally zero, and that is the one
// thing this arm does not inherit from that earlier shape. docs/loft-design.md
// §5.2 pins a station by its own NATURAL parameter — t == 0 or t == 1 — and
// never by the kind of segment it sits on, and §5.1's Table C marks a TRIMMED
// LineSeg end GENERATED for exactly that reason. So this arm reads what its
// two walks actually prove rather than asserting a zero its kind does not
// grant.
//
// It charges each side's startBound and NEVER its endBound. Each station this
// arm emits is its own walk's START, and a cell's terminal station is the NEXT
// segment's own start — or the loop's wrap back to the first segment's — which
// that segment charges when its own cell is generated. Charging endBound here
// would charge a point this build never holds, and would double-count the
// junction the two segments share.
//
// At an UNTRIMMED start the term is exactly zero, proven rather than assumed:
// lerp2 and dyadic.go's dyLerp both special-case t == 0 and t == 1 to the
// recorded Point2 verbatim, so lineWalkEndBound's two dyRoundedFloatError calls
// measure no gap at all. An untrimmed LineSeg-only pairing therefore still
// publishes the bit-identical zero delta — and the Exact readings §8 gives it
// — that it always did.
//
// At a TRIMMED start the term is whatever lineWalkEndBound proves. walkOf
// fills such a walk's start from lerp2 in FLOAT (extrude.go's LineSeg arm),
// while the point the record denotes is the exact dyLerp, and
// lineWalkEndBound stamps the outward-rounded gap between them. That gap is a
// real displacement of a held vertex from the point the record denotes, so
// leaving it uncharged would publish a bound smaller than the displacement the
// body actually carries. Such a record is caller-reachable rather than a
// corner case: seam.go's recordEdge records a certified Partial line fragment
// over a non-natural range verbatim, and no Table S row excludes one.
//
// The refusal is DEFENSIVE and no admitted record reaches it. walkEndPlaneDelta
// answers +Inf only where lineWalkEndBound could not state the denoted point as
// a rational, and dyLerp fails solely on a non-finite coordinate — which the
// record gates exclude long before any walk is resolved. It stands so that an
// underivable term can never be published as a finite bound, which is the S14
// discipline §5.2's table states for every term in it.
func LoftLineCellStations(w0, w1 survey2d.SegmentWalk) ([]LoftStation, []LoftStation, float64, []float64, float64, error) {
	round := math.Max(WalkEndPlaneDelta(w0.StartBound), WalkEndPlaneDelta(w1.StartBound))
	if proofbound.IsNonFinite(round) {
		return nil, nil, 0, nil, 0, ErrLoftStationDisplacementUnderivable
	}
	return []LoftStation{{U: w0.StartU, V: w0.StartV}},
		[]LoftStation{{U: w1.StartU, V: w1.StartV}}, 0, []float64{0}, round, nil
}

// LoftCertifiedSagittaUpper is docs/loft-design.md §5.2's certified per-cell
// sagitta for one side of a chorded circular pair, at a candidate station
// count m: the in-section-plane distance from one chord to the recorded curve
// piece it chords, published as ONE outward rounding of an enclosure's upper
// end.
//
// The quantity is 2·r·sin²(Δθ/4m) with r the segment's radius and Δθ the angle
// its walk sweeps. Both come from circularWalkEnclosures (moments.go), which
// states them from the RECORD — an ArcSeg's proofbound.RatSqrtDown/proofbound.RatSqrtUp radius and
// its proofbound.Atan2Interval swept angle, a CircleSeg's recorded radius and exact
// rational turn — never from the walk's held math.Hypot radius and math.Atan2
// angles, neither of which the walk can enclose (extrude.go's circularWalk).
// survey2d.RadSinCosSpan supplies the sine of the enclosed cell half-angle, and the
// squaring goes through proofbound.IntervalMul, whose four-corner upper end dominates
// max x² over the span whatever the span's sign.
//
// The derivation of the form itself belongs to §5.2's table, on its per-cell
// sagitta row, and this comment restates none of it.
//
// An enclosure the record cannot state answers +Inf — the refusal that row
// assigns this term — and the caller refuses at Table S row S14 rather than
// publishing a finite substitute or a zero.
func LoftCertifiedSagittaUpper(radius, sweep proofbound.RatInterval, enclosed bool, m int) float64 {
	if m <= 0 || !enclosed {
		return math.Inf(1)
	}
	half := proofbound.IntervalScale(sweep, big.NewRat(1, 4*int64(m)))
	sin, _, ok := survey2d.RadSinCosSpan(half)
	if !ok {
		return math.Inf(1)
	}
	s := proofbound.IntervalMul(proofbound.IntervalScale(radius, big.NewRat(2, 1)), proofbound.IntervalMul(sin, sin))
	up := proofbound.RatFloatUp(s.Hi)
	if proofbound.IsNonFinite(up) {
		return math.Inf(1)
	}
	return up
}

// LoftCertifiedChordLower is a PROVEN LOWER bound on ONE uniform-angle cell's
// own true chord length at a station count m — the chord between the two points
// the RECORD denotes at the cell's own two parameters, which is
// proofbound.UniformSpeedTangentEnergyUpper's own chordLower obligation and must never
// overstate that chord.
//
// The quantity is 2·r·sin(Δθ/2m), r the segment's radius and Δθ the angle its
// walk sweeps: the same two enclosures loftCertifiedSagittaUpper reads, from the
// same owner (moments.go's circularWalkEnclosures), taken at the cell's own HALF
// angle rather than its quarter. Reading them here rather than the walk's held
// w.radius/w.th0/w.th1 is not a preference: those floats carry no enclosure
// (circularWalkEnclosures' own doc comment), a math.Atan2 endpoint's own error
// is amplified by 1/Δθ on a short arc, and the published energy DECREASES in
// this operand — so a float-derived "lower" bound that lands above the true
// chord understates the energy and every area allowance composed from it.
//
// The product runs through proofbound.IntervalMul, whose four-corner LOWER end is a bound
// on 2·r·sin over the whole enclosure and so on the true chord wherever in it
// the true radius and sweep lie. A record this bracket cannot state, or a
// bracket whose own lower end is not positive, answers 0 — a valid, if empty,
// lower bound on any chord, which costs proofbound.UniformSpeedTangentEnergyUpper its
// sharpness and never its soundness.
func LoftCertifiedChordLower(radius, sweep proofbound.RatInterval, enclosed bool, m int) float64 {
	if m <= 0 || !enclosed {
		return 0
	}
	half := proofbound.IntervalScale(sweep, big.NewRat(1, 2*int64(m)))
	sin, _, ok := survey2d.RadSinCosSpan(half)
	if !ok {
		return 0
	}
	c := proofbound.IntervalMul(proofbound.IntervalScale(radius, big.NewRat(2, 1)), sin)
	lo := proofbound.RatFloatDown(c.Lo)
	if proofbound.IsNonFinite(lo) || lo <= 0 {
		return 0
	}
	return lo
}

// LoftSettleStationCount runs docs/loft-design.md §5.1's JOINT WALK-UP for one
// same-kind circular pair and answers the count it settles on, together with
// each side's own certified per-cell sagitta AT that count.
//
// chordCount picks each side's OWN count against target independently, and
// max(m0, m1) only SEEDS the walk: from there BOTH sides' certified sagittae
// are recomputed at each candidate and the count increments until both are at
// or below the target. Settling that way needs no monotonicity claim about the
// certified sagitta as the count grows.
//
// It is a pure function of the two walks, the two RECORDED segments and the
// target, and it charges no work budget — which is what lets the record-only
// station-cap gate (loftStationCapGate, S15) settle the same m the
// construction phase will, with no station built and no triangle assembled.
//
// The two refusals it raises are the two docs/loft-design.md §4's gate-order
// paragraph assigns to this walk-up: errLoftSagittaUnderivable is S14's
// DERIVATION arm — a candidate count whose certified sagitta has no derivation
// from the record — and freeform.ErrTooManyChords is the per-walk ceiling chordCount
// itself enforces, raised again here because the certified sagitta shrinks
// with m but is floored by its own enclosure width, so a target below that
// floor would otherwise walk forever. That per-walk ceiling is NOT the station
// cap: it bounds one curve's own chording and knows nothing of how many curves
// the build holds, while loftStationCap bounds the build's station total.
func LoftSettleStationCount(a, b LoftCircularSide, target float64) (int, float64, float64, error) {
	m0, _, err := tessellation.ChordCount(a.Walk, target, tessellation.ChordWalkMin(a.Walk))
	if err != nil {
		return 0, 0, 0, err
	}
	m1, _, err := tessellation.ChordCount(b.Walk, target, tessellation.ChordWalkMin(b.Walk))
	if err != nil {
		return 0, 0, 0, err
	}
	m := max(m0, m1)
	for {
		s0 := LoftCertifiedSagittaUpper(a.Radius, a.Sweep, a.Enclosed, m)
		s1 := LoftCertifiedSagittaUpper(b.Radius, b.Sweep, b.Enclosed, m)
		if proofbound.IsNonFinite(s0) || proofbound.IsNonFinite(s1) {
			return 0, 0, 0, ErrLoftSagittaUnderivable
		}
		if s0 <= target && s1 <= target {
			return m, s0, s1, nil
		}
		if m >= freeform.MaxChordsPerWalk {
			return 0, 0, 0, freeform.ErrTooManyChords
		}
		m++
	}
}

// LoftCircularStationChain walks this segment's own uniform-angle stations of w,
// at parameter t_k = k/m — its OWN interior stations, excluding its shared end
// point (loftCellStations' own doc comment states why), so the chain holds the
// entries a loop's own count under docs/loft-design.md §7 needs from it rather
// than a count stated here.
//
// Station 0 is the WALK's own start point, never a recomputed cos/sin at th0.
// The two are not the same reading: walkOf runs pinArcWalkEnds over every arc
// walk, and arcWalkEnd restates an UNTRIMMED end as the recorded Start / End
// verbatim under a zero bound (extrude.go), so at that kind of end the walk's
// point IS a recorded coordinate while circularWalk's own cU + r·cos(th0) is a
// float that misses it. Recomputing here would displace such a station off the
// coordinate the record states — measurably, by an ulp of the coordinate — and
// would leave every reading that reads a pinned station's own published
// displacement (docs/loft-design.md §5.2's pinned station kinds) stating it of
// a vertex the build does not hold. Reading the walk keeps the pin and the station
// the same point.
//
// The chain's own terminal station is the next segment's station 0, or the
// loop's wrap back to the first segment's, so it takes the identical pin from
// that segment's own walk: a loop's segments are contiguous on the record, and
// each junction carries exactly one station.
//
// The second return is this segment's own reading of the stationRound term
// docs/loft-design.md §5.2's table lists, as an in-section-plane distance;
// that table owns the quantity it bounds, the certified source it is read from
// and its max-versus-sum rule. Only station 0 is a coordinate reading
// alone: every interior station is a math.Sincos evaluation at an angle
// composed from the walk's own held math.Atan2 / math.Hypot floats, none of
// which the walk can enclose (circularWalk's own doc comment), so the built
// point can miss the recorded curve's own point by a rounding the certified
// sagitta says nothing about. circularPointBound (extrude.go) encloses that
// recorded point from the RECORD, at the station's own EXACT rational
// parameter t_k = TStart + (k/m)·(TEnd − TStart), and proofbound.Radius2D turns the two
// componentwise gaps into the plane distance the caller's chord bound is
// stated in.
//
// The chain's own two pinned ends are charged from the walk's own
// startBound/endBound rather than recomputed: those are the same readings
// arcWalkEnd already decided (zero at a natural bound, circularPointBound's
// enclosure at a trimmed one), and this segment's LAST cell reaches the walk
// end, so both bounds belong to cells this chain draws.
//
// Those two readings are not the whole charge at an UNTRIMMED ArcSeg end.
// arcWalkEnd's zero states that the held station IS the recorded coordinate,
// which is a statement about the POSITION and not about the point the record
// DENOTES there: circularEndpointInterval (moments.go) takes the denoted
// curve's radius from Start alone, so at t == 1 the denoted point sits at
// Start's radius and End's own angle and misses the recorded End by the
// arc-end radial residual. arcNaturalEndRadialUpper charges exactly that
// residual, at t == 1 alone, and docs/loft-design.md §5.2 owns the term.
//
// A station the record cannot enclose answers +Inf, which the caller refuses
// on rather than publishing the certified sagitta as if it were the whole
// bound.
func LoftCircularStationChain(side LoftCircularSide, m int) ([]LoftStation, float64) {
	pts := make([]LoftStation, m)
	pts[0] = LoftStation{U: side.Walk.StartU, V: side.Walk.StartV}
	delta := math.Max(WalkEndPlaneDelta(side.Walk.StartBound), WalkEndPlaneDelta(side.Walk.EndBound))
	delta = math.Max(delta, side.EndRadialUpper)
	if !side.RangeOK {
		return pts, math.Inf(1)
	}
	for k := 1; k < m; k++ {
		t := float64(k) / float64(m)
		theta := side.Walk.Th0 + t*(side.Walk.Th1-side.Walk.Th0)
		sin, cos := math.Sincos(theta)
		pts[k] = LoftStation{U: side.Walk.CU + side.Walk.Radius*cos, V: side.Walk.CV + side.Walk.Radius*sin}
		tk := new(big.Rat).Add(side.TStart, new(big.Rat).Mul(big.NewRat(int64(k), int64(m)), side.DT))
		delta = math.Max(delta, WalkEndPlaneDelta(side.Bounder.BoundAt(tk, pts[k].U, pts[k].V)))
	}
	return pts, delta
}

// LoftCircularCellStations is the circular arm. It settles the pair's shared
// station count through loftSettleStationCount — docs/loft-design.md §5.1's
// JOINT WALK-UP, whose own doc comment owns that rule — and then walks BOTH
// sides at the count it settles on, never at either side's own smaller count,
// which is the correspondence a loft wall needs.
//
// What the walk-up compares against the target, and what this arm publishes,
// is loftCertifiedSagittaUpper's enclosure over the RECORD's own radius and
// sweep brackets — never the held float chordCount itself returns. That float
// is computed from the walk's math.Hypot radius and its math.Atan2 sweep, with
// no enclosure and no outward rounding, so it can decide a COUNT and nothing
// more (docs/loft-design.md §5.2's third rule). A candidate count whose
// certified sagitta has no derivation refuses at Table S row S14, in the arm
// and at the phase §4's gate-order paragraph assigns it, and the refusal never
// degrades into a published zero or a held substitute.
//
// The arm publishes the two terms §5.2's table lists for a chorded cell
// SEPARATELY, each read across the two sides under the max-versus-sum rule
// that table states for it rather than one stated here: sagittaUpper is the
// per-cell sagitta, which the caller accumulates into sectionDelta, and
// stationUpper is the stations' own displacement, which the caller
// accumulates into stationRound and thence into delta. The table's own
// sectionDelta and delta rows own that split, and the two terms are never
// added into one another here. Only the sagitta half is walked up against
// the target: the target names the chord DEPTH the chording commits to,
// while the station displacement is a rounding term of the generator's own
// arithmetic, read once the count has settled.
//
// It also discharges loftCellStations' own
// PARAMETER-MATCHED obligation: stations are placed at uniform PARAMETER t_k =
// TStart + (k/m)*(TEnd-TStart), which for a circular walk is uniform in ANGLE
// because t is the walk's own sweep fraction — and under that uniform-angle
// parametrization the exact departure sup_s |arc(s) - chord(s)| over one chord
// between the EXACT recorded points at t_k and t_(k+1) IS the cell's sagitta
// 2r·sin²(Δθ/4m), attained at s = 1/2. The certified half encloses THAT
// quantity from above.
//
// The chord this arm actually draws runs between two stations carrying the
// provenance §5.1's Table C marks them with rather than between those two
// exact points, which is precisely the gap the station-displacement half
// closes: the sagitta alone bounds a chord the build never
// drew, and a built chord can and does depart from the curve by more than it.
// A SET distance from an arbitrary chord point to the curve would bound
// something else entirely, and the uniform-angle station rule — read at an
// EXACT rational parameter, never a float rounding of one (circularPointBound)
// — is what keeps both halves readings of the same quantity.
//
// matchedDelta is loftCellStations' own per-cell obligation — the CHORD-TO-
// CURVE half of docs/loft-design.md §5.2's matchedDelta row, which the
// consumer composes with the build's own delta (chordCellDeltaUpper), never
// the whole row: this arm's sagitta discharges that half EXACTLY (the
// paragraph above), and every cell of one uniformly-stepped circular segment
// shares the same true angular width, so the same value — math.Max(s0, s1) —
// is the correct, exact per-cell reading for all m cells, not merely a safe
// upper bound repeated m times.
func LoftCircularCellStations(a, b LoftCircularSide, target float64) ([]LoftStation, []LoftStation, float64, []float64, float64, error) {
	m, s0, s1, err := LoftSettleStationCount(a, b, target)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}
	stations0, d0 := LoftCircularStationChain(a, m)
	stations1, d1 := LoftCircularStationChain(b, m)
	stationUpper := math.Max(d0, d1)
	if proofbound.IsNonFinite(stationUpper) {
		return nil, nil, 0, nil, 0, ErrLoftStationDisplacementUnderivable
	}
	for k := range m - 1 {
		eq0 := stations0[k] == stations0[k+1]
		eq1 := stations1[k] == stations1[k+1]
		if eq0 != eq1 {
			return nil, nil, 0, nil, 0, fmt.Errorf(`%w: chord cell %d of this paired segment collapses to one point on only one of the two sections`, decaderr.ErrUnsupported, k)
		}
	}
	sagitta := math.Max(s0, s1)
	matchedDelta := make([]float64, m)
	for i := range matchedDelta {
		matchedDelta[i] = sagitta
	}
	return stations0, stations1, sagitta, matchedDelta, stationUpper, nil
}
