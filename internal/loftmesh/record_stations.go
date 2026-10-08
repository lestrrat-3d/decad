package loftmesh

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type loftCircularBounder struct{ seg sectionrecord.CurveSegment }

func (b loftCircularBounder) BoundAt(t *big.Rat, u, v float64) proofbound.WalkEndBound {
	return boundarywalk.CircularPointBound(b.seg, t, u, v)
}

func RecordCircularSide(w survey2d.SegmentWalk, seg sectionrecord.CurveSegment) LoftCircularSide {
	radius, sweep, enclosed := circularbounds.WalkEnclosures(circularbounds.RecordSegment(seg))
	tStart, dt, rangeOK := CircularSegmentRange(seg)
	return LoftCircularSide{
		Walk: w, Radius: radius, Sweep: sweep, Enclosed: enclosed,
		TStart: tStart, DT: dt, RangeOK: rangeOK,
		EndRadialUpper: ArcNaturalEndRadialUpper(seg), Bounder: loftCircularBounder{seg: seg},
	}
}

func stationPoints(stations []LoftStation) []sectionrecord.Point2 {
	points := make([]sectionrecord.Point2, len(stations))
	for i, station := range stations {
		points[i] = sectionrecord.Point2{U: station.U, V: station.V}
	}
	return points
}

// StationCap is docs/loft-design.md §5.1's ceiling on a build's TOTAL station
// count Σstations (§7), as a function of the build's paired-segment count P
// (docs/loft-gear-bounds-design.md §7):
//
//	stationCap(P) = min(max(StationCapFloor, StationsPerPair·P), StationCapCeiling)
//
// The cap scales with P because a record's need for stations does: the
// helical gear fixture of docs/loft-gear-bounds-design.md needs 124–132
// stations for one tooth (P = 6) and 952–1960 for a full outline (P = 48–240).
// S15 reads the cap through StationShare, which splits it evenly over the
// chorded pairs, and a gear's flanks need far more cells than its arcs: the
// z = 8 outline's flanks need 48 cells each. 64 stations per paired segment
// gives them a share of 95; 32 would give 47 and refuse that outline. The
// floor keeps a record of a few curves the room the docs/loft-design.md §14
// reference wedges need.
//
// The hard ceiling bounds what the cap protects. §7 fixes the assembled
// triangle count at F = 4·Σstations + 4H − 4 over H hole loops, and H is at
// most Σstations − 1 (every loop holds at least one segment and every paired
// segment chords at m ≥ 1), so F ≤ 8·Σstations − 8. At Σstations =
// StationCapCeiling that is MaxLoftAuditTriangles, the triangle ceiling S8
// checks before the audit lifts a single triangle, so a build whose P is
// below the cap and every pair of which passes S15 never reaches S8 on its
// triangle count. The free-form work ceiling is sized from the same cap
// (StationWorkLimit).
//
// The cap is NOT freeform.MaxChordsPerWalk (tessellate.go). That constant
// bounds how finely ONE curve may be chorded and knows nothing of how many
// curves a build holds; this one bounds the build.
func StationCap(p uint64) int {
	if p >= StationCapCeiling/StationsPerPair {
		return StationCapCeiling
	}
	return int(max(StationCapFloor, StationsPerPair*p)) //nolint:gosec // p < StationCapCeiling/StationsPerPair here, so the product is below StationCapCeiling.
}

// StationCap's three constants (docs/loft-gear-bounds-design.md §7).
const (
	StationCapFloor   = 512
	StationsPerPair   = 64
	StationCapCeiling = 8192
)

// StationWorkUnits is the free-form work one station may charge each record's
// counter. On the gear fixtures of docs/loft-gear-bounds-design.md the loft's
// walk resolution and station walk together charge 5300–7000 units per
// station per record, averaged over every station of the build, and
// TestLoftStationWalkWorkPerStation keeps that average below this constant.
const StationWorkUnits = 8192

// StationWorkLimit is the free-form work ceiling a loft raises each record's
// counter to before it resolves the records' walks
// (docs/loft-gear-bounds-design.md §7):
//
//	max(freeform.FreeformWorkLimit, spent + StationWorkUnits·stationCap(P))
//
// spent is the counter's total at the raise. Every charge before the raise
// met the default ceiling, so spent is at most freeform.FreeformWorkLimit and
// the result is at most FreeformWorkLimit + StationWorkUnits·StationCapCeiling
// = 2^20 + 2^26. The same counter stays the record's one counter for the
// operation: the raise sets its ceiling and never resets what it has spent.
func StationWorkLimit(spent, p uint64) uint64 {
	return max(freeform.FreeformWorkLimit, spent+StationWorkUnits*uint64(StationCap(p))) //nolint:gosec // StationCap is in [StationCapFloor, StationCapCeiling].
}

// PairCounts reads docs/loft-design.md §5.1's two build-wide counts off
// Table P over both records: P, the total paired-segment count, and C, the
// number of CHORDED pairs among them — same-kind circular and same-kind Tier A
// free-form together. Both are decided from the two authenticated records
// alone — no station is generated to read either.
//
// A pair is chorded when BOTH sides' walks are circular or BOTH are free-form;
// a mixed-kind pair is
// S3's refusal (ValidateLoftRecords) and is counted in P like any other, since
// P's own entitlement is one station per paired segment whatever its kind. Only
// loop0's segment counts are read: S2 has already proved loop1 carries the same
// count, which is what makes one loop's shape the pair count for both.
//
// The accumulation is checked (proofbound.WallCheckedAdd, internal/proofbound/budget.go) and answers false on
// overflow rather than wrapping, the discipline §5.1 states for every sum the
// mMax comparison reads.
func PairCounts(loops0 []sectionrecord.LoopRecord, offsets []int, walks0, walks1 [][]survey2d.SegmentWalk) (uint64, uint64, bool) {
	var p, c uint64
	for i := range loops0 {
		n := len(loops0[i].Segments)
		off := offsets[i]
		for j := range n {
			var ok bool
			if p, ok = proofbound.WallCheckedAdd(p, 1); !ok {
				return 0, 0, false
			}
			k := (j + off) % n
			if IsChordedPair(walks0[i][j], walks1[i][k]) {
				if c, ok = proofbound.WallCheckedAdd(c, 1); !ok {
					return 0, 0, false
				}
			}
		}
	}
	return p, c, true
}

// IsChordedPair reports whether a paired segment chords (docs/loft-design.md
// §5.1): both sides circular, or both free-form.
func IsChordedPair(w0, w1 survey2d.SegmentWalk) bool {
	return w0.Kind == w1.Kind && (w0.Kind == survey2d.WalkCircular || w0.Kind == survey2d.WalkFreeform)
}

// StationShare allocates docs/loft-design.md §5.1's per-segment share of
// the station cap:
//
//	mMax = 1 + max(0, (stationCap(P) - P) / C)      // integer division
//
// Every paired segment is entitled to its first station — a LineSeg pair's
// whole entitlement (m = 1, §7) — and each of the C chorded pairs may take at
// most mMax. Because C counts the chorded pairs AMONG P, a chorded pair's m
// stations SUBSUME that first-station entitlement rather than adding to it, so
// §5.1's own sum shows no build every pair of which passes S15 can exceed the
// cap.
//
// The caller must not reach here with C == 0: §5.1 states a build with no
// chorded pair never consults the cap at all, and dividing by C would be
// undefined besides.
//
// A record whose own P already reaches the cap — P at or above
// StationCapCeiling, since below it the cap is at least 64·P — clamps to
// mMax = 1, which §5.1 carves
// out deliberately: such a record is past chording altogether and S8 is what
// refuses it, over the assembled triangle count §6's own preflight computes.
// Refusing it here instead would refuse a mixed build while admitting an
// all-LineSeg build of the identical triangle count.
func StationShare(p, c uint64) int {
	q := max(int64(0), (int64(StationCap(p))-int64(p))/int64(c)) //nolint:gosec // p and c are paired-segment counts proofbound.WallCheckedAdd already proved do not overflow, and a record large enough to pass int64 cannot be built from the process's memory limits.
	return 1 + int(q)
}

// RecordCellStations generates one paired LineSeg or circular loft segment's
// shared chord stations: a kind switch on w0/w1's own survey2d.WalkKind. Both
// arms publish the identical contract — two per-plane station chains at a
// SHARED count, plus the sagitta this cell's own chording commits. A
// same-kind Tier A free-form pair is FreeformCellPoints' arm instead, which
// PairRecords dispatches itself: that arm needs the segment's share of the
// station cap and publishes a per-cell arc-length bound, and neither fits
// this signature.
//
// stations0/stations1 each carry ONLY this segment's own interior stations,
// never its shared end point — the next segment's own first station (or the
// loop's wrap) supplies it, the convention LoopPair's own doc comment
// states, and what makes a loop's own chain total the count
// docs/loft-design.md §7 states for it rather than one stated here.
//
// sagittaUpper is the proven upper bound on max_s |curve(s) - chord(s)|
// under the SAME parameter this cell's own uniform stations walk — a
// PARAMETER-MATCHED bound, never a set distance from some chord point to the
// curve (internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper doc comment states the
// distinction this field's name exists to keep visible). The LineSeg arm's
// chord IS the recorded segment, so its bound is exactly zero; the circular
// arm composes the two terms docs/loft-design.md §5.2's table lists for a
// chorded cell — that table's certified per-cell sagitta and the displacement
// its stations carry, each with the derivation and rounding direction that
// table states and the provenance mark §5.1's Table C gives those stations —
// and answers the refusal §5.2's table assigns those terms.
//
// seg0/seg1 are the two RECORDED segments w0/w1 were resolved from. An arm
// whose bound is a proof rather than a held float needs them: every enclosure
// docs/loft-design.md §5.2 names is stated by the record, never by the walk,
// whose radius is a math.Hypot and whose angles are a math.Atan2 the walk
// itself declares it cannot enclose (extrude.go's circularWalk).
//
// target is ChordTarget's own single per-build reading, never
// recomputed per cell. work0/work1 are the two records' own free-form work
// counters (docs/spline-design.md §5.2). Neither arm below charges them;
// FreeformCellPoints, the free-form arm, charges the same two counters.
//
// stationRoundUpper is docs/loft-design.md Table S row S14 (a10-plan.md Part
// 3 PR 6): the proven rounding a COMPUTED station commits, taken as a MAX
// over this cell's own stations on both sides — a component of delta, never
// sectionDelta. NEITHER arm is exempt, and the LineSeg arm is not the
// zero it would be if a kind could grant one: §5.2 PINS a station by its own
// NATURAL parameter, never by the kind of segment it sits on, so this arm
// charges exactly zero where its two stations are UNTRIMMED recorded
// endpoints and charges its own certified lineWalkEndBound wherever a TRIMMED
// parameter made lerp2 compute one. Each arm's own doc comment states its
// mechanism.
//
// matchedDelta is the CHORD-TO-CURVE HALF of docs/loft-design.md §5.2's
// matchedDelta row — the half a consumer composes with the build's own delta
// (ChordCellDeltaUpper) to reach internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper own
// matchedDeltaUpper obligation (F1's rule) — ONE ENTRY PER CELL, never a single per-segment
// scalar, since the bisected free-form arm (FreeformCellPoints) settles cells
// of one paired segment at different depths and so at different readings.
// len(matchedDelta) always equals len(stations0), the per-cell count every
// arm below publishes. It is read per cell rather than from sagittaUpper: the
// LineSeg arm's chord IS the curve, so every entry is exactly 0; the circular
// arm's own sagitta discharges that half exactly, so every entry equals
// the segment's own sagittaUpper (CircularCellPoints' own doc comment).
func RecordCellStations(w0, w1 survey2d.SegmentWalk, seg0, seg1 sectionrecord.CurveSegment, target float64, work0, work1 *freeform.FreeformWork) ([]sectionrecord.Point2, []sectionrecord.Point2, float64, []float64, float64, error) {
	switch {
	case w0.Kind == survey2d.WalkLine && w1.Kind == survey2d.WalkLine:
		return LineCellPoints(w0, w1)
	case w0.Kind == survey2d.WalkCircular && w1.Kind == survey2d.WalkCircular:
		return CircularCellPoints(w0, w1, seg0, seg1, target)
	default:
		// Unreached from any real build: ValidateLoftRecords' own S3 gate
		// refuses every mixed-kind pair before PairRecords ever calls this
		// function (SameKindGate), and PairRecords sends a free-form pair to
		// FreeformCellPoints. A defensive refusal, so a kind this switch has
		// no case for fails loud rather than falling into either analytic
		// arm's own assumptions.
		return nil, nil, 0, nil, 0, fmt.Errorf(`%w: this loft evaluator has no chord station rule for this segment-kind pairing`, decaderr.ErrUnsupported)
	}
}

// LineCellPoints delegates the station proof to internal/tessellation.
func LineCellPoints(w0, w1 survey2d.SegmentWalk) ([]sectionrecord.Point2, []sectionrecord.Point2, float64, []float64, float64, error) {
	a, b, sagitta, matched, round, err := LoftLineCellStations(w0, w1)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}
	return stationPoints(a), stationPoints(b), sagitta, matched, round, nil
}

// SettleRecordStationCount delegates the station proof to internal/tessellation.
func SettleRecordStationCount(w0, w1 survey2d.SegmentWalk, seg0, seg1 sectionrecord.CurveSegment, target float64) (int, float64, float64, error) {
	return LoftSettleStationCount(RecordCircularSide(w0, seg0), RecordCircularSide(w1, seg1), target)
}

// CircularCellPoints delegates the station proof to internal/tessellation.
func CircularCellPoints(w0, w1 survey2d.SegmentWalk, seg0, seg1 sectionrecord.CurveSegment, target float64) ([]sectionrecord.Point2, []sectionrecord.Point2, float64, []float64, float64, error) {
	a, b, sagitta, matched, round, err := LoftCircularCellStations(
		RecordCircularSide(w0, seg0), RecordCircularSide(w1, seg1), target,
	)
	if err != nil {
		return nil, nil, 0, nil, 0, err
	}
	return stationPoints(a), stationPoints(b), sagitta, matched, round, nil
}

// PerCellArcUpper is one paired segment's own per-cell arc-length upper
// bound, shared by every one of its m uniformly-stepped cells
// (ComputeLoftChordedAllow, loft_chord_allow.go). Uniform angular stepping means
// each of the m cells carries the SAME true share of the whole sweep, so
// dividing a proven upper bound on the WHOLE segment's length by m stays an
// upper bound on each share.
//
// For a circular segment (sectionrecord.CircleSeg/sectionrecord.ArcSeg) that whole-length bound is
// moments.go's circularLengthInterval — an EXACT rational bracket on the
// segment's true length — never survey2d.SegmentWalk.lengthUpper: that field's own
// bound is deliberately loose (proofbound.CircularSweepUpper bounds any sectionrecord.ArcSeg's sweep
// by the full 2*pi it could reach, never the sweep THIS record states, per
// its own doc comment), so a quarter-turn arc's lengthUpper overstates its
// true length by roughly 4x — a slack that would flow straight through this
// division into proofbound.CellChordCurveAreaUpper's own arcLenUpper argument and
// quadruple the wall/seam/cap terms it feeds (an earlier version of this
// function did exactly that, measured Suspect on the calibrated reference
// wedge before this fix). The tight bracket is what keeps the per-cell share
// close to the true one.
//
// For the LineSeg arm (m=1, and every other kind circularLengthInterval
// declines) this falls back to survey2d.SegmentWalk.lengthUpper exactly, unaffected —
// a straight chord's own recorded length bound was never the loose one. A
// non-finite whole-length bound propagates rather than silently shrinking
// under the division.
func PerCellArcUpper(seg sectionrecord.CurveSegment, w survey2d.SegmentWalk, m int) float64 {
	if ns, err := sectionrecord.NormalizeSegment(seg); err == nil {
		if iv, ok := circularbounds.LengthInterval(circularbounds.RecordSegment(ns)); ok {
			return proofbound.UpRound(proofbound.RatFloatUp(iv.Hi) / float64(m))
		}
	}
	if proofbound.IsNonFinite(w.LengthUpper) {
		return math.Inf(1)
	}
	return proofbound.UpRound(w.LengthUpper / float64(m))
}

// PerCellTangentEnergy is internal/proofbound/bounds.go's proofbound.CellChordCurveAreaAllow own
// tangentEnergyUpper obligation for ONE cell of this walk: a proven upper bound
// on the integral of |curve'(s) - chord|^2 over the cell's own shared
// parameter, or +Inf where this evaluator cannot prove one.
//
// Its two operands are BOTH read from the RECORD's own certified enclosures —
// PerCellArcUpper over circularLengthInterval, and CertifiedChordLower over
// circularWalkEnclosures — never from the walk's own held math.Hypot radius and
// math.Atan2 angles, neither of which the walk can enclose (extrude.go's
// circularWalk). proofbound.UniformSpeedTangentEnergyUpper's published energy DECREASES in
// its chord operand, so a chord read off those floats can overstate the true
// chord and understate the energy every consumer downstream spends: a held
// value wearing a proof's clothes, which circularWalkEnclosures' own doc
// comment forbids.
//
// It is dispatched on the WALK KIND rather than shared across every arm,
// because the obligation proofbound.UniformSpeedTangentEnergyUpper discharges rests on the
// shared parametrization having CONSTANT SPEED — a property of the arm that
// placed the stations, not of the cell's geometry. The circular arm's
// uniform-ANGLE stations (CircularCellPoints) are constant speed on a
// circle, which is what discharges it; a straight walk's chord IS its curve, so
// its deviation is identically zero. Every other kind answers +Inf here, so it
// degrades proofbound.CellChordCurveAreaAllow to that helper's premise-free arm
// rather than being silently handed a bound whose premise it does not meet. A
// same-kind free-form pair never reaches this function: its span-native
// parameter is not constant speed, so FreeformCellPoints reads each cell's
// exact energy (freeform.SpanTangentEnergyUpper) instead.
func PerCellTangentEnergy(seg sectionrecord.CurveSegment, w survey2d.SegmentWalk, m int) float64 {
	switch w.Kind {
	case survey2d.WalkLine:
		return 0
	case survey2d.WalkCircular:
		return proofbound.UniformSpeedTangentEnergyUpper(PerCellArcUpper(seg, w, m), CertifiedChordLower(seg, m))
	default:
		return math.Inf(1)
	}
}

// CircularStationChain delegates the station proof to internal/tessellation.
func CircularStationChain(w survey2d.SegmentWalk, seg sectionrecord.CurveSegment, m int) ([]sectionrecord.Point2, float64) {
	stations, delta := LoftCircularStationChain(RecordCircularSide(w, seg), m)
	return stationPoints(stations), delta
}

// ArcNaturalEndRadialUpper charges docs/loft-design.md §5.2's ARC-END RADIAL
// RESIDUAL: an upper bound on | ‖End − Center‖ − ‖Start − Center‖ | for a
// recorded sectionrecord.ArcSeg whose walk reaches the natural bound t == 1, and exactly
// zero for every other segment and every other bound.
//
// The term exists because a pin is a statement about a POSITION and not about
// the point the record denotes at that parameter. An sectionrecord.ArcSeg records three
// points and the curve it denotes takes its radius from Start ALONE —
// circularEndpointInterval and circularWalkEnclosures (moments.go) both read
// |Start − Center|, the reading docs/sketch-seam-design.md states outright —
// so the denoted point at t == 1 lies at THAT radius and End's own angle. The
// walk holds the recorded End there (arcWalkEnd), and the two coincide only
// where the two radii are equal. Nothing certifies that: validateSegment's
// sectionrecord.ArcSeg arm (record.go) tests point finiteness and the parameter range,
// seam.go records geom.Arc's three points verbatim, and sketch's own arc
// radius constraint is solved to solver tolerance rather than proven. So the
// residual is CHARGED. It is not a gate: a record whose radii differ is
// measured, never admitted or refused on the measurement, which is CLAUDE.md's
// reject-only rule read correctly — this term bounds a record and can never
// bless one.
//
// It is charged at t == 1 ALONE. At t == 0 the denoted point IS Start, by the
// definition of the denoted radius, so the true displacement there is zero and
// a generator's enclosure WIDTH read at that bound would publish a positive
// displacement for a station that has none. This is why the charge lives here
// rather than in arcWalkEnd, whose zero every other consumer of a pinned
// endpoint POSITION already relies on (pinArcWalkEnds' own doc comment).
//
// circularbounds.ArcRadialResidualUpper owns the exact-rational bound. Equal
// squared radii answer exactly zero, so a record that does state an exact
// circle keeps the zero delta §5.2 grants it, and an underivable bound
// answers +Inf, which the caller refuses on rather than publishing a
// substitute — the S14 discipline §5.2's table states for every term in it.
func ArcNaturalEndRadialUpper(seg sectionrecord.CurveSegment) float64 {
	arc, ok := seg.(sectionrecord.ArcSeg)
	if !ok || (arc.TStart != 1 && arc.TEnd != 1) {
		return 0
	}
	return circularbounds.ArcRadialResidualUpper(arc)
}

// CircularSegmentRange states a recorded circular segment's own parameter
// range exactly: the start parameter and the signed width TEnd − TStart, both
// over the rationals. CircularStationChain divides that width into m equal
// parts, so the division has to happen where no rounding can enter it — a
// station parameter rounded to a float would name a point that divides the
// sweep slightly unevenly, and the per-cell sagitta the caller publishes is
// derived from the EVEN division alone (circularPointBound's own doc comment).
//
// A kind with no circular parameter range answers false, and the caller
// refuses.
func CircularSegmentRange(seg sectionrecord.CurveSegment) (*big.Rat, *big.Rat, bool) {
	var tStart, tEnd float64
	switch seg := seg.(type) {
	case sectionrecord.CircleSeg:
		tStart, tEnd = seg.TStart, seg.TEnd
	case sectionrecord.ArcSeg:
		tStart, tEnd = seg.TStart, seg.TEnd
	default:
		return nil, nil, false
	}
	start := proofarith.FloatRat(tStart)
	if start == nil {
		return nil, nil, false
	}
	return start, circularbounds.ExactCoordinateDelta(tEnd, tStart), true
}

// CertifiedSagittaUpper delegates the station proof to internal/tessellation.
func CertifiedSagittaUpper(seg sectionrecord.CurveSegment, m int) float64 {
	if m <= 0 {
		return math.Inf(1)
	}
	radius, sweep, enclosed := circularbounds.WalkEnclosures(circularbounds.RecordSegment(seg))
	return LoftCertifiedSagittaUpper(radius, sweep, enclosed, m)
}

// CertifiedChordLower delegates the station proof to internal/tessellation.
func CertifiedChordLower(seg sectionrecord.CurveSegment, m int) float64 {
	if m <= 0 {
		return 0
	}
	radius, sweep, enclosed := circularbounds.WalkEnclosures(circularbounds.RecordSegment(seg))
	return LoftCertifiedChordLower(radius, sweep, enclosed, m)
}
