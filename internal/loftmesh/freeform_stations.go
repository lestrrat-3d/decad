package loftmesh

import (
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// StationCapError is docs/loft-design.md Table S row S15's refusal: a chorded
// pair whose station count passes the cells remaining in the shared cap.
// It names the pair that exhausted that remainder. errors.Is answers for
// freeform.ErrTooManyChords, the sentinel §5.1
// assigns this row, and through it for ErrUnsupported.
//
// AtLeast is set where the generator stopped at its allowance rather than settling
// a count: the free-form arm's dyadic walk refuses the moment its proven lower
// bound on the chord count passes MMax, so M is that lower bound and the
// finished chain would carry at least M cells.
type StationCapError struct {
	Loop, Seg int
	M, MMax   int
	AtLeast   bool
}

func (e *StationCapError) Error() string {
	needs := fmt.Sprintf("%d", e.M)
	if e.AtLeast {
		needs = fmt.Sprintf("at least %d", e.M)
	}
	return fmt.Sprintf(
		`%s: loop %d segment %d needs %s chord cells to meet the loft chord target, past the %d remaining station cap allows`,
		decaderr.ErrUnsupported.Error(), e.Loop, e.Seg, needs, e.MMax,
	)
}

func (e *StationCapError) Unwrap() error { return freeform.ErrTooManyChords }

// FreeformCell is one same-kind Tier A free-form pair's chord chain
// (docs/loft-design.md §5.1's free-form arm), in the shape PairRecords appends
// to a loop.
//
// Stations0/Stations1 carry this segment's own stations in walk order, the
// segment's start included and its end excluded: the next segment's first
// station, or the loop's wrap, supplies the end. Every other slice carries one
// entry per cell, parallel to the station lists. ArcUpper0/ArcUpper1 are each
// side's SpanSpeedUpper over the cell's own dyadic sub-span, and MatchedDelta
// is the larger of the two sides' SpanMatchedDeltaUpper over the same
// sub-spans — the chord-to-curve half of §5.2's matchedDelta row.
// Energy0/Energy1 are each side's SpanTangentEnergyUpper over the same
// sub-span, §5.2's tangentEnergy_k row. Sagitta is the largest measured cell
// sagitta over both sides, and Round is the largest station rounding over both
// sides, the segment's own end included.
type FreeformCell struct {
	Stations0, Stations1 []sectionrecord.Point2
	Sagitta              float64
	MatchedDelta         []float64
	ArcUpper0, ArcUpper1 []float64
	Energy0, Energy1     []float64
	Round                float64
}

// FreeformCellPoints chords one same-kind Tier A free-form pair
// (docs/loft-design.md §5.1's free-form arm).
//
// Each side's converted chain is put in WALK order first: a forward side keeps
// its natural spans, and a reversed side takes its spans last to first with
// each span's control points reversed. The chains are then divided into the
// least common multiple of their span counts, with exact rational de Casteljau
// splits. Slot 0 of each side is its own recorded walk start. Neither reversal
// nor refinement writes through the recorded walk's control points.
//
// freeform.PairChainStations measures, accepts and bisects every cell on both
// sides together, so the two sides hold one dyadic cell set. Its ceiling is
// maxCells, the shared station cap's current remainder: passing it refuses
// with a StationCapError whose M is the walk's proven lower bound (S15), and
// the caller names the loop and segment. A cell whose measured sagitta is
// non-finite refuses with ErrLoftSagittaUnderivable (S14) rather than bisecting
// to the ceiling, and so does a non-finite speed or matched-departure bound. A
// non-finite tangent energy does not refuse: §5.2's tangentEnergy_k row passes
// it on as +Inf, which costs the ruled area leg its sharp arm and nothing else.
//
// Every station is an exact rational point on the curve, rounded once into a
// Point2. Its rounding is charged as §5.2's free-form stationRound arm states:
// the exact rational's own gap from the held float, which is zero exactly when
// the rounding was exact. The segment's end station is held by the next
// segment, so its own walk end bound is charged here as well.
func FreeformCellPoints(w0, w1 survey2d.SegmentWalk, target float64, maxCells int, work0, work1 *freeform.FreeformWork) (FreeformCell, error) {
	spans0, spans1 := walkOrderSpans(w0), walkOrderSpans(w1)
	if len(spans0) == 0 || len(spans1) == 0 {
		return FreeformCell{}, fmt.Errorf(`%w: a paired free-form station chain needs a span on each side`, decaderr.ErrDegenerate)
	}
	common, ok := commonSpanCount(len(spans0), len(spans1), maxCells)
	if !ok {
		return FreeformCell{}, &StationCapError{M: maxCells + 1, MMax: maxCells, AtLeast: true}
	}
	var err error
	spans0, err = freeform.RefineSpanChain(spans0, common/len(spans0), work0)
	if err != nil {
		return FreeformCell{}, err
	}
	spans1, err = freeform.RefineSpanChain(spans1, common/len(spans1), work1)
	if err != nil {
		return FreeformCell{}, err
	}
	chain, err := freeform.PairChainStations(spans0, spans1, target, freeform.PairChainLimits{
		MaxChords:   maxCells,
		Underivable: ErrLoftSagittaUnderivable,
	}, work0, work1)
	if err != nil {
		if errors.Is(err, freeform.ErrTooManyChords) {
			return FreeformCell{}, &StationCapError{M: maxCells + 1, MMax: maxCells, AtLeast: true}
		}
		return FreeformCell{}, err
	}
	for k := range chain.MatchedDelta {
		if proofbound.IsNonFinite(chain.MatchedDelta[k]) ||
			proofbound.IsNonFinite(chain.ArcUpper[0][k]) || proofbound.IsNonFinite(chain.ArcUpper[1][k]) {
			return FreeformCell{}, ErrLoftSagittaUnderivable
		}
	}

	cells := len(chain.MatchedDelta)
	out := FreeformCell{
		Sagitta:      chain.Sagitta,
		MatchedDelta: chain.MatchedDelta,
		ArcUpper0:    chain.ArcUpper[0],
		ArcUpper1:    chain.ArcUpper[1],
		Energy0:      chain.TangentEnergy[0],
		Energy1:      chain.TangentEnergy[1],
		Round:        math.Max(WalkEndPlaneDelta(w0.EndBound), WalkEndPlaneDelta(w1.EndBound)),
	}
	for side, stations := range chain.Stations {
		points := make([]sectionrecord.Point2, cells)
		for k := range cells {
			p, round, err := roundedStation(stations[k])
			if err != nil {
				return FreeformCell{}, err
			}
			points[k] = p
			out.Round = math.Max(out.Round, round)
		}
		if side == 0 {
			out.Stations0 = points
		} else {
			out.Stations1 = points
		}
	}
	if proofbound.IsNonFinite(out.Round) {
		return FreeformCell{}, ErrLoftStationDisplacementUnderivable
	}
	return out, nil
}

// commonSpanCount gives both span-uniform chains the same slot boundaries.
// Check the current station allowance before multiplying, so a large least common multiple
// cannot allocate a chain or overflow an int.
func commonSpanCount(a, b, maxCells int) (int, bool) {
	if a < 1 || b < 1 || maxCells < 1 {
		return 0, false
	}
	x, y := a, b
	for y != 0 {
		x, y = y, x%y
	}
	if a/x > maxCells/b {
		return 0, false
	}
	return a / x * b, true
}

// roundedStation rounds one exact chain station into the Point2 the build
// holds and reports the in-plane distance between the two.
func roundedStation(station freeform.RatPoint) (sectionrecord.Point2, float64, error) {
	p, ok := splinebezier.Point2Of(station)
	if !ok {
		return sectionrecord.Point2{}, 0, fmt.Errorf(`%w: a free-form loft station has no representable plane coordinate`, decaderr.ErrUnsupported)
	}
	bound := proofbound.WalkEndBound{
		U: proofarith.RationalFloatError(station.U, p.U),
		V: proofarith.RationalFloatError(station.V, p.V),
	}
	return p, WalkEndPlaneDelta(bound), nil
}

// walkOrderSpans returns the walk's converted chain in walk order. A forward
// walk's natural chain is returned as is. A reversed walk's chain is rebuilt
// last span first, each span's control points reversed, in new slices that
// share the walk's rationals without writing to them.
func walkOrderSpans(w survey2d.SegmentWalk) []freeform.BezierSpan {
	if !w.Reversed {
		return w.Spans
	}
	out := make([]freeform.BezierSpan, len(w.Spans))
	for i, span := range w.Spans {
		rev := slices.Clone(span)
		slices.Reverse(rev)
		out[len(w.Spans)-1-i] = rev
	}
	return out
}
