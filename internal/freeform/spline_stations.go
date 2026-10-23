package freeform

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PairChainLimits narrows PairChainStations' walk for a caller that owns a
// smaller ceiling than MaxChordsPerWalk or a refusal of its own for a sagitta
// with no derivation.
type PairChainLimits struct {
	// MaxChords caps the finished chain's chord count. Zero means
	// MaxChordsPerWalk; a value above it is clamped to it. Passing the cap
	// refuses with ErrTooManyChords exactly as PairStations does.
	MaxChords int
	// Underivable, when non-nil, is returned the moment a cell's measured
	// sagitta is non-finite. A nil value keeps PairStations' behaviour, which
	// bisects such a cell until the cap refuses.
	Underivable error
}

// PairChain is PairChainStations' answer. Stations, MatchedDelta and Sagitta
// carry PairStations' own meanings. ArcUpper[side][k] is SpanSpeedUpper of
// accepted cell k's own dyadic sub-span on that side: a proven upper bound on
// the cell's tangent speed under its native [0, 1] parameter, and so on its arc
// length, never below its chord length. TangentEnergy[side][k] is
// SpanTangentEnergyUpper of that same sub-span: the exact integral of
// |C'(t) − Δ|² under the same parameter, rounded outward once.
type PairChain struct {
	Stations      [2][]RatPoint
	MatchedDelta  []float64
	ArcUpper      [2][]float64
	TangentEnergy [2][]float64
	Sagitta       float64
}

// PairChainStations is PairStations under a caller's own limits, returning the
// per-cell speed bounds and tangent energies a same-kind free-form loft cell
// needs beside the matched-departure bounds (agent-docs/loft-design.md §5.2's
// arcLenUpper_k and tangentEnergy_k rows). The
// walk, its sharing of one dyadic cell set between the two sides, its
// determinism and its charging are PairStations' own. It differs in what
// decides a cell: it also bisects while either side's parameter-matched
// departure exceeds target (SagittaStationWalk.Matched), so every entry of
// MatchedDelta is at or below target, and Sagitta still reports the measured
// sagitta maximum.
func PairChainStations(spans0, spans1 []BezierSpan, target float64, limits PairChainLimits, work0, work1 *FreeformWork) (PairChain, error) {
	reader := &PairCellReader{}
	gen := NewSagittaStationWalk(target, reader, len(spans0), work0, work1)
	gen.Matched = true
	gen.Limit = limits.MaxChords
	gen.Underivable = limits.Underivable
	stations0, stations1, err := pairWalk(spans0, spans1, gen)
	if err != nil {
		return PairChain{}, err
	}
	return PairChain{
		Stations:      [2][]RatPoint{stations0, stations1},
		MatchedDelta:  reader.MatchedDelta,
		ArcUpper:      reader.ArcUpper,
		TangentEnergy: reader.TangentEnergy,
		Sagitta:       gen.SagittaUpper,
	}, nil
}

// pairWalk runs PairStations' guards and its cell walk over gen, returning the
// two station lists with the chain's own final station appended to each.
func pairWalk(spans0, spans1 []BezierSpan, gen *SagittaStationWalk) ([]RatPoint, []RatPoint, error) {
	work0, work1 := gen.Works[0], gen.Works[1]
	if len(spans0) != len(spans1) {
		return nil, nil, fmt.Errorf(
			`%w: two paired free-form span chains of different length (%d vs %d) share no common dyadic parameter domain`,
			decaderr.ErrUnsupported, len(spans0), len(spans1),
		)
	}
	if len(spans0) == 0 {
		return nil, nil, fmt.Errorf(`%w: a paired free-form station chain needs at least one span on each side`, decaderr.ErrDegenerate)
	}
	// Every span carries at least one chord even when it needs no bisection at
	// all, so a chain of more spans than the ceiling admits already exceeds the
	// chord count ErrTooManyChords names.
	if len(spans0) > gen.limit() {
		return nil, nil, ErrTooManyChords
	}
	// A span with no control points at all is not a Bézier of any degree — it
	// has no chord and no curve, unlike a COLLAPSED span (every control point
	// coincident, §5.1), which still has a degree and a (single-point) chord.
	// DyadicSpan.split preserves point count at every depth (spline_length.go),
	// so refusing it HERE, before the first DyadicSpanOf conversion, is enough
	// to keep every cell SagittaStationWalk.WalkCell ever sees at n >= 1: DyadicSpanSagittaUpper's
	// own n==0 guard exists for a caller that reaches it directly (SpanSagittaUpper,
	// whose only error is the counter's), not for this walk, whose accept branch
	// unconditionally reads DyadicSpan.RatPointAt(0) — reachable only because that guard's 0
	// answer let the cell through, an index-out-of-range panic otherwise, never a
	// wrong Measurement.
	for i := range spans0 {
		if len(spans0[i]) == 0 || len(spans1[i]) == 0 {
			return nil, nil, fmt.Errorf(
				`%w: a free-form span with no control points at index %d has no chord to bisect`,
				decaderr.ErrDegenerate, i,
			)
		}
	}

	for i := range spans0 {
		// The DyadicSpanOf conversion that opens the walk charges itself, like
		// every step inside it: it runs its own exact big.Int arithmetic per
		// control point, and leaving it free would let a chain of very wide
		// spans do unbounded work before the first cell is ever measured.
		cell0, err := DyadicSpanOf(work0, spans0[i])
		if err != nil {
			return nil, nil, err
		}
		cell1, err := DyadicSpanOf(work1, spans1[i])
		if err != nil {
			return nil, nil, err
		}
		if err := gen.WalkCell([]DyadicSpan{cell0, cell1}); err != nil {
			return nil, nil, err
		}
	}
	// The whole chain's own final station is the last span's own last control
	// point on each side, read directly off the ORIGINAL (unsplit) chain
	// rather than the recursion's own bookkeeping: a Bézier interpolates its
	// last control point exactly, and DyadicSpan.split's own "right" half
	// always carries that same point unchanged at every depth (spline_length.go's
	// split leaves right[n-1] = the original last point, untouched by any
	// blend), so reading it here is the identical value the deepest possible
	// recursion would have produced, without walking there.
	//
	// COPIED, not aliased: every other station in the two returned lists comes
	// from DyadicSpan.RatPointAt, whose own doc comment guarantees the *big.Rat it returns
	// shares no storage with the DyadicSpan it was read from. Appending the
	// caller's own last0[len(last0)-1]/last1[len(last1)-1] RatPoint directly
	// would break that guarantee for this ONE station — it would alias the
	// input span's own *big.Rat fields, so a caller mutating a returned station
	// in place would silently corrupt the span it originally passed in. The copy
	// runs through RatPointCopy, which charges for it, because copying a wide
	// rational is real work and every other reconstruction in this walk pays.
	last0 := spans0[len(spans0)-1][len(spans0[len(spans0)-1])-1]
	last1 := spans1[len(spans1)-1][len(spans1[len(spans1)-1])-1]
	end0, err := RatPointCopy(work0, last0)
	if err != nil {
		return nil, nil, err
	}
	end1, err := RatPointCopy(work1, last1)
	if err != nil {
		return nil, nil, err
	}
	gen.Stations[0] = append(gen.Stations[0], end0)
	gen.Stations[1] = append(gen.Stations[1], end1)
	return gen.Stations[0], gen.Stations[1], nil
}

// FreeformChain is ChainStations' answer: agent-docs/spline-design.md §6.2.1's dyadic
// station chain over ONE Tier A span chain, beside the per-cell readings an
// extruded free-form wall's own area and occupied-volume proofs need
// (agent-docs/tessellation-reach-design.md §5).
type FreeformChain struct {
	// stations are exact rational points ON the curve, one per cell boundary in
	// chain order, the chain's START included and its END excluded. The end is
	// carried separately because a consumer emitting one boundary sample per
	// chord needs exactly this list forward, and needs the end only as the
	// FIRST sample of a walk that runs the chain backwards.
	Stations []RatPoint
	// end is the chain's own last cell boundary — the last span's last control
	// point, which a Bézier interpolates exactly — copied so it aliases no
	// input span (PairStations' own rule for the station it reads the same way).
	End RatPoint
	// sagitta is the MEASURED post-subdivision maximum over the accepted cells,
	// never a sum: a curve point lies in exactly one cell, so the widest cell's
	// own departure bounds the whole chain.
	Sagitta float64
	// cellArcUpper is SpanSpeedUpper of each accepted cell's own dyadic
	// sub-span — a proven upper bound on that cell's arc length, since the
	// sub-span carries its own [0, 1] parameter and its speed bounds the
	// integrand over it.
	CellArcUpper []float64
	// cellChordLower is a proven LOWER bound on each accepted cell's own chord
	// length, in the same order. The pair brackets the arc-versus-chord deficit
	// a chorded wall's area slack owes from ABOVE: cellArcUpper[k] −
	// cellChordLower[k] is never below the deficit the chord actually takes.
	CellChordLower []float64
}

// ChainStations chords ONE Tier A span chain at a target sagitta: measure each
// dyadic cell's sagitta, bisect what misses, accept what fits
// (agent-docs/spline-design.md §6.2.1). It is PairStations' single-chain twin —
// same walk, same cap, same charge discipline, same termination argument — and
// the primitive agent-docs/tessellation-reach-design.md §5 gives tessellation.ChordLoop's
// free-form arm. It decides each cell on the sagitta alone: no consumer of a
// single chain reads a parameter-matched departure, so the walk does not
// measure one (SagittaStationWalk.Matched).
//
// Every guard PairStations states for a pair holds here for the one side: an
// empty chain is ErrDegenerate, a chain of more spans than MaxChordsPerWalk
// admits already exceeds the chord count ErrTooManyChords names, and a span
// with no control points at all has no chord to bisect. work is the RECORD's
// own free-form counter; this function mints none and states no cost of its
// own, so every unit spent is spent inside the primitive that does the work.
func ChainStations(spans []BezierSpan, target float64, work *FreeformWork) (FreeformChain, error) {
	if len(spans) == 0 {
		return FreeformChain{}, fmt.Errorf(`%w: a free-form station chain needs at least one span`, decaderr.ErrDegenerate)
	}
	if len(spans) > MaxChordsPerWalk {
		return FreeformChain{}, ErrTooManyChords
	}
	for i := range spans {
		if len(spans[i]) == 0 {
			return FreeformChain{}, fmt.Errorf(
				`%w: a free-form span with no control points at index %d has no chord to bisect`,
				decaderr.ErrDegenerate, i,
			)
		}
	}

	reader := &ChainArcChordReader{}
	gen := NewSagittaStationWalk(target, reader, len(spans), work)
	for i := range spans {
		cell, err := DyadicSpanOf(work, spans[i])
		if err != nil {
			return FreeformChain{}, err
		}
		if err := gen.WalkCell([]DyadicSpan{cell}); err != nil {
			return FreeformChain{}, err
		}
	}
	// The chain's own final boundary, read off the ORIGINAL chain for the
	// reason PairStations states: split's right half carries the last control
	// point unchanged at every depth, so this IS the value the deepest
	// recursion would have produced. Copied, never aliased.
	last := spans[len(spans)-1][len(spans[len(spans)-1])-1]
	end, err := RatPointCopy(work, last)
	if err != nil {
		return FreeformChain{}, err
	}
	return FreeformChain{
		Stations:       gen.Stations[0],
		End:            end,
		Sagitta:        gen.SagittaUpper,
		CellArcUpper:   reader.ArcUpper,
		CellChordLower: reader.ChordLower,
	}, nil
}

// SagittaStationWalk accumulates one PairStations call's own state across its
// recursive cell walk: the two growing station lists, the parallel per-cell
// matchedDelta list, the running sagitta maximum, and the two shared counts
// the hard cap reads — chords, the cells already accepted as chords of the
// finished chain, and frontier, the cells created but not yet accepted. Their
// sum is a proven lower bound on the chord count the finished chain carries:
// it starts at one cell per span, holds steady when a cell is accepted, and
// rises by one per split (PairStations' own doc comment).
type SagittaStationWalk struct {
	Target float64
	// works and stations are indexed BY SIDE, so the walk is written once for
	// any number of sides and its two consumers differ only in how many they
	// hand it: PairStations two, ChainStations one. Every side is measured,
	// accepted and bisected TOGETHER, which is what keeps a pair's two station
	// lists on one shared set of dyadic cell boundaries.
	Works    []*FreeformWork
	Stations [][]RatPoint
	// reader records whatever ELSE each accepted cell owes its own consumer —
	// the pair's matched-delta obligation, the chain's arc/chord bracket —
	// which is the only place the two consumers' arithmetic differs.
	Reader StationCellReader
	// Matched, when true, makes the walk measure each side's
	// SpanMatchedDeltaUpper beside its sagitta, bisect while EITHER exceeds the
	// target, and hand the accepted cell's matched value (the larger of the
	// sides') to the reader. PairChainStations sets it, so every accepted
	// cell's parameter-matched departure is at or below the target as a
	// property of the walk (agent-docs/loft-gear-bounds-design.md §5). PairStations
	// and ChainStations leave it false: they decide on the sagitta alone and
	// hand the reader 0.
	Matched      bool
	SagittaUpper float64
	Chords       int
	Frontier     int
	// Limit is the chord-count ceiling the walk refuses past. Zero means
	// MaxChordsPerWalk, and a larger value is clamped to it.
	Limit int
	// Underivable, when non-nil, is returned as soon as a measured sagitta is
	// non-finite, rather than bisecting that cell until the ceiling refuses.
	Underivable error
}

// limit is the chord-count ceiling WalkCell reads: Limit when it is positive
// and below MaxChordsPerWalk, and MaxChordsPerWalk otherwise.
func (g *SagittaStationWalk) limit() int {
	if g.Limit > 0 && g.Limit < MaxChordsPerWalk {
		return g.Limit
	}
	return MaxChordsPerWalk
}

// NewSagittaStationWalk opens a walk over sides sides, one per counter, with
// the frontier seeded at one cell per span: the sum chords+frontier is a proven
// lower bound on the finished chain's own chord count from the first cell on
// (this file's own cap argument).
func NewSagittaStationWalk(target float64, reader StationCellReader, spans int, works ...*FreeformWork) *SagittaStationWalk {
	return &SagittaStationWalk{
		Target:   target,
		Works:    works,
		Stations: make([][]RatPoint, len(works)),
		Reader:   reader,
		Frontier: spans,
	}
}

// StationCellReader is the per-consumer half of the shared station walk. The
// walk itself measures, accepts and bisects; the reader takes each ACCEPTED
// cell's own reconstructed spans, one per side, and records the reading its
// consumer owes for that cell — in the same left-to-right cell order the
// station lists carry. matched is the cell's parameter-matched departure the
// walk already measured to decide it (SagittaStationWalk.Matched), or 0 when
// the walk does not measure one.
//
// It states no cost of its own either: every implementation below spends its
// units inside the metered primitive that does the work.
type StationCellReader interface {
	AcceptCell(spans []BezierSpan, matched float64, works []*FreeformWork) error
}

// PairMatchedDeltaReader is PairStations' reading: internal/proofbound/bounds.go's
// proofbound.CellChordCurveAreaUpper matchedDeltaUpper obligation (F1's rule), one entry
// per accepted cell, the larger of the two sides' own PARAMETER-MATCHED bounds
// under the span-uniform native fraction — never the SET-distance sagitta the
// walk measured to decide the cell. PairStations' walk decides on the sagitta
// alone and hands it no matched value, so it measures one here.
type PairMatchedDeltaReader struct {
	MatchedDelta []float64
}

// PairCellReader is PairChainStations' reading: the per-cell matched-departure
// bound PairMatchedDeltaReader also records, plus each side's SpanSpeedUpper
// and SpanTangentEnergyUpper over the same accepted dyadic sub-span, in the
// same left-to-right cell order. PairChainStations' walk measures the matched
// value to decide the cell (SagittaStationWalk.Matched), so this reader
// records the value it is handed rather than measuring it again.
type PairCellReader struct {
	MatchedDelta  []float64
	ArcUpper      [2][]float64
	TangentEnergy [2][]float64
}

func (r *PairCellReader) AcceptCell(spans []BezierSpan, matched float64, works []*FreeformWork) error {
	r.MatchedDelta = append(r.MatchedDelta, matched)
	for side := range r.ArcUpper {
		arc, err := SpanSpeedUpper(works[side], spans[side])
		if err != nil {
			return err
		}
		energy, err := SpanTangentEnergyUpper(works[side], spans[side])
		if err != nil {
			return err
		}
		r.ArcUpper[side] = append(r.ArcUpper[side], arc)
		r.TangentEnergy[side] = append(r.TangentEnergy[side], energy)
	}
	return nil
}

// ChainArcChordReader is ChainStations' reading: each accepted cell's own
// arc-length upper bound and chord-length lower bound, the pair
// agent-docs/tessellation-reach-design.md §5 turns into a chorded wall's area slack.
// Both are proven in their own direction, so their difference never
// understates the deficit the chord actually takes.
//
// A cell of fewer than two control points has no chord and no hodograph, so
// both readings are 0 — SpanSpeedUpper's own guard, restated for the chord.
type ChainArcChordReader struct {
	ArcUpper   []float64
	ChordLower []float64
}

func (r *ChainArcChordReader) AcceptCell(spans []BezierSpan, _ float64, works []*FreeformWork) error {
	span, work := spans[0], works[0]
	arc, err := SpanSpeedUpper(work, span)
	if err != nil {
		return err
	}
	chord := 0.0
	if len(span) >= 2 {
		squared, err := SpanChordSquared(work, span)
		if err != nil {
			return err
		}
		chord, err = ChargedRatSqrtDown(work, squared)
		if err != nil {
			return err
		}
	}
	r.ArcUpper = append(r.ArcUpper, arc)
	r.ChordLower = append(r.ChordLower, chord)
	return nil
}

// WalkCell measures one dyadic cell — one sub-span per side — and either
// accepts it, appending its own start station on each side, handing the
// accepted cell's reconstructed spans to the reader, and folding its measured
// sagitta into the running maximum, or bisects it on EVERY side together and
// recurses left then right, in that order, which is what makes the whole walk
// deterministic and left-to-right (PairStations' own doc comment).
//
// The cell is decided on the WIDEST side's sagitta, so a pair is bisected
// whenever either side misses the target and the two sides keep the identical
// set of dyadic cell boundaries. A one-sided walk (ChainStations) reads that
// same maximum over its single side. A walk with Matched set also measures
// each side's SpanMatchedDeltaUpper and bisects while that maximum misses the
// target, because a sagitta under the target bounds the matched departure by
// no fixed factor (TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses); the
// accepted cell's matched value goes to the reader with the spans.
//
// DyadicSpanSagittaUpperWithSpan reconstructs the accepted cell's own control
// points exactly (DyadicSpan.RatPointAt), so every
// reading the reader takes measures the SAME dyadic sub-span the sagitta
// measurement above already split to, never a re-derived one. That matters
// most for the pair's matched-delta obligation, which is a PARAMETER-MATCHED
// bound under the span-uniform native fraction and never the SET-distance
// sagitta this walk decided the cell on: the two coincide only for a line or a
// circular arc under its own uniform-angle parametrization (internal/proofbound/bounds.go's own
// doc comment), neither of which this file ever reaches.
//
// Accepting moves this cell off the frontier and into the chord count, leaving
// their sum unchanged; splitting raises it by one, which is why the cap is
// read here and only here, before the split runs. Since the recursion below is
// reachable only through that split, the same read is what bounds the walk's own
// depth and breadth (PairStations' own doc comment states the termination
// argument in full).
//
// NO CHARGE IS SPENT HERE. Every measurement, reconstruction and bisection below
// charges its own counter from inside the primitive that performs it, so this
// function never restates what any of them costs or how often it runs.
func (g *SagittaStationWalk) WalkCell(cells []DyadicSpan) error {
	spans := make([]BezierSpan, len(cells))
	worst := 0.0
	matched := 0.0
	for i, cell := range cells {
		sag, span, err := DyadicSpanSagittaUpperWithSpan(g.Works[i], cell)
		if err != nil {
			return err
		}
		spans[i] = span
		worst = math.Max(worst, sag)
		if !g.Matched {
			continue
		}
		md, err := SpanMatchedDeltaUpper(g.Works[i], span)
		if err != nil {
			return err
		}
		matched = math.Max(matched, md)
	}
	if g.Underivable != nil && (proofbound.IsNonFinite(worst) || proofbound.IsNonFinite(matched)) {
		return g.Underivable
	}
	if worst <= g.Target && matched <= g.Target {
		if err := g.Reader.AcceptCell(spans, matched, g.Works); err != nil {
			return err
		}
		starts := make([]RatPoint, len(cells))
		for i, cell := range cells {
			start, err := cell.RatPointAt(g.Works[i], 0)
			if err != nil {
				return err
			}
			starts[i] = start
		}
		g.Frontier--
		g.Chords++
		for i, start := range starts {
			g.Stations[i] = append(g.Stations[i], start)
		}
		g.SagittaUpper = math.Max(g.SagittaUpper, worst)
		return nil
	}

	if g.Chords+g.Frontier+1 > g.limit() {
		return ErrTooManyChords
	}
	lefts := make([]DyadicSpan, len(cells))
	rights := make([]DyadicSpan, len(cells))
	for i, cell := range cells {
		left, right, err := cell.Split(g.Works[i])
		if err != nil {
			return err
		}
		lefts[i], rights[i] = left, right
	}
	g.Frontier++
	if err := g.WalkCell(lefts); err != nil {
		return err
	}
	return g.WalkCell(rights)
}
