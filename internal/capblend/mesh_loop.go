package capblend

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// LoopMesh is one recorded loop's complete chording: the walks the
// build resolved, the offset joins the cap contour is trimmed at, the ONE chord
// count per walk every level of that loop reads, and the plane-local samples
// and mesh vertices that count produced.
//
// A loop chamfered on either cap carries a cap contour ring as well as its side
// ring. The contour does not depend on WHICH cap the loop is chamfered on — the
// in-plane offset is the same either way (MixedOffsetProfile) —
// so the samples are resolved once and lifted to whichever cap level(s) the
// selection named.
type LoopMesh struct {
	LoopIndex int
	Loop      sectionrecord.LoopRecord
	Walks     []survey2d.SideWalk
	// Joins is the per-corner offset join capband.OffsetJoins resolves, nil for an
	// unchamfered loop and for the one cornerless closed circle (Whole).
	Joins          []offset2d.Join
	Whole          bool
	OnStart, OnEnd bool
	Chamfered      bool
	ZLo, ZHi       proofbound.BoundedScalar
	Count          []int     // the shared chord count of walk i
	SideSag        []float64 // walk i's own side-directrix sagitta at that count
	CapSag         []float64 // walk i's cap-directrix sagitta at that count
	CapRadius      []float64
	CapTh0, CapTh1 []float64
	ArcCount       []int // a reflex corner's connector-arc chord count, 0 elsewhere
	ArcSag         []float64
	ArcTh0, ArcTh1 []float64
	LocusGap       []float64 // corner i's ruling-versus-locus gap, both caps alike
	SidePts        []sectionrecord.Point2
	SideBound      []proofbound.WalkEndBound
	SideStart      []int // walk i's first side sample
	CapPts         []sectionrecord.Point2
	CapBound       []proofbound.WalkEndBound
	CapWallStart   []int // walk i's first cap sample
	// CapMotion is each cap sample's plane-local displacement from the point
	// docs/tessellation-reach-design.md §7's ideal polyhedron B1 places there,
	// filled by CapMotion only when the occupied-volume proof is built.
	// It answers a different question from CapBound — a station's gap from the
	// held offset circle — and never replaces it.
	CapMotion      []float64
	CapArcStart    []int // corner i's first connector-arc sample, −1 when not reflex
	SideLo, SideHi []int // mesh vertices of each side sample at ZLo and ZHi
	CapLoV, CapHiV []int // mesh vertices of each cap sample at z0 and z1
}

// Proof passes the numeric loop record without copying its walk or sample slices.
func (lm *LoopMesh) Proof() tessellation.CapBlendLoopProof {
	return tessellation.CapBlendLoopProof{
		Walks: lm.Walks, Count: lm.Count,
		SideSag: lm.SideSag, CapSag: lm.CapSag,
		CapRadius: lm.CapRadius, CapTh0: lm.CapTh0, CapTh1: lm.CapTh1,
		ArcCount: lm.ArcCount, ArcSag: lm.ArcSag,
		ArcTh0: lm.ArcTh0, ArcTh1: lm.ArcTh1,
		ZLo: lm.ZLo, ZHi: lm.ZHi,
		Chamfered: lm.Chamfered, OnStart: lm.OnStart, OnEnd: lm.OnEnd,
	}
}

// CapMotion reads the loop's numeric record through internal/tessellation.
func CapMotion(budget *proofbound.WorkBudget, cbp Payload, lm *LoopMesh) error {
	in := tessellation.CapBlendMotionInput{
		CapBlendLoopProof: lm.Proof(),
		Loop:              lm.LoopIndex, Segments: lm.Loop.Segments,
		CapPts: slices.Clone(lm.CapPts), CapWallStart: lm.CapWallStart,
		Whole: lm.Whole, D: cbp.LoopOffset(lm.LoopIndex), DDelta: cbp.LoopSetback(lm.LoopIndex).DcDelta,
		Amounts: cbp.WalkAmounts(lm.LoopIndex, lm.Walks, cbp.LoopOffset(lm.LoopIndex)),
	}
	in.BandDelta[0], in.HasBandDelta[0] = cbp.BandDelta[BandKey{Loop: lm.LoopIndex, Start: true}]
	in.BandDelta[1], in.HasBandDelta[1] = cbp.BandDelta[BandKey{Loop: lm.LoopIndex, Start: false}]
	motion, err := tessellation.CapBlendCapMotion(budget, in, stationbound.CapOffsetStationBound)
	if err != nil {
		return err
	}
	lm.CapMotion = motion
	return nil
}

// ChordVolume adapts the resolved loops to the slice-wise proof.
func ChordVolume(cbp Payload, lms []LoopMesh) float64 {
	loops := make([]tessellation.CapBlendLoopProof, len(lms))
	for i := range lms {
		loops[i] = lms[i].Proof()
	}
	height := [2]float64{
		proofbound.AbsSumUpper(cbp.Start.Ds, cbp.Start.DsDelta),
		proofbound.AbsSumUpper(cbp.End.Ds, cbp.End.DsDelta),
	}
	if cbp.Draft {
		h := cbp.DraftBandHeightUpper()
		height = [2]float64{h, h}
	}
	return tessellation.CapBlendChordVolume(height, loops)
}

// ChordLoop resolves ONE loop's walks the way capblend.BuildBand does and
// chords each of them ONCE, at the single count every level of that loop then
// reads (docs/tessellation-reach-design.md §7).
//
// For a circular wall of a chamfered loop the count is the larger of what the
// wall's own arc needs and what its cap-level offset arc needs, so the side
// ring, the band patch and the cap contour all carry the same number of
// stations and the band's strips meet along shared vertices rather than along
// two independently sampled polylines. A straight wall needs one sample; a
// reflex corner's connector arc is a directrix of its own and chords with its
// own count.
func ChordLoop(ctx context.Context, budget *proofbound.WorkBudget, cbp Payload, li int, loop sectionrecord.LoopRecord, chord float64, work *freeform.FreeformWork) (LoopMesh, error) {
	if err := ctx.Err(); err != nil {
		return LoopMesh{}, err
	}
	cl, err := OneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return LoopMesh{}, err
	}
	walks := cl.Walks
	n := len(walks)
	lm := LoopMesh{
		LoopIndex: li,
		Loop:      loop,
		Walks:     walks,
		Whole:     n == 1 && walks[0].Closed,
		OnStart:   cbp.StartLoops[li],
		OnEnd:     cbp.EndLoops[li],
	}
	lm.Chamfered = lm.OnStart || lm.OnEnd

	lm.ZLo = proofbound.MeasuredScalar(cbp.Z0, cbp.Z0Delta)
	lm.ZHi = proofbound.MeasuredScalar(cbp.Z1, cbp.Z1Delta)
	if lm.OnStart {
		lm.ZLo = proofbound.BoundedAdd(lm.ZLo, proofbound.MeasuredScalar(cbp.Start.Ds, cbp.Start.DsDelta))
	}
	if lm.OnEnd {
		lm.ZHi = proofbound.BoundedSub(lm.ZHi, proofbound.MeasuredScalar(cbp.End.Ds, cbp.End.DsDelta))
	}
	if cbp.Draft {
		// A draft band's side level is the near cap itself, and no trimmed
		// wall separates two side rings.
		lm.ZLo = cbp.DraftNearLevel()
		lm.ZHi = lm.ZLo
	}

	if lm.Chamfered && !lm.Whole {
		lm.Joins, err = cbp.OffsetJoins(budget, li, cl, cbp.LoopOffset(li))
		if err != nil {
			return LoopMesh{}, err
		}
	}

	chords, err := tessellation.ChordCapBlendLoop(tessellation.CapBlendChordInput{
		Walks: walks, Segments: loop.Segments, Joins: SampleJoins(lm.Joins),
		Whole: lm.Whole, Chamfered: lm.Chamfered, D: cbp.LoopOffset(li), Chord: chord,
		Setback: ProofSetback(cbp.LoopSetback(li)), FootDelta: cbp.LoopBandDelta(li),
	}, budget, func(w survey2d.SideWalk, d float64) (float64, error) {
		if cbp.Fillet && filletband.SphereWalk(w, d) {
			return 0, nil
		}
		// A subset draft's kept wall keeps its own radius (offset2d.DraftWalkAmounts).
		if cbp.Draft {
			return capband.WallRadius(w, offset2d.DraftWalkAmounts(cbp.DraftKept, li, []survey2d.SideWalk{w}, d)[0], offset2d.ShellTol)
		}
		return capband.BandRadius(w, d, offset2d.ShellTol)
	}, capband.WallSweep)
	if err != nil {
		return LoopMesh{}, err
	}
	lm.Count, lm.SideSag, lm.CapSag = chords.Count, chords.SideSag, chords.CapSag
	lm.CapRadius, lm.CapTh0, lm.CapTh1 = chords.CapRadius, chords.CapTh0, chords.CapTh1
	lm.ArcCount, lm.ArcSag = chords.ArcCount, chords.ArcSag
	lm.ArcTh0, lm.ArcTh1, lm.LocusGap = chords.ArcTh0, chords.ArcTh1, chords.LocusGap

	if err := emitSamples(budget, cbp, &lm); err != nil {
		return LoopMesh{}, err
	}
	return lm, nil
}

// emitSamples adapts the resolved offset joins to the shared sampler.
func emitSamples(budget *proofbound.WorkBudget, cbp Payload, lm *LoopMesh) error {
	samples, err := tessellation.SampleCapBlend(tessellation.CapBlendSampleInput{
		Walks: lm.Walks, Segments: lm.Loop.Segments, Joins: SampleJoins(lm.Joins),
		Count: lm.Count, CapRadius: lm.CapRadius, CapTh0: lm.CapTh0, CapTh1: lm.CapTh1,
		ArcCount: lm.ArcCount, ArcTh0: lm.ArcTh0, ArcTh1: lm.ArcTh1,
		Whole: lm.Whole, Chamfered: lm.Chamfered, D: cbp.LoopOffset(lm.LoopIndex),
	}, budget, stationbound.ChordStationBound)
	if err != nil {
		return err
	}
	lm.SidePts, lm.SideBound, lm.SideStart = slices.Clone(samples.SidePts), samples.SideBound, samples.SideStart
	lm.CapPts, lm.CapBound = slices.Clone(samples.CapPts), samples.CapBound
	lm.CapWallStart, lm.CapArcStart = samples.CapWallStart, samples.CapArcStart
	return nil
}

func SampleJoins(joins []offset2d.Join) []tessellation.CapBlendJoin {
	if joins == nil {
		return nil
	}
	out := make([]tessellation.CapBlendJoin, len(joins))
	for i, j := range joins {
		out[i] = tessellation.CapBlendJoin{
			Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV, M: j.M, PA: j.PA, PB: j.PB,
		}
	}
	return out
}

func ProofSetback(s Setback) tessellation.CapBlendSetback {
	return tessellation.CapBlendSetback{
		AxialUpper: s.AxialUpper(), Dc: s.Dc, DcDelta: s.DcDelta,
		Ds: s.Ds, DsDelta: s.DsDelta,
	}
}
