package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayBlindChordStructure is the charged column chording and exposed-patch
// structure of one sealed blind stacked result. Face assignment and mesh
// proof happen after this stage.
type ReplayBlindChordStructure struct {
	Columns []WallColumn
	Rows    [][]SlabLoop
	Patches []Patch
	Loops   []tessellation.ChordSamples[int]
}

// ReplayBlindChordsWithSource chords the supported sealed result after the
// caller binds host to this result's immutable public Cut input. New circular
// holes are resolved from the output's own sealed analytic record.
func ReplayBlindChordsWithSource(ctx context.Context, body, host *topo.Body,
	requestedChord float64, budget *proofbound.ReplayBudget) (ReplayBlindChordStructure, bool, error) {
	if body == nil || host == nil || budget == nil || requestedChord <= 0 ||
		proofbound.IsNonFinite(requestedChord) {
		return ReplayBlindChordStructure{}, false, nil
	}
	columns, rows, patches, ok, err := ReplayBlindTessStructure(ctx, body, budget)
	if err != nil || !ok {
		return ReplayBlindChordStructure{}, false, err
	}
	sp, ok := body.Payload.(Payload)
	if !ok {
		return ReplayBlindChordStructure{}, false, nil
	}
	pp, ok := host.Payload.(prismrecord.Payload)
	if !ok || pp.Walks == nil || !host.HasStrictPrismWalkSeal() {
		return ReplayBlindChordStructure{}, false, nil
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return ReplayBlindChordStructure{}, false, err
	}
	if !replaySameFrameBits(sp.Frame, pp.Frame) || !replaySameTransformBits(sp.Xform, pp.Xform) {
		return ReplayBlindChordStructure{}, false, nil
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(momentinput.Profile{}))); err != nil {
		return ReplayBlindChordStructure{}, false, err
	}
	sealed, err := host.ReplayPrismBuildPayload(ctx, pp.Frame, pp.Xform,
		pp.Z0, pp.Z1, pp.Z0Delta, pp.Z1Delta, pp.SectionDelta,
		pp.Z0Denoted, pp.Z1Denoted,
		[]sectionrecord.AnalyticProfileView{pp.Profile.AnalyticSealView()}, budget)
	if err != nil || !sealed {
		return ReplayBlindChordStructure{}, false, err
	}
	if err := budget.SpendWork(ctx, 4); err != nil {
		return ReplayBlindChordStructure{}, false, err
	}
	chord := requestedChord
	if sp.SectionDelta > 0 {
		chord = freeform.DownRound(freeform.DownRound(chord - sp.SectionDelta))
	}
	if chord <= 0 || proofbound.IsNonFinite(chord) {
		return ReplayBlindChordStructure{}, false, nil
	}
	if err := budget.ReserveArray(ctx, uint64(len(columns)),
		uint64(unsafe.Sizeof(tessellation.ChordSamples[int]{}))); err != nil {
		return ReplayBlindChordStructure{}, false, err
	}
	loops := make([]tessellation.ChordSamples[int], len(columns))
	for ci, col := range columns {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindChordStructure{}, false, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(col.Loop.Segments)),
			uint64(unsafe.Sizeof(survey2d.SegmentWalk{}))); err != nil {
			return ReplayBlindChordStructure{}, false, err
		}
		walks := make([]survey2d.SegmentWalk, len(col.Loop.Segments))
		if col.LoopIndex <= len(pp.Profile.Holes) {
			matches, err := pp.Walks.LoopMatchesReplay(ctx, col.LoopIndex, col.Loop, budget)
			if err != nil || !matches {
				return ReplayBlindChordStructure{}, false, err
			}
			for i := range walks {
				walk, ok, err := prismrecord.ReplayPrismBuiltWalk(ctx, host, pp.Walks, col.LoopIndex, i, budget)
				if err != nil || !ok {
					return ReplayBlindChordStructure{}, false, err
				}
				walks[i] = walk
			}
		} else {
			if len(walks) != 1 {
				return ReplayBlindChordStructure{}, false, nil
			}
			circle, ok := col.Loop.Segments[0].(sectionrecord.CircleSeg)
			if !ok {
				return ReplayBlindChordStructure{}, false, nil
			}
			walk, err := boundarywalk.WholeCircleWalkReplay(ctx, circle, budget)
			if err != nil {
				return ReplayBlindChordStructure{}, false, err
			}
			walks[0] = walk
		}
		if col.Start < 0 || col.End >= len(sp.Slabs) || col.Start > col.End {
			return ReplayBlindChordStructure{}, false, decaderr.ErrUnsupported
		}
		height := sp.Slabs[col.End].Z1 - sp.Slabs[col.Start].Z0
		loops[ci], err = tessellation.ReplayChordLoop(ctx, col.Loop, walks, chord, height, budget)
		if err != nil {
			return ReplayBlindChordStructure{}, false, err
		}
	}
	return ReplayBlindChordStructure{Columns: columns, Rows: rows, Patches: patches, Loops: loops}, true, nil
}

// ReplayBlindChordClearanceWithSource carries the same consumer meter through
// source-bound chording and every slab's cross-loop clearance check.
func ReplayBlindChordClearanceWithSource(ctx context.Context, body, host *topo.Body,
	requestedChord float64, budget *proofbound.ReplayBudget) (ReplayBlindChordStructure, bool, error) {
	structure, ok, err := ReplayBlindChordsWithSource(ctx, body, host, requestedChord, budget)
	if err != nil || !ok {
		return ReplayBlindChordStructure{}, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(structure.Loops)),
		uint64(unsafe.Sizeof(tessellation.StackedRing[int]{}))); err != nil {
		return ReplayBlindChordStructure{}, false, err
	}
	rings := make([]tessellation.StackedRing[int], len(structure.Loops))
	for i, loop := range structure.Loops {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindChordStructure{}, false, err
		}
		rings[i] = tessellation.StackedRing[int]{Samples: loop.Samples, Sag: loop.MaxSag}
	}
	for _, row := range structure.Rows {
		if err := budget.ReserveArray(ctx, uint64(len(row)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return ReplayBlindChordStructure{}, false, err
		}
		columns := make([]int, len(row))
		for i, entry := range row {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return ReplayBlindChordStructure{}, false, err
			}
			columns[i] = entry.Column
		}
		if err := tessellation.ReplayStackedSlabClearance(ctx, rings, columns, budget); err != nil {
			return ReplayBlindChordStructure{}, false, err
		}
	}
	return structure, true, nil
}
