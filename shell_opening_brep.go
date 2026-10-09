package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
)

// This file records a prism side opening (docs/shell-opening-design.md §4):
// with both caps removed the wall section W swept over the receiver's own
// sweep is a prism (BO1); otherwise the result is a stack of at most three
// slabs — a kept cap's slab over the cap region, the wall slab over W — which
// internal/stackedbrep restates into a brepPayload (BO2) with flush walls
// merged into one planar face, the removed face's own plane holding the
// opening as its hole. Every loop is decad's own record, so no sketch scene
// is built; a topology the engine does not cover is SO5, never a fallback.

// evalSideOpeningPrism is BO1: W over the receiver's interval, frame and
// placement, carrying the offset's section displacement and, where a rim on
// a removed arc names its cut at a float parameter, three times that cut's
// gap on top (§4.4).
func evalSideOpeningPrism(ctx context.Context, d *Document, ref producerID, pp prismPayload, sec sideOpeningSection) (*Body, error) {
	delta := sec.delta
	if sec.cutGap > 0 {
		delta = proofbound.AbsSumUpper(delta, proofbound.ProductUpper(3, sec.cutGap))
		if proofbound.IsNonFinite(delta) {
			return nil, offset2d.ErrUnbounded
		}
	}
	return evalPrismContext(ctx, d, ref, prismPayload{
		profile:      sec.wall,
		frame:        pp.frame,
		z0:           pp.z0,
		z1:           pp.z1,
		z0Delta:      pp.z0Delta,
		z1Delta:      pp.z1Delta,
		xform:        pp.xform,
		sectionDelta: delta,
	}, freeform.NewFreeformWork())
}

// shellLevel is one derived level of a shell's stack: from moved by `by`,
// carrying its source end's displacement, the thickness conversion and this
// float sum's own rounding (docs/shell-opening-design.md §3).
func shellLevel(from, delta, by, tDelta float64) stackedUnionLevel {
	to := from + by
	return stackedUnionLevel{held: to, delta: proofbound.AbsSumUpper(delta, tDelta, proofarith.AddRoundError(from, by, to))}
}

// sideOpeningBrep states BO2 (§4.1–§4.5). Inward the stack is the cap region
// P on [z0, z0 + t] where the start cap is kept, W over the cavity's height,
// and P on [z1 − t, z1] where the end cap is kept; outward O on [z0 − t, z0],
// W on [z0, z1] and O on [z1, z1 + t]. Each interface between a cap slab and
// the wall slab exposes the cavity region once, facing the cavity: up at the
// floor, down at the ceiling. A reflex end vertex is marked at both cavity
// levels (Engine.Event), so the cavity's walk along the removed face's
// carrier splits where the rim face meets the floor strip. Every face carries
// the larger of the offset's displacement and the engine's largest
// canonical-vertex allowance (§4.4). An engine miss is SO5, ErrUnsupported.
func sideOpeningBrep(ctx context.Context, budget *proofbound.WorkBudget, pp prismPayload, sec sideOpeningSection, removedStart, removedEnd bool, s, tmm, tDelta float64) (brepPayload, error) {
	bottom := stackedUnionLevel{held: pp.z0, delta: pp.z0Delta}
	top := stackedUnionLevel{held: pp.z1, delta: pp.z1Delta}
	var levels []stackedUnionLevel
	var regions []profileRecord
	wallAt := 0
	if s > 0 {
		// Inward the kept caps' slabs eat t of the sweep at each end.
		levels = append(levels, bottom)
		if !removedStart {
			levels = append(levels, shellLevel(pp.z0, pp.z0Delta, tmm, tDelta))
			regions = append(regions, sec.caps)
			wallAt = 1
		}
		regions = append(regions, sec.wall)
		if !removedEnd {
			levels = append(levels, shellLevel(pp.z1, pp.z1Delta, -tmm, tDelta))
			regions = append(regions, sec.caps)
		}
		levels = append(levels, top)
	} else {
		// Outward the kept caps' slabs extend t beyond each end.
		if !removedStart {
			levels = append(levels, shellLevel(pp.z0, pp.z0Delta, -tmm, tDelta))
			regions = append(regions, sec.caps)
			wallAt = 1
		}
		levels = append(levels, bottom)
		regions = append(regions, sec.wall)
		levels = append(levels, top)
		if !removedEnd {
			levels = append(levels, shellLevel(pp.z1, pp.z1Delta, tmm, tDelta))
			regions = append(regions, sec.caps)
		}
	}
	for i := 1; i < len(levels); i++ {
		if !(levels[i].held > levels[i-1].held) {
			return brepPayload{}, fmt.Errorf(`%w: a side opening's slab levels do not ascend (shell-opening SO3)`, ErrDegenerate)
		}
	}
	bp, err := sideOpeningRecord(ctx, budget, pp, sec, levels, regions, wallAt)
	if errors.Is(err, brepgeom.ErrStackedWallMiss) {
		return brepPayload{}, fmt.Errorf(`%w: the stacked record build does not state this side opening's faces (shell-opening SO5)`, ErrUnsupported)
	}
	return bp, err
}

// sideOpeningRecord runs the engine over the slabs between consecutive
// levels, regions[k] being slab k's one region and wallAt the wall slab.
func sideOpeningRecord(ctx context.Context, budget *proofbound.WorkBudget, pp prismPayload, sec sideOpeningSection, levels []stackedUnionLevel, regions []profileRecord, wallAt int) (brepPayload, error) {
	n := len(regions)
	held := make([]float64, len(levels))
	for i, l := range levels {
		held[i] = l.held
	}
	geom := stackedbrep.NewEngine(held)
	for _, region := range regions {
		loop, err := geom.LoopOf(brepgeom.Profile{Outer: region.Outer, Holes: region.Holes}, budget)
		if err != nil {
			return brepPayload{}, err
		}
		geom.SlabLoops = append(geom.SlabLoops, []stackedbrep.Loop{loop})
	}
	if err := geom.AddFace(geom.SlabLoops[0][0], 0, false); err != nil {
		return brepPayload{}, err
	}
	if err := geom.AddFace(geom.SlabLoops[n-1][0], n, true); err != nil {
		return brepPayload{}, err
	}
	// The cavity region at each interface: the floor under the wall slab
	// faces up, the ceiling over it faces down.
	for _, iface := range []struct {
		level int
		up    bool
	}{{wallAt, true}, {wallAt + 1, false}} {
		if iface.level <= 0 || iface.level >= n {
			continue
		}
		loop, err := geom.LoopOf(brepgeom.Profile{Outer: sec.cavity.Outer, Holes: sec.cavity.Holes}, budget)
		if err != nil {
			return brepPayload{}, err
		}
		if err := geom.AddFace(loop, iface.level, iface.up); err != nil {
			return brepPayload{}, err
		}
	}
	for _, v := range sec.corners {
		geom.Event(v, wallAt)
		geom.Event(v, wallAt+1)
	}
	geom.RecordJunctions(n)
	delta := math.Max(sec.delta, geom.Allow())
	return stackedBrepRecord(ctx, budget, geom, levels, pp.frame, pp.xform, delta)
}
