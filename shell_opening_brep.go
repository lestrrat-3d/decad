package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
func evalSideOpeningPrism(ctx context.Context, d *Document, ref producerID, pp prismPayload, sec prismshell.Section) (*Body, error) {
	delta := sec.Delta
	if sec.CutGap > 0 {
		delta = proofbound.AbsSumUpper(delta, proofbound.ProductUpper(3, sec.CutGap))
		if proofbound.IsNonFinite(delta) {
			return nil, offset2d.ErrUnbounded
		}
	}
	return evalPrismContext(ctx, d, ref, prismPayload{
		profile:      sec.Wall,
		frame:        pp.frame,
		z0:           pp.z0,
		z1:           pp.z1,
		z0Delta:      pp.z0Delta,
		z1Delta:      pp.z1Delta,
		xform:        pp.xform,
		sectionDelta: delta,
	}, freeform.NewFreeformWork())
}

// sideOpeningBrep records the stack's built faces under the receiver frame and
// placement. The section and slab construction runs in internal/prismshell.
func sideOpeningBrep(ctx context.Context, budget *proofbound.WorkBudget, pp prismPayload,
	sec prismshell.Section, removedStart, removedEnd bool, s, tmm, tDelta float64) (brepPayload, error) {
	geom, levels, delta, err := prismshell.Stack(budget, prismshell.StackInput{
		Section: sec, Z0: pp.z0, Z1: pp.z1, Z0Delta: pp.z0Delta, Z1Delta: pp.z1Delta,
		RemovedStart: removedStart, RemovedEnd: removedEnd, Sense: s,
		Thickness: tmm, ThicknessDelta: tDelta,
	})
	var bp brepPayload
	if err == nil {
		bp, err = stackedBrepRecord(ctx, budget, geom, levels, pp.frame, pp.xform, delta)
	}
	if errors.Is(err, brepgeom.ErrStackedWallMiss) {
		return brepPayload{}, fmt.Errorf(`%w: the stacked record build does not state this side opening's faces (shell-opening SO5)`, ErrUnsupported)
	}
	return bp, err
}
