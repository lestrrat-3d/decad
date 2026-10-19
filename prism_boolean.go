package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tryPrismBoolean keeps the root-only holed Intersect route ahead of the
// analytic entry, then passes the existing payloads to prismcells.
func tryPrismBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body) (prismPayload, bool, error) {
	if op == meshbool.OpIntersect {
		if result, ok, err := tryPrismHoledIntersect(ctx, a, b); ok || err != nil {
			return result, ok, err
		}
	}
	return prismcells.TryAnalytic(ctx, op, a.n.Payload, b.n.Payload)
}

// admitPrismPairBudget keeps the root body callers on their original budget.
func admitPrismPairBudget(budget *proofbound.WorkBudget, op meshbool.OperationKind, a, b *Body) (prismPayload, prismPayload, bool, error) {
	return prismcells.AdmitPairBudget(budget, op, a.n.Payload, b.n.Payload)
}

// tryPrismHoledIntersect admits one cleanly nested, one-hole prism under a
// hole-free prism. Intersect is symmetric, so put the hole-free operand first.
// The shared Intersect admission used by the overlap-area reading stays
// hole-free; this body-only arm uses the same gates and work cap explicitly.
func tryPrismHoledIntersect(ctx context.Context, a, b *Body) (prismPayload, bool, error) {
	ah, aok := a.n.Payload.(prismPayload)
	bh, bok := b.n.Payload.(prismPayload)
	if !aok || !bok {
		return prismPayload{}, false, nil
	}
	if len(ah.Profile.Holes) == 1 && len(bh.Profile.Holes) == 0 {
		a, b = b, a
	} else if len(ah.Profile.Holes) != 0 || len(bh.Profile.Holes) != 1 {
		return prismPayload{}, false, nil
	}

	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return prismPayload{}, false, err
	}
	pa, pb, ok, err := admitPrismPairBudget(budget, meshbool.OpIntersect, a, b)
	if err != nil || !ok {
		return prismPayload{}, false, err
	}
	for _, profile := range []profileRecord{pa.Profile, pb.Profile} {
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, profile.Outer, profile.Holes)
		if err != nil {
			return prismPayload{}, false, err
		}
		if trimmed {
			return prismPayload{}, false, nil
		}
	}
	if !prismplacement.IntersectZIntervalOverlaps(pa.PlacementOperand(), pb.PlacementOperand()) {
		return prismPayload{}, false, nil
	}
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, pa.Profile, pb.Profile)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !withinCap {
		return prismPayload{}, false, fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d`,
			ErrUnsupported, meshbool.OpIntersect, segments, prismcells.MaxArrangementSegments)
	}
	reexpress, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil {
		return prismPayload{}, false, err
	}
	return prismcells.ResolveAndBuildIntersect(ctx, budget, pa, pb, reexpress)
}

// tryBlindStackedCut admits a cleanly nested blind tool against a prism,
// including one that encloses existing holes. The private sketch scene proves
// the two slab sections before they are recorded.
func tryBlindStackedCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return stackedPrismPayload{}, false, err
	}
	target, tool, ok, err := admitPrismPairBudget(budget, meshbool.OpCut, a, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	return stackedrecord.BuildBlindCut(ctx, budget, target, tool)
}

// tryStackedBlindCut splits the slab containing a blind tool's inner end, or
// reuses an exact existing interface. Every reached slab needs its own
// whole-loop cut proof, and the resulting interfaces must pass the audit.
func tryStackedBlindCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	sp, ok := a.n.Payload.(stackedPrismPayload)
	if !ok || len(sp.OuterRuns()) != 1 {
		return stackedPrismPayload{}, false, nil
	}
	for _, slab := range sp.Slabs {
		if len(slab.Regions) != 1 {
			return stackedPrismPayload{}, false, nil
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.OuterPrism()
	proxy := bornBody(&Body{n: topo.Body{Payload: outer}})
	_, tool, ok, err := admitPrismPairBudget(budget, meshbool.OpCut, proxy, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	return stackedrecord.ApplyBlindCut(ctx, budget, sp, outer, tool)
}

// tryStackedThroughCut applies a spanning prism tool to every slab. Each
// private scene proves either a new region or that the tool lies wholly in an
// existing hole and leaves that slab unchanged.
func tryStackedThroughCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	sp, ok := a.n.Payload.(stackedPrismPayload)
	if !ok {
		return stackedPrismPayload{}, false, nil
	}
	// A union-built stack changes its outer loop at an interface. Its exposed
	// patches are not the exclusive-hole patches this cut re-derives below, so
	// it takes the mesh path (docs/stacked-prism-design.md §6).
	if len(sp.OuterRuns()) != 1 {
		return stackedPrismPayload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.OuterPrism()
	proxy := bornBody(&Body{n: topo.Body{Payload: outer}})
	_, tool, ok, err := admitPrismPairBudget(budget, meshbool.OpCut, proxy, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	if len(tool.Profile.Holes) != 0 ||
		!prismplacement.CutZIntervalSpans(outer.PlacementOperand(), tool.PlacementOperand()) {
		return stackedPrismPayload{}, false, nil
	}
	trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, tool.Profile.Outer, tool.Profile.Holes)
	if err != nil || trimmed {
		return stackedPrismPayload{}, false, err
	}
	reexpress, err := prismcells.NewReexpression(outer.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	active := make([]bool, len(sp.Slabs))
	for i := range active {
		active[i] = true
	}
	return stackedrecord.CutSlabs(ctx, budget, sp, outer, tool, reexpress, active, sp.Interfaces, -1)
}

// tryCrossingBlindCut builds the exact rectangular case where a blind tool
// crosses one existing through hole. Its section and interface scenes are
// Sketch's; stackedbrep splits the partial hole walls at the pocket floor.
func tryCrossingBlindCut(ctx context.Context, a, b *Body) (brepPayload, bool, error) {
	target, targetOK := a.n.Payload.(prismPayload)
	tool, toolOK := b.n.Payload.(prismPayload)
	if !targetOK || !toolOK || len(target.Profile.Holes) != 1 || len(tool.Profile.Holes) != 0 {
		return brepPayload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	pa, pb, admitted, err := admitPrismPairBudget(budget, meshbool.OpCut, a, b)
	if err != nil || !admitted {
		return brepPayload{}, false, err
	}
	if pa.SectionDelta != 0 || pb.SectionDelta != 0 ||
		pa.Z0Delta != 0 || pa.Z1Delta != 0 || pb.Z0Delta != 0 || pb.Z1Delta != 0 ||
		!momentinput.CrossingBlindAxisLines(pa.Profile) || !momentinput.CrossingBlindAxisLines(pb.Profile) {
		return brepPayload{}, false, nil
	}
	reexpress, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil || !reexpress.Identity {
		return brepPayload{}, false, err
	}
	z0, z1, ok := prismplacement.ShiftedInterval(pa.PlacementOperand(), pb.PlacementOperand())
	if !ok || z0.Cmp(proofarith.FloatRat(pa.Z0)) <= 0 ||
		z0.Cmp(proofarith.FloatRat(pa.Z1)) >= 0 || z1.Cmp(proofarith.FloatRat(pa.Z1)) != 0 {
		return brepPayload{}, false, nil
	}
	va, _ := stackedunion.OperandFromPayload(a.n.Payload)
	vb, _ := stackedunion.OperandFromPayload(b.n.Payload)
	shift := prismplacement.ZShift(pa.PlacementOperand(), pb.PlacementOperand())
	levels, ok := stackedrecord.UnionLevels(va.Slabs, vb.Slabs, shift)
	if !ok || len(levels) != 3 || levels[0].Delta != 0 || levels[1].Delta != 0 || levels[2].Delta != 0 {
		return brepPayload{}, false, nil
	}
	zero := new(big.Rat)
	reach := make([][2]int, 2)
	for k := range reach {
		reach[k][0] = stackedrecord.UnionSlabOf(va.Slabs, zero, levels[k].Exact, levels[k+1].Exact)
		reach[k][1] = stackedrecord.UnionSlabOf(vb.Slabs, shift, levels[k].Exact, levels[k+1].Exact)
	}
	if reach[0] != [2]int{0, -1} || reach[1] != [2]int{0, 0} {
		return brepPayload{}, false, nil
	}
	result, err := stackedunion.Cut(ctx, budget, va, vb, reexpress, levels, reach)
	if errors.Is(err, brepgeom.ErrStackedWallMiss) || errors.Is(err, ErrUnsupported) {
		return brepPayload{}, false, nil
	}
	if err != nil {
		return brepPayload{}, false, err
	}
	return result, true, nil
}

// tryAxialLoftPrismUnion admits a Loft only when its authenticated source
// denotes an exact straight prism. The prism arranger then certifies every
// planar crossing and seam through sketch's exact fragments. This first
// subcase has no segment-count, radial-placement, or source-identity gate.
// A varying or curved Loft remains on the existing paths until its own wall
// intersection and source-boundary proofs are implemented.
func tryAxialLoftPrismUnion(ctx context.Context, op meshbool.OperationKind, a, b *Body) (prismPayload, bool, error) {
	if op != meshbool.OpUnion {
		return prismPayload{}, false, nil
	}
	pa, aPrism := a.n.Payload.(prismPayload)
	pb, bPrism := b.n.Payload.(prismPayload)
	la, aLoft := a.n.Payload.(loftPayload)
	lb, bLoft := b.n.Payload.(loftPayload)
	if aPrism && bLoft {
		var ok bool
		var err error
		pb, ok, err = loftrecord.ExactAxialPrism(ctx, lb)
		if err != nil || !ok {
			return prismPayload{}, false, err
		}
	} else if aLoft && bPrism {
		var ok bool
		var err error
		pa, ok, err = loftrecord.ExactAxialPrism(ctx, la)
		if err != nil || !ok {
			return prismPayload{}, false, err
		}
	} else {
		return prismPayload{}, false, nil
	}
	left := bornBody(&Body{n: topo.Body{Payload: pa}})
	right := bornBody(&Body{n: topo.Body{Payload: pb}})
	// A shared edge with no interior overlap is a tangent contact. Require
	// the same arranger to resolve a positive-area intersection before
	// asking it to merge the two outlines.
	_, overlap, err := tryPrismBoolean(ctx, meshbool.OpIntersect, left, right)
	if err != nil || !overlap {
		return prismPayload{}, false, err
	}
	return tryPrismBoolean(ctx, op, left, right)
}

// tryPrismFacePatchUnion admits two exact rectangular prisms whose interiors
// are disjoint and whose boundaries share a positive-area patch on one face.
// It re-expresses the second box as a prism in the first box's frame, then
// lets the existing prism or stacked Union scenes split the shared patch.
func tryPrismFacePatchUnion(ctx context.Context, a, b *Body) (topo.Payload, bool, error) {
	pa, okA := a.n.Payload.(prismPayload)
	pb, okB := b.n.Payload.(prismPayload)
	if !okA || !okB || pa.Xform != pb.Xform || pa.SurfaceResult || pb.SurfaceResult {
		return nil, false, nil
	}
	// A1's unequal-level BRep keeps planar box volumes exact. Try that
	// reference first, then the equal-level prism scene if neither frame
	// admits A1.
	for pass := range 2 {
		for _, pair := range [][2]prismPayload{{pa, pb}, {pb, pa}} {
			x, y := pair[0], pair[1]
			if x.Frame.N() == y.Frame.N() || x.Frame.N() == y.Frame.N().Scale(-1) {
				continue // class A already reads co-directional boxes.
			}
			xBox, ok := x.ExactBoxInFrame(x.Frame)
			if !ok {
				continue
			}
			yBox, ok := y.ExactBoxInFrame(x.Frame)
			if !ok || !prismplacement.BoxesShareFacePatch(xBox, yBox, 2) {
				continue
			}
			equalLevels := xBox.Lo[2] == yBox.Lo[2] && xBox.Hi[2] == yBox.Hi[2]
			if equalLevels != (pass == 1) {
				continue
			}
			yReframed := prismrecord.Payload{
				Profile: prismplacement.BoxProfile(yBox), Frame: x.Frame, Xform: x.Xform,
				Z0: yBox.Lo[2], Z1: yBox.Hi[2],
			}
			var payload topo.Payload
			var built bool
			var err error
			if equalLevels {
				payload, built, err = tryPrismBoolean(ctx, meshbool.OpUnion,
					bornBody(&Body{n: topo.Body{Payload: x}}), bornBody(&Body{n: topo.Body{Payload: yReframed}}))
			} else {
				payload, built, err = tryStackedUnion(ctx, bornBody(&Body{n: topo.Body{Payload: x}}), bornBody(&Body{n: topo.Body{Payload: yReframed}}))
			}
			if err != nil || built {
				return payload, built, err
			}
			// A silent miss in this frame may be admitted with the operands
			// reversed: that changes which in-plane axis A1 splits into slabs.
		}
	}
	return nil, false, nil
}
