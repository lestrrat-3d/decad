// Package prismboolean admits prism, stacked-prism and prism-group operand
// payloads to the analytic boolean routes of agent-docs/prism-boolean-design.md and
// agent-docs/general-boolean-design.md §3 (A1, A5, A7). It reads only the two
// operand payloads, runs the reject-only gates, and passes the admitted
// records to prismcells, stackedrecord and stackedunion, which build the
// result payload. The root boolean dispatch builds and commits the body.
//
// Every entry returns ok=false with a nil error for a silent miss: the caller
// takes its next route, ending at the mesh path. A non-nil error is a genuine
// refusal the caller propagates (prism-boolean §3.4).
package prismboolean

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Try is the analytic prism boolean of prism-boolean §3-§4. For Intersect it
// tries the one-hole nesting route first, then passes the payloads to
// prismcells.TryAnalytic.
func Try(ctx context.Context, op meshbool.OperationKind, a, b topo.Payload) (prismrecord.Payload, bool, error) {
	if op == meshbool.OpIntersect {
		if result, ok, err := tryHoledIntersect(ctx, a, b); ok || err != nil {
			return result, ok, err
		}
	}
	return prismcells.TryAnalytic(ctx, op, a, b)
}

// tryHoledIntersect admits one cleanly nested, one-hole prism under a
// hole-free prism. Intersect is symmetric, so put the hole-free operand first.
// The shared Intersect admission used by the overlap-area reading stays
// hole-free; this body-only arm uses the same gates and work cap explicitly.
func tryHoledIntersect(ctx context.Context, a, b topo.Payload) (prismrecord.Payload, bool, error) {
	ah, aok := a.(prismrecord.Payload)
	bh, bok := b.(prismrecord.Payload)
	if !aok || !bok {
		return prismrecord.Payload{}, false, nil
	}
	if len(ah.Profile.Holes) == 1 && len(bh.Profile.Holes) == 0 {
		a, b = b, a
	} else if len(ah.Profile.Holes) != 0 || len(bh.Profile.Holes) != 1 {
		return prismrecord.Payload{}, false, nil
	}

	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return prismrecord.Payload{}, false, err
	}
	pa, pb, ok, err := prismcells.AdmitPairBudget(budget, meshbool.OpIntersect, a, b)
	if err != nil || !ok {
		return prismrecord.Payload{}, false, err
	}
	for _, profile := range []momentinput.Profile{pa.Profile, pb.Profile} {
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, profile.Outer, profile.Holes)
		if err != nil {
			return prismrecord.Payload{}, false, err
		}
		if trimmed {
			return prismrecord.Payload{}, false, nil
		}
	}
	if !prismplacement.IntersectZIntervalOverlaps(pa.PlacementOperand(), pb.PlacementOperand()) {
		return prismrecord.Payload{}, false, nil
	}
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, pa.Profile, pb.Profile)
	if err != nil {
		return prismrecord.Payload{}, false, err
	}
	if !withinCap {
		return prismrecord.Payload{}, false, prismcells.WithWorkLimit(fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d`,
			decaderr.ErrUnsupported, meshbool.OpIntersect, segments, prismcells.MaxArrangementSegments), segments)
	}
	reexpress, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil {
		return prismrecord.Payload{}, false, err
	}
	return prismcells.ResolveAndBuildIntersect(ctx, budget, pa, pb, reexpress)
}

// TryBlindStackedCut admits a cleanly nested blind tool against a prism,
// including one that encloses existing holes. The private sketch scene proves
// the two slab sections before they are recorded.
func TryBlindStackedCut(ctx context.Context, a, b topo.Payload) (stackedrecord.Payload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return stackedrecord.Payload{}, false, err
	}
	target, tool, ok, err := prismcells.AdmitPairBudget(budget, meshbool.OpCut, a, b)
	if err != nil || !ok {
		return stackedrecord.Payload{}, false, err
	}
	return stackedrecord.BuildBlindCut(ctx, budget, target, tool)
}

// TryStackedBlindCut splits the slab containing a blind tool's inner end, or
// reuses an exact existing interface. Every reached slab needs its own
// whole-loop cut proof, and the resulting interfaces must pass the audit.
func TryStackedBlindCut(ctx context.Context, a, b topo.Payload) (stackedrecord.Payload, bool, error) {
	sp, ok := a.(stackedrecord.Payload)
	if !ok || len(sp.OuterRuns()) != 1 {
		return stackedrecord.Payload{}, false, nil
	}
	for _, slab := range sp.Slabs {
		if len(slab.Regions) != 1 {
			return stackedrecord.Payload{}, false, nil
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.OuterPrism()
	_, tool, ok, err := prismcells.AdmitPairBudget(budget, meshbool.OpCut, outer, b)
	if err != nil || !ok {
		return stackedrecord.Payload{}, false, err
	}
	return stackedrecord.ApplyBlindCut(ctx, budget, sp, outer, tool)
}

// TryOpposedBlindCut builds a blind tool ending exactly at an existing
// interface whose opposed exclusive holes overlap in plan
// (agent-docs/general-boolean-design.md §3 A8). TryStackedBlindCut's
// admission applies; the result is a BRep because the stacked payload cannot
// hold the partial floor and ceiling. Opposed holes that touch, and any
// scene or topology the build does not cover, are silent misses.
func TryOpposedBlindCut(ctx context.Context, a, b topo.Payload) (breprecord.Payload, bool, error) {
	sp, ok := a.(stackedrecord.Payload)
	if !ok || len(sp.OuterRuns()) != 1 {
		return breprecord.Payload{}, false, nil
	}
	for _, slab := range sp.Slabs {
		if len(slab.Regions) != 1 {
			return breprecord.Payload{}, false, nil
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.OuterPrism()
	_, tool, ok, err := prismcells.AdmitPairBudget(budget, meshbool.OpCut, outer, b)
	if err != nil || !ok {
		return breprecord.Payload{}, false, err
	}
	oc, ok, err := stackedrecord.ApplyOpposedBlindCut(ctx, budget, sp, outer, tool)
	if errors.Is(err, decaderr.ErrUnsupported) || errors.Is(err, decaderr.ErrDegenerate) {
		// TryStackedBlindCut already raised every refusal of the slab cuts it
		// shares; an interface this arm cannot derive leaves the pair to the
		// mesh path.
		return breprecord.Payload{}, false, nil
	}
	if err != nil || !ok {
		return breprecord.Payload{}, false, err
	}
	result, err := stackedunion.OpposedCut(ctx, budget, oc)
	if errors.Is(err, brepgeom.ErrStackedWallMiss) || errors.Is(err, decaderr.ErrUnsupported) {
		return breprecord.Payload{}, false, nil
	}
	if err != nil {
		return breprecord.Payload{}, false, err
	}
	return result, true, nil
}

// TryStackedThroughCut applies a spanning prism tool to every slab. Each
// private scene proves either a new region or that the tool lies wholly in an
// existing hole and leaves that slab unchanged.
func TryStackedThroughCut(ctx context.Context, a, b topo.Payload) (stackedrecord.Payload, bool, error) {
	sp, ok := a.(stackedrecord.Payload)
	if !ok {
		return stackedrecord.Payload{}, false, nil
	}
	// A union-built stack changes its outer loop at an interface. Its exposed
	// patches are not the exclusive-hole patches this cut re-derives below, so
	// it takes the mesh path (agent-docs/stacked-prism-design.md §6).
	if len(sp.OuterRuns()) != 1 {
		return stackedrecord.Payload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.OuterPrism()
	_, tool, ok, err := prismcells.AdmitPairBudget(budget, meshbool.OpCut, outer, b)
	if err != nil || !ok {
		return stackedrecord.Payload{}, false, err
	}
	if len(tool.Profile.Holes) != 0 ||
		!prismplacement.CutZIntervalSpans(outer.PlacementOperand(), tool.PlacementOperand()) {
		return stackedrecord.Payload{}, false, nil
	}
	trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, tool.Profile.Outer, tool.Profile.Holes)
	if err != nil || trimmed {
		return stackedrecord.Payload{}, false, err
	}
	reexpress, err := prismcells.NewReexpression(outer.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return stackedrecord.Payload{}, false, err
	}
	active := make([]bool, len(sp.Slabs))
	for i := range active {
		active[i] = true
	}
	return stackedrecord.CutSlabs(ctx, budget, sp, outer, tool, reexpress, active, sp.Interfaces, -1)
}

// TryCrossingBlindCut builds the case where a blind tool crosses one existing
// through hole, every loop of both operands made of axis-aligned lines, circles
// and arcs. Its section and interface scenes are Sketch's; stackedbrep splits
// the partial hole walls at the pocket floor.
func TryCrossingBlindCut(ctx context.Context, a, b topo.Payload) (breprecord.Payload, bool, error) {
	target, targetOK := a.(prismrecord.Payload)
	tool, toolOK := b.(prismrecord.Payload)
	if !targetOK || !toolOK || len(target.Profile.Holes) != 1 || len(tool.Profile.Holes) != 0 {
		return breprecord.Payload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	pa, pb, admitted, err := prismcells.AdmitPairBudget(budget, meshbool.OpCut, a, b)
	if err != nil || !admitted {
		return breprecord.Payload{}, false, err
	}
	if pa.SectionDelta != 0 || pb.SectionDelta != 0 ||
		!momentinput.CrossingBlindLoops(pa.Profile) || !momentinput.CrossingBlindLoops(pb.Profile) {
		return breprecord.Payload{}, false, nil
	}
	reexpress, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil || !reexpress.Identity {
		return breprecord.Payload{}, false, err
	}
	va, _ := stackedunion.OperandFromPayload(a)
	vb, _ := stackedunion.OperandFromPayload(b)
	shift := prismplacement.ZShift(pa.PlacementOperand(), pb.PlacementOperand())
	levels, reach, ok := stackedrecord.CrossingBlindLevels(va.Slabs, vb.Slabs, shift)
	if !ok {
		return breprecord.Payload{}, false, nil
	}
	// The tool ends at one target cap (or passes it) and starts inside the
	// target at the other end: the cut slab is the one the tool reaches.
	topArm := reach[0] == [2]int{0, -1} && reach[1] == [2]int{0, 0}
	bottomArm := reach[0] == [2]int{0, 0} && reach[1] == [2]int{0, -1}
	if !topArm && !bottomArm {
		return breprecord.Payload{}, false, nil
	}
	result, err := stackedunion.Cut(ctx, budget, va, vb, reexpress, levels, reach)
	if errors.Is(err, brepgeom.ErrStackedWallMiss) || errors.Is(err, decaderr.ErrUnsupported) {
		return breprecord.Payload{}, false, nil
	}
	if err != nil {
		return breprecord.Payload{}, false, err
	}
	return result, true, nil
}

// TryAxialLoftUnion admits a Loft only when its authenticated source denotes
// an exact straight prism. The prism arranger then certifies every planar
// crossing and seam through sketch's exact fragments. This first subcase has
// no segment-count, radial-placement, or source-identity gate. A varying or
// curved Loft remains on the existing paths until its own wall intersection
// and source-boundary proofs are implemented.
func TryAxialLoftUnion(ctx context.Context, op meshbool.OperationKind, a, b topo.Payload) (prismrecord.Payload, bool, error) {
	if op != meshbool.OpUnion {
		return prismrecord.Payload{}, false, nil
	}
	pa, aPrism := a.(prismrecord.Payload)
	pb, bPrism := b.(prismrecord.Payload)
	la, aLoft := a.(loftrecord.Payload)
	lb, bLoft := b.(loftrecord.Payload)
	if aPrism && bLoft {
		var ok bool
		var err error
		pb, ok, err = loftrecord.ExactAxialPrism(ctx, lb)
		if err != nil || !ok {
			return prismrecord.Payload{}, false, err
		}
	} else if aLoft && bPrism {
		var ok bool
		var err error
		pa, ok, err = loftrecord.ExactAxialPrism(ctx, la)
		if err != nil || !ok {
			return prismrecord.Payload{}, false, err
		}
	} else {
		return prismrecord.Payload{}, false, nil
	}
	// A shared edge with no interior overlap is a tangent contact. Require
	// the same arranger to resolve a positive-area intersection before
	// asking it to merge the two outlines.
	_, overlap, err := Try(ctx, meshbool.OpIntersect, pa, pb)
	if err != nil || !overlap {
		return prismrecord.Payload{}, false, err
	}
	return Try(ctx, op, pa, pb)
}

// TryFacePatchUnion admits two exact rectangular prisms whose interiors are
// disjoint and whose boundaries share a positive-area patch on one face. It
// re-expresses the second box as a prism in the first box's frame, then lets
// the prism or stacked Union scenes split the shared patch.
func TryFacePatchUnion(ctx context.Context, a, b topo.Payload) (topo.Payload, bool, error) {
	pa, okA := a.(prismrecord.Payload)
	pb, okB := b.(prismrecord.Payload)
	if !okA || !okB || pa.Xform != pb.Xform || pa.SurfaceResult || pb.SurfaceResult {
		return nil, false, nil
	}
	// A1's unequal-level BRep keeps planar box volumes exact. Try that
	// reference first, then the equal-level prism scene if neither frame
	// admits A1.
	for pass := range 2 {
		for _, pair := range [][2]prismrecord.Payload{{pa, pb}, {pb, pa}} {
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
				payload, built, err = Try(ctx, meshbool.OpUnion, x, yReframed)
			} else {
				payload, built, err = TryStackedUnion(ctx, x, yReframed)
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
