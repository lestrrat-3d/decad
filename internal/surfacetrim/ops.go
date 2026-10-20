package surfacetrim

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Extend builds the extension of b along the selected edges to tool's section.
// S1 routes the pair by family before either arm's gate runs. A MIXED pair
// reaches neither arm: it falls through to AdmitExtendPair, whose own S1 names
// the two generators. The result is not yet committed.
func Extend(ctx context.Context, env topo.Env, b *topo.Body, selected []*topo.Edge, tool *topo.Body) (*topo.Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if RevolvePair(b, tool) {
		return extendRevolve(ctx, env, budget, b, selected, tool)
	}
	rcv, tl, err := AdmitExtendPair(budget, b, tool)
	if err != nil {
		return nil, err
	}
	rcv, err = ExtendPrism(ctx, budget, b, rcv, tl, selected)
	if err != nil {
		return nil, err
	}
	result, err := chainrecord.Build(ctx, env, rcv, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return result, nil
}

// Trim builds the trim of b by tool's section, keeping the pieces inside it
// when keepInside is set and outside it otherwise. S1 routes the pair by
// family before either arm's gate runs. A MIXED pair reaches neither: it falls
// through to AdmitTrimPair, whose own S1 names the two generators that share
// nothing (T181). The result is not yet committed.
func Trim(ctx context.Context, env topo.Env, b, tool *topo.Body, keepInside bool) (*topo.Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if RevolvePair(b, tool) {
		return trimRevolve(ctx, env, budget, b, tool, keepInside)
	}
	rcv, tl, err := AdmitTrimPair(budget, b, tool)
	if err != nil {
		return nil, err
	}
	chains, sectionDelta, err := prismcells.ResolveSurfaceTrim(ctx, budget,
		prismcells.SurfaceOperandOf(rcv), prismcells.SurfaceOperandOf(tl), keepInside)
	if err != nil {
		return nil, err
	}
	pp := chainrecord.Payload{
		Chains:       chains,
		Frame:        rcv.Frame,
		Z0:           rcv.Z0,
		Z1:           rcv.Z1,
		Z0Delta:      rcv.Z0Delta,
		Z1Delta:      rcv.Z1Delta,
		Xform:        rcv.Xform,
		SectionDelta: sectionDelta,
	}
	body, err := chainrecord.Build(ctx, env, pp, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return body, nil
}

// Split builds the pieces of target that tool separates. The point-loft fan is
// a separate arm; a revolve pair takes its own arm (SplitRevolve); a mixed pair
// falls through to AdmitSplitPair, whose S1 names both generators. The pieces
// are not yet committed.
func Split(ctx context.Context, env topo.Env, target, tool *topo.Body) ([]*topo.Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if pieces, handled, err := splitPointPlane(ctx, budget, env, target, tool); handled {
		return pieces, err
	}
	if RevolvePair(target, tool) {
		return SplitRevolve(ctx, budget, env, target, tool)
	}
	rcv, tl, err := AdmitSplitPair(budget, target, tool)
	if err != nil {
		return nil, err
	}
	pieces, err := prismcells.SplitPieces(ctx, budget, rcv, tl)
	if err != nil {
		return nil, err
	}
	bodies := make([]*topo.Body, len(pieces))
	for i, piece := range pieces {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		bodies[i], err = prismrecord.Build(ctx, topo.Env{Doc: env.Doc, Ref: env.Ref + provenance.Producer(i), Alloc: env.Alloc},
			piece, freeform.NewFreeformWork())
		if err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return bodies, nil
}

func trimRevolve(ctx context.Context, env topo.Env, budget *proofbound.WorkBudget, b, tool *topo.Body, keepInside bool) (*topo.Body, error) {
	// Read S7 from both payload shapes before the revolve-only gate.
	rcv, rcvView, tlView, err := revolveplan.AdmitTrimPair(ctx, budget, b, tool, SectionDelta(b), SectionDelta(tool))
	if err != nil {
		return nil, err
	}
	chains, sectionDelta, err := prismcells.ResolveSurfaceTrim(ctx, budget,
		prismcells.SurfaceOperandOf(rcvView), prismcells.SurfaceOperandOf(tlView), keepInside)
	if err != nil {
		return nil, err
	}
	pp := revolveplan.ChainPayload{
		Chains: chains,
		Frame:  rcv.Frame,
		Ax:     rcv.Ax,
		Phi0:   rcv.Phi0,
		Phi1:   rcv.Phi1,
		Full:   rcv.Full,
		Den:    rcv.Den,
		Xform:  rcv.Xform,
		// §7.1's fold reads this field as its gate, and ResolveSurfaceTrim's
		// δ_cut is the whole of it: S4 and S7 zero every other term prism §7
		// derives.
		SectionDelta: sectionDelta,
	}
	body, err := revolveplan.BuildChain(ctx, env, pp, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return body, nil
}

func extendRevolve(ctx context.Context, env topo.Env, budget *proofbound.WorkBudget, b *topo.Body, selected []*topo.Edge, tool *topo.Body) (*topo.Body, error) {
	// Read S7 from both payload shapes before the revolve-only gate.
	rcv, rcvView, tlView, err := revolveplan.AdmitExtendPair(ctx, budget, b, tool, SectionDelta(b), SectionDelta(tool))
	if err != nil {
		return nil, err
	}
	rcv, err = ExtendRevolve(ctx, budget, b, rcv, rcvView, tlView, selected)
	if err != nil {
		return nil, err
	}
	result, err := revolveplan.BuildChain(ctx, env, rcv, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return result, nil
}

// splitPointPlane is a separate Split arm. It handles only authenticated
// point-loft fans and one exact chain-sheet class; other pairs continue to
// the shared-generator Split dispatcher.
func splitPointPlane(ctx context.Context, budget *proofbound.WorkBudget, env topo.Env,
	target, tool *topo.Body) ([]*topo.Body, bool, error) {
	record, ok := facetedrecord.PointPlaneSource(target)
	if !ok {
		return nil, false, nil
	}
	plane, ok := facetedrecord.PointPlaneSheet(tool, record.Base.PointSection.Apex)
	if !ok {
		return nil, false, nil
	}
	if err := facetedrecord.ValidatePointPlane(record.Base, plane); err != nil {
		return nil, true, err
	}
	if center, proved, err := facetedrecord.PointPlaneCapCentroid(ctx, record.Base, plane); err != nil {
		return nil, true, err
	} else {
		plane.Centroid, plane.HasCentroid = center, proved
	}
	if record.Lower != nil {
		switch facetedrecord.PointPlaneOrder(record.Base, record.Lower, plane) {
		case -1:
			// The new plane is strictly beyond the existing lower cap.
		case 1:
			return nil, true, fmt.Errorf("%w: the split sheet misses this point-loft slab", decaderr.ErrDegenerate)
		default:
			return nil, true, fmt.Errorf("%w: the split plane meets or crosses the slab's lower plane", decaderr.ErrUnsupported)
		}
	}
	if record.Upper != nil {
		switch facetedrecord.PointPlaneOrder(record.Base, plane, record.Upper) {
		case -1:
			// The new plane is strictly before the existing upper cap.
		case 1:
			return nil, true, fmt.Errorf("%w: the split sheet misses this point-loft slab", decaderr.ErrDegenerate)
		default:
			return nil, true, fmt.Errorf("%w: the split plane meets or crosses the slab's upper plane", decaderr.ErrUnsupported)
		}
	}
	if err := budget.Err(); err != nil {
		return nil, true, err
	}
	inner := record
	inner.Upper = plane
	outer := record
	outer.Lower = plane
	first, err := facetedrecord.BuildPointPlaneSlab(ctx, env, inner)
	if err != nil {
		return nil, true, fmt.Errorf("point-loft inner slab: %w", err)
	}
	second, err := facetedrecord.BuildPointPlaneSlab(ctx, topo.Env{Doc: env.Doc, Ref: env.Ref + 1, Alloc: env.Alloc}, outer)
	if err != nil {
		return nil, true, fmt.Errorf("point-loft outer slab: %w", err)
	}
	if err := ctx.Err(); err != nil {
		return nil, true, err
	}
	return []*topo.Body{first, second}, true, nil
}
