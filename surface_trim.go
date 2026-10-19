package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/surfacetrim"
)

// This file implements Trim, Extend and Split: the public entry checks, the
// result builds and the commits. Their §2.1 gates live in
// internal/surfacetrim (prism family), internal/revolveplan (revolve Trim and
// Extend) and surface_split_revolve.go (revolve Split), and share
// prism-boolean's exact generator comparison, identity re-expression check and
// sweep-span relation. All three build a private scene through prismcells;
// Trim reads prismcells.Classify unchanged, while Split reads only the target
// label. Every miss is a typed refusal at the call: a sheet's mesh proves no
// occupied volume, and a mesh trim would decide topology from float signs on
// chorded triangles (docs/surface-intersection-design.md §2.1).
//
// Trim and Extend take the REVOLVE family too, over the two operands' meridian
// views: §11's PR4, this file's own last section. Split's revolve arm is §11's
// PR5, in surface_split_revolve.go.

// Extend lengthens b along the named edges' own carriers until
// they meet tool, and returns the lengthened sheet. It admits a b and
// tool sharing ONE generator, exactly as Trim does — both a straight sweep, or
// both a revolve about one axis — under surface-intersection §2.1's own gate.
// A nil context or a nil b is ErrDegenerate.
func Extend(ctx context.Context, b *Body, edges *EdgeQuery, tool *Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an extension`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := d.requireLive(tool); err != nil {
		return nil, err
	}
	if b == tool {
		return nil, fmt.Errorf(`%w: an extension needs two distinct bodies`, ErrDegenerate)
	}
	selected, err := edges.SelectEdges(b)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	// S1 routes the pair by family before either arm's gate runs, exactly as
	// Trim's own entry does. A MIXED pair reaches neither arm: it falls through
	// to surfacetrim.AdmitExtendPair, whose own S1 names the two generators.
	if surfacetrim.RevolvePair(&b.n, &tool.n) {
		return b.extendRevolve(ctx, budget, selected, tool)
	}
	rcv, tl, err := surfacetrim.AdmitExtendPair(budget, &b.n, &tool.n)
	if err != nil {
		return nil, err
	}
	rcv, err = surfacetrim.ExtendPrism(ctx, budget, &b.n, rcv, tl, edgeNodes(selected))
	if err != nil {
		return nil, err
	}
	result, err := bodyResult(chainrecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), rcv, freeform.NewFreeformWork()))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(result, b, tool)
	return result, nil
}

// TrimSide names which pieces of the receiver a Trim keeps
// (docs/surface-intersection-design.md §8).
type TrimSide int

const (
	// KeepOutside keeps the pieces of the receiver outside the tool's
	// section. KeepInside keeps the pieces inside it.
	KeepOutside TrimSide = iota
	KeepInside
)

// String renders the side for diagnostics.
func (s TrimSide) String() string {
	switch s {
	case KeepOutside:
		return "KeepOutside"
	case KeepInside:
		return "KeepInside"
	default:
		return fmt.Sprintf("TrimSide(%d)", int(s))
	}
}

// Trim cuts b where it meets tool and returns the kept pieces as
// one sheet body (docs/surface-intersection-design.md §8). b and
// tool are consumed on the document's uniform terms (core §6); a refusal
// consumes neither. It admits a pair whose two sweeps share ONE generator —
// both the prism family (a straight Extrude/chain Extrude sweep) or both the
// revolve family (a Revolve/chain Revolve spin about one axis,
// docs/api-design.md's evaluator §5). A mixed pair, or any condition §2.1's
// S1-S7 states, is [ErrUnsupported] at the call, never a fallback: a sheet's
// mesh proves no occupied volume (booloperand.RequireVolumeProof),
// so there is no mesh path for a miss to reroute to. A tool that separates no
// fragment of b, or every fragment, is [ErrDegenerate] (R30): no
// trimmed body exists either way. A nil context or a nil b is ErrDegenerate.
func Trim(ctx context.Context, b *Body, tool *Body, side TrimSide) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a trim`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := d.requireLive(tool); err != nil {
		return nil, err
	}
	if b == tool {
		return nil, fmt.Errorf(`%w: a trim needs two distinct bodies`, ErrDegenerate)
	}
	if side != KeepOutside && side != KeepInside {
		return nil, fmt.Errorf(`%w: unknown trim side %d`, ErrDegenerate, int(side))
	}

	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}

	// S1 routes the pair by family before either arm's gate runs. A MIXED pair
	// reaches neither: it falls through to surfacetrim.AdmitTrimPair, whose
	// own S1 names the two generators that share nothing (T181).
	if surfacetrim.RevolvePair(&b.n, &tool.n) {
		return b.trimRevolve(ctx, budget, tool, side == KeepInside)
	}

	rcv, tl, err := surfacetrim.AdmitTrimPair(budget, &b.n, &tool.n)
	if err != nil {
		return nil, err
	}

	chains, sectionDelta, err := prismcells.ResolveSurfaceTrim(ctx, budget,
		prismcells.SurfaceOperandOf(rcv), prismcells.SurfaceOperandOf(tl), side == KeepInside)
	if err != nil {
		return nil, err
	}

	pp := chainPayload{
		Chains:       chains,
		Frame:        rcv.Frame,
		Z0:           rcv.Z0,
		Z1:           rcv.Z1,
		Z0Delta:      rcv.Z0Delta,
		Z1Delta:      rcv.Z1Delta,
		Xform:        rcv.Xform,
		SectionDelta: sectionDelta,
	}
	ref := d.nextProducerID()
	body, err := bodyResult(chainrecord.Build(ctx, envOf(documentNode(d), ref), pp, freeform.NewFreeformWork()))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b, tool)
	return body, nil
}

// Split cuts target with tool and returns one solid per arranged target cell
// in sketch's cell order. The target is a solid and the tool a sheet sharing
// its generator: both straight sweeps, or both revolves about one axis
// (docs/surface-intersection-design.md §2.1). Any other pair, or a miss on
// that section's gate, is ErrUnsupported. Both operands are consumed only
// after every piece has been built. A tool that separates no target cell is
// ErrDegenerate, as is a nil context or a nil target. The document is the
// target's own.
func Split(ctx context.Context, target, tool *Body) ([]*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a split`, ErrDegenerate)
	}
	if target == nil || target.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the split target belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(target.n.Doc)
	if err := d.requireLive(target); err != nil {
		return nil, err
	}
	if err := d.requireLive(tool); err != nil {
		return nil, err
	}
	if target == tool {
		return nil, fmt.Errorf(`%w: a split needs two distinct bodies`, ErrDegenerate)
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if pieces, handled, err := d.splitPointPlane(ctx, budget, target, tool); handled {
		return pieces, err
	}
	// S1 routes a revolve pair to its own arm (surface_split_revolve.go); a
	// mixed pair falls through to surfacetrim.AdmitSplitPair, whose S1 names
	// both generators.
	if surfacetrim.RevolvePair(&target.n, &tool.n) {
		return d.splitRevolve(ctx, budget, target, tool)
	}
	rcv, tl, err := surfacetrim.AdmitSplitPair(budget, &target.n, &tool.n)
	if err != nil {
		return nil, err
	}
	pieces, err := prismcells.SplitPieces(ctx, budget, rcv, tl)
	if err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	bodies := make([]*Body, len(pieces))
	for i, piece := range pieces {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		bodies[i], err = bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref+producerID(i)), piece, freeform.NewFreeformWork()))
		if err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commitMany(bodies, target, tool)
	return bodies, nil
}

// This section is the revolve family's arm of the gate and of Trim
// (docs/surface-intersection-design.md §11's PR4). Only S1's routing, S4's
// generator test and S6's span relation differ from the prism arm; §3's whole
// resolution — prismcells.BuildScene, prismcells.Classify, the survivor chaining and
// the cut-displacement reading — is consumed VERBATIM over the two operands'
// MERIDIAN views, which is what §3.1 claims and what revolvePayload.Meridian
// supplies.

// trimRevolve is Trim over a pair S1 has already routed to the revolve family.
// The receiver's section must CLOSE (§2.2), exactly as the prism arm's does and
// for the identical reason: §3.2's side reading needs every fragment to bound a
// cell of the receiver's own interior, and sketch prunes a ribbon's dangling
// free-end fragments before publishing its regions, so an inside stub and an
// outside stub arrive identically (§4's own row, RS3).
func (b *Body) trimRevolve(ctx context.Context, budget *proofbound.WorkBudget, tool *Body, keepInside bool) (*Body, error) {
	d := documentHandle(b.n.Doc)
	// Read S7 from both payload shapes before the revolve-only gate.
	rcv, rcvView, tlView, err := revolveplan.AdmitTrimPair(ctx, budget,
		&b.n, &tool.n, surfacetrim.SectionDelta(&b.n), surfacetrim.SectionDelta(&tool.n))
	if err != nil {
		return nil, err
	}
	chains, sectionDelta, err := prismcells.ResolveSurfaceTrim(ctx, budget,
		prismcells.SurfaceOperandOf(rcvView), prismcells.SurfaceOperandOf(tlView), keepInside)
	if err != nil {
		return nil, err
	}
	pp := chainRevolvePayload{
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
	body, err := bodyResult(revolveplan.BuildChain(ctx, envOf(documentNode(d), d.nextProducerID()), pp, freeform.NewFreeformWork()))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b, tool)
	return body, nil
}

// extendRevolve is Extend over a pair S1 has already routed to the revolve
// family: revolveplan's gate, then surfacetrim.ExtendRevolve's widening over
// the two MERIDIAN views (§3.1), then the build and commit.
func (b *Body) extendRevolve(ctx context.Context, budget *proofbound.WorkBudget, selected []*Edge, tool *Body) (*Body, error) {
	d := documentHandle(b.n.Doc)
	// Read S7 from both payload shapes before the revolve-only gate.
	rcv, rcvView, tlView, err := revolveplan.AdmitExtendPair(ctx, budget,
		&b.n, &tool.n, surfacetrim.SectionDelta(&b.n), surfacetrim.SectionDelta(&tool.n))
	if err != nil {
		return nil, err
	}
	rcv, err = surfacetrim.ExtendRevolve(ctx, budget, &b.n, rcv, rcvView, tlView, edgeNodes(selected))
	if err != nil {
		return nil, err
	}
	result, err := bodyResult(revolveplan.BuildChain(ctx, envOf(documentNode(d), d.nextProducerID()), rcv, freeform.NewFreeformWork()))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(result, b, tool)
	return result, nil
}
