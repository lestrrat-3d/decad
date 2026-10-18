package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
)

// This file implements Trim, Extend and Split. Their §2.1
// gates share prism-boolean's exact generator comparison, identity
// re-expression check and sweep-span relation. All three build a private scene
// through prismcells;
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
	// to admitExtendPair, whose own S1 names the two generators.
	if bodyTrimFamily(b) == trimFamilyRevolve && bodyTrimFamily(tool) == trimFamilyRevolve {
		return b.extendRevolve(ctx, budget, selected, tool)
	}
	rcv, tl, err := admitExtendPair(budget, b, tool)
	if err != nil {
		return nil, err
	}
	chains := make([]chainRecord, len(rcv.Chains))
	for i, chain := range rcv.Chains {
		chains[i] = chainRecord{Segments: append([]curveSegment(nil), chain.Segments...)}
	}
	cutDelta := 0.0
	for _, edge := range selected {
		ci, si, atStart, err := extendEndSegment(b, rcv, edge)
		if err != nil {
			return nil, err
		}
		widened, delta, err := prismcells.ResolveSurfaceExtend(ctx, budget,
			prismcells.SurfaceOperandOf(rcv.Prism()), prismcells.SurfaceOperandOf(tl), chains[ci].Segments[si], atStart)
		if err != nil {
			return nil, err
		}
		chains[ci].Segments[si] = widened
		cutDelta = math.Max(cutDelta, delta)
	}
	rcv.Chains = chains
	rcv.SectionDelta = cutDelta
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

// extendEndSegment maps a selected topology edge to the recorded end it names.
// The ribbon builder places the free sweep edges at the first and last wall's
// vertical sides; it preserves walk order through the one-lump-per-chain build.
//
// The loop indexes b.lumps and reads pp.chains at the SAME index, which assumes
// the body's i-th lump is the payload's i-th chain. That holds because every
// chainPayload a body carrying zero section displacement can hold comes from
// one of two producers — chain Extrude (extrude.go) and this file's own Extend —
// and both build exactly one lump per chain in chain order through
// chainrecord.Build; Trim's chainPayload is the only other one, and S7
// already refused it for its nonzero sectionDelta. What would break it: a
// producer that emits a lump per connected PIECE rather than per chain (one
// self-touching walk splitting into two lumps, or two walks meeting at a vertex
// merging into one), or one that reorders lumps. Either would silently widen a
// bound on the wrong chain, so a producer adding that shape owes this function
// a recorded lump-to-chain map rather than the shared index.
func extendEndSegment(b *Body, pp chainPayload, edge *Edge) (int, int, bool, error) {
	if !edge.IsFree() {
		return 0, 0, false, fmt.Errorf(`%w: Extend needs a free sweep edge at an open section end`, ErrUnsupported)
	}
	for ci, lump := range b.n.Lumps {
		if ci >= len(pp.Chains) || len(lump.Shells) != 1 || len(lump.Shells[0].Faces) == 0 {
			continue
		}
		faces := lump.Shells[0].Faces
		first := faces[0].Loops[0].CoEdges
		last := faces[len(faces)-1].Loops[0].CoEdges
		if edge == edgeHandle(first[3].Edge) {
			return ci, 0, true, nil
		}
		if edge == edgeHandle(last[1].Edge) {
			return ci, len(pp.Chains[ci].Segments) - 1, false, nil
		}
	}
	return 0, 0, false, fmt.Errorf(`%w: Extend needs a free sweep edge at an open section end`, ErrUnsupported)
}

// admitExtendPair applies S1-S4, S6 and S7. The selected segment enters the
// scene at full domain; every other receiver segment stays outside it.
func admitExtendPair(budget *proofbound.WorkBudget, receiver, tool *Body) (chainPayload, prismPayload, error) {
	// A revolve pair never reaches here: Extend routes it to its own arm
	// before this gate runs, exactly as Trim routes one.
	rf, tf := bodyTrimFamily(receiver), bodyTrimFamily(tool)
	if rf != tf || rf != trimFamilyPrism {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Extend needs a receiver and tool sharing a straight-sweep generator (receiver %s, tool %s)`,
			ErrUnsupported, trimFamilyName(rf), trimFamilyName(tf))
	}
	rcv, ok := receiver.n.Payload.(chainPayload)
	if !ok || receiver.Kind() != BodySheet {
		return chainPayload{}, prismPayload{}, fmt.Errorf(`%w: Extend's receiver needs an open section`, ErrUnsupported)
	}
	var tl prismPayload
	switch p := tool.n.Payload.(type) {
	case prismPayload:
		tl = p
	case chainPayload:
		tl = p.Prism()
	default:
		return chainPayload{}, prismPayload{}, fmt.Errorf(`%w: Extend's tool has no prism section`, ErrUnsupported)
	}
	if rcv.SectionDelta != 0 || trimOperandSectionDelta(tool) != 0 {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Extend does not admit an operand carrying its own section displacement`, ErrUnsupported)
	}
	view := rcv.Prism()
	if err := prismcells.AdmitSurfacePair(budget, prismcells.SurfaceExtend,
		prismcells.SurfaceOperandOf(view), prismcells.SurfaceOperandOf(tl)); err != nil {
		return chainPayload{}, prismPayload{}, err
	}
	return rcv, tl, nil
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
// mesh proves no occupied volume (boolean.go's requireVolumeProvingPayload),
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
	// reaches neither: it falls through to admitTrimPair, whose own S1 names
	// the two generators that share nothing (T181).
	if bodyTrimFamily(b) == trimFamilyRevolve && bodyTrimFamily(tool) == trimFamilyRevolve {
		return b.trimRevolve(ctx, budget, tool, side == KeepInside)
	}

	rcv, tl, err := admitTrimPair(budget, b, tool)
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

// trimSweepFamily is S1's own vocabulary: which one-parameter rigid motion a
// payload's sweep is generated by. Two payloads reduce to a common 2D section
// space only when they share one (docs/surface-intersection-design.md §1).
type trimSweepFamily uint8

const (
	trimFamilyUnknown trimSweepFamily = iota
	trimFamilyPrism
	trimFamilyRevolve
)

// trimFamilyName is what a refusal message names — the generator, never the
// payload class (T181): "a straight sweep" or "a revolve" reads the same
// whether the body came from Extrude or chain Extrude, Revolve or
// chain Revolve.
func trimFamilyName(f trimSweepFamily) string {
	switch f {
	case trimFamilyPrism:
		return "a straight sweep"
	case trimFamilyRevolve:
		return "a revolve"
	default:
		return "an unrecognized sweep"
	}
}

// bodyTrimFamily reads S1 off a body's own payload.
func bodyTrimFamily(b *Body) trimSweepFamily {
	switch b.n.Payload.(type) {
	case prismPayload, chainPayload:
		return trimFamilyPrism
	case revolvePayload, chainRevolvePayload:
		return trimFamilyRevolve
	default:
		return trimFamilyUnknown
	}
}

// trimOperandSectionDelta reads S7's own section-displacement bound off an
// admitted body, generically over all four shapes one may carry: a
// prismPayload's or revolvePayload's own sectionDelta, or a chainPayload's or
// chainRevolvePayload's identical field
// (docs/surface-intersection-design.md §3.4) — the term this design's own
// Trim is the first construction to set nonzero on a ribbon. Called only
// once S1 has already proved the body is one of the four, so no fifth case
// arises.
func trimOperandSectionDelta(b *Body) float64 {
	switch p := b.n.Payload.(type) {
	case prismPayload:
		return p.SectionDelta
	case chainPayload:
		return p.SectionDelta
	case revolvePayload:
		return p.SectionDelta
	case chainRevolvePayload:
		return p.SectionDelta
	default:
		return 0
	}
}

// admitTrimPair runs S1-S7 (docs/surface-intersection-design.md §2.1) for a
// Trim over the prism family. Every miss is ErrUnsupported at the call —
// there is no mesh fallback for this family of operations (§2.1). The
// revolve family is PR4's hook: S1 refuses it exactly as it refuses a mixed
// pair, since this file builds no revolve-side admission.
func admitTrimPair(budget *proofbound.WorkBudget, receiver, tool *Body) (rcv, tl prismPayload, err error) {
	// S1: both operands the same sweep family, and this PR admits only the
	// prism family.
	rf, tf := bodyTrimFamily(receiver), bodyTrimFamily(tool)
	if rf == trimFamilyUnknown || tf == trimFamilyUnknown {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim needs a body built by a straight sweep or a revolve`, ErrUnsupported)
	}
	if rf != tf {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the receiver's %s and the tool's %s share no generator`, ErrUnsupported, trimFamilyName(rf), trimFamilyName(tf))
	}

	// S7's own section-displacement clause is checked here, ahead of the
	// closed-loop requirement below, and reads generically off EITHER a
	// prismPayload or a chainPayload receiver: a body this design already
	// trimmed carries a chainPayload with a nonzero sectionDelta, and a
	// second Trim on it must name THAT — its own displacement — rather than
	// the open-walk shape every chainPayload receiver carries regardless
	// (T173: "the refusal is the displacement's doing, not the tool's").
	rcvDelta, tlDelta := trimOperandSectionDelta(receiver), trimOperandSectionDelta(tool)
	if rcvDelta != 0 || tlDelta != 0 {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim does not admit an operand carrying its own section displacement (receiver %v mm, tool %v mm)`,
			ErrUnsupported, rcvDelta, tlDelta)
	}

	// The receiver's section must be closed (§2.2; RS3 for the open-walk
	// case) and the tool's must be one closed, hole-free loop (S5). Both are
	// prism-family here, so an operand that is not a prismPayload is a
	// chainPayload — an open walk.
	rcv, rcvOk := receiver.n.Payload.(prismPayload)
	if !rcvOk {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim's receiver section is an open walk, which closes onto nothing for the tool to trim`, ErrUnsupported)
	}
	tl, tlOk := tool.n.Payload.(prismPayload)
	if !tlOk {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim's tool section is an open walk, which bounds no region to keep a side of`, ErrUnsupported)
	}

	if err := prismcells.AdmitSurfacePair(budget, prismcells.SurfaceTrim,
		prismcells.SurfaceOperandOf(rcv), prismcells.SurfaceOperandOf(tl)); err != nil {
		return prismPayload{}, prismPayload{}, err
	}

	return rcv, tl, nil
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
	// mixed pair falls through to admitSplitPair, whose S1 names both
	// generators.
	if bodyTrimFamily(target) == trimFamilyRevolve && bodyTrimFamily(tool) == trimFamilyRevolve {
		return d.splitRevolve(ctx, budget, target, tool)
	}
	rcv, tl, err := admitSplitPair(budget, target, tool)
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

// admitSplitPair applies S1-S4, S6 and S7 to a solid prism target and a
// prism-family sheet. S5 belongs to Trim alone: Split reads no tool side.
// The closed-sheet and chain-sheet views share prismcells.BuildScene's input shape.
func admitSplitPair(budget *proofbound.WorkBudget, target, tool *Body) (prismPayload, prismPayload, error) {
	rf, tf := bodyTrimFamily(target), bodyTrimFamily(tool)
	if rf != tf || rf != trimFamilyPrism {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Split needs a solid and sheet sharing a straight-sweep generator (target %s, tool %s)`,
			ErrUnsupported, trimFamilyName(rf), trimFamilyName(tf))
	}
	rcv, ok := target.n.Payload.(prismPayload)
	if !ok || target.Kind() != BodySolid || !target.IsSolid() {
		return prismPayload{}, prismPayload{}, fmt.Errorf(`%w: Split's target must be a solid prism`, ErrUnsupported)
	}
	if tool.Kind() != BodySheet {
		return prismPayload{}, prismPayload{}, fmt.Errorf(`%w: Split's tool must be a sheet`, ErrUnsupported)
	}
	var tl prismPayload
	switch p := tool.n.Payload.(type) {
	case prismPayload:
		tl = p
	case chainPayload:
		tl = p.Prism()
	default:
		return prismPayload{}, prismPayload{}, fmt.Errorf(`%w: Split's tool has no prism section`, ErrUnsupported)
	}
	if trimOperandSectionDelta(target) != 0 || trimOperandSectionDelta(tool) != 0 {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Split does not admit an operand carrying its own section displacement`, ErrUnsupported)
	}
	if err := prismcells.AdmitSurfacePair(budget, prismcells.SurfaceSplit,
		prismcells.SurfaceOperandOf(rcv), prismcells.SurfaceOperandOf(tl)); err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	return rcv, tl, nil
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
	// Read S7 from both root payload shapes before the revolve-only gate.
	rcvDelta, tlDelta := trimOperandSectionDelta(b), trimOperandSectionDelta(tool)
	rcv, rcvView, tlView, err := revolveplan.AdmitTrimPair(ctx, budget,
		&b.n, &tool.n, rcvDelta, tlDelta)
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
// family. Everything past the gate is the prism arm's own: ResolveSurfaceExtend reads
// sketch's cut parameter off the recreated carrier in the two operands'
// MERIDIAN views (§3.1), and the widened range goes back into the receiver's
// own record. Only the gate and the free-edge reading differ.
func (b *Body) extendRevolve(ctx context.Context, budget *proofbound.WorkBudget, selected []*Edge, tool *Body) (*Body, error) {
	d := documentHandle(b.n.Doc)
	// Read S7 from both root payload shapes before the revolve-only gate.
	rcvDelta, tlDelta := trimOperandSectionDelta(b), trimOperandSectionDelta(tool)
	rcv, rcvView, tlView, err := revolveplan.AdmitExtendPair(ctx, budget,
		&b.n, &tool.n, rcvDelta, tlDelta)
	if err != nil {
		return nil, err
	}
	chains := make([]chainRecord, len(rcv.Chains))
	for i, chain := range rcv.Chains {
		chains[i] = chainRecord{Segments: append([]curveSegment(nil), chain.Segments...)}
	}
	cutDelta := 0.0
	for _, edge := range selected {
		ci, si, atStart, err := extendRevolveEndSegment(b, rcv, edge)
		if err != nil {
			return nil, err
		}
		widened, delta, err := prismcells.ResolveSurfaceExtend(ctx, budget,
			prismcells.SurfaceOperandOf(rcvView), prismcells.SurfaceOperandOf(tlView), chains[ci].Segments[si], atStart)
		if err != nil {
			return nil, err
		}
		chains[ci].Segments[si] = widened
		cutDelta = math.Max(cutDelta, delta)
	}
	rcv.Chains = chains
	// §7.1's fold reads this field as its gate, and ResolveSurfaceExtend's δ_cut is
	// the whole of it: S4 and S7 zero every other term prism §7 derives.
	rcv.SectionDelta = cutDelta
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

// extendRevolveEndSegment maps a selected topology edge to the recorded
// meridian end it names — extendEndSegment's own reading, re-derived for the
// chain-revolve build, whose free ends are SWEPT latitude circles rather than
// straight sweep edges.
//
// Under a full revolution buildChainRevolveWalls gives every wall face its
// junctions' latitude circles as SEPARATE single-coedge loops through
// fullRevLoops (internal/revolveplan/build.go), which sets forward true for the wall's own
// START junction and false for its END. That flag is the map: the receiver's
// two free ends are junction 0 of the FIRST wall and junction n of the LAST,
// and reading the flag rather than the loop position is what makes the reading
// survive fullRevLoops' own outer-loop swap. The first and last wall are never
// the skipped wallAxis kind — a segment lying on the axis puts both its ends at
// ρ == 0, which revolveaxis.RequireChainAxisIncidence already refuses at a chain's free end
// — so faces[0] and faces[len-1] are walk 0 and walk n-1.
//
// The loop indexes b.lumps and reads pp.chains at the SAME index, on
// extendEndSegment's own terms: revolveplan.BuildChain appends each chain's
// faces in chain order and topobuild.SheetLumps preserves first-appearance order, so the
// body's i-th lump is the payload's i-th chain. A producer that emits a lump
// per connected PIECE rather than per chain owes this function a recorded map
// rather than the shared index.
func extendRevolveEndSegment(b *Body, pp chainRevolvePayload, edge *Edge) (int, int, bool, error) {
	miss := func() (int, int, bool, error) {
		return 0, 0, false, fmt.Errorf(
			`%w: Extend needs a free swept edge at an open meridian end`, ErrUnsupported)
	}
	if !edge.IsFree() {
		return miss()
	}
	for ci, lump := range b.n.Lumps {
		if ci >= len(pp.Chains) || len(lump.Shells) != 1 || len(lump.Shells[0].Faces) == 0 {
			continue
		}
		faces := lump.Shells[0].Faces
		if named, ok := revolveLatitudeLoopEdge(faceHandle(faces[0]), true); ok && named == edge {
			return ci, 0, true, nil
		}
		if named, ok := revolveLatitudeLoopEdge(faceHandle(faces[len(faces)-1]), false); ok && named == edge {
			return ci, len(pp.Chains[ci].Segments) - 1, false, nil
		}
	}
	return miss()
}

// revolveLatitudeLoopEdge reads one wall face's own latitude loop for the
// junction fullRevLoops' forward flag names: true for the wall's start
// junction, false for its end. A junction ON the axis mints no latitude circle
// at all, so that end answers absent and the caller refuses.
func revolveLatitudeLoopEdge(f *Face, start bool) (*Edge, bool) {
	for _, l := range f.n.Loops {
		if len(l.CoEdges) != 1 || l.CoEdges[0].Forward != start {
			continue
		}
		return edgeHandle(l.CoEdges[0].Edge), true
	}
	return nil, false
}
