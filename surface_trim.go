package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/sketch"
)

// This file implements Body.Trim, Body.Extend and Document.Split. Their §2.1
// gates share prism-boolean's exact generator comparison, identity
// re-expression check and sweep-span relation. All three use buildPrismScene;
// Trim reads classifyPrismCells unchanged, while Split reads only the target
// label. Every miss is a typed refusal at the call: a sheet's mesh proves no
// occupied volume, and a mesh trim would decide topology from float signs on
// chorded triangles (docs/surface-intersection-design.md §2.1).
//
// Trim and Extend take the REVOLVE family too, over the two operands' meridian
// views: §11's PR4, this file's own last section. Split's revolve arm is staged
// (RS13) and refuses by name.

// Extend lengthens the receiver along the named edges' own carriers until
// they meet tool, and returns the lengthened sheet. It admits a receiver and
// tool sharing ONE generator, exactly as Trim does — both a straight sweep, or
// both a revolve about one axis — under surface-intersection §2.1's own gate.
func (b *Body) Extend(ctx context.Context, edges *EdgeQuery, tool *Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an extension`, ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the receiver belongs to no document`, ErrDegenerate)
	}
	d := b.doc
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
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
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
	chains := make([]ChainRecord, len(rcv.chains))
	for i, chain := range rcv.chains {
		chains[i] = ChainRecord{Segments: append([]CurveSegment(nil), chain.Segments...)}
	}
	cutDelta := 0.0
	for _, edge := range selected {
		ci, si, atStart, err := extendEndSegment(b, rcv, edge)
		if err != nil {
			return nil, err
		}
		widened, delta, err := resolveExtend(ctx, budget, rcv.prism(), tl, chains[ci].Segments[si], atStart)
		if err != nil {
			return nil, err
		}
		chains[ci].Segments[si] = widened
		cutDelta = math.Max(cutDelta, delta)
	}
	rcv.chains = chains
	rcv.sectionDelta = cutDelta
	result, err := evalChainExtrudeContext(ctx, d, d.nextProducerID(), rcv, newFreeformWork())
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
// one of two producers — ExtrudeChain (extrude.go) and this file's own Extend —
// and both build exactly one lump per chain in chain order through
// evalChainExtrudeContext; Trim's chainPayload is the only other one, and S7
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
	for ci, lump := range b.lumps {
		if ci >= len(pp.chains) || len(lump.shells) != 1 || len(lump.shells[0].faces) == 0 {
			continue
		}
		faces := lump.shells[0].faces
		first := faces[0].loops[0].coedges
		last := faces[len(faces)-1].loops[0].coedges
		if edge == first[3].edge {
			return ci, 0, true, nil
		}
		if edge == last[1].edge {
			return ci, len(pp.chains[ci].Segments) - 1, false, nil
		}
	}
	return 0, 0, false, fmt.Errorf(`%w: Extend needs a free sweep edge at an open section end`, ErrUnsupported)
}

// admitExtendPair applies S1-S4, S6 and S7. The selected segment enters the
// scene at full domain; every other receiver segment stays outside it.
func admitExtendPair(budget *workBudget, receiver, tool *Body) (chainPayload, prismPayload, error) {
	// A revolve pair never reaches here: Body.Extend routes it to its own arm
	// before this gate runs, exactly as Trim routes one.
	rf, tf := bodyTrimFamily(receiver), bodyTrimFamily(tool)
	if rf != tf || rf != trimFamilyPrism {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Extend needs a receiver and tool sharing a straight-sweep generator (receiver %s, tool %s)`,
			ErrUnsupported, trimFamilyName(rf), trimFamilyName(tf))
	}
	rcv, ok := receiver.payload.(chainPayload)
	if !ok || receiver.Kind() != BodySheet {
		return chainPayload{}, prismPayload{}, fmt.Errorf(`%w: Extend's receiver needs an open section`, ErrUnsupported)
	}
	var tl prismPayload
	switch p := tool.payload.(type) {
	case prismPayload:
		tl = p
	case chainPayload:
		tl = p.prism()
	default:
		return chainPayload{}, prismPayload{}, fmt.Errorf(`%w: Extend's tool has no prism section`, ErrUnsupported)
	}
	if rcv.sectionDelta != 0 || trimOperandSectionDelta(tool) != 0 {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Extend does not admit an operand carrying its own section displacement`, ErrUnsupported)
	}
	view := rcv.prism()
	if view.reflected() || tl.reflected() {
		return chainPayload{}, prismPayload{}, fmt.Errorf(`%w: Extend does not admit a reflected operand`, ErrUnsupported)
	}
	rcvAnalytic, err := prismProfileIsAnalytic(budget, view.profile)
	if err != nil {
		return chainPayload{}, prismPayload{}, err
	}
	tlAnalytic, err := prismProfileIsAnalytic(budget, tl.profile)
	if err != nil {
		return chainPayload{}, prismPayload{}, err
	}
	if !rcvAnalytic || !tlAnalytic {
		return chainPayload{}, prismPayload{}, fmt.Errorf(`%w: Extend admits only line, circle and arc segments`, ErrUnsupported)
	}
	// S4 uses the exact stored-float equality and literal zero of Trim.
	worldNormalRcv := view.xform.ApplyDir(view.frame.N())
	worldNormalTool := tl.xform.ApplyDir(tl.frame.N())
	if worldNormalRcv != worldNormalTool {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the receiver and tool do not sweep along the same generator, exactly`, ErrUnsupported)
	}
	worldOriginRcv := view.xform.Apply(view.frame.Origin())
	worldOriginTool := tl.xform.Apply(tl.frame.Origin())
	if worldOriginTool.Sub(worldOriginRcv).Dot(worldNormalRcv) != 0.0 {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the receiver and tool do not sweep along the same generator, exactly`, ErrUnsupported)
	}
	if !prismCutZIntervalSpans(view, tl) {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the tool does not span the receiver over the sweep parameter`, ErrUnsupported)
	}
	reexpress, err := newPrismReexpression(view, tl)
	if err != nil {
		return chainPayload{}, prismPayload{}, err
	}
	if !reexpress.identity {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the receiver and tool do not share one frame and placement, so their re-expression is not the identity`, ErrUnsupported)
	}
	tlWhole, err := trimProfileFullyWhole(budget, tl.profile)
	if err != nil {
		return chainPayload{}, prismPayload{}, err
	}
	if !tlWhole {
		return chainPayload{}, prismPayload{}, fmt.Errorf(
			`%w: every segment the tool consumes must span its entity's own natural domain`, ErrUnsupported)
	}
	return rcv, tl, nil
}

// fullExtendSegment copies the entity's defining data and names its natural
// parameter bounds, ALWAYS ascending — t from 0 to 1 — whichever way the
// receiver's own recorded range runs. No coordinate is computed, even for an
// arc or circle.
//
// The ascending order is what makes the scene entity's parameterisation the
// receiver's own, so §3.2's nearest-cut reading and the bound it stores live in
// ONE space and no map runs between them. This is not decad re-deriving a 2D
// answer: the cut parameter stays sketch's, and the only thing settled here is
// which of decad's own two parameterisations of decad's own recorded entity the
// scene is built in. The derivation is per kind, off buildPrismScene's own
// entity creation (prism_boolean.go), which reads walkOf's walked geometry of
// exactly the segment handed to it:
//
//   - LineSeg. buildPrismScene creates CreateLine(P(TStart), P(TEnd)). At
//     (0, 1) those are the record's own Start and End, so the scene line runs
//     Start to End and sketch's parameter is the record's own lerp parameter:
//     the identity. At (1, 0) it would run End to Start and the scene parameter
//     would be 1 − t, which is why the order is fixed here rather than mapped
//     back later.
//   - CircleSeg. buildPrismScene creates CreateCircle(centre, radius) from the
//     record's own two fields, so no walk direction reaches the entity at all.
//     walkOf indexes the record's t as the angle 2πt and sketch indexes its own
//     the same way, so the map is the identity in either sense; CCW is set true
//     only because walkOf refuses a CCW flag contradicting an ascending range.
//   - ArcSeg. buildPrismScene creates CreateArc(centre, lo, hi) with lo and hi
//     the walked endpoints put in ascending-angle order. At (0, 1)
//     pinArcWalkEnds (segment_walk.go) pins them to the record's own Start and
//     End verbatim and th1 > th0 leaves them unswapped, so the scene arc sweeps
//     CCW from Start to End — the very angle interval a0 → a0 + sweep the
//     record's own t indexes. The identity.
//
// No arithmetic runs on any parameter here, so no bound is charged: every field
// written is either a recorded field copied verbatim or one of the two literal
// natural bounds.
func fullExtendSegment(seg CurveSegment) (CurveSegment, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return nil, err
	}
	switch s := seg.(type) {
	case LineSeg:
		s.TStart, s.TEnd = 0, 1
		return s, nil
	case CircleSeg:
		s.CCW, s.TStart, s.TEnd = true, 0, 1
		return s, nil
	case ArcSeg:
		s.TStart, s.TEnd = 0, 1
		return s, nil
	default:
		return nil, fmt.Errorf(`%w: a %T segment has no admitted full domain`, ErrUnsupported, seg)
	}
}

// extendCarrierDomain names the carrier a refusal is about — its kind, its own
// defining coordinates and its natural parameter range — so RS4's message
// states the domain the tool missed rather than only that it missed one
// (docs/surface-design.md §15's T179).
func extendCarrierDomain(seg CurveSegment) string {
	switch s := seg.(type) {
	case LineSeg:
		return fmt.Sprintf("the line from (%v, %v) to (%v, %v) over t in [0, 1]",
			s.Start.U, s.Start.V, s.End.U, s.End.V)
	case CircleSeg:
		return fmt.Sprintf("the circle of radius %v about (%v, %v) over t in [0, 1]",
			s.Radius, s.Center.U, s.Center.V)
	case ArcSeg:
		return fmt.Sprintf("the arc about (%v, %v) from (%v, %v) to (%v, %v) over t in [0, 1]",
			s.Center.U, s.Center.V, s.Start.U, s.Start.V, s.End.U, s.End.V)
	default:
		return fmt.Sprintf("a %T carrier over t in [0, 1]", seg)
	}
}

func extendSetBound(seg CurveSegment, atStart bool, bound float64) CurveSegment {
	switch s := seg.(type) {
	case LineSeg:
		if atStart {
			s.TStart = bound
		} else {
			s.TEnd = bound
		}
		return s
	case CircleSeg:
		if atStart {
			s.TStart = bound
		} else {
			s.TEnd = bound
		}
		return s
	case ArcSeg:
		if atStart {
			s.TStart = bound
		} else {
			s.TEnd = bound
		}
		return s
	}
	return seg
}

// resolveExtend reads the nearest cut from sketch's parameter order on the
// recreated entity, then widens only the receiver's named recorded bound.
//
// EVERY parameter this function compares or stores is in the RECEIVER'S OWN
// recorded parameterisation, and that is the whole of the space discipline
// here: the direction test below reads the record's TStart/TEnd, the stored
// bound is written back into them, and the candidates arrive in the same space
// because fullExtendSegment recreates the scene entity in the record's own
// ascending order (its own comment derives the identity per segment kind).
// Reading a candidate in the SCENE's order and storing it in the RECORD's would
// publish a boundary at 1 − t for a reversed LineSeg receiver — a wrong
// boundary, not a refusal. The candidates themselves stay sketch's own cut
// parameters, untouched: decad selects among them and never computes one.
// view is the receiver's own section view — a prism ribbon's prism() or a
// revolve ribbon's meridian() — carrying the frame and placement the scene is
// built in; its profile is replaced below by the one recreated carrier.
func resolveExtend(ctx context.Context, budget *workBudget, view prismPayload, tool prismPayload,
	seg CurveSegment, atStart bool) (CurveSegment, float64, error) {
	t0, t1, err := trimSegmentParamRange(seg)
	if err != nil {
		return nil, 0, err
	}
	old := t1
	if atStart {
		old = t0
	}
	if old == 0 || old == 1 {
		return nil, 0, fmt.Errorf(`%w: the named section end already reaches its carrier's own domain`, ErrUnsupported)
	}
	full, err := fullExtendSegment(seg)
	if err != nil {
		return nil, 0, err
	}
	view.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{full}}}
	segments, withinCap, err := prismSceneWithinWorkCap(budget, view, tool)
	if err != nil {
		return nil, 0, err
	}
	if !withinCap {
		return nil, 0, fmt.Errorf(`%w: the extend scene charges %d segments against the cap of %d`,
			ErrUnsupported, segments, prismMaxArrangementSegments)
	}
	reexpress, err := newPrismReexpression(view, tool)
	if err != nil {
		return nil, 0, err
	}
	s, tags, _, err := buildPrismScene(budget, view, tool, reexpress)
	if err != nil {
		return nil, 0, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, 0, err
	}
	if err := budget.err(); err != nil {
		return nil, 0, err
	}
	// The receiver side of this scene holds exactly ONE entity: §3.1's
	// Extend scene carries the extended segment's own carrier and the tool's
	// section and nothing else, and view.profile above was replaced by a
	// single-segment loop for precisely that reason. So breaking out of the
	// map range is deterministic — there is one non-isB tag to find, and map
	// iteration order cannot change which. Give the Extend scene a second
	// receiver segment and this stops holding: it would then pick an arbitrary
	// one of them, and the pick would vary run to run.
	var source sketch.Entity
	for entity, tag := range tags {
		if !tag.isB {
			source = entity
			break
		}
	}
	if source == nil {
		return nil, 0, fmt.Errorf(`%w: the extended carrier has no scene entity`, ErrUnsupported)
	}
	var fragments []sketch.BoundaryEdge
	for _, profile := range profiles {
		if !profile.Valid {
			return nil, 0, fmt.Errorf(`%w: the extend arrangement has an invalid cell`, ErrUnsupported)
		}
		for _, loop := range append([][]sketch.BoundaryEdge{profile.Outer}, profile.Holes...) {
			fragments = append(fragments, loop...)
		}
	}
	chains, err := extendChainsContext(ctx, s.Chains)
	if err != nil {
		return nil, 0, err
	}
	if err := budget.err(); err != nil {
		return nil, 0, err
	}
	for _, chain := range chains {
		if !chain.Valid {
			return nil, 0, fmt.Errorf(`%w: the extend arrangement has an invalid chain`, ErrUnsupported)
		}
		fragments = append(fragments, chain.Edges...)
	}
	// Which way the named bound widens, read off the record's own range order
	// alone. TStart is the covered interval's LOWER bound when the record runs
	// ascending and its UPPER bound when it runs reversed, so widening TStart
	// decreases it in the first case and increases it in the second; TEnd is
	// the mirror. forward means "increasing t".
	forward := t1 > t0
	if atStart {
		forward = !forward
	}
	nearest := 0.0
	found := false
	for _, edge := range fragments {
		if err := budget.step(); err != nil {
			return nil, 0, err
		}
		if edge.Entity != source || !edge.Partial {
			continue
		}
		for _, candidate := range []float64{edge.TStart, edge.TEnd} {
			if candidate == 0 || candidate == 1 {
				continue
			}
			if forward && candidate <= old || !forward && candidate >= old {
				continue
			}
			if !found || forward && candidate < nearest || !forward && candidate > nearest {
				nearest, found = candidate, true
			}
		}
	}
	if !found {
		return nil, 0, fmt.Errorf(
			`%w: the tool has no cut past the named end at t = %v inside the carrier's own natural domain, which is %s`,
			ErrUnsupported, old, extendCarrierDomain(seg))
	}
	for _, edge := range fragments {
		if edge.Entity != source || edge.TStart != nearest && edge.TEnd != nearest {
			continue
		}
		if _, err := recordEdge(edge); err != nil {
			return nil, 0, err
		}
		break
	}
	widened := extendSetBound(seg, atStart, nearest)
	delta, err := prismUnionCutDelta(sketch.BoundaryEdge{Partial: true}, widened)
	if err != nil {
		return nil, 0, err
	}
	return widened, delta, nil
}

// extendChainsContext gives the second bounded arrangement publication the
// same cancellation discipline as prismProfilesContext.
func extendChainsContext(ctx context.Context, chains func() []*sketch.Chain) ([]*sketch.Chain, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	done := make(chan []*sketch.Chain)
	go func() { done <- chains() }()
	select {
	case result := <-done:
		return result, nil
	case <-ctx.Done():
		<-done
		return nil, ctx.Err()
	}
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

// Trim cuts the receiver where it meets tool and returns the kept pieces as
// one sheet body (docs/surface-intersection-design.md §8). The receiver and
// tool are consumed on the document's uniform terms (core §6); a refusal
// consumes neither. It admits a pair whose two sweeps share ONE generator —
// both the prism family (a straight Extrude/ExtrudeChain sweep) or both the
// revolve family (a Revolve/RevolveChain spin about one axis,
// docs/api-design.md's evaluator §5). A mixed pair, or any condition §2.1's
// S1-S7 states, is [ErrUnsupported] at the call, never a fallback: a sheet's
// mesh proves no occupied volume (boolean.go's requireVolumeProvingPayload),
// so there is no mesh path for a miss to reroute to. A tool that separates no
// fragment of the receiver, or every fragment, is [ErrDegenerate] (R30): no
// trimmed body exists either way.
func (b *Body) Trim(ctx context.Context, tool *Body, side TrimSide) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a trim`, ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the receiver belongs to no document`, ErrDegenerate)
	}
	d := b.doc
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

	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
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

	chains, sectionDelta, err := resolveTrim(ctx, budget, rcv, tl, side == KeepInside)
	if err != nil {
		return nil, err
	}

	pp := chainPayload{
		chains:       chains,
		frame:        rcv.frame,
		z0:           rcv.z0,
		z1:           rcv.z1,
		z0Delta:      rcv.z0Delta,
		z1Delta:      rcv.z1Delta,
		xform:        rcv.xform,
		sectionDelta: sectionDelta,
	}
	ref := d.nextProducerID()
	body, err := evalChainExtrudeContext(ctx, d, ref, pp, newFreeformWork())
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
// whether the body came from Extrude or ExtrudeChain, Revolve or
// RevolveChain.
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
	switch b.payload.(type) {
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
	switch p := b.payload.(type) {
	case prismPayload:
		return p.sectionDelta
	case chainPayload:
		return p.sectionDelta
	case revolvePayload:
		return p.sectionDelta
	case chainRevolvePayload:
		return p.sectionDelta
	default:
		return 0
	}
}

// admitTrimPair runs S1-S7 (docs/surface-intersection-design.md §2.1) for a
// Trim over the prism family. Every miss is ErrUnsupported at the call —
// there is no mesh fallback for this family of operations (§2.1). The
// revolve family is PR4's hook: S1 refuses it exactly as it refuses a mixed
// pair, since this file builds no revolve-side admission.
func admitTrimPair(budget *workBudget, receiver, tool *Body) (rcv, tl prismPayload, err error) {
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
	rcv, rcvOk := receiver.payload.(prismPayload)
	if !rcvOk {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim's receiver section is an open walk, which closes onto nothing for the tool to trim`, ErrUnsupported)
	}
	tl, tlOk := tool.payload.(prismPayload)
	if !tlOk {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim's tool section is an open walk, which bounds no region to keep a side of`, ErrUnsupported)
	}

	// S2: neither operand's accumulated placement is a reflection.
	if rcv.reflected() || tl.reflected() {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim does not admit a reflected operand`, ErrUnsupported)
	}

	// S3: every segment of both operands' records is a LineSeg, CircleSeg or
	// ArcSeg — the whole-scene TExact gate blinds on a free-form one.
	rcvAnalytic, err := prismProfileIsAnalytic(budget, rcv.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	tlAnalytic, err := prismProfileIsAnalytic(budget, tl.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	if !rcvAnalytic || !tlAnalytic {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim admits only line, circle and arc segments; a free-form segment blinds sketch's whole-scene TExact gate`, ErrUnsupported)
	}

	// S4: the generators are the same, exactly — prism G3's test unchanged,
	// Go == on the stored r3.Vec floats and a dot product against the
	// literal zero (docs/prism-boolean-design.md §3.1).
	worldNormalRcv := rcv.xform.ApplyDir(rcv.frame.N())
	worldNormalTool := tl.xform.ApplyDir(tl.frame.N())
	if worldNormalRcv != worldNormalTool {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the receiver and tool do not sweep along the same generator, exactly`, ErrUnsupported)
	}
	worldOriginRcv := rcv.xform.Apply(rcv.frame.Origin())
	worldOriginTool := tl.xform.Apply(tl.frame.Origin())
	if worldOriginTool.Sub(worldOriginRcv).Dot(worldNormalRcv) != 0.0 {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the receiver and tool do not sweep along the same generator, exactly`, ErrUnsupported)
	}

	// S5: for Trim, the tool's section is one closed hole-free loop.
	if len(tl.profile.Holes) != 0 {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Trim's tool section carries a hole, whose interior this evaluator cannot yet distinguish as a side`, ErrUnsupported)
	}

	// S6: the tool spans the receiver over the sweep parameter — the
	// identical relation Cut's own G5 states (prism_boolean_nesting.go).
	if !prismCutZIntervalSpans(rcv, tl) {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the tool does not span the receiver over the sweep parameter`, ErrUnsupported)
	}

	// S7's remaining two clauses: the re-expression is the identity in the
	// stored floats, and every segment either operand's own record consumes
	// spans its entity's natural domain. The section-displacement clause was
	// already checked above, generically over either payload shape.
	reexpress, err := newPrismReexpression(rcv, tl)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	if !reexpress.identity {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the receiver and tool do not share one frame and placement, so their re-expression is not the identity`, ErrUnsupported)
	}
	rcvWhole, err := trimProfileFullyWhole(budget, rcv.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	tlWhole, err := trimProfileFullyWhole(budget, tl.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	if !rcvWhole || !tlWhole {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: every segment the receiver or tool consumes must span its entity's own natural domain`, ErrUnsupported)
	}

	return rcv, tl, nil
}

// trimProfileFullyWhole reports whether every segment of every loop of p
// spans its entity's own natural domain (S7's third clause) — read off the
// recorded range through wholeSegmentRange, the identical prism-boolean
// reading (prism_boolean.go), and never off any walk's closed-ness. Stating
// it this way is a record property, decidable before the arrangement runs,
// which is what lets every miss here be one refusal at the call
// (docs/surface-intersection-design.md §2.1 S7).
func trimProfileFullyWhole(budget *workBudget, p ProfileRecord) (bool, error) {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			if err := budget.step(); err != nil {
				return false, err
			}
			whole, err := trimSegmentIsWhole(seg)
			if err != nil {
				return false, err
			}
			if !whole {
				return false, nil
			}
		}
	}
	return true, nil
}

// trimSegmentIsWhole reads one S3-admitted segment's own recorded range
// through wholeSegmentRange (prism_boolean.go) — exact float equality
// against 0 and 1, never a tolerance in either direction.
func trimSegmentIsWhole(seg CurveSegment) (bool, error) {
	t0, t1, err := trimSegmentParamRange(seg)
	if err != nil {
		return false, err
	}
	return wholeSegmentRange(t0, t1), nil
}

// trimSegmentParamRange reads an S3-admitted segment's own recorded TStart/
// TEnd — the one place decad names, per field, whether that end is the
// entity's own natural bound or a coordinate the arrangement computed. Every
// admitted kind (LineSeg, CircleSeg, ArcSeg) carries the two fields directly.
func trimSegmentParamRange(seg CurveSegment) (t0, t1 float64, err error) {
	seg, err = normalizeSegment(seg)
	if err != nil {
		return 0, 0, err
	}
	switch s := seg.(type) {
	case LineSeg:
		return s.TStart, s.TEnd, nil
	case CircleSeg:
		return s.TStart, s.TEnd, nil
	case ArcSeg:
		return s.TStart, s.TEnd, nil
	default:
		return 0, 0, fmt.Errorf(`%w: a %T segment is not part of the admitted class`, ErrUnsupported, seg)
	}
}

// trimCutChargeUV is trimBoundsWalks's own per-component reading of §7's
// δ_cut: how far a cut endpoint's u and v coordinates can EACH sit from the
// crossing they denote, given cutDisplacementAllow's own parameter-to-
// coordinate scaling (bounds.go). A CircleSeg/ArcSeg's position varies with
// BOTH components under a cos/sin walk, so both take carrierSpeedUpper's
// existing isotropic reading (prism_boolean.go) unchanged. A LineSeg's does
// not: its walk is Start + t·(End−Start), so a coordinate whose OWN
// End−Start difference is exactly zero — a horizontal line's v, a vertical
// line's u — carries no displacement AT ALL as t moves, however uncertain t
// itself is, and charging it anyway would smear a widened cut candidate's
// slop onto an axis the cut never touches (T170's own top and bottom walls,
// whose v never moves at any parameter). Each component's own exact
// End−Start difference, taken over the recorded floats and rounded outward
// (ratL1Upper), is what carrierSpeedUpper already reduces to a single L1
// figure for the isotropic case; reading it per component instead is the
// same mechanism, not a new one.
func trimCutChargeUV(seg CurveSegment) (chargeU, chargeV float64, err error) {
	seg, err = normalizeSegment(seg)
	if err != nil {
		return 0, 0, err
	}
	if line, ok := seg.(LineSeg); ok {
		du := ratL1Upper(exactCoordinateDelta(line.End.U, line.Start.U))
		dv := ratL1Upper(exactCoordinateDelta(line.End.V, line.Start.V))
		return cutDisplacementAllow(du), cutDisplacementAllow(dv), nil
	}
	speed, err := carrierSpeedUpper(seg)
	if err != nil {
		return 0, 0, err
	}
	charge := cutDisplacementAllow(speed)
	return charge, charge, nil
}

// trimRevolveSegmentCharges is trimBoundsWalks's own per-endpoint rule, read
// for the revolve family (docs/surface-intersection-design.md §7.1): the
// per-component charge trimCutChargeUV states lands at exactly the endpoint
// whose OWN recorded parameter is not a natural bound (0 or 1, per field —
// never per segment, since a segment cut on only one side keeps its natural
// end exact), and at NEITHER endpoint of a segment the arrangement did not
// cut. The decision reads only the two recorded floats.
//
// delta is the payload's own sectionDelta, read as a gate and never as the
// charge: a payload no construction displaced has every recorded parameter
// natural anyway, so the two answers would be absent regardless, and gating
// keeps an ordinary RevolveChain off this path entirely.
func trimRevolveSegmentCharges(seg CurveSegment, delta float64) (walkEndBound, walkEndBound, error) {
	if delta == 0 {
		return walkEndBound{}, walkEndBound{}, nil
	}
	t0, t1, err := trimSegmentParamRange(seg)
	if err != nil {
		return walkEndBound{}, walkEndBound{}, err
	}
	chargeU, chargeV, err := trimCutChargeUV(seg)
	if err != nil {
		return walkEndBound{}, walkEndBound{}, err
	}
	var start, end walkEndBound
	if t0 != 0 && t0 != 1 {
		start = walkEndBound{u: chargeU, v: chargeV}
	}
	if t1 != 0 && t1 != 1 {
		end = walkEndBound{u: chargeU, v: chargeV}
	}
	return start, end, nil
}

// trimBoundsWalks resolves profile's Outer-then-Holes walks for a trimmed
// ribbon's own Bounds reading (docs/surface-intersection-design.md §7),
// charging trimCutChargeUV into exactly the endpoint bound whose OWN
// recorded parameter is not a natural bound (0 or 1, per field — never per
// segment, since a segment cut on only one side keeps its natural end exact)
// — the coordinate the arrangement computed for that end, never one the
// record states verbatim. An endpoint whose own parameter IS natural gets no
// charge, whether or not the segment's OTHER end was cut, which is what lets
// an extreme won by an untouched vertex stay exactly as tight as the
// untrimmed sheet's own (T170) while an extreme won by a genuine cut point
// carries the bound its own construction owes (docs/surface-design.md's
// end-cut fixture). The decision reads only the two recorded floats — never a
// coordinate comparison, never whether a value "looks like" it moved.
//
// This is never called for a plain ExtrudeChain ribbon: evalChainExtrudeContext
// gates it on pp.sectionDelta != 0, which no construction but this design's
// Trim ever sets, so an ordinary ribbon keeps resolving through walkOf with
// no augmentation.
func trimBoundsWalks(profile ProfileRecord, work *freeformWork) (*profileWalks, error) {
	before, beforeRecon := workSpent(work)
	walkCharged := func(seg CurveSegment) (segmentWalk, error) {
		w, err := walkOf(seg, work)
		if err != nil {
			return segmentWalk{}, err
		}
		t0, t1, err := trimSegmentParamRange(seg)
		if err != nil {
			return segmentWalk{}, err
		}
		chargeU, chargeV, err := trimCutChargeUV(seg)
		if err != nil {
			return segmentWalk{}, err
		}
		if t0 != 0 && t0 != 1 {
			w.startBound = walkEndBound{
				u: absSumUpper(w.startBound.u, chargeU),
				v: absSumUpper(w.startBound.v, chargeV),
			}
		}
		if t1 != 0 && t1 != 1 {
			w.endBound = walkEndBound{
				u: absSumUpper(w.endBound.u, chargeU),
				v: absSumUpper(w.endBound.v, chargeV),
			}
		}
		return w, nil
	}

	outer := make([]segmentWalk, len(profile.Outer.Segments))
	for i, seg := range profile.Outer.Segments {
		w, err := walkCharged(seg)
		if err != nil {
			return nil, err
		}
		outer[i] = w
	}
	holes := make([][]segmentWalk, len(profile.Holes))
	for hi, hole := range profile.Holes {
		hw := make([]segmentWalk, len(hole.Segments))
		for i, seg := range hole.Segments {
			w, err := walkCharged(seg)
			if err != nil {
				return nil, err
			}
			hw[i] = w
		}
		holes[hi] = hw
	}
	after, afterRecon := workSpent(work)
	return &profileWalks{
		profile:             profile,
		outer:               outer,
		holes:               holes,
		spent:               after - before,
		reconstructionSpent: afterRecon - beforeRecon,
		metered:             true,
	}, nil
}

// resolveTrim is §3's design over an admitted pair: buildPrismScene's own
// scene (§3.1, reused unchanged), the structural no-crossing check and
// classifyPrismCells's side reading (§3.2), and §3.3's open-walk chaining.
// keepInside selects classifyPrismCells's own tool-membership label a
// surviving fragment must carry.
func resolveTrim(ctx context.Context, budget *workBudget, rcv, tl prismPayload, keepInside bool) ([]ChainRecord, float64, error) {
	segments, withinCap, err := prismSceneWithinWorkCap(budget, rcv, tl)
	if err != nil {
		return nil, 0, err
	}
	if !withinCap {
		return nil, 0, fmt.Errorf(
			`%w: the trim scene charges at least %d arranger segments against this evaluator's cap of %d; simplify the receiver or tool before trimming`,
			ErrUnsupported, segments, prismMaxArrangementSegments)
	}

	// S7 already proved this is the identity; buildPrismScene still takes it
	// as an explicit argument, exactly as prism-boolean's own callers do.
	reexpress, err := newPrismReexpression(rcv, tl)
	if err != nil {
		return nil, 0, err
	}

	s, tags, _, err := buildPrismScene(budget, rcv, tl, reexpress)
	if err != nil {
		return nil, 0, err
	}
	if err := budget.err(); err != nil {
		return nil, 0, err
	}

	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, 0, err
	}
	if err := budget.err(); err != nil {
		return nil, 0, err
	}
	if len(profiles) == 0 {
		return nil, 0, fmt.Errorf(`%w: the receiver and tool's arrangement holds no bounded cell`, ErrUnsupported)
	}

	// The structural no-crossing check (this file's own trimNoCrossingSide)
	// runs BEFORE classifyPrismCells: a cell carrying a hole — the shape
	// every no-crossing configuration produces, tool nested in receiver or
	// receiver nested in tool — is explicitly outside classifyPrismCells's
	// own hole-free scope (prism_boolean_crossing.go), and a wholly disjoint
	// pair leaves the two operands' cells with no edge in common for its
	// propagation to reach across either. All three are genuine trim
	// answers, not unresolved topology, so they are read structurally rather
	// than reported as a classifier miss.
	insideTool, noCrossing, err := trimNoCrossingSide(budget, tags, profiles, rcv)
	if err != nil {
		return nil, 0, err
	}
	if noCrossing {
		if insideTool == keepInside {
			return nil, 0, fmt.Errorf(`%w: the tool separates no fragment of the receiver; every fragment is kept`, ErrDegenerate)
		}
		return nil, 0, fmt.Errorf(`%w: the tool separates no fragment of the receiver; none is kept`, ErrDegenerate)
	}

	matterRcv, matterTool, resolved, err := classifyPrismCells(budget, tags, profiles)
	if err != nil {
		return nil, 0, err
	}
	if !resolved {
		return nil, 0, fmt.Errorf(`%w: the receiver and tool's arrangement is not one this evaluator's crossing classifier resolves`, ErrUnsupported)
	}

	survivors, total, err := trimSurvivingFragments(budget, tags, matterRcv, matterTool, profiles, keepInside)
	if err != nil {
		return nil, 0, err
	}
	if len(survivors) == 0 {
		return nil, 0, fmt.Errorf(`%w: the tool separates no fragment of the receiver; none is kept`, ErrDegenerate)
	}
	if len(survivors) == total {
		return nil, 0, fmt.Errorf(`%w: the tool separates no fragment of the receiver; every fragment is kept`, ErrDegenerate)
	}

	walks, resolved, err := chainTrimSurvivorWalks(budget, survivors)
	if err != nil {
		return nil, 0, err
	}
	if !resolved {
		return nil, 0, fmt.Errorf(`%w: the surviving fragments do not chain into open walks this evaluator resolves`, ErrUnsupported)
	}

	// Point of no return: every further problem (a rejected TExact fragment,
	// RS9; a walk that does not join at an interior junction, RS10) is
	// genuine.
	chains := make([]ChainRecord, len(walks))
	cutDelta := 0.0
	for wi, walk := range walks {
		segs := make([]CurveSegment, len(walk))
		joins := make([]loopJoin, len(walk))
		for i, e := range walk {
			if err := budget.step(); err != nil {
				return nil, 0, err
			}
			seg, err := recordEdge(e)
			if err != nil {
				return nil, 0, err
			}
			segs[i] = seg
			join, err := edgeJoin(e, seg)
			if err != nil {
				return nil, 0, err
			}
			joins[i] = join
			d, err := prismUnionCutDelta(e, seg)
			if err != nil {
				return nil, 0, err
			}
			cutDelta = math.Max(cutDelta, d)
		}
		if err := falsifyChainJoins(joins); err != nil {
			return nil, 0, err
		}
		chains[wi] = ChainRecord{Segments: segs}
	}
	return chains, cutDelta, nil
}

// trimNoCrossingSide decides, without classifyPrismCells, whether the tool's
// boundary crosses the receiver's at all. Three structural shapes answer it,
// each a pure data comparison against buildPrismScene's own tag map
// (prismFindLoopMatch, prism_boolean_nesting.go) rather than a geometric
// test: the receiver's own cell, untouched, carrying no further hole (wholly
// disjoint); the receiver's own cell, untouched, carrying the tool's own
// solid as one further hole (the tool nested inside the receiver); or the
// tool's own cell, untouched, carrying the receiver's own outer as its one
// hole (the receiver nested inside the tool — S5 already keeps the tool
// itself hole-free). resolved=false means none of the three applies, so the
// tool's boundary genuinely crosses the receiver's and classifyPrismCells's
// own propagation is what answers it.
func trimNoCrossingSide(budget *workBudget, tags map[sketch.Entity]prismEntityOrigin, profiles []*sketch.Profile, rcv prismPayload) (insideTool, resolved bool, err error) {
	rcvOuter, err := prismLoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return false, false, err
	}
	rcvHoles := make([]map[sketch.Entity]struct{}, len(rcv.profile.Holes))
	for i := range rcv.profile.Holes {
		hs, err := prismLoopEntitySet(budget, tags, false, i)
		if err != nil {
			return false, false, err
		}
		rcvHoles[i] = hs
	}
	toolOuter, err := prismLoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return false, false, err
	}

	invalid := func(match *sketch.Profile) error {
		if match.Valid {
			return nil
		}
		return fmt.Errorf(`%w: the trim scene's arrangement reports an invalid region`, ErrUnsupported)
	}

	if disjointMatch, ok, err := prismFindLoopMatch(budget, profiles, rcvOuter, rcvHoles); err != nil {
		return false, false, err
	} else if ok {
		return false, true, invalid(disjointMatch)
	}

	toolInsideHoles := append(append([]map[sketch.Entity]struct{}{}, rcvHoles...), toolOuter)
	if toolInsideMatch, ok, err := prismFindLoopMatch(budget, profiles, rcvOuter, toolInsideHoles); err != nil {
		return false, false, err
	} else if ok {
		return false, true, invalid(toolInsideMatch)
	}

	if rcvInsideMatch, ok, err := prismFindLoopMatch(budget, profiles, toolOuter, []map[sketch.Entity]struct{}{rcvOuter}); err != nil {
		return false, false, err
	} else if ok {
		return true, true, invalid(rcvInsideMatch)
	}

	return false, false, nil
}

// trimSurvivingFragments reads §3.2's Trim side: for every RECEIVER boundary
// fragment, classifyPrismCells's own tool-side reading for the cell that is
// the RECEIVER's own material (matterRcv true) decides its fate.
//
// A fragment's OTHER side is usually the unbounded face s.Profiles() does not
// return, in which case matterRcv is true for the one cell it bounds and
// "one cell settles it" (§3.2) needs nothing further. It is a SECOND bounded
// cell only where the tool's own section extends past the receiver's on that
// side — a tool taller or wider than the sheet it trims, which every fixture
// here that spans the sheet axially produces on its unswept sides — and that
// second cell is never the receiver's own material (matterRcv false there),
// so filtering on matterRcv alone picks the one occurrence that matters
// without a special case for which shape produced it. Two occurrences BOTH
// reporting matterRcv true is RS5's own coincident carrier: the tool's
// boundary runs exactly along the receiver's, so a receiver fragment bounds
// the receiver's material on both sides at once.
//
// keepInside selects the survivors: false keeps a fragment whose settling
// cell classifyPrismCells puts outside the tool's material, true keeps one
// it puts inside. total is the count of distinct receiver fragments the
// arrangement produced, for RS6's "keeps every fragment or none" check.
func trimSurvivingFragments(budget *workBudget, tags map[sketch.Entity]prismEntityOrigin, matterRcv, matterTool []bool, profiles []*sketch.Profile, keepInside bool) (survivors []sketch.BoundaryEdge, total int, err error) {
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	type settled struct {
		edge   sketch.BoundaryEdge
		inTool bool
	}
	byKey := map[edgeKey]settled{}
	var order []edgeKey
	for i, p := range profiles {
		if err := budget.step(); err != nil {
			return nil, 0, err
		}
		if !p.Valid {
			return nil, 0, fmt.Errorf(`%w: a cell the trim resolution depends on is not valid`, ErrUnsupported)
		}
		if !matterRcv[i] {
			continue // not the receiver's own material: never the settling cell
		}
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.step(); err != nil {
					return nil, 0, err
				}
				origin, ok := tags[e.Entity]
				if !ok {
					return nil, 0, fmt.Errorf(`decad: a trim arrangement entity traces to neither operand`)
				}
				if origin.isB {
					continue // tool-sourced: never part of a trimmed result
				}
				key := edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}
				if _, dup := byKey[key]; dup {
					return nil, 0, fmt.Errorf(`%w: a receiver boundary fragment coincides with the tool's own boundary`, ErrUnsupported)
				}
				byKey[key] = settled{edge: e, inTool: matterTool[i]}
				order = append(order, key)
			}
		}
	}
	total = len(order)
	for _, key := range order {
		s := byKey[key]
		if s.inTool == keepInside {
			survivors = append(survivors, s.edge)
		}
	}
	return survivors, total, nil
}

// chainTrimSurvivorWalks is §3.3's open-walk chaining: chainPrismUnionSurvivors
// (prism_boolean.go) generalized two ways, and nothing else — the walk is not
// required to close, and a survivor set that falls into several runs yields
// several walks rather than failing. Connectivity reads sketch's own walked
// Polyline endpoints exactly as chainPrismUnionSurvivors does: bookkeeping on
// an answer sketch already computed, never a re-derived geometric fact.
// resolved=false (err always nil in that case) means the survivors do not
// partition cleanly into dangling-ended runs — a shape this evaluator does
// not cover.
func chainTrimSurvivorWalks(budget *workBudget, survivors []sketch.BoundaryEdge) ([][]sketch.BoundaryEdge, bool, error) {
	type endpoints struct{ start, end Point2 }
	pts := make([]endpoints, len(survivors))
	byStart := make(map[Point2]int, len(survivors))
	for i, e := range survivors {
		if err := budget.step(); err != nil {
			return nil, false, err
		}
		if len(e.Polyline) < 2 {
			return nil, false, nil // defensive: no walked endpoints to key on
		}
		start := Point2{U: e.Polyline[0][0], V: e.Polyline[0][1]}
		end := Point2{U: e.Polyline[len(e.Polyline)-1][0], V: e.Polyline[len(e.Polyline)-1][1]}
		if _, dup := byStart[start]; dup {
			return nil, false, nil // ambiguous: more than one survivor leaves this vertex
		}
		byStart[start] = i
		pts[i] = endpoints{start: start, end: end}
	}
	hasIncoming := make(map[Point2]bool, len(survivors))
	for _, p := range pts {
		hasIncoming[p.end] = true
	}

	used := make([]bool, len(survivors))
	var walks [][]sketch.BoundaryEdge
	for i := range survivors {
		if used[i] || hasIncoming[pts[i].start] {
			continue // not a run's own start: reached by following its predecessor
		}
		var walk []sketch.BoundaryEdge
		cur := i
		for {
			if err := budget.step(); err != nil {
				return nil, false, err
			}
			used[cur] = true
			walk = append(walk, survivors[cur])
			next, ok := byStart[pts[cur].end]
			if !ok {
				break // a free end: this run is done
			}
			if used[next] {
				return nil, false, nil // a cycle folding back without a dangling start
			}
			cur = next
		}
		walks = append(walks, walk)
	}
	for _, u := range used {
		if !u {
			return nil, false, nil // a pure cycle among the survivors: not a shape this covers
		}
	}
	return walks, true, nil
}

// Split cuts target with tool and returns one solid per arranged target cell
// in sketch's cell order. Both operands are consumed only after every piece
// has been built. A tool that separates no target cell is ErrDegenerate.
func (d *Document) Split(ctx context.Context, target, tool *Body) ([]*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a split`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a split needs a document`, ErrDegenerate)
	}
	if err := d.requireLive(target); err != nil {
		return nil, err
	}
	if err := d.requireLive(tool); err != nil {
		return nil, err
	}
	if target == tool {
		return nil, fmt.Errorf(`%w: a split needs two distinct bodies`, ErrDegenerate)
	}
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return nil, err
	}
	rcv, tl, err := admitSplitPair(budget, target, tool)
	if err != nil {
		return nil, err
	}
	pieces, err := resolveSplit(ctx, budget, rcv, tl)
	if err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	bodies := make([]*Body, len(pieces))
	for i, piece := range pieces {
		if err := budget.step(); err != nil {
			return nil, err
		}
		bodies[i], err = evalPrismContext(ctx, d, ref+producerID(i), piece, newFreeformWork())
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
// The closed-sheet and chain-sheet views share buildPrismScene's input shape.
func admitSplitPair(budget *workBudget, target, tool *Body) (prismPayload, prismPayload, error) {
	rf, tf := bodyTrimFamily(target), bodyTrimFamily(tool)
	// RS13: a revolve pair clears S1 and is refused BY NAME, ahead of the
	// mixed-pair message, so the refusal states the staging rather than
	// claiming the two generators differ. What it waits on is stated in
	// docs/surface-intersection-design.md §3.4: a split piece is a SOLID
	// revolve, and §7.1 derives the section displacement's reach into the area
	// and the box alone, never into the Pappus volume and centroid a solid
	// publishes.
	if rf == trimFamilyRevolve && tf == trimFamilyRevolve {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Split over the revolve family waits on a solid revolve's own section-displacement terms; this evaluator charges them for a sheet's area and box alone`,
			ErrUnsupported)
	}
	if rf != tf || rf != trimFamilyPrism {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Split needs a solid and sheet sharing a straight-sweep generator (target %s, tool %s)`,
			ErrUnsupported, trimFamilyName(rf), trimFamilyName(tf))
	}
	rcv, ok := target.payload.(prismPayload)
	if !ok || target.Kind() != BodySolid || !target.IsSolid() {
		return prismPayload{}, prismPayload{}, fmt.Errorf(`%w: Split's target must be a solid prism`, ErrUnsupported)
	}
	if tool.Kind() != BodySheet {
		return prismPayload{}, prismPayload{}, fmt.Errorf(`%w: Split's tool must be a sheet`, ErrUnsupported)
	}
	var tl prismPayload
	switch p := tool.payload.(type) {
	case prismPayload:
		tl = p
	case chainPayload:
		tl = p.prism()
	default:
		return prismPayload{}, prismPayload{}, fmt.Errorf(`%w: Split's tool has no prism section`, ErrUnsupported)
	}
	if trimOperandSectionDelta(target) != 0 || trimOperandSectionDelta(tool) != 0 {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Split does not admit an operand carrying its own section displacement`, ErrUnsupported)
	}
	if rcv.reflected() || tl.reflected() {
		return prismPayload{}, prismPayload{}, fmt.Errorf(`%w: Split does not admit a reflected operand`, ErrUnsupported)
	}
	rcvAnalytic, err := prismProfileIsAnalytic(budget, rcv.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	tlAnalytic, err := prismProfileIsAnalytic(budget, tl.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	if !rcvAnalytic || !tlAnalytic {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: Split admits only line, circle and arc segments`, ErrUnsupported)
	}
	// S4 is the same exact stored-float comparison as Trim and prism G3.
	worldNormalRcv := rcv.xform.ApplyDir(rcv.frame.N())
	worldNormalTool := tl.xform.ApplyDir(tl.frame.N())
	if worldNormalRcv != worldNormalTool {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the target and tool do not sweep along the same generator, exactly`, ErrUnsupported)
	}
	worldOriginRcv := rcv.xform.Apply(rcv.frame.Origin())
	worldOriginTool := tl.xform.Apply(tl.frame.Origin())
	if worldOriginTool.Sub(worldOriginRcv).Dot(worldNormalRcv) != 0.0 {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the target and tool do not sweep along the same generator, exactly`, ErrUnsupported)
	}
	if !prismCutZIntervalSpans(rcv, tl) {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the tool does not span the target over the sweep parameter`, ErrUnsupported)
	}
	reexpress, err := newPrismReexpression(rcv, tl)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	if !reexpress.identity {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: the target and tool do not share one frame and placement, so their re-expression is not the identity`, ErrUnsupported)
	}
	rcvWhole, err := trimProfileFullyWhole(budget, rcv.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	tlWhole, err := trimProfileFullyWhole(budget, tl.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, err
	}
	if !rcvWhole || !tlWhole {
		return prismPayload{}, prismPayload{}, fmt.Errorf(
			`%w: every segment the target or tool consumes must span its entity's own natural domain`, ErrUnsupported)
	}
	return rcv, tl, nil
}

// resolveSplit asks sketch for the bounded cells of the private scene, keeps
// precisely the cells on the target's material side, and authenticates each
// selected cell through RecordProfile before rebuilding it as a prism.
func resolveSplit(ctx context.Context, budget *workBudget, target, tool prismPayload) ([]prismPayload, error) {
	segments, withinCap, err := prismSceneWithinWorkCap(budget, target, tool)
	if err != nil {
		return nil, err
	}
	if !withinCap {
		return nil, fmt.Errorf(`%w: the split scene charges %d arranger segments against the cap of %d`,
			ErrUnsupported, segments, prismMaxArrangementSegments)
	}
	reexpress, err := newPrismReexpression(target, tool)
	if err != nil {
		return nil, err
	}
	s, tags, _, err := buildPrismScene(budget, target, tool, reexpress)
	if err != nil {
		return nil, err
	}
	if err := budget.err(); err != nil {
		return nil, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, err
	}
	if err := budget.err(); err != nil {
		return nil, err
	}
	if len(profiles) == 0 {
		return nil, fmt.Errorf(`%w: the split arrangement holds no bounded cell`, ErrUnsupported)
	}
	unchanged, err := splitUnchangedTargetCell(budget, tags, profiles, target.profile)
	if err != nil {
		return nil, err
	}
	if unchanged {
		return nil, fmt.Errorf(`%w: the tool separates no part of the target`, ErrDegenerate)
	}
	matterTarget, err := classifySplitTargetCells(budget, tags, profiles)
	if err != nil {
		return nil, err
	}
	selected, err := selectPrismCells(budget, profiles, matterTarget, make([]bool, len(profiles)),
		func(a, _ bool) bool { return a })
	if err != nil {
		return nil, err
	}
	if len(selected) < 2 {
		return nil, fmt.Errorf(`%w: the tool separates no part of the target`, ErrDegenerate)
	}
	result := make([]prismPayload, len(selected))
	for i, cell := range selected {
		if err := budget.step(); err != nil {
			return nil, err
		}
		record, err := prismRecordProfileContext(ctx, s, cell)
		if err != nil {
			return nil, err
		}
		cutDelta := 0.0
		for _, loop := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
			for _, edge := range loop {
				if err := budget.step(); err != nil {
					return nil, err
				}
				seg, err := recordEdge(edge)
				if err != nil {
					return nil, err
				}
				delta, err := prismUnionCutDelta(edge, seg)
				if err != nil {
					return nil, err
				}
				cutDelta = math.Max(cutDelta, delta)
			}
		}
		result[i] = prismPayload{
			profile: record, frame: target.frame, xform: target.xform,
			z0: target.z0, z1: target.z1,
			z0Delta: target.z0Delta, z1Delta: target.z1Delta,
			sectionDelta: cutDelta,
		}
	}
	return result, nil
}

// splitUnchangedTargetCell finds the target's original loops reproduced in
// one sketch cell. An inside stub or an outside tool leaves this exact cell;
// neither creates a piece boundary. The match reads only entity identity and
// whole-edge flags from sketch's publication.
func splitUnchangedTargetCell(budget *workBudget, tags map[sketch.Entity]prismEntityOrigin,
	profiles []*sketch.Profile, target ProfileRecord) (bool, error) {
	outer, err := prismLoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return false, err
	}
	holes := make([]map[sketch.Entity]struct{}, len(target.Holes))
	for i := range holes {
		holes[i], err = prismLoopEntitySet(budget, tags, false, i)
		if err != nil {
			return false, err
		}
	}
	match, found, err := prismFindLoopMatch(budget, profiles, outer, holes)
	if err != nil || !found {
		return false, err
	}
	if !match.Valid {
		return false, fmt.Errorf(`%w: the unchanged target cell is invalid`, ErrUnsupported)
	}
	return true, nil
}

// classifySplitTargetCells uses classifyPrismCells's orientation and
// propagation rule for the target alone. A sheet tool has no material side;
// its edges only divide cells. In particular a circular tool may leave a
// holed target cell, so every published loop participates in the link map.
func classifySplitTargetCells(budget *workBudget, tags map[sketch.Entity]prismEntityOrigin, profiles []*sketch.Profile) ([]bool, error) {
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	type occurrence struct {
		cell int
		isB  bool
	}
	member := make([]prismCellMembership, len(profiles))
	occ := map[edgeKey][]occurrence{}
	for i, p := range profiles {
		if err := budget.step(); err != nil {
			return nil, err
		}
		if !p.Valid {
			return nil, fmt.Errorf(`%w: a cell the split resolution depends on is invalid`, ErrUnsupported)
		}
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, edge := range loop {
				if err := budget.step(); err != nil {
					return nil, err
				}
				origin, ok := tags[edge.Entity]
				if !ok {
					return nil, fmt.Errorf(`%w: a split cell edge traces to neither operand`, ErrUnsupported)
				}
				if !origin.isB {
					match := edge.Reversed == origin.authoredReversed
					if member[i].known && member[i].val != match {
						return nil, fmt.Errorf(`%w: a split cell has conflicting target-side labels`, ErrUnsupported)
					}
					member[i] = prismCellMembership{known: true, val: match}
				}
				key := edgeKey{entity: edge.Entity, t0: edge.TStart, t1: edge.TEnd}
				occ[key] = append(occ[key], occurrence{cell: i, isB: origin.isB})
			}
		}
	}
	var links []prismCellLink
	for _, uses := range occ {
		if err := budget.step(); err != nil {
			return nil, err
		}
		switch len(uses) {
		case 1:
		case 2:
			if uses[0].cell == uses[1].cell || uses[0].isB != uses[1].isB {
				return nil, fmt.Errorf(`%w: a split edge has inconsistent cell ownership`, ErrUnsupported)
			}
			links = append(links, prismCellLink{a: uses[0].cell, b: uses[1].cell, isB: uses[0].isB})
		default:
			return nil, fmt.Errorf(`%w: a split edge belongs to more than two cells`, ErrUnsupported)
		}
	}
	if err := propagatePrismMembership(budget, links, true, member); err != nil {
		return nil, err
	}
	for _, link := range links {
		if link.isB && member[link.a].known && member[link.b].known && member[link.a].val != member[link.b].val {
			return nil, fmt.Errorf(`%w: adjacent split cells disagree on the target side of a tool edge`, ErrUnsupported)
		}
	}
	matter := make([]bool, len(profiles))
	for i, m := range member {
		if !m.known {
			return nil, fmt.Errorf(`%w: a split cell has no target-side label`, ErrUnsupported)
		}
		matter[i] = m.val
	}
	return matter, nil
}

// This section is the revolve family's arm of the gate and of Trim
// (docs/surface-intersection-design.md §11's PR4). Only S1's routing, S4's
// generator test and S6's span relation differ from the prism arm; §3's whole
// resolution — buildPrismScene, classifyPrismCells, the survivor chaining and
// the cut-displacement reading — is consumed VERBATIM over the two operands'
// MERIDIAN views, which is what §3.1 claims and what revolvePayload.meridian
// supplies.

// trimRevolve is Trim over a pair S1 has already routed to the revolve family.
// The receiver's section must CLOSE (§2.2), exactly as the prism arm's does and
// for the identical reason: §3.2's side reading needs every fragment to bound a
// cell of the receiver's own interior, and sketch prunes a ribbon's dangling
// free-end fragments before publishing its regions, so an inside stub and an
// outside stub arrive identically (§4's own row, RS3).
func (b *Body) trimRevolve(ctx context.Context, budget *workBudget, tool *Body, keepInside bool) (*Body, error) {
	d := b.doc
	rcv, rcvView, tlView, err := admitTrimRevolvePair(ctx, budget, b, tool)
	if err != nil {
		return nil, err
	}
	chains, sectionDelta, err := resolveTrim(ctx, budget, rcvView, tlView, keepInside)
	if err != nil {
		return nil, err
	}
	pp := chainRevolvePayload{
		chains: chains,
		frame:  rcv.frame,
		ax:     rcv.ax,
		phi0:   rcv.phi0,
		phi1:   rcv.phi1,
		full:   rcv.full,
		den:    rcv.den,
		xform:  rcv.xform,
		// §7.1's fold reads this field as its gate, and resolveTrim's own
		// δ_cut is the whole of it: S4 and S7 zero every other term prism §7
		// derives.
		sectionDelta: sectionDelta,
	}
	body, err := evalChainRevolveContext(ctx, d, d.nextProducerID(), pp, newFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b, tool)
	return body, nil
}

// admitTrimRevolvePair runs S2-S7 for a Trim over the revolve family; S1 has
// already routed the pair here. It returns the receiver's own revolve record —
// what the result is rebuilt on — beside the two MERIDIAN views §3's
// resolution consumes.
func admitTrimRevolvePair(ctx context.Context, budget *workBudget, receiver, tool *Body) (revolvePayload, prismPayload, prismPayload, error) {
	fail := func(format string, args ...any) (revolvePayload, prismPayload, prismPayload, error) {
		return revolvePayload{}, prismPayload{}, prismPayload{}, fmt.Errorf(format, args...)
	}

	// S7's section-displacement clause runs first and reads generically off
	// EITHER revolve payload shape, so a body this design already trimmed names
	// ITS OWN displacement rather than the open-walk shape every
	// chainRevolvePayload receiver carries regardless.
	rcvDelta, tlDelta := trimOperandSectionDelta(receiver), trimOperandSectionDelta(tool)
	if rcvDelta != 0 || tlDelta != 0 {
		return fail(`%w: Trim does not admit an operand carrying its own section displacement (receiver %v mm, tool %v mm)`,
			ErrUnsupported, rcvDelta, tlDelta)
	}

	rcv, rcvOk := receiver.payload.(revolvePayload)
	if !rcvOk {
		return fail(`%w: Trim's receiver meridian is an open walk, which closes onto nothing for the tool to trim`, ErrUnsupported)
	}
	tl, tlOk := tool.payload.(revolvePayload)
	if !tlOk {
		return fail(`%w: Trim's tool meridian is an open walk, which bounds no region to keep a side of`, ErrUnsupported)
	}

	rcvView, tlView := rcv.meridian(), tl.meridian()

	// S2: neither operand's accumulated placement is a reflection.
	if rcvView.reflected() || tlView.reflected() {
		return fail(`%w: Trim does not admit a reflected operand`, ErrUnsupported)
	}

	// S3: every segment of both meridians is a LineSeg, CircleSeg or ArcSeg.
	rcvAnalytic, err := prismProfileIsAnalytic(budget, rcv.profile)
	if err != nil {
		return revolvePayload{}, prismPayload{}, prismPayload{}, err
	}
	tlAnalytic, err := prismProfileIsAnalytic(budget, tl.profile)
	if err != nil {
		return revolvePayload{}, prismPayload{}, prismPayload{}, err
	}
	if !rcvAnalytic || !tlAnalytic {
		return fail(`%w: Trim admits only line, circle and arc segments; a free-form segment blinds sketch's whole-scene TExact gate`, ErrUnsupported)
	}

	// S4's revolve arm. The plane frames and the RESOLVED AXIS must agree
	// component-wise on the stored floats, and both meridians must stand clear
	// of that axis.
	if rcv.frame != tl.frame {
		return fail(`%w: the receiver and tool do not spin about the same frame, exactly`, ErrUnsupported)
	}
	if !revolveAxisIdentical(rcv.ax, tl.ax) {
		return fail(`%w: the receiver and tool do not spin about the same axis, exactly`, ErrUnsupported)
	}
	rcvClear, err := revolveMeridianClearOfAxis(ctx, rcv)
	if err != nil {
		return revolvePayload{}, prismPayload{}, prismPayload{}, err
	}
	tlClear, err := revolveMeridianClearOfAxis(ctx, tl)
	if err != nil {
		return revolvePayload{}, prismPayload{}, prismPayload{}, err
	}
	if !rcvClear || !tlClear {
		return fail(`%w: Trim needs both meridians clear of the revolve axis; one of them touches it, and the fragment a cut would leave there sweeps a pole this arm does not yet place`, ErrUnsupported)
	}

	// S5: for Trim, the tool's meridian is one closed hole-free loop.
	if len(tl.profile.Holes) != 0 {
		return fail(`%w: Trim's tool meridian carries a hole, whose interior this evaluator cannot yet distinguish as a side`, ErrUnsupported)
	}

	// S6's revolve arm: the same span relation the prism arm reads over its
	// sweep levels, taken over the angular interval instead. S4 has already
	// proven the two axes identical, so the prism arm's origin shift has no
	// counterpart here and the comparison is on the stored endpoint floats
	// alone.
	if tl.phi0 > rcv.phi0 || tl.phi1 < rcv.phi1 {
		return fail(`%w: the tool's angular span [%v, %v] does not cover the receiver's [%v, %v]`,
			ErrUnsupported, tl.phi0, tl.phi1, rcv.phi0, rcv.phi1)
	}

	// S7's remaining two clauses.
	reexpress, err := newPrismReexpression(rcvView, tlView)
	if err != nil {
		return revolvePayload{}, prismPayload{}, prismPayload{}, err
	}
	if !reexpress.identity {
		return fail(`%w: the receiver and tool do not share one frame and placement, so their re-expression is not the identity`, ErrUnsupported)
	}
	rcvWhole, err := trimProfileFullyWhole(budget, rcv.profile)
	if err != nil {
		return revolvePayload{}, prismPayload{}, prismPayload{}, err
	}
	tlWhole, err := trimProfileFullyWhole(budget, tl.profile)
	if err != nil {
		return revolvePayload{}, prismPayload{}, prismPayload{}, err
	}
	if !rcvWhole || !tlWhole {
		return fail(`%w: every segment the receiver or tool consumes must span its entity's own natural domain`, ErrUnsupported)
	}

	return rcv, rcvView, tlView, nil
}

// revolveAxisIdentical is S4's revolve comparison: the resolved axis's own
// anchor, direction and each of those four fields' proven bound, under Go ==
// on the stored floats and never a tolerance
// (docs/surface-intersection-design.md §2.1 S4).
//
// It compares EIGHT fields and not the whole axisFrame. The remaining four —
// snapTol, radialAdmitAllow, axialExtentUpper and snap — are each operand's own
// admission allowances, derived from ITS OWN section rather than from the
// generator the two share, so two genuinely co-axial operands differ in them by
// construction and comparing them would refuse every admissible pair.
func revolveAxisIdentical(a, b axisFrame) bool {
	return a.aU == b.aU && a.aV == b.aV &&
		a.aUBound == b.aUBound && a.aVBound == b.aVBound &&
		a.dU == b.dU && a.dV == b.dV &&
		a.dUBound == b.dUBound && a.dVBound == b.dVBound
}

// revolveMeridianClearOfAxis reads S4's clearance clause off the axis snap that
// has ALREADY run: axisFrame.walk assigns a walk endpoint ρ exactly zero when
// the resolved axis's own snap tolerance covers it, so "clear" is the published
// structural fact that no endpoint carries that zero, never a fresh measurement
// decad takes on the geometry it was handed. A meridian touching the axis
// sweeps a pole, and a cut fragment ending at one would need pole topology this
// arm does not place.
func revolveMeridianClearOfAxis(ctx context.Context, rp revolvePayload) (bool, error) {
	work := newFreeformWork()
	for _, loop := range append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...) {
		resolved, err := revolveLoopWalks(ctx, rp, loop, work, "the trim axis-clearance gate")
		if err != nil {
			return false, err
		}
		for _, w := range resolved.walks {
			if w.startV == 0 || w.endV == 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// revolveChainClearOfAxis is revolveMeridianClearOfAxis over an OPEN meridian
// walk set, read through the chain resolver rather than the loop one: an open
// walk's last segment does not continue into its first, so coalescing it as a
// loop could merge the two free ends into one walk and hide an endpoint the
// clearance clause has to see.
func revolveChainClearOfAxis(ctx context.Context, rp chainRevolvePayload) (bool, error) {
	work := newFreeformWork()
	for ci := range rp.chains {
		resolved, err := chainRevolveWalks(ctx, rp.walkView(ci), rp.chains[ci], work)
		if err != nil {
			return false, err
		}
		for _, w := range resolved.walks {
			if w.startV == 0 || w.endV == 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// extendRevolve is Extend over a pair S1 has already routed to the revolve
// family. Everything past the gate is the prism arm's own: resolveExtend reads
// sketch's cut parameter off the recreated carrier in the two operands'
// MERIDIAN views (§3.1), and the widened range goes back into the receiver's
// own record. Only the gate and the free-edge reading differ.
func (b *Body) extendRevolve(ctx context.Context, budget *workBudget, selected []*Edge, tool *Body) (*Body, error) {
	d := b.doc
	rcv, rcvView, tlView, err := admitExtendRevolvePair(ctx, budget, b, tool)
	if err != nil {
		return nil, err
	}
	chains := make([]ChainRecord, len(rcv.chains))
	for i, chain := range rcv.chains {
		chains[i] = ChainRecord{Segments: append([]CurveSegment(nil), chain.Segments...)}
	}
	cutDelta := 0.0
	for _, edge := range selected {
		ci, si, atStart, err := extendRevolveEndSegment(b, rcv, edge)
		if err != nil {
			return nil, err
		}
		widened, delta, err := resolveExtend(ctx, budget, rcvView, tlView, chains[ci].Segments[si], atStart)
		if err != nil {
			return nil, err
		}
		chains[ci].Segments[si] = widened
		cutDelta = math.Max(cutDelta, delta)
	}
	rcv.chains = chains
	// §7.1's fold reads this field as its gate, and resolveExtend's own δ_cut is
	// the whole of it: S4 and S7 zero every other term prism §7 derives.
	rcv.sectionDelta = cutDelta
	result, err := evalChainRevolveContext(ctx, d, d.nextProducerID(), rcv, newFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(result, b, tool)
	return result, nil
}

// admitExtendRevolvePair runs S2-S4, S6 and S7 for an Extend over the revolve
// family; S1 has already routed the pair here. S5 does not reach an Extend,
// which reads no side. It returns the receiver's own chain record — what the
// result is rebuilt on — beside the two MERIDIAN views §3's resolution
// consumes.
func admitExtendRevolvePair(ctx context.Context, budget *workBudget, receiver, tool *Body) (chainRevolvePayload, prismPayload, prismPayload, error) {
	fail := func(format string, args ...any) (chainRevolvePayload, prismPayload, prismPayload, error) {
		return chainRevolvePayload{}, prismPayload{}, prismPayload{}, fmt.Errorf(format, args...)
	}
	pass := func(err error) (chainRevolvePayload, prismPayload, prismPayload, error) {
		return chainRevolvePayload{}, prismPayload{}, prismPayload{}, err
	}

	// S7's section-displacement clause runs first and reads generically off
	// either revolve payload shape, so a body this design already trimmed names
	// ITS OWN displacement rather than the shape of its section.
	rcvDelta, tlDelta := trimOperandSectionDelta(receiver), trimOperandSectionDelta(tool)
	if rcvDelta != 0 || tlDelta != 0 {
		return fail(`%w: Extend does not admit an operand carrying its own section displacement (receiver %v mm, tool %v mm)`,
			ErrUnsupported, rcvDelta, tlDelta)
	}

	rcv, rcvOk := receiver.payload.(chainRevolvePayload)
	if !rcvOk || receiver.Kind() != BodySheet {
		return fail(`%w: Extend's receiver needs an open meridian to lengthen; a closed one has no free end and its other free direction is its own sweep`, ErrUnsupported)
	}
	// The FULL-revolution restriction, and it is a reading rather than a bound:
	// extendRevolveEndSegment below names a free end by the single-coedge
	// latitude loop fullRevLoops (revolve_build.go) puts on the first and last
	// wall. A PARTIAL revolution carries no such loop — each wall holds one loop
	// gathering its two cap edges beside the swept arcs, and which member of it
	// is the meridian's own free end is not a recorded fact — so the pair is
	// refused rather than resolved by position.
	if !rcv.full {
		return fail(`%w: Extend over the revolve family admits a full revolution alone; a partially revolved ribbon's swept free edge shares one loop with its wall's two cap edges, and no recorded fact separates them`, ErrUnsupported)
	}
	var tl revolvePayload
	switch p := tool.payload.(type) {
	case revolvePayload:
		tl = p
	case chainRevolvePayload:
		tl = p.revolve()
	default:
		return fail(`%w: Extend's tool has no revolve meridian`, ErrUnsupported)
	}

	rcvRev := rcv.revolve()
	rcvView, tlView := rcvRev.meridian(), tl.meridian()

	// S2: neither operand's accumulated placement is a reflection.
	if rcvView.reflected() || tlView.reflected() {
		return fail(`%w: Extend does not admit a reflected operand`, ErrUnsupported)
	}

	// S3: every segment of both meridians is a LineSeg, CircleSeg or ArcSeg.
	rcvAnalytic, err := prismProfileIsAnalytic(budget, rcvRev.profile)
	if err != nil {
		return pass(err)
	}
	tlAnalytic, err := prismProfileIsAnalytic(budget, tl.profile)
	if err != nil {
		return pass(err)
	}
	if !rcvAnalytic || !tlAnalytic {
		return fail(`%w: Extend admits only line, circle and arc segments; a free-form segment blinds sketch's whole-scene TExact gate`, ErrUnsupported)
	}

	// S4's revolve arm, the identical eight-field comparison Trim's own takes.
	if rcv.frame != tl.frame {
		return fail(`%w: the receiver and tool do not spin about the same frame, exactly`, ErrUnsupported)
	}
	if !revolveAxisIdentical(rcv.ax, tl.ax) {
		return fail(`%w: the receiver and tool do not spin about the same axis, exactly`, ErrUnsupported)
	}
	rcvClear, err := revolveChainClearOfAxis(ctx, rcv)
	if err != nil {
		return pass(err)
	}
	tlClear, err := revolveMeridianClearOfAxis(ctx, tl)
	if err != nil {
		return pass(err)
	}
	if !rcvClear || !tlClear {
		return fail(`%w: Extend needs both meridians clear of the revolve axis; one of them touches it, and the end a lengthening would place there sweeps a pole this arm does not build`, ErrUnsupported)
	}

	// S6's revolve arm: S4 has proven the two axes identical, so the prism
	// arm's origin shift has no counterpart and the comparison is on the
	// stored endpoint floats alone.
	if tl.phi0 > rcv.phi0 || tl.phi1 < rcv.phi1 {
		return fail(`%w: the tool's angular span [%v, %v] does not cover the receiver's [%v, %v]`,
			ErrUnsupported, tl.phi0, tl.phi1, rcv.phi0, rcv.phi1)
	}

	// S7's remaining two clauses. Only the TOOL is required whole: §3.1 puts
	// the receiver's extended segment into the scene over its own full domain
	// and leaves every other receiver segment outside it, so a receiver
	// carrying narrowed segments elsewhere clears S7 for an Extend and refuses
	// it for a Trim.
	reexpress, err := newPrismReexpression(rcvView, tlView)
	if err != nil {
		return pass(err)
	}
	if !reexpress.identity {
		return fail(`%w: the receiver and tool do not share one frame and placement, so their re-expression is not the identity`, ErrUnsupported)
	}
	tlWhole, err := trimProfileFullyWhole(budget, tl.profile)
	if err != nil {
		return pass(err)
	}
	if !tlWhole {
		return fail(`%w: every segment the tool consumes must span its entity's own natural domain`, ErrUnsupported)
	}

	return rcv, rcvView, tlView, nil
}

// extendRevolveEndSegment maps a selected topology edge to the recorded
// meridian end it names — extendEndSegment's own reading, re-derived for the
// chain-revolve build, whose free ends are SWEPT latitude circles rather than
// straight sweep edges.
//
// Under a full revolution buildChainRevolveWalls gives every wall face its
// junctions' latitude circles as SEPARATE single-coedge loops through
// fullRevLoops (revolve_build.go), which sets forward true for the wall's own
// START junction and false for its END. That flag is the map: the receiver's
// two free ends are junction 0 of the FIRST wall and junction n of the LAST,
// and reading the flag rather than the loop position is what makes the reading
// survive fullRevLoops' own outer-loop swap. The first and last wall are never
// the skipped wallAxis kind — a segment lying on the axis puts both its ends at
// ρ == 0, which requireChainAxisIncidence already refuses at a chain's free end
// — so faces[0] and faces[len-1] are walk 0 and walk n-1.
//
// The loop indexes b.lumps and reads pp.chains at the SAME index, on
// extendEndSegment's own terms: evalChainRevolveContext appends each chain's
// faces in chain order and sheetLumps preserves first-appearance order, so the
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
	for ci, lump := range b.lumps {
		if ci >= len(pp.chains) || len(lump.shells) != 1 || len(lump.shells[0].faces) == 0 {
			continue
		}
		faces := lump.shells[0].faces
		if named, ok := revolveLatitudeLoopEdge(faces[0], true); ok && named == edge {
			return ci, 0, true, nil
		}
		if named, ok := revolveLatitudeLoopEdge(faces[len(faces)-1], false); ok && named == edge {
			return ci, len(pp.chains[ci].Segments) - 1, false, nil
		}
	}
	return miss()
}

// revolveLatitudeLoopEdge reads one wall face's own latitude loop for the
// junction fullRevLoops' forward flag names: true for the wall's start
// junction, false for its end. A junction ON the axis mints no latitude circle
// at all, so that end answers absent and the caller refuses.
func revolveLatitudeLoopEdge(f *Face, start bool) (*Edge, bool) {
	for _, l := range f.loops {
		if len(l.coedges) != 1 || l.coedges[0].forward != start {
			continue
		}
		return l.coedges[0].edge, true
	}
	return nil, false
}
