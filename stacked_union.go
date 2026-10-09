package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/sketch"
)

// This file is docs/general-boolean-design.md §3's class A1: a Union of two
// co-directional prisms (or stacked prisms) whose sweep intervals overlap or
// touch without being equal. The distinct levels of both operands, compared
// exactly, cut the union into slabs. A slab one operand reaches takes that
// operand's region verbatim. A slab both reach takes the select-all merge of
// prism-boolean §4.2, or the containing operand's region verbatim when the
// clean-nesting match proves one region holds the other's whole. Each
// interface where two adjacent regions differ is decided by the same match:
// the narrower region's outer must come back whole as a hole of the wider
// region's cell, and the exposed record is the wider region with that hole.
//
// Admission is reject-only and silent up to the first private scene: G1-G4
// on the two operands' outer prisms, G4 and G6 (hole-free) on every slab
// region, prism-boolean's trimmed-circular refusal, and the interval relation
// `z0_b' <= z1_a && z0_a <= z1_b'` over big.Rat. Past that point an
// unresolved topology still falls back to the mesh path, as prism-boolean
// §4.4 states: a disjoint overlap, and a cell the match cannot find. An
// interface the clean-nesting match leaves unresolved — footprints that
// cross or share a wall — hands the same slabs to stacked_union_brep.go,
// which classifies that interface's cells and states the result as a
// brepPayload; a pair that build does not cover takes the mesh path.

// stackedUnionOperand is one operand of the stacked union read as slabs: a
// prism is one slab over its own interval; a stacked prism is its own slabs.
// proxy carries the frame, placement and section displacement the gates read.
type stackedUnionOperand struct {
	proxy prismPayload
	slabs []stackedrecord.Slab
	// brep marks an A1 result read through its stack: its slabs may hold
	// several regions, and only the brep build arranges them.
	brep bool
}

func stackedUnionOperandOf(b *Body) (stackedUnionOperand, bool) {
	switch p := b.payload.(type) {
	case prismPayload:
		return stackedUnionOperand{proxy: p, slabs: []stackedrecord.Slab{{
			Regions: []profileRecord{p.profile},
			Z0:      p.z0, Z1: p.z1, Z0Delta: p.z0Delta, Z1Delta: p.z1Delta,
		}}}, true
	case stackedPrismPayload:
		return stackedUnionOperand{proxy: p.outerPrism(), slabs: p.slabs}, true
	case brepPayload:
		// An A1 result keeps the slabs it was built from; a class-B result
		// keeps none and is no operand here.
		if p.stack == nil || len(p.stack.slabs) == 0 {
			return stackedUnionOperand{}, false
		}
		first, last := p.stack.slabs[0], p.stack.slabs[len(p.stack.slabs)-1]
		return stackedUnionOperand{proxy: prismPayload{
			profile: profileRecord{Outer: first.Regions[0].Outer},
			frame:   p.faces[0].frame, xform: p.xform, sectionDelta: p.stack.delta,
			z0: first.Z0, z0Delta: first.Z0Delta, z1: last.Z1, z1Delta: last.Z1Delta,
		}, slabs: p.stack.slabs, brep: true}, true
	default:
		return stackedUnionOperand{}, false
	}
}

// view is the prismPayload a private scene reads for one of this operand's
// regions: the region, the operand's frame and placement, and its section
// displacement for prism-boolean §3.4's crossing charge.
func (o stackedUnionOperand) view(region profileRecord) prismPayload {
	v := o.proxy
	v.profile = region
	return v
}

// tryStackedUnion is general-boolean §3 A1. ok=false with a nil error is a
// silent miss: the caller takes the mesh path. A non-nil error is a genuine
// refusal the caller propagates (prism-boolean §3.4). The payload is a
// stackedPrismPayload when every interface resolves through the clean-nesting
// match, and a brepPayload (stacked_union_brep.go) when one does not.
func tryStackedUnion(ctx context.Context, a, b *Body) (featurePayload, bool, error) {
	va, aok := stackedUnionOperandOf(a)
	vb, bok := stackedUnionOperandOf(b)
	if !aok || !bok {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	// G1-G4 on the outer prisms; G4, G6 and the trimmed-circular refusal on
	// every region, since a stacked operand's proxy names its first slab only.
	if _, _, ok, err := admitPrismPairBudget(budget, &Body{payload: va.proxy}, &Body{payload: vb.proxy}); err != nil || !ok {
		return nil, false, err
	}
	// A brep operand, or a slab holding several regions, is the brep build's
	// alone: the stacked record states one region per slab.
	brepOnly := va.brep || vb.brep
	for _, op := range []stackedUnionOperand{va, vb} {
		for _, slab := range op.slabs {
			if len(slab.Regions) == 0 {
				return nil, false, nil
			}
			if len(slab.Regions) != 1 {
				brepOnly = true
			}
			for _, region := range slab.Regions {
				if len(region.Holes) != 0 { // G6
					return nil, false, nil
				}
				analytic, err := prismcells.ProfileAnalytic(budget, region) // G4
				if err != nil || !analytic {
					return nil, false, err
				}
				trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, region.Outer, region.Holes)
				if err != nil || trimmed {
					return nil, false, err
				}
			}
		}
	}
	shift := prismplacement.ZShift(prismPlacementOf(va.proxy), prismPlacementOf(vb.proxy))
	levels, ok := stackedrecord.UnionLevels(va.slabs, vb.slabs, shift)
	if !ok || len(levels) < 3 {
		// Equal intervals are prism-boolean §3.2's Union row, not a stack.
		return nil, false, nil
	}
	a0 := proofarith.FloatRat(va.slabs[0].Z0)
	a1 := proofarith.FloatRat(va.slabs[len(va.slabs)-1].Z1)
	b0 := proofarith.FloatRat(vb.slabs[0].Z0)
	b1 := proofarith.FloatRat(vb.slabs[len(vb.slabs)-1].Z1)
	b0.Add(b0, shift)
	b1.Add(b1, shift)
	if b0.Cmp(a1) > 0 || a0.Cmp(b1) > 0 { // G5 for a stacked union: the intervals overlap or touch
		return nil, false, nil
	}
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(va.proxy), prismPlacementOf(vb.proxy))
	if err != nil {
		return nil, false, err
	}

	st := &stackedUnionState{budget: budget, va: va, vb: vb, reexpress: reexpress,
		bRecorded: map[int]profileRecord{}, scenes: map[[2]stackedUnionRegionRef]*stackedNesting{}}
	sp := stackedPrismPayload{frame: va.proxy.frame, xform: va.proxy.xform}
	zero := new(big.Rat)
	var reach [][2]int
	for k := 0; k+1 < len(levels); k++ {
		lo, hi := levels[k], levels[k+1]
		ia := stackedrecord.UnionSlabOf(va.slabs, zero, lo.Exact, hi.Exact)
		ib := stackedrecord.UnionSlabOf(vb.slabs, shift, lo.Exact, hi.Exact)
		if ia < 0 && ib < 0 {
			return nil, false, nil
		}
		reach = append(reach, [2]int{ia, ib})
	}
	if brepOnly {
		return st.brep(ctx, levels, reach)
	}
	for k := 0; k+1 < len(levels); k++ {
		lo, hi := levels[k], levels[k+1]
		region, ok, err := st.slabRegion(ctx, reach[k][0], reach[k][1])
		if err != nil || !ok {
			return nil, false, err
		}
		sp.slabs = append(sp.slabs, stackedrecord.Slab{Regions: []profileRecord{region},
			Z0: lo.Held, Z1: hi.Held, Z0Delta: lo.Delta, Z1Delta: hi.Delta})
	}
	for k := 0; k+1 < len(sp.slabs); k++ {
		boundary, ok, err := st.interfaceOf(ctx, sp.slabs[k].Regions[0], sp.slabs[k+1].Regions[0])
		if err != nil {
			return nil, false, err
		}
		if !ok {
			// A1's flush or crossing interface: the slabs are right and the
			// stacked record cannot state the result, so the brep build
			// classifies every interface's cells instead.
			return st.brep(ctx, levels, reach)
		}
		sp.interfaces = append(sp.interfaces, boundary)
	}
	sp.sectionDelta = st.sectionDelta()
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.slabs, Interfaces: sp.interfaces}); err != nil {
		return nil, false, err
	}
	return sp, true, nil
}

// sectionDelta is §7's formula, with each term the largest over every scene
// this union arranged: A's walk charge beside A's displacement, B's walk
// charge and re-expression beside B's, the interface scenes' walk charge on
// regions already in the result's frame, every merge's crossing charge (A6),
// and every merge's cut charge on top.
func (st *stackedUnionState) sectionDelta() float64 {
	return proofbound.AbsSumUpper(
		max(
			proofbound.AbsSumUpper(st.va.proxy.sectionDelta, st.walkA),
			proofbound.AbsSumUpper(st.vb.proxy.sectionDelta, st.walkB, st.reexpress.Delta),
			st.walkInterface,
			st.crossing,
		),
		st.cutDelta,
	)
}

// stackedUnionRegionRef names one operand slab region: operand B's slab ib,
// or operand A's slab ia.
type stackedUnionRegionRef struct {
	isB  bool
	slab int
}

// stackedUnionState is the per-call state one tryStackedUnion threads through
// its scenes: the charges each scene adds, B's regions already recorded in
// A's frame, and every scene of two operand regions arranged so far, keyed by
// the two regions, so a slab merge and the interfaces that read the same two
// regions share one arrangement and its vertices.
type stackedUnionState struct {
	budget        *proofbound.WorkBudget
	va, vb        stackedUnionOperand
	reexpress     *prismReexpression
	bRecorded     map[int]profileRecord
	scenes        map[[2]stackedUnionRegionRef]*stackedNesting
	walkA, walkB  float64
	walkInterface float64
	cutDelta      float64
	crossing      float64
}

// view is the prism a private scene reads for a reference: the region under
// its operand's frame, placement and section displacement.
func (st *stackedUnionState) view(ref stackedUnionRegionRef) prismPayload {
	if ref.isB {
		return st.vb.view(st.vb.slabs[ref.slab].Regions[0])
	}
	return st.va.view(st.va.slabs[ref.slab].Regions[0])
}

// scene arranges two operand regions once and keeps the arrangement: the
// clean-nesting answer, the cells and their tags, and the scene's walk
// charge. The region named first enters as operand A of the scene.
func (st *stackedUnionState) scene(ctx context.Context, x, y stackedUnionRegionRef) (*stackedNesting, error) {
	key := [2]stackedUnionRegionRef{x, y}
	if m, ok := st.scenes[key]; ok {
		return m, nil
	}
	px, py := st.view(x), st.view(y)
	if err := st.withinCap(px, py); err != nil {
		return nil, err
	}
	reexpress := st.reexpress
	if x.isB == y.isB {
		// Two regions of one operand share a frame: the brep build, the only
		// caller that arranges such a pair, admits the identity
		// re-expression alone, so B's regions are already in A's frame.
		reexpress = &prismReexpression{Identity: true}
	}
	m, err := stackedNestingOf(ctx, st.budget, px, py, reexpress)
	if err != nil {
		return nil, err
	}
	if x.isB {
		st.walkB = max(st.walkB, m.sceneDelta.A)
	} else {
		st.walkA = max(st.walkA, m.sceneDelta.A)
	}
	if y.isB {
		st.walkB = max(st.walkB, m.sceneDelta.B)
	} else {
		st.walkA = max(st.walkA, m.sceneDelta.B)
	}
	st.scenes[key] = &m
	return &m, nil
}

// slabRegion is one result slab's region from the operand slabs ia and ib
// (-1 when that operand does not reach it).
func (st *stackedUnionState) slabRegion(ctx context.Context, ia, ib int) (profileRecord, bool, error) {
	switch {
	case ia < 0 && ib < 0:
		return profileRecord{}, false, nil
	case ib < 0:
		return st.va.slabs[ia].Regions[0], true, nil
	case ia < 0:
		return st.bRegion(ctx, ib)
	}
	ra, rb := st.va.slabs[ia].Regions[0], st.vb.slabs[ib].Regions[0]
	if st.reexpress.Identity {
		same, err := loopRecordsEqual(st.budget, ra.Outer, rb.Outer)
		if err != nil {
			return profileRecord{}, false, err
		}
		if same {
			return ra, true, nil
		}
	}
	pa, pb := st.va.view(ra), st.vb.view(rb)
	m, err := st.scene(ctx, stackedUnionRegionRef{slab: ia}, stackedUnionRegionRef{isB: true, slab: ib})
	if err != nil {
		return profileRecord{}, false, err
	}
	switch m.nest {
	case stackedNestBInA:
		return ra, true, nil
	case stackedNestAInB:
		return st.bRegion(ctx, ib)
	}
	// Neither holds the other whole: prism-boolean §4.2's select-all merge,
	// with §3.4's crossing charge on every cut a displaced source can move
	// (docs/general-boolean-design.md §3 A6). No cell at all is a scene
	// prismcells.CellProfiles could not restate: the mesh path.
	if len(m.profiles) == 0 {
		return profileRecord{}, false, nil
	}
	if ok, err := m.charge(st.budget, pa, pb, st.reexpress); err != nil || !ok {
		return profileRecord{}, false, err
	}
	// Select-all is the union only without an enclosed void (§4.2): a void
	// is unresolved, never an error, so the pair takes the mesh path.
	voidFree, err := prismcells.CellsHaveNoVoid(st.budget, m.tags, m.profiles)
	if err != nil || !voidFree {
		return profileRecord{}, false, err
	}
	if ok, err := m.sceneDelta.SharedSpansBounded(st.budget, m.profiles); err != nil || !ok {
		return profileRecord{}, false, err
	}
	merged, cutDelta, resolved, err := prismcells.Merge(st.budget, m.profiles, "union")
	if fallBack, err := prismcells.AmplifiedFallback(m.sceneDelta.Amplified, err); fallBack || err != nil || !resolved {
		return profileRecord{}, false, err
	}
	if fallBack, err := prismcells.AmplifiedFallback(m.sceneDelta.Amplified, auditPrismMergeSection(st.budget, pa, merged)); fallBack || err != nil {
		return profileRecord{}, false, err
	}
	st.cutDelta = max(st.cutDelta, cutDelta)
	st.crossing = max(st.crossing, m.sceneDelta.Crossing)
	return merged, true, nil
}

// bRegion is operand B's slab region in A's frame. Under the identity
// re-expression that is B's own record. Otherwise B's region is arranged
// alone in a private scene of re-expressed entities and recorded from the
// one cell sketch returns, so every coordinate is a sketch-recorded edge and
// the re-expression's rounding rides in reexpress.Delta. The record is kept
// per B slab so every result slab that reads it holds one record.
func (st *stackedUnionState) bRegion(ctx context.Context, ib int) (profileRecord, bool, error) {
	region := st.vb.slabs[ib].Regions[0]
	if st.reexpress.Identity {
		return region, true, nil
	}
	if recorded, ok := st.bRecorded[ib]; ok {
		return recorded, true, nil
	}
	pb := st.vb.view(region)
	empty := st.va.view(profileRecord{})
	if err := st.withinCap(empty, pb); err != nil {
		return profileRecord{}, false, err
	}
	s, tags, sceneDelta, err := buildPrismScene(st.budget, empty, pb, st.reexpress)
	if err != nil {
		return profileRecord{}, false, err
	}
	st.walkB = max(st.walkB, sceneDelta.B)
	profiles, err := prismcells.ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return profileRecord{}, false, err
	}
	bOuter, err := prismcells.LoopEntitySet(st.budget, tags, true, -1)
	if err != nil {
		return profileRecord{}, false, err
	}
	match, resolved, err := prismcells.FindLoopMatch(st.budget, profiles, bOuter, nil)
	if err != nil || !resolved {
		return profileRecord{}, false, err
	}
	if !match.Valid {
		return profileRecord{}, false, prismcells.InvalidRegionError("union")
	}
	recorded, err := prismRecordArrangedProfileContext(ctx, match)
	if err != nil {
		return profileRecord{}, false, err
	}
	st.bRecorded[ib] = recorded
	return recorded, true, nil
}

// interfaceOf decides one interface between two result slab regions, both in
// A's frame. Equal outers expose nothing. Otherwise the clean-nesting match
// must find the narrower outer whole as a hole of the wider region's cell:
// the wider side alone holds the exposed record, the wider region with the
// narrower outer reversed as its hole. Any other outcome, a split boundary
// included, is unresolved and falls back to the mesh path.
func (st *stackedUnionState) interfaceOf(ctx context.Context, lower, upper profileRecord) (stackedrecord.Interface, bool, error) {
	same, err := loopRecordsEqual(st.budget, lower.Outer, upper.Outer)
	if err != nil {
		return stackedrecord.Interface{}, false, err
	}
	if same {
		// Hole-free regions with one outer: nothing is exposed, recorded as
		// I7's own empty derivation.
		none, err := stackedrecord.Exposed(ctx, nil)
		if err != nil {
			return stackedrecord.Interface{}, false, err
		}
		return stackedrecord.Interface{LowerExposed: none, UpperExposed: append([]profileRecord{}, none...)}, true, nil
	}
	m, err := stackedUnionInterfaceMatch(ctx, st.budget, st.va.proxy, lower, upper)
	if err != nil {
		return stackedrecord.Interface{}, false, err
	}
	st.walkInterface = max(st.walkInterface, m.sceneDelta.A, m.sceneDelta.B)
	switch m.nest {
	case stackedNestBInA:
		exposed, err := stackedrecord.UnionExposed(ctx, lower, upper)
		if err != nil {
			return stackedrecord.Interface{}, false, err
		}
		return stackedrecord.Interface{LowerExposed: exposed}, true, nil
	case stackedNestAInB:
		exposed, err := stackedrecord.UnionExposed(ctx, upper, lower)
		if err != nil {
			return stackedrecord.Interface{}, false, err
		}
		return stackedrecord.Interface{UpperExposed: exposed}, true, nil
	}
	// A crossing (m.split) or any other unmatched pair: prism-boolean §4.4's
	// unresolved topology, a silent fallback.
	return stackedrecord.Interface{}, false, nil
}

// stackedUnionInterfaceMatch arranges two adjacent result regions, both
// already in the result's frame, with the identity re-expression.
func stackedUnionInterfaceMatch(ctx context.Context, budget *proofbound.WorkBudget, base prismPayload, lower, upper profileRecord) (stackedNesting, error) {
	pl, pu := base, base
	pl.profile, pu.profile = lower, upper
	if err := stackedSceneWithinCap(budget, pl, pu); err != nil {
		return stackedNesting{}, err
	}
	return stackedNestingOf(ctx, budget, pl, pu, &prismReexpression{Identity: true})
}

func (st *stackedUnionState) withinCap(pa, pb prismPayload) error {
	return stackedSceneWithinCap(st.budget, pa, pb)
}

func stackedSceneWithinCap(budget *proofbound.WorkBudget, pa, pb prismPayload) error {
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, pa.profile, pb.profile)
	if err != nil {
		return err
	}
	if !withinCap {
		return fmt.Errorf(
			`%w: the analytic union scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1)`,
			ErrUnsupported, segments, prismcells.MaxArrangementSegments)
	}
	return nil
}

type stackedNest int

const (
	stackedNestNone stackedNest = iota
	stackedNestBInA             // B's outer is a whole hole of A's whole cell
	stackedNestAInB             // A's outer is a whole hole of B's whole cell
)

// stackedNesting is one private scene's clean-nesting answer: which operand's
// cell carries the other's outer whole as a hole, the scene's cells for a
// merge, whether any arranged edge is Partial, and the scene's walk charge.
// charged records that chargeCrossings has run over the scene, so a second
// reader of one cached scene takes the charge already in sceneDelta.
type stackedNesting struct {
	nest       stackedNest
	split      bool
	profiles   []*sketch.Profile
	tags       map[sketch.Entity]prismcells.Origin
	sceneDelta prismSceneDelta
	charged    bool
	chargeOK   bool
}

// charge runs A6's crossing charge once over the scene's cells. ok=false is
// a crossing or a span with no proven charge: the mesh path.
func (m *stackedNesting) charge(budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (bool, error) {
	if m.charged {
		return m.chargeOK, nil
	}
	ok, err := m.sceneDelta.ChargeCrossings(budget, m.tags, m.profiles, pa.sectionDelta, pb.sectionDelta, reexpress.Delta)
	if err != nil {
		return false, err
	}
	m.charged, m.chargeOK = true, ok
	return ok, nil
}

// stackedNestingOf runs prism-boolean §4.2's structural whole-loop match in
// both directions over one private scene of two hole-free regions. A matched
// proof cell that sketch reports invalid is RB1.
func stackedNestingOf(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (stackedNesting, error) {
	s, tags, sceneDelta, err := buildPrismScene(budget, pa, pb, reexpress)
	if err != nil {
		return stackedNesting{}, err
	}
	if err := budget.Err(); err != nil {
		return stackedNesting{}, err
	}
	profiles, err := prismcells.CellProfiles(ctx, budget, s)
	if err != nil {
		return stackedNesting{}, err
	}
	out := stackedNesting{profiles: profiles, tags: tags, sceneDelta: sceneDelta}
	aOuter, err := prismcells.LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return stackedNesting{}, err
	}
	bOuter, err := prismcells.LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return stackedNesting{}, err
	}
	for _, dir := range []struct {
		outer, hole map[sketch.Entity]struct{}
		nest        stackedNest
	}{{aOuter, bOuter, stackedNestBInA}, {bOuter, aOuter, stackedNestAInB}} {
		proof, matched, err := prismcells.FindLoopMatch(budget, profiles, dir.outer, []map[sketch.Entity]struct{}{dir.hole})
		if err != nil {
			return stackedNesting{}, err
		}
		if !matched {
			continue
		}
		if !proof.Valid {
			return stackedNesting{}, prismcells.InvalidRegionError("union")
		}
		out.nest = dir.nest
		return out, nil
	}
	out.split, err = prismcells.HasSplitBoundary(budget, profiles)
	if err != nil {
		return stackedNesting{}, err
	}
	return out, nil
}
