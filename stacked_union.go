package decad

import (
	"context"
	"fmt"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/prismcells"
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
	slabs []prismSlab
	// brep marks an A1 result read through its stack: its slabs may hold
	// several regions, and only the brep build arranges them.
	brep bool
}

func stackedUnionOperandOf(b *Body) (stackedUnionOperand, bool) {
	switch p := b.payload.(type) {
	case prismPayload:
		return stackedUnionOperand{proxy: p, slabs: []prismSlab{{
			regions: []ProfileRecord{p.profile},
			z0:      p.z0, z1: p.z1, z0Delta: p.z0Delta, z1Delta: p.z1Delta,
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
			profile: ProfileRecord{Outer: first.regions[0].Outer},
			frame:   p.faces[0].frame, xform: p.xform, sectionDelta: p.stack.delta,
			z0: first.z0, z0Delta: first.z0Delta, z1: last.z1, z1Delta: last.z1Delta,
		}, slabs: p.stack.slabs, brep: true}, true
	default:
		return stackedUnionOperand{}, false
	}
}

// view is the prismPayload a private scene reads for one of this operand's
// regions: the region, the operand's frame and placement, and its section
// displacement for prism-boolean §3.4's crossing charge.
func (o stackedUnionOperand) view(region ProfileRecord) prismPayload {
	v := o.proxy
	v.profile = region
	return v
}

// stackedUnionLevel is one distinct level of the union: its exact rational,
// the float the result holds, and that level's proven axial displacement.
type stackedUnionLevel struct {
	exact *big.Rat
	held  float64
	delta float64
}

// stackedUnionLevels lists both operands' slab boundaries on A's axis, sorted
// exactly. A's levels are its own floats. B's are floatRat(z) + s, rounded
// once to the nearest float, with rationalFloatError of that rounding added
// to B's own displacement (general-boolean §3 A1, stacked §1's blind-cut
// charge). A tie keeps A's float and the larger displacement. ok is false when
// a level does not lift or two distinct levels round to one float, either of
// which is a silent miss.
func stackedUnionLevels(va, vb stackedUnionOperand, shift *big.Rat) ([]stackedUnionLevel, bool) {
	var levels []stackedUnionLevel
	add := func(z, delta float64, shifted bool) bool {
		exact := proofarith.FloatRat(z)
		if exact == nil {
			return false
		}
		held := z
		if shifted {
			exact.Add(exact, shift)
			held, _ = exact.Float64()
			if round := proofarith.RationalFloatError(exact, held); round != 0 {
				if delta == 0 {
					delta = round
				} else {
					delta = proofbound.AbsSumUpper(delta, round)
				}
			}
		}
		levels = append(levels, stackedUnionLevel{exact: exact, held: held, delta: delta})
		return true
	}
	for _, op := range []struct {
		v       stackedUnionOperand
		shifted bool
	}{{va, false}, {vb, true}} {
		for i, slab := range op.v.slabs {
			if i == 0 && !add(slab.z0, slab.z0Delta, op.shifted) {
				return nil, false
			}
			if !add(slab.z1, slab.z1Delta, op.shifted) {
				return nil, false
			}
		}
	}
	sort.SliceStable(levels, func(i, j int) bool { return levels[i].exact.Cmp(levels[j].exact) < 0 })
	out := levels[:0]
	for _, l := range levels {
		if n := len(out); n > 0 && out[n-1].exact.Cmp(l.exact) == 0 {
			// A's levels are added first and the sort is stable, so the kept
			// entry is A's whenever A states this level.
			out[n-1].delta = max(out[n-1].delta, l.delta)
			continue
		}
		out = append(out, l)
	}
	for i := 1; i < len(out); i++ {
		if out[i-1].held >= out[i].held {
			return nil, false
		}
	}
	return out, true
}

// stackedUnionSlabOf finds the operand slab whose interval covers [lo, hi],
// shifted by shift, compared exactly; -1 when the operand does not reach it.
func stackedUnionSlabOf(o stackedUnionOperand, shift *big.Rat, lo, hi *big.Rat) int {
	for i, slab := range o.slabs {
		z0, z1 := proofarith.FloatRat(slab.z0), proofarith.FloatRat(slab.z1)
		z0.Add(z0, shift)
		z1.Add(z1, shift)
		if z0.Cmp(lo) <= 0 && z1.Cmp(hi) >= 0 {
			return i
		}
	}
	return -1
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
			if len(slab.regions) == 0 {
				return nil, false, nil
			}
			if len(slab.regions) != 1 {
				brepOnly = true
			}
			for _, region := range slab.regions {
				if len(region.Holes) != 0 { // G6
					return nil, false, nil
				}
				analytic, err := prismcells.ProfileAnalytic(budget, region) // G4
				if err != nil || !analytic {
					return nil, false, err
				}
				trimmed, err := prismProfileHasTrimmedCircularSource(budget, region)
				if err != nil || trimmed {
					return nil, false, err
				}
			}
		}
	}
	shift := prismZShift(va.proxy, vb.proxy)
	levels, ok := stackedUnionLevels(va, vb, shift)
	if !ok || len(levels) < 3 {
		// Equal intervals are prism-boolean §3.2's Union row, not a stack.
		return nil, false, nil
	}
	a0 := proofarith.FloatRat(va.slabs[0].z0)
	a1 := proofarith.FloatRat(va.slabs[len(va.slabs)-1].z1)
	b0 := proofarith.FloatRat(vb.slabs[0].z0)
	b1 := proofarith.FloatRat(vb.slabs[len(vb.slabs)-1].z1)
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
		bRecorded: map[int]ProfileRecord{}, scenes: map[[2]stackedUnionRegionRef]*stackedNesting{}}
	sp := stackedPrismPayload{frame: va.proxy.frame, xform: va.proxy.xform}
	zero := new(big.Rat)
	var reach [][2]int
	for k := 0; k+1 < len(levels); k++ {
		lo, hi := levels[k], levels[k+1]
		ia := stackedUnionSlabOf(va, zero, lo.exact, hi.exact)
		ib := stackedUnionSlabOf(vb, shift, lo.exact, hi.exact)
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
		sp.slabs = append(sp.slabs, prismSlab{regions: []ProfileRecord{region},
			z0: lo.held, z1: hi.held, z0Delta: lo.delta, z1Delta: hi.delta})
	}
	for k := 0; k+1 < len(sp.slabs); k++ {
		boundary, ok, err := st.interfaceOf(ctx, sp.slabs[k].regions[0], sp.slabs[k+1].regions[0])
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
	if err := stackedrecord.Falsify(ctx, stackedRecordOf(sp)); err != nil {
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
	bRecorded     map[int]ProfileRecord
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
		return st.vb.view(st.vb.slabs[ref.slab].regions[0])
	}
	return st.va.view(st.va.slabs[ref.slab].regions[0])
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
func (st *stackedUnionState) slabRegion(ctx context.Context, ia, ib int) (ProfileRecord, bool, error) {
	switch {
	case ia < 0 && ib < 0:
		return ProfileRecord{}, false, nil
	case ib < 0:
		return st.va.slabs[ia].regions[0], true, nil
	case ia < 0:
		return st.bRegion(ctx, ib)
	}
	ra, rb := st.va.slabs[ia].regions[0], st.vb.slabs[ib].regions[0]
	if st.reexpress.Identity {
		same, err := loopRecordsEqual(st.budget, ra.Outer, rb.Outer)
		if err != nil {
			return ProfileRecord{}, false, err
		}
		if same {
			return ra, true, nil
		}
	}
	pa, pb := st.va.view(ra), st.vb.view(rb)
	m, err := st.scene(ctx, stackedUnionRegionRef{slab: ia}, stackedUnionRegionRef{isB: true, slab: ib})
	if err != nil {
		return ProfileRecord{}, false, err
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
	// prismCellProfiles could not restate: the mesh path.
	if len(m.profiles) == 0 {
		return ProfileRecord{}, false, nil
	}
	if ok, err := m.charge(st.budget, pa, pb, st.reexpress); err != nil || !ok {
		return ProfileRecord{}, false, err
	}
	// Select-all is the union only without an enclosed void (§4.2): a void
	// is unresolved, never an error, so the pair takes the mesh path.
	voidFree, err := prismcells.CellsHaveNoVoid(st.budget, m.tags, m.profiles)
	if err != nil || !voidFree {
		return ProfileRecord{}, false, err
	}
	if ok, err := m.sceneDelta.SharedSpansBounded(st.budget, m.profiles); err != nil || !ok {
		return ProfileRecord{}, false, err
	}
	merged, cutDelta, resolved, err := mergePrismCells(st.budget, m.profiles, "union")
	if fallBack, err := prismcells.AmplifiedFallback(m.sceneDelta.Amplified, err); fallBack || err != nil || !resolved {
		return ProfileRecord{}, false, err
	}
	if fallBack, err := prismcells.AmplifiedFallback(m.sceneDelta.Amplified, auditPrismMergeSection(st.budget, pa, merged)); fallBack || err != nil {
		return ProfileRecord{}, false, err
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
func (st *stackedUnionState) bRegion(ctx context.Context, ib int) (ProfileRecord, bool, error) {
	region := st.vb.slabs[ib].regions[0]
	if st.reexpress.Identity {
		return region, true, nil
	}
	if recorded, ok := st.bRecorded[ib]; ok {
		return recorded, true, nil
	}
	pb := st.vb.view(region)
	empty := st.va.view(ProfileRecord{})
	if err := st.withinCap(empty, pb); err != nil {
		return ProfileRecord{}, false, err
	}
	s, tags, sceneDelta, err := buildPrismScene(st.budget, empty, pb, st.reexpress)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	st.walkB = max(st.walkB, sceneDelta.B)
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	bOuter, err := prismcells.LoopEntitySet(st.budget, tags, true, -1)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	match, resolved, err := prismcells.FindLoopMatch(st.budget, profiles, bOuter, nil)
	if err != nil || !resolved {
		return ProfileRecord{}, false, err
	}
	if !match.Valid {
		return ProfileRecord{}, false, prismcells.InvalidRegionError("union")
	}
	recorded, err := prismRecordArrangedProfileContext(ctx, match)
	if err != nil {
		return ProfileRecord{}, false, err
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
func (st *stackedUnionState) interfaceOf(ctx context.Context, lower, upper ProfileRecord) (prismSlabInterface, bool, error) {
	same, err := loopRecordsEqual(st.budget, lower.Outer, upper.Outer)
	if err != nil {
		return prismSlabInterface{}, false, err
	}
	if same {
		// Hole-free regions with one outer: nothing is exposed, recorded as
		// I7's own empty derivation.
		none, err := stackedrecord.Exposed(ctx, nil)
		if err != nil {
			return prismSlabInterface{}, false, err
		}
		return prismSlabInterface{lowerExposed: none, upperExposed: append([]ProfileRecord{}, none...)}, true, nil
	}
	m, err := stackedUnionInterfaceMatch(ctx, st.budget, st.va.proxy, lower, upper)
	if err != nil {
		return prismSlabInterface{}, false, err
	}
	st.walkInterface = max(st.walkInterface, m.sceneDelta.A, m.sceneDelta.B)
	switch m.nest {
	case stackedNestBInA:
		exposed, err := stackedrecord.UnionExposed(ctx, lower, upper)
		if err != nil {
			return prismSlabInterface{}, false, err
		}
		return prismSlabInterface{lowerExposed: exposed}, true, nil
	case stackedNestAInB:
		exposed, err := stackedrecord.UnionExposed(ctx, upper, lower)
		if err != nil {
			return prismSlabInterface{}, false, err
		}
		return prismSlabInterface{upperExposed: exposed}, true, nil
	}
	// A crossing (m.split) or any other unmatched pair: prism-boolean §4.4's
	// unresolved topology, a silent fallback.
	return prismSlabInterface{}, false, nil
}

// stackedUnionInterfaceMatch arranges two adjacent result regions, both
// already in the result's frame, with the identity re-expression.
func stackedUnionInterfaceMatch(ctx context.Context, budget *proofbound.WorkBudget, base prismPayload, lower, upper ProfileRecord) (stackedNesting, error) {
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
	segments, withinCap, err := prismSceneWithinWorkCap(budget, pa, pb)
	if err != nil {
		return err
	}
	if !withinCap {
		return fmt.Errorf(
			`%w: the analytic union scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1)`,
			ErrUnsupported, segments, prismMaxArrangementSegments)
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
	profiles, err := prismCellProfiles(ctx, budget, s)
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
	out.split, err = prismProfilesHaveSplitBoundary(budget, profiles)
	if err != nil {
		return stackedNesting{}, err
	}
	return out, nil
}
