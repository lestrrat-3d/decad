package stackedunion

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/sketch"
)

type (
	profileRecord       = momentinput.Profile
	loopRecord          = sectionrecord.LoopRecord
	prismPayload        = prismrecord.Payload
	brepPayload         = breprecord.Payload
	brepStack           = breprecord.Stack
	stackedPrismPayload = stackedrecord.Payload
	prismReexpression   = prismcells.Reexpression
	prismSceneDelta     = prismcells.SceneDelta
)

// Operand carries a root-admitted prism or stack into the internal builder.
type Operand struct {
	Proxy prismPayload
	Slabs []stackedrecord.Slab
}

// OperandOf retains the root operand's record and slab order.
func OperandOf(proxy prismPayload, slabs []stackedrecord.Slab) Operand {
	return Operand{Proxy: proxy, Slabs: slabs}
}

func (o Operand) view(region profileRecord) prismPayload {
	v := o.Proxy
	v.Profile = region
	return v
}

// Join resolves root-admitted stacked Union slabs and their interfaces.
func Join(ctx context.Context, budget *proofbound.WorkBudget, va, vb Operand,
	reexpress *prismReexpression, levels []stackedrecord.UnionLevel, shift *big.Rat,
	brepOnly bool) (topo.Payload, bool, error) {
	st := &stackedUnionState{budget: budget, va: va, vb: vb, reexpress: reexpress,
		bRecorded: map[int]profileRecord{}, scenes: map[[2]stackedUnionRegionRef]*Nesting{}}
	sp := stackedPrismPayload{Frame: va.Proxy.Frame, Xform: va.Proxy.Xform}
	zero := new(big.Rat)
	var reach [][2]int
	for k := 0; k+1 < len(levels); k++ {
		lo, hi := levels[k], levels[k+1]
		ia := stackedrecord.UnionSlabOf(va.Slabs, zero, lo.Exact, hi.Exact)
		ib := stackedrecord.UnionSlabOf(vb.Slabs, shift, lo.Exact, hi.Exact)
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
		sp.Slabs = append(sp.Slabs, stackedrecord.Slab{Regions: []profileRecord{region},
			Z0: lo.Held, Z1: hi.Held, Z0Delta: lo.Delta, Z1Delta: hi.Delta})
	}
	for k := 0; k+1 < len(sp.Slabs); k++ {
		boundary, ok, err := st.interfaceOf(ctx, sp.Slabs[k].Regions[0], sp.Slabs[k+1].Regions[0])
		if err != nil {
			return nil, false, err
		}
		if !ok {
			// A1's flush or crossing interface: the slabs are right and the
			// stacked record cannot state the result, so the brep build
			// classifies every interface's cells instead.
			return st.brep(ctx, levels, reach)
		}
		sp.Interfaces = append(sp.Interfaces, boundary)
	}
	sp.SectionDelta = st.sectionDelta()
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
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
			proofbound.AbsSumUpper(st.va.Proxy.SectionDelta, st.walkA),
			proofbound.AbsSumUpper(st.vb.Proxy.SectionDelta, st.walkB, st.reexpress.Delta),
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

// stackedUnionState tracks one Join or Cut call's scene charges, B regions
// recorded in A's frame, and slab scenes keyed by their operand regions.
type stackedUnionState struct {
	budget        *proofbound.WorkBudget
	va, vb        Operand
	reexpress     *prismReexpression
	bRecorded     map[int]profileRecord
	scenes        map[[2]stackedUnionRegionRef]*Nesting
	walkA, walkB  float64
	walkInterface float64
	cutDelta      float64
	crossing      float64
}

// view is the prism a private scene reads for a reference: the region under
// its operand's frame, placement and section displacement.
func (st *stackedUnionState) view(ref stackedUnionRegionRef) prismPayload {
	if ref.isB {
		return st.vb.view(st.vb.Slabs[ref.slab].Regions[0])
	}
	return st.va.view(st.va.Slabs[ref.slab].Regions[0])
}

// scene arranges two operand regions once and keeps the arrangement: the
// clean-nesting answer, the cells and their tags, and the scene's walk
// charge. The region named first enters as operand A of the scene.
func (st *stackedUnionState) scene(ctx context.Context, x, y stackedUnionRegionRef) (*Nesting, error) {
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
		// Two regions of one operand share a frame and need no re-expression.
		reexpress = &prismReexpression{Identity: true}
	}
	m, err := nestingOf(ctx, st.budget, px, py, reexpress)
	if err != nil {
		return nil, err
	}
	if x.isB {
		st.walkB = max(st.walkB, m.SceneDelta.A)
	} else {
		st.walkA = max(st.walkA, m.SceneDelta.A)
	}
	if y.isB {
		st.walkB = max(st.walkB, m.SceneDelta.B)
	} else {
		st.walkA = max(st.walkA, m.SceneDelta.B)
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
		return st.va.Slabs[ia].Regions[0], true, nil
	case ia < 0:
		return st.bRegion(ctx, ib)
	}
	ra, rb := st.va.Slabs[ia].Regions[0], st.vb.Slabs[ib].Regions[0]
	if st.reexpress.Identity {
		same, err := momentinput.ExactLoopEqual(st.budget, ra.Outer, rb.Outer)
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
	switch m.Nest {
	case NestBInA:
		return ra, true, nil
	case NestAInB:
		return st.bRegion(ctx, ib)
	}
	// Neither holds the other whole: prism-boolean §4.2's select-all merge,
	// with §3.4's crossing charge on every cut a displaced source can move
	// (docs/general-boolean-design.md §3 A6). No cell at all is a scene
	// prismcells.CellProfiles could not restate: the mesh path.
	if len(m.Profiles) == 0 {
		return profileRecord{}, false, nil
	}
	if ok, err := m.charge(st.budget, pa, pb, st.reexpress); err != nil || !ok {
		return profileRecord{}, false, err
	}
	// Select-all is the union only without an enclosed void (§4.2): a void
	// is unresolved, never an error, so the pair takes the mesh path.
	voidFree, err := prismcells.CellsHaveNoVoid(st.budget, m.Tags, m.Profiles)
	if err != nil || !voidFree {
		return profileRecord{}, false, err
	}
	if ok, err := m.SceneDelta.SharedSpansBounded(st.budget, m.Profiles); err != nil || !ok {
		return profileRecord{}, false, err
	}
	merged, cutDelta, resolved, err := prismcells.Merge(st.budget, m.Profiles, "union")
	if fallBack, err := prismcells.AmplifiedFallback(m.SceneDelta.Amplified, err); fallBack || err != nil || !resolved {
		return profileRecord{}, false, err
	}
	if fallBack, err := prismcells.AmplifiedFallback(m.SceneDelta.Amplified, prismcells.AuditMergeSection(st.budget, pa, merged)); fallBack || err != nil {
		return profileRecord{}, false, err
	}
	st.cutDelta = max(st.cutDelta, cutDelta)
	st.crossing = max(st.crossing, m.SceneDelta.Crossing)
	return merged, true, nil
}

// bRegion is operand B's slab region in A's frame. Under the identity
// re-expression that is B's own record. Otherwise B's region is arranged
// alone in a private scene of re-expressed entities and recorded from the
// one cell sketch returns, so every coordinate is a sketch-recorded edge and
// the re-expression's rounding rides in reexpress.Delta. The record is kept
// per B slab so every result slab that reads it holds one record.
func (st *stackedUnionState) bRegion(ctx context.Context, ib int) (profileRecord, bool, error) {
	region := st.vb.Slabs[ib].Regions[0]
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
	s, tags, sceneDelta, err := prismcells.BuildScene(st.budget, empty.Profile, pb.Profile, st.reexpress)
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
	recorded, err := prismcells.RecordArrangedProfileContext(ctx, match)
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
// narrower outer reversed as its hole. A split or other miss tries BRep;
// a silent BRep miss lets the caller take the mesh path.
func (st *stackedUnionState) interfaceOf(ctx context.Context, lower, upper profileRecord) (stackedrecord.Interface, bool, error) {
	same, err := momentinput.ExactLoopEqual(st.budget, lower.Outer, upper.Outer)
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
	m, err := InterfaceMatch(ctx, st.budget, st.va.Proxy, lower, upper)
	if err != nil {
		return stackedrecord.Interface{}, false, err
	}
	st.walkInterface = max(st.walkInterface, m.SceneDelta.A, m.SceneDelta.B)
	switch m.Nest {
	case NestBInA:
		exposed, err := stackedrecord.UnionExposed(ctx, lower, upper)
		if err != nil {
			return stackedrecord.Interface{}, false, err
		}
		return stackedrecord.Interface{LowerExposed: exposed}, true, nil
	case NestAInB:
		exposed, err := stackedrecord.UnionExposed(ctx, upper, lower)
		if err != nil {
			return stackedrecord.Interface{}, false, err
		}
		return stackedrecord.Interface{UpperExposed: exposed}, true, nil
	}
	// A crossing (m.Split) or any other unmatched pair: prism-boolean §4.4's
	// unresolved topology, a silent fallback.
	return stackedrecord.Interface{}, false, nil
}

// InterfaceMatch arranges two adjacent result regions, both
// already in the result's frame, with the identity re-expression.
func InterfaceMatch(ctx context.Context, budget *proofbound.WorkBudget, base prismPayload, lower, upper profileRecord) (Nesting, error) {
	pl, pu := base, base
	pl.Profile, pu.Profile = lower, upper
	if err := prismcells.RequireRegionsWithinCap(budget, pl.Profile, pu.Profile); err != nil {
		return Nesting{}, err
	}
	return nestingOf(ctx, budget, pl, pu, &prismReexpression{Identity: true})
}

func (st *stackedUnionState) withinCap(pa, pb prismPayload) error {
	return prismcells.RequireRegionsWithinCap(st.budget, pa.Profile, pb.Profile)
}

type Nest int

const (
	NestNone Nest = iota
	NestBInA      // B's outer is a whole hole of A's whole cell
	NestAInB      // A's outer is a whole hole of B's whole cell
)

// Nesting is one private scene's clean-nesting answer: which operand's
// cell carries the other's outer whole as a hole, the scene's cells for a
// merge, whether any arranged edge is Partial, and the scene's walk charge.
// charged records that prismcells.SceneDelta.ChargeCrossings has run over the scene, so a second
// reader of one cached scene takes the charge already in sceneDelta.
type Nesting struct {
	Nest       Nest
	Split      bool
	Profiles   []*sketch.Profile
	Tags       map[sketch.Entity]prismcells.Origin
	SceneDelta prismSceneDelta
	Charged    bool
	ChargeOK   bool
}

// charge runs A6's crossing charge once over the scene's cells. ok=false is
// a crossing or a span with no proven charge: the mesh path.
func (m *Nesting) charge(budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (bool, error) {
	if m.Charged {
		return m.ChargeOK, nil
	}
	ok, err := m.SceneDelta.ChargeCrossings(budget, m.Tags, m.Profiles, pa.SectionDelta, pb.SectionDelta, reexpress.Delta)
	if err != nil {
		return false, err
	}
	m.Charged, m.ChargeOK = true, ok
	return ok, nil
}

// nestingOf runs prism-boolean §4.2's structural whole-loop match in
// both directions over one private scene of two hole-free regions. A matched
// proof cell that sketch reports invalid is RB1.
func nestingOf(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (Nesting, error) {
	s, tags, sceneDelta, err := prismcells.BuildScene(budget, pa.Profile, pb.Profile, reexpress)
	if err != nil {
		return Nesting{}, err
	}
	if err := budget.Err(); err != nil {
		return Nesting{}, err
	}
	profiles, err := prismcells.CellProfiles(ctx, budget, s)
	if err != nil {
		return Nesting{}, err
	}
	out := Nesting{Profiles: profiles, Tags: tags, SceneDelta: sceneDelta}
	aOuter, err := prismcells.LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return Nesting{}, err
	}
	bOuter, err := prismcells.LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return Nesting{}, err
	}
	for _, dir := range []struct {
		outer, hole map[sketch.Entity]struct{}
		nest        Nest
	}{{aOuter, bOuter, NestBInA}, {bOuter, aOuter, NestAInB}} {
		proof, matched, err := prismcells.FindLoopMatch(budget, profiles, dir.outer, []map[sketch.Entity]struct{}{dir.hole})
		if err != nil {
			return Nesting{}, err
		}
		if !matched {
			continue
		}
		if !proof.Valid {
			return Nesting{}, prismcells.InvalidRegionError("union")
		}
		out.Nest = dir.nest
		return out, nil
	}
	out.Split, err = prismcells.HasSplitBoundary(budget, profiles)
	if err != nil {
		return Nesting{}, err
	}
	return out, nil
}
