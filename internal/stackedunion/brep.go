package stackedunion

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/sketch"
)

// This file builds agent-docs/prism-boolean-design.md Part G §3 A1's brep result
// from admitted operand slabs. It handles operands that already require a
// BRep, slabs whose records stay apart (separate bosses on a plate), and
// interfaces that clean nesting cannot state as a stacked-prism record, such
// as a flush or crossing boss. An operand with holed regions enters only under
// the shared-span gate (holed_gate.go, holed_scenes.go, holed_merge.go): a
// hole of one operand must share a span with an outer of the other.
//
// Every 2D answer is a private scene's: a slab both operands reach arranges
// every record reaching it and its regions are the select-all merge's loops,
// proven apart when there are several; each interface arranges every record
// reaching the slab below or above it and prismcells.ClassifyRegions names
// the cells on each side (a union's, where footprints share no edge,
// prismcells.ClassifyRegionsApart), so the exposed floors and ceilings are cells,
// recorded as sketch returned them. A union floor round separate bosses is
// one cell with a hole per boss, whose rings must not touch.
// An operand is a prism, a stacked prism, or an A1 result read through the
// slabs it keeps (brepStack). decad then only restates what the scenes recorded: every
// junction becomes one canonical vertex (a line's exact level, or the one
// float the keyed table holds for a line crossing a circle, as class B's
// §10 table does), every segment is rewritten between its vertices, and every
// straight wall on one axis-aligned plane becomes one planar face in its own
// frame — the rectangles each slab's segment sweeps, with their shared edges
// cancelled — while every curved or oblique wall is a swept piece between its
// vertices. A pair this build does not cover is a silent miss: the caller
// takes the mesh path with no error, as prism-boolean §4.4 states for an
// unresolved topology.

type ubLoop = stackedbrep.Loop

// ubRef names one operand record: a region of one operand's slab.
type ubRef struct {
	isB          bool
	slab, region int
}

// ubScene is one private scene over a set of records, cached by that set:
// operand A's records enter as the scene's A regions and operand B's as its
// B regions, so a span two records share reads through
// prismcells.CoincidentEdgesRegions whichever operand each belongs to.
type ubScene struct {
	refsA, refsB []ubRef
	sketch       *sketch.Sketch
	profiles     []*sketch.Profile
	tags         map[sketch.Entity]prismcells.Origin
	delta        prismSceneDelta
	charged      bool
	chargeOK     bool
	matter       map[prismcells.RegionKey][]bool
}

// key is a record's key in the scene's classification.
func (sc *ubScene) key(ref ubRef) (prismcells.RegionKey, bool) {
	refs := sc.refsA
	if ref.isB {
		refs = sc.refsB
	}
	for i, have := range refs {
		if have == ref {
			return prismcells.RegionKey{IsB: ref.isB, Region: i}, true
		}
	}
	return prismcells.RegionKey{}, false
}

// ubBuild is one brep build over the stacked union's slabs.
type ubBuild struct {
	st  *stackedUnionState
	cut bool
	// sideNotch relaxes the cut slab to general-boolean §3 A9's rules: one
	// cell, no ring contact, any number of holes. A7 keeps one outer and one
	// hole.
	sideNotch bool
	levels    []stackedrecord.UnionLevel
	// reach names, per result slab, each operand's slab reaching it (−1
	// when it does not).
	reach  [][2]int
	scenes map[string]*ubScene
	// slabs holds each result slab's regions.
	slabs    [][]profileRecord
	geom     *stackedbrep.Engine
	cutDelta float64
	walk     float64
	crossing float64

	// holedUnion marks a union with a holed record that passed the
	// pre-filter (holed_gate.go); Cut never sets it. segmentCap is the
	// arrangement cap of its scenes, and aliasCharge the carrier alias's
	// charge (holed_scenes.go).
	holedUnion  bool
	segmentCap  int
	aliasCharge float64

	// apart marks a union interface whose classification read a cell outside
	// a record it shares no edge with (prismcells.ClassifyRegionsApart), and
	// touch an interface cell inside a record of each operand. A build with
	// apart and no touch misses. The guard narrows where the reading is tried;
	// it neither makes the result correct, which ClassifyRegionsApart's own
	// contact check decides, nor keeps it in one piece: an operand of two
	// pieces meeting the other with one of them builds an exact two-lump
	// brep, which is accepted.
	apart, touch bool
}

// errUBMiss marks a topology this build does not cover, found after a scene
// was built: the caller treats it as a silent miss.
var errUBMiss = brepgeom.ErrStackedWallMiss

// brep states the stacked union as a brepPayload. ok=false with a nil error
// is a silent miss. A non-nil error is cancellation or a refusal past the
// gate that the stacked path itself raises.
func (st *stackedUnionState) brep(ctx context.Context, levels []stackedrecord.UnionLevel, reach [][2]int) (topo.Payload, bool, error) {
	if !st.reexpress.Identity || st.va.Proxy.SectionDelta != 0 || st.vb.Proxy.SectionDelta != 0 {
		// B's records enter every scene verbatim only under the identity
		// re-expression; a re-expressed B would be recorded once per scene.
		// Every straight wall becomes a planar face at its recorded level,
		// which is the true wall only when neither operand carries a
		// section displacement (class B's B5 rule): a planar face states no
		// band for a wall that moved.
		return nil, false, nil
	}
	held := make([]float64, len(levels))
	for i, level := range levels {
		held[i] = level.Held
	}
	b := &ubBuild{st: st, levels: levels, reach: reach,
		scenes: map[string]*ubScene{}, geom: stackedbrep.NewEngine(held)}
	if st.hasHoledRecord() {
		if !holedCandidate(st.va, st.vb) {
			return nil, false, nil
		}
		b.holedUnion, b.segmentCap = true, gatedArrangementSegments
		b.geom.SetHoledUnion(func(p brepgeom.Profile) error {
			rec := profileRecord{Outer: p.Outer, Holes: p.Holes}
			return prismcells.AuditMergeSection(st.budget, prismrecord.Payload{Profile: rec}, rec)
		})
	}
	bp, err := b.run(ctx)
	if errors.Is(err, errUBMiss) || errors.Is(err, decaderr.ErrUnsupported) {
		return nil, false, nil
	}
	if err == nil && b.apart && !b.touch {
		// No interface cell lies in both operands: the apart reading is not
		// tried for operands that meet nowhere.
		return nil, false, nil
	}
	if b.holedUnion && err != nil && !errors.Is(err, context.Canceled) && !errors.Is(err, context.DeadlineExceeded) {
		// A holed pair the build does not cover takes the mesh path
		// whatever the build's refusal was.
		return nil, false, nil
	}
	if err != nil {
		return nil, false, err
	}
	return bp, true, nil
}

// Cut builds the already-admitted blind cut from its two prism slabs: A7's
// crossing blind cut, or with sideNotch A9's side notch (general-boolean §3). It
// returns the builder's original miss or refusal for prismboolean to map.
func Cut(ctx context.Context, budget *proofbound.WorkBudget, va, vb Operand,
	reexpress *prismReexpression, levels []stackedrecord.UnionLevel, reach [][2]int, sideNotch bool) (breprecord.Payload, error) {
	state := &stackedUnionState{budget: budget, va: va, vb: vb, reexpress: reexpress}
	build := &ubBuild{st: state, cut: true, sideNotch: sideNotch, levels: levels, reach: reach,
		scenes: map[string]*ubScene{}, geom: stackedbrep.NewEngine([]float64{
			levels[0].Held, levels[1].Held, levels[2].Held,
		})}
	return build.run(ctx)
}

func (b *ubBuild) run(ctx context.Context) (brepPayload, error) {
	if b.holedUnion {
		if err := b.prepareHoled(ctx); err != nil {
			return brepPayload{}, err
		}
	}
	// Slab loops first: their recorded corners seed the table, so a scene's
	// computed crossing at a recorded corner takes the record's own point.
	n := len(b.reach)
	for k := range n {
		regions, err := b.slabRegions(ctx, k)
		if err != nil {
			return brepPayload{}, err
		}
		var loops []ubLoop
		for _, region := range regions {
			for _, ring := range append([]loopRecord{region.Outer}, region.Holes...) {
				loop, err := b.geom.LoopOf(brepgeom.Profile{Outer: ring}, b.st.budget)
				if err != nil {
					return brepPayload{}, err
				}
				loops = append(loops, loop)
			}
		}
		b.slabs = append(b.slabs, regions)
		b.geom.SlabLoops = append(b.geom.SlabLoops, loops)
	}
	for _, region := range b.slabs[0] {
		outer, holes, err := b.regionLoops(region)
		if err != nil {
			return brepPayload{}, err
		}
		if err := b.geom.AddFaceRegion(outer, holes, 0, false); err != nil {
			return brepPayload{}, err
		}
	}
	for _, region := range b.slabs[n-1] {
		outer, holes, err := b.regionLoops(region)
		if err != nil {
			return brepPayload{}, err
		}
		if err := b.geom.AddFaceRegion(outer, holes, n, true); err != nil {
			return brepPayload{}, err
		}
	}
	for k := 1; k < n; k++ {
		if err := b.interfaceFaces(ctx, k); err != nil {
			return brepPayload{}, err
		}
	}
	b.geom.RecordJunctions(n)
	if b.walk != 0 || (b.crossing != 0 && !b.holedUnion) {
		// A walked endpoint or an amplified crossing moves a carrier, not
		// just a vertex on it; only a swept face carries that as a band. A
		// holed union carries the width of its shared spans as that band
		// (aliasCharge), and no displacement enters it, so its crossing is
		// that width alone.
		return brepPayload{}, errUBMiss
	}
	// Every vertex lies on exact carriers: an axis-aligned plane at its
	// recorded level, or a circle about its recorded centre. A cut vertex
	// sits within its own allowance of the crossing it denotes along those
	// carriers, so each planar face's region and each swept piece's pinned
	// arc is within the largest canonical-vertex allowance of the face it
	// denotes. No face coordinate reads a slab-merge cut parameter: a
	// junction is a canonical vertex, and a canonical vertex that reads a
	// cut end already carries that end's cut charge (stackedbrep ubEnds).
	// The faces therefore carry geom.Allow() alone. The kept slab fragments
	// are the only record that reads a merge's cut parameter verbatim, so
	// the stack carries cutDelta on top of that allowance.
	faceDelta := b.geom.Allow()
	if b.holedUnion {
		faceDelta = proofbound.AbsSumUpper(faceDelta, b.aliasCharge)
	}
	stackDelta := proofbound.AbsSumUpper(b.cutDelta, faceDelta)
	faceLevels := make([]stackedbrep.Level, len(b.levels))
	for i, level := range b.levels {
		faceLevels[i] = stackedbrep.Level{Held: level.Held, Delta: level.Delta}
	}
	out, err := breprecord.StackedRecord(ctx, b.st.budget, b.geom, faceLevels, b.st.va.Proxy.Frame, b.st.va.Proxy.Xform, faceDelta)
	if err != nil {
		return brepPayload{}, err
	}
	out.SlabCut = b.cut
	out.CarrierCharge = b.aliasCharge
	if !b.cut {
		// A1's union result keeps its slabs for a further co-directional
		// Union (§4.1). The crossing Cut's holed slabs are not A1 operands.
		stack := &brepStack{Delta: stackDelta}
		for k, regions := range b.slabs {
			lo, hi := b.levels[k], b.levels[k+1]
			stack.Slabs = append(stack.Slabs, stackedrecord.Slab{Regions: regions,
				Z0: lo.Held, Z1: hi.Held, Z0Delta: lo.Delta, Z1Delta: hi.Delta,
				Z0Denoted: lo.Denotation(), Z1Denoted: hi.Denotation()})
		}
		out.Stack = stack
	} else if b.cut && b.st.va.Proxy.SectionDelta == 0 && b.st.vb.Proxy.SectionDelta == 0 &&
		lineOnlyOperand(b.st.va) && lineOnlyOperand(b.st.vb) && lineOnlySlabs(b.slabs) {
		// Keep the exact source prisms for point queries. Their regularized Cut
		// is the set this A7 BRep records, and they remain separate from the A1
		// operand stack read by OperandFromPayload.
		out.PointQueryCut = &breprecord.CutOperands{Target: b.st.va.Proxy, Tool: b.st.vb.Proxy}
	}
	return out, nil
}

func lineOnlyOperand(op Operand) bool {
	for _, slab := range op.Slabs {
		for _, region := range slab.Regions {
			if !region.AllLines() {
				return false
			}
		}
	}
	return len(op.Slabs) != 0
}

func lineOnlySlabs(slabs [][]profileRecord) bool {
	for _, slab := range slabs {
		for _, region := range slab {
			if !region.AllLines() {
				return false
			}
		}
	}
	return len(slabs) != 0
}

func (b *ubBuild) regionLoops(region profileRecord) (ubLoop, []ubLoop, error) {
	outer, err := b.geom.LoopOf(brepgeom.Profile{Outer: region.Outer}, b.st.budget)
	if err != nil {
		return ubLoop{}, nil, err
	}
	var holes []ubLoop
	for _, ring := range region.Holes {
		hole, err := b.geom.LoopOf(brepgeom.Profile{Outer: ring}, b.st.budget)
		if err != nil {
			return ubLoop{}, nil, err
		}
		holes = append(holes, hole)
	}
	return outer, holes, nil
}

// refs names every record of one operand's slab.
func (b *ubBuild) refs(isB bool, slab int) []ubRef {
	if slab < 0 {
		return nil
	}
	op := b.st.va
	if isB {
		op = b.st.vb
	}
	out := make([]ubRef, len(op.Slabs[slab].Regions))
	for r := range out {
		out[r] = ubRef{isB: isB, slab: slab, region: r}
	}
	return out
}

func (b *ubBuild) record(ref ubRef) profileRecord {
	if ref.isB {
		return b.st.vb.Slabs[ref.slab].Regions[ref.region]
	}
	return b.st.va.Slabs[ref.slab].Regions[ref.region]
}

// scene arranges a set of records once and keeps the arrangement. A scene
// whose walk charge is not zero, or whose cells sketch leaves unresolved,
// is a miss.
func (b *ubBuild) scene(ctx context.Context, refsA, refsB []ubRef) (*ubScene, error) {
	key := fmt.Sprint(refsA, refsB)
	if sc, ok := b.scenes[key]; ok {
		return sc, nil
	}
	var regionsA, regionsB []profileRecord
	for _, ref := range refsA {
		regionsA = append(regionsA, b.record(ref))
	}
	for _, ref := range refsB {
		regionsB = append(regionsB, b.record(ref))
	}
	segCap := prismcells.MaxArrangementSegments
	if b.segmentCap != 0 {
		segCap = b.segmentCap
	}
	segments, within, err := prismcells.RegionsWithinSegmentCap(b.st.budget, segCap,
		append(append([]profileRecord{}, regionsA...), regionsB...)...)
	if err != nil {
		return nil, err
	}
	if !within {
		return nil, prismcells.WithWorkLimit(fmt.Errorf(
			`%w: the analytic union scene charges at least %d arranger segments against this evaluator's cap of %d (each line costs 1, each circle or arc up to 256 by its turn)`,
			decaderr.ErrUnsupported, segments, segCap), segments)
	}
	s, tags, delta, err := prismcells.BuildProfileScene(b.st.budget, regionsA, regionsB, &prismReexpression{Identity: true})
	if err != nil {
		return nil, err
	}
	if err := b.st.budget.Err(); err != nil {
		return nil, err
	}
	profiles, err := prismcells.CellProfiles(ctx, b.st.budget, s)
	if err != nil {
		return nil, err
	}
	if b.cut {
		for _, cell := range profiles {
			for _, ring := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
				for _, edge := range ring {
					if err := b.st.budget.Step(); err != nil {
						return nil, err
					}
					if edge.Partial && !edge.TExact {
						return nil, errUBMiss
					}
					// The build states a cut vertex exactly only on an
					// axis-aligned line or a circle (A7); an oblique record
					// may reach a cut result only whole (A9).
					if line, ok := edge.Entity.(*sketch.Line); ok && edge.Partial &&
						line.Start.X() != line.End.X() && line.Start.Y() != line.End.Y() {
						return nil, errUBMiss
					}
				}
			}
		}
	}
	if len(profiles) == 0 {
		return nil, errUBMiss
	}
	b.walk = math.Max(b.walk, math.Max(delta.A, delta.B))
	sc := &ubScene{refsA: refsA, refsB: refsB, sketch: s, profiles: profiles, tags: tags, delta: delta}
	b.scenes[key] = sc
	return sc, nil
}

// charge runs A6's crossing charge once over the scene's cells and reads
// the shared spans' width into the build's crossing term.
func (b *ubBuild) charge(sc *ubScene) error {
	if !sc.charged {
		ok, err := sc.delta.ChargeCrossings(b.st.budget, sc.tags, sc.profiles, b.st.va.Proxy.SectionDelta, b.st.vb.Proxy.SectionDelta, 0)
		if err != nil {
			return err
		}
		sc.charged, sc.chargeOK = true, ok
	}
	if !sc.chargeOK {
		return errUBMiss
	}
	b.crossing = math.Max(b.crossing, sc.delta.Crossing)
	return nil
}

// slabRegions is one result slab's regions: one operand's records verbatim
// where the other does not reach, else the select-all merge's loops over
// every record reaching the slab (prism-boolean §4.2 with its enclosed-void
// check, A5's several loops proven apart by prismcells.ProveGroupApart with
// its walk charge, §6's audit per loop).
func (b *ubBuild) slabRegions(ctx context.Context, k int) ([]profileRecord, error) {
	refsA, refsB := b.refs(false, b.reach[k][0]), b.refs(true, b.reach[k][1])
	var verbatim []ubRef
	switch {
	case len(refsA) == 0:
		verbatim = refsB
	case len(refsB) == 0:
		verbatim = refsA
	}
	if verbatim != nil {
		if b.cut && len(refsA) == 0 {
			return nil, errUBMiss
		}
		out := make([]profileRecord, len(verbatim))
		for i, ref := range verbatim {
			out[i] = b.record(ref)
		}
		return out, nil
	}
	sc, err := b.scene(ctx, refsA, refsB)
	if err != nil {
		return nil, err
	}
	if err := b.charge(sc); err != nil {
		return nil, err
	}
	if b.holedUnion && b.sceneHasHoles(sc) {
		return b.holedSlabRegions(ctx, sc)
	}
	if b.cut {
		return b.cutSlabRegion(sc)
	}
	voidFree, err := prismcells.CellsHaveNoVoid(b.st.budget, sc.tags, sc.profiles)
	if err != nil {
		return nil, err
	}
	if !voidFree {
		return nil, errUBMiss
	}
	if ok, err := sc.delta.SharedSpansBounded(b.st.budget, sc.profiles); err != nil || !ok {
		if err != nil {
			return nil, err
		}
		return nil, errUBMiss
	}
	loops, cutDelta, resolved, err := prismcells.MergeLoopsExactLineCuts(b.st.budget, sc.sketch, sc.profiles, "union")
	if err != nil {
		return nil, err
	}
	if !resolved {
		return nil, errUBMiss
	}
	b.cutDelta = math.Max(b.cutDelta, cutDelta)
	out := make([]profileRecord, len(loops))
	for i, loop := range loops {
		area, err := classbgeom.LoopSignedArea(loop)
		if err != nil {
			return nil, err
		}
		if !(area > 0) {
			// A clockwise loop would be a hole of a merged region, which the
			// void check already refuses; a holed union's merge is
			// holedSlabRegions, which owns its holes.
			return nil, errUBMiss
		}
		out[i] = profileRecord{Outer: loop}
		if err := prismcells.AuditMergeSection(b.st.budget, prismrecord.Payload{Profile: out[i]}, out[i]); err != nil {
			return nil, err
		}
	}
	if err := b.proveApart(ctx, out); err != nil {
		return nil, err
	}
	return out, nil
}

// proveApart requires several loops to be separate lumps of the slab: apart
// as well as disjoint (A5). Two tangent boss circles close as two whole loops
// that meet at a point no loop names.
func (b *ubBuild) proveApart(ctx context.Context, out []profileRecord) error {
	if len(out) <= 1 {
		return nil
	}
	apart, walk, err := prismcells.ProveGroupApart(ctx, b.st.budget, out)
	if err != nil {
		return err
	}
	if !apart {
		return errUBMiss
	}
	b.walk = math.Max(b.walk, walk)
	return nil
}

// cutSlabRegion keeps the cells inside the target and outside the tool.
// The selected boundary must close into one outer ring and one inner ring;
// the exact scene owns every crossing and shared span in those rings.
func (b *ubBuild) cutSlabRegion(sc *ubScene) ([]profileRecord, error) {
	if ok, err := sc.delta.SharedSpansBounded(b.st.budget, sc.profiles); err != nil || !ok {
		if err != nil {
			return nil, err
		}
		return nil, errUBMiss
	}
	if sc.matter == nil {
		matter, resolved, err := prismcells.ClassifyRegions(b.st.budget, sc.tags, sc.profiles)
		if err != nil {
			return nil, err
		}
		if !resolved {
			return nil, errUBMiss
		}
		sc.matter = matter
	}
	selected := make([]*sketch.Profile, 0, len(sc.profiles))
	for i, cell := range sc.profiles {
		insideA, insideB := false, false
		for r := range sc.refsA {
			insideA = insideA || sc.matter[prismcells.RegionKey{Region: r}][i]
		}
		for r := range sc.refsB {
			insideB = insideB || sc.matter[prismcells.RegionKey{IsB: true, Region: r}][i]
		}
		if insideA && !insideB {
			selected = append(selected, cell)
		}
	}
	if b.sideNotch {
		// Two cells inside the target and outside the tool share no edge: a
		// target edge has target material on one side only, and a tool edge
		// has the tool on one side. Several such cells are separate lumps, or
		// touch at a point, which no single region states. The one cell's
		// rings must not touch each other at a point the arrangement leaves
		// uncut (a round pocket tangent to a hole); a ring touching itself
		// there is the section audit's below.
		if len(selected) != 1 {
			return nil, errUBMiss
		}
		touch, err := prismcells.RingContact(b.st.budget, selected[0])
		if err != nil {
			return nil, err
		}
		if touch {
			return nil, errUBMiss
		}
	}
	loops, cutDelta, resolved, err := prismcells.MergeLoops(b.st.budget, selected, "cut")
	if err != nil {
		return nil, err
	}
	if !resolved || len(loops) == 0 || !b.sideNotch && len(loops) != 2 {
		return nil, errUBMiss
	}
	b.cutDelta = math.Max(b.cutDelta, cutDelta)
	var region profileRecord
	for _, loop := range loops {
		area, err := classbgeom.LoopSignedArea(loop)
		if err != nil {
			return nil, err
		}
		if area > 0 {
			if len(region.Outer.Segments) != 0 {
				return nil, errUBMiss
			}
			region.Outer = loop
		} else if area < 0 {
			region.Holes = append(region.Holes, loop)
		} else {
			return nil, errUBMiss
		}
	}
	if len(region.Outer.Segments) == 0 || !b.sideNotch && len(region.Holes) != 1 {
		return nil, errUBMiss
	}
	if err := prismcells.AuditMergeSection(b.st.budget, prismrecord.Payload{Profile: region}, region); err != nil {
		return nil, err
	}
	return []profileRecord{region}, nil
}

// sameRegion reports whether two records of one operand are one region: the
// same outer loop and the same holes, in order, compared exactly. Two slabs
// whose outers match but whose holes differ, such as a counterbore's shoulder
// slab and its bore slab, are two records, so the interface scene sees the
// shoulder between them.
func sameRegion(budget *proofbound.WorkBudget, a, b profileRecord) (bool, error) {
	if len(a.Holes) != len(b.Holes) {
		return false, nil
	}
	pairs := [][2]loopRecord{{a.Outer, b.Outer}}
	for i := range a.Holes {
		pairs = append(pairs, [2]loopRecord{a.Holes[i], b.Holes[i]})
	}
	for _, pair := range pairs {
		equal, err := momentinput.ExactLoopEqual(budget, pair[0], pair[1])
		if err != nil || !equal {
			return false, err
		}
	}
	return true, nil
}

// interfaceFaces classifies the interface at level k: the scene of every
// record reaching the slab below or the slab above it, each cell's
// membership in every record read by prismcells.ClassifyRegions, and the
// cells inside some record of one side and none of the other recorded as
// that side's exposed faces. A union's exposed cell with holes, such as a
// plate's floor round separate bosses, becomes one planar face with those
// holes when prismcells.RingContact finds no two of its rings touching; a
// cut's holed cell misses. A stacked operand's equal regions on both sides,
// outer and holes alike (sameRegion), enter once.
func (b *ubBuild) interfaceFaces(ctx context.Context, k int) error {
	sides, refsA, refsB, ok, err := b.interfaceRefs(k)
	if err != nil || !ok {
		return err
	}
	sc, err := b.scene(ctx, refsA, refsB)
	if err != nil {
		return err
	}
	if err := b.charge(sc); err != nil {
		return err
	}
	if sc.matter == nil {
		// An exposed floor round a boss standing inside it is a cell with
		// holes, for a union as for a cut.
		matter, resolved, err := prismcells.ClassifyRegions(b.st.budget, sc.tags, sc.profiles)
		if err != nil {
			return err
		}
		if !resolved && !b.cut {
			// Blocks of unequal heights standing apart on a plate leave, above
			// the lower one's top, footprints that share no edge.
			var apart bool
			matter, resolved, apart, err = prismcells.ClassifyRegionsApart(b.st.budget, sc.tags, sc.profiles)
			if err != nil {
				return err
			}
			b.apart = b.apart || apart
		}
		if !resolved {
			return errUBMiss
		}
		sc.matter = matter
	}
	// exposed holds each cell's material side: +1 a floor (material below
	// only), −1 a ceiling (above only), 0 neither.
	exposed := make([]int, len(sc.profiles))
	for i := range sc.profiles {
		var sideMatter [2][2]bool
		for op := range 2 {
			for _, sd := range sides[op] {
				key, ok := sc.key(sd.ref)
				if !ok {
					return errUBMiss
				}
				in := sc.matter[key][i]
				sideMatter[op][0] = sideMatter[op][0] || (sd.below && in)
				sideMatter[op][1] = sideMatter[op][1] || (sd.above && in)
			}
		}
		if (sideMatter[0][0] || sideMatter[0][1]) && (sideMatter[1][0] || sideMatter[1][1]) {
			// Both operands hold this cell's area, at or beside the interface.
			b.touch = true
		}
		below := sideMatter[0][0] || sideMatter[1][0]
		above := sideMatter[0][1] || sideMatter[1][1]
		if b.cut {
			below = sideMatter[0][0] && !sideMatter[1][0]
			above = sideMatter[0][1] && !sideMatter[1][1]
		}
		switch {
		case below && !above:
			exposed[i] = 1
		case above && !below:
			exposed[i] = -1
		}
	}
	if !b.cut {
		if err := b.interfaceManifold(sc, sides, exposed); err != nil {
			return err
		}
	}
	for i, p := range sc.profiles {
		if exposed[i] == 0 {
			continue
		}
		below := exposed[i] > 0
		region, err := prismcells.RecordArrangedProfileContext(ctx, p)
		if fallBack, err := prismcells.AmplifiedFallback(sc.delta.Amplified, err); fallBack || err != nil {
			if err != nil {
				return err
			}
			return errUBMiss
		}
		if len(region.Holes) != 0 && !b.cut {
			// A union floor round separate bosses: each boss footprint is a
			// hole of the floor cell. Rings that touch would pinch the face.
			touch, err := prismcells.RingContact(b.st.budget, p)
			if err != nil {
				return err
			}
			if touch {
				return errUBMiss
			}
			outer, holes, err := b.regionLoops(region)
			if err != nil {
				return err
			}
			if err := b.geom.AddFaceRegion(outer, holes, k, below); err != nil {
				return err
			}
			continue
		}
		loop, err := b.geom.LoopOf(brepgeom.Profile{Outer: region.Outer, Holes: region.Holes}, b.st.budget)
		if err != nil {
			return err
		}
		if err := b.geom.AddFace(loop, k, below); err != nil {
			return err
		}
	}
	return nil
}

// interfaceManifold refuses a union interface whose floors and ceilings meet
// at a point where the body meets itself (general-boolean §3 "A1 as a brep").
// One connected group of cells can touch itself so: a U-shaped bridge resting
// on a tower by one arm while its other arm's corner meets the tower's top
// corner, or rests there on the tower's edge from outside, or is tangent to a
// boss's top circle. The cells' membership still resolves, the record pairs
// every edge, and nothing else reads that point. A floor and a ceiling meet
// only where a record reaching just the slab below and one reaching just the
// slab above both pass; a record reaching both sides changes neither side's
// material there unless its own boundary passes the point too. So:
//
//   - every point where a record of one side touches a record of the other, a
//     record reaching both sides read against each, is a vertex the scene
//     cut, so every cell round it walks it (prismcells.ContactsCut). A
//     transverse crossing inside both records' entities is exempt: its four
//     half-edges alternate round it, cut or not;
//   - at a vertex a floor and a ceiling both walk, exactly one record
//     reaching only the slab below and one reaching only the slab above have
//     edges ending there, a shared span's edge counting for both its records,
//     and none reaching both. Each record's boundary
//     passes the point once, so the material below there is the one sector of
//     the first record and the material above that of the second;
//   - every floor and ceiling corner there turns from one of those records'
//     edges to the other's, on no span two records share (opposedManifold).
//     Two sectors whose four bounding edges alternate between the records are
//     a transverse crossing, where the body is manifold. Any other order puts
//     a floor or a ceiling between two edges of one record: the floor and
//     ceiling of a corner touch, or two notches whose outsides meet.
//
// Every point and edge is sketch's own, matched by exact equality; the check
// only refuses.
func (b *ubBuild) interfaceManifold(sc *ubScene, sides [2][]ifaceSide, exposed []int) error {
	const (
		reachBoth = iota
		reachBelow
		reachAbove
	)
	reachOf := map[prismcells.RegionKey]int{}
	for op := range 2 {
		for _, sd := range sides[op] {
			key, ok := sc.key(sd.ref)
			if !ok {
				return errUBMiss
			}
			switch {
			case sd.below && sd.above:
				reachOf[key] = reachBoth
			case sd.below:
				reachOf[key] = reachBelow
			default:
				reachOf[key] = reachAbove
			}
		}
	}
	reach := func(e sketch.Entity) (prismcells.RegionKey, int, bool) {
		origin, ok := sc.tags[e]
		if !ok {
			return prismcells.RegionKey{}, 0, false
		}
		key := prismcells.RegionKey{IsB: origin.IsB, Region: origin.Region}
		r, ok := reachOf[key]
		return key, r, ok
	}
	coincident, ok, err := prismcells.CoincidentEdgesRegions(b.st.budget, sc.tags, sc.profiles)
	if err != nil {
		return err
	}
	if !ok {
		return errUBMiss
	}
	for _, bothBelow := range []bool{true, false} {
		tags := map[sketch.Entity]prismcells.Origin{}
		for e := range sc.tags {
			_, r, ok := reach(e)
			if !ok {
				return errUBMiss
			}
			tags[e] = prismcells.Origin{IsB: r == reachAbove || r == reachBoth && !bothBelow}
		}
		cut, err := prismcells.ContactsCut(b.st.budget, tags, sc.profiles, coincident)
		if err != nil {
			return err
		}
		if !cut {
			return errUBMiss
		}
	}
	type point struct {
		floor, ceiling bool
		records        map[prismcells.RegionKey]struct{}
	}
	points := map[[2]float64]*point{}
	for i, cell := range sc.profiles {
		for _, ring := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
			for _, e := range ring {
				if err := b.st.budget.Step(); err != nil {
					return err
				}
				key, _, ok := reach(e.Entity)
				if !ok || len(e.Polyline) == 0 {
					return errUBMiss
				}
				keys := []prismcells.RegionKey{key}
				if c, shared := coincident.Lookup(e.Entity, e.TStart, e.TEnd); shared {
					// The span's partner record passes here too.
					partner, _, ok := reach(c.Partner)
					if !ok {
						return errUBMiss
					}
					keys = append(keys, partner)
				}
				for _, at := range [][2]float64{e.Polyline[0], e.Polyline[len(e.Polyline)-1]} {
					pt := points[at]
					if pt == nil {
						pt = &point{records: map[prismcells.RegionKey]struct{}{}}
						points[at] = pt
					}
					pt.floor = pt.floor || exposed[i] > 0
					pt.ceiling = pt.ceiling || exposed[i] < 0
					for _, k := range keys {
						pt.records[k] = struct{}{}
					}
				}
			}
		}
	}
	for _, pt := range points {
		if !pt.floor || !pt.ceiling {
			continue
		}
		var count [3]int
		for key := range pt.records {
			count[reachOf[key]]++
		}
		if count[reachBoth] != 0 || count[reachBelow] != 1 || count[reachAbove] != 1 {
			return errUBMiss
		}
	}
	turnTags := map[sketch.Entity]prismcells.Origin{}
	for e := range sc.tags {
		if _, r, _ := reach(e); r != reachBoth {
			turnTags[e] = prismcells.Origin{IsB: r == reachAbove}
		}
	}
	classes := make([]opposedClass, len(exposed))
	for i, x := range exposed {
		switch {
		case x > 0:
			classes[i] = opposedFloor
		case x < 0:
			classes[i] = opposedCeiling
		}
	}
	manifold, err := opposedManifold(b.st.budget, turnTags, coincident, sc.profiles, classes)
	if err != nil {
		return err
	}
	if !manifold {
		return errUBMiss
	}
	return nil
}

// ifaceSide is one record entering an interface scene and which side of the
// interface it reaches.
type ifaceSide struct {
	ref          ubRef
	below, above bool
}

// interfaceRefs is the record set of the interface at level k: each
// operand's records reaching the slab below or above it, a stacked operand's
// equal regions on both sides (sameRegion) entering once. ok is false when
// fewer than two records meet there, which has no interface scene.
func (b *ubBuild) interfaceRefs(k int) ([2][]ifaceSide, []ubRef, []ubRef, bool, error) {
	var sides [2][]ifaceSide
	add := func(ref ubRef, below bool) error {
		op := 0
		if ref.isB {
			op = 1
		}
		for i := range sides[op] {
			have := &sides[op][i]
			if have.ref == ref {
				have.below, have.above = have.below || below, have.above || !below
				return nil
			}
			equal, err := sameRegion(b.st.budget, b.record(have.ref), b.record(ref))
			if err != nil {
				return err
			}
			if equal {
				have.below, have.above = have.below || below, have.above || !below
				return nil
			}
		}
		sides[op] = append(sides[op], ifaceSide{ref: ref, below: below, above: !below})
		return nil
	}
	for op := range 2 {
		for _, ref := range b.refs(op == 1, b.reach[k-1][op]) {
			if err := add(ref, true); err != nil {
				return sides, nil, nil, false, err
			}
		}
		for _, ref := range b.refs(op == 1, b.reach[k][op]) {
			if err := add(ref, false); err != nil {
				return sides, nil, nil, false, err
			}
		}
	}
	if len(sides[0])+len(sides[1]) < 2 {
		return sides, nil, nil, false, nil
	}
	var refsA, refsB []ubRef
	for _, sd := range sides[0] {
		refsA = append(refsA, sd.ref)
	}
	for _, sd := range sides[1] {
		refsB = append(refsB, sd.ref)
	}
	return sides, refsA, refsB, true, nil
}
