package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// This file is docs/general-boolean-design.md §3 A1's brep result: a
// stacked union whose interface the clean-nesting match leaves unresolved — a
// boss flush with the plate's wall, or crossing its outline — stated as a
// brepPayload (§4) from the slabs tryStackedUnion already built.
//
// Every 2D answer is a private scene's: a slab both operands reach arranges
// every record reaching it and its regions are the select-all merge's loops;
// each interface arranges every record reaching the slab below or above it
// and prismcells.ClassifyRegions names the cells on each side, so the
// exposed floors and ceilings are cells, recorded as sketch returned them.
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
	st     *stackedUnionState
	levels []stackedUnionLevel
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
}

// errUBMiss marks a topology this build does not cover, found after a scene
// was built: the caller treats it as a silent miss.
var errUBMiss = brepgeom.ErrStackedWallMiss

// brep states the stacked union as a brepPayload. ok=false with a nil error
// is a silent miss. A non-nil error is cancellation or a refusal past the
// gate that the stacked path itself raises.
func (st *stackedUnionState) brep(ctx context.Context, levels []stackedUnionLevel, reach [][2]int) (featurePayload, bool, error) {
	if !st.reexpress.Identity || st.va.proxy.sectionDelta != 0 || st.vb.proxy.sectionDelta != 0 {
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
		held[i] = level.held
	}
	b := &ubBuild{st: st, levels: levels, reach: reach,
		scenes: map[string]*ubScene{}, geom: stackedbrep.NewEngine(held)}
	bp, err := b.run(ctx)
	if errors.Is(err, errUBMiss) || errors.Is(err, ErrUnsupported) {
		return nil, false, nil
	}
	if err != nil {
		return nil, false, err
	}
	return bp, true, nil
}

func (b *ubBuild) run(ctx context.Context) (brepPayload, error) {
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
			loop, err := b.geom.LoopOf(brepgeom.Profile{Outer: region.Outer, Holes: region.Holes}, b.st.budget)
			if err != nil {
				return brepPayload{}, err
			}
			loops = append(loops, loop)
		}
		b.slabs = append(b.slabs, regions)
		b.geom.SlabLoops = append(b.geom.SlabLoops, loops)
	}
	for _, loop := range b.geom.SlabLoops[0] {
		if err := b.geom.AddFace(loop, 0, false); err != nil {
			return brepPayload{}, err
		}
	}
	for _, loop := range b.geom.SlabLoops[n-1] {
		if err := b.geom.AddFace(loop, n, true); err != nil {
			return brepPayload{}, err
		}
	}
	for k := 1; k < n; k++ {
		if err := b.interfaceFaces(ctx, k); err != nil {
			return brepPayload{}, err
		}
	}
	b.geom.RecordJunctions(n)
	if b.walk != 0 || b.crossing != 0 {
		// A walked endpoint or an amplified crossing moves a carrier, not
		// just a vertex on it; only a swept face carries that as a band.
		return brepPayload{}, errUBMiss
	}
	// Every vertex lies on exact carriers: an axis-aligned plane at its
	// recorded level, or a circle about its recorded centre. A cut vertex
	// sits within delta of the crossing it denotes along those carriers, so
	// each planar face's region and each swept piece's pinned arc is within
	// delta of the face it denotes: the merges' cut charge plus the largest
	// allowance of any canonical vertex, charged to every face alike.
	delta := proofbound.AbsSumUpper(b.cutDelta, b.geom.Allow())
	out, err := stackedBrepRecord(ctx, b.st.budget, b.geom, b.levels, b.st.va.proxy.frame, b.st.va.proxy.xform, delta)
	if err != nil {
		return brepPayload{}, err
	}
	// The result keeps its slabs, so a further co-directional Union reads
	// it as an operand (§4.1).
	stack := &brepStack{delta: delta}
	for k, regions := range b.slabs {
		lo, hi := b.levels[k], b.levels[k+1]
		stack.slabs = append(stack.slabs, prismSlab{regions: regions,
			z0: lo.held, z1: hi.held, z0Delta: lo.delta, z1Delta: hi.delta})
	}
	out.stack = stack
	return out, nil
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
	out := make([]ubRef, len(op.slabs[slab].regions))
	for r := range out {
		out[r] = ubRef{isB: isB, slab: slab, region: r}
	}
	return out
}

func (b *ubBuild) record(ref ubRef) profileRecord {
	if ref.isB {
		return b.st.vb.slabs[ref.slab].regions[ref.region]
	}
	return b.st.va.slabs[ref.slab].regions[ref.region]
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
	segments, within, err := prismcells.RegionsWithinWorkCap(b.st.budget,
		append(append([]profileRecord{}, regionsA...), regionsB...)...)
	if err != nil {
		return nil, err
	}
	if !within {
		return nil, fmt.Errorf(
			`%w: the analytic union scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1)`,
			ErrUnsupported, segments, prismcells.MaxArrangementSegments)
	}
	s, tags, delta, err := buildPrismSceneRegions(b.st.budget, regionsA, regionsB, &prismReexpression{Identity: true})
	if err != nil {
		return nil, err
	}
	if err := b.st.budget.Err(); err != nil {
		return nil, err
	}
	profiles, err := prismCellProfiles(ctx, b.st.budget, s)
	if err != nil {
		return nil, err
	}
	if len(profiles) == 0 {
		return nil, errUBMiss
	}
	b.walk = math.Max(b.walk, math.Max(delta.A, delta.B))
	sc := &ubScene{refsA: refsA, refsB: refsB, profiles: profiles, tags: tags, delta: delta}
	b.scenes[key] = sc
	return sc, nil
}

// charge runs A6's crossing charge once over the scene's cells and reads
// the shared spans' width into the build's crossing term.
func (b *ubBuild) charge(sc *ubScene) error {
	if !sc.charged {
		ok, err := sc.delta.ChargeCrossings(b.st.budget, sc.tags, sc.profiles, b.st.va.proxy.sectionDelta, b.st.vb.proxy.sectionDelta, 0)
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
// check, A5's several disjoint loops, §6's audit per loop).
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
	loops, cutDelta, resolved, err := prismcells.MergeLoops(b.st.budget, sc.profiles, "union")
	if err != nil {
		return nil, err
	}
	if !resolved {
		return nil, errUBMiss
	}
	b.cutDelta = math.Max(b.cutDelta, cutDelta)
	out := make([]profileRecord, len(loops))
	for i, loop := range loops {
		area, err := loopSignedAreaCB(loop)
		if err != nil {
			return nil, err
		}
		if !(area > 0) {
			// A clockwise loop would be a hole of a merged region, which the
			// void check already refuses; nothing here owns one.
			return nil, errUBMiss
		}
		out[i] = profileRecord{Outer: loop}
		if err := auditPrismMergeSection(b.st.budget, prismPayload{profile: out[i]}, out[i]); err != nil {
			return nil, err
		}
	}
	return out, nil
}

// interfaceFaces classifies the interface at level k: the scene of every
// record reaching the slab below or the slab above it, each cell's
// membership in every record read by prismcells.ClassifyRegions, and the
// cells inside some record of one side and none of the other recorded as
// that side's exposed faces. A stacked operand's equal regions on both sides
// enter once.
func (b *ubBuild) interfaceFaces(ctx context.Context, k int) error {
	type side struct {
		ref          ubRef
		below, above bool
	}
	var sides [2][]side
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
			equal, err := loopRecordsEqual(b.st.budget, b.record(have.ref).Outer, b.record(ref).Outer)
			if err != nil {
				return err
			}
			if equal {
				have.below, have.above = have.below || below, have.above || !below
				return nil
			}
		}
		sides[op] = append(sides[op], side{ref: ref, below: below, above: !below})
		return nil
	}
	for op := range 2 {
		for _, ref := range b.refs(op == 1, b.reach[k-1][op]) {
			if err := add(ref, true); err != nil {
				return err
			}
		}
		for _, ref := range b.refs(op == 1, b.reach[k][op]) {
			if err := add(ref, false); err != nil {
				return err
			}
		}
	}
	if len(sides[0])+len(sides[1]) < 2 {
		return nil
	}
	var refsA, refsB []ubRef
	for _, sd := range sides[0] {
		refsA = append(refsA, sd.ref)
	}
	for _, sd := range sides[1] {
		refsB = append(refsB, sd.ref)
	}
	sc, err := b.scene(ctx, refsA, refsB)
	if err != nil {
		return err
	}
	if err := b.charge(sc); err != nil {
		return err
	}
	if sc.matter == nil {
		matter, resolved, err := prismcells.ClassifyRegions(b.st.budget, sc.tags, sc.profiles)
		if err != nil {
			return err
		}
		if !resolved {
			return errUBMiss
		}
		sc.matter = matter
	}
	for i, p := range sc.profiles {
		below, above := false, false
		for op := range 2 {
			for _, sd := range sides[op] {
				key, ok := sc.key(sd.ref)
				if !ok {
					return errUBMiss
				}
				in := sc.matter[key][i]
				below = below || (sd.below && in)
				above = above || (sd.above && in)
			}
		}
		if below == above {
			continue
		}
		region, err := prismRecordArrangedProfileContext(ctx, p)
		if fallBack, err := prismcells.AmplifiedFallback(sc.delta.Amplified, err); fallBack || err != nil {
			if err != nil {
				return err
			}
			return errUBMiss
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

// stackedBrepRecord states an engine's slabs and horizontal faces as a
// brepPayload in reference frame ref under xform, every face carrying the
// section displacement delta: the horizontal faces, every curved or oblique
// wall piece (stackedBrepSweptFaces) and every axis-aligned wall plane
// (stackedBrepWallFaces). Each planar face record then faces modify §5's audit
// (simplicity, orientation and nesting; general-boolean §5), roles are
// assigned, and the record must close by counting (§4.2). geom has read
// every slab loop and horizontal face and recorded its junctions. A topology
// the engine does not cover is brepgeom.ErrStackedWallMiss.
func stackedBrepRecord(ctx context.Context, budget *proofbound.WorkBudget, geom *stackedbrep.Engine, levels []stackedUnionLevel, ref r3.Frame, xform r3.Transform, delta float64) (brepPayload, error) {
	faceLevels := make([]stackedbrep.Level, len(levels))
	for i, level := range levels {
		faceLevels[i] = stackedbrep.Level{Held: level.held, Delta: level.delta}
	}
	faces, err := geom.AssembleFaces(faceLevels, ref, delta)
	if err != nil {
		return brepPayload{}, err
	}
	out := brepPayload{xform: xform}
	for _, f := range faces {
		var region *profileRecord
		if f.Region != nil {
			p := profileRecord{Outer: f.Region.Outer, Holes: f.Region.Holes}
			region = &p
		}
		out.faces = append(out.faces, brepFace{
			frame: f.Frame, region: region, outward: f.Outward, sweep: f.Sweep, wall: f.Wall,
			z0: f.Z0, z1: f.Z1, z0Delta: f.Z0Delta, z1Delta: f.Z1Delta,
			side0: f.Side0, side1: f.Side1, delta: f.Delta,
		})
	}
	// Modify §5's audit per planar face record (general-boolean §5):
	// simplicity, orientation and nesting. It refuses; it admits nothing.
	for _, f := range out.faces {
		if f.region == nil {
			continue
		}
		if err := auditPrismMergeSection(budget, prismPayload{profile: *f.region}, *f.region); err != nil {
			return brepPayload{}, err
		}
	}
	out.assignRoles()
	// The record must close by counting (§4.2).
	if _, err := brepTopologyContext(ctx, out); err != nil {
		return brepPayload{}, err
	}
	return out, nil
}
