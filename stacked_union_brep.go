package decad

import (
	"context"
	"errors"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// This file is docs/general-boolean-design.md §3 A1's brep result: a
// stacked union whose interface the clean-nesting match leaves unresolved — a
// boss flush with the plate's wall, or crossing its outline — stated as a
// brepPayload (§4) from the slabs tryStackedUnion already built.
//
// Every 2D answer is a private scene's: each interface arranges the two
// operand records that reach it and prismcells.Classify names the cells on
// each side, so the exposed floors and ceilings are cells, recorded as sketch
// returned them. decad then only restates what the scenes recorded: every
// junction becomes one canonical vertex (a line's exact level, or the one
// float the keyed table holds for a line crossing a circle, as class B's
// §10 table does), every segment is rewritten between its vertices, and every
// straight wall on one axis-aligned plane becomes one planar face in its own
// frame — the rectangles each slab's segment sweeps, with their shared edges
// cancelled — while every curved or oblique wall is a swept piece between its
// vertices. A pair this build does not cover is a silent miss: the caller
// takes the mesh path with no error, as prism-boolean §4.4 states for an
// unresolved topology.

// ubKind names what a segment's carrier is.
const (
	ubPlane  = iota // an axis-aligned line: the plane axis == level, axis 0 for U and 1 for V
	ubLine          // an oblique line, keyed by its two recorded endpoints
	ubCircle        // a circle or arc, keyed by its centre and radius
)

// ubCarrier identifies the surface a segment lies on, from the segment's
// recorded data alone: a line by its recorded endpoints (an axis-aligned
// one by its plane), a circle or arc by its centre and radius.
type ubCarrier struct {
	kind  int
	axis  int
	level float64
	a, b  Point2
	r     float64
}

// ubEnd is one walked end of a segment and how far it can sit from the point
// it denotes: the walk's own rounding, plus the cut displacement when the
// parameter there is one the arrangement computed.
type ubEnd struct {
	p     Point2
	allow float64
}

// ubUnit is one maximal run of a loop's consecutive segments on one carrier,
// walked in one sense, between its two canonical vertices.
type ubUnit struct {
	c        ubCarrier
	ccw      bool
	from, to Point2
}

// ubLoop is a loop restated as units; closed marks one whole circle.
type ubLoop struct {
	units  []ubUnit
	closed bool
}

// ubFace is one horizontal planar face before its record is written: a
// loop at a level, facing up (outward) or down.
type ubFace struct {
	loop    ubLoop
	level   int
	outward bool
}

// ubVertex is one canonical vertex of the section: the carriers it lies on
// and its proven displacement from the point it denotes.
type ubVertex struct {
	carriers []ubCarrier
	delta    float64
}

// ubKey keys a crossing of two carriers; side tells the two crossings of a
// line with a circle apart.
type ubKey struct {
	c1, c2 ubCarrier
	side   int
}

type ubEntry struct {
	p     Point2
	delta float64
}

type ubWallKey = brepgeom.StackedWallKey
type ubSeg3 = brepgeom.StackedWallSegment

// ubPieceKey names one swept piece of a curved or oblique wall by its
// carrier, its two vertices and its sense.
type ubPieceKey struct {
	c        ubCarrier
	from, to Point2
	ccw      bool
	closed   bool
}

// ubBuild is one brep build over the stacked union's slabs.
type ubBuild struct {
	st     *stackedUnionState
	slabs  []prismSlab
	levels []stackedUnionLevel
	reach  [][2]int
	// levelAt maps a held level to its index.
	levelAt map[float64]int
	table   map[ubKey]ubEntry
	verts   map[Point2]*ubVertex
	// events lists, per vertex, the levels at which it is a vertex of the
	// body: a horizontal face's loop vertex, or a level where the wall
	// junction there starts, ends or changes carriers.
	events map[Point2]map[int]struct{}
	// junctions records, per vertex and slab, the two carriers meeting there.
	junctions map[Point2]map[int][2]ubCarrier
	faces     []ubFace
	slabLoops []ubLoop
	allow     float64
}

// errUBMiss marks a topology this build does not cover, found after a scene
// was built: the caller treats it as a silent miss.
var errUBMiss = brepgeom.ErrStackedWallMiss

// brep states the stacked union as a brepPayload. ok=false with a nil error
// is a silent miss. A non-nil error is cancellation or a refusal past the
// gate that the stacked path itself raises.
func (st *stackedUnionState) brep(ctx context.Context, slabs []prismSlab, levels []stackedUnionLevel, reach [][2]int) (featurePayload, bool, error) {
	if !st.reexpress.identity || st.va.proxy.sectionDelta != 0 || st.vb.proxy.sectionDelta != 0 {
		// B's records enter every scene verbatim only under the identity
		// re-expression; a re-expressed B would be recorded once per scene.
		// Every straight wall becomes a planar face at its recorded level,
		// which is the true wall only when neither operand carries a
		// section displacement (class B's B5 rule): a planar face states no
		// band for a wall that moved.
		return nil, false, nil
	}
	b := &ubBuild{st: st, slabs: slabs, levels: levels, reach: reach,
		levelAt:   map[float64]int{},
		table:     map[ubKey]ubEntry{},
		verts:     map[Point2]*ubVertex{},
		events:    map[Point2]map[int]struct{}{},
		junctions: map[Point2]map[int][2]ubCarrier{},
	}
	for i, l := range levels {
		b.levelAt[l.held] = i
	}
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
	for _, slab := range b.slabs {
		loop, err := b.loopOf(slab.regions[0])
		if err != nil {
			return brepPayload{}, err
		}
		b.slabLoops = append(b.slabLoops, loop)
	}
	n := len(b.slabs)
	if err := b.addFace(b.slabLoops[0], 0, false); err != nil {
		return brepPayload{}, err
	}
	if err := b.addFace(b.slabLoops[n-1], n, true); err != nil {
		return brepPayload{}, err
	}
	for k := 1; k < n; k++ {
		if err := b.interfaceFaces(ctx, k); err != nil {
			return brepPayload{}, err
		}
	}
	b.recordJunctions()
	if b.st.walkA != 0 || b.st.walkB != 0 || b.st.crossing != 0 {
		// A walked endpoint or an amplified crossing moves a carrier, not
		// just a vertex on it; only a swept face carries that as a band.
		return brepPayload{}, errUBMiss
	}
	// Every vertex lies on exact carriers: an axis-aligned plane at its
	// recorded level, or a circle about its recorded centre. A cut vertex
	// sits within delta of the crossing it denotes along those carriers, so
	// each planar face's region and each swept piece's pinned arc is within
	// delta of the face it denotes: the merge's cut charge plus the largest
	// allowance of any canonical vertex, charged to every face alike.
	delta := proofbound.AbsSumUpper(b.st.sectionDelta(), b.allow)

	out := brepPayload{xform: b.st.va.proxy.xform}
	ref := b.st.va.proxy.frame
	for _, f := range b.faces {
		region, err := b.faceRecord(f)
		if err != nil {
			return brepPayload{}, err
		}
		l := b.levels[f.level]
		out.faces = append(out.faces, brepFace{frame: ref, region: &region, outward: f.outward,
			z0: l.held, z1: l.held, z0Delta: l.delta, z1Delta: l.delta, delta: delta})
	}
	swept, err := b.sweptFaces(delta)
	if err != nil {
		return brepPayload{}, err
	}
	out.faces = append(out.faces, swept...)
	walls, err := b.wallFaces(delta)
	if err != nil {
		return brepPayload{}, err
	}
	out.faces = append(out.faces, walls...)
	// Modify §5's audit per planar face record (general-boolean §5):
	// simplicity, orientation and nesting. It refuses; it admits nothing.
	for _, f := range out.faces {
		if f.region == nil {
			continue
		}
		if err := auditPrismMergeSection(b.st.budget, prismPayload{profile: *f.region}, *f.region); err != nil {
			return brepPayload{}, err
		}
	}
	out.assignRoles()
	// The record must close by counting (§4.2); a record this build left
	// unpaired is an uncovered topology, not a refusal.
	if _, err := brepTopologyContext(ctx, out); err != nil {
		return brepPayload{}, err
	}
	return out, nil
}

// carrierOf reads a segment's carrier from its record and walk.
func ubCarrierOf(seg CurveSegment, w survey2d.SegmentWalk) (ubCarrier, error) {
	switch s := seg.(type) {
	case LineSeg:
		switch {
		case s.Start.U == s.End.U && s.Start.V == s.End.V:
			return ubCarrier{}, errUBMiss
		case s.Start.U == s.End.U:
			return ubCarrier{kind: ubPlane, axis: 0, level: s.Start.U + 0}, nil
		case s.Start.V == s.End.V:
			return ubCarrier{kind: ubPlane, axis: 1, level: s.Start.V + 0}, nil
		}
		a, c := s.Start, s.End
		if c.U < a.U || (c.U == a.U && c.V < a.V) {
			a, c = c, a
		}
		return ubCarrier{kind: ubLine, a: a, b: c}, nil
	case CircleSeg, ArcSeg:
		if !w.IsCircular() {
			return ubCarrier{}, errUBMiss
		}
		return ubCarrier{kind: ubCircle, a: Point2{U: w.CU + 0, V: w.CV + 0}, r: w.Radius}, nil
	}
	return ubCarrier{}, errUBMiss
}

// ubEnds reads a segment's two walked ends with their allowances.
func ubEnds(seg CurveSegment, w survey2d.SegmentWalk) (ubEnd, ubEnd, error) {
	t0, t1, err := ubRange(seg)
	if err != nil {
		return ubEnd{}, ubEnd{}, err
	}
	cut := 0.0
	if (t0 != 0 && t0 != 1) || (t1 != 0 && t1 != 1) {
		speed, err := prismcells.CarrierSpeedUpper(seg)
		if err != nil {
			return ubEnd{}, ubEnd{}, err
		}
		cut = proofbound.CutDisplacementAllow(speed)
	}
	start := ubEnd{p: Point2{U: w.StartU + 0, V: w.StartV + 0}, allow: proofbound.WalkEndBoundAllow(w.StartBound)}
	end := ubEnd{p: Point2{U: w.EndU + 0, V: w.EndV + 0}, allow: proofbound.WalkEndBoundAllow(w.EndBound)}
	if t0 != 0 && t0 != 1 {
		start.allow = proofbound.AbsSumUpper(start.allow, cut)
	}
	if t1 != 0 && t1 != 1 {
		end.allow = proofbound.AbsSumUpper(end.allow, cut)
	}
	return start, end, nil
}

func ubRange(seg CurveSegment) (float64, float64, error) {
	switch s := seg.(type) {
	case LineSeg:
		return s.TStart, s.TEnd, nil
	case CircleSeg:
		return s.TStart, s.TEnd, nil
	case ArcSeg:
		return s.TStart, s.TEnd, nil
	}
	return 0, 0, errUBMiss
}

// ubLess orders carriers so a pair keys the table one way.
func ubLess(a, b ubCarrier) bool {
	keys := func(c ubCarrier) [8]float64 {
		return [8]float64{float64(c.kind), float64(c.axis), c.level, c.a.U, c.a.V, c.b.U, c.b.V, c.r}
	}
	ka, kb := keys(a), keys(b)
	for i := range ka {
		if ka[i] != kb[i] {
			return ka[i] < kb[i]
		}
	}
	return false
}

func ubPair(a, b ubCarrier) [2]ubCarrier {
	if ubLess(b, a) {
		return [2]ubCarrier{b, a}
	}
	return [2]ubCarrier{a, b}
}

// loopOf restates one recorded loop as units between canonical vertices.
func (b *ubBuild) loopOf(region ProfileRecord) (ubLoop, error) {
	if len(region.Holes) != 0 {
		return ubLoop{}, errUBMiss
	}
	segs := region.Outer.Segments
	n := len(segs)
	if n == 0 {
		return ubLoop{}, errUBMiss
	}
	carriers := make([]ubCarrier, n)
	starts := make([]ubEnd, n)
	ends := make([]ubEnd, n)
	ccw := make([]bool, n)
	dir := make([]Point2, n)
	for i, seg := range segs {
		if err := b.st.budget.Step(); err != nil {
			return ubLoop{}, err
		}
		w, err := walkOf(seg, nil)
		if err != nil {
			return ubLoop{}, err
		}
		if err := requireAnalyticWalk(w, "a stacked union face"); err != nil {
			return ubLoop{}, err
		}
		carriers[i], err = ubCarrierOf(seg, w)
		if err != nil {
			return ubLoop{}, err
		}
		starts[i], ends[i], err = ubEnds(seg, w)
		if err != nil {
			return ubLoop{}, err
		}
		ccw[i] = w.Th1 > w.Th0
		dir[i] = Point2{U: w.EndU - w.StartU, V: w.EndV - w.StartV}
	}
	// Consecutive segments on one carrier are one unit: a circle cut at its
	// own seam, or a line run restated at other cells' vertices.
	same := func(i, j int) bool {
		if carriers[i] != carriers[j] {
			return false
		}
		if carriers[i].kind == ubCircle {
			return ccw[i] == ccw[j]
		}
		return dir[i].U*dir[j].U+dir[i].V*dir[j].V > 0
	}
	start := -1
	for i := range n {
		if !same((i+n-1)%n, i) {
			start = i
			break
		}
	}
	if start < 0 {
		// One carrier round the whole loop: a whole circle, possibly cut
		// at its seam. A straight loop closes nothing.
		if carriers[0].kind != ubCircle {
			return ubLoop{}, errUBMiss
		}
		return ubLoop{closed: true, units: []ubUnit{{c: carriers[0], ccw: ccw[0]}}}, nil
	}
	type group struct {
		first, last int
	}
	var groups []group
	for k := range n {
		i := (start + k) % n
		if k > 0 && same((i+n-1)%n, i) {
			groups[len(groups)-1].last = i
			continue
		}
		groups = append(groups, group{first: i, last: i})
	}
	if len(groups) < 2 {
		return ubLoop{}, errUBMiss
	}
	units := make([]ubUnit, len(groups))
	for gi, g := range groups {
		units[gi] = ubUnit{c: carriers[g.first], ccw: ccw[g.first]}
	}
	for gi := range groups {
		gj := (gi + 1) % len(groups)
		p, delta, err := b.junction(units[gi].c, ends[groups[gi].last], units[gj].c, starts[groups[gj].first])
		if err != nil {
			return ubLoop{}, err
		}
		units[gi].to, units[gj].from = p, p
		b.record(p, delta, units[gi].c, units[gj].c)
	}
	return ubLoop{units: units}, nil
}

// junction is the one canonical vertex where carriers ci and cj meet, near
// the two walked ends the loop states there. Two axis-aligned lines meet at
// their exact levels. A line crossing a circle takes the keyed table's one
// float for that crossing — the first scene to reach the key records the
// line's walked point, whose fixed coordinate is exact, with its allowance —
// keyed by the side of the circle's centre the crossing lies on; a crossing
// too close to the centre's coordinate to decide its side misses. Two
// circles crossing have no exact record and miss.
func (b *ubBuild) junction(ci ubCarrier, ei ubEnd, cj ubCarrier, sj ubEnd) (Point2, float64, error) {
	if ci == cj {
		return Point2{}, 0, errUBMiss
	}
	if ci.kind == ubPlane && cj.kind == ubPlane {
		if ci.axis == cj.axis {
			return Point2{}, 0, errUBMiss
		}
		if ci.axis == 0 {
			return Point2{U: ci.level, V: cj.level}, 0, nil
		}
		return Point2{U: cj.level, V: ci.level}, 0, nil
	}
	if ci.kind == ubCircle && cj.kind == ubCircle {
		return Point2{}, 0, errUBMiss
	}
	// Prefer the straight carrier's own walked point: an axis-aligned
	// line's fixed coordinate is exact.
	hint, line, curve := ei, ci, cj
	if ci.kind == ubCircle || (ci.kind == ubLine && cj.kind == ubPlane) {
		hint, line, curve = sj, cj, ci
	}
	p := hint.p
	if line.kind == ubPlane {
		if line.axis == 0 {
			p.U = line.level
		} else {
			p.V = line.level
		}
	}
	side := 0
	if curve.kind == ubCircle {
		var diff, scale float64
		switch line.kind {
		case ubPlane:
			if line.axis == 0 {
				diff = p.V - curve.a.V
			} else {
				diff = p.U - curve.a.U
			}
			scale = 1
		default:
			du, dv := line.b.U-line.a.U, line.b.V-line.a.V
			diff = du*(p.V-curve.a.V) - dv*(p.U-curve.a.U)
			scale = math.Abs(du) + math.Abs(dv)
		}
		if !(math.Abs(diff) > 2*hint.allow*scale) {
			return Point2{}, 0, errUBMiss
		}
		side = 1
		if diff < 0 {
			side = -1
		}
	}
	pair := ubPair(ci, cj)
	key := ubKey{c1: pair[0], c2: pair[1], side: side}
	entry, ok := b.table[key]
	if !ok {
		entry = ubEntry{p: p, delta: hint.allow}
		b.table[key] = entry
	}
	b.allow = math.Max(b.allow, entry.delta)
	return entry.p, entry.delta, nil
}

// record registers a canonical vertex with the carriers it was found on.
func (b *ubBuild) record(p Point2, delta float64, carriers ...ubCarrier) {
	v, ok := b.verts[p]
	if !ok {
		v = &ubVertex{}
		b.verts[p] = v
	}
	v.delta = math.Max(v.delta, delta)
	for _, c := range carriers {
		if !slices.Contains(v.carriers, c) {
			v.carriers = append(v.carriers, c)
		}
	}
}

func (b *ubBuild) event(p Point2, level int) {
	if b.events[p] == nil {
		b.events[p] = map[int]struct{}{}
	}
	b.events[p][level] = struct{}{}
}

func (b *ubBuild) hasEvent(p Point2, level int) bool {
	_, ok := b.events[p][level]
	return ok
}

// addFace keeps one horizontal face and marks its vertices at its level.
func (b *ubBuild) addFace(loop ubLoop, level int, outward bool) error {
	b.faces = append(b.faces, ubFace{loop: loop, level: level, outward: outward})
	if loop.closed {
		return nil
	}
	for _, u := range loop.units {
		b.event(u.from, level)
	}
	return nil
}

// interfaceFaces classifies the interface at level k: the scene of the two
// distinct operand records reaching the slabs below and above it, each
// cell's membership read by prismcells.Classify, and the cells on one side
// only recorded as that side's exposed faces. More than two distinct
// records at one level is a scene this build does not arrange.
func (b *ubBuild) interfaceFaces(ctx context.Context, k int) error {
	lo, hi := b.reach[k-1], b.reach[k]
	var refs []stackedUnionRegionRef
	var inLo, inHi []bool
	add := func(ref stackedUnionRegionRef, below, above bool) error {
		for i, have := range refs {
			if have == ref {
				inLo[i] = inLo[i] || below
				inHi[i] = inHi[i] || above
				return nil
			}
			if have.isB == ref.isB {
				equal, err := loopRecordsEqual(b.st.budget, b.st.region(have).Outer, b.st.region(ref).Outer)
				if err != nil {
					return err
				}
				if equal {
					inLo[i] = inLo[i] || below
					inHi[i] = inHi[i] || above
					return nil
				}
			}
		}
		refs = append(refs, ref)
		inLo = append(inLo, below)
		inHi = append(inHi, above)
		return nil
	}
	for op := range 2 {
		isB := op == 1
		if lo[op] >= 0 {
			if err := add(stackedUnionRegionRef{isB: isB, slab: lo[op]}, true, false); err != nil {
				return err
			}
		}
		if hi[op] >= 0 {
			if err := add(stackedUnionRegionRef{isB: isB, slab: hi[op]}, false, true); err != nil {
				return err
			}
		}
	}
	switch len(refs) {
	case 1:
		return nil
	case 2:
	default:
		return errUBMiss
	}
	m, err := b.st.scene(ctx, refs[0], refs[1])
	if err != nil {
		return err
	}
	if len(m.profiles) == 0 {
		return errUBMiss
	}
	if ok, err := m.charge(b.st.budget, b.st.view(refs[0]), b.st.view(refs[1]), b.st.reexpress); err != nil || !ok {
		if err != nil {
			return err
		}
		return errUBMiss
	}
	b.st.crossing = math.Max(b.st.crossing, m.sceneDelta.crossing)
	matterX, matterY, resolved, err := prismcells.Classify(b.st.budget, m.tags, m.profiles)
	if err != nil {
		return err
	}
	if !resolved {
		return errUBMiss
	}
	for i, p := range m.profiles {
		below := (inLo[0] && matterX[i]) || (inLo[1] && matterY[i])
		above := (inHi[0] && matterX[i]) || (inHi[1] && matterY[i])
		if below == above {
			continue
		}
		region, err := prismRecordArrangedProfileContext(ctx, p)
		if fallBack, err := prismAmplifiedFallback(m.sceneDelta.amplified, err); fallBack || err != nil {
			if err != nil {
				return err
			}
			return errUBMiss
		}
		loop, err := b.loopOf(region)
		if err != nil {
			return err
		}
		if err := b.addFace(loop, k, below); err != nil {
			return err
		}
	}
	return nil
}

// recordJunctions reads every slab loop's junctions, then marks the levels
// where a junction starts, ends or changes carriers as vertex events.
func (b *ubBuild) recordJunctions() {
	for k, loop := range b.slabLoops {
		if loop.closed {
			continue
		}
		for i, u := range loop.units {
			if b.junctions[u.from] == nil {
				b.junctions[u.from] = map[int][2]ubCarrier{}
			}
			prev := loop.units[(i+len(loop.units)-1)%len(loop.units)]
			b.junctions[u.from][k] = ubPair(prev.c, u.c)
		}
	}
	n := len(b.slabs)
	for p, bySlab := range b.junctions {
		for level := 0; level <= n; level++ {
			below, hasBelow := bySlab[level-1]
			above, hasAbove := bySlab[level]
			if hasBelow != hasAbove || (hasBelow && below != above) {
				b.event(p, level)
			}
		}
	}
}

// cutsOnLine lists the vertices at level lying strictly inside a plane
// unit, in walk order. Position along the line is the free coordinate,
// compared exactly.
func (b *ubBuild) cutsOnLine(u ubUnit, level int) []Point2 {
	free := func(p Point2) float64 {
		if u.c.axis == 0 {
			return p.V
		}
		return p.U
	}
	lo, hi := free(u.from), free(u.to)
	forward := lo < hi
	if !forward {
		lo, hi = hi, lo
	}
	var cuts []Point2
	for p, v := range b.verts {
		if !slices.Contains(v.carriers, u.c) || !b.hasEvent(p, level) {
			continue
		}
		if f := free(p); f > lo && f < hi {
			cuts = append(cuts, p)
		}
	}
	slices.SortFunc(cuts, func(a, c Point2) int { return cmpFloat(free(a), free(c)) })
	if !forward {
		slices.Reverse(cuts)
	}
	return cuts
}

// circlePoints lists a circle unit's vertices in walk order: for an open
// unit its two ends with every vertex strictly inside its sweep, for a whole
// circle every vertex on it from the first back to itself. Two vertices
// within 1e-9 rad are not ordered by their held points and miss, as class
// B's split does (general-boolean §5).
func (b *ubBuild) circlePoints(u ubUnit) ([]Point2, error) {
	const tol = 1e-9
	c := u.c.a
	angle := func(p Point2) float64 { return math.Atan2(p.V-c.V, p.U-c.U) }
	sweep := func(from, to float64) float64 {
		d := to - from
		if !u.ccw {
			d = -d
		}
		for d < 0 {
			d += 2 * math.Pi
		}
		for d >= 2*math.Pi {
			d -= 2 * math.Pi
		}
		return d
	}
	type cut struct {
		p Point2
		s float64
	}
	var cuts []cut
	a0, total := 0.0, 2*math.Pi
	closed := u.from == u.to
	if !closed {
		a0 = angle(u.from)
		total = sweep(a0, angle(u.to))
	}
	for p, v := range b.verts {
		if !slices.Contains(v.carriers, u.c) {
			continue
		}
		if !closed && (p == u.from || p == u.to) {
			continue
		}
		s := sweep(a0, angle(p))
		if !closed {
			if s >= total+tol {
				continue
			}
			if s <= tol || s >= total-tol {
				return nil, errUBMiss
			}
		}
		cuts = append(cuts, cut{p: p, s: s})
	}
	slices.SortFunc(cuts, func(x, y cut) int { return cmpFloat(x.s, y.s) })
	for i := 1; i < len(cuts); i++ {
		if cuts[i].s-cuts[i-1].s <= tol {
			return nil, errUBMiss
		}
	}
	if closed && len(cuts) > 1 && 2*math.Pi-cuts[len(cuts)-1].s+cuts[0].s <= tol {
		return nil, errUBMiss
	}
	var points []Point2
	if !closed {
		points = append(points, u.from)
	}
	for _, k := range cuts {
		points = append(points, k.p)
	}
	if closed {
		if len(points) > 0 {
			points = append(points, points[0])
		}
		return points, nil
	}
	return append(points, u.to), nil
}

// ubArc is the recorded piece of a circle carrier between two points, in a
// sense; a whole circle when from and to coincide with no vertex between.
func ubArc(c ubCarrier, from, to Point2, ccw bool) CurveSegment {
	return arcSegment(c.a, from, to, ccw)
}

func ubWholeCircle(c ubCarrier, ccw bool) CurveSegment {
	seg := CircleSeg{Center: c.a, Radius: units.Millimeters(c.r), CCW: ccw, TStart: 0, TEnd: 1}
	if !ccw {
		seg.TStart, seg.TEnd = 1, 0
	}
	return seg
}

// unitSegments writes a unit's record pieces: a plane line split at the
// vertices at level, an oblique line whole, a circle split at every vertex
// on it.
func (b *ubBuild) unitSegments(u ubUnit, closed bool, level int) ([]CurveSegment, error) {
	switch u.c.kind {
	case ubPlane:
		points := append([]Point2{u.from}, b.cutsOnLine(u, level)...)
		points = append(points, u.to)
		var out []CurveSegment
		for i := 0; i+1 < len(points); i++ {
			out = append(out, LineSeg{Start: points[i], End: points[i+1], TStart: 0, TEnd: 1})
		}
		return out, nil
	case ubLine:
		for p, v := range b.verts {
			if slices.Contains(v.carriers, u.c) && p != u.from && p != u.to {
				return nil, errUBMiss
			}
		}
		return []CurveSegment{LineSeg{Start: u.from, End: u.to, TStart: 0, TEnd: 1}}, nil
	}
	points, err := b.circlePoints(u)
	if err != nil {
		return nil, err
	}
	if closed && len(points) == 0 {
		return []CurveSegment{ubWholeCircle(u.c, u.ccw)}, nil
	}
	var out []CurveSegment
	for i := 0; i+1 < len(points); i++ {
		out = append(out, ubArc(u.c, points[i], points[i+1], u.ccw))
	}
	return out, nil
}

// faceRecord writes one horizontal face's region.
func (b *ubBuild) faceRecord(f ubFace) (ProfileRecord, error) {
	var loop LoopRecord
	for _, u := range f.loop.units {
		segs, err := b.unitSegments(u, f.loop.closed, f.level)
		if err != nil {
			return ProfileRecord{}, err
		}
		loop.Segments = append(loop.Segments, segs...)
	}
	return ProfileRecord{Outer: loop}, nil
}

// sweptFaces builds every curved and oblique wall piece: each slab's unit
// split at its vertices, the identical piece in consecutive slabs joined
// into one face unless a vertex at the level between ends an edge at either
// of its two side lines, which a swept face cannot carry.
func (b *ubBuild) sweptFaces(delta float64) ([]brepFace, error) {
	type run struct {
		seg      CurveSegment
		from, to Point2
		closed   bool
		k0, k1   int
	}
	var runs []*run
	byKey := map[ubPieceKey]*run{}
	for k, loop := range b.slabLoops {
		for _, u := range loop.units {
			if u.c.kind == ubPlane {
				continue
			}
			segs, err := b.unitSegments(u, loop.closed, k)
			if err != nil {
				return nil, err
			}
			for _, seg := range segs {
				w, err := walkOf(seg, nil)
				if err != nil {
					return nil, err
				}
				from, to := Point2{U: w.StartU + 0, V: w.StartV + 0}, Point2{U: w.EndU + 0, V: w.EndV + 0}
				key := ubPieceKey{c: u.c, from: from, to: to, ccw: u.ccw, closed: w.Closed}
				if r, ok := byKey[key]; ok && r.k1 == k-1 {
					if !w.Closed && (b.hasEvent(from, k) || b.hasEvent(to, k)) {
						return nil, errUBMiss
					}
					r.k1 = k
					continue
				}
				r := &run{seg: seg, from: from, to: to, closed: w.Closed, k0: k, k1: k}
				runs = append(runs, r)
				byKey[key] = r
			}
		}
	}
	ref := b.st.va.proxy.frame
	out := make([]brepFace, 0, len(runs))
	for _, r := range runs {
		lo, hi := b.levels[r.k0], b.levels[r.k1+1]
		out = append(out, brepFace{frame: ref, wall: r.seg, z0: lo.held, z1: hi.held,
			z0Delta: lo.delta, z1Delta: hi.delta, delta: delta})
	}
	return out, nil
}

// wallFaces builds one planar face per axis-aligned carrier plane and
// material side: every slab's segment on it sweeps a rectangle whose edges
// are split at the body's vertices, the edges two rectangles share cancel,
// and the rest chain into the face's loops.
func (b *ubBuild) wallFaces(delta float64) ([]brepFace, error) {
	pieces := map[ubWallKey]map[ubSeg3]struct{}{}
	var order []ubWallKey
	add := func(key ubWallKey, from, to [3]float64) error {
		set, ok := pieces[key]
		if !ok {
			set = map[ubSeg3]struct{}{}
			pieces[key] = set
			order = append(order, key)
		}
		if _, dup := set[ubSeg3{From: from, To: to}]; dup {
			return errUBMiss
		}
		if _, rev := set[ubSeg3{From: to, To: from}]; rev {
			delete(set, ubSeg3{From: to, To: from})
			return nil
		}
		set[ubSeg3{From: from, To: to}] = struct{}{}
		return nil
	}
	for k, loop := range b.slabLoops {
		z0, z1 := b.levels[k].held, b.levels[k+1].held
		for _, u := range loop.units {
			if u.c.kind != ubPlane {
				continue
			}
			key := ubWallKey{Axis: u.c.axis, Level: u.c.level}
			if u.c.axis == 0 {
				key.Sign = 1
				if u.to.V < u.from.V {
					key.Sign = -1
				}
			} else {
				key.Sign = -1
				if u.to.U < u.from.U {
					key.Sign = 1
				}
			}
			at := func(p Point2, z float64) [3]float64 { return [3]float64{p.U + 0, p.V + 0, z} }
			bottom := append([]Point2{u.from}, b.cutsOnLine(u, k)...)
			bottom = append(bottom, u.to)
			for i := 0; i+1 < len(bottom); i++ {
				if err := add(key, at(bottom[i], z0), at(bottom[i+1], z0)); err != nil {
					return nil, err
				}
			}
			if err := add(key, at(u.to, z0), at(u.to, z1)); err != nil {
				return nil, err
			}
			top := append([]Point2{u.from}, b.cutsOnLine(u, k+1)...)
			top = append(top, u.to)
			for i := len(top) - 1; i > 0; i-- {
				if err := add(key, at(top[i], z1), at(top[i-1], z1)); err != nil {
					return nil, err
				}
			}
			if err := add(key, at(u.from, z1), at(u.from, z0)); err != nil {
				return nil, err
			}
		}
	}
	var out []brepFace
	for _, key := range order {
		frame, embed, err := brepgeom.StackedWallFrame(b.st.va.proxy.frame, key)
		if err != nil {
			return nil, err
		}
		loops, err := brepgeom.ChainStackedWall(pieces[key], b.levelAt, b.events)
		if err != nil {
			return nil, err
		}
		for _, loop := range loops {
			wallRegion, level, err := brepgeom.StackedWallRegion(embed, loop)
			if err != nil {
				return nil, err
			}
			region := ProfileRecord{Outer: wallRegion.Outer, Holes: wallRegion.Holes}
			out = append(out, brepFace{frame: frame, region: &region, outward: true,
				z0: level, z1: level, delta: delta})
		}
	}
	return out, nil
}
