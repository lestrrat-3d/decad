package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is brepPayload — the analytically trimmed face record of
// docs/general-boolean-design.md §4 — together with the face view that states a
// prism or a stacked prism in it, the record audit, and the body build:
// topology and roles (§4.2). brep_measure.go owns the measurements (§4.3) and
// tessellate_brep.go the mesh and its occupied-volume proof (§4.4).

// brepFace is one face of a brepPayload. Exactly one of region and wall is
// set.
//
// A planar face (region set) lies in frame's plane at level z0 along
// frame.N(); z1 equals z0 and z1Delta equals z0Delta. Its region is stated in
// frame coordinates with the material on the left of every loop's walk (outer
// counter-clockwise, holes clockwise, as a section records them), and outward
// is true when its outward normal is +frame.N().
//
// A swept face (wall set) is the wall segment, stated in frame coordinates and
// walked with the material on its left, swept along frame.N() over [z0, z1].
//
// delta is the face's own section displacement and z0Delta/z1Delta its levels'
// displacements (docs/prism-boolean-design.md §7); role is the face's role,
// face(k) or wall(k) for its index k in the record.
type brepFace struct {
	frame            r3.Frame
	region           *ProfileRecord
	outward          bool
	wall             CurveSegment
	z0, z1           float64
	z0Delta, z1Delta float64
	delta            float64
	role             string
}

// brepPayload is the evaluator's record of an analytically trimmed body. Every
// face frame is stated in the payload's unplaced coordinates and xform places
// the whole body, as every payload does.
type brepPayload struct {
	faces []brepFace
	xform r3.Transform
}

func (f brepFace) planar() bool { return f.region != nil }

// view is the prism the face's own frame, levels and displacements describe.
// A planar face's view carries its whole region at a zero height; a swept
// face's carries its one wall segment as an open single-segment loop, which
// only the per-segment readings (extents, witnesses) consume.
func (f brepFace) view(xform r3.Transform) prismPayload {
	pp := prismPayload{
		frame: f.frame, xform: xform,
		z0: f.z0, z1: f.z1, z0Delta: f.z0Delta, z1Delta: f.z1Delta,
		sectionDelta: f.delta,
	}
	if f.planar() {
		pp.profile = *f.region
	} else {
		pp.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{f.wall}}}
	}
	return pp
}

func (bp brepPayload) transform() r3.Transform { return bp.xform }

// placed re-evaluates the record under the composed motion. A reflection is
// read at build time, as prismPayload.reflected() is, so the record itself is
// unchanged.
func (bp brepPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	bp.xform = composed
	return evalBrepContext(ctx, d, ref, bp)
}

// axialDelta is the largest level displacement over every face.
func (bp brepPayload) axialDelta() float64 {
	var out float64
	for _, f := range bp.faces {
		out = max(out, f.z0Delta, f.z1Delta)
	}
	return out
}

// sectionDelta is the largest section displacement over every face.
func (bp brepPayload) sectionDelta() float64 {
	var out float64
	for _, f := range bp.faces {
		out = max(out, f.delta)
	}
	return out
}

// requireNotBrepReceiver is modify-reach Table RX's RX7 and Table SX's SX16:
// Fillet, Chamfer and Shell refuse a brep receiver with ErrUnsupported. The
// refusal is staged, not SX9's permanent exclusion: the faces are analytic
// carriers with recorded trims, and rewriting a planar face's region and
// re-trimming its walls is not built yet (docs/general-boolean-design.md
// §4.5).
func requireNotBrepReceiver(payload featurePayload, op string) error {
	if _, ok := payload.(brepPayload); ok {
		return fmt.Errorf(`%w: this evaluator does not yet rewrite an analytically trimmed (brep) body's faces; it %s a straight prism only (modify-reach SX16)`, ErrUnsupported, op)
	}
	return nil
}

// assignRoles names every face by its index in the record: face(k) for a
// planar face and wall(k) for a swept one (§4.2).
func (bp brepPayload) assignRoles() {
	for k := range bp.faces {
		if bp.faces[k].planar() {
			bp.faces[k].role = fmt.Sprintf("face(%d)", k)
			continue
		}
		bp.faces[k].role = fmt.Sprintf("wall(%d)", k)
	}
}

// brepOfPrism is a prism's face view (§4.1): one swept face per recorded
// segment, then the start and end caps. It is built on demand and never stored
// for a prism. A surface result has no closed face view and is ErrUnsupported.
func brepOfPrism(pp prismPayload) (brepPayload, error) {
	if pp.surfaceResult {
		return brepPayload{}, fmt.Errorf(`%w: a surface-result prism has no closed face view`, ErrUnsupported)
	}
	profile, allow, err := brepJoinProfile(pp.profile)
	if err != nil {
		return brepPayload{}, err
	}
	pp.profile = profile
	if allow > 0 {
		pp.sectionDelta = proofbound.AbsSumUpper(pp.sectionDelta, allow)
	}
	bp := brepPayload{xform: pp.xform}
	for _, loop := range append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		for _, seg := range loop.Segments {
			bp.faces = append(bp.faces, brepFace{
				frame: pp.frame, wall: seg, z0: pp.z0, z1: pp.z1,
				z0Delta: pp.z0Delta, z1Delta: pp.z1Delta, delta: pp.sectionDelta,
			})
		}
	}
	region := pp.profile
	bp.faces = append(bp.faces,
		brepFace{frame: pp.frame, region: &region, z0: pp.z0, z1: pp.z0,
			z0Delta: pp.z0Delta, z1Delta: pp.z0Delta, delta: pp.sectionDelta},
		brepFace{frame: pp.frame, region: &region, outward: true, z0: pp.z1, z1: pp.z1,
			z0Delta: pp.z1Delta, z1Delta: pp.z1Delta, delta: pp.sectionDelta},
	)
	bp.assignRoles()
	return bp, nil
}

// brepOfStacked is a stacked prism's face view (§4.1): one swept face per
// segment of every loop column (docs/stacked-prism-design.md §2.3), then the
// two caps and every interface's exposed floors and ceilings. The record is
// audited first, so a stack its own build refuses has no face view either.
func brepOfStacked(ctx context.Context, sp stackedPrismPayload) (brepPayload, error) {
	if err := falsifyStackedPayload(ctx, sp); err != nil {
		return brepPayload{}, err
	}
	sp, err := brepJoinStacked(sp)
	if err != nil {
		return brepPayload{}, err
	}
	columns, _, err := stackedColumns(sp)
	if err != nil {
		return brepPayload{}, err
	}
	bp := brepPayload{xform: sp.xform}
	for _, col := range columns {
		first, last := sp.slabs[col.start], sp.slabs[col.end]
		for _, seg := range col.loop.Segments {
			bp.faces = append(bp.faces, brepFace{
				frame: sp.frame, wall: seg, z0: first.z0, z1: last.z1,
				z0Delta: first.z0Delta, z1Delta: last.z1Delta, delta: sp.sectionDelta,
			})
		}
	}
	addPlanar := func(region ProfileRecord, z, zDelta float64, outward bool) {
		bp.faces = append(bp.faces, brepFace{frame: sp.frame, region: &region, outward: outward,
			z0: z, z1: z, z0Delta: zDelta, z1Delta: zDelta, delta: sp.sectionDelta})
	}
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	addPlanar(first.regions[0], first.z0, first.z0Delta, false)
	addPlanar(last.regions[0], last.z1, last.z1Delta, true)
	for k, boundary := range sp.interfaces {
		z, zDelta := sp.slabs[k].z1, sp.slabs[k].z1Delta
		for _, region := range boundary.lowerExposed {
			addPlanar(region, z, zDelta, true)
		}
		for _, region := range boundary.upperExposed {
			addPlanar(region, z, zDelta, false)
		}
	}
	bp.assignRoles()
	return bp, nil
}

// brepJoinLoop makes every junction of a loop one point. A boolean's cut
// fragment records its carrier and a narrowed range, and the two fragments
// meeting at a cut walk to that cut at two different floats — a line's lerp
// and a circle's cosine — so the brep's pairing by record identity (§4.2)
// would not meet them. Each junction takes one of the two: a line's point,
// whose fixed coordinate the lerp keeps exact, else the lexicographically
// smaller, so a loop and its reversal choose alike. Every segment is then
// rewritten between its two junctions: a line whole, a circular fragment as
// an arc pinned there about its recorded centre. Both walked points sit
// within the record's own section displacement of the crossing they denote,
// plus their walk's rounding, so the rewrite moves the boundary by at most
// that rounding beyond the record's displacement; allow is its largest
// value, zero when every junction already met. A whole closed segment is left
// alone.
func brepJoinLoop(loop LoopRecord) (LoopRecord, float64, error) {
	n := len(loop.Segments)
	if n < 2 {
		return loop, 0, nil
	}
	for _, seg := range loop.Segments {
		switch seg.(type) {
		case LineSeg, CircleSeg, ArcSeg:
		default:
			// A free-form loop has no brep face (falsifyBrepPayload refuses it),
			// so it has nothing to join.
			return loop, 0, nil
		}
	}
	walks := make([]survey2d.SegmentWalk, n)
	for i, seg := range loop.Segments {
		w, err := walkOf(seg, nil)
		if err != nil {
			return LoopRecord{}, 0, err
		}
		walks[i] = w
	}
	joins := make([]Point2, n)
	allow := 0.0
	met := true
	for i := range n {
		j := (i + 1) % n
		end := Point2{U: walks[i].EndU, V: walks[i].EndV}
		start := Point2{U: walks[j].StartU, V: walks[j].StartV}
		if end == start {
			joins[i] = end
			continue
		}
		met = false
		pick, bound := end, walks[i].EndBound
		switch {
		case walks[j].IsLine() && !walks[i].IsLine():
			pick, bound = start, walks[j].StartBound
		case walks[i].IsLine() && !walks[j].IsLine():
		case start.U < end.U || (start.U == end.U && start.V < end.V):
			pick, bound = start, walks[j].StartBound
		}
		joins[i] = pick
		allow = math.Max(allow, proofbound.WalkEndBoundAllow(bound))
	}
	if met {
		return loop, 0, nil
	}
	out := LoopRecord{Segments: make([]CurveSegment, n)}
	for i, w := range walks {
		from, to := joins[(i+n-1)%n], joins[i]
		if w.IsLine() {
			out.Segments[i] = LineSeg{Start: from, End: to, TStart: 0, TEnd: 1}
			continue
		}
		out.Segments[i] = arcSegment(Point2{U: w.CU, V: w.CV}, from, to, w.Th1 > w.Th0)
	}
	return out, allow, nil
}

// brepJoinProfile is brepJoinLoop over every loop of a region.
func brepJoinProfile(p ProfileRecord) (ProfileRecord, float64, error) {
	outer, allow, err := brepJoinLoop(p.Outer)
	if err != nil {
		return ProfileRecord{}, 0, err
	}
	out := ProfileRecord{Outer: outer}
	for _, hole := range p.Holes {
		joined, a, err := brepJoinLoop(hole)
		if err != nil {
			return ProfileRecord{}, 0, err
		}
		out.Holes = append(out.Holes, joined)
		allow = math.Max(allow, a)
	}
	return out, allow, nil
}

// brepJoinStacked is brepJoinLoop over every region and exposed record of a
// stack, charging the largest allow to its section displacement. Equal loops
// join alike, so the columns stackedColumns derives are unchanged.
func brepJoinStacked(sp stackedPrismPayload) (stackedPrismPayload, error) {
	allow := 0.0
	join := func(p ProfileRecord) (ProfileRecord, error) {
		out, a, err := brepJoinProfile(p)
		allow = math.Max(allow, a)
		return out, err
	}
	out := sp
	out.slabs = make([]prismSlab, len(sp.slabs))
	for k, slab := range sp.slabs {
		out.slabs[k] = slab
		out.slabs[k].regions = make([]ProfileRecord, len(slab.regions))
		for r, region := range slab.regions {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.slabs[k].regions[r] = joined
		}
	}
	out.interfaces = make([]prismSlabInterface, len(sp.interfaces))
	for k, boundary := range sp.interfaces {
		for _, region := range boundary.lowerExposed {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.interfaces[k].lowerExposed = append(out.interfaces[k].lowerExposed, joined)
		}
		for _, region := range boundary.upperExposed {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.interfaces[k].upperExposed = append(out.interfaces[k].upperExposed, joined)
		}
	}
	if allow > 0 {
		out.sectionDelta = proofbound.AbsSumUpper(sp.sectionDelta, allow)
	}
	return out, nil
}

// brepEmbed maps one face frame's local axes onto the reference frame's: local
// axis i is reference axis axis[i] scaled by sign[i]. Every sign is ±1, so the
// map moves a float coordinate exactly in both directions.
type brepEmbed = brepgeom.Embed

// brepEmbeds states every face frame in the first face's frame, the reference
// every reading is taken in. A face frame must share the reference origin bit
// for bit and carry each reference axis, or its negation, bit for bit as one of
// its own, with the map keeping handedness. Such a map is a signed permutation,
// so every coordinate and level carries over exactly and the record's records
// compare by identity (§4.2). Any other frame is ErrUnsupported: this build has
// no exact map for it.
func brepEmbeds(faces []brepFace) ([]brepEmbed, error) {
	frames := make([]r3.Frame, len(faces))
	for fi, f := range faces {
		frames[fi] = f.frame
	}
	return brepgeom.Embeds(frames, ErrUnsupported)
}

// falsifyBrepPayload refuses a record no brep body matches (ErrDegenerate) or
// one this evaluator does not build (ErrUnsupported): a face with neither or
// both of a region and a wall, a non-finite level or displacement, an empty
// sweep interval, a planar face whose two levels differ, a role that is empty
// or repeated, or a segment other than a line, a circle or an arc (§3 B2). It
// runs before topology and before chording, and repairs nothing.
func falsifyBrepPayload(ctx context.Context, bp brepPayload) error {
	if len(bp.faces) == 0 {
		return fmt.Errorf(`%w: a brep payload holds no face`, ErrDegenerate)
	}
	roles := make(map[string]struct{}, len(bp.faces))
	finite := func(values ...float64) bool {
		for _, v := range values {
			if math.IsNaN(v) || math.IsInf(v, 0) {
				return false
			}
		}
		return true
	}
	for fi, f := range bp.faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		if !f.frame.IsValid() {
			return fmt.Errorf(`%w: brep face %d has no valid frame`, ErrDegenerate, fi)
		}
		if f.planar() == (f.wall != nil) {
			return fmt.Errorf(`%w: brep face %d must carry exactly one of a region and a wall`, ErrDegenerate, fi)
		}
		if !finite(f.z0, f.z1, f.z0Delta, f.z1Delta, f.delta) || f.z0Delta < 0 || f.z1Delta < 0 || f.delta < 0 {
			return fmt.Errorf(`%w: brep face %d has a non-finite level or a negative displacement`, ErrDegenerate, fi)
		}
		if f.role == "" {
			return fmt.Errorf(`%w: brep face %d has no role`, ErrDegenerate, fi)
		}
		if _, seen := roles[f.role]; seen {
			return fmt.Errorf(`%w: brep role %q names two faces`, ErrDegenerate, f.role)
		}
		roles[f.role] = struct{}{}
		var segs []CurveSegment
		if f.planar() {
			if f.z0 != f.z1 || f.z0Delta != f.z1Delta {
				return fmt.Errorf(`%w: planar brep face %d has two levels`, ErrDegenerate, fi)
			}
			for _, loop := range append([]LoopRecord{f.region.Outer}, f.region.Holes...) {
				if len(loop.Segments) == 0 {
					return fmt.Errorf(`%w: planar brep face %d has an empty loop`, ErrDegenerate, fi)
				}
				segs = append(segs, loop.Segments...)
			}
		} else {
			if !(f.z0 < f.z1) {
				return fmt.Errorf(`%w: swept brep face %d has an empty interval`, ErrDegenerate, fi)
			}
			segs = []CurveSegment{f.wall}
		}
		for _, seg := range segs {
			seg, err := normalizeSegment(seg)
			if err != nil {
				return err
			}
			switch seg.(type) {
			case LineSeg, CircleSeg, ArcSeg:
			default:
				return fmt.Errorf(`%w: a brep face carries lines, circles and arcs only, not %T`, ErrUnsupported, seg)
			}
		}
	}
	return nil
}

// brepPart names which boundary piece of its face an edge use is.
type brepPart = brepgeom.Part

const (
	brepLoopSeg = brepgeom.LoopSeg // a planar face's loop segment
	brepRim0    = brepgeom.Rim0    // a swept face's wall at z0
	brepRim1    = brepgeom.Rim1    // a swept face's wall at z1
	brepSide0   = brepgeom.Side0   // a swept face's line at the wall's start
	brepSide1   = brepgeom.Side1   // a swept face's line at the wall's end
)

// brepEdgeKey is an edge's identity in reference coordinates (§4.2): a line by
// its two endpoints in sorted order; an arc by its centre, normal axis and its
// counter-clockwise start and end about that axis; a whole circle by its
// centre, normal axis and radius. Two uses with one key are one edge.
type brepEdgeKey = brepgeom.EdgeKey

// brepUse is one face's use of one edge. from/to run the way the face's own
// loop walks the use; dirFrom/dirTo run the use's natural way — a wall's walk
// for a rim or a loop segment, bottom to top for a side line. sense and
// dirSense are the same two readings of a circular use as counter-clockwise
// senses about its reference axis, which is all a whole circle has.
type brepUse struct {
	face, loop, seg int
	part            brepPart
	key             brepEdgeKey
	from, to        [3]float64
	dirFrom, dirTo  [3]float64
	sense, dirSense bool
	walk            survey2d.SegmentWalk
	level           float64
	levelDelta      float64
}

// brepTopology is the shared reading of a record that the body build and the
// tessellator both take: each face's walks, every edge use, and how the uses
// pair into edges.
type brepTopology struct {
	embeds []brepEmbed
	// walls holds a swept face's walk, planar holds each planar face's walks
	// per region loop.
	walls   map[int]survey2d.SegmentWalk
	planar  map[int][][]survey2d.SegmentWalk
	uses    []brepUse
	keyUses []brepgeom.Use
	// edges lists the two use indices of every edge, the owner first: a rim use
	// where the edge has one, then a side line, then a loop segment. The
	// owner's natural direction is the edge's direction.
	edges [][2]int
	// edgeOf maps a use index to its edge index.
	edgeOf []int
	// faceUses lists each face's use indices in loop order: a swept face's
	// rim0, side1, rim1, side0 (a whole circle's rim0 and rim1 alone); a
	// planar face's loops in record order.
	faceUses [][]int
	// coordUpper is the largest |u|+|v| over every walk and |z| over every
	// level: the input magnitude the frame and placement lift rounds against.
	coordUpper float64
	// regions caches each planar face's integrated region (brepTopology.region).
	regions map[int]brepRegion
}

// brepTopologyContext walks every face, keys every edge use, and pairs them.
// An edge that does not bound exactly two distinct faces refuses: the build
// proves closure by counting (§4.2), so an unpaired use is a record this
// evaluator cannot close and is ErrUnsupported.
func brepTopologyContext(ctx context.Context, bp brepPayload) (*brepTopology, error) {
	if err := falsifyBrepPayload(ctx, bp); err != nil {
		return nil, err
	}
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		return nil, err
	}
	topo := &brepTopology{
		embeds: embeds,
		walls:  map[int]survey2d.SegmentWalk{},
		planar: map[int][][]survey2d.SegmentWalk{},
	}
	topo.faceUses = make([][]int, len(bp.faces))
	work := freeform.NewFreeformWork()
	walk := func(seg CurveSegment) (survey2d.SegmentWalk, error) {
		w, err := walkOf(seg, work)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
		if err := requireAnalyticWalk(w, "a brep face"); err != nil {
			return survey2d.SegmentWalk{}, err
		}
		topo.coordUpper = math.Max(topo.coordUpper, w.CoordUpper)
		return w, nil
	}
	add := func(u brepUse) {
		topo.faceUses[u.face] = append(topo.faceUses[u.face], len(topo.uses))
		topo.uses = append(topo.uses, u)
	}
	for fi, f := range bp.faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		e := embeds[fi]
		topo.coordUpper = math.Max(topo.coordUpper, math.Max(math.Abs(f.z0), math.Abs(f.z1)))
		if f.planar() {
			loops := append([]LoopRecord{f.region.Outer}, f.region.Holes...)
			walks := make([][]survey2d.SegmentWalk, len(loops))
			for li, loop := range loops {
				for si, seg := range loop.Segments {
					w, err := walk(seg)
					if err != nil {
						return nil, err
					}
					walks[li] = append(walks[li], w)
					u := brepUse{face: fi, loop: li, seg: si, part: brepLoopSeg, walk: w,
						level: f.z0, levelDelta: f.z0Delta}
					u.key, u.sense = brepCurveKey(e, w, f.z0)
					u.dirSense = u.sense
					u.from, u.to = e.Canon(w.StartU, w.StartV, f.z0), e.Canon(w.EndU, w.EndV, f.z0)
					u.dirFrom, u.dirTo = u.from, u.to
					add(u)
				}
			}
			topo.planar[fi] = walks
			continue
		}
		w, err := walk(f.wall)
		if err != nil {
			return nil, err
		}
		topo.walls[fi] = w
		s0, s1 := e.Canon(w.StartU, w.StartV, f.z0), e.Canon(w.StartU, w.StartV, f.z1)
		t0, t1 := e.Canon(w.EndU, w.EndV, f.z0), e.Canon(w.EndU, w.EndV, f.z1)
		rim := func(part brepPart, z, zDelta float64, from, to, dirFrom, dirTo [3]float64, reversed bool) brepUse {
			u := brepUse{face: fi, loop: -1, seg: -1, part: part, walk: w, level: z, levelDelta: zDelta,
				from: from, to: to, dirFrom: dirFrom, dirTo: dirTo}
			u.key, u.sense = brepCurveKey(e, w, z)
			u.dirSense = u.sense
			if reversed {
				u.sense = !u.sense
			}
			return u
		}
		add(rim(brepRim0, f.z0, f.z0Delta, s0, t0, s0, t0, false))
		if w.Closed {
			add(rim(brepRim1, f.z1, f.z1Delta, s1, s1, s1, s1, true))
			continue
		}
		add(brepUse{face: fi, loop: -1, seg: -1, part: brepSide1, key: brepLineKey(t0, t1),
			from: t0, to: t1, dirFrom: t0, dirTo: t1})
		add(rim(brepRim1, f.z1, f.z1Delta, t1, s1, s1, t1, true))
		add(brepUse{face: fi, loop: -1, seg: -1, part: brepSide0, key: brepLineKey(s0, s1),
			from: s1, to: s0, dirFrom: s0, dirTo: s1})
	}
	topo.keyUses = make([]brepgeom.Use, len(topo.uses))
	for ui, u := range topo.uses {
		topo.keyUses[ui] = brepgeom.Use{Face: u.face, Loop: u.loop, Part: u.part, Key: u.key,
			From: u.from, To: u.to, DirFrom: u.dirFrom, DirTo: u.dirTo,
			Sense: u.sense, DirSense: u.dirSense, Walk: u.walk}
	}
	topo.edges, topo.edgeOf, err = brepgeom.Pair(topo.keyUses, ErrUnsupported)
	if err != nil {
		return nil, err
	}
	return topo, nil
}

// forward reports whether use ui walks its edge in the edge's own direction,
// which is its owner use's natural one. A whole circle compares senses: it has
// no endpoints to compare.
func (topo *brepTopology) forward(ui int) bool {
	return brepgeom.Forward(topo.keyUses, topo.edges, topo.edgeOf, ui)
}

func brepIsRim(p brepPart) bool { return brepgeom.IsRim(p) }

// brepLineKey keys a line edge by its two endpoints in sorted order.
func brepLineKey(a, b [3]float64) brepEdgeKey {
	return brepgeom.LineKey(a, b)
}

// brepCurveKey keys one walk at one level and reports its counter-clockwise
// sense about the reference axis the walk's own normal lands on. A walk
// counter-clockwise in its frame turns about +frame.N(), and that axis lands
// on the reference axis with sign e.Sign[2]; the map keeps handedness, so the
// sense flips exactly when the sign is negative.
func brepCurveKey(e brepEmbed, w survey2d.SegmentWalk, z float64) (brepEdgeKey, bool) {
	return brepgeom.CurveKey(e, w, z)
}

// evalBrepContext builds the body a brepPayload records (§4.2, §4.3). Every
// edge is shared by exactly two faces, identified by record identity in the
// reference frame, so the body closes by construction and the count proves it.
// Roles are the record's own face(k)/wall(k) under ref; no capStart or capEnd
// is minted and no operand provenance is carried.
func evalBrepContext(ctx context.Context, d *Document, ref producerID, bp brepPayload) (*Body, error) {
	topo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	refView := bp.refView()
	frameLift := proofbound.FrameAndPlacementRoundAllow(refView.frame, refView.xform, topo.coordUpper)

	// Vertices are shared by reference coordinates. Each carries the largest
	// displacement any use placing it states: its face's section displacement,
	// its level's, the frame and placement lift, and its walk end's own bound.
	vertices := map[[3]float64]*Vertex{}
	vertexAt := func(c [3]float64, bound float64) *Vertex {
		v, ok := vertices[c]
		if !ok {
			v = &Vertex{position: refView.point(c[0], c[1], c[2])}
			vertices[c] = v
		}
		if bound > v.bound.Base() {
			v.bound = units.Millimeters(bound)
		}
		return v
	}
	for _, u := range topo.uses {
		// A whole circle's seam is its rim's: a loop that walks the same
		// circle from another seam places no vertex of its own.
		if (u.part != brepLoopSeg && !brepIsRim(u.part)) || (u.part == brepLoopSeg && u.key.Closed) {
			continue
		}
		f := bp.faces[u.face]
		base := proofbound.AbsSumUpper(f.delta, u.levelDelta, frameLift)
		vertexAt(u.dirFrom, proofbound.AbsSumUpper(base, proofbound.WalkEndBoundAllow(u.walk.StartBound)))
		vertexAt(u.dirTo, proofbound.AbsSumUpper(base, proofbound.WalkEndBoundAllow(u.walk.EndBound)))
	}

	edges := make([]*Edge, len(topo.edges))
	for ei, pair := range topo.edges {
		edge, err := brepEdge(ctx, bp, topo, pair, vertexAt)
		if err != nil {
			return nil, err
		}
		edges[ei] = edge
	}
	coedgeOf := func(ui int) coedge {
		return coedge{edge: edges[topo.edgeOf[ui]], forward: topo.forward(ui)}
	}

	faces := make([]*Face, len(bp.faces))
	for fi, f := range bp.faces {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := brepFaceBody(ctx, bp, topo, fi, body, ref)
		if err != nil {
			return nil, err
		}
		uses := topo.faceUses[fi]
		switch {
		case f.planar():
			loops := make([]*Loop, len(topo.planar[fi]))
			for li := range loops {
				loops[li] = &Loop{outer: li == 0}
			}
			for _, ui := range uses {
				l := loops[topo.uses[ui].loop]
				l.coedges = append(l.coedges, coedgeOf(ui))
			}
			face.loops = loops
		case len(uses) == 2:
			face.loops = []*Loop{
				{coedges: []coedge{coedgeOf(uses[0])}, outer: true},
				{coedges: []coedge{coedgeOf(uses[1])}, outer: true},
			}
		default:
			l := &Loop{outer: true}
			for _, ui := range uses {
				l.coedges = append(l.coedges, coedgeOf(ui))
			}
			face.loops = []*Loop{l}
		}
		faces[fi] = face
	}
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}
	body.lumps = sheetLumps(faces)
	if err := measureBrepContext(ctx, bp, topo, body); err != nil {
		return nil, err
	}
	body.payload = bp
	return body, nil
}

// refView is the prism view of the reference frame (the first face's) under
// the placement: the one map every vertex, centroid and mesh point is lifted
// through.
func (bp brepPayload) refView() prismPayload {
	return prismPayload{frame: bp.faces[0].frame, xform: bp.xform}
}

// brepEdge builds one edge from its owner use: a rim takes its wall's curve and
// walk length, a side line runs bottom to top over the wall's bounded height,
// and a loop segment shared by two planar faces is a line of its walk's
// length. Convexity reads evaluator §3's walked boundary (brepEdgeConvex).
func brepEdge(ctx context.Context, bp brepPayload, topo *brepTopology, pair [2]int, vertexAt func([3]float64, float64) *Vertex) (*Edge, error) {
	owner := topo.uses[pair[0]]
	f := bp.faces[owner.face]
	start := vertexAt(owner.dirFrom, 0)
	end := vertexAt(owner.dirTo, 0)
	convex, err := brepEdgeConvex(ctx, topo, pair)
	if err != nil {
		return nil, err
	}
	switch owner.part {
	case brepSide0, brepSide1:
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(f.z1, f.z1Delta), proofbound.MeasuredScalar(f.z0, f.z0Delta))
		return &Edge{curve: Line3{}, start: start, end: end, convex: convex,
			length: f.z1 - f.z0, lengthBound: height.Bound}, nil
	case brepRim0, brepRim1:
		w := topo.walls[owner.face]
		pp := f.view(bp.xform)
		bottom, top, _, _, err := buildWallGeometry(pp, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}},
			convex, w.Closed, start, end, start, end)
		if err != nil {
			return nil, err
		}
		edge := bottom
		if owner.part == brepRim1 {
			edge = top
		}
		edge.lengthBound = proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.delta, 1))
		return edge, nil
	default:
		w := owner.walk
		return &Edge{curve: Line3{}, start: start, end: end, convex: convex, length: w.Length,
			lengthBound: proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(f.delta, 1))}, nil
	}
}

// brepEdgeConvex decides an edge's walked-boundary convexity (topology.go's
// Edge.IsConvex). A rim reads the wall it runs along: a circular wall by its
// own turn, a straight wall by the role of the loop that wall belongs to,
// which the planar face's loop states — its own role when it walks the rim the
// way the wall does, the other role when it walks it the opposite way (a
// stacked floor is a reversed hole). A side line between two walls is a
// junction: convex when the walk turns left there about the sweep axis. Any
// other line reads the first planar face's loop role: outer convex, hole
// concave.
func brepEdgeConvex(ctx context.Context, topo *brepTopology, pair [2]int) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	return brepgeom.Convex(pair, topo.keyUses, topo.embeds, topo.walls, ErrUnsupported)
}
