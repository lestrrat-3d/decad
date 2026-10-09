package decad

import (
	"context"
	"errors"
	"fmt"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// This file is route P of docs/brep-modify-design.md ("brep-modify §N"): a
// brep or stacked receiver that reads as a prism along a reference axis
// (§4.1, P1–P5) is handed to the op's own prism path as that prism (§4.2).
// Every test below is an exact comparison of recorded floats and records,
// moved between face frames by signed permutations; nothing is sampled,
// solved or admitted on a residual.

// prismCaps names the two cap faces a modify op classifies a prism selection
// against (brep-modify §4.2): start at the prism's z0 level, end at its z1
// level. startLoop and endLoop map a cap face's loop index to the section
// loop it bounds (0 the outer, 1+i hole i); nil reads as the identity. A nil
// face names no cap, and a selection on it classifies as no cap edge.
type prismCaps struct {
	start, end         *Face
	startLoop, endLoop []int
}

// prismCapsOf names a prism body's own capStart and capEnd faces.
func prismCapsOf(b *Body) prismCaps {
	roles := facesByRole(b)
	return prismCaps{start: roles[roleCapStart], end: roles[roleCapEnd]}
}

// sectionLoop maps one cap face's loop index to its section loop index.
func (c prismCaps) sectionLoop(start bool, li int) int {
	loops := c.endLoop
	if start {
		loops = c.startLoop
	}
	if loops == nil {
		return li
	}
	return loops[li]
}

// brepPrismRoute is brep-modify §4.2: axes 0, 1, 2 in order, the first whose
// prism reading (§4.1) the op's own classification admits is taken. The
// returned route carries that prism and its caps. When no axis admits, the
// route is empty and refusal is the first axis's classification refusal, or
// nil when no axis reads as a prism at all. err is a context error, or the
// refusal of a Shell of a record carrying route L chamfer bands.
//
// A record carrying route L bands (docs/modify-general-design.md §4) reads as
// no prism: a prism reading would read its faces alone and drop the band
// patches the body also holds, so a Fillet or Chamfer goes on to route E or
// route L over the record. A Shell of it refuses, since its erosion meets the
// band patches, which no through-cut record holds (modify-general SG3).
func brepPrismRoute(ctx context.Context, b *Body, bp brepPayload, req brepModifyRequest) (brepRoute, error, error) {
	if len(bp.loopBands) > 0 {
		if req.shell {
			return brepRoute{}, nil, fmt.Errorf(`%w: this evaluator shells no brep body carrying route L chamfer bands; their patches are oblique planes and cones no through-cut record holds (modify-general SG3)`, ErrUnsupported)
		}
		return brepRoute{}, nil, nil
	}
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		// A record the build took has embeds; one that has none reads as no
		// prism, and the caller's own refusal follows.
		return brepRoute{}, nil, nil //nolint:nilerr // no embeds is no prism reading
	}
	var refusal error
	for k := range 3 {
		read, ok, err := recognisePrism(ctx, bp, embeds, k)
		if err != nil {
			return brepRoute{}, nil, err
		}
		if !ok {
			continue
		}
		caps := read.caps(b, bp)
		admitErr := req.admits(read.pp, caps)
		if admitErr == nil {
			pp := read.pp
			return brepRoute{prism: &pp, caps: caps}, nil, nil
		}
		if ctxErr := ctx.Err(); ctxErr != nil {
			return brepRoute{}, nil, ctxErr
		}
		if errors.Is(admitErr, context.Canceled) || errors.Is(admitErr, context.DeadlineExceeded) {
			return brepRoute{}, nil, admitErr
		}
		if refusal == nil {
			refusal = admitErr
		}
	}
	return brepRoute{}, refusal, nil
}

// brepPrismRead is one prism reading of a brep record along a reference axis
// (brep-modify §4.1): the recognised prism, the indices of the record's
// bottom and top faces, and which section loop each of bottom's region loops
// is (top's region is the section, loop for loop).
type brepPrismRead struct {
	pp          prismPayload
	bottom, top int
	bottomLoop  []int
}

// caps names the reading's cap faces on the receiver body: bottom is the
// prism's start cap and top its end cap. A stacked receiver's faces carry the
// stacked body's roles, not its face view's, so it names no cap face.
func (r brepPrismRead) caps(b *Body, bp brepPayload) prismCaps {
	if _, ok := b.payload.(brepPayload); !ok {
		return prismCaps{}
	}
	roles := facesByRole(b)
	return prismCaps{
		start:     roles[bp.faces[r.bottom].role],
		end:       roles[bp.faces[r.top].role],
		startLoop: r.bottomLoop,
	}
}

// brepPrismWalls collects what P2 and P5 read from the record's other faces:
// every swept face along the axis as its wall re-expressed into F, every
// rectangle across it as its projection onto F's plane with the outward
// normal there, and the level displacements the walls state at zlo and zhi.
type brepPrismWalls struct {
	walls              []survey2d.SegmentWalk
	rects              []brepPrismRect
	zloDelta, zhiDelta float64
}

// brepPrismRect is one P2(b) or P2(c) rectangle projected along the axis onto
// F's plane: the segment's two ends and the outward normal, a unit vector
// along one of F's in-plane axes.
type brepPrismRect struct {
	p, q   Point2
	nu, nv float64
}

// brepPrismCaps is what P1, P3 and P4 read: the record's two cap faces across
// reference axis k, their levels, the prism frame F with its embed, the
// section S in F, and which section loop each of the bottom's region loops is.
type brepPrismCaps struct {
	bottom, top int
	zlo, zhi    float64
	frame       r3.Frame
	eF          brepEmbed
	section     ProfileRecord
	bottomLoop  []int
}

// readPrismCaps reads brep-modify §4.1's P1, P3 and P4 along reference axis
// k, or reports false. It reads no wall, so a caller that classifies the other
// faces itself (docs/modify-general-design.md Table TC) shares the cap reading
// with recognisePrism.
func readPrismCaps(bp brepPayload, embeds []brepEmbed, k int) (brepPrismCaps, bool) {
	// P1: exactly two planar faces across the axis, bottom facing −k and top
	// facing +k, with zlo < zhi.
	bottom, top := -1, -1
	planar := 0
	for fi, f := range bp.faces {
		if !f.planar() || embeds[fi].Axis[2] != k {
			continue
		}
		planar++
		if brepOutwardSign(f, embeds[fi]) < 0 {
			bottom = fi
		} else {
			top = fi
		}
	}
	if planar != 2 || bottom < 0 || top < 0 {
		return brepPrismCaps{}, false
	}
	zlo := brepLevel(bp.faces[bottom], embeds[bottom])
	zhi := brepLevel(bp.faces[top], embeds[top])
	if !(zlo < zhi) {
		return brepPrismCaps{}, false
	}

	// P3: the prism frame F.
	var frame r3.Frame
	var eF brepEmbed
	switch {
	case embeds[top].Sign[2] > 0:
		frame, eF = bp.faces[top].frame, embeds[top]
	case embeds[bottom].Sign[2] > 0:
		frame, eF = bp.faces[bottom].frame, embeds[bottom]
	default:
		var err error
		frame, eF, err = brepgeom.AxisFrame(bp.faces[0].frame, k)
		if err != nil {
			return brepPrismCaps{}, false // no exact F is no prism reading
		}
	}

	// P4: the section S is top's region in F; bottom's region in F equals it.
	section, ok := newBrepPlaneMap(embeds[top], eF).region(*bp.faces[top].region)
	if !ok {
		return brepPrismCaps{}, false
	}
	bottomRegion, ok := newBrepPlaneMap(embeds[bottom], eF).region(*bp.faces[bottom].region)
	if !ok {
		return brepPrismCaps{}, false
	}
	bottomLoop, ok := brepSameRegion(section, bottomRegion)
	if !ok {
		return brepPrismCaps{}, false
	}
	return brepPrismCaps{
		bottom: bottom, top: top, zlo: zlo, zhi: zhi,
		frame: frame, eF: eF, section: section, bottomLoop: bottomLoop,
	}, true
}

// classifyPrismWalls is brep-modify §4.1's P2 and P5 over the faces other than
// the caps: every other face is a wall along the axis or a rectangle across
// it, and each claims one segment of the section exactly once. It reports
// false for a record that fails either. The error is a context error only.
func classifyPrismWalls(ctx context.Context, bp brepPayload, embeds []brepEmbed, k int, caps brepPrismCaps) (brepPrismWalls, bool, error) {
	// P2: every other face is a wall along the axis or a rectangle across it.
	var walls brepPrismWalls
	work := freeform.NewFreeformWork()
	for fi, f := range bp.faces {
		if err := ctx.Err(); err != nil {
			return brepPrismWalls{}, false, err
		}
		if fi == caps.bottom || fi == caps.top {
			continue
		}
		if !walls.add(f, embeds[fi], caps.eF, k, caps.zlo, caps.zhi, work) {
			return brepPrismWalls{}, false, nil
		}
	}

	// P5: every wall and rectangle claims one segment of S, each exactly once.
	if !walls.claim(caps.section, work) {
		return brepPrismWalls{}, false, nil
	}
	return walls, true, nil
}

// recognisePrism reads the record as a prism along reference axis k, or
// reports false (brep-modify §4.1, P1–P5): readPrismCaps for P1, P3 and P4,
// then classifyPrismWalls for P2 and P5. The error is a context error only.
func recognisePrism(ctx context.Context, bp brepPayload, embeds []brepEmbed, k int) (brepPrismRead, bool, error) {
	caps, ok := readPrismCaps(bp, embeds, k)
	if !ok {
		return brepPrismRead{}, false, nil
	}
	walls, ok, err := classifyPrismWalls(ctx, bp, embeds, k, caps)
	if err != nil || !ok {
		return brepPrismRead{}, false, err
	}
	return brepPrismRead{
		pp: prismPayload{
			profile: caps.section, frame: caps.frame, xform: bp.xform,
			z0: caps.zlo, z1: caps.zhi,
			z0Delta: max(bp.faces[caps.bottom].z0Delta, walls.zloDelta),
			z1Delta: max(bp.faces[caps.top].z0Delta, walls.zhiDelta),
		},
		bottom: caps.bottom, top: caps.top, bottomLoop: caps.bottomLoop,
	}, true, nil
}

// brepLevel is a planar face's level as a reference coordinate.
func brepLevel(f brepFace, e brepEmbed) float64 { return e.Sign[2]*f.z0 + 0 }

// brepOutwardSign is the sign of a planar face's outward normal along its
// reference axis.
func brepOutwardSign(f brepFace, e brepEmbed) float64 {
	if f.outward {
		return e.Sign[2]
	}
	return -e.Sign[2]
}

// add classifies one face against P2 and records what P5 reads from it. It
// reports false for a face P2 does not admit.
func (w *brepPrismWalls) add(f brepFace, e, eF brepEmbed, k int, zlo, zhi float64, work *freeform.FreeformWork) bool {
	switch {
	case !f.planar() && e.Axis[2] == k:
		// (a): a wall along the axis over exactly [zlo, zhi], unsplit.
		if len(f.side0) != 0 || len(f.side1) != 0 {
			return false
		}
		l0, l1 := e.Sign[2]*f.z0+0, e.Sign[2]*f.z1+0
		d0, d1 := f.z0Delta, f.z1Delta
		if l0 > l1 {
			l0, l1, d0, d1 = l1, l0, d1, d0
		}
		if l0 != zlo || l1 != zhi {
			return false
		}
		seg, ok := newBrepPlaneMap(e, eF).segment(f.wall)
		if !ok {
			return false
		}
		walk, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return false
		}
		w.walls = append(w.walls, walk)
		w.zloDelta, w.zhiDelta = max(w.zloDelta, d0), max(w.zhiDelta, d1)
		return true
	case f.planar():
		// (b): a rectangle across the axis spanning exactly [zlo, zhi].
		rect, ok := brepPrismPlanarRect(f, e, eF, k, zlo, zhi)
		if ok {
			w.rects = append(w.rects, rect)
		}
		return ok
	default:
		// (c): the same rectangle stated as a straight wall along the axis.
		rect, ok := brepPrismSweptRect(f, e, eF, k, zlo, zhi)
		if ok {
			w.rects = append(w.rects, rect)
		}
		return ok
	}
}

// brepPrismPlanarRect reads a P2(b) face: one loop of four natural-range
// LineSegs, each along one in-plane axis, whose corners sit at exactly zlo
// and zhi along the axis, two at each. The projection is the rectangle's
// trace in F's plane; the outward normal is the face's own. The face's level
// is a coordinate of the prism's section, which RB1 admits only without a
// displacement, so a face whose level carries one reads as no rectangle.
func brepPrismPlanarRect(f brepFace, e, eF brepEmbed, k int, zlo, zhi float64) (brepPrismRect, bool) {
	region := f.region
	if f.z0Delta != 0 || len(region.Holes) != 0 || len(region.Outer.Segments) != 4 {
		return brepPrismRect{}, false
	}
	corners := make([][3]float64, 0, 4)
	segs := region.Outer.Segments
	for i, seg := range segs {
		from, to, ok := brepNaturalLine(seg)
		if !ok {
			return brepPrismRect{}, false
		}
		if (from.U == to.U) == (from.V == to.V) {
			return brepPrismRect{}, false
		}
		next, _, ok := brepNaturalLine(segs[(i+1)%len(segs)])
		if !ok || next != to {
			return brepPrismRect{}, false
		}
		corners = append(corners, e.Canon(from.U, from.V, f.z0))
	}
	m := 3 - k - e.Axis[2]
	lo, hi, ok := brepRectSpan(corners, k, m, zlo, zhi)
	if !ok {
		return brepPrismRect{}, false
	}
	level := e.Sign[2]*f.z0 + 0
	var normal [3]float64
	normal[e.Axis[2]] = brepOutwardSign(f, e)
	return brepPrismRectIn(eF, e.Axis[2], m, level, level, lo, hi, normal), true
}

// brepRectSpan checks four rectangle corners in reference coordinates: two at
// zlo and two at zhi along axis k, two at each of two distinct values along
// axis m, all four distinct. It returns the two values along m.
func brepRectSpan(corners [][3]float64, k, m int, zlo, zhi float64) (float64, float64, bool) {
	lo, hi := corners[0][m], corners[0][m]
	for _, c := range corners {
		lo, hi = min(lo, c[m]), max(hi, c[m])
	}
	if !(lo < hi) {
		return 0, 0, false
	}
	seen := map[[2]float64]struct{}{}
	for _, c := range corners {
		if (c[k] != zlo && c[k] != zhi) || (c[m] != lo && c[m] != hi) {
			return 0, 0, false
		}
		seen[[2]float64{c[k], c[m]}] = struct{}{}
	}
	return lo, hi, len(seen) == 4
}

// brepPrismSweptRect reads a P2(c) face: an unsplit wall that is a
// natural-range LineSeg along one in-plane axis, its ends at exactly zlo and
// zhi along the axis. It states the same rectangle as P2(b) (brep-modify
// §5.2): its trace in F's plane is the wall's constant coordinate over the
// sweep interval, and its outward normal is the wall's right-hand normal.
// Its levels are section coordinates, so either carrying a displacement
// reads as no rectangle, as in brepPrismPlanarRect.
func brepPrismSweptRect(f brepFace, e, eF brepEmbed, k int, zlo, zhi float64) (brepPrismRect, bool) {
	if f.z0Delta != 0 || f.z1Delta != 0 || len(f.side0) != 0 || len(f.side1) != 0 {
		return brepPrismRect{}, false
	}
	from, to, ok := brepNaturalLine(f.wall)
	if !ok {
		return brepPrismRect{}, false
	}
	du, dv := to.U-from.U, to.V-from.V
	if (du == 0) == (dv == 0) {
		return brepPrismRect{}, false
	}
	a, b := e.Canon(from.U, from.V, 0), e.Canon(to.U, to.V, 0)
	if !((a[k] == zlo && b[k] == zhi) || (a[k] == zhi && b[k] == zlo)) {
		return brepPrismRect{}, false
	}
	m := 3 - k - e.Axis[2]
	if a[m] != b[m] {
		return brepPrismRect{}, false
	}
	normal := e.Canon(brepSign(dv), brepSign(-du), 0)
	return brepPrismRectIn(eF, m, e.Axis[2], a[m], a[m], e.Sign[2]*f.z0+0, e.Sign[2]*f.z1+0, normal), true
}

// brepPrismRectIn states a projected rectangle in F's plane: the segment from
// (fixedAxis = fixed0, spanAxis = s0) to (fixedAxis = fixed1, spanAxis = s1)
// in reference coordinates, and the outward normal, both through F's embed.
func brepPrismRectIn(eF brepEmbed, fixedAxis, spanAxis int, fixed0, fixed1, s0, s1 float64, normal [3]float64) brepPrismRect {
	var p, q [3]float64
	p[fixedAxis], p[spanAxis] = fixed0, s0
	q[fixedAxis], q[spanAxis] = fixed1, s1
	lp, lq, ln := eF.Local(p), eF.Local(q), eF.Local(normal)
	return brepPrismRect{
		p:  Point2{U: lp[0], V: lp[1]},
		q:  Point2{U: lq[0], V: lq[1]},
		nu: ln[0], nv: ln[1],
	}
}

// brepSign is −1, 0 or +1 by the sign of x.
func brepSign(x float64) float64 {
	switch {
	case x > 0:
		return 1
	case x < 0:
		return -1
	default:
		return 0
	}
}

// brepNaturalLine reads a LineSeg over its natural range as its walked ends:
// 0→1 walks Start to End, 1→0 End to Start. Any other segment or range
// reports false.
func brepNaturalLine(seg CurveSegment) (Point2, Point2, bool) {
	l, ok := seg.(LineSeg)
	switch {
	case !ok:
		return Point2{}, Point2{}, false
	case l.TStart == 0 && l.TEnd == 1:
		return l.Start, l.End, true
	case l.TStart == 1 && l.TEnd == 0:
		return l.End, l.Start, true
	default:
		return Point2{}, Point2{}, false
	}
}

// claim is P5: every wall equals one segment of S as a walk, every rectangle
// projects onto one straight segment of S as a set with that segment's
// right-hand unit normal, and every segment of S is claimed exactly once.
func (w *brepPrismWalls) claim(section ProfileRecord, work *freeform.FreeformWork) bool {
	var walks []survey2d.SegmentWalk
	for _, loop := range append([]LoopRecord{section.Outer}, section.Holes...) {
		for _, seg := range loop.Segments {
			walk, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return false
			}
			walks = append(walks, walk)
		}
	}
	if len(w.walls)+len(w.rects) != len(walks) {
		return false
	}
	claimed := make([]bool, len(walks))
	take := func(match func(survey2d.SegmentWalk) bool) bool {
		for i, walk := range walks {
			if !claimed[i] && match(walk) {
				claimed[i] = true
				return true
			}
		}
		return false
	}
	for _, wall := range w.walls {
		key := brepWalkKeyOf(wall)
		if !take(func(s survey2d.SegmentWalk) bool { return brepWalkKeyOf(s) == key }) {
			return false
		}
	}
	for _, r := range w.rects {
		if !take(r.matches) {
			return false
		}
	}
	return true
}

// matches reports whether a straight walk of S is the rectangle's trace: the
// same two ends as a set, and the walk's right-hand normal (outward, with the
// material on the walk's left) along the rectangle's outward normal.
func (r brepPrismRect) matches(s survey2d.SegmentWalk) bool {
	if s.IsCircular() || s.Closed {
		return false
	}
	from, to := Point2{U: s.StartU, V: s.StartV}, Point2{U: s.EndU, V: s.EndV}
	sameEnds := (from == r.p && to == r.q) || (from == r.q && to == r.p)
	if !sameEnds {
		return false
	}
	du, dv := to.U-from.U, to.V-from.V
	return brepSign(dv) == r.nu && brepSign(-du) == r.nv
}

// brepWalkKey is a walk's directed geometry: kind, ends, and for a circular
// walk its centre, radius and sense. A whole circle has no ends to compare.
type brepWalkKey struct {
	circular, closed bool
	su, sv, eu, ev   float64
	cu, cv, r        float64
	ccw              bool
}

func brepWalkKeyOf(w survey2d.SegmentWalk) brepWalkKey {
	k := brepWalkKey{circular: w.IsCircular(), closed: w.Closed}
	if k.circular {
		k.cu, k.cv, k.r, k.ccw = w.CU, w.CV, w.Radius, w.Th1 > w.Th0
	}
	if !k.closed {
		k.su, k.sv, k.eu, k.ev = w.StartU, w.StartV, w.EndU, w.EndV
	}
	return k
}

// brepSameRegion is P4's comparison of bottom's region, re-expressed into F,
// with the section: equal outer loops, and each hole of one equal to exactly
// one hole of the other. Two loops are equal when one's segment records, in
// order, are the other's from some starting segment. It returns, per bottom
// region loop, the section loop it equals.
func brepSameRegion(section, bottom ProfileRecord) ([]int, bool) {
	if len(section.Holes) != len(bottom.Holes) || !brepLoopsEqual(section.Outer, bottom.Outer) {
		return nil, false
	}
	loops := make([]int, 1+len(bottom.Holes))
	used := make([]bool, len(section.Holes))
	for bi, hole := range bottom.Holes {
		found := false
		for si, want := range section.Holes {
			if !used[si] && brepLoopsEqual(want, hole) {
				used[si], loops[1+bi], found = true, 1+si, true
				break
			}
		}
		if !found {
			return nil, false
		}
	}
	return loops, true
}

// brepLoopsEqual compares two loops' segment records up to the segment the
// loop starts at.
func brepLoopsEqual(a, b LoopRecord) bool {
	n := len(a.Segments)
	if n != len(b.Segments) || n == 0 {
		return false
	}
	for r := range n {
		same := true
		for i := range n {
			if !reflect.DeepEqual(a.Segments[i], b.Segments[(i+r)%n]) {
				same = false
				break
			}
		}
		if same {
			return true
		}
	}
	return false
}

// brepPlaneMap carries plane-local records from one face frame into another
// whose normal lies on the same reference axis (brep-modify §5.4): the
// signed permutation from's Canon then to's Local, restricted to the two
// in-plane axes. Every coordinate moves exactly. A reflecting map re-winds
// what it carries as general-boolean A4 states, so material stays on the
// left of every walk.
type brepPlaneMap struct {
	from, to brepEmbed
	reflect  bool
}

func newBrepPlaneMap(from, to brepEmbed) brepPlaneMap {
	a, b := to.Local(from.Canon(1, 0, 0)), to.Local(from.Canon(0, 1, 0))
	return brepPlaneMap{from: from, to: to, reflect: a[0]*b[1]-a[1]*b[0] < 0}
}

func (m brepPlaneMap) point(p Point2) Point2 {
	c := m.to.Local(m.from.Canon(p.U, p.V, 0))
	return Point2{U: c[0], V: c[1]}
}

// region carries a whole region, loop by loop, keeping the loop order. The
// identity map returns the region as recorded.
func (m brepPlaneMap) region(p ProfileRecord) (ProfileRecord, bool) {
	if m.from == m.to {
		return p, true
	}
	outer, ok := m.loop(p.Outer)
	if !ok {
		return ProfileRecord{}, false
	}
	out := ProfileRecord{Outer: outer, Holes: make([]LoopRecord, 0, len(p.Holes))}
	for _, h := range p.Holes {
		hole, ok := m.loop(h)
		if !ok {
			return ProfileRecord{}, false
		}
		out.Holes = append(out.Holes, hole)
	}
	return out, true
}

// loop carries one loop; a reflecting map reverses its segment order.
func (m brepPlaneMap) loop(l LoopRecord) (LoopRecord, bool) {
	segs := make([]CurveSegment, len(l.Segments))
	for i, seg := range l.Segments {
		mapped, ok := m.segment(seg)
		if !ok {
			return LoopRecord{}, false
		}
		j := i
		if m.reflect {
			j = len(segs) - 1 - i
		}
		segs[j] = mapped
	}
	return LoopRecord{Segments: segs}, true
}

// segment carries one segment, re-wound under a reflecting map as A4 states:
// a LineSeg swaps its ends, an ArcSeg becomes {m(Center), m(End), m(Start)},
// both keeping their range, and a CircleSeg maps its centre alone. A4 carries
// a narrowed line or arc range under a reflection inexactly, so such a
// segment reports false, as does any kind but a line, an arc or a circle.
func (m brepPlaneMap) segment(seg CurveSegment) (CurveSegment, bool) {
	natural := func(t0, t1 float64) bool { return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0) }
	switch s := seg.(type) {
	case LineSeg:
		start, end := m.point(s.Start), m.point(s.End)
		if !m.reflect {
			return LineSeg{Start: start, End: end, TStart: s.TStart, TEnd: s.TEnd}, true
		}
		if !natural(s.TStart, s.TEnd) {
			return nil, false
		}
		return LineSeg{Start: end, End: start, TStart: s.TStart, TEnd: s.TEnd}, true
	case ArcSeg:
		c, start, end := m.point(s.Center), m.point(s.Start), m.point(s.End)
		if !m.reflect {
			return ArcSeg{Center: c, Start: start, End: end, TStart: s.TStart, TEnd: s.TEnd}, true
		}
		if !natural(s.TStart, s.TEnd) {
			return nil, false
		}
		return ArcSeg{Center: c, Start: end, End: start, TStart: s.TStart, TEnd: s.TEnd}, true
	case CircleSeg:
		s.Center = m.point(s.Center)
		return s, true
	default:
		return nil, false
	}
}

// requireLateralEdges is Fillet's route P classification (brep-modify §4.2):
// every selected edge is a lateral edge of pp, the junction at one corner of
// its section (matchCornerBudget). Any other edge is modify S1's class.
func requireLateralEdges(ctx context.Context, pp prismPayload, edges []*Edge) error {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := prismCornerLoopsBudget(budget, pp)
	if err != nil {
		return err
	}
	for ei, e := range edges {
		_, _, found, err := matchCornerBudget(budget, pp, loops, e)
		if err != nil {
			return err
		}
		if !found {
			return fmt.Errorf(`%w: %s is not a lateral edge of the prism`, ErrUnsupported, selectedEdgeContext(ei, e))
		}
	}
	return nil
}
