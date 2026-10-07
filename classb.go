package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is class B of docs/general-boolean-design.md §3: an operand X read
// through its face view (§4.1) — a prism, a stacked prism or a brep body — and
// a prism Y whose axes are signed axes of X's reference frame. Every reading
// is taken in X's reference frame, the first face's: d is the reference axis
// Y sweeps along, e and w the other two.
//
// This increment builds the through-nesting reach (§3 B.3), decided by exact
// box comparisons of recorded coordinates. Y's tube — its section box swept
// over its interval — meets one or two faces of X and no other, each a face
// across d (a planar face whose normal lies on d, or a straight wall at
// constant d) whose region holds Y's section. Over the tube, X is then the
// slab between two such faces (through) or the half-space past one (rooted),
// and every boolean of X and Y is Y's section swept over an interval, joined
// to X's faces by those one or two faces taking Y's section as a hole. Every
// other pair misses silently and takes the mesh path.

// classBPair is an admitted pair stated in X's reference frame: X's face view
// with each face's map onto the reference, Y's axes as signed reference axes,
// and Y's record re-expressed exactly into the frame g (Y's axes at the
// reference origin).
type classBPair struct {
	x      brepPayload
	embeds []brepEmbed
	ref    r3.Frame
	y      prismPayload // re-expressed: frame g, coordinates shifted exactly
	g      r3.Frame
	// axis/sign map Y's local axis i (U, V, N) onto reference axis axis[i]
	// with sign[i]; axis[2] is d.
	axis [3]int
	sign [3]float64
}

// d is the reference axis Y sweeps along.
func (cp classBPair) d() int { return cp.axis[2] }

// xLocalOfY places a point of Y's re-expressed section at Y level z in
// reference coordinates; the map is a signed permutation, so it is exact.
func (cp classBPair) xLocalOfY(u, v, z float64) [3]float64 {
	var out [3]float64
	for i, c := range [3]float64{u, v, z} {
		out[cp.axis[i]] = cp.sign[i]*c + 0
	}
	return out
}

// yLocalOfX is xLocalOfY's inverse.
func (cp classBPair) yLocalOfX(x [3]float64) [3]float64 {
	var out [3]float64
	for i := range out {
		out[i] = cp.sign[i]*x[cp.axis[i]] + 0
	}
	return out
}

// tryClassB attempts op over the pair. ok=false (err nil) is a silent miss:
// the caller takes the mesh path unchanged. A non-nil err is a refusal past
// the point of no return (prism-boolean §3.4) or a cancellation. Cut takes X
// as the target and Y as the tool; Union and Intersect, being symmetric, also
// try the operands the other way round, so a brep body or a stacked prism
// enters in either position.
func tryClassB(ctx context.Context, op meshbool.OperationKind, a, b *Body) (featurePayload, bool, error) {
	pairs := [][2]*Body{{a, b}}
	if op != meshbool.OpCut {
		pairs = append(pairs, [2]*Body{b, a})
	}
	for _, pair := range pairs {
		cp, ok, err := admitClassBPair(ctx, pair[0], pair[1])
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		reach, ok, err := classBThroughReach(ctx, cp)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		return buildClassB(ctx, op, cp, reach)
	}
	return nil, false, nil
}

// classBFaceView is X's face view, or ok=false where X has none this
// increment reads: a prism (not a surface result), a stacked prism whose
// slabs keep one outer loop and one region, or a brep body.
func classBFaceView(ctx context.Context, b *Body) (brepPayload, bool, error) {
	switch p := b.payload.(type) {
	case prismPayload:
		if p.surfaceResult {
			return brepPayload{}, false, nil
		}
		bp, err := brepOfPrism(p)
		return bp, err == nil, err
	case stackedPrismPayload:
		if p.isGroup() {
			return brepPayload{}, false, nil
		}
		runs, err := p.outerRuns()
		if err != nil || len(runs) != 1 {
			return brepPayload{}, false, err
		}
		bp, err := brepOfStacked(ctx, p)
		if err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return brepPayload{}, false, ctxErr
			}
			// A stack its own audit refuses has no face view: a silent miss.
			return brepPayload{}, false, nil
		}
		return bp, true, nil
	case brepPayload:
		return p, true, nil
	default:
		return brepPayload{}, false, nil
	}
}

// admitClassBPair runs §3's entry gate B1–B8 in order, plus the two conditions
// the brep record itself needs: one shared placement, and Y's axes carried
// bit for bit as signed reference axes at the reference origin (§4.1). Every
// miss is silent.
func admitClassBPair(ctx context.Context, xBody, yBody *Body) (classBPair, bool, error) {
	// B1: X exposes a face view; Y is a prism.
	y, okY := yBody.payload.(prismPayload)
	if !okY || y.surfaceResult {
		return classBPair{}, false, nil
	}
	x, ok, err := classBFaceView(ctx, xBody)
	if err != nil || !ok {
		return classBPair{}, false, err
	}
	if err := ctx.Err(); err != nil {
		return classBPair{}, false, err
	}
	// B5: no section displacement to amplify.
	if x.sectionDelta() != 0 || y.sectionDelta != 0 {
		return classBPair{}, false, nil
	}
	if x.xform != y.xform {
		return classBPair{}, false, nil
	}
	embeds, err := brepEmbeds(x.faces)
	if err != nil {
		return classBPair{}, false, nil //nolint:nilerr // frames with no exact map onto the reference are a silent miss, not a refusal
	}
	ref := x.faces[0].frame
	// B3 and B4: every axis dot is exactly 0 or ±1, and Y's axes are the
	// reference axes, signed, bit for bit.
	refAxes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	yAxes := [3]r3.Vec{y.frame.U(), y.frame.V(), y.frame.N()}
	for _, a := range refAxes {
		for _, b := range yAxes {
			if dot := a.Dot(b); dot != 0 && dot != 1 && dot != -1 {
				return classBPair{}, false, nil
			}
		}
	}
	cp := classBPair{x: x, embeds: embeds, ref: ref}
	used := [3]bool{}
	for i, b := range yAxes {
		found := false
		for j, a := range refAxes {
			switch {
			case b == a:
				cp.axis[i], cp.sign[i], found = j, 1, true
			case b == a.Scale(-1):
				cp.axis[i], cp.sign[i], found = j, -1, true
			}
			if found {
				break
			}
		}
		if !found || used[cp.axis[i]] {
			return classBPair{}, false, nil
		}
		used[cp.axis[i]] = true
	}
	// B3 for a prism or a stacked X: the sweeps are perpendicular. A
	// co-directional pair is class A's. A brep body has no single sweep, and
	// takes Y along any reference axis.
	if _, isBrep := xBody.payload.(brepPayload); !isBrep && cp.axis[2] == 2 {
		return classBPair{}, false, nil
	}
	g, err := r3.NewFrame(ref.Origin(), yAxes[0], yAxes[1])
	if err != nil {
		return classBPair{}, false, fmt.Errorf(`%w: the tool's axes at the target's origin make no frame: %s`, ErrDegenerate, err)
	}
	if g.U() != yAxes[0] || g.V() != yAxes[1] || g.N() != yAxes[2] {
		return classBPair{}, false, nil
	}
	cp.g = g
	// B2: lines, circles and arcs only, each over its natural range, so every
	// recorded point is the point the record states.
	if !classBNaturalRecord(y.profile) {
		return classBPair{}, false, nil
	}
	for _, f := range x.faces {
		if !classBNaturalRecord(classBFaceRecord(f)) {
			return classBPair{}, false, nil
		}
	}
	// G6: the tool is hole-free.
	if len(y.profile.Holes) != 0 {
		return classBPair{}, false, nil
	}
	shifted, ok := classBShiftedTool(ref, y, g)
	if !ok {
		return classBPair{}, false, nil
	}
	cp.y = shifted
	// B6: every line runs along a reference axis. Under B4 that is parallel
	// or perpendicular to the other operand's normal.
	if !classBAxisAligned(cp.y.profile) {
		return classBPair{}, false, nil
	}
	for _, f := range x.faces {
		if !classBAxisAligned(classBFaceRecord(f)) {
			return classBPair{}, false, nil
		}
	}
	ok, err = classBCurvedApart(ctx, cp)
	if err != nil || !ok {
		return classBPair{}, false, err
	}
	ok, err = classBNoCoplanarFaces(ctx, cp)
	if err != nil || !ok {
		return classBPair{}, false, err
	}
	return cp, true, nil
}

// classBFaceRecord is one face's segments as a record: a planar face's
// region, or a swept face's one wall as a single-segment loop.
func classBFaceRecord(f brepFace) ProfileRecord {
	if f.planar() {
		return *f.region
	}
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{f.wall}}}
}

// classBNaturalRecord reports whether every segment is a line, circle or arc
// recorded over its natural range, walked either way: a line or an arc over
// [0, 1] or [1, 0], a circle over one whole turn.
func classBNaturalRecord(p ProfileRecord) bool {
	natural := func(t0, t1 float64) bool { return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0) }
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			switch s := seg.(type) {
			case LineSeg:
				if !natural(s.TStart, s.TEnd) {
					return false
				}
			case ArcSeg:
				if !natural(s.TStart, s.TEnd) {
					return false
				}
			case CircleSeg:
				if !natural(s.TStart, s.TEnd) {
					return false
				}
			default:
				return false
			}
		}
	}
	return true
}

// classBShiftedTool re-expresses Y's record into g, Y's axes at the reference
// origin: every coordinate moves by the exact rational dot of the origin
// difference with its axis. The shift is admitted only where every shifted
// coordinate and level is itself a float, so the re-expression rounds
// nothing: a datum plane and its CreateOffsetPlane at a float distance meet
// that, and a pair whose sums would round misses.
func classBShiftedTool(ref r3.Frame, y prismPayload, g r3.Frame) (prismPayload, bool) {
	ox, oy := ref.Origin(), y.frame.Origin()
	diff := [3]*big.Rat{
		new(big.Rat).Sub(proofarith.FloatRat(oy.X), proofarith.FloatRat(ox.X)),
		new(big.Rat).Sub(proofarith.FloatRat(oy.Y), proofarith.FloatRat(ox.Y)),
		new(big.Rat).Sub(proofarith.FloatRat(oy.Z), proofarith.FloatRat(ox.Z)),
	}
	var shift [3]*big.Rat
	for i, axis := range [3]r3.Vec{g.U(), g.V(), g.N()} {
		shift[i] = proofbound.RatAdd(
			proofbound.RatMul(diff[0], proofarith.FloatRat(axis.X)),
			proofbound.RatMul(diff[1], proofarith.FloatRat(axis.Y)),
			proofbound.RatMul(diff[2], proofarith.FloatRat(axis.Z)),
		)
	}
	ok := true
	move := func(value float64, i int) float64 {
		exact := new(big.Rat).Add(proofarith.FloatRat(value), shift[i])
		held, _ := exact.Float64()
		if proofarith.RationalFloatError(exact, held) != 0 {
			ok = false
		}
		return held
	}
	point := func(p Point2) Point2 { return Point2{U: move(p.U, 0), V: move(p.V, 1)} }
	loops := append([]LoopRecord{y.profile.Outer}, y.profile.Holes...)
	moved := make([]LoopRecord, len(loops))
	for li, loop := range loops {
		for _, seg := range loop.Segments {
			switch s := seg.(type) {
			case LineSeg:
				s.Start, s.End = point(s.Start), point(s.End)
				seg = s
			case ArcSeg:
				s.Center, s.Start, s.End = point(s.Center), point(s.Start), point(s.End)
				seg = s
			case CircleSeg:
				s.Center = point(s.Center)
				seg = s
			}
			moved[li].Segments = append(moved[li].Segments, seg)
		}
	}
	out := y
	out.frame = g
	out.profile = ProfileRecord{Outer: moved[0], Holes: moved[1:]}
	out.z0, out.z1 = move(y.z0, 2), move(y.z1, 2)
	out.walks = nil
	return out, ok
}

// classBAxisAligned is B6 for one record. The other operand's normal lies on
// a reference axis (B4), and every face frame's axes are reference axes, so a
// line runs along or across it exactly when one of its two coordinate
// differences is zero. Circles and arcs have no direction and pass.
func classBAxisAligned(p ProfileRecord) bool {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			line, ok := seg.(LineSeg)
			if !ok {
				continue
			}
			du, dv := line.End.U-line.Start.U, line.End.V-line.Start.V
			if du != 0 && dv != 0 {
				return false
			}
		}
	}
	return true
}

// classBBox is a segment's outward box in its own record's plane
// coordinates, as exact rationals.
type classBBox struct{ lo, hi [2]*big.Rat }

// classBSegmentBox bounds one natural-range segment in its own plane: a line
// by its endpoints, a circle or an arc by its whole circle, with an arc's
// computed radius rounded outward by its proven bound.
func classBSegmentBox(seg CurveSegment) (classBBox, error) {
	rat := proofarith.FloatRat
	switch s := seg.(type) {
	case LineSeg:
		b := classBBox{}
		for i, pair := range [2][2]float64{{s.Start.U, s.End.U}, {s.Start.V, s.End.V}} {
			lo, hi := pair[0], pair[1]
			if lo > hi {
				lo, hi = hi, lo
			}
			b.lo[i], b.hi[i] = rat(lo), rat(hi)
		}
		return b, nil
	default:
		w, err := walkOf(seg, freeform.NewFreeformWork())
		if err != nil {
			return classBBox{}, err
		}
		r := rat(proofbound.UpRound(proofbound.AbsSumUpper(w.Radius, w.RadiusBound)))
		if r == nil {
			return classBBox{}, fmt.Errorf(`%w: a circular segment states no finite radius`, ErrNotFinite)
		}
		c := [2]*big.Rat{rat(w.CU), rat(w.CV)}
		b := classBBox{}
		for i := range c {
			b.lo[i] = new(big.Rat).Sub(c[i], r)
			b.hi[i] = new(big.Rat).Add(c[i], r)
		}
		return b, nil
	}
}

// classBApart reports whether two closed intervals are separated by an exact
// comparison.
func classBApart(alo, ahi, blo, bhi *big.Rat) bool {
	return ahi.Cmp(blo) < 0 || bhi.Cmp(alo) < 0
}

// box3 is an exact axis-aligned box in reference coordinates.
type box3 struct{ lo, hi [3]*big.Rat }

// apart reports whether two boxes are separated along some axis.
func (a box3) apart(b box3) bool {
	for k := range 3 {
		if classBApart(a.lo[k], a.hi[k], b.lo[k], b.hi[k]) {
			return true
		}
	}
	return false
}

// classBPlace lifts a plane box and a level interval through one frame's map
// onto the reference axes: local axis i lands on axis[i], and a negative sign
// swaps that axis's two ends.
func classBPlace(b classBBox, zlo, zhi *big.Rat, axis [3]int, sign [3]float64) box3 {
	lo := [3]*big.Rat{b.lo[0], b.lo[1], zlo}
	hi := [3]*big.Rat{b.hi[0], b.hi[1], zhi}
	var placed box3
	for i := range 3 {
		k := axis[i]
		if sign[i] > 0 {
			placed.lo[k], placed.hi[k] = lo[i], hi[i]
			continue
		}
		placed.lo[k], placed.hi[k] = new(big.Rat).Neg(hi[i]), new(big.Rat).Neg(lo[i])
	}
	return placed
}

// classBUnion is the smallest box holding both.
func classBUnion(a, b classBBox) classBBox {
	out := a
	for i := range 2 {
		if b.lo[i].Cmp(out.lo[i]) < 0 {
			out.lo[i] = b.lo[i]
		}
		if b.hi[i].Cmp(out.hi[i]) > 0 {
			out.hi[i] = b.hi[i]
		}
	}
	return out
}

// classBFaceBox is one face of X as a reference box, widened by its level
// displacements: a planar face by its outer loop's segments at its level, a
// swept face by its wall over its interval.
func classBFaceBox(f brepFace, e brepEmbed) (box3, error) {
	segs := []CurveSegment{f.wall}
	if f.planar() {
		segs = f.region.Outer.Segments
	}
	var plane classBBox
	for i, seg := range segs {
		b, err := classBSegmentBox(seg)
		if err != nil {
			return box3{}, err
		}
		if i == 0 {
			plane = b
			continue
		}
		plane = classBUnion(plane, b)
	}
	rat := proofarith.FloatRat
	zlo := new(big.Rat).Sub(rat(f.z0), rat(f.z0Delta))
	zhi := new(big.Rat).Add(rat(f.z1), rat(f.z1Delta))
	return classBPlace(plane, zlo, zhi, e.axis, e.sign), nil
}

// classBCurvedApart is B7: every curved wall of X and every curved wall of Y
// have outward boxes separated along some axis by an exact comparison.
func classBCurvedApart(ctx context.Context, cp classBPair) (bool, error) {
	var xs []box3
	for fi, f := range cp.x.faces {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if f.planar() {
			continue
		}
		if _, line := f.wall.(LineSeg); line {
			continue
		}
		b, err := classBFaceBox(f, cp.embeds[fi])
		if err != nil {
			return false, err
		}
		xs = append(xs, b)
	}
	var ys []box3
	rat := proofarith.FloatRat
	lz, hz := rat(min(cp.y.z0, cp.y.z1)), rat(max(cp.y.z0, cp.y.z1))
	for _, seg := range cp.y.profile.Outer.Segments {
		if _, line := seg.(LineSeg); line {
			continue
		}
		b, err := classBSegmentBox(seg)
		if err != nil {
			return false, err
		}
		ys = append(ys, classBPlace(b, lz, hz, cp.axis, cp.sign))
	}
	for _, a := range xs {
		for _, b := range ys {
			if !a.apart(b) {
				return false, nil
			}
		}
	}
	return true, nil
}

// classBPlane is one planar face's carrier in reference coordinates: the plane
// x[axis] == level.
type classBPlane struct {
	axis  int
	level *big.Rat
}

// classBRecordPlanes lists the planes of one record's straight walls: a line
// runs along one in-plane axis, so its plane holds the other in-plane
// coordinate fixed. axis and sign place the record's local axes on the
// reference's.
func classBRecordPlanes(p ProfileRecord, axis [3]int, sign [3]float64) []classBPlane {
	var out []classBPlane
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			line, ok := seg.(LineSeg)
			if !ok {
				continue
			}
			if line.Start.U == line.End.U {
				out = append(out, classBPlane{axis: axis[0], level: proofarith.FloatRat(sign[0]*line.Start.U + 0)})
				continue
			}
			out = append(out, classBPlane{axis: axis[1], level: proofarith.FloatRat(sign[1]*line.Start.V + 0)})
		}
	}
	return out
}

// classBNoCoplanarFaces is B8: no planar face or straight wall of X lies in
// the plane of one of Y's caps or straight walls, compared exactly.
func classBNoCoplanarFaces(ctx context.Context, cp classBPair) (bool, error) {
	var xs []classBPlane
	for fi, f := range cp.x.faces {
		e := cp.embeds[fi]
		if f.planar() {
			xs = append(xs, classBPlane{axis: e.axis[2], level: proofarith.FloatRat(e.sign[2]*f.z0 + 0)})
			continue
		}
		xs = append(xs, classBRecordPlanes(classBFaceRecord(f), e.axis, e.sign)...)
	}
	ys := classBRecordPlanes(cp.y.profile, cp.axis, cp.sign)
	for _, z := range []float64{cp.y.z0, cp.y.z1} {
		ys = append(ys, classBPlane{axis: cp.axis[2], level: proofarith.FloatRat(cp.sign[2]*z + 0)})
	}
	for _, a := range xs {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		for _, b := range ys {
			if a.axis == b.axis && a.level.Cmp(b.level) == 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// classBSlab is one face of X across d that Y's tube meets: the face's index,
// its level along d in reference coordinates and that level's displacement,
// and the side its outward normal points to along d (+1 or −1).
type classBSlab struct {
	face       int
	level      float64
	levelDelta float64
	outward    int
}

// classBThrough is the reach of an admitted pair: one slab (rooted) or two,
// ordered by level (through).
type classBThrough struct {
	slabs []classBSlab
}

// classBThroughReach decides, by exact comparisons of recorded coordinates,
// whether the pair is in the through-nesting reach. T is Y's tube: its
// section's outward box swept over its interval, in reference coordinates.
//
//   - The faces of X whose widened boxes are not strictly apart from T are
//     one or two, and each is across d: a planar face whose normal lies on d,
//     or a straight wall running at constant d.
//   - Each lies strictly inside Y's interval along d, with both level
//     displacements as margin.
//   - Two faces bound X's material between them: the lower one's outward
//     normal points −d and the upper one's +d.
//
// No other face of X meets T, so inside T the only boundary of X is those
// faces' patches over Y's section, which §5's perpendicular-face scene proves
// lie inside each face's region (buildClassB). X over T is then the slab
// between the two faces, or, with one, the part of T on its material side.
// A pair outside it misses.
func classBThroughReach(ctx context.Context, cp classBPair) (classBThrough, bool, error) {
	rat := proofarith.FloatRat
	d := cp.d()
	var section classBBox
	for i, seg := range cp.y.profile.Outer.Segments {
		b, err := classBSegmentBox(seg)
		if err != nil {
			return classBThrough{}, false, err
		}
		if i == 0 {
			section = b
			continue
		}
		section = classBUnion(section, b)
	}
	tube := classBPlace(section, rat(min(cp.y.z0, cp.y.z1)), rat(max(cp.y.z0, cp.y.z1)), cp.axis, cp.sign)
	var slabs []classBSlab
	for fi, f := range cp.x.faces {
		if err := ctx.Err(); err != nil {
			return classBThrough{}, false, err
		}
		e := cp.embeds[fi]
		b, err := classBFaceBox(f, e)
		if err != nil {
			return classBThrough{}, false, err
		}
		if b.apart(tube) {
			continue
		}
		slab, ok := classBAcross(f, e, d)
		if !ok {
			return classBThrough{}, false, nil
		}
		slab.face = fi
		slabs = append(slabs, slab)
	}
	if len(slabs) == 0 || len(slabs) > 2 {
		return classBThrough{}, false, nil
	}
	if len(slabs) == 2 && slabs[1].level < slabs[0].level {
		slabs[0], slabs[1] = slabs[1], slabs[0]
	}
	if len(slabs) == 2 && (!(slabs[0].level < slabs[1].level) || slabs[0].outward != -1 || slabs[1].outward != 1) {
		return classBThrough{}, false, nil
	}
	// Y's interval along d in reference coordinates, its level displacements
	// as inward margins.
	lowY := cp.xLocalOfY(0, 0, cp.y.z0)[d]
	highY := cp.xLocalOfY(0, 0, cp.y.z1)[d]
	lowDelta, highDelta := cp.y.z0Delta, cp.y.z1Delta
	if lowY > highY {
		lowY, highY = highY, lowY
		lowDelta, highDelta = highDelta, lowDelta
	}
	b0 := new(big.Rat).Add(rat(lowY), rat(lowDelta))
	b1 := new(big.Rat).Sub(rat(highY), rat(highDelta))
	for _, s := range slabs {
		lo := new(big.Rat).Sub(rat(s.level), rat(s.levelDelta))
		hi := new(big.Rat).Add(rat(s.level), rat(s.levelDelta))
		if b0.Cmp(lo) >= 0 || hi.Cmp(b1) >= 0 {
			return classBThrough{}, false, nil
		}
	}
	return classBThrough{slabs: slabs}, true, nil
}

// classBAcross reads one face of X as a face across d: a planar face whose
// normal lies on d, at its level, outward per its flag; or a straight wall
// whose line holds d constant, at that coordinate, outward to the right of
// its walk. Anything else is not across d.
func classBAcross(f brepFace, e brepEmbed, d int) (classBSlab, bool) {
	if f.planar() {
		if e.axis[2] != d {
			return classBSlab{}, false
		}
		out := int(e.sign[2])
		if !f.outward {
			out = -out
		}
		return classBSlab{level: e.sign[2]*f.z0 + 0, levelDelta: f.z0Delta, outward: out}, true
	}
	line, ok := f.wall.(LineSeg)
	if !ok {
		return classBSlab{}, false
	}
	start, end := [2]float64{line.Start.U, line.Start.V}, [2]float64{line.End.U, line.End.V}
	for i := range 2 {
		if e.axis[i] != d || start[i] != end[i] {
			continue
		}
		// The wall runs along local axis 1−i. The material lies on the left
		// of its walk, so the outward normal is its right: (t_v, −t_u).
		t := [2]float64{end[0] - start[0], end[1] - start[1]}
		right := [2]float64{t[1], -t[0]}
		sign := 1
		if right[i]*e.sign[i] < 0 {
			sign = -1
		}
		return classBSlab{level: e.sign[i]*start[i] + 0, outward: sign}, true
	}
	return classBSlab{}, false
}

// classBSlabFace is one slab face rebuilt in g: its region before Y's
// section is cut from it, its level along g's normal, the level's
// displacement, the face's own section displacement, and its outward flag
// against g's normal.
type classBSlabFace struct {
	region     ProfileRecord
	level      float64
	levelDelta float64
	delta      float64
	outward    bool
}

// classBSlabInG restates a slab face in g. A straight wall becomes the
// rectangle it sweeps, whose outer edges lie on its own sweep's ends and so
// move with those levels' displacements. A planar face's region maps through
// the signed permutation between its frame and g; where that map reverses
// the plane's orientation, the loops are walked back (rewindLoop), as a
// reflected record is.
func classBSlabInG(budget *proofbound.WorkBudget, cp classBPair, s classBSlab) (classBSlabFace, error) {
	f := cp.x.faces[s.face]
	e := cp.embeds[s.face]
	toG := func(u, v, z float64) [3]float64 { return cp.yLocalOfX(e.canon(u, v, z)) }
	out := classBSlabFace{
		level:      cp.sign[2]*s.level + 0,
		levelDelta: s.levelDelta,
		outward:    float64(s.outward)*cp.sign[2] > 0,
	}
	if !f.planar() {
		line, ok := f.wall.(LineSeg)
		if !ok {
			return classBSlabFace{}, fmt.Errorf(`%w: a slab wall of the through-nesting reach is not a line`, ErrDegenerate)
		}
		var corners []Point2
		for _, c := range [][3]float64{
			{line.Start.U, line.Start.V, f.z0}, {line.End.U, line.End.V, f.z0},
			{line.End.U, line.End.V, f.z1}, {line.Start.U, line.Start.V, f.z1},
		} {
			p := toG(c[0], c[1], c[2])
			corners = append(corners, Point2{U: p[0], V: p[1]})
		}
		out.region = ProfileRecord{Outer: classBRectLoop(corners)}
		out.delta = max(f.z0Delta, f.z1Delta)
		return out, nil
	}
	origin, pu, pv := toG(0, 0, f.z0), toG(1, 0, f.z0), toG(0, 1, f.z0)
	det := (pu[0]-origin[0])*(pv[1]-origin[1]) - (pu[1]-origin[1])*(pv[0]-origin[0])
	mapPoint := func(p Point2) (Point2, error) { //nolint:unparam // rewindLoop's point map may fail; this exact one never does
		q := toG(p.U, p.V, f.z0)
		return Point2{U: q[0], V: q[1]}, nil
	}
	mapLoop := func(loop LoopRecord) (LoopRecord, error) {
		if det < 0 {
			rewound, _, err := rewindLoop(budget, loop, mapPoint)
			return rewound, err
		}
		segs := make([]CurveSegment, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return LoopRecord{}, err
			}
			switch sg := seg.(type) {
			case LineSeg:
				sg.Start, _ = mapPoint(sg.Start)
				sg.End, _ = mapPoint(sg.End)
				segs[i] = sg
			case ArcSeg:
				sg.Center, _ = mapPoint(sg.Center)
				sg.Start, _ = mapPoint(sg.Start)
				sg.End, _ = mapPoint(sg.End)
				segs[i] = sg
			case CircleSeg:
				sg.Center, _ = mapPoint(sg.Center)
				segs[i] = sg
			default:
				return LoopRecord{}, fmt.Errorf(`%w: a %T segment has no class-B face map`, ErrUnsupported, seg)
			}
		}
		return LoopRecord{Segments: segs}, nil
	}
	outer, err := mapLoop(f.region.Outer)
	if err != nil {
		return classBSlabFace{}, err
	}
	out.region = ProfileRecord{Outer: outer}
	for _, hole := range f.region.Holes {
		mapped, err := mapLoop(hole)
		if err != nil {
			return classBSlabFace{}, err
		}
		out.region.Holes = append(out.region.Holes, mapped)
	}
	out.delta = f.delta
	return out, nil
}

// buildClassB assembles op's result (§4) over the reach, in g. Each slab face
// becomes a planar face in g carrying Y's section as a new hole, its region
// the perpendicular-face scene's answer (§5): prism-boolean's clean-nesting
// match of the face's region against Y's section. A scene that does not
// return that match leaves the pair outside the reach and misses silently
// (prism-boolean §3.4); every refusal after it is an error.
//
// Along g's normal, Y spans [a, b] and the slab faces sit at s (rooted) or
// s0 < s1 (through). X's material over Y's section is [s0, s1], or the side of
// s its face's outward normal points away from. Then:
//
//   - Cut keeps X's other faces, the slab faces, and Y's walls reversed over
//     the inside interval; rooted, the inside end is the hole's floor, Y's
//     section facing back up the hole.
//   - Union keeps X's other faces, the slab faces, and Y's walls with Y's caps
//     over the parts of [a, b] outside X.
//   - Intersect is Y's section swept over the inside interval: a prism.
func buildClassB(ctx context.Context, op meshbool.OperationKind, cp classBPair, reach classBThrough) (featurePayload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	slabs := make([]classBSlabFace, len(reach.slabs))
	for i, s := range reach.slabs {
		face, err := classBSlabInG(budget, cp, s)
		if err != nil {
			return nil, false, err
		}
		region, matched, err := classBPerpendicularRegion(ctx, cp, face.region)
		if err != nil || !matched {
			return nil, false, err
		}
		face.region = region
		slabs[i] = face
	}
	if len(slabs) == 2 && slabs[1].level < slabs[0].level {
		slabs[0], slabs[1] = slabs[1], slabs[0]
	}
	// The interval of [a, b] inside X, each end with its displacement.
	type end struct{ level, delta float64 }
	a, b := end{cp.y.z0, cp.y.z0Delta}, end{cp.y.z1, cp.y.z1Delta}
	var inside [2]end
	var outside [][2]end
	switch {
	case len(slabs) == 2:
		inside = [2]end{{slabs[0].level, slabs[0].levelDelta}, {slabs[1].level, slabs[1].levelDelta}}
		outside = [][2]end{{a, inside[0]}, {inside[1], b}}
	case slabs[0].outward:
		// Outward +N: the material lies below the face.
		inside = [2]end{a, {slabs[0].level, slabs[0].levelDelta}}
		outside = [][2]end{{inside[1], b}}
	default:
		inside = [2]end{{slabs[0].level, slabs[0].levelDelta}, b}
		outside = [][2]end{{a, inside[0]}}
	}
	if op == meshbool.OpIntersect {
		return prismPayload{profile: cp.y.profile, frame: cp.g, xform: cp.x.xform,
			z0: inside[0].level, z1: inside[1].level, z0Delta: inside[0].delta, z1Delta: inside[1].delta}, true, nil
	}
	// Point of no return (prism-boolean §3.4).
	drop := map[int]struct{}{}
	for _, s := range reach.slabs {
		drop[s.face] = struct{}{}
	}
	out := brepPayload{xform: cp.x.xform}
	for fi, f := range cp.x.faces {
		if _, gone := drop[fi]; gone {
			continue
		}
		out.faces = append(out.faces, f)
	}
	for _, s := range slabs {
		region := s.region
		out.faces = append(out.faces, brepFace{frame: cp.g, region: &region, outward: s.outward,
			z0: s.level, z1: s.level, z0Delta: s.levelDelta, z1Delta: s.levelDelta, delta: s.delta})
	}
	walls := func(loop LoopRecord, lo, hi end) {
		for _, seg := range loop.Segments {
			out.faces = append(out.faces, brepFace{frame: cp.g, wall: seg,
				z0: lo.level, z1: hi.level, z0Delta: lo.delta, z1Delta: hi.delta})
		}
	}
	section := cp.y.profile
	addCap := func(at end, outward bool) {
		region := section
		out.faces = append(out.faces, brepFace{frame: cp.g, region: &region, outward: outward,
			z0: at.level, z1: at.level, z0Delta: at.delta, z1Delta: at.delta})
	}
	switch op {
	case meshbool.OpCut:
		hole, err := reverseLoopRecordContext(ctx, section.Outer)
		if err != nil {
			return nil, false, err
		}
		walls(hole, inside[0], inside[1])
		if len(slabs) == 1 {
			// The floor faces back up the hole: outward toward the open end.
			if slabs[0].outward {
				addCap(inside[0], true)
			} else {
				addCap(inside[1], false)
			}
		}
	case meshbool.OpUnion:
		for _, part := range outside {
			walls(section.Outer, part[0], part[1])
		}
		if len(slabs) == 2 || !slabs[0].outward {
			addCap(a, false)
		}
		if len(slabs) == 2 || slabs[0].outward {
			addCap(b, true)
		}
	default:
		return nil, false, fmt.Errorf(`%w: class B has no %s`, ErrUnsupported, op)
	}
	out.assignRoles()
	return out, true, nil
}

// classBRectLoop is the counter-clockwise loop through four corners given in
// either winding.
func classBRectLoop(c []Point2) LoopRecord {
	area := 0.0
	for i := range c {
		j := (i + 1) % len(c)
		area += c[i].U*c[j].V - c[j].U*c[i].V
	}
	if area < 0 {
		c[1], c[3] = c[3], c[1]
	}
	var loop LoopRecord
	for i := range c {
		loop.Segments = append(loop.Segments, LineSeg{Start: c[i], End: c[(i+1)%len(c)], TStart: 0, TEnd: 1})
	}
	return loop
}

// classBPerpendicularRegion is the perpendicular-face scene (§5) of one slab
// face: its region and Y's section, both in g, resolved by prism-boolean's
// clean-nesting Cut match and authenticated through RecordProfile. matched
// is false when the scene returned no such match.
func classBPerpendicularRegion(ctx context.Context, cp classBPair, region ProfileRecord) (ProfileRecord, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	target := prismPayload{profile: region, frame: cp.g, xform: cp.x.xform, z0: 0, z1: 1}
	tool := prismPayload{profile: cp.y.profile, frame: cp.g, xform: cp.x.xform, z0: 0, z1: 1}
	segments, within, err := prismSceneWithinWorkCap(budget, target, tool)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	if !within {
		return ProfileRecord{}, false, fmt.Errorf(`%w: the class-B face scene charges at least %d arranger segments against this evaluator's cap of %d`,
			ErrUnsupported, segments, prismMaxArrangementSegments)
	}
	reexpress, err := newPrismReexpression(target, tool)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	s, match, sceneDelta, resolved, err := resolvePrismCut(ctx, budget, target, tool, reexpress)
	if err != nil || !resolved || sceneDelta.a != 0 || sceneDelta.b != 0 {
		return ProfileRecord{}, false, err
	}
	profile, err := prismRecordProfileContext(ctx, s, match)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	return profile, true, nil
}
