package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the crossing reach of the class-B Cut
// (docs/general-boolean-design.md §3 B.3, §5): a pair whose scenes cut
// edges. Every planar face of either operand that the other reaches is
// decided by its own scene (§5's parallel- and perpendicular-face scenes),
// every crossing those scenes compute is replaced by the one float the keyed
// crossing table records for it (§10), every face edge is split at every
// vertex on it, and each cylinder's surviving pieces are read off the planar
// faces that bound them.

// cbCarrier names one surface of either operand: a cap (idx −1 at the sweep
// start, −2 at its end) or the wall of one outer-loop segment.
type cbCarrier struct{ op, idx int }

const (
	cbX = 0
	cbY = 1
)

// cbGeom is a carrier's geometry in X's local axes: a plane x[axis] == level,
// or a cylinder along axis about center with its section record seg.
type cbGeom struct {
	cyl    bool
	axis   int
	level  float64
	center [3]float64
	seg    CurveSegment
}

// cbFrame is one of the result's face frames: frame's local axis i lands on
// X's axis axis[i] with sign sign[i]. Every frame shares X's origin.
type cbFrame struct {
	frame r3.Frame
	axis  [3]int
	sign  [3]float64
}

func (f cbFrame) toLocal(x [3]float64) [3]float64 {
	return classbgeom.ToLocal(x, f.axis, f.sign)
}

func (f cbFrame) toX(l [3]float64) [3]float64 {
	return classbgeom.ToX(l, f.axis, f.sign)
}

// cbFace is one planar face of the result while it is built: its carrier,
// its frame and level, which side is outward, its region in frame
// coordinates, the carrier of every segment, and its displacement.
type cbFace struct {
	carrier  cbCarrier
	frame    cbFrame
	level    float64
	outward  bool
	region   ProfileRecord
	carriers [][]cbCarrier
	delta    float64
}

// cbBuild is one crossing-reach attempt: the admitted pair, its frames, the
// keyed crossing table, every canonical vertex found, and the faces.
type cbBuild struct {
	cp     classBPair
	x      prismPayload
	fX, fG cbFrame
	fE     cbFrame
	table  map[cbKey2]cbEntry
	verts  map[[3]float64]*cbVertex
	faces  []cbFace
	budget *proofbound.WorkBudget
}

// errCBMiss marks a topology this increment's reach does not cover, decided
// after a scene was built: the caller treats it as a silent miss
// (prism-boolean §3.4).
var errCBMiss = fmt.Errorf(`%w: the class-B crossing reach does not cover this pair`, ErrUnsupported)

// tryClassBCrossingCut attempts the crossing reach on an admitted pair. ok
// false with a nil error is a silent miss.
func tryClassBCrossingCut(ctx context.Context, x prismPayload, cp classBPair) (brepPayload, bool, error) {
	b, ok, err := newCBBuild(ctx, x, cp)
	if err != nil || !ok {
		return brepPayload{}, false, err
	}
	bp, err := b.run(ctx)
	if err == errCBMiss {
		return brepPayload{}, false, nil
	}
	if err != nil {
		return brepPayload{}, false, err
	}
	return bp, true, nil
}

// newCBBuild runs the reach's own entry conditions beyond B1–B8, each a
// silent miss: both operands hole-free, every level displacement zero, and a
// frame for faces normal to e that carries X's axes bit for bit.
func newCBBuild(ctx context.Context, x prismPayload, cp classBPair) (*cbBuild, bool, error) {
	if x.frame != cp.ref || cp.d() == 2 {
		return nil, false, nil
	}
	if len(x.profile.Holes) != 0 || len(cp.y.profile.Holes) != 0 {
		return nil, false, nil
	}
	if x.z0Delta != 0 || x.z1Delta != 0 || cp.y.z0Delta != 0 || cp.y.z1Delta != 0 {
		return nil, false, nil
	}
	axes := [3]r3.Vec{x.frame.U(), x.frame.V(), x.frame.N()}
	d := cp.d()
	fe, err := r3.NewFrame(x.frame.Origin(), axes[d], axes[2])
	if err != nil {
		return nil, false, fmt.Errorf(`%w: X's axes make no frame: %s`, ErrDegenerate, err)
	}
	b := &cbBuild{
		cp:     cp,
		x:      x,
		fX:     cbFrame{frame: x.frame, axis: [3]int{0, 1, 2}, sign: [3]float64{1, 1, 1}},
		fG:     cbFrame{frame: cp.g, axis: cp.axis, sign: cp.sign},
		table:  map[cbKey2]cbEntry{},
		verts:  map[[3]float64]*cbVertex{},
		budget: proofbound.NewWorkBudget(ctx),
	}
	b.fE = cbFrame{frame: fe, axis: [3]int{d, 2, b.e()}, sign: [3]float64{1, 1, 0}}
	switch {
	case fe.U() != axes[d] || fe.V() != axes[2]:
		return nil, false, nil
	case fe.N() == axes[b.e()]:
		b.fE.sign[2] = 1
	case fe.N() == axes[b.e()].Scale(-1):
		b.fE.sign[2] = -1
	default:
		return nil, false, nil
	}
	return b, true, nil
}

// e is X's in-plane axis other than d.
func (b *cbBuild) e() int { return 1 - b.cp.d() }

// geom states a carrier's geometry in X's local axes.
func (b *cbBuild) geom(c cbCarrier) cbGeom {
	op := b.x
	frame := b.fX
	if c.op == cbY {
		op, frame = b.cp.y, b.fG
	}
	if c.idx < 0 {
		z := op.z0
		if c.idx == -2 {
			z = op.z1
		}
		return cbGeom{axis: frame.axis[2], level: frame.sign[2]*z + 0}
	}
	seg := op.profile.Outer.Segments[c.idx]
	switch s := seg.(type) {
	case LineSeg:
		if s.Start.U == s.End.U {
			return cbGeom{axis: frame.axis[0], level: frame.sign[0]*s.Start.U + 0}
		}
		return cbGeom{axis: frame.axis[1], level: frame.sign[1]*s.Start.V + 0}
	default:
		w, _ := walkOf(seg, nil)
		return cbGeom{cyl: true, axis: frame.axis[2], seg: seg, center: frame.toX([3]float64{w.CU, w.CV, 0})}
	}
}

// run builds every face, canonicalizes and splits them, adds the cylinder
// pieces, and assembles the brep.
func (b *cbBuild) run(ctx context.Context) (brepPayload, error) {
	if err := b.seedRecordedCorners(); err != nil {
		return brepPayload{}, err
	}
	if err := b.buildXFaces(ctx); err != nil {
		return brepPayload{}, err
	}
	if err := b.buildYFaces(ctx); err != nil {
		return brepPayload{}, err
	}
	for i := range b.faces {
		if err := b.canonicalize(&b.faces[i]); err != nil {
			return brepPayload{}, err
		}
	}
	for i := range b.faces {
		if err := b.split(&b.faces[i]); err != nil {
			return brepPayload{}, err
		}
	}
	// Modify §5's audit per planar face record (§5): simplicity, orientation
	// and nesting. It refuses; it admits nothing.
	for _, f := range b.faces {
		if err := auditPrismMergeSection(b.budget, prismPayload{profile: f.region}, f.region); err != nil {
			return brepPayload{}, err
		}
	}
	out := brepPayload{xform: b.x.xform}
	// The reference frame is X's: the first face's frame must be it.
	for _, f := range b.faces {
		if f.frame.axis == b.fX.axis && f.frame.sign == b.fX.sign {
			out.faces = append(out.faces, b.brepFace(f))
		}
	}
	if len(out.faces) == 0 {
		return brepPayload{}, errCBMiss
	}
	for _, f := range b.faces {
		if f.frame.axis != b.fX.axis || f.frame.sign != b.fX.sign {
			out.faces = append(out.faces, b.brepFace(f))
		}
	}
	pieces, err := b.cylinderPieces()
	if err != nil {
		return brepPayload{}, err
	}
	out.faces = append(out.faces, pieces...)
	out.assignRoles()
	return out, nil
}

// brepFace states a built face in the record. A face on a wall's carrier
// records that operand's sweep, X's axis along which the wall runs; a cap's
// records none (§4.2).
func (b *cbBuild) brepFace(f cbFace) brepFace {
	region := f.region
	out := brepFace{frame: f.frame.frame, region: &region, outward: f.outward, z0: f.level, z1: f.level, delta: f.delta}
	if f.carrier.idx >= 0 {
		axes := [3]r3.Vec{b.x.frame.U(), b.x.frame.V(), b.x.frame.N()}
		sweep := b.fX.axis[2]
		if f.carrier.op == cbY {
			sweep = b.fG.axis[2]
		}
		out.sweep = axes[sweep]
	}
	return out
}

// cbSeg is one input segment of a face scene with its carrier, in the
// face's frame coordinates.
type cbSeg = classbgeom.CarrierSegment[cbCarrier]

// cbRectLoop builds a loop through corners given in X's local axes, each edge on
// its carrier, counter-clockwise in frame.
func cbRectLoop(frame cbFrame, corners [][3]float64, carriers []cbCarrier) []cbSeg {
	return classbgeom.RectLoop(frame.axis, frame.sign, corners, carriers)
}

// cbSectionSegs lists an operand's own section in its own frame, each
// segment on its own wall.
func cbSectionSegs(op int, p prismPayload) []cbSeg {
	var out []cbSeg
	for i, seg := range p.profile.Outer.Segments {
		out = append(out, cbSeg{Seg: seg, Carrier: cbCarrier{op, i}})
	}
	return out
}

// cbChord is one chord of a section along a trace line: its two ends along
// the line's free axis, in X's local axes, and the carrier each crosses.
type cbChord struct {
	lo, hi   [3]float64
	cLo, cHi cbCarrier
}

// chords asks sketch for the chords of an operand's section along the trace
// of carrier f, a plane parallel to the operand's sweep: the section's
// loops and one line through the whole section box, arranged in one private
// scene. The fragments of that line that bound the returned cells are the
// chords; each end is canonicalized as a vertex of f, the crossed carrier,
// and each of the other operand's two caps, which is where the chord's
// rectangle has its corners.
func (b *cbBuild) chords(ctx context.Context, op int, f cbCarrier) ([]cbChord, error) {
	p, frame := b.x, b.fX
	if op == cbY {
		p, frame = b.cp.y, b.fG
	}
	plane := b.geom(f)
	// The trace's own axis in the section's frame, and the free one.
	var fixedLocal int
	for i := range 2 {
		if frame.axis[i] == plane.axis {
			fixedLocal = i
		}
	}
	fixedValue := frame.sign[fixedLocal] * plane.level
	traces, trace, err := classbgeom.TraceChords(ctx, cbSectionSegs(op, p), fixedLocal, fixedValue, f, errCBMiss)
	if err != nil {
		return nil, err
	}
	// The chord sweeps over op's own interval, so its rectangle's corners sit
	// on op's own two caps.
	caps := []cbCarrier{{op, -1}, {op, -2}}
	var out []cbChord
	for _, t := range traces {
		var chord cbChord
		for k := range 2 {
			end := t.Ends[k]
			x := frame.toX([3]float64{end.U, end.V, 0})
			delta := proofbound.AbsSumUpper(proofbound.CutDisplacementAllow(classBSpeed(trace)),
				proofbound.WalkEndBoundAllow(t.Bounds[k]))
			var vx cbVertex
			for _, c := range caps {
				var err error
				// The chord end fills the table; it becomes a vertex only
				// where a face's region keeps it.
				vx, err = b.canonicalPoint(f, t.Crossed[k], c, b.withAxial(x, c), delta)
				if err != nil {
					return nil, err
				}
			}
			if k == 0 {
				chord.lo, chord.cLo = vx.point, t.Crossed[k]
			} else {
				chord.hi, chord.cHi = vx.point, t.Crossed[k]
			}
		}
		out = append(out, chord)
	}
	return out, nil
}

// withAxial places x on the plane of cap carrier c.
func (b *cbBuild) withAxial(x [3]float64, c cbCarrier) [3]float64 {
	g := b.geom(c)
	x[g.axis] = g.level
	return x
}

// classBSpeed bounds a segment's carrier speed.
func classBSpeed(seg CurveSegment) float64 {
	speed, err := prismcells.CarrierSpeedUpper(seg)
	if err != nil {
		return math.Inf(1)
	}
	return speed
}

// buildXFaces decides every planar face of X: both caps, against the chords
// of Y's section along each cap's trace; each line wall along d, against the
// chords along its trace; each line wall across d, against Y's whole section
// where Y's sweep crosses it. Each keeps what lies outside Y.
func (b *cbBuild) buildXFaces(ctx context.Context) error {
	x := b.x
	segs := x.profile.Outer.Segments
	n := len(segs)
	capCorners := cbSectionSegs(cbX, x)
	for _, end := range []int{-1, -2} {
		f := cbCarrier{cbX, end}
		level := x.z0
		if end == -2 {
			level = x.z1
		}
		chords, err := b.chords(ctx, cbY, f)
		if err != nil {
			return err
		}
		other, err := b.chordRects(b.fX, chords, cbY)
		if err != nil {
			return err
		}
		if err := b.decide(ctx, f, b.fX, level, end == -2, capCorners, other, true); err != nil {
			return err
		}
	}
	for i, seg := range segs {
		line, ok := seg.(LineSeg)
		if !ok {
			continue
		}
		f := cbCarrier{cbX, i}
		g := b.geom(f)
		prev, next := cbCarrier{cbX, (i + n - 1) % n}, cbCarrier{cbX, (i + 1) % n}
		start := [3]float64{line.Start.U, line.Start.V, 0}
		end := [3]float64{line.End.U, line.End.V, 0}
		corners := [][3]float64{
			{start[0], start[1], x.z0}, {end[0], end[1], x.z0},
			{end[0], end[1], x.z1}, {start[0], start[1], x.z1},
		}
		carriers := []cbCarrier{{cbX, -1}, next, {cbX, -2}, prev}
		outward := b.lineOutward(line)
		if g.axis == b.e() {
			// A wall along d: its plane is e = level.
			own := cbRectLoop(b.fE, corners, carriers)
			chords, err := b.chords(ctx, cbY, f)
			if err != nil {
				return err
			}
			other, err := b.chordRects(b.fE, chords, cbY)
			if err != nil {
				return err
			}
			if err := b.decide(ctx, f, b.fE, b.fE.toLocal(corners[0])[2], b.outwardIn(b.fE, outward), own, other, true); err != nil {
				return err
			}
			continue
		}
		// A wall across d: its plane is d = level, normal to Y's sweep.
		own := cbRectLoop(b.fG, corners, carriers)
		var other [][]cbSeg
		if b.strictlyInside(g.level, b.cp.y, b.fG) {
			other = [][]cbSeg{cbSectionSegs(cbY, b.cp.y)}
		}
		if err := b.decide(ctx, f, b.fG, b.fG.toLocal(corners[0])[2], b.outwardIn(b.fG, outward), own, other, true); err != nil {
			return err
		}
	}
	return nil
}

// buildYFaces decides every planar face of Y the same way, keeping what lies
// inside X; its outward side faces into Y, away from the result.
func (b *cbBuild) buildYFaces(ctx context.Context) error {
	y := b.cp.y
	segs := y.profile.Outer.Segments
	n := len(segs)
	for _, end := range []int{-1, -2} {
		f := cbCarrier{cbY, end}
		level := y.z0
		if end == -2 {
			level = y.z1
		}
		chords, err := b.chords(ctx, cbX, f)
		if err != nil {
			return err
		}
		if len(chords) == 0 {
			continue
		}
		other, err := b.chordRects(b.fG, chords, cbX)
		if err != nil {
			return err
		}
		// Y's material lies inside its interval: toward +N_g at its start.
		if err := b.decide(ctx, f, b.fG, level, end == -1, cbSectionSegs(cbY, y), other, false); err != nil {
			return err
		}
	}
	for i, seg := range segs {
		line, ok := seg.(LineSeg)
		if !ok {
			continue
		}
		f := cbCarrier{cbY, i}
		g := b.geom(f)
		prev, next := cbCarrier{cbY, (i + n - 1) % n}, cbCarrier{cbY, (i + 1) % n}
		at := func(p Point2, z float64) [3]float64 { return b.fG.toX([3]float64{p.U, p.V, z}) }
		corners := [][3]float64{at(line.Start, y.z0), at(line.End, y.z0), at(line.End, y.z1), at(line.Start, y.z1)}
		carriers := []cbCarrier{{cbY, -1}, next, {cbY, -2}, prev}
		// Y's material is left of its walk, so the result's outward side is
		// the walk's left: the reverse of the line's own outward normal.
		inward := b.lineOutwardG(line)
		for k := range inward {
			inward[k] = -inward[k] + 0
		}
		if g.axis == b.e() {
			own := cbRectLoop(b.fE, corners, carriers)
			chords, err := b.chords(ctx, cbX, f)
			if err != nil {
				return err
			}
			if len(chords) == 0 {
				continue
			}
			other, err := b.chordRects(b.fE, chords, cbX)
			if err != nil {
				return err
			}
			if err := b.decide(ctx, f, b.fE, b.fE.toLocal(corners[0])[2], b.outwardIn(b.fE, inward), own, other, false); err != nil {
				return err
			}
			continue
		}
		// A wall across w: its plane is w = level, normal to X's sweep.
		if !b.strictlyInside(g.level, b.x, b.fX) {
			continue
		}
		own := cbRectLoop(b.fX, corners, carriers)
		other := [][]cbSeg{cbSectionSegs(cbX, b.x)}
		if err := b.decide(ctx, f, b.fX, g.level, b.outwardIn(b.fX, inward), own, other, false); err != nil {
			return err
		}
	}
	return nil
}

// strictlyInside reports whether level, on op's own sweep axis in X's
// local axes, lies strictly inside op's sweep interval, compared exactly.
func (b *cbBuild) strictlyInside(level float64, op prismPayload, frame cbFrame) bool {
	lo := frame.sign[2]*op.z0 + 0
	hi := frame.sign[2]*op.z1 + 0
	if lo > hi {
		lo, hi = hi, lo
	}
	return lo < level && level < hi
}

// lineOutward is the outward normal, in X's local axes, of X's line wall:
// the walk's right side, since X's material lies on its left.
func (b *cbBuild) lineOutward(line LineSeg) [3]float64 {
	du, dv := line.End.U-line.Start.U, line.End.V-line.Start.V
	return [3]float64{dv, -du, 0}
}

// lineOutwardG is lineOutward for one of Y's lines, read in g and placed in
// X's local axes.
func (b *cbBuild) lineOutwardG(line LineSeg) [3]float64 {
	du, dv := line.End.U-line.Start.U, line.End.V-line.Start.V
	return b.fG.toX([3]float64{dv, -du, 0})
}

// outwardIn reports whether a direction in X's local axes points along
// frame's +N.
func (b *cbBuild) outwardIn(frame cbFrame, dir [3]float64) bool {
	return frame.toLocal(dir)[2] > 0
}

// chordRects turns each chord of op's section along f's trace into the
// rectangle it sweeps over op's own interval, in frame, each edge on its
// carrier: the chord's two crossed carriers and op's two caps.
func (b *cbBuild) chordRects(frame cbFrame, chords []cbChord, op int) ([][]cbSeg, error) {
	if len(chords) > 1 {
		return nil, errCBMiss
	}
	var out [][]cbSeg
	for _, c := range chords {
		lowCap, highCap := cbCarrier{op, -1}, cbCarrier{op, -2}
		corners := [][3]float64{b.withAxial(c.lo, lowCap), b.withAxial(c.hi, lowCap), b.withAxial(c.hi, highCap), b.withAxial(c.lo, highCap)}
		segs := cbRectLoop(frame, corners, []cbCarrier{lowCap, c.cHi, highCap, c.cLo})
		out = append(out, segs)
	}
	return out, nil
}

// decide resolves one planar face's scene (§5): A is the face's own loop,
// B the other operand's slice of it, both in frame. A Cut keeps A outside B,
// an Intersect A inside B, through prism-boolean's clean-nesting match or
// crossing sub-case. Each returned region becomes one face; an empty one
// drops the face.
func (b *cbBuild) decide(ctx context.Context, f cbCarrier, frame cbFrame, level float64, outward bool, own []cbSeg, other [][]cbSeg, cut bool) error {
	toLoop := func(segs []cbSeg) LoopRecord {
		var l LoopRecord
		for _, s := range segs {
			l.Segments = append(l.Segments, s.Seg)
		}
		return l
	}
	inputs := slices.Clone(own)
	for _, o := range other {
		inputs = append(inputs, o...)
	}
	add := func(region ProfileRecord) error {
		carriers, ok := classbgeom.CarriersOf(append([]LoopRecord{region.Outer}, region.Holes...), inputs)
		if !ok {
			return errCBMiss
		}
		b.faces = append(b.faces, cbFace{carrier: f, frame: frame, level: level, outward: outward,
			region: region, carriers: carriers})
		return nil
	}
	a := ProfileRecord{Outer: toLoop(own)}
	if len(other) == 0 {
		if cut {
			return add(a)
		}
		return nil
	}
	tool := ProfileRecord{Outer: toLoop(other[0])}
	target := prismPayload{profile: a, frame: frame.frame, xform: b.x.xform, z0: 0, z1: 1}
	toolP := prismPayload{profile: tool, frame: frame.frame, xform: b.x.xform, z0: 0, z1: 1}
	if _, within, err := prismSceneWithinWorkCap(b.budget, target, toolP); err != nil || !within {
		if err != nil {
			return err
		}
		return fmt.Errorf(`%w: a class-B face scene exceeds this evaluator's arrangement cap of %d`, ErrUnsupported, prismMaxArrangementSegments)
	}
	reexpress := &prismReexpression{identity: true}
	if cut {
		s, match, _, resolved, err := resolvePrismCut(ctx, b.budget, target, toolP, reexpress)
		if err != nil {
			return err
		}
		if resolved {
			region, err := prismRecordProfileContext(ctx, s, match)
			if err != nil {
				return err
			}
			return add(region)
		}
	} else {
		s, match, _, _, resolved, err := resolvePrismIntersect(ctx, b.budget, target, toolP, reexpress)
		if err != nil {
			return err
		}
		if resolved {
			region, err := prismRecordProfileContext(ctx, s, match)
			if err != nil {
				return err
			}
			return add(region)
		}
	}
	keep := func(inA, inB bool) bool { return inA && !inB }
	if !cut {
		keep = func(inA, inB bool) bool { return inA && inB }
	}
	selected, _, resolved, err := resolvePrismCrossingCells(ctx, b.budget, target, toolP, reexpress, keep)
	if err != nil {
		return err
	}
	if !resolved {
		return errCBMiss
	}
	if len(selected) == 0 {
		return nil
	}
	loops, _, merged, err := prismcells.MergeLoops(b.budget, selected, "class-B face")
	if err != nil {
		return err
	}
	if !merged {
		return errCBMiss
	}
	for _, loop := range loops {
		area, err := loopSignedAreaCB(loop)
		if err != nil {
			return err
		}
		if area <= 0 {
			// A hole in a crossing result has no outer to own it here.
			return errCBMiss
		}
		if err := add(ProfileRecord{Outer: loop}); err != nil {
			return err
		}
	}
	return nil
}

// loopSignedAreaCB is a loop's signed area, counter-clockwise positive.
func loopSignedAreaCB(loop LoopRecord) (float64, error) {
	var ig regionIntegrals
	for _, seg := range loop.Segments {
		if err := ig.add(seg, freeformPlan{}, Point2{}, freeform.MomentAreaOrder); err != nil {
			return 0, err
		}
	}
	return ig.area, nil
}
