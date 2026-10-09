package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
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

func cbCarrierOf(op, index int) cbCarrier { return cbCarrier{op, index} }

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
	cp       classBPair
	x        prismPayload
	fX, fG   cbFrame
	fE       cbFrame
	vertices *classbgeom.VertexTable[cbCarrier]
	faces    []cbFace
	budget   *proofbound.WorkBudget
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
	b.vertices = classbgeom.NewVertexTable(func(c cbCarrier) classbgeom.CarrierGeometry {
		g := b.geom(c)
		return classbgeom.CarrierGeometry{
			Cylinder: g.cyl, Axis: g.axis, Level: g.level, Center: g.center, Segment: g.seg,
		}
	}, errCBMiss)
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
		w, _ := boundarywalk.WalkOf(seg, nil)
		return cbGeom{cyl: true, axis: frame.axis[2], seg: seg,
			center: classbgeom.ToX([3]float64{w.CU, w.CV, 0}, frame.axis, frame.sign)}
	}
}

// run builds every face, canonicalizes and splits them, adds the cylinder
// pieces, and assembles the brep.
func (b *cbBuild) run(ctx context.Context) (brepPayload, error) {
	frames := [3]classbgeom.CrossingFrame{
		{Frame: b.fX.frame, Axis: b.fX.axis, Sign: b.fX.sign},
		{Frame: b.fG.frame, Axis: b.fG.axis, Sign: b.fG.sign},
		{Frame: b.fE.frame, Axis: b.fE.axis, Sign: b.fE.sign},
	}
	x := classbgeom.CrossingPrism{Segments: b.x.profile.Outer.Segments, Z0: b.x.z0, Z1: b.x.z1}
	y := classbgeom.CrossingPrism{Segments: b.cp.y.profile.Outer.Segments, Z0: b.cp.y.z0, Z1: b.cp.y.z1}
	if err := b.vertices.SeedRecordedCorners([2]classbgeom.CrossingPrism{x, y},
		[2]classbgeom.CrossingFrame{frames[0], frames[1]}, cbCarrierOf); err != nil {
		return brepPayload{}, err
	}
	geometry := func(c cbCarrier) (int, float64) {
		g := b.geom(c)
		return g.axis, g.level
	}
	hooks := classbgeom.CrossingFaceHooks[cbCarrier]{
		Carrier:  cbCarrierOf,
		Geometry: geometry,
		Chords: func(ctx context.Context, op int, f cbCarrier) ([]cbChord, error) {
			p, frame := b.x, b.fX
			if op == cbY {
				p, frame = b.cp.y, b.fG
			}
			return classbgeom.TraceCrossingChords(ctx, op, f, p.profile.Outer.Segments,
				classbgeom.CrossingFrame{Frame: frame.frame, Axis: frame.axis, Sign: frame.sign},
				cbCarrierOf, geometry,
				func(f, c1, c2 cbCarrier, at [3]float64, delta float64) ([3]float64, error) {
					v, err := b.vertices.CanonicalPoint(f, c1, c2, at, delta)
					return v.Point, err
				}, errCBMiss)
		},
		Decide: func(ctx context.Context, c cbCarrier, frame int, level float64, outward bool,
			own []cbSeg, other [][]cbSeg, cut bool) error {
			return b.decide(ctx, c, []cbFrame{b.fX, b.fG, b.fE}[frame], level, outward, own, other, cut)
		},
		Miss: errCBMiss,
	}
	if err := classbgeom.BuildXCrossingFaces(ctx, x, y, b.e(), frames, hooks); err != nil {
		return brepPayload{}, err
	}
	if err := classbgeom.BuildYCrossingFaces(ctx, x, y, b.e(), frames, hooks); err != nil {
		return brepPayload{}, err
	}
	canonical := classbgeom.CanonicalFaceHooks[cbCarrier]{
		Cylinder: func(c cbCarrier) bool { return b.geom(c).cyl },
		Point: func(f, c1, c2 cbCarrier, at [3]float64, delta float64) ([3]float64, float64, error) {
			v, err := b.vertices.CanonicalPoint(f, c1, c2, at, delta)
			return v.Point, v.Delta, err
		},
		Record: func(at [3]float64, carriers []cbCarrier, delta float64) {
			b.vertices.Record(classbgeom.CrossingVertex[cbCarrier]{Point: at, CarrierList: carriers, Delta: delta})
		},
	}
	for i := range b.faces {
		f := &b.faces[i]
		frame := classbgeom.CrossingFrame{Axis: f.frame.axis, Sign: f.frame.sign}
		region, carriers, delta, err := classbgeom.CanonicalizeFace(
			f.region, f.carriers, f.carrier, frame, f.level, f.delta, canonical)
		if err != nil {
			return brepPayload{}, err
		}
		f.region, f.carriers, f.delta = region, carriers, delta
	}
	for i := range b.faces {
		f := &b.faces[i]
		frame := classbgeom.CrossingFrame{Axis: f.frame.axis, Sign: f.frame.sign}
		region, carriers, err := classbgeom.SplitCrossingFace(
			f.region, f.carriers, f.carrier, frame, f.level, b.vertices.Vertices,
			func(c cbCarrier) (bool, int) {
				g := b.geom(c)
				return g.cyl, g.axis
			}, errCBMiss)
		if err != nil {
			return brepPayload{}, err
		}
		f.region, f.carriers = region, carriers
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
	pieceFaces := make([]classbgeom.CrossingPieceFace[cbCarrier], len(b.faces))
	for i, f := range b.faces {
		pieceFaces[i] = classbgeom.CrossingPieceFace[cbCarrier]{
			Region: f.region, Carriers: f.carriers,
			Frame: classbgeom.CrossingFrame{Frame: f.frame.frame, Axis: f.frame.axis, Sign: f.frame.sign},
			Level: f.level, Delta: f.delta,
		}
	}
	pieces, err := classbgeom.CylinderPieces(pieceFaces, [2]classbgeom.CrossingFrame{frames[0], frames[1]},
		[2][]CurveSegment{b.x.profile.Outer.Segments, b.cp.y.profile.Outer.Segments},
		func(c cbCarrier) (int, int, bool, int) {
			g := b.geom(c)
			return c.op, c.idx, g.cyl, g.axis
		}, errCBMiss)
	if err != nil {
		return brepPayload{}, err
	}
	for _, piece := range pieces {
		out.faces = append(out.faces, brepFace{
			frame: piece.Frame, wall: piece.Wall, z0: piece.Z0, z1: piece.Z1, delta: piece.Delta,
		})
	}
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

// cbChord is one chord of a section along a trace line: its two ends along
// the line's free axis, in X's local axes, and the carrier each crosses.
type cbChord = classbgeom.CrossingChord[cbCarrier]

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
	if _, within, err := prismcells.RegionsWithinWorkCap(b.budget, target.profile, toolP.profile); err != nil || !within {
		if err != nil {
			return err
		}
		return fmt.Errorf(`%w: a class-B face scene exceeds this evaluator's arrangement cap of %d`,
			ErrUnsupported, prismcells.MaxArrangementSegments)
	}
	reexpress := &prismReexpression{Identity: true}
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
		if err := ig.AddFor(seg, freeformPlan{}, Point2{}, freeform.MomentAreaOrder); err != nil {
			return 0, err
		}
	}
	return ig.Area, nil
}
