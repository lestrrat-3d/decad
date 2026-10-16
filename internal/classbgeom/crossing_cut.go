package classbgeom

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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
	seg    sectionrecord.CurveSegment
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
	region   momentinput.Profile
	carriers [][]cbCarrier
	delta    float64
}

// cbBuild is one crossing-reach attempt: the admitted pair, its frames, the
// keyed crossing table, every canonical vertex found, and the faces.
type cbBuild struct {
	cp       Pair
	x        prismrecord.Payload
	fX, fG   cbFrame
	fE       cbFrame
	vertices *VertexTable[cbCarrier]
	faces    []cbFace
	budget   *proofbound.WorkBudget
}

// errCBMiss marks a topology this increment's reach does not cover, decided
// after a scene was built: the caller treats it as a silent miss
// (prism-boolean §3.4).
var errCBMiss = fmt.Errorf(`%w: the class-B crossing reach does not cover this pair`, decaderr.ErrUnsupported)

// tryClassBCrossingCut attempts the crossing reach on an admitted pair. ok
// false with a nil error is a silent miss.
func tryClassBCrossingCut(ctx context.Context, x prismrecord.Payload, cp Pair) (breprecord.Payload, bool, error) {
	b, ok, err := newCBBuild(ctx, x, cp)
	if err != nil || !ok {
		return breprecord.Payload{}, false, err
	}
	bp, err := b.run(ctx)
	if err == errCBMiss {
		return breprecord.Payload{}, false, nil
	}
	if err != nil {
		return breprecord.Payload{}, false, err
	}
	return bp, true, nil
}

// newCBBuild runs the reach's own entry conditions beyond B1–B8, each a
// silent miss: both operands hole-free, every level displacement zero, and a
// frame for faces normal to e that carries X's axes bit for bit.
func newCBBuild(ctx context.Context, x prismrecord.Payload, cp Pair) (*cbBuild, bool, error) {
	if x.Frame != cp.ref || cp.d() == 2 {
		return nil, false, nil
	}
	if len(x.Profile.Holes) != 0 || len(cp.y.Profile.Holes) != 0 {
		return nil, false, nil
	}
	if x.Z0Delta != 0 || x.Z1Delta != 0 || cp.y.Z0Delta != 0 || cp.y.Z1Delta != 0 {
		return nil, false, nil
	}
	axes := [3]r3.Vec{x.Frame.U(), x.Frame.V(), x.Frame.N()}
	d := cp.d()
	fe, err := r3.NewFrame(x.Frame.Origin(), axes[d], axes[2])
	if err != nil {
		return nil, false, fmt.Errorf(`%w: X's axes make no frame: %s`, decaderr.ErrDegenerate, err)
	}
	b := &cbBuild{
		cp:     cp,
		x:      x,
		fX:     cbFrame{frame: x.Frame, axis: [3]int{0, 1, 2}, sign: [3]float64{1, 1, 1}},
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
	b.vertices = NewVertexTable(func(c cbCarrier) CarrierGeometry {
		g := b.geom(c)
		return CarrierGeometry{
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
		z := op.Z0
		if c.idx == -2 {
			z = op.Z1
		}
		return cbGeom{axis: frame.axis[2], level: frame.sign[2]*z + 0}
	}
	seg := op.Profile.Outer.Segments[c.idx]
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		if s.Start.U == s.End.U {
			return cbGeom{axis: frame.axis[0], level: frame.sign[0]*s.Start.U + 0}
		}
		return cbGeom{axis: frame.axis[1], level: frame.sign[1]*s.Start.V + 0}
	default:
		w, _ := boundarywalk.WalkOf(seg, nil)
		return cbGeom{cyl: true, axis: frame.axis[2], seg: seg,
			center: ToX([3]float64{w.CU, w.CV, 0}, frame.axis, frame.sign)}
	}
}

// run builds every face, canonicalizes and splits them, adds the cylinder
// pieces, and assembles the brep.
func (b *cbBuild) run(ctx context.Context) (breprecord.Payload, error) {
	frames := [3]CrossingFrame{
		{Frame: b.fX.frame, Axis: b.fX.axis, Sign: b.fX.sign},
		{Frame: b.fG.frame, Axis: b.fG.axis, Sign: b.fG.sign},
		{Frame: b.fE.frame, Axis: b.fE.axis, Sign: b.fE.sign},
	}
	x := CrossingPrism{Segments: b.x.Profile.Outer.Segments, Z0: b.x.Z0, Z1: b.x.Z1}
	y := CrossingPrism{Segments: b.cp.y.Profile.Outer.Segments, Z0: b.cp.y.Z0, Z1: b.cp.y.Z1}
	if err := b.vertices.SeedRecordedCorners([2]CrossingPrism{x, y},
		[2]CrossingFrame{frames[0], frames[1]}, cbCarrierOf); err != nil {
		return breprecord.Payload{}, err
	}
	geometry := func(c cbCarrier) (int, float64) {
		g := b.geom(c)
		return g.axis, g.level
	}
	hooks := CrossingFaceHooks[cbCarrier]{
		Carrier:  cbCarrierOf,
		Geometry: geometry,
		Chords: func(ctx context.Context, op int, f cbCarrier) ([]cbChord, error) {
			p, frame := b.x, b.fX
			if op == cbY {
				p, frame = b.cp.y, b.fG
			}
			return TraceCrossingChords(ctx, op, f, p.Profile.Outer.Segments,
				CrossingFrame{Frame: frame.frame, Axis: frame.axis, Sign: frame.sign},
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
	if err := BuildXCrossingFaces(ctx, x, y, b.e(), frames, hooks); err != nil {
		return breprecord.Payload{}, err
	}
	if err := BuildYCrossingFaces(ctx, x, y, b.e(), frames, hooks); err != nil {
		return breprecord.Payload{}, err
	}
	canonical := CanonicalFaceHooks[cbCarrier]{
		Cylinder: func(c cbCarrier) bool { return b.geom(c).cyl },
		Point: func(f, c1, c2 cbCarrier, at [3]float64, delta float64) ([3]float64, float64, error) {
			v, err := b.vertices.CanonicalPoint(f, c1, c2, at, delta)
			return v.Point, v.Delta, err
		},
		Record: func(at [3]float64, carriers []cbCarrier, delta float64) {
			b.vertices.Record(CrossingVertex[cbCarrier]{Point: at, CarrierList: carriers, Delta: delta})
		},
	}
	for i := range b.faces {
		f := &b.faces[i]
		frame := CrossingFrame{Axis: f.frame.axis, Sign: f.frame.sign}
		region, carriers, delta, err := CanonicalizeFace(
			f.region, f.carriers, f.carrier, frame, f.level, f.delta, canonical)
		if err != nil {
			return breprecord.Payload{}, err
		}
		f.region, f.carriers, f.delta = region, carriers, delta
	}
	for i := range b.faces {
		f := &b.faces[i]
		frame := CrossingFrame{Axis: f.frame.axis, Sign: f.frame.sign}
		region, carriers, err := SplitCrossingFace(
			f.region, f.carriers, f.carrier, frame, f.level, b.vertices.Vertices,
			func(c cbCarrier) (bool, int) {
				g := b.geom(c)
				return g.cyl, g.axis
			}, errCBMiss)
		if err != nil {
			return breprecord.Payload{}, err
		}
		f.region, f.carriers = region, carriers
	}
	// Modify §5's audit per planar face record (§5): simplicity, orientation
	// and nesting. It refuses; it admits nothing.
	for _, f := range b.faces {
		if err := prismcells.AuditMergeSection(b.budget, prismrecord.Payload{Profile: f.region}, f.region); err != nil {
			return breprecord.Payload{}, err
		}
	}
	out := breprecord.Payload{Xform: b.x.Xform}
	// The reference frame is X's: the first face's frame must be it.
	for _, f := range b.faces {
		if f.frame.axis == b.fX.axis && f.frame.sign == b.fX.sign {
			out.Faces = append(out.Faces, b.brepFace(f))
		}
	}
	if len(out.Faces) == 0 {
		return breprecord.Payload{}, errCBMiss
	}
	for _, f := range b.faces {
		if f.frame.axis != b.fX.axis || f.frame.sign != b.fX.sign {
			out.Faces = append(out.Faces, b.brepFace(f))
		}
	}
	pieceFaces := make([]CrossingPieceFace[cbCarrier], len(b.faces))
	for i, f := range b.faces {
		pieceFaces[i] = CrossingPieceFace[cbCarrier]{
			Region: f.region, Carriers: f.carriers,
			Frame: CrossingFrame{Frame: f.frame.frame, Axis: f.frame.axis, Sign: f.frame.sign},
			Level: f.level, Delta: f.delta,
		}
	}
	pieces, err := CylinderPieces(pieceFaces, [2]CrossingFrame{frames[0], frames[1]},
		[2][]sectionrecord.CurveSegment{b.x.Profile.Outer.Segments, b.cp.y.Profile.Outer.Segments},
		func(c cbCarrier) (int, int, bool, int) {
			g := b.geom(c)
			return c.op, c.idx, g.cyl, g.axis
		}, errCBMiss)
	if err != nil {
		return breprecord.Payload{}, err
	}
	for _, piece := range pieces {
		out.Faces = append(out.Faces, breprecord.Face{
			Frame: piece.Frame, Wall: piece.Wall, Z0: piece.Z0, Z1: piece.Z1, Delta: piece.Delta,
		})
	}
	out.AssignRoles()
	return out, nil
}

// brepFace states a built face in the record. A face on a wall's carrier
// records that operand's sweep, X's axis along which the wall runs; a cap's
// records none (§4.2).
func (b *cbBuild) brepFace(f cbFace) breprecord.Face {
	region := f.region
	out := breprecord.Face{Frame: f.frame.frame, Region: &region, Outward: f.outward, Z0: f.level, Z1: f.level, Delta: f.delta}
	if f.carrier.idx >= 0 {
		axes := [3]r3.Vec{b.x.Frame.U(), b.x.Frame.V(), b.x.Frame.N()}
		sweep := b.fX.axis[2]
		if f.carrier.op == cbY {
			sweep = b.fG.axis[2]
		}
		out.Sweep = axes[sweep]
	}
	return out
}

// cbSeg is one input segment of a face scene with its carrier, in the
// face's frame coordinates.
type cbSeg = CarrierSegment[cbCarrier]

// cbChord is one chord of a section along a trace line: its two ends along
// the line's free axis, in X's local axes, and the carrier each crosses.
type cbChord = CrossingChord[cbCarrier]

// decide resolves one planar face's scene (§5): A is the face's own loop,
// B the other operand's slice of it, both in frame. A Cut keeps A outside B,
// an Intersect A inside B, through prism-boolean's clean-nesting match or
// crossing sub-case. Each returned region becomes one face; an empty one
// drops the face.
func (b *cbBuild) decide(ctx context.Context, f cbCarrier, frame cbFrame, level float64, outward bool, own []cbSeg, other [][]cbSeg, cut bool) error {
	toLoop := func(segs []cbSeg) sectionrecord.LoopRecord {
		var l sectionrecord.LoopRecord
		for _, s := range segs {
			l.Segments = append(l.Segments, s.Seg)
		}
		return l
	}
	inputs := slices.Clone(own)
	for _, o := range other {
		inputs = append(inputs, o...)
	}
	add := func(region momentinput.Profile) error {
		carriers, ok := CarriersOf(append([]sectionrecord.LoopRecord{region.Outer}, region.Holes...), inputs)
		if !ok {
			return errCBMiss
		}
		b.faces = append(b.faces, cbFace{carrier: f, frame: frame, level: level, outward: outward,
			region: region, carriers: carriers})
		return nil
	}
	a := momentinput.Profile{Outer: toLoop(own)}
	if len(other) == 0 {
		if cut {
			return add(a)
		}
		return nil
	}
	tool := momentinput.Profile{Outer: toLoop(other[0])}
	target := prismrecord.Payload{Profile: a}
	toolP := prismrecord.Payload{Profile: tool}
	if _, within, err := prismcells.RegionsWithinWorkCap(b.budget, target.Profile, toolP.Profile); err != nil || !within {
		if err != nil {
			return err
		}
		return fmt.Errorf(`%w: a class-B face scene exceeds this evaluator's arrangement cap of %d`,
			decaderr.ErrUnsupported, prismcells.MaxArrangementSegments)
	}
	reexpress := &prismcells.Reexpression{Identity: true}
	if cut {
		s, match, _, resolved, err := prismcells.ResolveCut(ctx, b.budget, target.Profile, toolP.Profile, reexpress)
		if err != nil {
			return err
		}
		if resolved {
			region, err := prismcells.RecordProfileContext(ctx, s, match)
			if err != nil {
				return err
			}
			return add(region)
		}
	} else {
		s, match, _, _, resolved, err := prismcells.ResolveIntersect(ctx, b.budget, target.Profile, toolP.Profile, reexpress)
		if err != nil {
			return err
		}
		if resolved {
			region, err := prismcells.RecordProfileContext(ctx, s, match)
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
	selected, _, resolved, err := prismcells.ResolveCrossingCells(ctx, b.budget, target.Profile, toolP.Profile,
		target.SectionDelta, toolP.SectionDelta, reexpress, keep)
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
		area, err := LoopSignedArea(loop)
		if err != nil {
			return err
		}
		if area <= 0 {
			// A hole in a crossing result has no outer to own it here.
			return errCBMiss
		}
		if err := add(momentinput.Profile{Outer: loop}); err != nil {
			return err
		}
	}
	return nil
}

// LoopSignedArea is a loop's signed area, counter-clockwise positive.
func LoopSignedArea(loop sectionrecord.LoopRecord) (float64, error) {
	var ig momentinput.Integrals
	for _, seg := range loop.Segments {
		if err := ig.AddFor(seg, momentinput.Plan{}, sectionrecord.Point2{}, freeform.MomentAreaOrder); err != nil {
			return 0, err
		}
	}
	return ig.Area, nil
}
