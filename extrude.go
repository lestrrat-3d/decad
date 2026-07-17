package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the extrude of docs/evaluator-design.md §5: the feature call
// gates its live inputs, records the step, evaluates FROM the record, and
// commits atomically. The prism it builds is fully analytic — Plane and
// Cylinder faces, Exact measurements with zero bounds. The body-relative
// stops (ThroughAll/ThroughAllSide/ToFace) resolve through stops.go: the
// stop bodies are resolved at the call and recorded as StepRefs in the
// step's Inputs (core §6.2).

// ExtrudeOption configures Extrude.
type ExtrudeOption interface {
	option.Interface
	extrudeOption()
}

type extrudeOption struct{ option.Interface }

func (extrudeOption) extrudeOption() {}

type identTaper struct{}

// WithTaper sets the extrude taper: a SIGNED displacement angle — which way
// the wall leans — recorded exactly in the step's ExtrudeOpts. It records
// intent but always refuses a nonzero value in this evaluator: a nonzero taper
// is [ErrUnsupported] (docs/evaluator-design.md §5), because a tapered extrude
// of a general region is an offset problem, and a wrong-but-confident prism is
// the failure decad exists to prevent.
func WithTaper(a units.Value) ExtrudeOption {
	return extrudeOption{option.New(identTaper{}, a)}
}

// Extrude sweeps a profile of s along the sketch plane's normal per the
// linear extent e, and registers the new body. p MUST be a profile of s
// (ErrForeignProfile) and a current one (ErrStaleProfile); an invalid
// profile is ErrInvalidProfile, and a boundary decad cannot record exactly
// is ErrUnrecordableProfile (core §7). The step records the profile, the
// plane, the extent and the options; evaluation runs from that record, and a
// failed evaluation leaves the recipe and the document untouched.
func (d *Document) Extrude(s *sketch.Sketch, p *sketch.Profile, e Extent, opts ...ExtrudeOption) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	profile, plane, err := RecordProfile(s, p)
	if err != nil {
		return nil, err
	}
	if err := falsifyRecordedArea(profile, p.Area); err != nil {
		return nil, err
	}

	taper := units.Degrees(0)
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identTaper:
			v, ok := option.Get[units.Value](o)
			if !ok {
				return nil, fmt.Errorf(`%w: WithTaper carries no angle`, ErrDegenerate)
			}
			taper = v
		}
	}
	if taper.Kind() != units.Angle {
		return nil, fmt.Errorf(`%w: a taper must be an angle, got %s`, ErrUnitKind, taper.Kind())
	}
	if _, err := taper.In(units.Radian); err != nil {
		return nil, fmt.Errorf(`%w: the taper is not representable: %s`, ErrNotFinite, err)
	}
	if taper.Mag() != 0 {
		// Recorded exactly, then rejected: staging is explicit
		// (docs/evaluator-design.md §2/§5), never a silent untapered prism.
		return nil, fmt.Errorf(`%w: this evaluator extrudes straight (untapered) prisms only; omit WithTaper or pass a zero angle`, ErrUnsupported)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}

	e, err = normalizeExtent(e)
	if err != nil {
		return nil, err
	}
	z0, z1, inputs, err := d.resolveLinearExtent(e, frame)
	if err != nil {
		return nil, err
	}

	step := Step{
		Op:      OpExtrude,
		Inputs:  inputs,
		Profile: profile,
		Plane:   plane,
		Extent:  recordExtent(e),
		Opts:    ExtrudeOpts{Taper: taper},
	}
	ref := d.nextStepRef()
	body, err := evalPrism(d, ref, prismPayload{
		profile: profile,
		frame:   frame,
		z0:      z0,
		z1:      z1,
		xform:   r3.Identity(),
	})
	if err != nil {
		return nil, err
	}
	d.commit(step, body)
	return body, nil
}

// falsifyRecordedArea is evaluator-design §1's one live-profile read: the
// recorded region's closed-form area is compared against sketch's own Area
// answer, and a LARGE mismatch rejects the call — the record and the profile
// disagree, which is a bug somewhere. A small residual proves nothing and
// admits nothing; the check can only reject, the same one-sided shape as the
// seam's range falsifier.
func falsifyRecordedArea(profile ProfileRecord, sketchArea float64) error {
	ig, err := profile.integrals()
	if err != nil {
		return err
	}
	scale := math.Max(1, math.Abs(sketchArea))
	if math.Abs(ig.area-sketchArea) > 1e-9*scale {
		return fmt.Errorf(`%w: the recorded boundary's area %v does not reproduce sketch's %v; report upstream as a bug`,
			ErrUnrecordableProfile, ig.area, sketchArea)
	}
	return nil
}

// resolveLinearExtent turns a linear extent into the signed sweep interval
// [z0, z1] along the plane normal (docs/evaluator-design.md §5), plus the
// StepRefs of the bodies the extent's stops resolved against — named-extent
// refs in extent order first, through-all stop bodies after them in stop
// order along the sweep, deduplicated (core §6.2). Magnitudes are validated
// per core §8.1/§12; a zero-thickness sweep is ErrDegenerate.
func (d *Document) resolveLinearExtent(e Extent, frame r3.Frame) (float64, float64, []StepRef, error) {
	switch e := e.(type) {
	case Distance:
		m, err := magnitudeIn(e.D, units.Length, units.Millimeter, "the extent distance")
		if err != nil {
			return 0, 0, nil, err
		}
		if m == 0 {
			return 0, 0, nil, fmt.Errorf(`%w: a zero-distance extent sweeps no solid`, ErrDegenerate)
		}
		// An unknown Direction is malformed input, never silently Along.
		switch e.Dir {
		case Along:
			return 0, m, nil, nil
		case Against:
			return -m, 0, nil, nil
		default:
			return 0, 0, nil, fmt.Errorf(`%w: unknown direction %d`, ErrDegenerate, int(e.Dir))
		}
	case Symmetric:
		m, err := magnitudeIn(e.D, units.Length, units.Millimeter, "the symmetric distance")
		if err != nil {
			return 0, 0, nil, err
		}
		if m == 0 {
			return 0, 0, nil, fmt.Errorf(`%w: a zero-distance extent sweeps no solid`, ErrDegenerate)
		}
		half := m
		if e.FullLength {
			half = m / 2
		}
		return -half, half, nil, nil
	case TwoSided:
		one, err := d.resolveLinearSide(e.One, frame, 1, "the along side")
		if err != nil {
			return 0, 0, nil, err
		}
		two, err := d.resolveLinearSide(e.Two, frame, -1, "the against side")
		if err != nil {
			return 0, 0, nil, err
		}
		if one.z == 0 && two.z == 0 {
			return 0, 0, nil, fmt.Errorf(`%w: a zero-distance extent sweeps no solid`, ErrDegenerate)
		}
		named := append(append([]StepRef(nil), one.named...), two.named...)
		refs := dedupRefs(append(append(named, one.through...), two.through...))
		return two.z, one.z, refs, nil
	case ThroughAll:
		// An unknown Direction is malformed input, never silently Along.
		var travel float64
		switch e.Dir {
		case Along:
			travel = 1
		case Against:
			travel = -1
		default:
			return 0, 0, nil, fmt.Errorf(`%w: unknown direction %d`, ErrDegenerate, int(e.Dir))
		}
		stop, refs, err := d.resolveThroughAll(frame, travel)
		if err != nil {
			return 0, 0, nil, err
		}
		if travel > 0 {
			return 0, stop, refs, nil
		}
		return stop, 0, refs, nil
	case ToFace:
		stop, ref, err := d.resolveToFace(e, frame, 0, "a to-face extent")
		if err != nil {
			return 0, 0, nil, err
		}
		if stop > 0 {
			return 0, stop, []StepRef{ref}, nil
		}
		return stop, 0, []StepRef{ref}, nil
	case nil:
		return 0, 0, nil, fmt.Errorf(`%w: a nil extent sweeps nothing`, ErrDegenerate)
	default:
		return 0, 0, nil, fmt.Errorf(`%w: extent %T is not supported by this evaluator`, ErrUnsupported, e)
	}
}

// linearSide is one resolved side of a TwoSided: its signed boundary
// coordinate along the plane normal, and the stop refs it resolved —
// named-extent and through-all kept apart so the enclosing extent can order
// them per core §6.2.
type linearSide struct {
	z       float64
	named   []StepRef
	through []StepRef
}

// resolveLinearSide resolves one side of a TwoSided; travel is +1 for the
// along side, −1 for the against side.
func (d *Document) resolveLinearSide(s SideExtent, frame r3.Frame, travel float64, what string) (linearSide, error) {
	s, err := normalizeSideExtent(s)
	if err != nil {
		return linearSide{}, err
	}
	switch s := s.(type) {
	case DistanceSide:
		m, err := magnitudeIn(s.D, units.Length, units.Millimeter, what)
		if err != nil {
			return linearSide{}, err
		}
		return linearSide{z: travel * m}, nil
	case ThroughAllSide:
		stop, refs, err := d.resolveThroughAll(frame, travel)
		if err != nil {
			return linearSide{}, err
		}
		return linearSide{z: stop, through: refs}, nil
	case ToFace:
		stop, ref, err := d.resolveToFace(s, frame, travel, what)
		if err != nil {
			return linearSide{}, err
		}
		return linearSide{z: stop, named: []StepRef{ref}}, nil
	case nil:
		return linearSide{}, fmt.Errorf(`%w: a two-sided extent requires both sides`, ErrDegenerate)
	default:
		return linearSide{}, fmt.Errorf(`%w: side extent %T is not supported by this evaluator`, ErrUnsupported, s)
	}
}

// prismPayload is the evaluator's own record of a prism body: the recorded
// region, the plane frame it lifts through, the signed sweep interval, and
// the accumulated rigid placement. Every measurement and the whole topology
// derive from it, which is what makes Placed exact: it re-evaluates the same
// payload under the composed motion (docs/evaluator-design.md §8).
type prismPayload struct {
	profile ProfileRecord
	frame   r3.Frame
	z0, z1  float64
	xform   r3.Transform
}

// point lifts a plane-local (u, v) at height z into placed world space.
func (pp prismPayload) point(u, v, z float64) r3.Vec {
	local := pp.frame.ToWorldUV(u, v)
	n := pp.frame.N()
	return pp.xform.Apply(local.Add(n.Scale(z)))
}

// dir places a plane-local direction (du, dv, dz in frame coordinates) into
// world space.
func (pp prismPayload) dir(du, dv, dz float64) r3.Vec {
	world := pp.frame.U().Scale(du).Add(pp.frame.V().Scale(dv)).Add(pp.frame.N().Scale(dz))
	return pp.xform.ApplyDir(world)
}

// reflected reports whether the accumulated placement flips handedness — a
// reflected solid's face normals and arc senses invert with it, and every
// orientation decision below corrects for it.
func (pp prismPayload) reflected() bool { return pp.xform.IsReflection() }

// transform is the accumulated rigid placement.
func (pp prismPayload) transform() r3.Transform { return pp.xform }

// placed re-evaluates the same record under the composed motion.
func (pp prismPayload) placed(d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	pp.xform = composed
	return evalPrism(d, ref, pp)
}

// evalPrism builds the analytic prism body from the payload: side faces per
// boundary segment, two caps, shared edges and vertices, and Exact
// measurements (docs/evaluator-design.md §5). The payload's segment kinds
// are the increment-1 set; anything else has already been rejected by the
// mass-property integrals it runs first.
func evalPrism(d *Document, ref StepRef, pp prismPayload) (*Body, error) {
	ig, err := pp.profile.integrals()
	if err != nil {
		return nil, err
	}
	if ig.area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, ErrDegenerate)
	}
	h := pp.z1 - pp.z0
	if h <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: "body"}, solid: true}

	// Topology: one shell over every loop's side faces plus the two caps.
	var faces []*Face
	// The start cap faces −N (its outward normal leaves the material at the
	// bottom), so its frame swaps the in-plane axes; the end cap keeps them.
	startFrame, err := capFrame(pp, pp.z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := capFrame(pp, pp.z1, false)
	if err != nil {
		return nil, err
	}
	capStart := &Face{
		surface: Plane{Frame: startFrame},
		origins: []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:    body,
		area:    ig.area,
	}
	capEnd := &Face{
		surface: Plane{Frame: endFrame},
		origins: []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:    body,
		area:    ig.area,
	}

	perimeter := 0.0
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	for li, loop := range loops {
		sideFaces, bottom, top, loopLen, err := buildLoopSides(body, ref, pp, li, loop)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		perimeter += loopLen
		capStart.loops = append(capStart.loops, &Loop{coedges: bottom, outer: li == 0})
		capEnd.loops = append(capEnd.loops, &Loop{coedges: top, outer: li == 0})
	}
	faces = append(faces, capStart, capEnd)
	for _, f := range []*Face{capStart, capEnd} {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				ce.edge.faces = append(ce.edge.faces, f)
			}
		}
	}

	shell := &Shell{faces: faces}
	body.lumps = []*Lump{{shells: []*Shell{shell}}}

	// Measurements — all Exact (docs/evaluator-design.md §5).
	body.volume = Measurement{Value: units.CubicMillimeters(ig.area * h), Exactness: Exact, Bound: units.CubicMillimeters(0)}
	body.area = Measurement{Value: units.SquareMillimeters(2*ig.area + perimeter*h), Exactness: Exact, Bound: units.SquareMillimeters(0)}
	zc := (pp.z0 + pp.z1) / 2
	body.centroid = VecMeasurement{
		Value:     pp.point(ig.mu/ig.area, ig.mv/ig.area, zc),
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}
	bounds, err := prismBounds(pp)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds
	body.payload = pp
	return body, nil
}

// capFrame is the plane frame of a cap at height z, under the placement;
// flip swaps the in-plane axes so the frame's normal is the cap's OUTWARD
// normal, and a reflected placement swaps once more (a reflection flips the
// cross product's handedness). The payload frame is orthonormal and the
// placement rigid, so the transformed axes cannot be degenerate; NewFrame
// re-validates anyway and the error propagates rather than being discarded.
func capFrame(pp prismPayload, z float64, flip bool) (r3.Frame, error) {
	if pp.reflected() {
		flip = !flip
	}
	u, v := pp.dir(1, 0, 0), pp.dir(0, 1, 0)
	if flip {
		u, v = v, u
	}
	f, err := r3.NewFrame(pp.point(0, 0, z), u, v)
	if err != nil {
		return r3.Frame{}, fmt.Errorf(`%w: the placed cap frame is degenerate: %s`, ErrDegenerate, err)
	}
	return f, nil
}

// segmentWalk is one boundary segment's walk geometry in plane coordinates.
type segmentWalk struct {
	// start/end are the walk's endpoints in (u, v); closed is true for a
	// whole closed curve (no junction vertices at all).
	startU, startV float64
	endU, endV     float64
	closed         bool
	// tanIn/tanOut are the walk tangents at start and end (unit not
	// required), for junction convexity.
	tanInU, tanInV   float64
	tanOutU, tanOutV float64
	length           float64
	// circular geometry when the segment is a circle/arc walk.
	circular bool
	cU, cV   float64
	radius   float64
	th0, th1 float64
}

// walkOf resolves one increment-1 segment into its walk geometry.
func walkOf(seg CurveSegment) (segmentWalk, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return segmentWalk{}, err
	}
	switch seg := seg.(type) {
	case LineSeg:
		u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
		u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
		du, dv := u1-u0, v1-v0
		return segmentWalk{
			startU: u0, startV: v0, endU: u1, endV: v1,
			tanInU: du, tanInV: dv, tanOutU: du, tanOutV: dv,
			length: math.Hypot(du, dv),
		}, nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return segmentWalk{}, fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		if seg.CCW != (seg.TStart < seg.TEnd) {
			return segmentWalk{}, fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		th0, th1 := 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd
		w := circularWalk(seg.Center.U, seg.Center.V, r, th0, th1)
		w.closed = math.Abs(math.Abs(th1-th0)-2*math.Pi) < 1e-12
		return w, nil
	case ArcSeg:
		radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		return circularWalk(seg.Center.U, seg.Center.V, radius, a0+seg.TStart*sweep, a0+seg.TEnd*sweep), nil
	default:
		return segmentWalk{}, fmt.Errorf(`%w: this evaluator sweeps profiles of line, arc and circle segments only; the profile has a %T segment it cannot sweep into a side face yet`, ErrUnsupported, seg)
	}
}

// circularWalk builds the walk geometry of a circular path about (cu, cv).
func circularWalk(cu, cv, r, th0, th1 float64) segmentWalk {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	sign := 1.0
	if th1 < th0 {
		sign = -1
	}
	return segmentWalk{
		startU: cu + r*cos0, startV: cv + r*sin0,
		endU: cu + r*cos1, endV: cv + r*sin1,
		tanInU: -sign * sin0, tanInV: sign * cos0,
		tanOutU: -sign * sin1, tanOutV: sign * cos1,
		length:   r * math.Abs(th1-th0),
		circular: true,
		cU:       cu, cV: cv, radius: r, th0: th0, th1: th1,
	}
}

// sideWalk is one side face's walk after canonicalization: consecutive
// collinear line walks coalesce into one (evaluator §3 — "adjacent coplanar
// side faces merge"), and the merged face carries every constituent
// segment's role.
type sideWalk struct {
	segmentWalk
	segs []int // the recorded segment indices this walk covers
}

// coalesceWalks merges consecutive collinear line walks, wrap-around
// included. Circular walks never merge; a loop that is entirely one straight
// line is degenerate and left to the area gate.
func coalesceWalks(walks []sideWalk) []sideWalk {
	collinear := func(a, b sideWalk) bool {
		if a.circular || b.circular {
			return false
		}
		cross := a.tanOutU*b.tanInV - a.tanOutV*b.tanInU
		dot := a.tanOutU*b.tanInU + a.tanOutV*b.tanInV
		scale := math.Hypot(a.tanOutU, a.tanOutV) * math.Hypot(b.tanInU, b.tanInV)
		return dot > 0 && math.Abs(cross) <= 1e-12*scale
	}
	merge := func(a, b sideWalk) sideWalk {
		a.endU, a.endV = b.endU, b.endV
		a.tanOutU, a.tanOutV = b.tanOutU, b.tanOutV
		a.length += b.length
		a.segs = append(a.segs, b.segs...)
		return a
	}
	out := make([]sideWalk, 0, len(walks))
	for _, w := range walks {
		if len(out) > 0 && collinear(out[len(out)-1], w) {
			out[len(out)-1] = merge(out[len(out)-1], w)
			continue
		}
		out = append(out, w)
	}
	// Wrap-around: the loop's last walk may continue into its first.
	for len(out) > 1 && collinear(out[len(out)-1], out[0]) {
		out[0] = merge(out[len(out)-1], out[0])
		out = out[:len(out)-1]
	}
	return out
}

// buildLoopSides builds one loop's side faces with shared vertices and
// edges, returning the faces, the bottom and top cap coedges in walk order,
// and the loop's perimeter length. A loop's index is both its role index and,
// via li != 0, its orientation: loop 0 is an outer loop (material inside),
// every other a hole (material outside).
func buildLoopSides(body *Body, ref StepRef, pp prismPayload, li int, loop LoopRecord) ([]*Face, []coedge, []coedge, float64, error) {
	return buildLoopSidesAs(body, ref, pp, li, li != 0, loop)
}

// buildLoopSidesAs is buildLoopSides with the role index and the orientation
// decoupled: roleLoop names the walls' side(roleLoop,j) role, and holeLoop
// picks the material side of a straight wall independently. A cup's cavity
// walks each loop of the void the SOLID wraps — the void's outer boundary is a
// hole in the solid (holeLoop true), each of the void's own holes a solid post
// (holeLoop false) — a pairing the natural li != 0 rule cannot express
// (docs/modify-design.md §9).
func buildLoopSidesAs(body *Body, ref StepRef, pp prismPayload, roleLoop int, holeLoop bool, loop LoopRecord) ([]*Face, []coedge, []coedge, float64, error) {
	if len(loop.Segments) == 0 {
		return nil, nil, nil, 0, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	raw := make([]sideWalk, len(loop.Segments))
	total := 0.0
	for i, seg := range loop.Segments {
		w, err := walkOf(seg)
		if err != nil {
			return nil, nil, nil, 0, err
		}
		raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
		total += w.length
	}
	walks := coalesceWalks(raw)
	n := len(walks)

	// Junction vertices, shared between neighbors: junction i sits at walk
	// i's start (== walk i−1's end). A single whole closed curve has none.
	singleClosed := n == 1 && walks[0].closed
	var bottomV, topV []*Vertex
	if !singleClosed {
		bottomV = make([]*Vertex, n)
		topV = make([]*Vertex, n)
		for i, w := range walks {
			bottomV[i] = &Vertex{position: pp.point(w.startU, w.startV, pp.z0), bound: units.Millimeters(0)}
			topV[i] = &Vertex{position: pp.point(w.startU, w.startV, pp.z1), bound: units.Millimeters(0)}
		}
	}

	// Vertical edges at junctions, shared between side face i−1 and i.
	// Convexity from the 2D turn: a positive cross of the incoming and
	// outgoing tangents is a left turn — interior angle < π — which is a
	// convex edge on the outer loop and works out identically for hole
	// loops walked clockwise.
	var vertical []*Edge
	if !singleClosed {
		vertical = make([]*Edge, n)
		for i := range walks {
			prev := walks[(i+n-1)%n]
			cross := prev.tanOutU*walks[i].tanInV - prev.tanOutV*walks[i].tanInU
			vertical[i] = &Edge{
				curve:  Line3{},
				start:  bottomV[i],
				end:    topV[i],
				convex: cross > 0,
				length: pp.z1 - pp.z0,
			}
		}
	}

	// Side faces with bottom/top edges; cap coedges accumulate in walk order.
	h := pp.z1 - pp.z0
	faces := make([]*Face, 0, n)
	bottomCo := make([]coedge, 0, n)
	topCo := make([]coedge, 0, n)
	for i, w := range walks {
		var bStart, bEnd, tStart, tEnd *Vertex
		if !singleClosed {
			bStart, tStart = bottomV[i], topV[i]
			bEnd, tEnd = bottomV[(i+1)%n], topV[(i+1)%n]
		}
		var bottomEdge, topEdge *Edge
		var surf Surface
		faceReversed := false
		if w.circular {
			axis := pp.dir(0, 0, 1)
			// The material's side of a circular wall is decided by the WALK,
			// not by the loop's role: the outward normal is the walk tangent
			// turned a quarter turn against the walk's sense, which is the
			// radial direction away from the centre for a counter-clockwise
			// walk and toward the centre for a clockwise one. A hole is
			// walked clockwise — which is why its wall reverses — but so is a
			// CONCAVE round on an outer loop (a rounded bite out of the
			// boundary), whose material also lies outside the cylinder.
			clockwise := w.th1 < w.th0
			// An Arc3/Circle3 is CCW from start to end about its axis. A
			// clockwise walk, and a reflected placement (which flips
			// handedness), each invert that sense, so the EDGE axis carries
			// the corrected sign; the cylinder surface keeps the plain ruling
			// direction.
			edgeSign := 1.0
			if clockwise {
				edgeSign = -1
			}
			if pp.reflected() {
				edgeSign = -edgeSign
			}
			edgeAxis := axis.Scale(edgeSign)
			center0 := pp.point(w.cU, w.cV, pp.z0)
			center1 := pp.point(w.cU, w.cV, pp.z1)
			radius := units.Millimeters(w.radius)
			var curve0, curve1 Curve
			if singleClosed {
				// A full circle's edge closes on itself: one vertex per cap
				// edge, start == end (topology.go's Circle3 contract).
				seam0 := &Vertex{position: pp.point(w.startU, w.startV, pp.z0), bound: units.Millimeters(0)}
				seam1 := &Vertex{position: pp.point(w.startU, w.startV, pp.z1), bound: units.Millimeters(0)}
				bStart, bEnd = seam0, seam0
				tStart, tEnd = seam1, seam1
				curve0, curve1 = Circle3{Center: center0, Axis: edgeAxis, Radius: radius}, Circle3{Center: center1, Axis: edgeAxis, Radius: radius}
			} else {
				curve0, curve1 = Arc3{Center: center0, Axis: edgeAxis, Radius: radius}, Arc3{Center: center1, Axis: edgeAxis, Radius: radius}
			}
			// The rim edges read the same WALK: a clockwise wall's material
			// lies outside its cylinder, so the boundary turns into the metal
			// there and the rim is concave — a hole's rim (clockwise) and a
			// concave outer bite's (also clockwise) alike, while a
			// counter-clockwise round keeps its convex rim. The loop's role
			// decides nothing here.
			capConvex := !clockwise
			bottomEdge = &Edge{curve: curve0, start: bStart, end: bEnd, convex: capConvex, length: w.length}
			topEdge = &Edge{curve: curve1, start: tStart, end: tEnd, convex: capConvex, length: w.length}
			surf = Cylinder{Origin: center0, Axis: axis, Radius: radius}
			// A clockwise-walked wall has its material OUTSIDE the cylinder,
			// so its outward normal is the radial direction negated.
			faceReversed = clockwise
		} else {
			// A straight wall has no turn of its own to disagree with the
			// loop's: which side its material lies on is decided by the sense
			// the whole loop is walked, and that sense IS the loop's role —
			// the outer loop counter-clockwise, holes clockwise (moments.go).
			bottomEdge = &Edge{curve: Line3{}, start: bStart, end: bEnd, convex: !holeLoop, length: w.length}
			topEdge = &Edge{curve: Line3{}, start: tStart, end: tEnd, convex: !holeLoop, length: w.length}
			mid := pp.point((w.startU+w.endU)/2, (w.startV+w.endV)/2, pp.z0)
			// tangent × N is the outward normal for a CCW outer walk (and a
			// CW hole walk); a reflection flips the cross product, so the
			// tangent is negated to keep the frame's normal outward.
			tu, tv := w.tanInU, w.tanInV
			if pp.reflected() {
				tu, tv = -tu, -tv
			}
			f, err := r3.NewFrame(mid, pp.dir(tu, tv, 0), pp.dir(0, 0, 1))
			if err != nil {
				return nil, nil, nil, 0, fmt.Errorf(`%w: a boundary segment has no direction`, ErrDegenerate)
			}
			surf = Plane{Frame: f}
		}

		origins := make([]FeatureRef, len(w.segs))
		for oi, si := range w.segs {
			origins[oi] = FeatureRef{Step: ref, Role: fmt.Sprintf("side(%d,%d)", roleLoop, si)}
		}
		face := &Face{
			surface:  surf,
			origins:  origins,
			body:     body,
			area:     w.length * h,
			reversed: faceReversed,
		}
		if singleClosed {
			// A closed cylindrical band has TWO boundary loops — one circle
			// per cap — not one loop holding both.
			face.loops = []*Loop{
				{coedges: []coedge{{edge: bottomEdge, forward: true}}, outer: true},
				{coedges: []coedge{{edge: topEdge, forward: false}}, outer: true},
			}
		} else {
			face.loops = []*Loop{{coedges: []coedge{
				{edge: bottomEdge, forward: true},
				{edge: vertical[(i+1)%n], forward: true},
				{edge: topEdge, forward: false},
				{edge: vertical[i], forward: false},
			}, outer: true}}
		}

		bottomEdge.faces = append(bottomEdge.faces, face)
		topEdge.faces = append(topEdge.faces, face)
		if !singleClosed {
			vertical[i].faces = append(vertical[i].faces, face)
			vertical[(i+1)%n].faces = append(vertical[(i+1)%n].faces, face)
		}

		faces = append(faces, face)
		bottomCo = append(bottomCo, coedge{edge: bottomEdge, forward: true})
		topCo = append(topCo, coedge{edge: topEdge, forward: true})
	}
	return faces, bottomCo, topCo, total, nil
}

// extentAlong is the prism's exact extent interval along an arbitrary world
// direction g — the lifted linear functional point·g = origin·g + u·(U'·g)
// + v·(V'·g) + z·(N'·g), primes the placed directions, extremized over the
// region boundary and the sweep. Shared by prismBounds and the through-all
// stop resolution (docs/evaluator-design.md §5).
func (pp prismPayload) extentAlong(g r3.Vec) (float64, float64, error) {
	base := pp.xform.Apply(pp.frame.Origin()).Dot(g)
	gu := pp.dir(1, 0, 0).Dot(g)
	gv := pp.dir(0, 1, 0).Dot(g)
	gz := pp.dir(0, 0, 1).Dot(g)
	lo, hi, err := boundaryExtremes(pp.profile, gu, gv)
	if err != nil {
		return 0, 0, err
	}
	zlo := math.Min(pp.z0*gz, pp.z1*gz)
	zhi := math.Max(pp.z0*gz, pp.z1*gz)
	return base + lo + zlo, base + hi + zhi, nil
}

// prismBounds computes the exact axis-aligned bounds of the placed prism:
// for each world axis, the directional extreme of the region boundary under
// the lifted linear functional, plus the sweep's own extreme
// (docs/evaluator-design.md §5).
func prismBounds(pp prismPayload) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	for i, axis := range axes {
		lo, hi, err := pp.extentAlong(axis)
		if err != nil {
			return Box{}, err
		}
		minC[i] = lo
		maxC[i] = hi
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}, nil
}

// boundaryExtremes returns the min and max of the linear functional
// g(u, v) = gu·u + gv·v over the recorded region's boundary — exact per
// segment kind: line extremes at endpoints, circular extremes at the
// functional's own angle when the walk sweeps it.
func boundaryExtremes(profile ProfileRecord, gu, gv float64) (float64, float64, error) {
	lo, hi := math.Inf(1), math.Inf(-1)
	take := func(u, v float64) {
		g := gu*u + gv*v
		lo = math.Min(lo, g)
		hi = math.Max(hi, g)
	}
	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for _, seg := range loop.Segments {
			w, err := walkOf(seg)
			if err != nil {
				return 0, 0, err
			}
			take(w.startU, w.startV)
			take(w.endU, w.endV)
			if !w.circular {
				continue
			}
			// Interior extremes at θ* where the functional's gradient
			// aligns with the radius: θ* = atan2(gv, gu) (+π).
			gmag := math.Hypot(gu, gv)
			if gmag == 0 {
				continue
			}
			star := math.Atan2(gv, gu)
			tlo, thi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
			for _, cand := range []float64{star, star + math.Pi} {
				for k := math.Floor((tlo-cand)/(2*math.Pi)) * 2 * math.Pi; cand+k <= thi+1e-12; k += 2 * math.Pi {
					th := cand + k
					if th < tlo-1e-12 {
						continue
					}
					take(w.cU+w.radius*math.Cos(th), w.cV+w.radius*math.Sin(th))
				}
			}
		}
	}
	if math.IsInf(lo, 1) {
		return 0, 0, fmt.Errorf(`%w: the recorded region has no boundary`, ErrDegenerate)
	}
	return lo, hi, nil
}
