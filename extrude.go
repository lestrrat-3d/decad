package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the extrude of docs/evaluator-design.md §5: the feature call
// gates its live inputs, records the step, evaluates FROM the record, and
// commits atomically. The prism it builds is fully analytic — Plane and
// Cylinder faces, with bounded mass measurements. Exactly representable
// results retain zero bounds. The body-relative
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
// the wall leans. A nonzero taper is [ErrUnsupported]
// (docs/evaluator-design.md §5), returned before any step is recorded, so the
// recipe is left unchanged — because a tapered extrude of a general region is
// an offset problem, and a wrong-but-confident prism is the failure decad
// exists to prevent. Only a zero taper reaches the step's ExtrudeOpts.
func WithTaper(a units.Value) ExtrudeOption {
	return extrudeOption{option.New(identTaper{}, a)}
}

// Extrude sweeps a profile of s along the sketch plane's normal per the
// linear extent e, and registers the new body. p MUST be a profile of s
// (ErrForeignProfile) and a current, unaltered snapshot (ErrStaleProfile or
// ErrInvalidProfile); an invalid profile is also ErrInvalidProfile, and a
// boundary decad cannot record exactly is ErrUnrecordableProfile (core §7).
// This evaluator builds a straight prism from a profile of line, circle and arc
// segments; a free-form segment (a spline or ellipse, say) is [ErrUnsupported].
// The step records the profile, the plane, the extent and the options;
// evaluation runs from that record, and a failed evaluation leaves the recipe
// and the document untouched.
func (d *Document) Extrude(s *sketch.Sketch, p *sketch.Profile, e Extent, opts ...ExtrudeOption) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	profile, plane, profileArea, err := recordProfile(s, p)
	if err != nil {
		return nil, err
	}
	// ONE free-form work counter for this whole operation over this record: the
	// area falsifier's preflight opens it, the prism build's own preflight
	// continues it, and every walkOf under that build spends what is left
	// (docs/spline-design.md §5.2). A counter per phase would give the same
	// record a fresh full ceiling in each.
	work := newFreeformWork()
	if err := falsifyRecordedArea(profile, profileArea, work); err != nil {
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
		// Refused before the step is built: staging is explicit
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
	sweep, err := d.resolveLinearExtent(e, frame)
	if err != nil {
		return nil, err
	}

	step := Step{
		Op:      OpExtrude,
		Inputs:  sweep.inputs,
		Profile: profile,
		Plane:   plane,
		Extent:  recordExtent(e),
		Opts:    ExtrudeOpts{Taper: taper},
	}
	ref := d.nextStepRef()
	body, err := evalPrism(d, ref, prismPayload{
		profile: profile,
		frame:   frame,
		z0:      sweep.z0,
		z1:      sweep.z1,
		z0Delta: sweep.z0Delta,
		z1Delta: sweep.z1Delta,
		xform:   r3.Identity(),
	}, work)
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
func falsifyRecordedArea(profile ProfileRecord, sketchArea float64, work *freeformWork) error {
	ig, err := profile.evaluatorIntegrals(momentAreaOrder, work)
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

// linearSweep is a resolved linear extent: the signed sweep interval [z0, z1]
// along the plane normal, each end's own proven axial displacement, and the
// StepRefs of the bodies the extent's stops resolved against. A level the
// caller stated denotes itself and reports a zero displacement; a level the
// resolution COMPUTED reports the rounding that computation committed, which is
// what the prism payload carries into every level-derived reading.
type linearSweep struct {
	z0, z1  float64
	z0Delta float64
	z1Delta float64
	inputs  []StepRef
}

// resolveLinearExtent turns a linear extent into that sweep
// (docs/evaluator-design.md §5). The refs are ordered named-extent refs in
// extent order first, through-all stop bodies after them in stop order along
// the sweep, deduplicated (core §6.2). Magnitudes are validated per core
// §8.1/§12; a zero-thickness sweep is ErrDegenerate.
func (d *Document) resolveLinearExtent(e Extent, frame r3.Frame) (linearSweep, error) {
	switch e := e.(type) {
	case Distance:
		m, delta, err := magnitudeInBounded(e.D, units.Length, units.Millimeter, "the extent distance")
		if err != nil {
			return linearSweep{}, err
		}
		if m == 0 {
			return linearSweep{}, fmt.Errorf(`%w: a zero-distance extent sweeps no solid`, ErrDegenerate)
		}
		// An unknown Direction is malformed input, never silently Along. The
		// sketch plane is the end the caller did NOT state, so it stays exact
		// and the swept end takes the distance's own displacement.
		switch e.Dir {
		case Along:
			return linearSweep{z1: m, z1Delta: delta}, nil
		case Against:
			return linearSweep{z0: -m, z0Delta: delta}, nil
		default:
			return linearSweep{}, fmt.Errorf(`%w: unknown direction %d`, ErrDegenerate, int(e.Dir))
		}
	case Symmetric:
		m, delta, err := magnitudeInBounded(e.D, units.Length, units.Millimeter, "the symmetric distance")
		if err != nil {
			return linearSweep{}, err
		}
		if m == 0 {
			return linearSweep{}, fmt.Errorf(`%w: a zero-distance extent sweeps no solid`, ErrDegenerate)
		}
		half := m
		if e.FullLength {
			half = m / 2
		}
		// Halving is exact in binary, so each end sits within the distance's own
		// displacement of the level it denotes; charging the whole displacement
		// to each end rather than half of it is the conservative reading.
		return linearSweep{z0: -half, z1: half, z0Delta: delta, z1Delta: delta}, nil
	case TwoSided:
		one, err := d.resolveLinearSide(e.One, frame, 1, "the along side")
		if err != nil {
			return linearSweep{}, err
		}
		two, err := d.resolveLinearSide(e.Two, frame, -1, "the against side")
		if err != nil {
			return linearSweep{}, err
		}
		if one.z == 0 && two.z == 0 {
			return linearSweep{}, fmt.Errorf(`%w: a zero-distance extent sweeps no solid`, ErrDegenerate)
		}
		named := append(append([]StepRef(nil), one.named...), two.named...)
		refs := dedupRefs(append(append(named, one.through...), two.through...))
		return linearSweep{z0: two.z, z1: one.z, z0Delta: two.delta, z1Delta: one.delta, inputs: refs}, nil
	case ThroughAll:
		// An unknown Direction is malformed input, never silently Along.
		var travel float64
		switch e.Dir {
		case Along:
			travel = 1
		case Against:
			travel = -1
		default:
			return linearSweep{}, fmt.Errorf(`%w: unknown direction %d`, ErrDegenerate, int(e.Dir))
		}
		stop, delta, refs, err := d.resolveThroughAll(frame, travel)
		if err != nil {
			return linearSweep{}, err
		}
		if travel > 0 {
			return linearSweep{z1: stop, z1Delta: delta, inputs: refs}, nil
		}
		return linearSweep{z0: stop, z0Delta: delta, inputs: refs}, nil
	case ToFace:
		stop, delta, ref, err := d.resolveToFace(e, frame, 0, "a to-face extent")
		if err != nil {
			return linearSweep{}, err
		}
		if stop > 0 {
			return linearSweep{z1: stop, z1Delta: delta, inputs: []StepRef{ref}}, nil
		}
		return linearSweep{z0: stop, z0Delta: delta, inputs: []StepRef{ref}}, nil
	case nil:
		return linearSweep{}, fmt.Errorf(`%w: a nil extent sweeps nothing`, ErrDegenerate)
	default:
		return linearSweep{}, fmt.Errorf(`%w: extent %T is not supported by this evaluator`, ErrUnsupported, e)
	}
}

// linearSide is one resolved side of a TwoSided: its signed boundary
// coordinate along the plane normal, that coordinate's own axial displacement,
// and the stop refs it resolved — named-extent and through-all kept apart so
// the enclosing extent can order them per core §6.2.
type linearSide struct {
	z       float64
	delta   float64
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
		m, delta, err := magnitudeInBounded(s.D, units.Length, units.Millimeter, what)
		if err != nil {
			return linearSide{}, err
		}
		// travel is ±1, so the signed level is the magnitude itself and its
		// displacement carries across unchanged.
		return linearSide{z: travel * m, delta: delta}, nil
	case ThroughAllSide:
		stop, delta, refs, err := d.resolveThroughAll(frame, travel)
		if err != nil {
			return linearSide{}, err
		}
		return linearSide{z: stop, delta: delta, through: refs}, nil
	case ToFace:
		stop, delta, ref, err := d.resolveToFace(s, frame, travel, what)
		if err != nil {
			return linearSide{}, err
		}
		return linearSide{z: stop, delta: delta, named: []StepRef{ref}}, nil
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
//
// A prism-derived modify body (a filleted or chamfered prism, modify §2) also
// carries the blend-role descriptors of its own rewritten section: blendSegs
// names, per loop, the (loop, segment) indices whose side face is a blend wall,
// and blendKind is "fillet" or "chamfer". They are part of the re-evaluable
// record so a copy or placement re-mints its own blend roles from its own record
// (the modify §9 role rule) — a plain extrude leaves them empty (no-op).
//
// sectionDelta is the proven upper bound on how far any recorded boundary
// coordinate of the section sits from the section this payload's construction
// DENOTES (docs/prism-boolean-design.md §7). It is zero for every payload a
// caller draws directly — a plain extrude, a placement and every modify rewrite
// record their own coordinates, so the record IS the section they denote — and
// the analytic prism boolean is the one construction that sets it, to the
// displacement its coordinate re-expression commits. Being a payload field it
// re-evaluates with the payload, so a placement or copy keeps it, and every
// measurement evalPrism publishes composes it (never Exact while it is nonzero).
//
// z0Delta and z1Delta are the AXIAL twin of that displacement, one per end: each
// bounds how far the sweep level recorded beside it sits from the level this
// payload's construction denotes. A level the caller stated is its own denotation
// and carries zero — a Distance in millimetres records the number it was given —
// while a COMPUTED level carries the computation's own proven rounding: a ToFace
// or ThroughAll stop resolves its level by float arithmetic over another body's
// face (stops.go), a magnitude in a non-base unit rounds in its rescale to
// millimetres (magnitudeInBounded), and a chamfered end pulls its level in by the
// setback (capblend_moments.go). The two displacements are tracked apart and
// neither ever stands in for the other — sectionDelta moves a boundary coordinate
// IN the plane, these move a level ALONG the normal — while a reading both of them
// displace, a side vertex or the box, sums the two into its own bound. Every
// level-derived reading takes these: the sweep
// height and the volume, wall area and centroid built on it, the box, the side
// vertices and the vertical edge lengths. Being payload fields they re-evaluate
// with the payload, so a placement or copy keeps them.
type prismPayload struct {
	profile      ProfileRecord
	frame        r3.Frame
	z0, z1       float64
	z0Delta      float64
	z1Delta      float64
	xform        r3.Transform
	blendSegs    []map[int]struct{}
	blendKind    string
	sectionDelta float64
}

// z0Scalar and z1Scalar are the sweep levels as bounded readings — the recorded
// float beside its own axial displacement. Every measurement derived from a
// level integrates these rather than the bare float, so a level the evaluator
// computed can never publish itself as the level it denotes.
func (pp prismPayload) z0Scalar() boundedScalar { return measuredScalar(pp.z0, pp.z0Delta) }

func (pp prismPayload) z1Scalar() boundedScalar { return measuredScalar(pp.z1, pp.z1Delta) }

// axialDelta is the larger of the two ends' displacements: the figure a reading
// that cannot attribute its error to one particular end takes.
func (pp prismPayload) axialDelta() float64 { return math.Max(pp.z0Delta, pp.z1Delta) }

// point lifts a plane-local (u, v) at height z into placed world space.
func (pp prismPayload) point(u, v, z float64) r3.Vec {
	local := pp.frame.ToWorldUV(u, v)
	n := pp.frame.N()
	return pp.xform.Apply(local.Add(n.Scale(z)))
}

// prismPointBound carries plane-local coordinate error through the isometric
// frame/placement and charges the float operations that evaluate both maps.
func prismPointBound(pp prismPayload, u, v, z boundedScalar) float64 {
	source := radius3D(max(u.bound, v.bound, z.bound))
	held := pp.point(u.value, v.value, z.value)
	round := exactPrismPointRound(pp, u.value, v.value, z.value, held)
	// Frame and transform constructors enforce near-orthonormal maps. A factor
	// four safely carries the source ball through both held linear maps.
	return absSumUpper(productUpper(4, source), round)
}

func vecMaxAbs(v r3.Vec) float64 {
	return max(math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
}

func exactPrismPointRound(pp prismPayload, u, v, z float64, held r3.Vec) float64 {
	ratVec := func(x, y, z *big.Rat) [3]*big.Rat { return [3]*big.Rat{x, y, z} }
	ratOfVec := func(value r3.Vec) [3]*big.Rat {
		return ratVec(floatRat(value.X), floatRat(value.Y), floatRat(value.Z))
	}
	origin := ratOfVec(pp.frame.Origin())
	fu, fv, fn := ratOfVec(pp.frame.U()), ratOfVec(pp.frame.V()), ratOfVec(pp.frame.N())
	ru, rv, rz := floatRat(u), floatRat(v), floatRat(z)
	if ru == nil || rv == nil || rz == nil {
		return math.Inf(1)
	}
	local := [3]*big.Rat{}
	for i := range local {
		if origin[i] == nil || fu[i] == nil || fv[i] == nil || fn[i] == nil {
			return math.Inf(1)
		}
		local[i] = ratAdd(
			origin[i],
			ratMul(fu[i], ru),
			ratMul(fv[i], rv),
			ratMul(fn[i], rz),
		)
	}
	basis := pp.xform.Basis()
	ex, ey, ez := ratOfVec(basis.EX), ratOfVec(basis.EY), ratOfVec(basis.EZ)
	translation := ratOfVec(pp.xform.Translation())
	exact := [3]*big.Rat{}
	for i := range exact {
		if ex[i] == nil || ey[i] == nil || ez[i] == nil || translation[i] == nil {
			return math.Inf(1)
		}
		exact[i] = ratAdd(
			ratMul(ex[i], local[0]),
			ratMul(ey[i], local[1]),
			ratMul(ez[i], local[2]),
			translation[i],
		)
	}
	perCoord := max(
		rationalFloatError(exact[0], held.X),
		rationalFloatError(exact[1], held.Y),
		rationalFloatError(exact[2], held.Z),
	)
	return radius3D(perCoord)
}

func vecL1(v r3.Vec) float64 {
	return absSumUpper(v.X, v.Y, v.Z)
}

func profileCoordinateUpper(profile ProfileRecord, work *freeformWork) (float64, error) {
	upper := 0.0
	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for _, seg := range loop.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return 0, err
			}
			if err := requireAnalyticWalk(w, "a placed cap frame"); err != nil {
				return 0, err
			}
			upper = math.Max(upper, w.coordUpper)
		}
	}
	return upper, nil
}

// prismCentroidGeometryBound is a second, formula-independent proof. A solid's
// centroid is a convex combination of its material points, so it lies within
// the outer prism. The L1 envelope below bounds every such point through the
// frame and rigid placement, and therefore bounds the distance from held.
func prismCentroidGeometryBound(pp prismPayload, profile ProfileRecord, held r3.Vec, work *freeformWork) (float64, error) {
	coordUpper, err := profileCoordinateUpper(profile, work)
	if err != nil {
		return 0, err
	}
	zUpper := math.Max(math.Abs(pp.z0), math.Abs(pp.z1))
	frameUpper := absSumUpper(
		vecL1(pp.frame.Origin()),
		productUpper(vecL1(pp.frame.U()), coordUpper),
		productUpper(vecL1(pp.frame.V()), coordUpper),
		productUpper(vecL1(pp.frame.N()), zUpper),
	)
	// A rigid map has each output coordinate bounded by the input L1 norm.
	placedUpper := absSumUpper(productUpper(3, frameUpper), vecL1(pp.xform.Translation()))
	return absSumUpper(vecL1(held), placedUpper), nil
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

// placed re-evaluates the same record under the composed motion. It is a
// re-evaluation path: no moments preflight has run on this record within the
// call, so the build opens the record's one counter itself.
func (pp prismPayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	pp.xform = composed
	return evalPrismContext(ctx, d, ref, pp, newFreeformWork())
}

// evalPrism builds the analytic prism body from the payload: side faces per
// boundary segment, two caps, shared edges and vertices, and Exact
// measurements (docs/evaluator-design.md §5). The payload's segment kinds
// are line, circle and arc; anything else has already been rejected by the
// mass-property integrals it runs first.
//
// work is the record's ONE free-form work counter: the preflight this build runs
// and every walkOf under it charge the same ceiling, and a caller that already
// spent part of it on this record passes it in rather than opening a second one.
func evalPrism(d *Document, ref StepRef, pp prismPayload, work *freeformWork) (*Body, error) {
	return evalPrismContext(context.Background(), d, ref, pp, work)
}

func evalPrismContext(ctx context.Context, d *Document, ref StepRef, pp prismPayload, work *freeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ig, err := pp.profile.evaluatorIntegralsUncheckedContext(ctx, momentFirstOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, ErrDegenerate)
	}
	height := boundedSub(pp.z1Scalar(), pp.z0Scalar())
	h := height.value
	if h <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: roleBody}, solid: true}

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
		surface:       Plane{Frame: startFrame},
		origins:       []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:          body,
		area:          ig.area,
		areaBound:     ig.areaBound,
		axialDelta:    pp.z0Delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       Plane{Frame: endFrame},
		origins:       []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:          body,
		area:          ig.area,
		areaBound:     ig.areaBound,
		axialDelta:    pp.z1Delta,
		hasAxialDelta: true,
	}

	perimeter := boundedScalar{}
	walks := 0
	loops := append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...)
	for _, loop := range loops {
		walks += len(loop.Segments)
	}
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sideFaces, bottom, top, loopLen, err := buildLoopSides(ctx, body, ref, pp, li, loop, work)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		perimeter = boundedAdd(perimeter, loopLen)
		capStart.loops = append(capStart.loops, &Loop{coedges: bottom, outer: li == 0})
		capEnd.loops = append(capEnd.loops, &Loop{coedges: top, outer: li == 0})
	}
	faces = append(faces, capStart, capEnd)
	if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
		return nil, err
	}

	shell := &Shell{faces: faces}
	body.lumps = []*Lump{{shells: []*Shell{shell}}}

	// Measurements carry the closed forms' proven float bounds
	// (docs/evaluator-design.md §5), plus — where the payload carries one — the
	// section's own displacement from the section it denotes
	// (docs/prism-boolean-design.md §7). The area a displaced boundary moves is
	// bounds.go's sectionDisplacementArea, charged once into the region area, so
	// the volume takes it through the height and each cap takes it once; the
	// walls take it through the perimeter, which every walk already charged its
	// own length displacement into (buildLoopSidesAs).
	delta := pp.sectionDelta
	regionArea := measuredScalar(ig.area, absSumUpper(
		ig.areaBound,
		sectionDisplacementArea(delta, walks, absSumUpper(perimeter.value, perimeter.bound)),
	))
	capStart.areaBound = regionArea.bound
	capEnd.areaBound = regionArea.bound
	volume := boundedMul(regionArea, height)
	caps := boundedMul(exactScalar(2), regionArea)
	sides := boundedMul(perimeter, height)
	area := boundedAdd(caps, sides)
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volume.value),
		Exactness: exactnessOf(volume.bound),
		Bound:     units.CubicMillimeters(volume.bound),
	}
	body.area = Measurement{
		Value:     units.SquareMillimeters(area.value),
		Exactness: exactnessOf(area.bound),
		Bound:     units.SquareMillimeters(area.bound),
	}
	cu := boundedQuotient(ig.mu, ig.muBound, ig.area, ig.areaBound)
	cv := boundedQuotient(ig.mv, ig.mvBound, ig.area, ig.areaBound)
	zc := boundedDiv(boundedAdd(pp.z0Scalar(), pp.z1Scalar()), exactScalar(2))
	centroidValue := pp.point(cu.value, cv.value, zc.value)
	// A displaced section moves its own centroid, so the displacement enters the
	// plane-local source term prismPointBound already carries through the frame
	// and placement (docs/prism-boolean-design.md §7). The geometry envelope is a
	// separate proof — the centroid lies inside the prism — so it takes the same
	// displacement rather than being allowed to undercut the formula bound with a
	// figure proven only for the recorded section.
	cu.bound = absSumUpper(cu.bound, delta)
	cv.bound = absSumUpper(cv.bound, delta)
	centroidBound := prismPointBound(pp, cu, cv, zc)
	geometryBound, err := prismCentroidGeometryBound(pp, pp.profile, centroidValue, work)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, absSumUpper(geometryBound, delta))
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	bounds, err := prismBoundsContext(ctx, pp, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	// A prism-derived modify body re-mints its blend roles from its own record
	// under this build's ref, so a copy or placement reproduces them (modify §9).
	// A plain extrude carries no descriptors and this is a no-op.
	if err := addBlendRoles(ctx, body, ref, pp.blendSegs, pp.blendKind); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.payload = pp
	return body, nil
}

// attachFaceLoopsContext links each coedge's shared edge to its face. The
// assembly is private until the caller commits the body, but it can be large
// for a recorded region with many loops, so every nesting level polls ctx.
func attachFaceLoopsContext(ctx context.Context, faces []*Face) error {
	for _, f := range faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		for _, l := range f.loops {
			if err := ctx.Err(); err != nil {
				return err
			}
			for _, ce := range l.coedges {
				if err := ctx.Err(); err != nil {
					return err
				}
				ce.edge.faces = append(ce.edge.faces, f)
			}
		}
	}
	return nil
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

// walkKind discriminates what a segmentWalk's geometry IS. It replaced a
// line-versus-circular boolean because a free-form walk is neither: a two-state
// flag left every "not circular" branch silently building a straight line out of
// a spline, which is exactly the confidently-wrong answer decad exists to
// prevent (docs/spline-design.md §6.2).
//
// A switch on walkKind MUST be total. A consumer that cannot yet handle
// walkFreeform refuses before building an analytic face: where it needs a walk
// it uses requireAnalyticWalk, and where no resolution can contribute it gates
// the recorded free-form kind before walkOf.
type walkKind uint8

const (
	// walkLine is a straight walk between its endpoints.
	walkLine walkKind = iota
	// walkCircular is a circular walk about (cU, cV) — a circle or an arc.
	walkCircular
	// walkFreeform is a free-form walk, whose geometry lives in its converted
	// Bézier spans rather than in the circular fields.
	walkFreeform
)

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
	lengthBound      float64
	lengthUpper      float64
	coordUpper       float64
	axisRadiusUpper  float64
	axisMomentUpper  float64
	// kind says which geometry the walk carries; the fields below it are
	// meaningful only for walkCircular.
	kind     walkKind
	cU, cV   float64
	radius   float64
	th0, th1 float64
	// spans is the converted Bézier chain of a walkFreeform walk, in the
	// curve's natural direction; reversed says the walk runs against it. Both
	// are zero for every other kind.
	spans    []bezierSpan
	reversed bool
}

// isCircular reports whether the walk is a circle or arc — the question the
// closed-form circular branches ask.
func (w segmentWalk) isCircular() bool { return w.kind == walkCircular }

// isLine reports whether the walk is straight. It is NOT "not circular": a
// free-form walk answers false to both.
func (w segmentWalk) isLine() bool { return w.kind == walkLine }

// requireAnalyticWalk refuses a free-form walk on behalf of a consumer that has
// no free-form construction yet. Reaching it is a staging limit, never a wrong
// answer — the reason each consumer stages is its own row in
// docs/spline-design.md Table R.
//
// The one call site that reaches walkOf without this gate is
// moments_validate.go's validateMomentWalk: it runs only after every
// free-form segment kind has already been diverted to the exact integrator
// (spline_bezier.go/spline_moments.go), so a free-form segment never reaches
// it. This is deliberate, not a missed gate — adding one here would be dead
// code guarding an unreachable case.
func requireAnalyticWalk(w segmentWalk, what string) error {
	if w.kind != walkFreeform {
		return nil
	}
	return fmt.Errorf(`%w: %s does not support a free-form boundary segment`, ErrUnsupported, what)
}

// walkOf resolves one recorded segment into its walk geometry.
//
// work is the RECORD's free-form work counter (docs/spline-design.md §5.2), and
// walkOf NEVER mints one: the R7 ceiling bounds one record's total free-form
// work, so a counter minted per call would hand every segment — and every later
// phase of the same operation — a fresh full ceiling. Callers that already hold
// the counter a moments preflight opened for this record pass THAT one, so the
// walk's arc-length bracket spends what the preflight left rather than a second
// ceiling; callers with no preflight in hand mint exactly one for the whole
// record walk. An analytic segment charges nothing, so a nil counter is harmless
// there and refused on the free-form arm rather than quietly replaced.
func walkOf(seg CurveSegment, work *freeformWork) (segmentWalk, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return segmentWalk{}, err
	}
	switch seg := seg.(type) {
	case LineSeg:
		u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
		u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
		du, dv := u1-u0, v1-v0
		length := math.Hypot(du, dv)
		lengthBound, lengthUpper, coordUpper := lineWalkBounds(seg, length)
		return segmentWalk{
			startU: u0, startV: v0, endU: u1, endV: v1,
			tanInU: du, tanInV: dv, tanOutU: du, tanOutV: dv,
			length:      length,
			lengthBound: lengthBound,
			lengthUpper: lengthUpper,
			coordUpper:  coordUpper,
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
		w := circularWalk(
			seg.Center.U,
			seg.Center.V,
			r,
			th0,
			th1,
			math.Abs(r),
			circularSweepUpper(seg.TStart, seg.TEnd),
		)
		w.closed = math.Abs(math.Abs(th1-th0)-2*math.Pi) < 1e-12
		if iv, ok := circularLengthInterval(seg); ok {
			w.lengthBound = math.Min(w.lengthBound, intervalFloatError(iv, w.length))
		}
		return w, nil
	case ArcSeg:
		radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		w := circularWalk(
			seg.Center.U,
			seg.Center.V,
			radius,
			a0+seg.TStart*sweep,
			a0+seg.TEnd*sweep,
			arcRadiusUpper(seg),
			circularSweepUpper(seg.TStart, seg.TEnd),
		)
		if iv, ok := circularLengthInterval(seg); ok {
			w.lengthBound = math.Min(w.lengthBound, intervalFloatError(iv, w.length))
		}
		return w, nil
	default:
		if !isFreeformSegment(seg) {
			return segmentWalk{}, fmt.Errorf(`%w: this evaluator sweeps profiles of line, arc, circle and Tier A free-form segments only; the profile has a %T segment it cannot sweep into a side face yet`, ErrUnsupported, seg)
		}
		return freeformWalk(seg, work)
	}
}

// freeformWalk resolves a Tier A free-form segment into its walk geometry
// (docs/spline-design.md Table F). Every field it fills is a proof:
//
//   - the endpoints are the converted chain's own first and last control
//     points, which a Bézier interpolates exactly;
//   - the tangents are the hodograph at those ends, exact directions;
//   - the length is §6.1's proven two-sided bracket, so lengthBound is
//     positive and the walk NEVER claims an exact length — a control net
//     collapsed to a single point has no positive bracket and refuses as
//     ErrDegenerate rather than resolve into a walk (Table R row R14), and a
//     curve whose enclosure runs past MaxFloat64 refuses as ErrUnsupported
//     (R15); freeformArcLength owns both;
//   - coordUpper and lengthUpper are convex-hull envelopes, so they bound the
//     curve and not merely its control net.
//
// axisRadiusUpper and axisMomentUpper stay zero: they are revolve's readings,
// and revolve refuses a free-form walk before reaching them.
//
// The conversion and the length bracket are charged against the caller's counter
// — the record's, never one minted here. A caller that reaches this arm with no
// counter has no ceiling at all, which is the one thing §5.2 forbids, so the
// resolution refuses rather than run unbounded work.
func freeformWalk(seg CurveSegment, work *freeformWork) (segmentWalk, error) {
	if work == nil {
		return segmentWalk{}, errFreeformWalkUncounted
	}
	spans, reversed, err := freeformBezierSpans(seg, work)
	if err != nil {
		return segmentWalk{}, err
	}
	start, end, err := freeformEndpoints(spans, reversed)
	if err != nil {
		return segmentWalk{}, err
	}
	length, bound, err := freeformArcLength(spans, work)
	if err != nil {
		return segmentWalk{}, err
	}
	tanInU, tanInV, tanOutU, tanOutV, err := freeformEndTangents(spans, reversed)
	if err != nil {
		return segmentWalk{}, err
	}
	return segmentWalk{
		startU: start.U, startV: start.V,
		endU: end.U, endV: end.V,
		// A closed free-form curve returns to its start, so it carries no
		// junction vertex — the same fact CircleSeg's closed walk states.
		closed:      start == end,
		tanInU:      tanInU,
		tanInV:      tanInV,
		tanOutU:     tanOutU,
		tanOutV:     tanOutV,
		length:      length,
		lengthBound: bound,
		lengthUpper: upRound(length + bound),
		coordUpper:  freeformControlExtent(spans),
		kind:        walkFreeform,
		spans:       spans,
		reversed:    reversed,
	}, nil
}

// errFreeformWalkUncounted is the refusal of a free-form resolution handed no
// record counter. It is ErrUnsupported because the curve exists and this
// evaluator declines to resolve it without the ceiling §5.2 requires — never a
// silently minted counter, which is the second full ceiling the rule forbids.
var errFreeformWalkUncounted = fmt.Errorf(
	`%w: a free-form segment's walk needs its record's free-form work counter`, ErrUnsupported,
)

// circularWalk builds the walk geometry of a circular path about (cu, cv).
func circularWalk(cu, cv, r, th0, th1, radiusUpper, sweepUpper float64) segmentWalk {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	sign := 1.0
	if th1 < th0 {
		sign = -1
	}
	length := r * math.Abs(th1-th0)
	lengthUpper := productUpper(radiusUpper, sweepUpper)
	coordUpper := absSumUpper(cu, cv, radiusUpper, radiusUpper)
	return segmentWalk{
		startU: cu + r*cos0, startV: cv + r*sin0,
		endU: cu + r*cos1, endV: cv + r*sin1,
		tanInU: -sign * sin0, tanInV: sign * cos0,
		tanOutU: -sign * sin1, tanOutV: sign * cos1,
		length:      length,
		lengthBound: conservativeValueError(length, lengthUpper),
		lengthUpper: lengthUpper,
		coordUpper:  coordUpper,
		kind:        walkCircular,
		cU:          cu, cV: cv, radius: r, th0: th0, th1: th1,
	}
}

// lineWalkBounds compares the held square root with the segment's exact
// rational squared length. A Pythagorean or axis-aligned length that lands
// exactly keeps a zero bound; every other square root uses the exact L1 length
// as a finite magnitude envelope, without assuming a Hypot ulp guarantee. It
// also returns an L1 coordinate envelope for later revolution bounds.
func lineWalkBounds(seg LineSeg, held float64) (float64, float64, float64) {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return math.Inf(1), math.Inf(1), math.Inf(1)
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	)
	heldRat := floatRat(held)
	coordUpper := math.Max(ratL1Upper(u0, v0), ratL1Upper(u1, v1))
	if heldRat != nil && new(big.Rat).Mul(heldRat, heldRat).Cmp(lengthSquared) == 0 {
		return 0, held, coordUpper
	}
	l1 := new(big.Rat).Add(new(big.Rat).Abs(du), new(big.Rat).Abs(dv))
	upper, exact := l1.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	bound := math.Min(conservativeValueError(held, upper), sqrtIntervalError(lengthSquared, held))
	return bound, upper, coordUpper
}

// sqrtIntervalError proves |held-sqrt(lengthSquared)| from the
// directed-rounding square root bracket (ratSqrtDown/ratSqrtUp,
// spline_length.go), assuming no ulp contract from Hypot or Sqrt. It returns
// +Inf when the bracket cannot be built (a non-finite endpoint), so a
// math.Min against it can only ever keep the caller's own bound.
func sqrtIntervalError(lengthSquared *big.Rat, held float64) float64 {
	lo, hi := floatRat(ratSqrtDown(lengthSquared)), floatRat(ratSqrtUp(lengthSquared))
	if lo == nil || hi == nil {
		return math.Inf(1)
	}
	return intervalFloatError(interval(lo, hi), held)
}

func ratL1Upper(values ...*big.Rat) float64 {
	total := new(big.Rat)
	for _, value := range values {
		total.Add(total, new(big.Rat).Abs(value))
	}
	upper, exact := total.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	return upper
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
	out, _ := coalesceWalksBudget(walks, nil)
	return out
}

func coalesceWalksBudget(walks []sideWalk, budget *workBudget) ([]sideWalk, error) {
	return coalesceWalksWithPoll(func() error { return wallBudgetStep(budget) }, walks)
}

func coalesceWalksContext(ctx context.Context, walks []sideWalk) ([]sideWalk, error) {
	return coalesceWalksWithPoll(ctx.Err, walks)
}

func coalesceWalksWithPoll(poll func() error, walks []sideWalk) ([]sideWalk, error) {
	collinear := func(a, b sideWalk) bool {
		if !a.isLine() || !b.isLine() {
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
		length := boundedAdd(measuredScalar(a.length, a.lengthBound), measuredScalar(b.length, b.lengthBound))
		a.length, a.lengthBound = length.value, length.bound
		a.lengthUpper = absSumUpper(a.lengthUpper, b.lengthUpper)
		a.coordUpper = math.Max(a.coordUpper, b.coordUpper)
		a.axisRadiusUpper = math.Max(a.axisRadiusUpper, b.axisRadiusUpper)
		a.axisMomentUpper = absSumUpper(a.axisMomentUpper, b.axisMomentUpper)
		a.segs = append(a.segs, b.segs...)
		return a
	}
	out := make([]sideWalk, 0, len(walks))
	for _, w := range walks {
		if poll != nil {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		if len(out) > 0 && collinear(out[len(out)-1], w) {
			out[len(out)-1] = merge(out[len(out)-1], w)
			continue
		}
		out = append(out, w)
	}
	// Wrap-around: the loop's last walk may continue into its first.
	for len(out) > 1 && collinear(out[len(out)-1], out[0]) {
		if poll != nil {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		out[0] = merge(out[len(out)-1], out[0])
		out = out[:len(out)-1]
	}
	return out, nil
}

// buildLoopSides builds one loop's side faces with shared vertices and
// edges, returning the faces, the bottom and top cap coedges in walk order,
// and the loop's perimeter length. A loop's index is both its role index and,
// via li != 0, its orientation: loop 0 is an outer loop (material inside),
// every other a hole (material outside).
func buildLoopSides(ctx context.Context, body *Body, ref StepRef, pp prismPayload, li int, loop LoopRecord, work *freeformWork) ([]*Face, []coedge, []coedge, boundedScalar, error) {
	return buildLoopSidesAs(ctx, body, ref, pp, li, li != 0, loop, work)
}

// buildLoopSidesAs is buildLoopSides with the role index and the orientation
// decoupled: roleLoop names the walls' side(roleLoop,j) role, and holeLoop
// picks the material side of a straight wall independently. A cup's cavity
// walks each loop of the void the SOLID wraps — the void's outer boundary is a
// hole in the solid (holeLoop true), each of the void's own holes a solid post
// (holeLoop false) — a pairing the natural li != 0 rule cannot express
// (docs/modify-design.md §9).
func buildLoopSidesAs(ctx context.Context, body *Body, ref StepRef, pp prismPayload, roleLoop int, holeLoop bool, loop LoopRecord, work *freeformWork) ([]*Face, []coedge, []coedge, boundedScalar, error) {
	if err := ctx.Err(); err != nil {
		return nil, nil, nil, boundedScalar{}, err
	}
	if len(loop.Segments) == 0 {
		return nil, nil, nil, boundedScalar{}, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	// Every coordinate this loop's walks read sits within the payload's own
	// section displacement of the section it denotes, so each walk's length, each
	// junction vertex and each side face carries that displacement too
	// (docs/prism-boolean-design.md §7). It is zero for every payload a caller
	// draws, and the arithmetic below is then the unchanged one.
	delta := pp.sectionDelta
	walkLenAllow := sectionDisplacementLength(delta, 1)
	raw := make([]sideWalk, len(loop.Segments))
	total := boundedScalar{}
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		seg, err := normalizeSegment(seg)
		if err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		// The prism evaluator stages every free-form side face. Refuse by the
		// recorded kind before resolving its length bracket: that bracket is
		// expensive and cannot contribute to a body this build will reject.
		if isFreeformSegment(seg) {
			return nil, nil, nil, boundedScalar{}, fmt.Errorf(
				`%w: the prism side-face build does not support a free-form boundary segment`,
				ErrUnsupported,
			)
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		if err := requireAnalyticWalk(w, "the prism side-face build"); err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		w.lengthBound = absSumUpper(w.lengthBound, walkLenAllow)
		raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
		total = boundedAdd(total, measuredScalar(w.length, w.lengthBound))
	}
	walks, err := coalesceWalksContext(ctx, raw)
	if err != nil {
		return nil, nil, nil, boundedScalar{}, err
	}
	n := len(walks)

	// Junction vertices, shared between neighbors: junction i sits at walk
	// i's start (== walk i−1's end). A single whole closed curve has none.
	singleClosed := n == 1 && walks[0].closed
	// The sweep height is read from the two BOUNDED levels, so a level the
	// evaluator computed carries its own displacement into the vertical edge
	// lengths and the side face areas built from it below — a ToFace or
	// ThroughAll stop's float arithmetic (stops.go), a non-base unit's rescale,
	// and a chamfered end's setback alike.
	height := boundedSub(pp.z1Scalar(), pp.z0Scalar())
	// A side vertex sits at one recorded boundary coordinate and one sweep level,
	// so it carries both displacements: the section's, which moves it in the
	// plane, and its own end's, which moves it along the normal. Each is zero for
	// a coordinate the payload recorded from what the caller stated, which is what
	// keeps an ordinary extrude's vertices Exact; neither is a claim about the
	// other's axis, so they compose rather than one standing in for the other.
	var bottomV, topV []*Vertex
	bottomBound := units.Millimeters(absSumUpper(delta, pp.z0Delta))
	topBound := units.Millimeters(absSumUpper(delta, pp.z1Delta))
	if !singleClosed {
		bottomV = make([]*Vertex, n)
		topV = make([]*Vertex, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
			bottomV[i] = &Vertex{position: pp.point(w.startU, w.startV, pp.z0), bound: bottomBound}
			topV[i] = &Vertex{position: pp.point(w.startU, w.startV, pp.z1), bound: topBound}
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
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
			prev := walks[(i+n-1)%n]
			cross := prev.tanOutU*walks[i].tanInV - prev.tanOutV*walks[i].tanInU
			vertical[i] = &Edge{
				curve:       Line3{},
				start:       bottomV[i],
				end:         topV[i],
				convex:      cross > 0,
				length:      pp.z1 - pp.z0,
				lengthBound: height.bound,
			}
		}
	}

	// Side faces with bottom/top edges; cap coedges accumulate in walk order.
	faces := make([]*Face, 0, n)
	bottomCo := make([]coedge, 0, n)
	topCo := make([]coedge, 0, n)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		var bStart, bEnd, tStart, tEnd *Vertex
		if !singleClosed {
			bStart, tStart = bottomV[i], topV[i]
			bEnd, tEnd = bottomV[(i+1)%n], topV[(i+1)%n]
		}
		var bottomEdge, topEdge *Edge
		var surf Surface
		faceReversed := false
		if w.isCircular() {
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
				seam0 := &Vertex{position: pp.point(w.startU, w.startV, pp.z0), bound: bottomBound}
				seam1 := &Vertex{position: pp.point(w.startU, w.startV, pp.z1), bound: topBound}
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
			bottomEdge = &Edge{curve: curve0, start: bStart, end: bEnd, convex: capConvex, length: w.length, lengthBound: w.lengthBound}
			topEdge = &Edge{curve: curve1, start: tStart, end: tEnd, convex: capConvex, length: w.length, lengthBound: w.lengthBound}
			surf = Cylinder{Origin: center0, Axis: axis, Radius: radius}
			// A clockwise-walked wall has its material OUTSIDE the cylinder,
			// so its outward normal is the radial direction negated.
			faceReversed = clockwise
		} else {
			// A straight wall has no turn of its own to disagree with the
			// loop's: which side its material lies on is decided by the sense
			// the whole loop is walked, and that sense IS the loop's role —
			// the outer loop counter-clockwise, holes clockwise (moments.go).
			bottomEdge = &Edge{curve: Line3{}, start: bStart, end: bEnd, convex: !holeLoop, length: w.length, lengthBound: w.lengthBound}
			topEdge = &Edge{curve: Line3{}, start: tStart, end: tEnd, convex: !holeLoop, length: w.length, lengthBound: w.lengthBound}
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
				return nil, nil, nil, boundedScalar{}, fmt.Errorf(`%w: a boundary segment has no direction`, ErrDegenerate)
			}
			surf = Plane{Frame: f}
		}

		origins, err := sideOriginsContext(ctx, ref, roleLoop, w.segs)
		if err != nil {
			return nil, nil, nil, boundedScalar{}, err
		}
		faceArea := boundedMul(measuredScalar(w.length, w.lengthBound), height)
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.value,
			areaBound: faceArea.bound,
			reversed:  faceReversed,
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

func sideOriginsContext(ctx context.Context, ref StepRef, roleLoop int, segs []int) ([]FeatureRef, error) {
	origins := make([]FeatureRef, len(segs))
	for oi, si := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		origins[oi] = FeatureRef{Step: ref, Role: fmt.Sprintf("side(%d,%d)", roleLoop, si)}
	}
	return origins, nil
}

// extentAlong is the prism's exact extent interval along an arbitrary world
// direction g — the lifted linear functional point·g = origin·g + u·(U'·g)
// + v·(V'·g) + z·(N'·g), primes the placed directions, extremized over the
// region boundary and the sweep. A through-all stop records its result as an
// exact endpoint, so it refuses a prism with a proven section displacement.
// prismBoundsContext reads extentBoundedAlong directly and carries both that
// displacement and a free-form boundary's own bracket bound as its own
// outward bound instead.
// The stop and clearance callers hold no preflight counter for this record, so
// the interface forms open the record's own — one per extent reading, never one
// per segment.
func (pp prismPayload) extentAlong(g r3.Vec) (float64, float64, error) {
	if pp.sectionDelta != 0 {
		return 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a prism with a proven section displacement`, ErrUnsupported)
	}
	return pp.extentAlongContext(context.Background(), g)
}

func (pp prismPayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return pp.extentAlongWork(ctx, g, newFreeformWork())
}

// extentAlongWork is extentBoundedAlong's refusing wrapper (Table R row R11,
// docs/spline-design.md §6.4), mirroring capBlendPayload.extentAlongWork word
// for word: a through-all stop reads this extent as an exact endpoint and has
// no bound to widen, so a direction whose extreme a free-form bracket cannot
// state exactly refuses rather than fabricate one. This is deliberately WIDER
// than §6.4's own straddle rule — every nonzero bracket bound refuses here,
// not only one that straddles the sketch plane in the travel sense — and
// narrowing it to that test is a later step under the same non-permanent row.
func (pp prismPayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, error) {
	lo, hi, bound, err := pp.extentBoundedAlong(ctx, g, work)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: a free-form prism's extent along this direction is held by its own directional-extreme bracket, known only to a displacement of %v mm; a stop reads this coordinate as exact and has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// extentBoundedAlong is the bounded reading itself: the interval AND its
// proven half-width, folded from boundaryExtremesBoundedContext's own
// per-candidate enclosures (docs/spline-design.md §6.2). An all-analytic
// section carries only zero-width candidates, so the bound is zero and the
// interval exact — unchanged from before this bracket existed.
func (pp prismPayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, float64, error) {
	base := pp.xform.Apply(pp.frame.Origin()).Dot(g)
	gu := pp.dir(1, 0, 0).Dot(g)
	gv := pp.dir(0, 1, 0).Dot(g)
	gz := pp.dir(0, 0, 1).Dot(g)
	lo, hi, bound, err := boundaryExtremesBoundedContext(ctx, pp.profile, gu, gv, work)
	if err != nil {
		return 0, 0, 0, err
	}
	zlo := math.Min(pp.z0*gz, pp.z1*gz)
	zhi := math.Max(pp.z0*gz, pp.z1*gz)
	return base + lo + zlo, base + hi + zhi, bound, nil
}

// prismBoundsContext computes the exact axis-aligned bounds of the placed prism:
// for each world axis, the directional extreme of the region boundary under
// the lifted linear functional, plus the sweep's own extreme
// (docs/evaluator-design.md §5).
func prismBoundsContext(ctx context.Context, pp prismPayload, work *freeformWork) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	extremeBound := 0.0
	for i, axis := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, bound, err := pp.extentBoundedAlong(ctx, axis, work)
		if err != nil {
			return Box{}, err
		}
		minC[i] = lo
		maxC[i] = hi
		extremeBound = math.Max(extremeBound, bound)
	}
	// A displaced section displaces every extreme it holds, and the frame and
	// placement are isometries, so the box's own error carries the section
	// displacement itself — δ outward on every face
	// (docs/prism-boolean-design.md §7) — summed with the boundary's own
	// directional-extreme bracket bound (docs/spline-design.md §6.2). Both are
	// zero for every analytic, caller-drawn payload, which keeps the ordinary
	// prism's box Exact as before. The bracket's own term is what decides the
	// rest, never the section's kind: a free-form section whose extremes along
	// these three axes are all held by exactly representable candidate values
	// reports a zero width and stays Exact too (a span monotone along an axis
	// contributes its two exactly interpolated endpoints and nothing else),
	// while an extreme held by an irrational interior root publishes that
	// bracket's width and is Approximate — §6.2's own stated contract
	// consequence. The sum only
	// goes through absSumUpper's own per-term rounding where there are two
	// genuine terms to compose: bumping a lone sectionDelta a second time for
	// an always-zero extremeBound term would grow the box's bound past the
	// single upRound tessellate.go's own mesh bound composes it against.
	//
	// The sweep's own ends enter the same way. Each axis reading takes the
	// levels through zlo/zhi scaled by |gz| ≤ 1 (a unit axis against a placed
	// unit normal), so the larger end displacement bounds the box face either
	// level can move, and it composes with the section's term because the two
	// displace along different axes.
	axial := pp.axialDelta()
	terms := make([]float64, 0, 3)
	if pp.sectionDelta != 0 {
		terms = append(terms, pp.sectionDelta)
	}
	if extremeBound != 0 {
		terms = append(terms, extremeBound)
	}
	if axial != 0 {
		terms = append(terms, axial)
	}
	bound := 0.0
	switch len(terms) {
	case 0:
	case 1:
		bound = terms[0]
	default:
		bound = absSumUpper(terms...)
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// boundaryExtremes returns the min and max of the linear functional
// g(u, v) = gu·u + gv·v over the recorded region's boundary — exact per
// analytic segment kind: line extremes at endpoints, circular extremes at the
// functional's own angle when the walk sweeps it. It refuses a free-form
// section rather than report a bounded interval; see boundaryExtremesContext.
func boundaryExtremes(profile ProfileRecord, gu, gv float64, work *freeformWork) (float64, float64, error) {
	return boundaryExtremesContext(context.Background(), profile, gu, gv, work)
}

// boundaryExtremesContext is boundaryExtremesBoundedContext's refusing
// wrapper: it keeps the exact signature every existing caller holds
// (revolve.go's resolveAxisSide runs it BEFORE its own requireAnalyticWalk
// gate, and capblend.go's extentBoundedAlong relies on it the same way, so
// this refusal is what protects both from a free-form section today), and it
// refuses ErrUnsupported wherever the bounded scan's bracket carries a
// nonzero bound. Every analytic candidate reports a zero bound, so this
// preserves current behaviour exactly for every section this evaluator could
// already build — only the refusal's message changes.
func boundaryExtremesContext(ctx context.Context, profile ProfileRecord, gu, gv float64, work *freeformWork) (float64, float64, error) {
	lo, hi, bound, err := boundaryExtremesBoundedContext(ctx, profile, gu, gv, work)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: the boundary extreme scan's directional bracket has a nonzero bound of %v mm along this direction, and this caller has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// boundaryExtremesBoundedContext is the one scan, total over walkKind
// (docs/spline-design.md §6.2): the min and max of g(u, v) = gu·u + gv·v over
// the recorded region's boundary, AND the proven half-width of that interval.
// A line or circular candidate is exact — the same closed forms
// boundaryExtremesContext always ran, contributing a zero-width candidate — and
// a free-form walk folds each of its converted spans' own proven enclosure
// (spanExtremeEnclosureContext). The fold is the shipped
// capBlendPayload.extentBoundedAlong idiom: track the lower and upper ends of
// every candidate contributing to the region minimum (loLower/loUpper) and to
// the region maximum (hiLower/hiUpper) separately, so a candidate that loses
// the extremization contributes nothing to the reported bound, and report the
// midpoint of each composed interval with the larger of the two half widths,
// rounded up — the same convention freeformArcLength already uses.
//
// A span enclosure that convention cannot state in float64 refuses at the
// conversion rather than entering the fold (spline_extreme.go's
// freeformExtremeFloats, Table R row R18), so every number these accumulators
// hold is finite and the only reading left to the empty-region check below is
// a region that genuinely contributed no candidate.
//
// The direction is carried through this scan as the two FLOATS the caller
// holds, and it is gated by requireFiniteDirection, which reads them and
// allocates nothing. The rational lift each span's Bernstein coefficients need
// happens inside spanExtremeEnclosureContext, behind that span's own R7 charge
// — §5.2's rule is that every charge is levied before the work allocates, and
// a rational built here would allocate ahead of every charge this scan makes.
func boundaryExtremesBoundedContext(ctx context.Context, profile ProfileRecord, gu, gv float64, work *freeformWork) (float64, float64, float64, error) {
	if err := requireFiniteDirection(gu, gv); err != nil {
		return 0, 0, 0, err
	}

	loLower, loUpper := math.Inf(1), math.Inf(1)
	hiLower, hiUpper := math.Inf(-1), math.Inf(-1)
	takeLo := func(l, h float64) {
		loLower = math.Min(loLower, l)
		loUpper = math.Min(loUpper, h)
	}
	takeHi := func(l, h float64) {
		hiLower = math.Max(hiLower, l)
		hiUpper = math.Max(hiUpper, h)
	}
	take := func(u, v float64) {
		g := gu*u + gv*v
		takeLo(g, g)
		takeHi(g, g)
	}
	// Every span enclosure enters the fold through freeformExtremeFloats
	// (spline_extreme.go), which rounds outward through ratFloatDown/ratFloatUp
	// — never downRound/upRound: a directional value can be negative, and those
	// only ever move a POSITIVE bound toward zero (spline_length.go's
	// arc-length-only convention), the wrong direction for a negative
	// candidate and a spurious one-ulp widening of an exactly representable
	// value either way — and refuses ErrUnsupported for an enclosure the
	// float64 range cannot state, so no infinity ever reaches these
	// accumulators.

	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for _, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return 0, 0, 0, err
			}
			w, err := walkOf(seg, work)
			if err != nil {
				return 0, 0, 0, err
			}
			if w.kind == walkFreeform {
				for _, span := range w.spans {
					minIv, maxIv, err := spanExtremeEnclosureContext(ctx, span, gu, gv, work)
					if err != nil {
						return 0, 0, 0, err
					}
					minLo, minHi, err := freeformExtremeFloats(minIv)
					if err != nil {
						return 0, 0, 0, err
					}
					maxLo, maxHi, err := freeformExtremeFloats(maxIv)
					if err != nil {
						return 0, 0, 0, err
					}
					takeLo(minLo, minHi)
					takeHi(maxLo, maxHi)
				}
				continue
			}
			take(w.startU, w.startV)
			take(w.endU, w.endV)
			if !w.isCircular() {
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
	if math.IsInf(loLower, 1) {
		return 0, 0, 0, fmt.Errorf(`%w: the recorded region has no boundary`, ErrDegenerate)
	}
	loMid := loLower + (loUpper-loLower)/2
	hiMid := hiLower + (hiUpper-hiLower)/2
	bound := upRound(math.Max(
		math.Max(loMid-loLower, loUpper-loMid),
		math.Max(hiMid-hiLower, hiUpper-hiMid),
	))
	return loMid, hiMid, bound, nil
}
