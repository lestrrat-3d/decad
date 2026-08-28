package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"reflect"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the extrude of docs/evaluator-design.md §5: the feature call
// gates its live inputs, records the step, evaluates FROM the record, and
// commits atomically. The prism it builds is analytic — Plane and Cylinder
// faces — for every line/circle/arc boundary segment, and a NURBSSurface
// (docs/spline-design.md §7) for a Tier A free-form one, with bounded mass
// measurements throughout. Exactly representable results retain zero bounds.
// The body-relative
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
// This evaluator builds a straight prism from a profile of line, circle, arc
// and Tier A free-form segments (a spline, a closed spline, a fit spline, or a
// unit-weight NURBS curve — docs/spline-design.md Table F); a Tier B or Tier C
// free-form segment (a conic, a whole ellipse, or a NURBS curve with unequal
// weights) is [ErrUnsupported], as is a nonzero WithTaper. A Tier A kind is
// admitted but not thereby built: each free-form wall edge must also prove ONE
// curvature sign across every span and joint of its chain
// (docs/spline-design.md §6.5), and the whole profile's free-form work must fit
// the fixed budget. A chain whose curvature genuinely changes sign, or whose
// certificate the fixed subdivision depth does not close, is [ErrUnsupported]
// (Table R row R19), as is a profile past the budget (row R7). A free-form curve
// must meet its neighbours at shared endpoints, never by crossing
// (docs/spline-design.md §2.1) — join the endpoints in the sketch, or the
// profile is rejected as ErrUnrecordableProfile before this ever runs. The
// step records the profile, the plane, the extent and the options; evaluation
// runs from that record, and a failed evaluation leaves the recipe and the
// document untouched.
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

// walks is the profile's pre-resolved segment walks (docs/spline-design.md
// §5.2, this file's profileWalks doc comment), or nil to resolve each segment
// through walkOf as before. A non-nil walks that was not resolved from THIS
// profile — the recorded segments compared, not their count — is a plumbing
// bug and refuses rather than silently resolving anyway.
func profileCoordinateUpper(profile ProfileRecord, work *freeformWork, walks *profileWalks) (float64, error) {
	if walks != nil && !walks.matches(profile) {
		return 0, errResolvedWalksMismatch
	}
	upper := 0.0
	for li, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for si, seg := range loop.Segments {
			w, err := resolveOrRead(seg, work, walks, li, si)
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

// profileCoordinateEnvelope is profileCoordinateUpper without the analytic
// requirement: every walk kind, free-form included, states its own coordUpper
// (walkOf's per-kind construction — freeformControlExtent for a free-form
// span), so a caller that only needs a coordinate MAGNITUDE envelope — never a
// placed cap frame, which genuinely cannot represent a free-form wall — reads
// it directly rather than refusing on a section the caller's own reading
// already handles: extentBoundedAlong's boundary-extreme scan, and
// prismCentroidGeometryBound's convex-combination proof, each state their own
// account of a free-form span and need only the envelope beside it.
//
// walks is profileCoordinateUpper's own optional pre-resolved set, same
// contract: nil resolves as before, a non-matching non-nil set refuses.
func profileCoordinateEnvelope(profile ProfileRecord, work *freeformWork, walks *profileWalks) (float64, error) {
	if walks != nil && !walks.matches(profile) {
		return 0, errResolvedWalksMismatch
	}
	upper := 0.0
	for li, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for si, seg := range loop.Segments {
			w, err := resolveOrRead(seg, work, walks, li, si)
			if err != nil {
				return 0, err
			}
			upper = math.Max(upper, w.coordUpper)
		}
	}
	return upper, nil
}

// resolveOrRead is the shared "read the pre-resolved walk, or resolve one"
// step every profileWalks-aware consumer in this file uses: walks non-nil
// (and already checked against the profile by the caller) reads
// walks.at(loopIndex, segIndex); walks nil calls walkOf, exactly as every
// consumer did before profileWalks existed.
func resolveOrRead(seg CurveSegment, work *freeformWork, walks *profileWalks, loopIndex, segIndex int) (segmentWalk, error) {
	if walks != nil {
		return walks.at(loopIndex, segIndex), nil
	}
	return walkOf(seg, work)
}

// prismCentroidGeometryBound is a second, formula-independent proof. A solid's
// centroid is a convex combination of its material points, so it lies within
// the outer prism. The L1 envelope below bounds every such point through the
// frame and rigid placement, and therefore bounds the distance from held.
//
// It reads coordUpper through profileCoordinateEnvelope, never
// profileCoordinateUpper: this proof needs a coordinate MAGNITUDE envelope,
// never a placed cap frame, and every walk kind states one — a free-form
// span's own convex-hull envelope (freeformControlExtent) included — so the
// analytic-only refusal profileCoordinateUpper carries for its OTHER callers
// (capblend_centroid.go, revolve.go) would refuse a centroid this build must
// publish for a section this same build just proved buildable.
//
// walks is the profile's pre-resolved segment walks, or nil; same contract as
// profileCoordinateEnvelope's own.
func prismCentroidGeometryBound(pp prismPayload, profile ProfileRecord, held r3.Vec, work *freeformWork, walks *profileWalks) (float64, error) {
	coordUpper, err := profileCoordinateEnvelope(profile, work, walks)
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
	// pw resolves every boundary segment's walk exactly ONCE for this whole
	// build (this file's profileWalks doc comment): buildLoopSides below,
	// prismCentroidGeometryBound and prismBoundsContext's three per-axis
	// extentBoundedAlong calls all read it back instead of each calling
	// walkOf itself, which is what let one free-form segment's §5.2 charge be
	// spent eight times over in a single evalPrismContext call.
	pw, err := resolveProfileWalks(pp.profile, work)
	if err != nil {
		return nil, err
	}
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sideFaces, bottom, top, loopLen, err := buildLoopSides(ctx, body, ref, pp, li, loop, work, pw)
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
	geometryBound, err := prismCentroidGeometryBound(pp, pp.profile, centroidValue, work, pw)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, absSumUpper(geometryBound, delta))
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	bounds, err := prismBoundsContext(ctx, pp, work, pw)
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
	// startBound/endBound are the PROVEN error bounds on the endpoint beside
	// them, in the coordinates' own millimetres — radiusBound's twin two fields
	// up, and stated for the same reason. An endpoint is an exact leaf only
	// where the record STATES it: a line's own bounds and an arc's own bounds
	// are recorded coordinates the walk reads verbatim (lerp2, pinArcWalkEnds),
	// and those read zero. That zero is about THIS walk's own rounding, not
	// about the recorded coordinate agreeing with the curve the record denotes
	// at that parameter — arcWalkEnd's own doc comment states where an arc's
	// two readings part company, and names who owes the difference. Every other endpoint is computed — a trimmed line's
	// is a float lerp, a trimmed arc's and EVERY circle's is a
	// math.Cos/math.Sin at an angle this package itself computed — so each kind
	// STATES what its own endpoint is worth (lineWalkEndBound,
	// circularWalkEndBound, freeformEndpointBounds) or REFUSES with +Inf, never
	// leaves it silently zero. A reading that folds an endpoint into an answer
	// charges it through pointPerturbationAllow; one that cannot state the
	// charge refuses on the +Inf rather than publishing an exactness the
	// evaluator never proved.
	startBound, endBound walkEndBound
	closed               bool
	// tanIn/tanOut are the walk tangents at start and end (unit not
	// required), for junction convexity.
	tanInU, tanInV   float64
	tanOutU, tanOutV float64
	// tanInBound/tanOutBound are the PROVEN error bound on EITHER component
	// of the tangent beside them, in the coordinates' own millimetres. A
	// tangent is NOT an exact leaf the way a recorded coordinate is: a line
	// walk's is the float difference of two endpoints, a circular walk's runs
	// through math.Sincos at a computed angle, and a free-form walk's is an
	// exact rational leg rounded once into float64. Each kind therefore
	// STATES its bound or REFUSES with +Inf — never leaves it silently zero —
	// so a reading composed from a tangent can charge the error the evaluator
	// actually committed. The refusal is the circular kind's: its held
	// components come from a trig evaluation at an angle that is itself
	// computed, and this walk states no enclosure of either, so +Inf is the
	// underivable bound every consumer refuses on rather than publishes
	// (arcWalkRadiusBound's own convention).
	tanInBound, tanOutBound float64
	length                  float64
	lengthBound             float64
	lengthUpper             float64
	coordUpper              float64
	axisRadiusUpper         float64
	axisMomentUpper         float64
	// startVBound/endVBound/cVBound are the PROVEN error bounds on the radial
	// (V) axis-coordinate beside them — startV/endV/cV's own displacement from
	// the value the axis's TRUE (unrounded) direction and anchor would give,
	// through axisFrame.toAxisRhoBound, composed for startV/endV with whatever
	// magnitude that walk's own axis snap discarded to assign an endpoint
	// exactly zero. They are set ONLY by axisFrame.walk,
	// which re-expresses a plane-local walk into axis coordinates: a walk that
	// has not been through it (every use before revolve resolves an axis)
	// leaves them at their zero value, meaningless there. axisFrame.toAxis
	// itself states no such bound (its own doc comment), so a caller
	// composing a reading from startV/endV/cV — the revolve minimum-radius
	// meridian survey (survey.go's revolveMinRadius) — reads these instead of
	// the coordinate as an exact leaf; axisMoments (revolve.go) folds the
	// SAME axis-direction/anchor uncertainty into the region's moments through
	// bounded arithmetic instead, and does not read these fields.
	startVBound, endVBound, cVBound float64
	// kind says which geometry the walk carries; the fields below it are
	// meaningful only for walkCircular.
	kind   walkKind
	cU, cV float64
	radius float64
	// radiusBound is the PROVEN error bound on radius (millimetres). A
	// CircleSeg states its radius, so its walk holds that number and the bound
	// is zero; an ArcSeg states Start and Center only, so its walk's radius is
	// a math.Hypot evaluation and the bound is arcWalkRadiusBound's rational
	// bracket. It exists because radius is NOT an exact leaf the way a
	// recorded coordinate is, and a reading that treats it as one can publish
	// an interval its own truth sits outside of. The analytic surveys
	// (survey.go's minimum-radius arms, and survey2d.go through
	// surveyElem.rrBound) take it; a consumer that reads radius as a leaf
	// still owes its own account of the error, from its own envelope.
	radiusBound float64
	th0, th1    float64
	// spans is the converted Bézier chain of a walkFreeform walk, in the
	// curve's natural direction; reversed says the walk runs against it. Both
	// are zero for every other kind.
	spans    []bezierSpan
	reversed bool
	// fitInterpolated is set only for a walkFreeform walk whose chain came
	// from FitSplineSeg's §5.1.2 conversion (spline_fit.go's isFitSplineSeg,
	// read on the segment walkOf resolved — walkOf normalizes as its first
	// statement, so freeformWalk's own seg, and every isFitSplineSeg check
	// downstream of it, always sees the normalized value form). §6.5's
	// convexity certificate needs it to apply the FitSplineSeg carve-out: a
	// joint interior to that conversion's chain is verdict 0 BY CONSTRUCTION,
	// never by jointConvexitySign's cross product, because that cross carries
	// sketch's own rounded SecondDerivs solve rather than a turn of the
	// recorded curve (docs/spline-design.md §6.5, §5.1.2). It is false for
	// every other Tier A kind, whose joints are genuine C⁰ corners the cross
	// product must still fold.
	fitInterpolated bool
}

// walkEndBound is the proven error bound on a walk endpoint's two components,
// stated PER COMPONENT and never merged into one number. The two are
// independent readings and an endpoint routinely proves one exactly while the
// other carries error: a whole circle's own end is exactly that shape, since
// math.Cos returns 1 at the end angle while math.Sin does not return 0 there.
// Merging them would spend the exact axis's zero on the other axis's error, and
// a reading along the exact axis alone would then publish a width its own
// arithmetic never committed.
//
// A component the recorded data cannot enclose reads +Inf, the underivable
// bound every consumer refuses on, and never zero.
type walkEndBound struct {
	u, v float64
}

// derivable reports whether both components state a bound at all.
func (b walkEndBound) derivable() bool { return !isNonFinite(b.u) && !isNonFinite(b.v) }

// isCircular reports whether the walk is a circle or arc — the question the
// closed-form circular branches ask.
func (w segmentWalk) isCircular() bool { return w.kind == walkCircular }

// isLine reports whether the walk is straight. It is NOT "not circular": a
// free-form walk answers false to both.
func (w segmentWalk) isLine() bool { return w.kind == walkLine }

// requireAnalyticWalk refuses a free-form walk on behalf of a consumer that has
// no free-form construction yet. Reaching it is a staging limit, never a wrong
// answer — the reason each consumer stages is its own row in
// docs/spline-design.md Table R. The prism side-face build itself no longer
// calls this: buildLoopSidesAs switches on walkKind instead (§10 P4b), with its
// own free-form arm. Every remaining call site is a capability P4b does not
// reach — chording (tessellate.go's chordLoop), the modify ops (fillet.go,
// shell_offset.go, capblend_geom.go), revolve (revolve.go), and
// profileCoordinateUpper's own callers (capblend_centroid.go, revolve.go),
// which need a placed cap frame a free-form wall genuinely cannot represent.
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

// profileWalks is one profile's segment walks resolved ONCE, so that every
// consumer within a single prism evaluation reads the same resolution back
// instead of paying walkOf's own §5.2 charge again for it. Within one
// evalPrismContext call, buildLoopSidesAs, profileCoordinateEnvelope (called
// from prismCentroidGeometryBound and, four times over, from
// prismBoundsContext's per-axis extentBoundedAlong) and
// boundaryExtremesBoundedContext (three times, also from extentBoundedAlong)
// each used to call walkOf on the SAME recorded segment — eight resolutions of
// one segment, each recharging §5.2's exact-rational counter in full. On a
// 15-point involute fit spline that alone charged 230,168 units eight times
// over, tripping the R7 ceiling on a record whose deduplicated charge fits
// comfortably inside it. profileWalks is the fix: resolve every segment's walk
// once and let each consumer read it back.
//
// A nil *profileWalks means "resolve as before" everywhere below: revolve, the
// cap-loop chamfer, the shell cup, Verify, and every re-evaluation path
// (Placed/Duplicate/PlacedCopy) that has no preflight in hand pass nil and are
// unaffected, since they run over a DIFFERENT profile (a cap contour, an
// offset loop) or hold no single-evaluation resolution worth sharing.
type profileWalks struct {
	// profile is the record every walk below was resolved FROM, kept whole so
	// that a read against another profile is caught by comparing the recorded
	// segments themselves rather than their shape (matches).
	profile ProfileRecord
	// outer holds loop index 0's resolved walks, one per pp.profile.Outer
	// segment, in recorded order.
	outer []segmentWalk
	// holes holds loop index i>0's resolved walks as holes[i-1], one slice
	// per pp.profile.Holes entry, each in recorded order — the same
	// append([]LoopRecord{profile.Outer}, profile.Holes...) indexing every
	// consumer below already walks.
	holes [][]segmentWalk
}

// resolveProfileWalks resolves every segment of profile's outer loop and each
// hole loop through walkOf exactly once, charging work the same single time
// each segment's own conversion and length bracket cost (docs/spline-design.md
// §5.2), rather than once per consumer.
func resolveProfileWalks(profile ProfileRecord, work *freeformWork) (*profileWalks, error) {
	outer := make([]segmentWalk, len(profile.Outer.Segments))
	for i, seg := range profile.Outer.Segments {
		w, err := walkOf(seg, work)
		if err != nil {
			return nil, err
		}
		outer[i] = w
	}
	holes := make([][]segmentWalk, len(profile.Holes))
	for hi, hole := range profile.Holes {
		hw := make([]segmentWalk, len(hole.Segments))
		for i, seg := range hole.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return nil, err
			}
			hw[i] = w
		}
		holes[hi] = hw
	}
	return &profileWalks{profile: profile, outer: outer, holes: holes}, nil
}

// at returns the resolved walk for loop index loopIndex (0 the outer loop,
// i>0 profile.Holes[i-1]) and segment index segIndex within that loop, in
// the same indexing every consumer's
// append([]LoopRecord{profile.Outer}, profile.Holes...) walk already uses.
// Callers check matches first; at itself trusts the index it is given.
func (pw *profileWalks) at(loopIndex, segIndex int) segmentWalk {
	if loopIndex == 0 {
		return pw.outer[segIndex]
	}
	return pw.holes[loopIndex-1][segIndex]
}

// loopWalks returns the resolved walk slice for loop index loopIndex (the
// same convention as at), or nil if loopIndex is out of range for pw. A
// single-loop consumer (buildLoopSidesAs) uses this instead of at plus its
// own per-segment loop, since it already owns the per-segment index into the
// slice it gets back.
func (pw *profileWalks) loopWalks(loopIndex int) []segmentWalk {
	if loopIndex == 0 {
		return pw.outer
	}
	hi := loopIndex - 1
	if hi < 0 || hi >= len(pw.holes) {
		return nil
	}
	return pw.holes[hi]
}

// matches reports whether pw was resolved from THIS profile: the same loops
// in the same order, each holding the same recorded segments — the same
// variant with the same field values, compared exactly (identicalRecord).
// Shape alone is not enough, and never was: two profiles can carry the same
// outer, hole and per-hole segment counts while every coordinate differs, and
// a set resolved from one read against the other would report the first
// section's geometry as the second's, silently.
//
// Every consumer that reads a non-nil *profileWalks checks this FIRST and
// refuses rather than reading it — docs/spline-design.md §5.2's own discipline
// extended to this cache. The refusal is one-directional, like every other
// decad-side check: only an exact agreement between the two records reads the
// cache, and anything else — a differing value, a differing variant, a shape
// the comparison does not know how to traverse — refuses. There is no
// tolerance and no "close enough" arm, so a near-miss profile is rejected on
// the same terms as an unrelated one, and a plumbing bug never hides behind a
// correct-looking answer.
func (pw *profileWalks) matches(profile ProfileRecord) bool {
	if pw == nil {
		return false
	}
	return identicalRecord(pw.profile, profile)
}

// loopMatches reports whether pw holds, at loop index loopIndex (the same
// convention as at), the walks resolved from exactly this loop's recorded
// segments. It is matches for the single-loop consumer buildLoopSidesAs,
// which is handed one LoopRecord and a role index rather than the whole
// profile, and it refuses on the same exact-comparison terms.
func (pw *profileWalks) loopMatches(loopIndex int, loop LoopRecord) bool {
	if pw == nil {
		return false
	}
	loops := append([]LoopRecord{pw.profile.Outer}, pw.profile.Holes...)
	if loopIndex < 0 || loopIndex >= len(loops) {
		return false
	}
	return identicalRecord(loops[loopIndex], loop)
}

// identicalRecord reports whether two recorded values are the same record:
// the same dynamic type throughout, and every field, element and float bit
// equal. It is the exact structural comparison profileWalks' own guard rests
// on, and it is deliberately stricter than a numeric comparison — floats are
// compared by their BITS (math.Float64bits), so a value that merely rounds to
// the same number, or a zero of the other sign, is a mismatch rather than a
// match.
//
// The traversal is reflective rather than a per-variant type switch on the
// sealed CurveSegment set, and that is the point: a hand-written comparator
// that forgets a field a variant gains later would go on reporting two
// different records as the same one, which is exactly the failure this guard
// exists to prevent. Reflection covers a new field the moment it is declared.
//
// A shape the traversal does not know — a map, a channel, a function — is
// reported as a mismatch, never as a match. Every refusal here is safe: it
// costs the caller a cached read, which it can always resolve itself, whereas
// a wrong match publishes another section's geometry as this one's.
func identicalRecord(a, b any) bool {
	return identicalRecordValue(reflect.ValueOf(a), reflect.ValueOf(b))
}

// identicalRecordValue is identicalRecord's traversal. The zero reflect.Value
// (a nil interface handed to reflect.ValueOf) matches only another zero one.
func identicalRecordValue(a, b reflect.Value) bool {
	if !a.IsValid() || !b.IsValid() {
		return a.IsValid() == b.IsValid()
	}
	if a.Type() != b.Type() {
		return false
	}
	switch a.Kind() { //nolint:exhaustive // an unhandled kind is a mismatch, by the doc comment above.
	case reflect.Bool:
		return a.Bool() == b.Bool()
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		return a.Int() == b.Int()
	case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64, reflect.Uintptr:
		return a.Uint() == b.Uint()
	case reflect.Float32, reflect.Float64:
		// Float() widens a float32 exactly, so one comparison serves both.
		return math.Float64bits(a.Float()) == math.Float64bits(b.Float())
	case reflect.String:
		return a.String() == b.String()
	case reflect.Struct:
		for i := range a.NumField() {
			// Field reads an unexported field read-only, which is all this
			// traversal ever does — units.Value's own magnitude and unit are
			// unexported and are compared here like any other field.
			if !identicalRecordValue(a.Field(i), b.Field(i)) {
				return false
			}
		}
		return true
	case reflect.Slice, reflect.Array:
		if a.Len() != b.Len() {
			return false
		}
		for i := range a.Len() {
			if !identicalRecordValue(a.Index(i), b.Index(i)) {
				return false
			}
		}
		return true
	case reflect.Interface, reflect.Pointer:
		if a.IsNil() || b.IsNil() {
			return a.IsNil() && b.IsNil()
		}
		return identicalRecordValue(a.Elem(), b.Elem())
	default:
		return false
	}
}

// errResolvedWalksMismatch reports a *profileWalks handed to a consumer that
// does not match the profile it is read against — an evaluator plumbing
// invariant break, never a caller-reachable refusal: every call site in this
// package resolves walks from the exact profile it later reads them against,
// so reaching this error means a future edit broke that pairing, not that the
// recorded geometry is at fault.
var errResolvedWalksMismatch = fmt.Errorf(`%w: resolved walks do not match the profile they are read against`, ErrUnsupported)

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
		tangentBound := lineWalkTangentBound(seg, du, dv)
		return segmentWalk{
			startU: u0, startV: v0, endU: u1, endV: v1,
			startBound: lineWalkEndBound(seg, seg.TStart, u0, v0),
			endBound:   lineWalkEndBound(seg, seg.TEnd, u1, v1),
			tanInU:     du, tanInV: dv, tanOutU: du, tanOutV: dv,
			tanInBound:  tangentBound,
			tanOutBound: tangentBound,
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
		w.startBound = circularWalkEndBound(seg, seg.TStart, w.startU, w.startV)
		w.endBound = circularWalkEndBound(seg, seg.TEnd, w.endU, w.endV)
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
		w.radiusBound = arcWalkRadiusBound(seg, radius)
		pinArcWalkEnds(&w, seg)
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

// lineWalkTangentBound is the single owner of the proven bound on a line
// walk's tangent, and arcWalkRadiusBound's twin one field over: the record
// states the segment's endpoints and its parameter range, never the tangent,
// so the walk's held tangent is the float difference u1−u0, v1−v0 of two
// endpoints the float lerp already rounded. The tangent the record DENOTES is
// the difference of the exact rational lerps (ratLerp), which carries no
// rounding at either step, and the bound is the wider of the two components'
// gaps from it, rounded outward. A lerp that is not representable as a
// rational yields +Inf — the underivable bound consumers refuse on.
func lineWalkTangentBound(seg LineSeg, heldU, heldV float64) float64 {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return math.Inf(1)
	}
	return math.Max(
		rationalFloatError(new(big.Rat).Sub(u1, u0), heldU),
		rationalFloatError(new(big.Rat).Sub(v1, v0), heldV),
	)
}

// lineWalkEndBound is the single owner of the proven bound on a LINE walk's
// endpoint, and lineWalkTangentBound's twin one field over: the record states
// the segment's endpoints and its parameter range, never the point at a trimmed
// parameter, so the walk's held endpoint is lerp2's float evaluation. The point
// the record DENOTES is the exact rational lerp (ratLerp), which carries no
// rounding at either step, and the bound is the wider of the two components'
// gaps from it, rounded outward. A natural bound needs no argument of its own:
// lerp2 and ratLerp both special-case t = 0 and t = 1 to the recorded Point2
// verbatim, so the two agree exactly and this answers zero. A lerp that is not
// representable as a rational yields +Inf — the underivable bound consumers
// refuse on.
func lineWalkEndBound(seg LineSeg, t, heldU, heldV float64) walkEndBound {
	return walkEndBound{
		u: rationalFloatError(ratLerp(seg.Start.U, seg.End.U, t), heldU),
		v: rationalFloatError(ratLerp(seg.Start.V, seg.End.V, t), heldV),
	}
}

// circularWalkEndBound is the single owner of the proven bound on a CIRCULAR
// walk's endpoint: circularWalk reaches every endpoint through math.Sincos at
// an angle this package computed — a CircleSeg's from a float multiply by 2π,
// an ArcSeg's from math.Atan2 of the recorded differences — and neither the
// trig nor its argument is a quantity that walk can enclose from the record
// alone (circularWalk's own comment). circularEndpointInterval encloses the
// point the record DENOTES at that parameter instead, from the recorded data
// and certified trigonometry, and each component's bound is its own gap from
// that enclosure.
//
// An enclosure the recorded data cannot state yields +Inf — an underivable
// bound, which every consumer refuses on rather than publishes.
func circularWalkEndBound(seg CurveSegment, t, heldU, heldV float64) walkEndBound {
	rt := floatRat(t)
	if rt == nil {
		return walkEndBound{u: math.Inf(1), v: math.Inf(1)}
	}
	return circularPointBound(seg, rt, heldU, heldV)
}

// circularPointBound is circularWalkEndBound read at an EXACT RATIONAL
// parameter rather than a held float, and owns the derivation both spellings
// share. It exists for a caller that generates a point at a parameter the
// record's own arithmetic states exactly — a uniform station division
// t_k = TStart + (k/m)·(TEnd − TStart) (loft_build.go's circularStationChain)
// is the one such caller today. Rounding that parameter to a float first would
// enclose the recorded curve at a NEIGHBOURING parameter, and the bound would
// then be a proof about a point the construction never named: the cells either
// side of it would no longer divide the sweep uniformly, the division
// docs/loft-design.md §5.2's per-cell sagitta row derives that term over.
//
// An enclosure the recorded data cannot state yields +Inf on both components,
// the underivable bound every consumer refuses on.
func circularPointBound(seg CurveSegment, t *big.Rat, heldU, heldV float64) walkEndBound {
	uIv, vIv, ok := circularEndpointInterval(seg, t)
	if !ok {
		return walkEndBound{u: math.Inf(1), v: math.Inf(1)}
	}
	return walkEndBound{
		u: intervalFloatError(uIv, heldU),
		v: intervalFloatError(vIv, heldV),
	}
}

// arcWalkRadiusBound is the single owner of the proven bound on an ArcSeg
// walk's radius, and the reason segmentWalk carries radiusBound at all: the
// record states Start and Center, never the radius, so the walk's held radius
// is the float math.Hypot of their difference. The exact radius is
// √((Su−Cu)² + (Sv−Cv)²) over the recorded coordinates, which ratSqrtDown and
// ratSqrtUp bracket without rounding, and the bound is the wider side of that
// bracket about the held float, rounded outward. A bracket that overflows
// yields +Inf — an underivable bound, which every consumer refuses on rather
// than publishes.
func arcWalkRadiusBound(seg ArcSeg, held float64) float64 {
	dx := exactCoordinateDelta(seg.Start.U, seg.Center.U)
	dy := exactCoordinateDelta(seg.Start.V, seg.Center.V)
	r2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	rLo, rHi := ratSqrtDown(r2), ratSqrtUp(r2)
	if isNonFinite(rLo) || isNonFinite(rHi) {
		return math.Inf(1)
	}
	return math.Max(upRound(held-rLo), upRound(rHi-held))
}

// freeformWalk resolves a Tier A free-form segment into its walk geometry
// (docs/spline-design.md Table F). Every field it fills is a proof:
//
//   - the endpoints are the converted chain's own first and last control
//     points, which a Bézier interpolates exactly, each under the bound of the
//     one rounding that conversion committed (freeformEndpointBounds);
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
	tangents, err := freeformEndTangents(spans, reversed)
	if err != nil {
		return segmentWalk{}, err
	}
	startBound, endBound := freeformEndpointBounds(spans, reversed, start, end)
	return segmentWalk{
		startU: start.U, startV: start.V,
		endU: end.U, endV: end.V,
		startBound: startBound,
		endBound:   endBound,
		// A closed free-form curve returns to its start, so it carries no
		// junction vertex — the same fact CircleSeg's closed walk states.
		closed:          start == end,
		tanInU:          tangents.inU,
		tanInV:          tangents.inV,
		tanInBound:      tangents.inBound,
		tanOutU:         tangents.outU,
		tanOutV:         tangents.outV,
		tanOutBound:     tangents.outBound,
		length:          length,
		lengthBound:     bound,
		lengthUpper:     upRound(length + bound),
		coordUpper:      freeformControlExtent(spans),
		kind:            walkFreeform,
		spans:           spans,
		reversed:        reversed,
		fitInterpolated: isFitSplineSeg(seg),
	}, nil
}

// errFreeformWalkUncounted is the refusal of a free-form resolution handed no
// record counter. It is ErrUnsupported because the curve exists and this
// evaluator declines to resolve it without the ceiling §5.2 requires — never a
// silently minted counter, which is the second full ceiling the rule forbids.
var errFreeformWalkUncounted = fmt.Errorf(
	`%w: a free-form segment's walk needs its record's free-form work counter`, ErrUnsupported,
)

// pinArcWalkEnds states an arc walk's natural bounds as the record's own
// endpoints. A recorded arc runs Start → End over [0, 1] about Center
// (record.go), so its value at t = 0 is Start and at t = 1 is End, exactly,
// while circularWalk reaches those same two points through atan2 and cos/sin —
// a route that need not land back on them, because the angle it evaluates at
// the far bound is itself the rounded a0 + sweep. Only the two endpoints are
// restated; the walk's centre, radius, angles and tangents keep circularWalk's
// own values, and every reading derived from them keeps its own bound.
//
// This is the rule lerp2 (moments.go) applies at a line's own bounds, and the
// rule seam.go's edgeJoin applies when it reads an uncut bound off the record
// rather than off sketch's node. It matters for the same reason: buildPrismScene
// (prism_boolean.go) creates one sketch point per walked endpoint, so a walk
// that missed the vertex two segments share would offer sketch two points where
// the record states one, and RecordProfile would then refuse the region the
// arrangement admits on its own proximity threshold.
//
// A trimmed bound's POSITION is left alone: it has no recorded coordinate of
// its own, and inventing one is what this seam never does. What it does get is
// the bound circularWalk's route actually owes — see arcWalkEnd, which owns the
// natural-bound test for both readings so the pinned position and the zero
// bound can never drift apart.
func pinArcWalkEnds(w *segmentWalk, seg ArcSeg) {
	w.startU, w.startV, w.startBound = arcWalkEnd(seg, seg.TStart, w.startU, w.startV)
	w.endU, w.endV, w.endBound = arcWalkEnd(seg, seg.TEnd, w.endU, w.endV)
}

// arcWalkEnd states one arc walk end: its position and the proven bound on each
// of its components. At a natural bound the record states the point verbatim,
// so the walk reads Start or End and the bound is zero — the pin and the zero
// are one decision, taken here once. At any other parameter the walk keeps
// circularWalk's own held pair under the bound circularWalkEndBound proves for
// it.
//
// What the natural-bound zero states is that the held pair IS the recorded
// coordinate, with no rounding of this walk's own. It does NOT state that the
// recorded coordinate is the point the DENOTED curve passes through there. For
// an arc the two coincide at t == 0 and need not at t == 1: the denoted curve
// takes its radius from Start alone (circularEndpointInterval, moments.go), so
// its t == 1 point sits at Start's radius and End's angle, which is the
// recorded End only where the two recorded radii are equal — an equality
// nothing in this package certifies. A consumer that publishes a station's
// displacement from the DENOTED point owes that radial residual on top of this
// zero; docs/loft-design.md §5.2 names the term and loft_build.go's
// arcNaturalEndRadialUpper charges it for the loft.
func arcWalkEnd(seg ArcSeg, t, heldU, heldV float64) (float64, float64, walkEndBound) {
	switch t {
	case 0:
		return seg.Start.U, seg.Start.V, walkEndBound{}
	case 1:
		return seg.End.U, seg.End.V, walkEndBound{}
	}
	return heldU, heldV, circularWalkEndBound(seg, t, heldU, heldV)
}

// circularWalk builds the walk geometry of a circular path about (cu, cv).
//
// Its tangents REFUSE a bound (+Inf): the held components are math.Sincos
// evaluations at th0/th1, and those angles are themselves computed — a
// CircleSeg's from a float multiply by 2π, an ArcSeg's from math.Atan2 of the
// recorded differences — so neither the trig nor its argument is a quantity
// THIS function can enclose, holding floats alone. Stating zero there would
// hand a consumer an exactness the evaluator never proved; +Inf makes the
// absence visible, which is what every consumer refuses on.
//
// The endpoints are the same floats and carry the same absence, but they are
// not left at it: each caller holds the recorded segment those floats came
// from, and stamps the enclosure that record proves for its own endpoints
// (circularWalkEndBound) over the zero this function leaves behind. What has no
// enclosure is the tangent, not the point.
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
		tanInBound:  math.Inf(1),
		tanOutBound: math.Inf(1),
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
		// The merged walk leaves where b leaves, so it inherits b's leaving
		// tangent AND the bound b proved on it — never a's, and never zero.
		a.tanOutU, a.tanOutV = b.tanOutU, b.tanOutV
		a.tanOutBound = b.tanOutBound
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

// freeformVertexAllow folds a junction vertex's bound with a FREE-FORM walk's
// own endpoint bound (bounds.go's walkEndBoundAllow), and answers zero for
// every other kind. A vertex the payload recorded is exact only where every
// coordinate feeding it is; a free-form walk's endpoint is the converted
// chain's own control point (§5.1), read into float64 by the one rounding
// walkEndBoundAllow measures, so a vertex it touches must carry that rounding
// too (topology.go's Vertex.Position contract). An analytic walk's own
// endpoint bound is a separate question buildLoopSidesAs does not answer here.
func freeformVertexAllow(w segmentWalk, bound walkEndBound) float64 {
	if w.kind != walkFreeform {
		return 0
	}
	return walkEndBoundAllow(bound)
}

// buildLoopSides builds one loop's side faces with shared vertices and
// edges, returning the faces, the bottom and top cap coedges in walk order,
// and the loop's perimeter length. A loop's index is both its role index and,
// via li != 0, its orientation: loop 0 is an outer loop (material inside),
// every other a hole (material outside).
//
// resolved is pp.profile's pre-resolved segment walks, or nil to resolve each
// segment through walkOf as before (this file's profileWalks doc comment).
// buildLoopSides's own li IS resolved's loop index here — li walks the same
// append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) order a
// *profileWalks was resolved from — so it is passed straight through as
// buildLoopSidesAs's roleLoop, which resolved is read against.
func buildLoopSides(ctx context.Context, body *Body, ref StepRef, pp prismPayload, li int, loop LoopRecord, work *freeformWork, resolved *profileWalks) ([]*Face, []coedge, []coedge, boundedScalar, error) {
	return buildLoopSidesAs(ctx, body, ref, pp, li, li != 0, loop, work, resolved)
}

// buildLoopSidesAs is buildLoopSides with the role index and the orientation
// decoupled: roleLoop names the walls' side(roleLoop,j) role, and holeLoop
// picks the material side of a straight wall independently. A cup's cavity
// walks each loop of the void the SOLID wraps — the void's outer boundary is a
// hole in the solid (holeLoop true), each of the void's own holes a solid post
// (holeLoop false) — a pairing the natural li != 0 rule cannot express
// (docs/modify-design.md §9).
//
// resolved is a *profileWalks whose loop index roleLoop holds this loop's
// pre-resolved walks, or nil to resolve each segment through walkOf as
// before. A non-nil resolved whose loop at roleLoop was not resolved from
// exactly this loop's recorded segments (loopMatches — the segments
// themselves, not their count) is a plumbing bug and refuses rather than
// silently resolving anyway — the only caller that ever passes non-nil is
// buildLoopSides from evalPrismContext, where roleLoop already IS the loop
// index the *profileWalks was resolved at.
func buildLoopSidesAs(ctx context.Context, body *Body, ref StepRef, pp prismPayload, roleLoop int, holeLoop bool, loop LoopRecord, work *freeformWork, resolved *profileWalks) ([]*Face, []coedge, []coedge, boundedScalar, error) {
	if err := ctx.Err(); err != nil {
		return nil, nil, nil, boundedScalar{}, err
	}
	if len(loop.Segments) == 0 {
		return nil, nil, nil, boundedScalar{}, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	var loopWalks []segmentWalk
	if resolved != nil {
		if !resolved.loopMatches(roleLoop, loop) {
			return nil, nil, nil, boundedScalar{}, errResolvedWalksMismatch
		}
		loopWalks = resolved.loopWalks(roleLoop)
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
		// A Tier B or Tier C free-form kind (a conic, a whole ellipse, an
		// unequal-weight NURBS, or an elliptical arc) refuses inside walkOf
		// itself (freeformBezierSpans, Table R R2/R10) — this build stages no
		// gate of its own ahead of it any more (§10 P4b retires R6). A
		// resolved walk was already through walkOf once (resolveProfileWalks),
		// so it carries the same refusal already surfaced there.
		var w segmentWalk
		if loopWalks != nil {
			w = loopWalks[i]
		} else {
			w, err = walkOf(seg, work)
			if err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
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
	// A junction touching a FREE-FORM walk's own end also folds in that walk's
	// own endpoint bound (freeformVertexAllow, bounds.go's walkEndBoundAllow) —
	// the one rounding §5.1's exact-rational Bézier conversion committed taking
	// the endpoint into float64 (topology.go's Vertex.Position contract: a
	// COMPUTED coordinate carries its own computation's proven displacement).
	// An analytic walk contributes nothing here: widening a trimmed circular
	// walk's vertex is a separate question this build does not answer by
	// accident.
	bottomBoundBase := absSumUpper(delta, pp.z0Delta)
	topBoundBase := absSumUpper(delta, pp.z1Delta)
	var bottomV, topV []*Vertex
	// seamBottom/seamTop are the SINGLE seam vertex a lone closed walk's rim
	// edges share at each cap — one per cap, no junction vertex at all — the
	// free-form twin of a whole CircleSeg's own seam vertex. Hoisted out of the
	// per-kind switch below (§10 P4b): a closed free-form walk reaches
	// singleClosed on the same terms a whole circle does, and the switch's
	// walkFreeform arm needs the SAME seam pair the walkCircular arm builds.
	var seamBottom, seamTop *Vertex
	if singleClosed {
		w := walks[0]
		extra := math.Max(freeformVertexAllow(w.segmentWalk, w.startBound), freeformVertexAllow(w.segmentWalk, w.endBound))
		seamBottom = &Vertex{position: pp.point(w.startU, w.startV, pp.z0), bound: units.Millimeters(absSumUpper(bottomBoundBase, extra))}
		seamTop = &Vertex{position: pp.point(w.startU, w.startV, pp.z1), bound: units.Millimeters(absSumUpper(topBoundBase, extra))}
	} else {
		bottomV = make([]*Vertex, n)
		topV = make([]*Vertex, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
			prev := walks[(i+n-1)%n]
			extra := math.Max(freeformVertexAllow(w.segmentWalk, w.startBound), freeformVertexAllow(prev.segmentWalk, prev.endBound))
			bottomV[i] = &Vertex{position: pp.point(w.startU, w.startV, pp.z0), bound: units.Millimeters(absSumUpper(bottomBoundBase, extra))}
			topV[i] = &Vertex{position: pp.point(w.startU, w.startV, pp.z1), bound: units.Millimeters(absSumUpper(topBoundBase, extra))}
		}
	}

	// Vertical edges at junctions, shared between side face i−1 and i.
	// Convexity from the 2D turn: a positive cross of the incoming and
	// outgoing tangents is a left turn — interior angle < π — which is a
	// convex edge on the outer loop and works out identically for hole
	// loops walked clockwise. A free-form walk's end tangent is the
	// hodograph's own exact-rational leg (§5.1), rounded once into float64
	// with its own stated bound (tanInBound/tanOutBound) — at least as well
	// founded as the circular walk's own tangent, whose bound is +Inf by
	// segmentWalk's convention because it comes from trig at a computed
	// angle. This cross needs no change for either kind.
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
		if singleClosed {
			bStart, bEnd = seamBottom, seamBottom
			tStart, tEnd = seamTop, seamTop
		} else {
			bStart, tStart = bottomV[i], topV[i]
			bEnd, tEnd = bottomV[(i+1)%n], topV[(i+1)%n]
		}
		var bottomEdge, topEdge *Edge
		var surf Surface
		faceReversed := false
		switch w.kind {
		case walkCircular:
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
				// edge, start == end (topology.go's Circle3 contract) — the
				// seamBottom/seamTop pair the switch's caller already built.
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
		case walkFreeform:
			// §6.5's wall-edge convexity certificate, wired here for the first
			// time (docs/spline-design.md §6.5, Table R R19). It applies the
			// one reversal negation internally and states its verdict in the
			// LOOP'S OWN WALK direction, so the mapping below reads it
			// verbatim — a counter-clockwise turn convex, clockwise concave,
			// the identical convention the circular wall's own turn test
			// fixes above. NEVER negate again for a hole loop: a hole rim's
			// concavity falls out of the clockwise walk itself, exactly as it
			// does for a circular hole wall. An error is R19: propagate it so
			// the build refuses and no step commits.
			verdict, err := freeformWallConvexityContext(ctx, w.spans, w.closed, w.reversed, w.fitInterpolated, work)
			if err != nil {
				return nil, nil, nil, boundedScalar{}, err
			}
			var convex bool
			switch verdict {
			case freeformConvexityPositive:
				convex = true
			case freeformConvexityNegative:
				convex = false
			case freeformConvexityStraight:
				// Every live span lies on one line and no joint turns off it
				// (§6.5's Table K): the chain has no turn of its own, so
				// evaluator §3's straight-wall rule decides it by its loop's
				// role — the same expression the walkLine arm below uses.
				convex = !holeLoop
			}
			bottomEdge = &Edge{curve: NURBSCurve{}, start: bStart, end: bEnd, convex: convex, length: w.length, lengthBound: w.lengthBound}
			topEdge = &Edge{curve: NURBSCurve{}, start: tStart, end: tEnd, convex: convex, length: w.length, lengthBound: w.lengthBound}
			surf = NURBSSurface{}
			// faceReversed stays false: an opaque NURBSSurface publishes no
			// normal at all (§7's NormalAt refusal, topology.go), so
			// Face.reversed — "the outward normal is the surface's geometric
			// normal negated" — names nothing for this variant.
		default:
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

// extentAlong is the through-all stop's reading of the prism (stops.go): the
// extent interval along an arbitrary world direction g — the lifted linear
// functional point·g = origin·g + u·(U'·g) + v·(V'·g) + z·(N'·g), primes the
// placed directions, extremized over the region boundary and the sweep —
// beside the proven displacement extentBoundedAlong states for its two ends.
// The stop charges that displacement to the level it resolves and decides its
// own in-path test outside it, so a boundary extreme held by a bracket
// (docs/spline-design.md §6.2) still answers rather than refusing.
//
// The section displacement is NOT one of the terms this reading carries — it
// moves a coordinate IN the plane, and the interval is stated over the
// recorded section — so a prism holding one refuses instead
// (docs/prism-boolean-design.md §12). prismBoundsContext reads
// extentBoundedAlong directly and composes both terms into its own outward
// bound.
// The stop and clearance callers hold no preflight counter for this record, so
// the interface forms open the record's own — one per extent reading, never one
// per segment.
func (pp prismPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	if pp.sectionDelta != 0 {
		return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a prism with a proven section displacement`, ErrUnsupported)
	}
	return pp.extentBoundedAlong(context.Background(), g, newFreeformWork(), nil)
}

func (pp prismPayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return pp.extentAlongWork(ctx, g, newFreeformWork())
}

// extentAlongWork is extentBoundedAlong's refusing wrapper, mirroring
// revolvePayload.extentAlongWork word for word: the reading for a consumer that
// takes the interval as an exact one and has nowhere to put a displacement.
// clearance.go's payloadExtent is that consumer — its separating-plane
// short-circuit compares two bodies' intervals and simply loses the
// short-circuit where it cannot get an exact one — so a direction whose extreme
// only a bracket holds refuses here rather than publish a held coordinate as the
// one it denotes. Which candidate holds it does not matter and the refusal never
// names a kind: a free-form span's enclosure, a computed arc radius and a walked
// endpoint the record does not state all reach this wrapper the same way,
// through one nonzero bound. A through-all stop does not read through this wrapper:
// it consumes the bounded reading and charges the displacement to its own level
// (stops.go, docs/spline-design.md §6.4).
func (pp prismPayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, error) {
	lo, hi, bound, err := pp.extentBoundedAlong(ctx, g, work, nil)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: this prism's extent along this direction is known only to a proven displacement of %v mm; this reading has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// extentBoundedAlong is the bounded reading itself: the interval AND its
// proven half-width, folded from boundaryExtremesBoundedContext's own
// per-candidate enclosures (docs/spline-design.md §6.2) AND from the frame and
// placement's own rounding (prismPlacementCoeffAllow). The boundary-scan term
// follows the CANDIDATES the extremes are held by, never the section's kind: a
// section whose extremes are all values the record states — straight walls,
// and an arc or circle read where its own recorded endpoint or its exactly
// representable apex wins — carries only zero-width candidates, while a
// trimmed circular endpoint, a computed arc radius or a free-form span's
// enclosure each publish the width their own construction owes. The frame and
// placement term is independent of it and composes outward: a straight-walled
// section under a tilted placement still widens, and an unplaced or
// axis-aligned section still reports zero for this term, which is what keeps
// an ordinary prism's box Exact.
//
// A THIRD term composes outward with both: the reading's own final summation
// base + lo + zlo, charged exactly against the same terms by exactSumRound
// (bounds.go). It is not covered by either of the other two — a pure
// translation leaves every coefficient exactly right and every multiply exact,
// and the addition that follows still rounds — and it is zero exactly where
// that addition is exactly representable, so an unplaced prism's box stays
// Exact.
//
// walks is pp.profile's pre-resolved segment walks, or nil to resolve as
// before through boundaryExtremesBoundedContext and profileCoordinateEnvelope's
// own walkOf calls. prismBoundsContext passes the same *profileWalks to every
// one of its three per-axis calls, so the record's boundary walks resolve
// once for the whole box rather than once per axis (this file's profileWalks
// doc comment).
func (pp prismPayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeformWork, walks *profileWalks) (float64, float64, float64, error) {
	base := pp.xform.Apply(pp.frame.Origin()).Dot(g)
	gu := pp.dir(1, 0, 0).Dot(g)
	gv := pp.dir(0, 1, 0).Dot(g)
	gz := pp.dir(0, 0, 1).Dot(g)
	lo, hi, bound, err := boundaryExtremesBoundedContext(ctx, pp.profile, gu, gv, work, walks)
	if err != nil {
		return 0, 0, 0, err
	}
	zlo := math.Min(pp.z0*gz, pp.z1*gz)
	zhi := math.Max(pp.z0*gz, pp.z1*gz)
	coordUpper, err := profileCoordinateEnvelope(pp.profile, work, walks)
	if err != nil {
		return 0, 0, 0, err
	}
	zUpper := math.Max(math.Abs(pp.z0), math.Abs(pp.z1))
	placeAllow := prismPlacementCoeffAllow(pp, g, base, gu, gv, gz, coordUpper, zUpper)
	// The recombination is charged per ENDPOINT and composed outward: the two
	// ends are summed from different terms and round by different amounts, while
	// the scan's and the placement's terms speak for both ends alike, so the
	// reading publishes the larger of the two per-end totals — the same shape
	// revolvePayload.extentBoundedAlong states for its own per-end composition.
	loEnd, hiEnd := base+lo+zlo, base+hi+zhi
	sumAllow := math.Max(
		exactSumRound(loEnd, base, lo, zlo),
		exactSumRound(hiEnd, base, hi, zhi),
	)
	bound = absSumUpper(bound, placeAllow, sumAllow)
	return loEnd, hiEnd, bound, nil
}

// prismPlacementCoeffAllow bounds how far base/gu/gv/gz — the four scalar
// coefficients extentBoundedAlong lifts the boundary and sweep extremes
// through — can sit from the value the SAME frame-and-placement chain's exact
// arithmetic would give, through exactIsometryDotRound's rational check
// (bounds.go): zero exactly where the frame is axis-aligned and the placement
// is the identity, nonzero only where that isometry's own float evaluation
// genuinely rounds. Each coefficient's own displacement moves the published
// extreme at the rate of the coordinate it multiplies —
// directionalPerturbationAllow's own Lipschitz shape, coordUpper for gu/gv and
// zUpper for gz — while base's displaces the extreme directly, at both ends
// alike, since it is the section's own constant offset under this direction.
// capBlendPayload.extentBoundedAlong reuses this unchanged through
// prismLike's shared frame and placement. A second, independent term
// (prismDecompositionRoundAllow) covers the rounding this reading's own
// DECOMPOSITION gu*u + gv*v + gz*z commits even when every coefficient is
// itself exactly right: multiplying an EXACT but non-trivial coefficient by a
// recorded coordinate still rounds, and grouping the sum this way (the
// boundary scan's own gu*u+gv*v first) is a DIFFERENT float computation than
// applying the frame and placement to the point directly, even though the two
// are equal in exact arithmetic. The recombination with base that FOLLOWS is a
// third term, charged exactly at the call site rather than here
// (exactSumRound), because it rounds for placements this function's own check
// proves exact — a translation is committed there and nowhere else.
func prismPlacementCoeffAllow(pp prismPayload, g r3.Vec, base, gu, gv, gz, coordUpper, zUpper float64) float64 {
	baseRound := exactIsometryDotRound(pp.xform, pp.frame.Origin(), g, true, base)
	guRound := exactIsometryDotRound(pp.xform, pp.frame.U(), g, false, gu)
	gvRound := exactIsometryDotRound(pp.xform, pp.frame.V(), g, false, gv)
	gzRound := exactIsometryDotRound(pp.xform, pp.frame.N(), g, false, gz)
	return absSumUpper(
		baseRound,
		directionalPerturbationAllow(guRound, coordUpper),
		directionalPerturbationAllow(gvRound, coordUpper),
		directionalPerturbationAllow(gzRound, zUpper),
		prismDecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper),
	)
}

// prismDecompositionRoundAllow bounds the rounding the MULTIPLY-AND-SUM
// combination base + gu*u + gv*v + gz*z commits, given base/gu/gv/gz
// themselves proven exact against the isometry that produced them
// (prismPlacementCoeffAllow's own exactIsometryDotRound check, above). IEEE
// 754 multiplies exactly by 0, 1 or -1 for ANY operand — those three values
// are the only ones that never round a multiply — so a coefficient outside
// that set can round when it multiplies a recorded coordinate, and this
// reading's own left-to-right summation order (the boundary-extreme scan's
// gu*u+gv*v first, then +base, then +gz*z — a DIFFERENT grouping than
// applying the frame and placement to the point directly) can round again on
// top of that even where every individual multiply happens not to. Both are
// genuinely new roundings a non-axis-permuting frame commits, so the term is
// zero where every coefficient is trivial — and otherwise reuses
// analyticRoundBound's own established "a bounded number of basic ops at a
// magnitude" contract: at most 3 multiplies and 3 additions here, far under
// its 128-operation budget. |base| stays in that envelope because the same
// left-to-right evaluation folds base in, and charging it in both arms is only
// ever wider than the non-trivial arm owes.
//
// The trivial arm's zero speaks for the DECOMPOSITION alone, never for the
// whole endpoint. g is a unit world axis and the frame orthonormal, so
// gu²+gv²+gz² = 1 and an all-trivial reading has exactly one coefficient at
// ±1 with the other two at 0: every multiply is exact and so is the scan's own
// gu*u+gv*v, whatever the coordinates are. What that argument does NOT reach
// is the recombination with base and the sweep level, which rounds for a
// coefficient set this arm calls trivial — a pure translation is exactly that
// case — so extentBoundedAlong charges it separately and exactly through
// exactSumRound (bounds.go), and this term must never be read as covering it.
func prismDecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper float64) float64 {
	trivial := func(c float64) bool { return c == 0 || c == 1 || c == -1 }
	if trivial(gu) && trivial(gv) && trivial(gz) {
		return 0
	}
	// The non-trivial arm is loose, and it is loose in the safe direction: one
	// fixture measures both halves of that. A 5x5 mm section swept 1 mm on the
	// UNPLACED frame U=(0.6,0.8,0), V=(-0.8,0.6,0) reports Min=(-4,0,0),
	// Max=(3,7,1), Approximate, bound 3.9790393202565666e-13. That whole figure
	// is this term: divide it by analyticRoundBound's own 256*unitRoundoff and
	// it comes to 14 up to that helper's outward rounding, which is the scale
	// below — |gu|*coordUpper + |gv|*coordUpper = 0.6*10 + 0.8*10, the walk's
	// coordUpper being 10 for that section — while every sibling term answers
	// zero: exactIsometryDotRound on base/gu/gv/gz under the identity xform,
	// exactSumRound on base 0 with levels 0 and 1, and both displacement terms
	// prismBoundsContext composes. A zero bound on that fixture would be
	// unsound rather than tighter, because the extremes it would call exact are
	// not: summing this frame's OWN held entries over the rationals against the
	// section corners {0,5}^2 gives X-min = -5*float64(0.8), X-max =
	// 5*float64(0.6) and Y-max = 5*float64(0.6) + 5*float64(0.8), none of them
	// representable, since float64(0.8) sits above 4/5 and 5*float64(0.8) needs
	// 55 significand bits — it is exactly 4 + 2^-52, a QUARTER of the 2^-50 ulp
	// above 4, which ordinary round-to-nearest returns as 4 with no tie. Each of
	// the three published coordinates therefore misses its true extreme by a
	// representable amount (2.22e-16 in X-min, 1.11e-16 in X-max and Y-max),
	// X-min and Y-max landing INSIDE the true extreme and X-max landing outward
	// of it, a miss this term covers with wide margin.
	scale := absSumUpper(
		productUpper(math.Abs(gu), coordUpper),
		productUpper(math.Abs(gv), coordUpper),
		productUpper(math.Abs(gz), zUpper),
		math.Abs(base),
	)
	return analyticRoundBound(scale)
}

// prismBoundsContext computes the exact axis-aligned bounds of the placed prism:
// for each world axis, the directional extreme of the region boundary under
// the lifted linear functional, plus the sweep's own extreme
// (docs/evaluator-design.md §5).
//
// walks is pp.profile's pre-resolved segment walks, or nil to resolve each
// segment through walkOf as before. Passed straight to all three per-axis
// extentBoundedAlong calls below (this file's profileWalks doc comment), so a
// non-nil walks resolves the record's boundary once for the whole box instead
// of once per axis.
func prismBoundsContext(ctx context.Context, pp prismPayload, work *freeformWork, walks *profileWalks) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	extremeBound := 0.0
	for i, axis := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, bound, err := pp.extentBoundedAlong(ctx, axis, work, walks)
		if err != nil {
			return Box{}, err
		}
		minC[i] = lo
		maxC[i] = hi
		extremeBound = math.Max(extremeBound, bound)
	}
	// A displaced section displaces every extreme it holds, so the box's own
	// error carries the section displacement itself — δ outward on every face
	// (docs/prism-boolean-design.md §7) — summed with the boundary's own
	// directional-extreme bracket bound (docs/spline-design.md §6.2) and with
	// the frame and placement's own rounding (prismPlacementCoeffAllow) and the
	// endpoint summation's (exactSumRound), both folded into
	// extentBoundedAlong's own returned bound above: the frame and
	// placement ARE isometries in exact arithmetic, but their FLOAT evaluation
	// rounds wherever the frame is not axis-aligned or the placement is not the
	// identity, and adding the resulting terms into one published coordinate
	// rounds again — for a pure translation it is the ONLY rounding there is —
	// so a box that reads either as an exact leaf can miss
	// the true extreme by a representable amount. All three terms are zero for
	// a caller-drawn, unplaced, axis-aligned payload whose extremes are all
	// values its record states, which keeps the ordinary prism's box Exact as
	// before. The bracket's own term is what decides the rest, never the
	// section's kind: a straight-walled section reports zero, an analytic one whose extreme is
	// held by a trimmed circular endpoint or a computed arc radius reports that
	// candidate's own width, and a free-form section whose extremes along
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

// circularExtremeInterval encloses the two EXACT extremes of the functional
// g(u, v) = gu·u + gv·v over the whole circle a circular walk lies on. Writing
// the walk as c + r·(cos θ, sin θ) gives g(θ) = (gu·cU + gv·cV) + r·|(gu, gv)|·
// cos(θ − θ*), so the circle's own minimum and maximum are P ∓ r·|(gu, gv)| —
// an identity in which the angle does not appear at all.
//
// That is why the scan's circular candidate is bounded from here rather than
// from a certified sine and cosine at the candidate's own angle: the angle is a
// SELECTION (does the walk sweep the apex?), while the VALUE the fold publishes
// is this closed form, and reading it this way charges no π-rounding guard for
// an angle that never enters the answer. The three inputs that are not exact
// leaves each enter through the bounded arithmetic: the walk's radius under its
// own proven bound (segmentWalk.radiusBound — an ArcSeg states Start and Center,
// so its radius is a math.Hypot), the direction's magnitude through
// boundedNorm2's certified square-root brackets, and every product and sum
// through boundedMul/boundedAdd's own exact rounding terms. An exactly
// representable apex therefore still reports a zero bound, which is what lets a
// recorded circle's box stay Exact along an axis whose reading the apex holds —
// the walk's own endpoints answer for themselves there
// (segmentWalk.startBound/endBound).
func circularExtremeInterval(w segmentWalk, gu, gv float64) (boundedScalar, boundedScalar) {
	gmag := boundedNorm2(exactScalar(gu), exactScalar(gv))
	centre := boundedAdd(
		boundedMul(exactScalar(gu), exactScalar(w.cU)),
		boundedMul(exactScalar(gv), exactScalar(w.cV)),
	)
	amplitude := boundedMul(measuredScalar(w.radius, w.radiusBound), gmag)
	return boundedSub(centre, amplitude), boundedAdd(centre, amplitude)
}

// boundaryExtremesBoundedContext is the one scan, total over walkKind
// (docs/spline-design.md §6.2): the min and max of g(u, v) = gu·u + gv·v over
// the recorded region's boundary, AND the proven half-width every CANDIDATE's
// own position contributes to that interval.
//
// That half-width is the candidates' POSITIONAL displacement alone. The scan
// evaluates each candidate as the float gu·u + gv·v, and the rounding of that
// multiply-and-sum is the CALLER's to charge, at the coordinate envelope the
// caller's own geometry states: a prism reads it through
// prismDecompositionRoundAllow, a revolve through
// planeDotDecompositionRoundAllow (bounds.go), and a caller that charges
// neither publishes a candidate the record states verbatim — a zero-width one,
// on which this scan reports zero — as if the arithmetic reading it had
// committed nothing.
//
// An ENDPOINT candidate is the walk's own endpoint read through the direction
// the caller holds, which this evaluator reads as an exact leaf throughout (the
// convention survey2d.go's own file comment states). The endpoint itself is an
// exact leaf only where the record STATES it — a line's or an arc's natural
// bounds — and there the candidate has zero width, so an all-straight section's
// reading stays exact. Every other endpoint is one this evaluator computed, and
// the walk states what it is worth (segmentWalk.startBound/endBound);
// pointPerturbationAllow carries that displacement through the functional so
// the candidate enters at the width its own construction owes, never at zero.
// An endpoint whose bound no arithmetic could state refuses the whole scan
// rather than folding an infinity into the accumulators.
//
// An interior CIRCULAR candidate is a second computed reading: its position is
// the walk's radius times a cosine and a sine, and the radius itself is a
// math.Hypot for every ArcSeg, so it enters the fold under the proven enclosure
// circularExtremeInterval derives — the single owner of that term, charged where
// the candidate is produced rather than beside the fold by whichever consumer
// noticed. A free-form walk folds each of its converted spans' own proven
// enclosure (spanExtremeEnclosureContext) and takes no endpoint candidate of its
// own: a span enclosure already covers the span's whole parameter range,
// endpoints included.
//
// The fold is the shipped capBlendPayload.extentBoundedAlong idiom: track the
// lower and upper ends of every candidate contributing to the region minimum
// (loLower/loUpper) and to the region maximum (hiLower/hiUpper) separately, so
// a candidate that loses the extremization contributes nothing to the reported
// bound, and report the midpoint of each composed interval with the larger of
// the two half widths, rounded up — the same convention freeformArcLength
// already uses. The fold is sound because every candidate interval encloses a
// value the true boundary actually attains: the reported minimum's lower end is
// the least of the candidates' lower ends and so never exceeds the truth, and
// its upper end is the least of their upper ends, which the candidate attaining
// the true minimum keeps at or above it.
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
//
// walks is profile's pre-resolved segment walks, or nil to resolve each
// segment through walkOf as before (this file's profileWalks doc comment). A
// non-nil walks that was not resolved from THIS profile's own recorded
// segments refuses.
func boundaryExtremesBoundedContext(ctx context.Context, profile ProfileRecord, gu, gv float64, work *freeformWork, walks *profileWalks) (float64, float64, float64, error) {
	if err := requireFiniteDirection(gu, gv); err != nil {
		return 0, 0, 0, err
	}
	if walks != nil && !walks.matches(profile) {
		return 0, 0, 0, errResolvedWalksMismatch
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
	// take folds one candidate's held value under the proven bound its own
	// generator derived. A zero bound enters as the held value twice: widening
	// an exact candidate by a directed rounding would mint an error the
	// arithmetic provably did not commit. A nonzero one is stepped outward with
	// math.Nextafter rather than upRound/downRound, since a directional value
	// can be negative and those two only move a POSITIVE bound toward zero.
	take := func(g, allow float64) {
		if allow == 0 {
			takeLo(g, g)
			takeHi(g, g)
			return
		}
		lo := math.Nextafter(g-allow, math.Inf(-1))
		hi := math.Nextafter(g+allow, math.Inf(1))
		takeLo(lo, hi)
		takeHi(lo, hi)
	}
	takeVertex := func(u, v float64, bound walkEndBound) {
		take(gu*u+gv*v, pointPerturbationAllow(bound, gu, gv))
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

	for li, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for si, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return 0, 0, 0, err
			}
			w, err := resolveOrRead(seg, work, walks, li, si)
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
			if !w.startBound.derivable() || !w.endBound.derivable() {
				return 0, 0, 0, fmt.Errorf(`%w: a boundary segment's walked endpoint states no proven displacement, so this scan cannot bound the region's extremes`, ErrUnsupported)
			}
			takeVertex(w.startU, w.startV, w.startBound)
			takeVertex(w.endU, w.endV, w.endBound)
			if !w.isCircular() {
				continue
			}
			// Interior extremes at θ* where the functional's gradient
			// aligns with the radius: θ* = atan2(gv, gu) (+π). The angle
			// SELECTS which of the circle's two extremes the walk sweeps;
			// circularExtremeInterval states what that extreme is worth.
			gmag := math.Hypot(gu, gv)
			if gmag == 0 {
				continue
			}
			minIv, maxIv := circularExtremeInterval(w, gu, gv)
			star := math.Atan2(gv, gu)
			tlo, thi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
			for ci, cand := range [2]float64{star, star + math.Pi} {
				apex := maxIv
				if ci == 1 {
					apex = minIv
				}
				for k := math.Floor((tlo-cand)/(2*math.Pi)) * 2 * math.Pi; cand+k <= thi+1e-12; k += 2 * math.Pi {
					th := cand + k
					if th < tlo-1e-12 {
						continue
					}
					held := gu*(w.cU+w.radius*math.Cos(th)) + gv*(w.cV+w.radius*math.Sin(th))
					take(held, boundedFloatError(apex, held))
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
