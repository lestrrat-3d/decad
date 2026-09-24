package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the extrude of docs/evaluator-design.md §5: the feature call
// gates its live inputs, converts the profile to structural records, evaluates
// from those records, and commits atomically. The body-relative stops
// (ThroughAll/ThroughAllSide/ToFace) resolve through stops.go: the stop
// bodies are resolved at the call and tracked by private producer identities.
//
// The evaluation this call drives is spread over three sibling files, each
// with its own doc comment: prism_payload.go holds the record and the
// coordinate readings taken off it, prism_build.go builds the body, and
// prism_extent.go answers the extent questions asked of the result. The
// prism they produce is analytic — Plane and Cylinder faces — for every
// line/circle/arc boundary segment, and a NURBSSurface
// (docs/spline-design.md §7) for a Tier A free-form one, with bounded mass
// measurements throughout. Exactly representable results retain zero bounds.

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
// (docs/evaluator-design.md §5), returned before commit, so the
// document is unchanged — because a tapered extrude of a general region is
// an offset problem, and a wrong-but-confident prism is the failure decad
// exists to prevent. Only a zero taper reaches evaluation.
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
// evaluator converts the profile and plane to structural records; a failed
// evaluation leaves the document untouched. WithSurfaceResult() omits the two
// caps and publishes a sheet body instead of a solid (docs/surface-design.md §4).
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
	surfaceResult := false
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
		case identSurfaceResult:
			// A repeated WithSurfaceResult() is idempotent, matching
			// WithTaper's last-wins tolerance rather than Loft's
			// repeat-is-ErrDegenerate rule.
			surfaceResult = true
		}
	}
	if taper.Kind() != units.Angle {
		return nil, fmt.Errorf(`%w: a taper must be an angle, got %s`, ErrUnitKind, taper.Kind())
	}
	if _, err := taper.In(units.Radian); err != nil {
		return nil, fmt.Errorf(`%w: the taper is not representable: %s`, ErrNotFinite, err)
	}
	if taper.Mag() != 0 {
		// Refused before commit: staging is explicit
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

	ref := d.nextProducerID()
	body, err := evalPrism(d, ref, prismPayload{
		profile:       profile,
		frame:         frame,
		z0:            sweep.z0,
		z1:            sweep.z1,
		z0Delta:       sweep.z0Delta,
		z1Delta:       sweep.z1Delta,
		xform:         r3.Identity(),
		surfaceResult: surfaceResult,
	}, work)
	if err != nil {
		return nil, err
	}
	d.commit(body)
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
// private producer identities of the bodies the extent's stops resolved against. A level the
// caller stated denotes itself and reports a zero displacement; a level the
// resolution COMPUTED reports the rounding that computation committed, which is
// what the prism payload carries into every level-derived reading.
type linearSweep struct {
	z0, z1  float64
	z0Delta float64
	z1Delta float64
	inputs  []producerID
}

// resolveLinearExtent turns a linear extent into that sweep
// (docs/evaluator-design.md §5). The refs are ordered named-extent refs in
// extent order first, through-all stop bodies after them in stop order along
// the sweep, deduplicated. Magnitudes are validated per core
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
		named := append(append([]producerID(nil), one.named...), two.named...)
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
			return linearSweep{z1: stop, z1Delta: delta, inputs: []producerID{ref}}, nil
		}
		return linearSweep{z0: stop, z0Delta: delta, inputs: []producerID{ref}}, nil
	case nil:
		return linearSweep{}, fmt.Errorf(`%w: a nil extent sweeps nothing`, ErrDegenerate)
	default:
		return linearSweep{}, fmt.Errorf(`%w: extent %T is not supported by this evaluator`, ErrUnsupported, e)
	}
}

// linearSide is one resolved side of a TwoSided: its signed boundary
// coordinate along the plane normal, that coordinate's own axial displacement,
// and the stop refs it resolved — named-extent and through-all kept apart so
// the enclosing extent can order them deterministically.
type linearSide struct {
	z       float64
	delta   float64
	named   []producerID
	through []producerID
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
		return linearSide{z: stop, delta: delta, named: []producerID{ref}}, nil
	case nil:
		return linearSide{}, fmt.Errorf(`%w: a two-sided extent requires both sides`, ErrDegenerate)
	default:
		return linearSide{}, fmt.Errorf(`%w: side extent %T is not supported by this evaluator`, ErrUnsupported, s)
	}
}

// This section is ExtrudeChain of docs/surface-design.md §13: the open sketch
// chain's own sweep into a ribbon. It reuses resolveLinearExtent unchanged —
// the extent vocabulary and the frame it resolves against are identical to
// Extrude's — and reads its recorded walk through RecordChain (seam.go)
// rather than RecordProfile. This increment admits exactly one chain of one
// straight LineSeg segment, evaluated by evalChainExtrudeContext below;
// docs/surface-design.md Table G's multi-segment and curved-wall rows are
// staged to a later increment and refuse here with ErrUnsupported.

// ChainExtrudeOption configures ExtrudeChain. It is its own sealed tier
// rather than [ExtrudeOption]: WithSurfaceResult() does not implement it, so
// the compiler refuses that option outright rather than accepting it as a
// no-op — a chain-fed sweep always returns a sheet, so there is no "build a
// solid instead" state for the option to toggle (docs/surface-design.md
// §13.2). No option is a member of this tier yet; it exists so a later
// chain-only option has a tier to land on.
type ChainExtrudeOption interface {
	option.Interface
	chainExtrudeOption()
}

// chainPayload is ExtrudeChain's own record of a ribbon body: the recorded
// open walk, the plane frame it lifts through, the signed sweep interval, and
// the accumulated rigid placement — chainPayload is to ExtrudeChain what
// prismPayload is to Extrude (docs/surface-design.md §13.4). This increment
// admits exactly one segment, a LineSeg, read out once as line so the build
// never re-type-asserts chain.Segments[0].
type chainPayload struct {
	chain   ChainRecord
	line    LineSeg
	frame   r3.Frame
	z0, z1  float64
	z0Delta float64
	z1Delta float64
	xform   r3.Transform
}

func (pp chainPayload) z0Scalar() boundedScalar { return measuredScalar(pp.z0, pp.z0Delta) }
func (pp chainPayload) z1Scalar() boundedScalar { return measuredScalar(pp.z1, pp.z1Delta) }

// prism is a *view* of pp as a zero-section-delta, no-holes prismPayload —
// never a body this evaluator builds — used only to feed pp.point/pp.dir/
// reflected and prismBoundsContext, none of which cares whether the section
// it reads closes. The chain's own segment list stands in for a profile's
// outer loop with no holes: every reader below walks profile.Outer.Segments
// exactly as it would a real profile's, and an open chain's segments are
// exactly that list.
func (pp chainPayload) prism() prismPayload {
	return prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: pp.chain.Segments}},
		frame:   pp.frame,
		z0:      pp.z0, z1: pp.z1,
		z0Delta: pp.z0Delta, z1Delta: pp.z1Delta,
		xform: pp.xform,
	}
}

// transform is the accumulated rigid placement.
func (pp chainPayload) transform() r3.Transform { return pp.xform }

// placed re-evaluates the same record under the composed motion (core §8).
func (pp chainPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	pp.xform = composed
	return evalChainExtrudeContext(ctx, d, ref, pp)
}

// ExtrudeChain sweeps the open chain ch of sketch s along the sketch plane's
// normal per the linear extent e, and registers the new ribbon body. ch MUST
// be a chain of s (ErrForeignProfile) and a current, unaltered snapshot
// (ErrStaleProfile or ErrInvalidProfile); an invalid or self-intersecting
// chain is also ErrInvalidProfile, and a walk decad cannot record exactly is
// ErrUnrecordableProfile (docs/sketch-seam-design.md §2.2). The result is
// always a sheet — Kind() == BodySheet — one wall face per recorded segment
// (docs/surface-design.md §13.4, Table G), with no cap and no closing face:
// WithSurfaceResult() does not compile against this call. This increment
// builds a chain of exactly one straight LineSeg segment; a multi-segment
// chain, or one carrying a circular or free-form segment, is ErrUnsupported,
// staged to a later increment. A failed evaluation leaves the document
// untouched.
func (d *Document) ExtrudeChain(s *sketch.Sketch, ch *sketch.Chain, e Extent, opts ...ChainExtrudeOption) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}

	chain, plane, err := recordChain(s, ch)
	if err != nil {
		return nil, err
	}
	if len(chain.Segments) != 1 {
		return nil, fmt.Errorf(`%w: this evaluator sweeps a chain of exactly one straight segment; a %d-segment chain is staged to a later increment`, ErrUnsupported, len(chain.Segments))
	}
	line, ok := chain.Segments[0].(LineSeg)
	if !ok {
		return nil, fmt.Errorf(`%w: this evaluator sweeps a chain of exactly one straight segment; a %T wall is staged to a later increment`, ErrUnsupported, chain.Segments[0])
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

	ref := d.nextProducerID()
	body, err := evalChainExtrude(d, ref, chainPayload{
		chain:   chain,
		line:    line,
		frame:   frame,
		z0:      sweep.z0,
		z1:      sweep.z1,
		z0Delta: sweep.z0Delta,
		z1Delta: sweep.z1Delta,
		xform:   r3.Identity(),
	})
	if err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// evalChainExtrude builds a ribbon body from pp, over context.Background().
func evalChainExtrude(d *Document, ref producerID, pp chainPayload) (*Body, error) {
	return evalChainExtrudeContext(context.Background(), d, ref, pp)
}

// evalChainExtrudeContext builds the ribbon body: one Plane wall face over the
// chain's own single LineSeg segment, its two rim edges and the one sweep
// edge at each of its two free ends (docs/surface-design.md §13.4, Table G
// row 1), and the measurements the finished body publishes. It mirrors
// evalPrismContext's own order (prism_build.go) without ever building a solid
// or a closing cap: an open chain mints neither.
func evalChainExtrudeContext(ctx context.Context, d *Document, ref producerID, pp chainPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	height := boundedSub(pp.z1Scalar(), pp.z0Scalar())
	if height.value <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}

	w, err := walkOf(pp.line, nil)
	if err != nil {
		return nil, err
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: false, kind: BodySheet}
	prismView := pp.prism()

	// frameLiftAllow is the one proven bound this ribbon's four rim vertices
	// share for the payload's own frame lift and accumulated placement
	// (bounds.go's frameAndPlacementRoundAllow) — exactly zero for an
	// axis-aligned, unplaced payload, which is what keeps a plain
	// ExtrudeChain's rim vertices Exact, mirroring buildLoopSidesAs's own
	// frameLiftAllow (prism_build.go).
	frameLiftAllow := frameAndPlacementRoundAllow(pp.frame, pp.xform, math.Max(w.coordUpper, math.Max(math.Abs(pp.z0), math.Abs(pp.z1))))
	bottomBoundBase := absSumUpper(pp.z0Delta, frameLiftAllow)
	topBoundBase := absSumUpper(pp.z1Delta, frameLiftAllow)

	bStart := &Vertex{position: prismView.point(w.startU, w.startV, pp.z0), bound: units.Millimeters(bottomBoundBase)}
	bEnd := &Vertex{position: prismView.point(w.endU, w.endV, pp.z0), bound: units.Millimeters(bottomBoundBase)}
	tStart := &Vertex{position: prismView.point(w.startU, w.startV, pp.z1), bound: units.Millimeters(topBoundBase)}
	tEnd := &Vertex{position: prismView.point(w.endU, w.endV, pp.z1), bound: units.Millimeters(topBoundBase)}

	bottomEdge := &Edge{curve: Line3{}, start: bStart, end: bEnd, convex: true, length: w.length, lengthBound: w.lengthBound}
	topEdge := &Edge{curve: Line3{}, start: tStart, end: tEnd, convex: true, length: w.length, lengthBound: w.lengthBound}
	// startSweep and endSweep are Table G's "one sweep edge at each of its two
	// free ends" — the chain's own counterpart of buildLoopSidesAs's junction
	// verticals, except that a free end joins no neighbouring wall.
	startSweep := &Edge{curve: Line3{}, start: bStart, end: tStart, length: pp.z1 - pp.z0, lengthBound: height.bound}
	endSweep := &Edge{curve: Line3{}, start: bEnd, end: tEnd, length: pp.z1 - pp.z0, lengthBound: height.bound}

	// T × N: the walk tangent crossed with the plane normal, the identical
	// construction buildLoopSidesAs's own straight-wall branch takes for a
	// profile-fed wall's outward normal (docs/surface-design.md §13.4). A
	// reflected placement flips the cross product's handedness, so the
	// tangent is negated to keep it outward, exactly as the profile-fed
	// branch does.
	mid := prismView.point((w.startU+w.endU)/2, (w.startV+w.endV)/2, pp.z0)
	tu, tv := w.tanInU, w.tanInV
	if prismView.reflected() {
		tu, tv = -tu, -tv
	}
	wallFrame, err := r3.NewFrame(mid, prismView.dir(tu, tv, 0), prismView.dir(0, 0, 1))
	if err != nil {
		return nil, fmt.Errorf(`%w: the chain's own segment has no direction`, ErrDegenerate)
	}

	wallArea := boundedMul(measuredScalar(w.length, w.lengthBound), height)
	face := &Face{
		surface: Plane{Frame: wallFrame},
		// The role name matches sideOriginsContext's own "side(loop,segment)"
		// convention (prism_build.go): this increment's single wall is
		// loop 0, segment 0.
		origins:   []FeatureRef{{producer: ref, Role: fmt.Sprintf("side(%d,%d)", 0, 0)}},
		body:      body,
		area:      wallArea.value,
		areaBound: wallArea.bound,
		loops: []*Loop{{outer: true, coedges: []coedge{
			{edge: bottomEdge, forward: true},
			{edge: endSweep, forward: true},
			{edge: topEdge, forward: false},
			{edge: startSweep, forward: false},
		}}},
	}
	for _, e := range []*Edge{bottomEdge, topEdge, startSweep, endSweep} {
		e.faces = append(e.faces, face)
	}
	faces := []*Face{face}
	// sheetLumps derives IsOpen from the faces' own edge adjacency: every one
	// of this ribbon's four edges carries exactly one face, so its one lump's
	// one shell reads open, exactly as a profile-fed wall's own free rim does
	// (surface.go).
	body.lumps = sheetLumps(faces)

	// The ribbon's area is the sum of its per-wall areas, composed through
	// boundedAdd rather than added as raw floats (docs/surface-design.md
	// §13.4): one term this increment, and the same fold a later increment's
	// multi-segment wall set extends with no change here.
	total := boundedAdd(boundedScalar{}, wallArea)
	body.area = Measurement{
		Value:     units.SquareMillimeters(total.value),
		Exactness: exactnessOf(total.bound),
		Bound:     units.SquareMillimeters(total.bound),
	}
	// volume and centroid stay at their zero value: finite, so
	// validateAnalyticBodyMeasurements below passes, and neither is
	// reachable through Body.Volume/Body.Centroid while solid is false
	// (docs/surface-design.md §8), exactly as patch.go's evalPatchContext
	// leaves them.
	work := newFreeformWork()
	bounds, err := prismBoundsContext(ctx, prismView, work, nil)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = pp
	return body, nil
}
