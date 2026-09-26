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
// rather than RecordProfile. It builds a chain of any segment count and kind
// Table G's per-kind construction admits, through buildChainSides
// (prism_build.go's buildWallGeometry, shared with the profile-fed prism
// build) — the same evaluator, over an open rather than a closed walk.

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
// walk SET, the plane frame it lifts through, the signed sweep interval, and
// the accumulated rigid placement — chainPayload is to ExtrudeChain what
// prismPayload is to Extrude (docs/surface-design.md §13.4). A plain
// ExtrudeChain builds the one-walk case; docs/surface-intersection-design.md
// §3.4 is what first builds more than one, one lump per surviving walk, and
// sets sectionDelta beside them.
type chainPayload struct {
	chains  []ChainRecord
	frame   r3.Frame
	z0, z1  float64
	z0Delta float64
	z1Delta float64
	xform   r3.Transform
	// sectionDelta is prismPayload's own §7 term (docs/prism-boolean-design.md),
	// carried here on the identical terms: the proven upper bound on how far
	// any recorded boundary coordinate sits from the section its construction
	// denotes. Zero for every walk a caller draws through ExtrudeChain.
	sectionDelta float64
}

func (pp chainPayload) z0Scalar() boundedScalar { return measuredScalar(pp.z0, pp.z0Delta) }
func (pp chainPayload) z1Scalar() boundedScalar { return measuredScalar(pp.z1, pp.z1Delta) }

// prism is a *view* of pp as a zero-section-delta prismPayload — never a body
// this evaluator builds — used only to feed pp.point/pp.dir/reflected and
// prismBoundsContext, none of which cares whether the section it reads
// closes. sectionDelta stays zero in THIS view deliberately: prismPayload's
// own Bounds reading (prismBoundsContext) charges a nonzero sectionDelta as a
// BLANKET term, δ outward on every face regardless of which candidate wins
// each extreme, which is sound for an ordinary prismPayload but would widen a
// trimmed ribbon's UNTOUCHED extremes too — exactly the ones T170 requires to
// stay bit-identical to the untrimmed sheet's own. The tight, per-extreme
// charge this design actually needs is carried instead by the WALKS
// evalChainExtrudeContext resolves through trimBoundsWalks
// (surface_trim.go) when pp.sectionDelta != 0, never by this view's own
// field. The first chain stands in for a profile's outer loop, the rest for
// its holes — extentBoundedAlong reads every one the same way, caring only
// about the segments, never about winding or closure.
func (pp chainPayload) prism() prismPayload {
	profile := ProfileRecord{Outer: LoopRecord(pp.chains[0])}
	for _, c := range pp.chains[1:] {
		profile.Holes = append(profile.Holes, LoopRecord(c))
	}
	return prismPayload{
		profile: profile,
		frame:   pp.frame,
		z0:      pp.z0, z1: pp.z1,
		z0Delta: pp.z0Delta, z1Delta: pp.z1Delta,
		xform: pp.xform,
	}
}

// transform is the accumulated rigid placement.
func (pp chainPayload) transform() r3.Transform { return pp.xform }

// placed re-evaluates the same record under the composed motion (core §8). A
// re-evaluation path: no moments preflight has run on this record within the
// call, so the build opens the record's one free-form work counter itself
// (docs/spline-design.md §5.2), exactly as prismPayload.placed does.
func (pp chainPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	pp.xform = composed
	return evalChainExtrudeContext(ctx, d, ref, pp, newFreeformWork())
}

// ExtrudeChain sweeps the open chain ch of sketch s along the sketch plane's
// normal per the linear extent e, and registers the new ribbon body. ch MUST
// be a chain of s (ErrForeignProfile) and a current, unaltered snapshot
// (ErrStaleProfile or ErrInvalidProfile); an invalid or self-intersecting
// chain is also ErrInvalidProfile, and a walk decad cannot record exactly is
// ErrUnrecordableProfile (docs/sketch-seam-design.md §2.2). The result is
// always a sheet — Kind() == BodySheet — one wall face per recorded segment
// (docs/surface-design.md §13.4, Table G), with no cap and no closing face:
// WithSurfaceResult() does not compile against this call. A chain of any
// segment count and kind builds — a line wall Exact, an arc or circle
// fragment wall carrying rθ's own bound, a Tier A free-form wall carrying
// spline_length.go's proven bracket (docs/spline-design.md); a Tier B or
// Tier C free-form segment is ErrUnsupported, exactly as Extrude's own
// profile-fed wall refuses it. A failed evaluation leaves the document
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

	// ONE free-form work counter for this whole call, exactly as Extrude
	// opens for its own profile-fed build (docs/spline-design.md §5.2):
	// buildChainSides's own walkOf calls and the final bounds reading below
	// spend from the same ceiling rather than each opening a fresh one.
	work := newFreeformWork()
	ref := d.nextProducerID()
	body, err := evalChainExtrudeContext(context.Background(), d, ref, chainPayload{
		chains:  []ChainRecord{chain},
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
	d.commit(body)
	return body, nil
}

// evalChainExtrudeContext builds the ribbon body: one wall face per recorded
// segment of every walk in pp.chains, both rims of each walk and the one
// sweep edge at each of its two free ends (docs/surface-design.md §13.4,
// Table G row 1), and the measurements the finished body publishes. It
// mirrors evalPrismContext's own order (prism_build.go) without ever
// building a solid or a closing cap: an open chain mints neither. Each walk
// is its own lump — surface §2.2's one-lump-per-connected-boundary rule,
// which docs/surface-intersection-design.md §3.4 is the first construction
// to exercise with more than one. work is the record's ONE free-form work
// counter (docs/spline-design.md §5.2): the caller opens it once and every
// walk's build and the final bounds reading all spend from it.
func evalChainExtrudeContext(ctx context.Context, d *Document, ref producerID, pp chainPayload, work *freeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(pp.chains) == 0 {
		return nil, fmt.Errorf(`%w: a ribbon payload holds no walk`, ErrDegenerate)
	}
	height := boundedSub(pp.z1Scalar(), pp.z0Scalar())
	if height.value <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: false, kind: BodySheet}
	var allFaces []*Face
	total := boundedScalar{}
	var captures []chainWalkCapture
	if pp.sectionDelta == 0 {
		captures = make([]chainWalkCapture, len(pp.chains))
	}
	for ci, chain := range pp.chains {
		var capture *chainWalkCapture
		if captures != nil {
			capture = &captures[ci]
		}
		faces, area, err := buildChainSides(ctx, body, ref, pp, ci, chain, work, capture)
		if err != nil {
			return nil, err
		}
		allFaces = append(allFaces, faces...)
		total = boundedAdd(total, area)
	}
	body.lumps = sheetLumps(allFaces)

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
	//
	// A ribbon this design's Trim assembled (pp.sectionDelta != 0 — the only
	// construction that ever sets it, docs/surface-intersection-design.md
	// §3.4) reads its extent through walks that charge §7's δ_cut into
	// exactly the endpoint a cut produced, never into one the record states
	// verbatim (trimBoundsWalks, surface_trim.go). A plain ExtrudeChain
	// ribbon has zero sectionDelta and reuses the walks resolved for its sides.
	view := pp.prism()
	var boundsWalks *profileWalks
	if pp.sectionDelta != 0 {
		var err error
		boundsWalks, err = trimBoundsWalks(view.profile, work)
		if err != nil {
			return nil, err
		}
		if err := boundsWalks.charge(work); err != nil {
			return nil, err
		}
	} else {
		boundsWalks = chainBoundsWalks(view.profile, captures)
	}
	bounds, err := prismBoundsContext(ctx, view, work, boundsWalks)
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

type chainWalkCapture struct {
	walks   []segmentWalk
	charges []walkReadCharge
}

// chainBoundsWalks reads the exact pre-widening walks buildChainSides already
// resolved. The cache is local to this build; its per-segment measured charges
// are replayed by resolveOrRead at each bounds read.
func chainBoundsWalks(profile ProfileRecord, captures []chainWalkCapture) *profileWalks {
	reads := make([][]walkReadCharge, len(captures))
	walks := &profileWalks{profile: profile, readCharges: reads}
	for i, capture := range captures {
		reads[i] = capture.charges
		if i == 0 {
			walks.outer = capture.walks
		} else {
			walks.holes = append(walks.holes, capture.walks)
		}
	}
	return walks
}

// buildChainSides builds ONE walk's whole wall set with shared vertices and
// edges: one wall face per recorded segment (Table G row 1), never a cap and
// never a wraparound junction. chainIdx names which of pp.chains this is,
// feeding sideOriginsContext's roleLoop exactly as a prism's loop index does,
// so two different walks' faces never collide on one role string. n segments
// place n+1 rim posts at each sweep level, so the walk's two FREE ends (post
// 0 and post n) each carry a sweep edge touched by exactly one wall — Free()
// resolves them (Edge.IsFree, topology.go) — while every INTERIOR post
// (1..n-1) shares its sweep edge between the two walls it joins, exactly as
// buildLoopSidesAs's own junction verticals do for a closed loop
// (docs/surface-design.md §13.4). It returns the faces and the walk's own
// total wall area, folded through boundedAdd rather than summed as raw
// floats.
func buildChainSides(ctx context.Context, body *Body, ref producerID, pp chainPayload, chainIdx int, chain ChainRecord, work *freeformWork, capture *chainWalkCapture) ([]*Face, boundedScalar, error) {
	prismView := pp.prism()
	// Every coordinate this walk's segments read sits within pp's own section
	// displacement of the section it denotes, so each segment's own length
	// carries that displacement too — buildLoopSidesAs's identical charge,
	// prism-boolean §7's 12·π·δ per walk (bounds.go's
	// sectionDisplacementLength), zero for every payload ExtrudeChain builds
	// directly.
	walkLenAllow := sectionDisplacementLength(pp.sectionDelta, 1)
	raw := make([]sideWalk, len(chain.Segments))
	if capture != nil {
		capture.walks = make([]segmentWalk, len(chain.Segments))
		capture.charges = make([]walkReadCharge, len(chain.Segments))
	}
	for i, seg := range chain.Segments {
		if err := ctx.Err(); err != nil {
			return nil, boundedScalar{}, err
		}
		before, beforeRecon := workSpent(work)
		w, err := walkOf(seg, work)
		if err != nil {
			return nil, boundedScalar{}, err
		}
		if capture != nil {
			after, afterRecon := workSpent(work)
			capture.walks[i] = w
			capture.charges[i] = walkReadCharge{after - before, afterRecon - beforeRecon}
		}
		w.lengthBound = absSumUpper(w.lengthBound, walkLenAllow)
		raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
	}
	walks, err := coalesceChainWalksContext(ctx, raw)
	if err != nil {
		return nil, boundedScalar{}, err
	}
	n := len(walks)

	height := boundedSub(pp.z1Scalar(), pp.z0Scalar())
	maxCoordUpper := 0.0
	for _, w := range walks {
		maxCoordUpper = math.Max(maxCoordUpper, w.coordUpper)
	}
	// frameLiftAllow is the one proven bound this ribbon's rim vertices share
	// for the payload's own frame lift and accumulated placement
	// (bounds.go's frameAndPlacementRoundAllow) — exactly zero for an
	// axis-aligned, unplaced payload, which is what keeps a plain
	// ExtrudeChain's rim vertices Exact, mirroring buildLoopSidesAs's own
	// frameLiftAllow.
	frameLiftAllow := frameAndPlacementRoundAllow(pp.frame, pp.xform, math.Max(maxCoordUpper, math.Max(math.Abs(pp.z0), math.Abs(pp.z1))))
	bottomBoundBase := absSumUpper(pp.z0Delta, frameLiftAllow)
	topBoundBase := absSumUpper(pp.z1Delta, frameLiftAllow)

	// Rim posts 0..n, no wraparound: post i sits at walk i's start for
	// i < n, and at the LAST walk's own end for i == n — the chain's two free
	// ends. A post touching a FREE-FORM walk's own end also folds in that
	// walk's own endpoint bound (freeformVertexAllow), exactly as
	// buildLoopSidesAs does for a closed loop's junctions.
	bottomV := make([]*Vertex, n+1)
	topV := make([]*Vertex, n+1)
	for i := 0; i <= n; i++ {
		if err := ctx.Err(); err != nil {
			return nil, boundedScalar{}, err
		}
		var u, v, extra float64
		if i < n {
			u, v = walks[i].startU, walks[i].startV
			extra = freeformVertexAllow(walks[i].segmentWalk, walks[i].startBound)
		} else {
			u, v = walks[n-1].endU, walks[n-1].endV
			extra = freeformVertexAllow(walks[n-1].segmentWalk, walks[n-1].endBound)
		}
		if i > 0 && i < n {
			extra = math.Max(extra, freeformVertexAllow(walks[i-1].segmentWalk, walks[i-1].endBound))
		}
		bottomV[i] = &Vertex{position: prismView.point(u, v, pp.z0), bound: units.Millimeters(absSumUpper(bottomBoundBase, extra))}
		topV[i] = &Vertex{position: prismView.point(u, v, pp.z1), bound: units.Millimeters(absSumUpper(topBoundBase, extra))}
	}

	// Sweep edges at every post, free-end and interior alike. Convexity from
	// the 2D turn, exactly as buildLoopSidesAs's own junction verticals: only
	// an interior post has both a leaving and an entering tangent to cross,
	// so a free end's carries no turn of its own and keeps its zero
	// (concave) default — Table G assigns it no sense.
	vertical := make([]*Edge, n+1)
	for i := 0; i <= n; i++ {
		if err := ctx.Err(); err != nil {
			return nil, boundedScalar{}, err
		}
		convex := false
		if i > 0 && i < n {
			prev, w := walks[i-1], walks[i]
			convex = prev.tanOutU*w.tanInV-prev.tanOutV*w.tanInU > 0
		}
		vertical[i] = &Edge{curve: Line3{}, start: bottomV[i], end: topV[i], convex: convex, length: pp.z1 - pp.z0, lengthBound: height.bound}
	}

	faces := make([]*Face, 0, n)
	total := boundedScalar{}
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, boundedScalar{}, err
		}
		// holeLoop is always false: a chain has no hole, and the whole walk
		// takes the loop-0 (outer) convention (docs/surface-design.md §13.4).
		convex, err := rimConvexity(ctx, w, false, work)
		if err != nil {
			return nil, boundedScalar{}, err
		}
		bottomEdge, topEdge, surf, faceReversed, err := buildWallGeometry(prismView, w, convex, false, bottomV[i], bottomV[i+1], topV[i], topV[i+1])
		if err != nil {
			return nil, boundedScalar{}, err
		}
		origins, err := sideOriginsContext(ctx, ref, chainIdx, w.segs)
		if err != nil {
			return nil, boundedScalar{}, err
		}
		faceArea := boundedMul(measuredScalar(w.length, w.lengthBound), height)
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.value,
			areaBound: faceArea.bound,
			reversed:  faceReversed,
			loops: []*Loop{{outer: true, coedges: []coedge{
				{edge: bottomEdge, forward: true},
				{edge: vertical[i+1], forward: true},
				{edge: topEdge, forward: false},
				{edge: vertical[i], forward: false},
			}}},
		}
		for _, e := range []*Edge{bottomEdge, topEdge, vertical[i], vertical[i+1]} {
			e.faces = append(e.faces, face)
		}
		faces = append(faces, face)
		total = boundedAdd(total, faceArea)
	}
	return faces, total, nil
}
