package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the extrude of agent-docs/evaluator-design.md §5: the feature call
// gates its live inputs, converts the profile to structural records, evaluates
// from those records, and commits atomically. The body-relative stops
// (ThroughAll/ThroughAllSide/ToFace) resolve through stops.go: the stop
// bodies are resolved at the call and tracked by private producer identities.
//
// The evaluation this call drives is spread over three sibling files, each
// with its own doc comment: internal/prismrecord holds the record and the
// coordinate readings taken off it, internal/prismrecord/build.go builds the body, and
// internal/prismrecord/extent.go answers the extent questions asked of the result. The
// prism they produce is analytic — Plane and Cylinder faces — for every
// line/circle/arc boundary segment, and a NURBSSurface
// (agent-docs/spline-design.md §7) for a Tier A free-form one, with bounded mass
// measurements throughout. Exactly representable results retain zero bounds.
// A nonzero taper leaves this file for draft.go's extrudeDraft.

// ExtrudeOption configures Extrude. A chain section accepts none of them: both
// WithTaper and WithSurfaceResult return ErrUnsupported on a chain.
type ExtrudeOption interface {
	option.Interface
	extrudeOption()
}

// WithTaper sets the extrude taper: a SIGNED displacement angle — which way
// the wall leans. A positive taper leans every wall into the material as it
// leaves the sketch plane, so the body narrows toward the far end; a negative
// one widens it (agent-docs/draft-design.md §2). An angle at or past a right angle
// is [ErrDegenerate]. A nonzero taper builds over Distance, ToFace,
// ThroughAll, Symmetric or TwoSided. WithSurfaceResult is [ErrUnsupported],
// returned before commit. A zero taper builds the straight prism. A chain
// section returns [ErrUnsupported] for WithTaper at any angle, zero included.
func WithTaper(a units.Value) ExtrudeOption {
	return extrudeOptionValue{featureoption.WithTaper(a)}
}

// Extrude sweeps the section sec of sketch s along the sketch plane's normal
// per the linear extent e, registers the new body, and returns it. sec is a
// closed [*sketch.Profile], which extrudes to a solid (sheet under
// [WithSurfaceResult]), or an open [*sketch.Chain], which extrudes to a
// ribbon sheet. Both are recorded from s; the type argument S is inferred.
//
// A profile section MUST be a profile of s (ErrForeignProfile) and a
// current, unaltered snapshot (ErrStaleProfile or ErrInvalidProfile); an
// invalid profile is also ErrInvalidProfile, and a boundary decad cannot
// record exactly is ErrUnrecordableProfile (core §7). This evaluator builds a
// bounded oblique Body.Section profile only as a solid straight prism with an
// independent Distance extent, carrying its certified source displacement.
// Its private Sketch certificate and marked reference loop must still match.
// Other bounded-section consumers and options return [ErrUnsupported].
// Otherwise this evaluator builds a
// straight prism from a profile of line, circle, arc and Tier A free-form
// segments (a spline, a closed spline, a fit spline, or a unit-weight NURBS
// curve — agent-docs/spline-design.md Table F); a Tier B or Tier C free-form
// segment (a conic, a whole ellipse, or a NURBS curve with unequal weights) is
// [ErrUnsupported]. A Tier A kind is admitted but not thereby built: each
// free-form wall edge must also prove ONE curvature sign across every span and
// joint of its chain (agent-docs/spline-design.md §6.5), and the whole profile's
// free-form work must fit the fixed budget. A chain whose curvature genuinely
// changes sign, or whose certificate the fixed subdivision depth does not
// close, is [ErrUnsupported] (Table R row R19), as is a profile past the
// budget (row R7). A free-form curve must meet its neighbours at shared
// endpoints unless sketch certifies a fit-spline/circle trim
// (agent-docs/spline-design.md §2.1). Other crossing fragments are rejected as
// ErrUnrecordableProfile before this runs. WithSurfaceResult() omits the two
// caps and publishes a sheet body instead of a solid (agent-docs/surface-design.md
// §4).
//
// A nonzero [WithTaper] on a profile builds a draft body instead
// (agent-docs/draft-design.md): the far cap is the profile offset by h·tan α with
// every straight corner mitered, each straight wall a Plane leaning by the
// taper and each circular wall a Cone. Its profile must hold line, arc and
// circle segments only, and a circular wall must meet each neighbour
// tangentially; anything else, and every other row of that design's Table SD,
// is [ErrUnsupported] or [ErrDegenerate] as the row states. Every measurement
// of a draft body is Approximate, since the taper's tangent is only ever
// enclosed.
//
// A chain section MUST be a chain of s and a current, unaltered snapshot,
// with the same sentinels (ErrForeignProfile, ErrStaleProfile,
// ErrInvalidProfile, ErrUnrecordableProfile; an invalid or self-intersecting
// chain is also ErrInvalidProfile; agent-docs/sketch-seam-design.md §2.2). The
// result is always a sheet — Kind() == BodySheet — one wall face per recorded
// segment (agent-docs/surface-design.md §13.4, Table G), with no cap and no closing
// face. A chain of any segment count and kind builds — a line wall Exact, an
// arc or circle fragment wall carrying rθ's own bound, a Tier A free-form
// wall carrying internal/freeform/spline_length.go's proven bracket
// (agent-docs/spline-design.md); a Tier B or Tier C free-form segment is
// ErrUnsupported, exactly as a profile's own wall refuses it. A chain section
// returns ErrUnsupported for [WithSurfaceResult], which has no solid to
// toggle, and for [WithTaper] of any angle, zero included, since a chain
// extrude has no draft build. Both refusals follow the chain's seam gate and
// precede extent resolution (agent-docs/surface-design.md §13.2).
//
// A nil context, document, sketch or section returns ErrDegenerate. The
// context is checked at entry, between recording the section, resolving the
// extent and building the body, and inside the evaluators; a canceled context
// returns ctx.Err() unchanged. Every failure and cancellation leaves the
// document unchanged.
func Extrude[S Section](ctx context.Context, d *Document, s *sketch.Sketch, sec S, e Extent,
	opts ...ExtrudeOption) (*Body, error) {
	switch v := any(sec).(type) {
	case *sketch.Profile:
		return extrudeProfile(ctx, d, s, v, e, opts)
	case *sketch.Chain:
		return extrudeChain(ctx, d, s, v, e, opts)
	default:
		return nil, fmt.Errorf(`%w: Extrude section %T is not a profile or chain`, ErrDegenerate, sec)
	}
}

// extrudeProfile is the profile-fed Extrude body (agent-docs/evaluator-design.md
// §5). Its refusals run in this order: the record, the recorded-area
// falsifier, the options, the taper checks, then the extent.
func extrudeProfile(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile, e Extent,
	opts []ExtrudeOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an extrude`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || p == nil {
		return nil, fmt.Errorf(`%w: Extrude requires a non-nil sketch and profile`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	profile, plane, profileArea, certificate, err := sectioncert.RecordExtrudeProfile(s, p)
	if err != nil {
		return nil, err
	}
	// ONE free-form work counter for this whole operation over this record: the
	// area falsifier's preflight opens it, the prism build's own preflight
	// continues it, and every walkOf under that build spends what is left
	// (agent-docs/spline-design.md §5.2). A counter per phase would give the same
	// record a fresh full ceiling in each.
	work := freeform.NewFreeformWork()
	// A fit-spline section costs about 70000 units per cubic piece, so the
	// ceiling scales with the piece count, as a loft's does with its stations.
	loops := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	work.RaiseLimit(freeform.SpanWorkLimit(sectionrecord.FitSplinePieces(loops...)))
	if _, err := profile.FalsifyRecordedAreaContext(ctx, profileArea, work); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cfg, err := decodeExtrudeOptions(opts)
	if err != nil {
		return nil, err
	}
	taper, surfaceResult := cfg.Taper, cfg.SurfaceResult
	var alpha, alphaDelta float64
	if taper.Mag() != 0 {
		if alpha, alphaDelta, err = draftAngle(taper); err != nil {
			return nil, err
		}
	}
	var frame r3.Frame
	if certificate != nil {
		// The authenticated Sketch plane has this exact held frame. Rebuilding
		// it may change an oblique basis by one bit and move the source plane.
		frame = certificate.Frame
	} else {
		frame, err = r3.NewFrame(plane.Origin, plane.U, plane.V)
		if err != nil {
			return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
		}
	}
	if certificate != nil {
		if taper.Mag() != 0 || surfaceResult {
			return nil, fmt.Errorf(`%w: bounded Section needs its original frame and a solid straight Extrude`, ErrUnsupported)
		}
		if _, ok := e.(Distance); !ok {
			return nil, fmt.Errorf(`%w: bounded Section needs an independent Distance extent`, ErrUnsupported)
		}
	}

	e, err = normalizeExtent(e)
	if err != nil {
		return nil, err
	}
	if taper.Mag() != 0 {
		// A nonzero taper builds a draft body (agent-docs/draft-design.md), never a
		// straight prism.
		return d.extrudeDraft(ctx, profile, frame, e, alpha, alphaDelta, surfaceResult)
	}
	sweep, err := d.resolveLinearExtent(e, frame)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	ref := d.nextProducerID()
	var sectionDelta float64
	if certificate != nil {
		sectionDelta = certificate.Delta
	}
	body, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile:       profile,
		Frame:         frame,
		SectionDelta:  sectionDelta,
		Z0:            sweep.z0,
		Z1:            sweep.z1,
		Z0Delta:       sweep.z0Delta,
		Z1Delta:       sweep.z1Delta,
		Xform:         r3.Identity(),
		SurfaceResult: surfaceResult,
	}, work))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
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
// (agent-docs/evaluator-design.md §5). The refs are ordered named-extent refs in
// extent order first, through-all stop bodies after them in stop order along
// the sweep, deduplicated. Magnitudes are validated per core
// §8.1/§12; a zero-thickness sweep is ErrDegenerate.
func (d *Document) resolveLinearExtent(e Extent, frame r3.Frame) (linearSweep, error) {
	switch e := e.(type) {
	case Distance:
		m, delta, err := extent.MagnitudeInBounded(e.D, units.Length, units.Millimeter, "the extent distance")
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
		m, delta, err := extent.MagnitudeInBounded(e.D, units.Length, units.Millimeter, "the symmetric distance")
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
		refs := provenance.DedupProducers(append(append(named, one.through...), two.through...))
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
		stop, delta, refs, err := extent.ThroughAll(frame, travel, d.n.Live())
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
		m, delta, err := extent.MagnitudeInBounded(s.D, units.Length, units.Millimeter, what)
		if err != nil {
			return linearSide{}, err
		}
		// travel is ±1, so the signed level is the magnitude itself and its
		// displacement carries across unchanged.
		return linearSide{z: travel * m, delta: delta}, nil
	case ThroughAllSide:
		stop, delta, refs, err := extent.ThroughAll(frame, travel, d.n.Live())
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

// This section is the chain-fed Extrude of agent-docs/surface-design.md §13: the open sketch
// chain's own sweep into a ribbon. It reuses resolveLinearExtent unchanged —
// the extent vocabulary and the frame it resolves against are identical to
// Extrude's — and reads its recorded walk through recordChain (seam.go)
// rather than sectioncert.RecordProfile. It builds a chain of any segment count and kind
// Table G's per-kind construction admits, through chainrecord.Build
// (internal/chainrecord/build.go, which calls prismrecord.BuildWallGeometry, shared with the profile-fed prism
// build) — the same evaluator, over an open rather than a closed walk.

// extrudeChain is the chain-fed Extrude body (agent-docs/surface-design.md §13).
// Its refusals run in this order: the nil-option check, the seam gates, the
// option refusal, then the extent.
func extrudeChain(ctx context.Context, d *Document, s *sketch.Sketch, ch *sketch.Chain, e Extent,
	opts []ExtrudeOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an extrude`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || ch == nil {
		return nil, fmt.Errorf(`%w: Extrude requires a non-nil sketch and chain`, ErrDegenerate)
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	chain, plane, err := recordChain(s, ch)
	if err != nil {
		return nil, err
	}
	// The option refusal follows the seam gate and precedes the extent work,
	// so a repair the caller makes in the sketch is named first.
	if err := refuseChainExtrudeOptions(opts); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
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
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// ONE free-form work counter for this whole call, exactly as the profile
	// form opens for its own build (agent-docs/spline-design.md §5.2):
	// buildSides' own walkOf calls and the final bounds reading below
	// spend from the same ceiling rather than each opening a fresh one.
	work := freeform.NewFreeformWork()
	ref := d.nextProducerID()
	body, err := bodyResult(chainrecord.Build(ctx, envOf(documentNode(d), ref), chainPayload{
		Chains:  []chainRecord{chain},
		Frame:   frame,
		Z0:      sweep.z0,
		Z1:      sweep.z1,
		Z0Delta: sweep.z0Delta,
		Z1Delta: sweep.z1Delta,
		Xform:   r3.Identity(),
	}, work))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// refuseChainExtrudeOptions refuses every option a chain section cannot
// honour with ErrUnsupported. A chain extrude always returns a sheet, so
// WithSurfaceResult has no solid to toggle, and it has no draft build, so
// WithTaper is refused at any angle, zero included. opts holds no nil element.
func refuseChainExtrudeOptions(opts []ExtrudeOption) error {
	for _, o := range opts {
		switch o.(type) {
		case surfaceResultOptionValue:
			return fmt.Errorf(`%w: a chain extrude always returns a sheet; WithSurfaceResult has no solid to toggle`,
				ErrUnsupported)
		case extrudeOptionValue:
			return fmt.Errorf(`%w: a chain extrude has no draft build; WithTaper builds a solid from a profile only`,
				ErrUnsupported)
		default:
			return fmt.Errorf(`%w: the extrude option %T builds a solid from a profile only`, ErrUnsupported, o)
		}
	}
	return nil
}
