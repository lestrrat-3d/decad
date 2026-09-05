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
// commits atomically. The body-relative stops
// (ThroughAll/ThroughAllSide/ToFace) resolve through stops.go: the stop
// bodies are resolved at the call and recorded as StepRefs in the step's
// Inputs (core §6.2).
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
