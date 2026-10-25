package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file is agent-docs/helix-design.md's entry point: the CoilOption surface,
// Table CS's gates in §4's order, the coil payload that records a coil and
// its placement. internal/coilshell builds §5's held shell, Table CB's
// topology and Table CM's readings.

// maxCoilFacets is Table CS row CS8's facet ceiling over the wall and cap
// triangles together. Every triangle enters the crossing audit's exact pair
// classification and every station a 200-bit trig enclosure, so the
// ceiling bounds the build's time and memory before anything is allocated.
const maxCoilFacets = 1 << 20

// CoilOption configures Coil. Sealed: WithLeftHand is the one option.
// WithSurfaceResult does not implement it, so a coil is always a solid
// (Table CS row CS11).
type CoilOption interface {
	coilOption()
}

type leftHandOption struct{}

func (leftHandOption) coilOption() {}

// WithLeftHand turns the section left-handed about the axis direction while
// it advances along it. The default is right-handed. Passing it twice is
// ErrDegenerate.
func WithLeftHand() CoilOption {
	return leftHandOption{}
}

func decodeCoilOptions(opts []CoilOption) (bool, error) {
	leftHand := false
	for _, raw := range opts {
		if raw == nil {
			return false, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if _, ok := raw.(leftHandOption); !ok {
			return false, fmt.Errorf(`%w: the coil option is not a decad coil option (%T)`, ErrDegenerate, raw)
		}
		if leftHand {
			return false, fmt.Errorf(`%w: WithLeftHand was passed more than once`, ErrDegenerate)
		}
		leftHand = true
	}
	return leftHand, nil
}

// Coil moves the closed profile p of sketch s along the screw motion about
// axis (agent-docs/helix-design.md): turns full rotations about the axis,
// right-handed about the axis direction, advancing pitch along that
// direction per rotation. The section stays in the axis plane throughout, so
// a thread profile drawn beside the axis cuts the thread it draws. The axis
// is Revolve's own vocabulary — SketchLine, ConstructionAxis or EdgeAxis —
// and MUST lie in the sketch plane.
//
// pitch MUST be a finite positive length and turns a finite positive
// dimensionless count, whole or fractional (ErrUnitKind for the wrong kind,
// ErrNotFinite for a non-finite value, ErrDegenerate at or below zero). The
// profile MUST lie strictly on one side of the axis: a profile proven to
// touch or cross it is ErrDegenerate, and one whose side the axis's own
// rounding leaves undecided is ErrUnsupported. With one turn or more, the
// profile's extent along the axis MUST stay below the pitch, or two turns
// would meet (ErrUnsupported). Every profile segment MUST be a whole LineSeg,
// ArcSeg or CircleSeg (ErrUnsupported); an arc is held as 16 chords per turn
// of its sweep. A coil needing more than 32768 stations at 256 per turn, or
// more than 1048576 triangles, is ErrUnsupported, as is one whose held shell
// crosses itself where two turns pass closer than the station chords
// resolve, or whose crossing audit passes its triangle or pair ceiling.
//
// The result is a solid: two planar caps, one faceted wall per profile
// segment spanning every turn, read through Table CM's closed forms and, on
// an arc, agent-docs/helix-design.md §11.1's area bracket. A failed call leaves
// the document untouched.
func Coil(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile, axis Axis, pitch, turns units.Value, opts ...CoilOption) (*Body, error) {
	// CS1: nils, owned options and option arity.
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a coil`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || p == nil {
		return nil, fmt.Errorf(`%w: Coil requires a non-nil sketch and profile`, ErrDegenerate)
	}
	axis, err := normalizeAxis(axis)
	if err != nil {
		return nil, err
	}
	leftHand, err := decodeCoilOptions(opts)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// CS2: the profile seam, with the live-profile area falsifier every
	// profile-fed build runs.
	profile, plane, profileArea, err := sectioncert.RecordProfile(s, p)
	if err != nil {
		return nil, err
	}
	work := freeform.NewFreeformWork()
	if _, err := profile.FalsifyRecordedArea(profileArea, work); err != nil {
		return nil, err
	}

	// CS4: pitch and turns.
	pitchMM, err := coil.Magnitude(pitch, units.Length, units.Millimeter, "the coil pitch")
	if err != nil {
		return nil, err
	}
	turnCount, err := coil.Magnitude(turns, units.Dimensionless, units.One, "the coil turn count")
	if err != nil {
		return nil, err
	}

	// CS3: the axis, resolved into the plane exactly as Revolve resolves it.
	evalAxis := axis
	if ea, ok := axis.(EdgeAxis); ok {
		line, err := d.resolveEdgeAxis(ea)
		if err != nil {
			return nil, err
		}
		evalAxis = line
	}
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	line, err := axisInPlane(evalAxis, frame)
	if err != nil {
		return nil, err
	}

	// CS5 and CS6.
	side, err := coil.ProfileSide(ctx, profile, line, pitchMM, turnCount, work)
	if err != nil {
		return nil, err
	}

	// CS7 and CS8.
	prof, err := coil.Loops(profile.Outer, profile.Holes, coilshell.ArcChordsPerTurn)
	if err != nil {
		return nil, err
	}
	if err := coil.Preflight(turnCount, len(prof.Pts), len(prof.LoopIdx)-1, coilshell.StationsPerTurn, coilshell.MaxStations, maxCoilFacets); err != nil {
		return nil, err
	}

	body, err := bodyResult(coilshell.BuildBody(ctx, envOf(documentNode(d), d.nextProducerID()), coilPayload{
		Profile:  profile,
		Frame:    frame,
		Line:     line,
		Side:     side,
		Pitch:    pitchMM,
		Turns:    turnCount,
		LeftHand: leftHand,
		Xform:    r3.Identity(),
	}))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := d.commitOperationIn(ctx, d.ContainerOfSketch(s), OperationCoil, axisOperationInputs(axis), body); err != nil {
		return nil, err
	}
	return body, nil
}
