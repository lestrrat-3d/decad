package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/opcommand"
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

// coilConfig is the folded Coil input set. Thread, which derives a coil,
// fills it with a station density and whole lines the public Coil keeps at
// its documented fixed density.
type coilConfig struct {
	leftHand        bool
	stationsPerTurn int64
	wholeLines      bool
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
// of its sweep. A line is held as one chord or, where one chord's wall cells
// would depart from the true wall by more than 0.15% of the line's outer
// radius (a long radial run on a steep screw), as up to 256 chords graded by
// radius, each holding within 0.002 mm where the station step allows
// (agent-docs/helix-design.md §5.3). A coil needing more than
// 32768 stations at 256 per turn, or
// more than 1048576 triangles, is ErrUnsupported, as is one whose held shell
// crosses itself where two turns pass closer than the station chords
// resolve, or whose crossing audit passes its triangle or pair ceiling.
//
// The result is a solid: two planar caps, one faceted wall per profile
// segment spanning every turn, read through Table CM's closed forms and, on
// an arc, agent-docs/helix-design.md §11.1's area bracket. A failed call leaves
// the document untouched.
func Coil(d *Document, s *sketch.Sketch, p *sketch.Profile, axis Axis, pitch, turns units.Value) *CoilCommand {
	c := &CoilCommand{d: d, s: s, p: p, axis: copyAxis(axis), pitch: pitch, turns: turns}
	recordNilFaults(&c.state, "Coil", d, sketchArg(s), profileArg(p))
	return c
}

// CoilCommand moves a profile along a screw motion. Build it with [Coil].
type CoilCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	d     *Document       // stored as given
	s     *sketch.Sketch  // stored as given
	p     *sketch.Profile // stored as given
	axis  Axis            // copied at Coil, with the selector of an EdgeAxis
	pitch units.Value     // gated at Do
	turns units.Value     // gated at Do
	left  bool            // set by LeftHand
}

// LeftHand turns the section left-handed about the axis direction while it
// advances along it. The default is right-handed. Calling it twice is the same
// as calling it once. A coil is always a solid: it has no SurfaceResult (Table
// CS row CS11).
func (c *CoilCommand) LeftHand() *CoilCommand { c.left = true; return c }

// Do moves the profile, registers the new body and returns it, or returns the
// error of the first gate that refuses, leaving the document unchanged. A nil
// ctx is ErrDegenerate, and a second call returns ErrConsumed.
func (c *CoilCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationCoil, c.run)
}

func (c *CoilCommand) run(ctx context.Context) (*Body, error) {
	cfg := coilConfig{leftHand: c.left, stationsPerTurn: int64(coilshell.StationsPerTurn)}
	return coilBody(ctx, c.d, c.s, c.p, c.axis, c.pitch, c.turns, cfg)
}

// coilBody is Coil's implementation. ctx is not nil, and the errors it returns
// carry no operation tag (agent-docs/command-api-design.md §11).
func coilBody(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile, axis Axis,
	pitch, turns units.Value, cfg coilConfig) (*Body, error) {
	// CS1: nils and the axis.
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
	if cfg.stationsPerTurn <= 0 {
		return nil, fmt.Errorf(`%w: invalid coil station density`, ErrDegenerate)
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
	prof, err := coilshell.Loops(profile, coilshell.AxisInput{
		AU: line.AU, AV: line.AV, AUBound: line.AUBound, AVBound: line.AVBound,
		DU: line.DU, DV: line.DV, DUBound: line.DUBound, DVBound: line.DVBound,
		Side: side,
	}, pitchMM, turnCount, cfg.stationsPerTurn, coilshell.ArcChordsPerTurn, cfg.wholeLines)
	if err != nil {
		return nil, err
	}
	if err := coil.Preflight(turnCount, len(prof.Pts), len(prof.LoopIdx)-1, cfg.stationsPerTurn, coilshell.MaxStations, maxCoilFacets); err != nil {
		return nil, err
	}

	body, err := bodyResult(coilshell.BuildBody(ctx, envOf(documentNode(d), d.producerAt(ctx)), coilPayload{
		Profile:         profile,
		Frame:           frame,
		Line:            line,
		Side:            side,
		Pitch:           pitchMM,
		Turns:           turnCount,
		LeftHand:        cfg.leftHand,
		StationsPerTurn: cfg.stationsPerTurn,
		WholeLines:      cfg.wholeLines,
		Xform:           r3.Identity(),
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
