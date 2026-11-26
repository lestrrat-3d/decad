package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/sweepbuild"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file routes a zero-twist line or circular-arc path through the existing
// analytic prism and revolve evaluators. Composite paths are admitted when
// their transported frames and separation certificates close. The distinct
// sweepPayload preserves the operation's downstream staging and replays every
// reduction for placement. internal/twistsweep owns the bounded nonzero-twist case.
//
// SurfaceResult (agent-docs/surface-design.md §4, agent-docs/sweep-design.md §14
// increment 3) reaches every one of Sweep's three build paths, but not through
// one shared mechanism: a one-span straight or arc path sets the flag on its
// reduced prismPayload/revolvePayload and gets the whole sheet behaviour for
// free from internal/prismrecord/build.go and internal/revolveplan/build.go, while a composite path's spans
// stay solid (sweeprecord.SpanPayload never carries the flag) and
// compositesweep.Assemble (internal/compositesweep/assemble.go) consumes sweepPayload's own
// flag to omit the two outer caps after every span has built solid.
//
// An open Sweep mints two section caps. The exact two-arc closed circle of
// §3.3 reduces to a full Revolve. Under the §§3.4–3.5 closed cardinal loop
// gate, any section the arc builders and contact audit prove simple is admitted,
// and its analytic spans sew cyclically. Neither closed build mints section caps.

// Sweep moves the section sec along path, registers the resulting body, and
// returns it. sec is a closed [*sketch.Profile], which sweeps to a solid
// (sheet under SurfaceResult), or an open [*sketch.Chain], which sweeps
// to a sheet. Both are recorded from s; the type argument S is inferred.
//
// A profile section's path must start in the profile plane and its initial
// tangent must be exactly codirectional with the plane's positive normal.
// Composite paths also require tangent joins, exactly representable transported
// frames, and a certified absence of unintended span contact. Section 17 of
// the Sweep design states the bounded nonzero-twist reach. Sections 3.3–3.5
// admit bounded closed-path classes.
//
// A chain section is gated exactly as a chain [Extrude] gates it
// (agent-docs/surface-design.md §13.3); the seam's own sentinel wins before any path
// geometry is read. The path must start in the sketch plane with its initial
// tangent codirectional with that plane's positive normal (ErrDegenerate). The
// evaluator sweeps one straight span, one arc, or §15.8's cardinal line/arc pair;
// other composite paths are ErrUnsupported (Table SC). The result
// is always a sheet, one wall per recorded segment per span with no cap or
// closing face. A chain section therefore returns ErrUnsupported for
// SurfaceResult (there is no solid to toggle), [Twist],
// [MitredJoins] and [SectionScale]. That refusal follows the chain's
// seam gate and precedes every path-geometry gate (agent-docs/sweep-design.md §15.3).
//
// A nil context, document, sketch, section or path returns ErrDegenerate.
// Every failure and cancellation
// leaves the document unchanged.
func Sweep[S Section](d *Document, s *sketch.Sketch, sec S, path *Path) *SweepCommand {
	c := &SweepCommand{d: d, s: s, sec: any(sec), path: path}
	recordNilFaults(&c.state, "Sweep", d, sketchArg(s), sectionArg("sec", any(sec)),
		namedArg{"path", "path", path == nil})
	return c
}

// SweepCommand moves a section along a path. Build it with [Sweep].
type SweepCommand struct {
	state   opcommand.State                   // consumed flag and first recorded fault
	d       *Document                         // stored as given
	s       *sketch.Sketch                    // stored as given
	sec     any                               // the *sketch.Profile or *sketch.Chain given
	path    *Path                             // stored as given; a Path is immutable
	surface surfaceResultSetting              // set by SurfaceResult
	mitred  bool                              // set by MitredJoins
	scale   opcommand.Slot[sweepSectionScale] // set by SectionScale
	twist   opcommand.Slot[units.Value]       // set by Twist
}

// sweepSectionScale is the copied factors of a SectionScale call.
type sweepSectionScale struct{ factors []float64 }

// Twist rotates the profile about the path tangent as it travels.
// A nonzero angle requires an origin XY sketch and a strictly convex whole-line
// profile centred on the axis. Section 17 admits a positive-Z straight path
// and a bounded line followed by a cardinal XZ quarter arc. Other cases
// return ErrUnsupported without changing the document. A second call replaces
// the first.
func (c *SweepCommand) Twist(angle units.Value) *SweepCommand {
	c.twist.Set(angle, featureoption.ValidateTwist(angle))
	return c
}

// SurfaceResult builds the wall set and omits the two section caps,
// publishing a sheet body instead of a solid (agent-docs/surface-design.md §4).
// A chain section returns [ErrUnsupported] for it, and so does a mitred or
// scaled sweep, which builds a solid only. Calling it twice is the same as
// calling it once.
func (c *SweepCommand) SurfaceResult() *SweepCommand { c.surface.set(); return c }

// MitredJoins admits a LineTo-only path whose internal joins are corners:
// the two spans meeting at a join are each cut on one join plane and share the
// section polygon on it (agent-docs/sweep-design.md §16). The profile must be made
// of straight lines only. Calling it twice is the same as calling it once.
func (c *SweepCommand) MitredJoins() *SweepCommand { c.mitred = true; return c }

// SectionScale states one dimensionless factor per path segment, in path
// order: factors[k] is the ratio of the section at the END of span k to the
// authored profile, so a branch whose radius is r_k at path point k passes
// r_k / r_0. Each factor must be a positive, finite Dimensionless value, and
// the count must equal the path's segment count. On a path of two or more
// spans it requires [SweepCommand.MitredJoins] (agent-docs/sweep-design.md §16.2).
// A second call replaces the first.
func (c *SweepCommand) SectionScale(factors ...units.Value) *SweepCommand {
	scale, err := featureoption.ValidateSectionScale(factors)
	c.scale.Set(sweepSectionScale{factors: scale}, err)
	return c
}

// Do sweeps the section, registers the new body and returns it, or returns the
// error of the first gate that refuses, leaving the document unchanged. A nil
// ctx is ErrDegenerate, and a second call returns ErrConsumed.
func (c *SweepCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationSweep, c.run)
}

func (c *SweepCommand) run(ctx context.Context) (*Body, error) {
	in := featureoption.SweepInput{SurfaceResult: c.surface.on, Mitred: c.mitred}
	if c.scale.IsSet() {
		scale, err := c.scale.Get()
		in.ScaleSet, in.Scale, in.ScaleErr = true, scale.factors, err
	}
	if c.twist.IsSet() {
		in.TwistSet = true
		in.Twist, in.TwistErr = c.twist.Get()
	}
	return sweep(ctx, c.d, c.s, c.sec, pathNode(c.path), in)
}

// sweep is Sweep's implementation. ctx is not nil, and the errors it returns
// carry no operation tag, except the refusal of a stored method value
// (agent-docs/command-api-design.md §11). sec is a *sketch.Profile or a
// *sketch.Chain.
func sweep(ctx context.Context, d *Document, s *sketch.Sketch, sec any, path *sweepinput.Path,
	in featureoption.SweepInput) (*Body, error) {
	switch v := sec.(type) {
	case *sketch.Profile:
		return sweepProfile(ctx, d, s, v, path, in)
	case *sketch.Chain:
		return sweepChain(ctx, d, s, v, path, in)
	default:
		return nil, fmt.Errorf(`%w: Sweep section %T is not a profile or chain`, ErrDegenerate, sec)
	}
}

// foldSweepInput folds the stored sweep methods for a path of segments spans,
// naming the method whose value is refused.
func foldSweepInput(in featureoption.SweepInput, segments int) (featureoption.SweepConfig, error) {
	cfg, err := featureoption.FoldSweep(in, segments)
	if fault, ok := errors.AsType[*featureoption.SweepFaultError](err); ok {
		return featureoption.SweepConfig{}, subjectError(OperationSweep, fault.Method, fault.Err)
	}
	return cfg, err
}

// sweepProfile is the profile-fed Sweep body (agent-docs/sweep-design.md §3-§17).
func sweepProfile(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile, path *sweepinput.Path,
	in featureoption.SweepInput) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || p == nil || path == nil {
		return nil, fmt.Errorf(`%w: Sweep requires a non-nil sketch, profile, and path`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cfg, err := foldSweepInput(in, len(path.Segments))
	if err != nil {
		return nil, err
	}
	node, err := sweepbuild.Profile(ctx, envOf(documentNode(d), d.producerAt(ctx)), s, p, path, cfg)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := d.commitOperationIn(ctx, d.ContainerOfSketch(s), OperationSweep, nil, body); err != nil {
		return nil, err
	}
	return body, nil
}

// This section is the chain-fed form of Sweep, agent-docs/sweep-design.md §15: the open sketch
// chain swept along a Path. §15.1 states why the composite join's pairing rule
// survives an open walk — it pairs by recorded-segment index and reads no
// winding — and §15.2 states why a ONE-SPAN path needs no such rule at all: it
// has no join, so its whole build is the recorded walk transported over that
// one span. A one-span straight path reduces to the chain prism that chain
// Extrude already builds, over the path's own height rather than a resolved
// Extent. A one-span arc reduces to a chain Revolve. Composite paths outside
// §15.8's cardinal class stay ErrUnsupported (Table SC row SC7).

// sweepChain is the chain-fed Sweep body (agent-docs/sweep-design.md §15).
func sweepChain(ctx context.Context, d *Document, s *sketch.Sketch, ch *sketch.Chain, path *sweepinput.Path,
	in featureoption.SweepInput) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || ch == nil || path == nil {
		return nil, fmt.Errorf(`%w: Sweep requires a non-nil sketch, chain, and path`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// SC2 before SC3: a seam refusal names a repair the caller makes in the
	// sketch, and reporting a path refusal first would hide it behind geometry
	// the caller cannot act on (agent-docs/sweep-design.md Table SC).
	chain, plane, err := recordChain(s, ch)
	if err != nil {
		return nil, err
	}
	// The option refusal follows the seam gate and precedes every path gate.
	if err := refuseChainSweepSettings(in, len(path.Segments)); err != nil {
		return nil, err
	}
	node, err := sweepbuild.Chain(ctx, envOf(documentNode(d), d.producerAt(ctx)), chain, plane, path)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := d.commitOperationIn(ctx, d.ContainerOfSketch(s), OperationSweep, nil, body); err != nil {
		return nil, err
	}
	return body, nil
}

// refuseChainSweepSettings folds the stored methods like a profile Sweep and
// refuses each one a chain section cannot honour with ErrUnsupported. A chain
// sweep always returns a sheet, so SurfaceResult has no solid to toggle; the
// twist (Table S row S11), mitred-join and section-scale methods build solids
// from a profile only.
func refuseChainSweepSettings(in featureoption.SweepInput, segments int) error {
	cfg, err := foldSweepInput(in, segments)
	if err != nil {
		return err
	}
	switch {
	case cfg.SurfaceResult:
		return fmt.Errorf(`%w: a chain sweep always returns a sheet; SurfaceResult has no solid to toggle`,
			ErrUnsupported)
	case cfg.Twisted || cfg.Twist.Kind() == units.Angle:
		return fmt.Errorf(`%w: Twist is not implemented for a chain sweep (agent-docs/sweep-design.md Table S row S11)`,
			ErrUnsupported)
	case cfg.Mitred || cfg.Scaled:
		return fmt.Errorf(`%w: MitredJoins and SectionScale build a solid from a profile only`, ErrUnsupported)
	}
	return nil
}
