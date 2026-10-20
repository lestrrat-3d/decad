package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/sweepbuild"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file routes a zero-twist line or circular-arc path through the existing
// analytic prism and revolve evaluators. Composite paths are admitted when
// their transported frames and separation certificates close. The distinct
// sweepPayload preserves the operation's downstream staging and replays every
// reduction for placement. internal/twistsweep owns the bounded nonzero-twist case.
//
// WithSurfaceResult() (agent-docs/surface-design.md §4, agent-docs/sweep-design.md §14
// increment 3) reaches every one of Sweep's three build paths, but not through
// one shared mechanism: a one-span straight or arc path sets the flag on its
// reduced prismPayload/revolvePayload and gets the whole sheet behaviour for
// free from internal/prismrecord/build.go and internal/revolveplan/build.go, while a composite path's spans
// stay solid (sweeprecord.SpanPayload never carries the flag) and
// compositesweep.Assemble (internal/compositesweep/assemble.go) consumes sweepPayload's own
// flag to omit the two outer caps after every span has built solid.
//
// An open Sweep mints two section caps. The exact two-arc closed circle of
// §3.3 reduces to a full Revolve. The §§3.4–3.5 circular-section loops sew
// analytic spans cyclically. Neither closed build mints section caps.

// SweepOption configures Sweep.
type SweepOption interface {
	option.Interface
	sweepOption()
}

// WithSweepTwist rotates the profile about the path tangent as it travels.
// A nonzero angle requires an origin XY sketch and a strictly convex whole-line
// profile centred on the axis. Section 17 admits a positive-Z straight path
// and a bounded line followed by a cardinal XZ quarter arc. Other cases
// return ErrUnsupported without changing the document.
func WithSweepTwist(angle units.Value) SweepOption {
	return sweepOptionValue{featureoption.WithSweepTwist(angle)}
}

// Sweep moves the section sec along path, registers the resulting body, and
// returns it. sec is a closed [*sketch.Profile], which sweeps to a solid
// (sheet under [WithSurfaceResult]), or an open [*sketch.Chain], which sweeps
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
// [WithSurfaceResult] (there is no solid to toggle), [WithSweepTwist],
// [WithMitredJoins] and [WithSectionScale]. That refusal follows the chain's
// seam gate and precedes every path-geometry gate (agent-docs/sweep-design.md §15.3).
//
// A nil context, document, sketch, section or path returns ErrDegenerate.
// Every failure and cancellation
// leaves the document unchanged.
func Sweep[S Section](ctx context.Context, d *Document, s *sketch.Sketch, sec S, path *Path,
	opts ...SweepOption) (*Body, error) {
	switch v := any(sec).(type) {
	case *sketch.Profile:
		return sweepProfile(ctx, d, s, v, pathNode(path), opts)
	case *sketch.Chain:
		return sweepChain(ctx, d, s, v, pathNode(path), opts)
	default:
		return nil, fmt.Errorf(`%w: Sweep section %T is not a profile or chain`, ErrDegenerate, sec)
	}
}

// sweepProfile is the profile-fed Sweep body (agent-docs/sweep-design.md §3-§17).
func sweepProfile(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile, path *sweepinput.Path,
	opts []SweepOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a sweep`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || p == nil || path == nil {
		return nil, fmt.Errorf(`%w: Sweep requires a non-nil sketch, profile, and path`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cfg, err := decodeSweepOptions(opts, len(path.Segments))
	if err != nil {
		return nil, err
	}
	node, err := sweepbuild.Profile(ctx, envOf(documentNode(d), d.nextProducerID()), s, p, path, cfg)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	d.commit(body)
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
	opts []SweepOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a sweep`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || ch == nil || path == nil {
		return nil, fmt.Errorf(`%w: Sweep requires a non-nil sketch, chain, and path`, ErrDegenerate)
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
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
	if err := refuseChainSweepOptions(opts, len(path.Segments)); err != nil {
		return nil, err
	}
	node, err := sweepbuild.Chain(ctx, envOf(documentNode(d), d.nextProducerID()), chain, plane, path)
	if err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	d.commit(body)
	return body, nil
}

// refuseChainSweepOptions decodes opts like a profile Sweep and refuses each
// option a chain section cannot honour with ErrUnsupported. A chain sweep
// always returns a sheet, so WithSurfaceResult has no solid to toggle; the
// twist (Table S row S11), mitred-join and section-scale options build solids
// from a profile only.
func refuseChainSweepOptions(opts []SweepOption, segments int) error {
	cfg, err := decodeSweepOptions(opts, segments)
	if err != nil {
		return err
	}
	switch {
	case cfg.SurfaceResult:
		return fmt.Errorf(`%w: a chain sweep always returns a sheet; WithSurfaceResult has no solid to toggle`,
			ErrUnsupported)
	case cfg.Twisted || cfg.Twist.Kind() == units.Angle:
		return fmt.Errorf(`%w: WithSweepTwist is not implemented for a chain sweep (agent-docs/sweep-design.md Table S row S11)`,
			ErrUnsupported)
	case cfg.Mitred || cfg.Scaled:
		return fmt.Errorf(`%w: WithMitredJoins and WithSectionScale build a solid from a profile only`, ErrUnsupported)
	}
	return nil
}
