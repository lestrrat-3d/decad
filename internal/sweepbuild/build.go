// Package sweepbuild builds the bodies of Sweep over topology nodes: the
// profile-fed and chain-fed forms after the options are decoded
// (agent-docs/sweep-design.md). Root keeps the public gates and the commit.
package sweepbuild

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Profile builds the Sweep of profile p of sketch s along path under the
// decoded options cfg. Its refusals run in this order: the record, the
// recorded-area falsifier, the path, then the construction the options select.
// The result is not yet committed.
func Profile(ctx context.Context, env topo.Env, s *sketch.Sketch, p *sketch.Profile, path *sweepinput.Path,
	cfg featureoption.SweepConfig) (*topo.Body, error) {
	surfaceResult := cfg.SurfaceResult

	profile, plane, profileArea, err := sectioncert.RecordProfile(s, p)
	if err != nil {
		return nil, err
	}
	work := freeform.NewFreeformWork()
	if len(path.Records) > 1 && !cfg.Mitred && !cfg.Scaled {
		work = sweeprecord.NewCompositeWork(profile)
	}
	recordArea, err := profile.FalsifyRecordedArea(profileArea, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(path.Records) == 0 || len(path.Segments) == 0 {
		return nil, fmt.Errorf(`%w: a sweep path must contain at least one span`, decaderr.ErrDegenerate)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	if cfg.Twisted {
		if sweepinput.SamePathPoint(path.Start, path.End) {
			return nil, fmt.Errorf(`%w: a closed sweep path cannot carry nonzero twist`, decaderr.ErrUnsupported)
		}
		if err := sweepinput.ValidateSweepPathGeometry(path, plane); err != nil {
			return nil, err
		}
		body, err := twistsweep.Eval(ctx, env, profile, plane, path, cfg.Twist, recordArea, r3.Identity())
		if err != nil {
			return nil, err
		}
		return body, ctx.Err()
	}
	if cfg.Mitred || cfg.Scaled {
		// agent-docs/sweep-design.md §16: either option selects the mitred
		// polyline builder, whose Table SM gates replace S6 and S10.
		body, err := mitred(ctx, env, profile, plane, path, cfg)
		if err != nil {
			return nil, err
		}
		return body, ctx.Err()
	}
	if err := sweepinput.ValidateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if err := sweepinput.ValidateAnalyticProfile(profile); err != nil {
		return nil, err
	}

	if sweepinput.SamePathPoint(path.Start, path.End) {
		if surfaceResult {
			return nil, fmt.Errorf(`%w: a closed sweep surface result is not implemented`, decaderr.ErrUnsupported)
		}
		var body *topo.Body
		if _, circle := sweepinput.ClosedCircle(path.Records); circle {
			body, err = closedCircle(ctx, env, profile, plane, frame, path, work)
		} else {
			body, err = sweeprecord.BuildClosedCyclic(ctx, env, profile, plane, frame, path, work)
		}
		if err != nil {
			return nil, err
		}
		return body, ctx.Err()
	}

	var body *topo.Body
	if len(path.Segments) > 1 {
		body, err = sweeprecord.BuildComposite(ctx, env, profile, plane, frame, path, work, surfaceResult)
	} else if segment := path.Segments[0]; !segment.Arc {
		height, heightBound, lineErr := sweepinput.ValidateStraightSweepPath(path, frame)
		if lineErr != nil {
			return nil, lineErr
		}
		// The flag rides the REDUCED prismPayload, not just the finishing
		// sweepPayload literal below: internal/prismrecord/build.go's prismrecord.Build reads
		// pp.SurfaceResult to decide Kind()/solid, cap omission, topobuild.SheetLumps and
		// the area subtraction, which is what gives the line reduction the
		// whole sheet behaviour for free (agent-docs/surface-design.md §4).
		prism := prismrecord.Payload{
			Profile:       profile,
			Frame:         frame,
			Z1:            height,
			Z1Delta:       heightBound,
			Xform:         r3.Identity(),
			SurfaceResult: surfaceResult,
		}
		body, err = prismrecord.Build(ctx, env, prism, work)
		if err == nil {
			sweeprecord.FinishStraight(body, sweeprecord.Payload{Prism: prism, Path: path, SurfaceResult: surfaceResult})
		}
	} else {
		body, err = sweeprecord.BuildArc(ctx, env, profile, plane, frame, path, path.Records[0], work, surfaceResult)
	}
	if err != nil {
		return nil, err
	}
	return body, ctx.Err()
}

// Chain builds the chain-fed Sweep of the recorded chain along path
// (agent-docs/sweep-design.md §15). The seam and option gates have already run. The
// result is not yet committed.
func Chain(ctx context.Context, env topo.Env, chain sectionrecord.ChainRecord, plane sectionrecord.PlaneRecord,
	path *sweepinput.Path) (*topo.Body, error) {
	if len(path.Records) == 0 || len(path.Segments) == 0 {
		return nil, fmt.Errorf(`%w: a sweep path must contain at least one span`, decaderr.ErrDegenerate)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	if err := sweepinput.ValidateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if err := sweepinput.ValidateAnalyticSegments(chain.Segments, "chain"); err != nil {
		return nil, err
	}
	if sweepinput.SamePathPoint(path.Start, path.End) {
		return nil, fmt.Errorf(`%w: closed sweep paths are not implemented`, decaderr.ErrUnsupported)
	}

	if len(path.Segments) > 1 {
		return compositeChain(ctx, env, chain, plane, path)
	}
	if path.Segments[0].Arc {
		return arcChain(ctx, env, chain, plane, frame, path)
	}
	height, heightBound, err := sweepinput.ValidateStraightSweepPath(path, frame)
	if err != nil {
		return nil, err
	}

	// ONE free-form work counter for the whole call, exactly as chain Extrude
	// opens for its own build (agent-docs/spline-design.md §5.2).
	work := freeform.NewFreeformWork()
	// z0 is exactly zero and carries no bound: the recorded walk sits in the
	// sketch plane by construction and the path starts there (S5's own gate
	// above). Only the far level is derived, and z1Delta is where the path's
	// composed length bound lands — the term §15.2 names as the one thing a
	// chain sweep carries that the same walk's chain Extrude reading may not.
	reduction := chainrecord.Payload{
		Chains:  []sectionrecord.ChainRecord{chain},
		Frame:   frame,
		Z0:      0,
		Z1:      height,
		Z1Delta: heightBound,
		Xform:   r3.Identity(),
	}
	body, err := chainrecord.Build(ctx, env, reduction, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	sweeprecord.FinishChain(body, sweeprecord.ChainPayload{Chain: reduction, Path: path})
	return body, nil
}

func compositeChain(ctx context.Context, env topo.Env, chain sectionrecord.ChainRecord, plane sectionrecord.PlaneRecord,
	path *sweepinput.Path) (*topo.Body, error) {
	payload, work, err := sweeprecord.CompositeChainPayload(ctx, chain, plane, path)
	if err != nil {
		return nil, err
	}
	body, err := sweeprecord.ReplayChainWork(ctx, env, payload, work)
	if err != nil {
		return nil, err
	}
	return body, ctx.Err()
}

// arcChain builds sweeprecord.ArcChainReduction's chain Revolve wall and
// records it as an arc Sweep of the path.
func arcChain(ctx context.Context, env topo.Env, chain sectionrecord.ChainRecord,
	plane sectionrecord.PlaneRecord, frame r3.Frame, path *sweepinput.Path) (*topo.Body, error) {
	reduction, work, err := sweeprecord.ArcChainReduction(ctx, chain, plane, frame, path)
	if err != nil {
		return nil, err
	}
	body, err := revolveplan.BuildChain(ctx, env, reduction, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	sweeprecord.FinishChain(body, sweeprecord.ChainPayload{Revolve: reduction, Arc: true, Path: path})
	return body, nil
}

// closedCircle reduces the structurally proven full turn to the existing full
// Revolve evaluator. That evaluator supplies axis-incidence, topology, four
// bounded readings, and an occupied-volume mesh proof.
func closedCircle(ctx context.Context, env topo.Env, profile momentinput.Profile, plane sectionrecord.PlaneRecord,
	frame r3.Frame, path *sweepinput.Path, work *freeform.FreeformWork,
) (*topo.Body, error) {
	carrier, ok := sweepinput.ClosedCircle(path.Records)
	if !ok {
		return nil, fmt.Errorf(`%w: the closed sweep path is outside the exact two-arc full-circle class`, decaderr.ErrUnsupported)
	}
	first := path.Records[0]
	geometry, err := sweeparc.Derive(first.Start, carrier, first.ArcPhi, first.ArcAngle, plane)
	if err != nil {
		return nil, err
	}
	ax, side, err := revolveplan.ResolveAxisSide(ctx, profile, geometry.Line, work)
	if err != nil {
		return nil, err
	}
	phi0, phi1, den := 0.0, 2*math.Pi, revolveangle.FullTurn()
	if side < 0 {
		phi0, phi1 = -phi1, -phi0
		den.Phi0, den.Phi1 = den.Phi1.Neg(), den.Phi0.Neg()
	}
	revolve := revolveplan.Payload{
		Profile:     profile,
		Frame:       frame,
		Ax:          ax,
		Phi0:        phi0,
		Phi1:        phi1,
		Full:        true,
		Den:         den,
		Xform:       r3.Identity(),
		RadialProof: ax.RadialProof,
	}
	body, err := revolveplan.Build(ctx, env, revolve, work)
	if err != nil {
		return nil, err
	}
	witness := prismrecord.Payload{Profile: profile, Frame: frame, Xform: r3.Identity()}
	sweeprecord.FinishArc(body, sweeprecord.Payload{
		Prism:        witness,
		Revolve:      revolve,
		Arc:          true,
		ClosedCircle: true,
		Path:         path,
	})
	return body, nil
}

// mitred runs Table SM's gates in §5's order and builds the body. The options
// were validated by the caller (SM3, SM9's surface-result arm), and the
// profile is already authenticated.
func mitred(ctx context.Context, env topo.Env, profile momentinput.Profile, plane sectionrecord.PlaneRecord,
	path *sweepinput.Path, cfg featureoption.SweepConfig) (*topo.Body, error) {
	for k, segment := range path.Segments {
		if segment.Arc {
			return nil, fmt.Errorf(`%w: a mitred or scaled sweep follows LineTo segments only; path segment %d is %s`,
				decaderr.ErrUnsupported, k, "decad.ArcThrough")
		}
	}
	if cfg.Scaled && !cfg.Mitred && len(path.Segments) > 1 {
		return nil, fmt.Errorf(`%w: WithSectionScale on a path of %d spans requires WithMitredJoins`,
			decaderr.ErrUnsupported, len(path.Segments))
	}
	if err := sweepinput.ValidateSweepPathStart(path, plane); err != nil {
		return nil, err
	}
	if sweepinput.SamePathPoint(path.Start, path.End) {
		return nil, fmt.Errorf(`%w: closed sweep paths are not implemented`, decaderr.ErrUnsupported)
	}
	_, loopIdx, err := sweepmitre.Loops(profile.Outer, profile.Holes)
	if err != nil {
		return nil, err
	}
	if err := sweepmitre.Preflight(loopIdx, len(path.Segments), compositesweep.MaxSpansPerCall); err != nil {
		return nil, err
	}
	factors := cfg.Factors
	if factors == nil {
		factors = make([]float64, len(path.Segments))
		for k := range factors {
			factors[k] = 1
		}
	}
	return sweepmitre.Build(ctx, env, sweepmitre.Payload{
		Profile: profile,
		Plane:   plane,
		Path:    path,
		Factors: factors,
		Xform:   r3.Identity(),
	})
}
