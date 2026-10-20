package thickenbuild

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Thicken builds the solid a sheet body b thickens to, on side, by tmm
// millimetres (its conversion bound tDelta). b must be a sheet whose payload
// has an admitted generator. The result is not yet committed.
func Thicken(ctx context.Context, env topo.Env, b *topo.Body, side Side, tmm, tDelta float64) (*topo.Body, error) {
	switch payload := b.Payload.(type) {
	case patchchain.Payload:
		return thickenPatch(ctx, env, payload, side, tmm, tDelta)
	case prismrecord.Payload:
		return thickenPrism(ctx, env, payload, side, tmm, tDelta)
	case revolveplan.Payload:
		return Revolve(ctx, env, payload, side, tmm, tDelta)
	case sweeprecord.Payload:
		return thickenSweep(ctx, env, payload, side, tmm, tDelta)
	case chainrecord.Payload:
		return ChainExtrude(ctx, env, payload, side, tmm, tDelta)
	case revolveplan.ChainPayload:
		return ChainRevolve(ctx, env, payload, side, tmm, tDelta)
	case sweeprecord.ChainPayload:
		return thickenChainSweep(ctx, env, payload, side, tmm, tDelta)
	default:
		return nil, fmt.Errorf(`%w: this sheet has no admitted Thicken generator`, decaderr.ErrUnsupported)
	}
}

// thickenSweep builds through a one-span sheet's analytic reduction, then
// publishes the result as a Sweep. The reduced builder proves the offset and
// solid; the finishing step restores the path's wall and cap roles.
func thickenSweep(ctx context.Context, env topo.Env, sp sweeprecord.Payload, side Side, tmm, tDelta float64) (*topo.Body, error) {
	if !sp.SurfaceResult || len(sp.Spans) != 0 {
		return nil, fmt.Errorf(`%w: this sweep sheet has no admitted Thicken join proof`, decaderr.ErrUnsupported)
	}
	if sp.Arc {
		body, err := Revolve(ctx, env,
			sp.Revolve, side, tmm, tDelta)
		if err != nil {
			return nil, err
		}
		built, ok := body.Payload.(revolveplan.Payload)
		if !ok {
			return nil, fmt.Errorf(`%w: a thickened arc sweep has no revolve reduction`, decaderr.ErrDegenerate)
		}
		sp.Revolve = built
		sp.Prism.Profile = built.Profile
		sp.Prism.Xform = built.Xform
		sp.SurfaceResult = false
		sweeprecord.FinishArc(body, sp)
		return body, nil
	}
	body, err := thickenPrism(ctx, env, sp.Prism, side, tmm, tDelta)
	if err != nil {
		return nil, err
	}
	sp.SurfaceResult = false
	sweeprecord.FinishStraight(body, sp)
	return body, nil
}

func thickenPatch(ctx context.Context, env topo.Env, pp patchchain.Payload, side Side, tmm, tDelta float64) (*topo.Body, error) {
	level := proofbound.MeasuredScalar(tmm, tDelta)
	var z0, z1 proofbound.BoundedScalar
	switch side {
	case Positive:
		z1 = level
	case Negative:
		z0 = proofbound.BoundedNeg(level)
	case Centered:
		half := proofbound.BoundedQuotient(level.Value, level.Bound, 2, 0)
		z0, z1 = proofbound.BoundedNeg(half), half
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(z1, z0), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the thicken interval has no proven positive height`, decaderr.ErrUnsupported)
	}
	prism := pp.Prism()
	prism.Z0, prism.Z0Delta = z0.Value, z0.Bound
	prism.Z1, prism.Z1Delta = z1.Value, z1.Bound
	return prismrecord.Build(ctx, env, prism, freeform.NewFreeformWork())
}

// thickenPrism builds a through-height wall around a recorded prism sheet.
// This arm first proves that its generated section coordinates denote the
// exact offset of the source section; prismrecord.Build can then keep its
// zero-displacement section certificate.
func thickenPrism(ctx context.Context, env topo.Env, pp prismrecord.Payload, side Side, tmm, tDelta float64) (*topo.Body, error) {
	if !pp.SurfaceResult || pp.SectionDelta != 0 || len(pp.Profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this prism sheet has no admitted Thicken section`, decaderr.ErrUnsupported)
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar()), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the prism sheet has no proven positive sweep height`, decaderr.ErrUnsupported)
	}
	amount, err := Amount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	sec, err := SectionOf(ctx, pp.Profile, side, amount, budget, nil)
	if err != nil {
		return nil, err
	}
	if side != Centered {
		negative := side == Negative
		return prismshell.BuildTube(ctx, env,
			prismshell.CapShellInput{Prism: pp, Sense: sec.Sense(negative)}, sec.Generated(negative))
	}
	annulus, err := sec.Annulus(ctx)
	if err != nil {
		return nil, err
	}
	pp.Profile = annulus
	pp.SurfaceResult = false
	pp.Walks = nil
	return prismrecord.Build(ctx, env, pp, freeform.NewFreeformWork())
}

func thickenChainSweep(ctx context.Context, env topo.Env, sp sweeprecord.ChainPayload,
	side Side, tmm, tDelta float64) (*topo.Body, error) {
	if sp.Arc && len(sp.Spans) == 0 {
		return thickenArcChainSweep(ctx, env, sp, side, tmm, tDelta)
	}
	return thickenCompositeChainSweep(ctx, env, sp, side, tmm, tDelta)
}

// thickenArcChainSweep closes the source walk through the same exact-axis
// chain Revolve offset proof, then publishes its new closed section as a
// solid one-arc Sweep. The original open-chain reduction cannot describe it.
func thickenArcChainSweep(ctx context.Context, env topo.Env, sp sweeprecord.ChainPayload,
	side Side, tmm, tDelta float64) (*topo.Body, error) {
	if sp.Path == nil || len(sp.Path.Records) != 1 {
		return nil, fmt.Errorf(`%w: this chain Sweep has no one-arc Thicken path`, decaderr.ErrUnsupported)
	}
	reverse := sp.Revolve.Phi0 < 0 && sp.Revolve.Phi1 == 0
	if !reverse && (sp.Revolve.Phi0 != 0 || sp.Revolve.Phi1 <= 0) {
		return nil, fmt.Errorf(`%w: this chain Sweep has no path-order cap proof`, decaderr.ErrUnsupported)
	}
	body, err := ChainRevolve(ctx, env,
		sp.Revolve, side, tmm, tDelta)
	if err != nil {
		return nil, err
	}
	built, ok := body.Payload.(revolveplan.Payload)
	if !ok {
		return nil, fmt.Errorf(`%w: thickened chain arc has no solid Revolve reduction`, decaderr.ErrDegenerate)
	}
	sweeprecord.FinishArc(body, sweeprecord.Payload{
		Prism: prismrecord.Payload{
			Profile: built.Profile, Frame: built.Frame, Xform: built.Xform,
		},
		Revolve: built, Arc: true, ReverseArcCaps: reverse, Path: sp.Path,
	})
	return body, nil
}

// thickenCompositeChainSweep closes the authenticated radial line into a
// rectangular section and sends that one section through both path spans.
func thickenCompositeChainSweep(ctx context.Context, env topo.Env, sp sweeprecord.ChainPayload,
	side Side, tmm, tDelta float64) (*topo.Body, error) {
	if len(sp.Spans) != 2 || sp.Spans[0].Arc || !sp.Spans[1].Arc || sp.Path == nil ||
		len(sp.Spans[0].Chain.Chains) != 1 || len(sp.Spans[1].Revolve.Chains) != 1 ||
		sp.Spans[0].Chain.SectionDelta != 0 || sp.Spans[1].Revolve.SectionDelta != 0 ||
		sp.Spans[0].Chain.Xform != r3.Identity() || sp.Spans[1].Revolve.Xform != r3.Identity() {
		return nil, fmt.Errorf(`%w: this chain sweep has no admitted Thicken section`, decaderr.ErrUnsupported)
	}
	straight := sp.Spans[0].Chain
	frame := straight.Frame
	plane := sectionrecord.PlaneRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}
	if err := sweepinput.CardinalChainSweepClass(straight.Chains[0], plane, sp.Path); err != nil {
		return nil, err
	}
	amount, err := Amount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	right, left := RibbonSteps(side)
	// The path's exact half-circle rotates about U=R along +V. RibbonProfile's
	// interval scan checks every moving piece against that axis, so the whole
	// offset family has positive hoop radius before either solid span builds.
	arc, ok := sp.Path.Segments[1], sp.Path.Segments[1].Arc
	if !ok {
		return nil, fmt.Errorf(`%w: the chain sweep has no cardinal arc source`, decaderr.ErrUnsupported)
	}
	radial := thickenaxis.NewRadial(proofarith.FloatRat(arc.Through.X), big.NewRat(0, 1),
		big.NewRat(0, 1), big.NewRat(1, 1))
	section, err := thickenaxis.RibbonProfile(ctx, straight.Chains[0], right, left, amount, budget, work, &radial)
	if err != nil {
		return nil, err
	}
	return sweeprecord.BuildComposite(ctx, env, section, plane,
		frame, sp.Path, work, false)
}
