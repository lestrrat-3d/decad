package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
	"github.com/lestrrat-3d/r3"
)

func thickenChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if sp.Arc && len(sp.Spans) == 0 {
		return thickenArcChainSweep(ctx, d, sp, side, tmm, tDelta)
	}
	return thickenCompositeChainSweep(ctx, d, sp, side, tmm, tDelta)
}

// thickenArcChainSweep closes the source walk through the same exact-axis
// chain Revolve offset proof, then publishes its new closed section as a
// solid one-arc Sweep. The original open-chain reduction cannot describe it.
func thickenArcChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if sp.Path == nil || len(sp.Path.Records) != 1 {
		return nil, fmt.Errorf(`%w: this chain Sweep has no one-arc Thicken path`, ErrUnsupported)
	}
	reverse := sp.Revolve.Phi0 < 0 && sp.Revolve.Phi1 == 0
	if !reverse && (sp.Revolve.Phi0 != 0 || sp.Revolve.Phi1 <= 0) {
		return nil, fmt.Errorf(`%w: this chain Sweep has no path-order cap proof`, ErrUnsupported)
	}
	body, err := thickenChainRevolve(ctx, d, sp.Revolve, side, tmm, tDelta)
	if err != nil {
		return nil, err
	}
	built, ok := body.n.Payload.(revolvePayload)
	if !ok {
		return nil, fmt.Errorf(`%w: thickened chain arc has no solid Revolve reduction`, ErrDegenerate)
	}
	sweeprecord.FinishArc(bodyNode(body), sweepPayload{
		Prism: prismrecord.Payload{
			Profile: built.Profile, Frame: built.Frame, Xform: built.Xform,
		},
		Revolve: built, Arc: true, ReverseArcCaps: reverse, Path: sp.Path,
	})
	return body, nil
}

// thickenCompositeChainSweep closes the authenticated radial line into a
// rectangular section and sends that one section through both path spans.
func thickenCompositeChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if len(sp.Spans) != 2 || sp.Spans[0].Arc || !sp.Spans[1].Arc || sp.Path == nil ||
		len(sp.Spans[0].Chain.Chains) != 1 || len(sp.Spans[1].Revolve.Chains) != 1 ||
		sp.Spans[0].Chain.SectionDelta != 0 || sp.Spans[1].Revolve.SectionDelta != 0 ||
		sp.Spans[0].Chain.Xform != r3.Identity() || sp.Spans[1].Revolve.Xform != r3.Identity() {
		return nil, fmt.Errorf(`%w: this chain sweep has no admitted Thicken section`, ErrUnsupported)
	}
	straight := sp.Spans[0].Chain
	frame := straight.Frame
	plane := planeRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}
	if err := sweepinput.CardinalChainSweepClass(straight.Chains[0], plane, sp.Path); err != nil {
		return nil, err
	}
	amount, err := thickenAmount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	right, left := thickenRibbonSteps(side)
	// The path's exact half-circle rotates about U=R along +V. RibbonProfile's
	// interval scan checks every moving piece against that axis, so the whole
	// offset family has positive hoop radius before either solid span builds.
	arc, ok := sp.Path.Segments[1], sp.Path.Segments[1].Arc
	if !ok {
		return nil, fmt.Errorf(`%w: the chain sweep has no cardinal arc source`, ErrUnsupported)
	}
	radial := thickenaxis.NewRadial(proofarith.FloatRat(arc.Through.X), big.NewRat(0, 1),
		big.NewRat(0, 1), big.NewRat(1, 1))
	section, err := thickenaxis.RibbonProfile(ctx, straight.Chains[0], right, left, amount, budget, work, &radial)
	if err != nil {
		return nil, err
	}
	return bodyResult(sweeprecord.BuildComposite(ctx, envOf(documentNode(d), d.nextProducerID()), section, plane,
		frame, sp.Path, work, false))
}
