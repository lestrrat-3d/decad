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
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
	"github.com/lestrrat-3d/r3"
)

func thickenChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if sp.arc && len(sp.spans) == 0 {
		return thickenArcChainSweep(ctx, d, sp, side, tmm, tDelta)
	}
	return thickenCompositeChainSweep(ctx, d, sp, side, tmm, tDelta)
}

// thickenArcChainSweep closes the source walk through the same exact-axis
// chain Revolve offset proof, then publishes its new closed section as a
// solid one-arc Sweep. The original open-chain reduction cannot describe it.
func thickenArcChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if sp.path == nil || len(sp.path.Records) != 1 {
		return nil, fmt.Errorf(`%w: this chain Sweep has no one-arc Thicken path`, ErrUnsupported)
	}
	reverse := sp.revolve.phi0 < 0 && sp.revolve.phi1 == 0
	if !reverse && (sp.revolve.phi0 != 0 || sp.revolve.phi1 <= 0) {
		return nil, fmt.Errorf(`%w: this chain Sweep has no path-order cap proof`, ErrUnsupported)
	}
	body, err := thickenChainRevolve(ctx, d, sp.revolve, side, tmm, tDelta)
	if err != nil {
		return nil, err
	}
	built, ok := body.n.Payload.(revolvePayload)
	if !ok {
		return nil, fmt.Errorf(`%w: thickened chain arc has no solid Revolve reduction`, ErrDegenerate)
	}
	finishArcSweepBody(body, sweepPayload{
		prism: prismrecord.Payload{
			Profile: built.profile, Frame: built.frame, Xform: built.xform,
		},
		revolve: built, arc: true, reverseArcCaps: reverse, path: sp.path,
	})
	return body, nil
}

// thickenCompositeChainSweep closes the authenticated radial line into a
// rectangular section and sends that one section through both path spans.
func thickenCompositeChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if len(sp.spans) != 2 || sp.spans[0].arc || !sp.spans[1].arc || sp.path == nil ||
		len(sp.spans[0].chain.Chains) != 1 || len(sp.spans[1].revolve.chains) != 1 ||
		sp.spans[0].chain.SectionDelta != 0 || sp.spans[1].revolve.sectionDelta != 0 ||
		sp.spans[0].chain.Xform != r3.Identity() || sp.spans[1].revolve.xform != r3.Identity() {
		return nil, fmt.Errorf(`%w: this chain sweep has no admitted Thicken section`, ErrUnsupported)
	}
	straight := sp.spans[0].chain
	frame := straight.Frame
	plane := planeRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}
	if err := sweepinput.CardinalChainSweepClass(straight.Chains[0], plane, sp.path); err != nil {
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
	arc, ok := sp.path.Segments[1], sp.path.Segments[1].Arc
	if !ok {
		return nil, fmt.Errorf(`%w: the chain sweep has no cardinal arc source`, ErrUnsupported)
	}
	radial := thickenaxis.NewRadial(proofarith.FloatRat(arc.Through.X), big.NewRat(0, 1),
		big.NewRat(0, 1), big.NewRat(1, 1))
	section, err := thickenaxis.RibbonProfile(ctx, straight.Chains[0], right, left, amount, budget, work, &radial)
	if err != nil {
		return nil, err
	}
	return evalCompositeSweepContext(ctx, d, d.nextProducerID(), section, plane, frame,
		sp.path, work, false)
}
