package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
	"github.com/lestrrat-3d/r3"
)

// thickenCompositeChainSweep closes the authenticated radial line into a
// rectangular section and sends that one section through both path spans.
func thickenCompositeChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if len(sp.spans) != 2 || sp.spans[0].arc || !sp.spans[1].arc || sp.path == nil ||
		len(sp.spans[0].chain.chains) != 1 || len(sp.spans[1].revolve.chains) != 1 ||
		sp.spans[0].chain.sectionDelta != 0 || sp.spans[1].revolve.sectionDelta != 0 ||
		sp.spans[0].chain.xform != r3.Identity() || sp.spans[1].revolve.xform != r3.Identity() {
		return nil, fmt.Errorf(`%w: this chain sweep has no admitted Thicken section`, ErrUnsupported)
	}
	straight := sp.spans[0].chain
	frame := straight.frame
	plane := planeRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}
	if err := cardinalChainSweepClass(straight.chains[0], plane, sp.path); err != nil {
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
	arc, ok := sp.path.segments[1].(ArcThrough)
	if !ok {
		return nil, fmt.Errorf(`%w: the chain sweep has no cardinal arc source`, ErrUnsupported)
	}
	radial := thickenaxis.NewRadial(proofarith.FloatRat(arc.Through.X), big.NewRat(0, 1),
		big.NewRat(0, 1), big.NewRat(1, 1))
	section, err := thickenaxis.RibbonProfile(ctx, straight.chains[0], right, left, amount, budget, work, &radial)
	if err != nil {
		return nil, err
	}
	return evalCompositeSweepContext(ctx, d, d.nextProducerID(), section, plane, frame,
		sp.path, work, false)
}
