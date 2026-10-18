package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismrecord"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"
)

// thickenPrism builds a through-height wall around a recorded prism sheet.
// This arm first proves that its generated section coordinates denote the
// exact offset of the source section; prismrecord.Build can then keep its
// zero-displacement section certificate.
func thickenPrism(ctx context.Context, d *Document, pp prismPayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if !pp.SurfaceResult || pp.SectionDelta != 0 || len(pp.Profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this prism sheet has no admitted Thicken section`, ErrUnsupported)
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar()), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the prism sheet has no proven positive sweep height`, ErrUnsupported)
	}
	amount, err := thickenbuild.Amount(tmm, tDelta, thickenbuild.Side(side))
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	sec, err := thickenbuild.SectionOf(ctx, pp.Profile, thickenbuild.Side(side), amount, budget, nil)
	if err != nil {
		return nil, err
	}
	if side != ThickenCentered {
		negative := side == ThickenNegative
		return evalTubeContext(ctx, d, d.nextProducerID(), pp, sec.Generated(negative), sec.Sense(negative))
	}
	annulus, err := sec.Annulus(ctx)
	if err != nil {
		return nil, err
	}
	pp.Profile = annulus
	pp.SurfaceResult = false
	pp.Walks = nil
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), pp, freeform.NewFreeformWork()))
}
