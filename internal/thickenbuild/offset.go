package thickenbuild

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Offset builds the sheet a sheet body b offsets to, by dmm millimetres (its
// conversion bound dDelta) on the negative side when negative. b must be a
// sheet whose payload has an admitted generator. The result is not yet
// committed.
func Offset(ctx context.Context, env topo.Env, b *topo.Body, negative bool, dmm, dDelta float64) (*topo.Body, error) {
	switch payload := b.Payload.(type) {
	case patchchain.Payload:
		return offsetPatch(ctx, env, payload, negative, dmm, dDelta)
	case prismrecord.Payload:
		return offsetPrism(ctx, env, payload, negative, dmm, dDelta)
	}
	return nil, fmt.Errorf(`%w: this sheet has no admitted Offset generator`, decaderr.ErrUnsupported)
}

func offsetPatch(ctx context.Context, env topo.Env, pp patchchain.Payload, negative bool, dmm, dDelta float64) (*topo.Body, error) {
	if dDelta != 0 {
		return nil, fmt.Errorf(
			`%w: the offset distance's millimetre conversion displaces the translation, and a patch payload carries no term for it`,
			decaderr.ErrUnsupported)
	}
	signed := dmm
	if negative {
		signed = -dmm
	}
	motion, err := r3.Translation(pp.Prism().Dir(0, 0, 1).Scale(signed))
	if err != nil {
		return nil, fmt.Errorf(`%w: the offset translation is not a placement: %s`, decaderr.ErrUnsupported, err)
	}
	composed, err := pp.Transform().Then(motion)
	if err != nil {
		return nil, fmt.Errorf(`%w: composing the offset placement failed: %s`, decaderr.ErrUnsupported, err)
	}
	return pp.Placed(ctx, env, composed)
}

func offsetPrism(ctx context.Context, env topo.Env, pp prismrecord.Payload, negative bool, dmm, dDelta float64) (*topo.Body, error) {
	if !pp.SurfaceResult || pp.SectionDelta != 0 || len(pp.Profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this prism sheet has no admitted Offset section`, decaderr.ErrUnsupported)
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar()), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the prism sheet has no proven positive sweep height`, decaderr.ErrUnsupported)
	}
	if dDelta != 0 {
		return nil, fmt.Errorf(`%w: the offset distance cannot be generated exactly in millimetres`, decaderr.ErrUnsupported)
	}
	sense := -1
	if negative {
		sense = +1
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	section, err := thickenaxis.OffsetSection(ctx, pp.Profile, sense, dmm, offset2d.ShellTol, budget)
	if err != nil {
		return nil, err
	}
	pp.Profile = section
	pp.Walks = nil
	pp.BlendSegs, pp.BlendKind = nil, ""
	return prismrecord.Build(ctx, env, pp, freeform.NewFreeformWork())
}
