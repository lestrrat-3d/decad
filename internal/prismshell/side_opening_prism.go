package prismshell

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// SideOpeningPrism is BO1: W over the receiver's interval, frame and
// placement, carrying the offset's section displacement and, where a rim on
// a removed arc names its cut at a float parameter, three times that cut's
// gap on top (§4.4).
func SideOpeningPrism(ctx context.Context, env topo.Env, pp prismrecord.Payload, sec Section) (*topo.Body, error) {
	delta := sec.Delta
	if sec.CutGap > 0 {
		delta = proofbound.AbsSumUpper(delta, proofbound.ProductUpper(3, sec.CutGap))
		if proofbound.IsNonFinite(delta) {
			return nil, offset2d.ErrUnbounded
		}
	}
	return prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile:      sec.Wall,
		Frame:        pp.Frame,
		Z0:           pp.Z0,
		Z1:           pp.Z1,
		Z0Delta:      pp.Z0Delta,
		Z1Delta:      pp.Z1Delta,
		Xform:        pp.Xform,
		SectionDelta: delta,
	}, freeform.NewFreeformWork())
}
