package modifyselect

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// RequireLateral is Fillet's route P classification (brep-modify §4.2):
// every selected edge is a lateral edge of pp, the junction at one corner of
// its section (MatchCorner). Any other edge is modify S1's class.
func RequireLateral(ctx context.Context, pp prismrecord.Payload, edges []*topo.Edge) error {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return err
	}
	for ei, e := range edges {
		_, _, found, err := MatchCorner(budget, pp, loops, e)
		if err != nil {
			return err
		}
		if !found {
			return fmt.Errorf(`%w: %s is not a lateral edge of the prism`, decaderr.ErrUnsupported, EdgeContext(ei, e))
		}
	}
	return nil
}
