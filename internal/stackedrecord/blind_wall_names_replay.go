package stackedrecord

import (
	"context"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// nameOwnWallsReplay gives the same roles as OwnPlan.NameWalls. Each origin's
// bounded decimal buffer and resulting owned string are reserved first.
func nameOwnWallsReplay(ctx context.Context, col WallColumn, faces []*topo.Face,
	budget *proofbound.ReplayBudget) error {
	for _, face := range faces {
		for i := range face.Origins {
			old := face.Origins[i].Role
			// Two signed machine ints have at most 20 decimal bytes each.
			if len(old) > int(^uint(0)>>1)-56 {
				return decaderr.ErrUnsupported
			}
			maxLen := len("slab().region().") + 40 + len(old)
			if err := budget.ReserveBundle(ctx, 1, 1, 0); err != nil {
				return err
			}
			if err := budget.ReserveArray(ctx, uint64(maxLen), 1); err != nil {
				return err
			}
			if err := budget.ReserveBytes(ctx, uint64(maxLen)+128); err != nil {
				return err
			}
			buf := make([]byte, 0, maxLen)
			buf = append(buf, "slab("...)
			buf = strconv.AppendInt(buf, int64(col.Start), 10)
			buf = append(buf, ").region("...)
			buf = strconv.AppendInt(buf, int64(col.Region), 10)
			buf = append(buf, ")."...)
			buf = append(buf, old...)
			face.Origins[i].Role = string(buf)
		}
	}
	return nil
}
