package meshbool

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// refuseWeldedAwayComponentReference is the behavior of RefuseWeldedAwayComponent
// before its early return: it always builds the full facet adjacency.
func refuseWeldedAwayComponentReference(ctx context.Context, tris [][3]int, dropped []bool) error {
	comp := make([]int, len(tris))
	for i := range comp {
		comp[i] = -1
	}
	adj, err := FacetAdjacencyContext(ctx, tris)
	if err != nil {
		return err
	}
	for i := range tris {
		if comp[i] != -1 {
			continue
		}
		queue := []int{i}
		comp[i] = i
		alive := false
		for len(queue) > 0 {
			f := queue[0]
			queue = queue[1:]
			alive = alive || !dropped[f]
			for _, nb := range adj[f] {
				if comp[nb] != -1 {
					continue
				}
				comp[nb] = i
				queue = append(queue, nb)
			}
		}
		if !alive {
			return fmt.Errorf(`%w: reference`, decaderr.ErrUnsupported)
		}
	}
	return nil
}
