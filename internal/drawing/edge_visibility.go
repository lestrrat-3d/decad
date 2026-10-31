package drawing

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/r3"
)

const minParallelEdges = 128

// Each edge reads the completed depth buffer and owns its output slices.
// Append after the workers join to preserve the sorted edge and run order.
func splitEdgesParallel(ctx context.Context, verts []r3.Vec, edges []edgeKey,
	proj raster.Projection, depths []float64, workers int) (Lines, error) {
	parts, err := orderedwork.Run(ctx, ctx, edges, workers, func(_ context.Context, e edgeKey) (Lines, error) {
		var part Lines
		splitEdge(verts[e[0]], verts[e[1]], proj, depths, &part)
		return part, nil
	})
	if err != nil {
		return Lines{}, err
	}
	var lines Lines
	for i, part := range parts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return Lines{}, err
			}
		}
		lines.Visible = append(lines.Visible, part.Visible...)
		lines.Hidden = append(lines.Hidden, part.Hidden...)
	}
	if err := ctx.Err(); err != nil {
		return Lines{}, err
	}
	return lines, nil
}
