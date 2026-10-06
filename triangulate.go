package decad

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/triangulation"
)

// cross2 returns the z-component of (b − a) × (c − a).
func cross2(a, b, c Point2) float64 {
	return (b.U-a.U)*(c.V-a.V) - (b.V-a.V)*(c.U-a.U)
}

// triangulate2DContext maps recorded cap points to the private triangulator.
func triangulate2DContext(ctx context.Context, pts []Point2, loops [][]int) ([][3]int, error) {
	tris, err := triangulation.Triangulate(ctx, triangulationPoints(pts), loops)
	return tris, triangulationError(err)
}

// earClip keeps the root package's geometry tests on the same error boundary.
func earClip(ctx context.Context, pts []Point2, poly []int) ([][3]int, error) {
	tris, err := triangulation.EarClip(ctx, triangulationPoints(pts), poly)
	return tris, triangulationError(err)
}

// bridgeHole keeps the root package's geometry tests on the same error boundary.
func bridgeHole(ctx context.Context, pts []Point2, merged, hole []int) ([]int, error) {
	indices, err := triangulation.BridgeHole(ctx, triangulationPoints(pts), merged, hole)
	return indices, triangulationError(err)
}

func triangulationPoints(pts []Point2) []triangulation.Point2 {
	out := make([]triangulation.Point2, len(pts))
	for i, p := range pts {
		out[i] = triangulation.Point2{U: p.U, V: p.V}
	}
	return out
}

func triangulationError(err error) error {
	if err == nil {
		return nil
	}
	var expected *triangulation.ExpectedError
	if errors.As(err, &expected) {
		return &tessellationExpectedError{err: err}
	}
	return err
}
