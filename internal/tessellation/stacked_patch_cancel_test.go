package tessellation

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

type patchCancelAtPoll struct {
	context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	stop, polls     int
}

func (c *patchCancelAtPoll) Err() error {
	c.polls++
	if c.polls >= c.stop {
		return context.Canceled
	}
	return nil
}

func TestStackedPatchTrianglesChecksCancellationAfterCopy(t *testing.T) {
	points := []sectionrecord.Point2{{U: 0, V: 0}, {U: 2, V: 0}, {U: 0, V: 2}}
	rings := []StackedRing[int]{{Samples: points, Top: []int{0, 1, 2}}}
	loops := []StackedPatchLoop{{Column: 0, Top: true, Outer: true, ColumnOuter: true}}
	// Four polls cover setup, ring gathering, the preallocation check, and
	// the one output triangle. The fifth checks before publication.
	ctx := &patchCancelAtPoll{Context: context.Background(), stop: 5}
	out, err := StackedPatchTriangles(ctx, rings, loops, false,
		func(context.Context, []sectionrecord.Point2, [][]int, []float64) error { return nil },
		func(context.Context, []sectionrecord.Point2, [][]int) ([][3]int, error) {
			return [][3]int{{0, 1, 2}}, nil
		})
	if !errors.Is(err, context.Canceled) || len(out.Triangles) != 0 || ctx.polls != 5 {
		t.Fatalf("polls=%d triangles=%d error=%v", ctx.polls, len(out.Triangles), err)
	}
}
