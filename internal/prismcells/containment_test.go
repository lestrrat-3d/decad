package prismcells

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// TestDeclareOperandLoopsNeedsEverySegment declares a square's four lines as
// one loop, and refuses a loop whose recorded segments do not each have their
// own entity, as when the scene shared one entity between two segments.
func TestDeclareOperandLoopsNeedsEverySegment(t *testing.T) {
	t.Parallel()
	build := func(skip int) (*sketch.Sketch, map[sketch.Entity]Origin) {
		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		require.NoError(t, err)
		pts := []*sketch.Point{s.CreatePoint(0, 0), s.CreatePoint(1, 0), s.CreatePoint(1, 1), s.CreatePoint(0, 1)}
		tags := map[sketch.Entity]Origin{}
		for i := range pts {
			line := s.CreateLine(pts[i], pts[(i+1)%len(pts)])
			if i != skip {
				tags[line] = Origin{Hole: -1, Segment: i}
			}
		}
		return s, tags
	}
	s, tags := build(-1)
	loops, ok, err := DeclareOperandLoops(proofbound.NewWorkBudget(t.Context()), s, tags)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, []OperandLoop{{Hole: -1}}, loops)

	s, tags = build(1)
	_, ok, err = DeclareOperandLoops(proofbound.NewWorkBudget(t.Context()), s, tags)
	require.NoError(t, err)
	require.False(t, ok)
}
