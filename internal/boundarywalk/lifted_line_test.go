package boundarywalk_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

func liftWork() *freeform.FreeformWork {
	work := freeform.NewFreeformWork()
	work.RaiseLimit(1 << 30)
	return work
}

func TestLiftedLineWalkIsTheLineAsOneCubicSpan(t *testing.T) {
	t.Parallel()
	line := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 1, V: 2}, End: sectionrecord.Point2{U: 7, V: 10}, TStart: 0, TEnd: 1}
	walk, err := boundarywalk.LiftedLineWalk(line, liftWork())
	require.NoError(t, err)
	require.Equal(t, survey2d.WalkFreeform, walk.Kind)
	require.Len(t, walk.Spans, 1)
	require.Len(t, walk.Spans[0], 4)
	require.Equal(t, [2]float64{1, 2}, [2]float64{walk.StartU, walk.StartV})
	require.Equal(t, [2]float64{7, 10}, [2]float64{walk.EndU, walk.EndV})
	require.InDelta(t, 10, walk.Length, 1e-12, "a 6-8-10 triangle's hypotenuse")
	// Every control point lies on the line: the cross product with its direction is exactly zero.
	for _, c := range walk.Spans[0] {
		cross := new(big.Rat).Sub(
			new(big.Rat).Mul(new(big.Rat).Sub(c.U, ratOf(1)), ratOf(8)),
			new(big.Rat).Mul(new(big.Rat).Sub(c.V, ratOf(2)), ratOf(6)))
		require.Zero(t, cross.Sign())
	}
}

func TestLiftedLineWalkFollowsATrimmedReversedRange(t *testing.T) {
	t.Parallel()
	line := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0}, End: sectionrecord.Point2{U: 8, V: 0}, TStart: 0.75, TEnd: 0.25}
	walk, err := boundarywalk.LiftedLineWalk(line, liftWork())
	require.NoError(t, err)
	require.InDelta(t, 6, walk.StartU, 0)
	require.InDelta(t, 2, walk.EndU, 0)
	require.InDelta(t, 4, walk.Length, 1e-12)
	require.False(t, math.IsNaN(walk.LengthBound))
}

func TestLiftedLineWalkRefusesWithoutACounter(t *testing.T) {
	t.Parallel()
	line := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0}, End: sectionrecord.Point2{U: 1, V: 0}, TStart: 0, TEnd: 1}
	_, err := boundarywalk.LiftedLineWalk(line, nil)
	require.Error(t, err)
	_, err = boundarywalk.LiftedLineWalk(sectionrecord.CircleSeg{}, liftWork())
	require.Error(t, err)
}

func ratOf(v int64) *big.Rat { return big.NewRat(v, 1) }
