package wallsurvey

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestWholeArcCarrierKey(t *testing.T) {
	point := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	a := sectionrecord.ArcSeg{Center: point(0, 0), Start: point(10, 0), End: point(0, 10), TEnd: 1}
	b := sectionrecord.ArcSeg{Center: point(0, 0), Start: point(0, 10), End: point(-10, 0), TEnd: 1}
	key, ok := wholeArcCarrierKey(a)
	require.True(t, ok)
	require.NotEmpty(t, key)
	other, ok := wholeArcCarrierKey(&b)
	require.True(t, ok)
	require.Equal(t, key, other)
	b.TStart, b.TEnd = 1, 0
	other, ok = wholeArcCarrierKey(b)
	require.True(t, ok)
	require.Equal(t, key, other)
	shifted := sectionrecord.ArcSeg{Center: point(1, 0), Start: point(11, 0), End: point(1, 10), TEnd: 1}
	shiftedKey, ok := wholeArcCarrierKey(shifted)
	require.True(t, ok)
	require.NotEqual(t, key, shiftedKey)
	smaller := sectionrecord.ArcSeg{Start: point(8, 0), End: point(0, 8), TEnd: 1}
	smallerKey, ok := wholeArcCarrierKey(smaller)
	require.True(t, ok)
	require.NotEqual(t, key, smallerKey)
	one := sectionrecord.ArcSeg{Start: point(1, 0), End: point(0, 1), TEnd: 1}
	e := math.Ldexp(1, -30)
	near := sectionrecord.ArcSeg{Start: point(1, e), End: point(e, 1), TEnd: 1}
	require.Equal(t, math.Hypot(1, 0), math.Hypot(1, e))
	k1, ok := wholeArcCarrierKey(one)
	require.True(t, ok)
	k2, ok := wholeArcCarrierKey(near)
	require.True(t, ok)
	require.NotEqual(t, k1, k2)
	for _, bad := range []sectionrecord.ArcSeg{
		{Start: point(1, 0), End: point(0, 2), TEnd: 1},
		{TEnd: 1}, {Start: point(math.NaN(), 0), End: point(0, 1), TEnd: 1},
		{Start: point(math.Inf(1), 0), End: point(0, 1), TEnd: 1},
		{Start: point(1, 0), End: point(0, 1), TStart: .25, TEnd: .75},
	} {
		_, ok := wholeArcCarrierKey(bad)
		require.False(t, ok)
	}
	_, ok = wholeArcCarrierKey(sectionrecord.LineSeg{})
	require.False(t, ok)
	_, ok = wholeArcCarrierKey(sectionrecord.CircleSeg{})
	require.False(t, ok)
}

func TestRevolveWallCarriers(t *testing.T) {
	a := sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: 10}, End: sectionrecord.Point2{V: 10}, TEnd: 1}
	b := sectionrecord.ArcSeg{Start: sectionrecord.Point2{V: 10}, End: sectionrecord.Point2{U: -10}, TEnd: 1}
	c := sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: 8}, End: sectionrecord.Point2{V: 8}, TEnd: 1}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{a, b, c, sectionrecord.LineSeg{}}}, Holes: []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{a}}}}
	walk := func(indices ...int) survey2d.SideWalk {
		return survey2d.SideWalk{Kind: survey2d.WalkCircular, Segs: indices}
	}
	loops := [][]survey2d.SideWalk{{walk(0), walk(1), walk(2), walk(0, 2), walk(3)}, {walk(0)}}
	axis := revolveaxis.Frame{DU: 1}
	ids, err := revolveWallCarriers(nil, profile, axis, loops)
	require.NoError(t, err)
	require.Equal(t, []int{1, 1, 2, 0, 0}, ids[0])
	require.Equal(t, 1, ids[1][0])
	axis.DUBound = 1
	ids, err = revolveWallCarriers(nil, profile, axis, loops)
	require.NoError(t, err)
	require.Nil(t, ids)
	axis.DUBound = 0
	axis.DU = .5
	ids, err = revolveWallCarriers(nil, profile, axis, loops)
	require.NoError(t, err)
	require.Nil(t, ids)
	axis.DU = 1
	loops[0][0].CV = 1
	ids, err = revolveWallCarriers(nil, profile, axis, loops)
	require.NoError(t, err)
	require.Zero(t, ids[0][0])
	loops[0][0].CV = 0
	loops[0][0].CVBound = 1
	ids, err = revolveWallCarriers(nil, profile, axis, loops)
	require.NoError(t, err)
	require.Zero(t, ids[0][0])
	loops[0][0] = walk()
	_, err = revolveWallCarriers(nil, profile, axis, loops)
	require.ErrorContains(t, err, "revolve wall carrier table:")
	loops[0][0] = walk(100)
	_, err = revolveWallCarriers(nil, profile, axis, loops)
	require.ErrorContains(t, err, "revolve wall carrier table:")
	loops[0][0] = walk(0)
	budget := &proofbound.WorkBudget{ErrFn: func() error { return context.Canceled }}
	_, err = revolveWallCarriers(budget, profile, axis, loops)
	require.ErrorIs(t, err, context.Canceled)
	steps := 0
	budget = &proofbound.WorkBudget{ErrFn: func() error { return nil }, StepFn: func() error {
		steps++
		if steps == 3 {
			return context.Canceled
		}
		return nil
	}}
	_, err = revolveWallCarriers(budget, profile, axis, loops)
	require.ErrorIs(t, err, context.Canceled)
}
