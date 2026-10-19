package revolveaxis_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/stretchr/testify/require"
)

func TestWalkChargedPreservesZeroCharge(t *testing.T) {
	ax := revolveaxis.Frame{DU: 0, DV: 1}
	walk, err := boundarywalk.WalkOf(boundarywalk.LineSeg{
		Start: boundarywalk.Point2{U: 3, V: 0}, End: boundarywalk.Point2{U: 3, V: 20}, TStart: 0, TEnd: 1,
	}, nil)
	require.NoError(t, err)

	got := ax.WalkCharged(walk, proofbound.WalkEndBound{}, proofbound.WalkEndBound{})
	require.Equal(t, ax.ToAxisRhoBound(walk.StartU, walk.StartV), got.StartVBound)
	require.Equal(t, ax.ToAxisRhoBound(walk.EndU, walk.EndV), got.EndVBound)
	require.Equal(t, walk.LengthBound, got.LengthBound)
	require.Equal(t, walk.LengthUpper, got.LengthUpper)
	require.Equal(t, walk.CoordUpper, got.CoordUpper)
	require.Equal(t, ax.Walk(walk), got)
}

func TestWalkChargedWidensOnlyTheChargedEndpointAndSharedBounds(t *testing.T) {
	ax := revolveaxis.Frame{DU: 0, DV: 1}
	walk, err := boundarywalk.WalkOf(boundarywalk.LineSeg{
		Start: boundarywalk.Point2{U: 3, V: 0}, End: boundarywalk.Point2{U: 3, V: 20}, TStart: 0, TEnd: 1,
	}, nil)
	require.NoError(t, err)
	plain := ax.Walk(walk)
	charge := proofbound.WalkEndBound{U: 1e-9, V: 1e-9}

	startOnly := ax.WalkCharged(walk, charge, proofbound.WalkEndBound{})
	require.Greater(t, startOnly.StartVBound, plain.StartVBound)
	require.Equal(t, plain.EndVBound, startOnly.EndVBound)
	require.Greater(t, startOnly.LengthBound, plain.LengthBound)
	require.Greater(t, startOnly.LengthUpper, plain.LengthUpper)
	require.Greater(t, startOnly.CoordUpper, plain.CoordUpper)
	require.Greater(t, startOnly.AxisMomentUpper, plain.AxisMomentUpper)
	require.Greater(t, startOnly.AxisRadiusUpper, plain.AxisRadiusUpper)

	both := ax.WalkCharged(walk, charge, charge)
	require.Greater(t, both.EndVBound, plain.EndVBound)
	require.Greater(t, both.LengthBound, startOnly.LengthBound)
}
