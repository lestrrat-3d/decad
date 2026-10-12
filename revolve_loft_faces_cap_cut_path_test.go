package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelCapCutsFollowLiveOccupiedArcs(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	region, ok, err := certifyRevolveLoftFacesHeelRegion(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for _, arc := range []*revolveLoftFacesToolCapOccupiedArc{region.lower, region.upper} {
		cut, built, cutErr := revolveLoftFacesHeldCapCutPath(t.Context(), proof, arc)
		require.NoError(t, cutErr)
		require.True(t, built)
		require.Len(t, cut.points, 71)
		require.Equal(t, 0, cut.points[0].endpoint)
		require.Equal(t, 1, cut.points[len(cut.points)-1].endpoint)
		seam, seamOK, seamErr := revolveLoftFacesCapArcSeamVertices(t.Context(), proof, arc)
		require.NoError(t, seamErr)
		require.True(t, seamOK)
		require.Equal(t, seam[0].point, cut.points[0].point)
		require.Equal(t, seam[1].point, cut.points[len(cut.points)-1].point)
		maximum := 0.0
		for index, point := range cut.points {
			require.Greater(t, point.ray[0].Sign(), 0)
			require.Less(t, point.sourceBound, 0.1)
			maximum = max(maximum, point.facetBound+point.moveBound+point.sourceBound)
			if index > 0 && index+1 < len(cut.points) {
				require.Equal(t, -1, point.endpoint)
			}
		}
		require.Less(t, maximum, 0.05)
	}
	changed := *region.lower
	changed.time = lfAdd(changed.time, big.NewRat(1, 1<<20))
	cut, built, err := revolveLoftFacesHeldCapCutPath(t.Context(), proof, &changed)
	require.NoError(t, err)
	require.False(t, built)
	require.Nil(t, cut)

	changedCircle := *region.lower
	changedCircle.circle = region.lower.circle
	changedCircle.circle.radius = lfAdd(changedCircle.circle.radius, big.NewRat(1, 1<<20))
	cut, built, err = revolveLoftFacesHeldCapCutPath(t.Context(), proof, &changedCircle)
	require.NoError(t, err)
	require.False(t, built)
	require.Nil(t, cut)

	originalRevision := proof.trim.source.source.revision
	proof.trim.source.source.revision++
	defer func() { proof.trim.source.source.revision = originalRevision }()
	cut, built, err = revolveLoftFacesHeldCapCutPath(t.Context(), proof, region.lower)
	require.NoError(t, err)
	require.False(t, built)
	require.Nil(t, cut)
}

func TestRevolveLoftFacesCapCutIntervalMulSigns(t *testing.T) {
	tests := []struct {
		a, b, want loftFaceInterval
	}{
		{
			a:    loftFaceInterval{big.NewRat(-2, 1), big.NewRat(-1, 1)},
			b:    loftFaceInterval{big.NewRat(3, 1), big.NewRat(4, 1)},
			want: loftFaceInterval{big.NewRat(-8, 1), big.NewRat(-3, 1)},
		},
		{
			a:    loftFaceInterval{big.NewRat(-2, 1), big.NewRat(1, 1)},
			b:    loftFaceInterval{big.NewRat(-3, 1), big.NewRat(4, 1)},
			want: loftFaceInterval{big.NewRat(-8, 1), big.NewRat(6, 1)},
		},
	}
	for _, test := range tests {
		got := revolveLoftFacesCapCutIntervalMul(test.a, test.b)
		require.Equal(t, 0, got.lo.Cmp(test.want.lo))
		require.Equal(t, 0, got.hi.Cmp(test.want.hi))
	}
}

func TestRevolveLoftFacesCapCutFinalRaysRejectRoundedReversal(t *testing.T) {
	one := big.NewRat(1, 1)
	interior := revolveLoftFacesCapCutPoint{ray: [2]*big.Rat{one, one}}
	last := revolveLoftFacesCapCutPoint{ray: [2]*big.Rat{one, big.NewRat(2, 1)}}
	original := revolveLoftFacesCapCutPoint{ray: [2]*big.Rat{
		one, lfF(math.Nextafter(1, 0)),
	}}
	require.True(t, revolveLoftFacesCapCutFinalRaysOrdered(
		[]revolveLoftFacesCapCutPoint{original, interior, last}))

	rounded := original
	rounded.ray[1] = lfF(math.Nextafter(1, math.Inf(1)))
	require.False(t, revolveLoftFacesCapCutFinalRaysOrdered(
		[]revolveLoftFacesCapCutPoint{rounded, interior, last}))
}
