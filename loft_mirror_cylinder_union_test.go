package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftMirrorCylinderLevelsRequireExactMidpoint(t *testing.T) {
	t.Parallel()
	require.True(t, loftMirrorCylinderLevels(0, 1, 2))
	require.True(t, loftMirrorCylinderLevels(-3, -1, 1))
	require.False(t, loftMirrorCylinderLevels(0, math.Nextafter(1, 2), 2))
	require.False(t, loftMirrorCylinderLevels(0, 1, math.Nextafter(2, 3)))
	require.False(t, loftMirrorCylinderLevels(0, math.NaN(), 2))
}

func TestLoftMirrorCylinderCapsMustTileWithOppositeWinding(t *testing.T) {
	t.Parallel()
	lower := facetedPayload{
		tris: [][3]int{{0, 1, 2}, {0, 2, 3}}, src: []int{4, 4},
	}
	upper := facetedPayload{
		tris: [][3]int{{2, 1, 0}, {3, 2, 0}}, src: []int{4, 4},
	}
	require.True(t, loftMirrorCylinderCapsOppose(lower, upper, []int{0, 1, 2, 3}, 4, 4))
	upper.tris[1] = [3]int{0, 2, 3}
	require.False(t, loftMirrorCylinderCapsOppose(lower, upper, []int{0, 1, 2, 3}, 4, 4))
	upper.tris[1] = [3]int{3, 2, 0}
	require.False(t, loftMirrorCylinderCapsOppose(lower, upper, []int{0, 1, 2, 4}, 4, 4))
	upper.src[1] = 3
	require.False(t, loftMirrorCylinderCapsOppose(lower, upper, []int{0, 1, 2, 3}, 4, 4))
}

func TestLoftMirrorCylinderPairKeepsAuthenticatedSource(t *testing.T) {
	t.Parallel()
	source := &loftMirrorJoinRecord{}
	other := &loftMirrorJoinRecord{}
	identity := r3.Identity()
	motion, ok := loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorJoin: source, xform: identity,
	}, source)
	require.True(t, ok)
	require.Equal(t, identity, motion)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorJoin: other, xform: identity,
	}, source)
	require.False(t, ok)

	rotated, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/4))
	require.NoError(t, err)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorCopy: &loftMirrorPlacementRecord{source: source, motion: rotated},
		xform:          rotated,
	}, source)
	require.True(t, ok)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorCopy: &loftMirrorPlacementRecord{source: other, motion: rotated},
		xform:          rotated,
	}, source)
	require.False(t, ok)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorCopy: &loftMirrorPlacementRecord{source: source, motion: rotated},
		xform:          identity,
	}, source)
	require.False(t, ok)
	shifted, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorCopy: &loftMirrorPlacementRecord{source: source, motion: shifted},
		xform:          shifted,
	}, source)
	require.False(t, ok)
}
