package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

func TestBlindCutBuildsAnalyticPocket(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)

	pocket, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(pocket))
	require.Len(t, pocket.Lumps(), 1)
	volume, err := pocket.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 936.0, volumeMM(t, volume))
	requireBodyWatertight(t, pocket)
}
