package boundarywalk_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestRequireAnalyticWalkRefusesFreeform(t *testing.T) {
	t.Parallel()
	walk := survey2d.SegmentWalk{Kind: survey2d.WalkFreeform}
	err := boundarywalk.RequireAnalyticWalk(walk, "test consumer")
	require.ErrorIs(t, err, boundarywalk.ErrUnsupported)
}
