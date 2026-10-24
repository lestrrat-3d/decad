package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type edgeModifyCall func(context.Context, *decad.Body, decad.EdgeSelector, units.Value) (*decad.Body, error)

func testIndependentBossRootRefusal(t *testing.T, call edgeModifyCall) {
	t.Helper()
	t.Parallel()
	joined := squareBossOnPlate(t)
	roots := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(-5, -5, 5)))
	roots.Or(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(-5, 5, 5)))
	got, err := call(t.Context(), joined, roots.Exactly(2), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "interior material wedge")
	require.Nil(t, got)
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Equal(t, 2500.0, volume.Value.Base())
}
