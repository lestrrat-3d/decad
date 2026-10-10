package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftCertifiedRewriteWorkLimitKeepsDefault(t *testing.T) {
	t.Parallel()
	const pairs = uint64(360)
	ordinary := loftmesh.StationWorkLimit(0, pairs)
	require.Equal(t, ordinary, (loftPayload{}).rewriteWorkLimit(pairs))
	trusted := loftPayload{authenticatedReconstruction: true}
	require.Equal(t, 2*ordinary, trusted.rewriteWorkLimit(pairs))
	require.LessOrEqual(t, trusted.rewriteWorkLimit(pairs), uint64(1<<27))

	defaultWork := freeform.NewFreeformWork()
	defaultWork.RaiseLimit((loftPayload{}).rewriteWorkLimit(pairs))
	loftWork := freeform.NewFreeformWork()
	loftWork.RaiseLimit(trusted.rewriteWorkLimit(pairs))
	for range ordinary / freeform.FreeformWorkLimit {
		require.NoError(t, defaultWork.Step(freeform.FreeformWorkLimit))
		require.NoError(t, loftWork.Step(freeform.FreeformWorkLimit))
	}
	require.ErrorIs(t, defaultWork.Step(1), ErrUnsupported)
	require.NoError(t, loftWork.Step(1))
}

func TestCertifyLoftCircleHoleRequiresStrictContainment(t *testing.T) {
	t.Parallel()
	outer := squareLoop(0, 0, 2, true)
	for _, tc := range []struct {
		name   string
		center Point2
		want   bool
	}{
		{name: "inside", center: pt(0, 0), want: true},
		{name: "outside", center: pt(4, 0)},
		{name: "crosses boundary", center: pt(1.75, 0)},
		{name: "touches boundary", center: pt(1.5, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			hole := circleSeg{Center: tc.center, Radius: units.Millimeters(0.5), TStart: 1, TEnd: 0}
			got, err := certifyLoftCircleHole(proofbound.NewWorkBudget(t.Context()), outer, hole, 0,
				freeform.NewFreeformWork())
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
