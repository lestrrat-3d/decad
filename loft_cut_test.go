package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

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
			got, err := certifyLoftCircleHole(proofbound.NewWorkBudget(t.Context()), outer, hole, freeform.NewFreeformWork())
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
