package threadplan_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/threadplan"
	"github.com/stretchr/testify/require"
)

func TestComputeExternalM10(t *testing.T) {
	t.Parallel()
	plan, err := threadplan.Compute(5, 1.5, 20, true)
	require.NoError(t, err)
	root, mouth := plan.Profile[0][0], plan.Profile[3][0]
	require.InDelta(t, 5-0.61343*1.5, root, 1e-4, "the root sits h3 under the major radius")
	require.InDelta(t, 5+0.05*1.5, mouth, 1e-12, "the mouth opens a twentieth of a pitch past the surface")
	// The root flat is an eighth of the pitch and the flanks are 30 degrees off radial.
	require.InDelta(t, 1.5/8, plan.Profile[1][1]-plan.Profile[0][1], 1e-12)
	flank := plan.Profile[2][1] - plan.Profile[1][1]
	require.InDelta(t, (mouth-root)*math.Tan(math.Pi/6), flank, 1e-12)
}

func TestComputeInternalReachesPastTheBore(t *testing.T) {
	t.Parallel()
	plan, err := threadplan.Compute(4.5, 1.5, 12, false)
	require.NoError(t, err)
	mouth, root := plan.Profile[0][0], plan.Profile[1][0]
	require.Less(t, mouth, 4.5, "the mouth opens into the bore")
	require.InDelta(t, 4.5+0.54127*1.5, root, 1e-4, "the root lies deeper into the wall")
	require.InDelta(t, 1.5/4, plan.Profile[2][1]-plan.Profile[1][1], 1e-12, "an internal root flat is a quarter of the pitch")
}

func TestComputeTurnsCarryTheGrooveClearOfBothEnds(t *testing.T) {
	t.Parallel()
	for _, external := range []bool{true, false} {
		plan, err := threadplan.Compute(5, 1.5, 20, external)
		require.NoError(t, err)
		lowest, highest := math.Inf(1), math.Inf(-1)
		for _, c := range plan.Profile {
			lowest, highest = math.Min(lowest, c[1]), math.Max(highest, c[1])
		}
		require.Less(t, highest, 0.0, "the first groove lies wholly before the start plane")
		last := plan.Turns * 1.5 // the axial advance of the last groove's centre
		first := (lowest + highest) / 2
		require.Greater(t, first+last-(highest-lowest)/2, 20.0, "the last groove lies wholly past the far end")
	}
}

func TestComputeProfileStaysNarrowerThanThePitch(t *testing.T) {
	t.Parallel()
	for _, external := range []bool{true, false} {
		plan, err := threadplan.Compute(8, 2, 10, external)
		require.NoError(t, err)
		lowest, highest := math.Inf(1), math.Inf(-1)
		for _, c := range plan.Profile {
			lowest, highest = math.Min(lowest, c[1]), math.Max(highest, c[1])
		}
		require.Less(t, highest-lowest, 2.0)
	}
}

func TestComputeRefusals(t *testing.T) {
	t.Parallel()
	cases := map[string][4]any{
		"coarse pitch reaches the axis": {1.0, 2.0, 10.0, true},
		"zero pitch":                    {5.0, 0.0, 10.0, true},
		"zero length":                   {5.0, 1.0, 0.0, true},
		"nan radius":                    {math.NaN(), 1.0, 10.0, true},
		"infinite length":               {5.0, 1.0, math.Inf(1), false},
	}
	for name, a := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			_, err := threadplan.Compute(a[0].(float64), a[1].(float64), a[2].(float64), a[3].(bool))
			require.Error(t, err)
		})
	}
}
