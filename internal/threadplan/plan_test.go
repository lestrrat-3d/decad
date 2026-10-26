package threadplan_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/threadplan"
	"github.com/stretchr/testify/require"
)

func TestComputeExternalM10(t *testing.T) {
	t.Parallel()
	plan, err := threadplan.Compute(5, 1.5, 20, true, false)
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
	plan, err := threadplan.Compute(4.5, 1.5, 12, false, false)
	require.NoError(t, err)
	mouth, root := plan.Profile[0][0], plan.Profile[1][0]
	require.Less(t, mouth, 4.5, "the mouth opens into the bore")
	require.InDelta(t, 4.5+0.54127*1.5, root, 1e-4, "the root lies deeper into the wall")
	require.InDelta(t, 1.5/4, plan.Profile[2][1]-plan.Profile[1][1], 1e-12, "an internal root flat is a quarter of the pitch")
}

func TestComputeTurnsCarryTheGrooveClearOfBothEnds(t *testing.T) {
	t.Parallel()
	for _, external := range []bool{true, false} {
		plan, err := threadplan.Compute(5, 1.5, 20, external, false)
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
		plan, err := threadplan.Compute(8, 2, 10, external, false)
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
			_, err := threadplan.Compute(a[0].(float64), a[1].(float64), a[2].(float64), a[3].(bool), false)
			require.Error(t, err)
		})
	}
}

func TestComputePartialEndsTheLastGrooveAtTheLength(t *testing.T) {
	t.Parallel()
	const pitch, length = 1.5, 8.0
	for _, external := range []bool{true, false} {
		plan, err := threadplan.Compute(5, pitch, length, external, true)
		require.NoError(t, err)
		lowest, highest := math.Inf(1), math.Inf(-1)
		for _, c := range plan.Profile {
			lowest, highest = math.Min(lowest, c[1]), math.Max(highest, c[1])
		}
		require.Less(t, highest, 0.0, "the first groove still starts wholly before the start plane")
		// The last groove has advanced Turns·pitch, and its far flank sits at the length.
		require.InDelta(t, length, highest+plan.Turns*pitch, 1e-12, "the last groove's far flank lands on the length")
		full, err := threadplan.Compute(5, pitch, length, external, false)
		require.NoError(t, err)
		require.Less(t, plan.Turns, full.Turns, "a partial thread is shorter than one that runs out")
	}
}

func TestComputeFormUsesItsSlopeDepthAndFlat(t *testing.T) {
	t.Parallel()
	// Slope 0.25, depth 1, flat 0.5 on a 5 mm rod at a 2 mm pitch: the root is at 4, the
	// mouth at 5.1, and each flank widens by 0.25 per unit of depth.
	plan, err := threadplan.ComputeForm(5, 2, 20, true, false, threadplan.Form{Slope: 0.25, Depth: 1, Flat: 0.5})
	require.NoError(t, err)
	p := plan.Profile
	require.InDelta(t, 4, p[0][0], 1e-12)
	require.InDelta(t, 5.1, p[2][0], 1e-12)
	require.InDelta(t, 0.5, p[1][1]-p[0][1], 1e-12, "the root flat")
	require.InDelta(t, 0.5+2*0.25*1.1, p[2][1]-p[3][1], 1e-12, "the mouth")
	zero, err := threadplan.ComputeForm(5, 2, 20, true, false, threadplan.Form{})
	require.NoError(t, err)
	iso, err := threadplan.Compute(5, 2, 20, true, false)
	require.NoError(t, err)
	require.Equal(t, iso, zero)
	_, err = threadplan.ComputeForm(5, 2, 20, true, false, threadplan.Form{Slope: 0.25, Depth: 1})
	require.ErrorIs(t, err, decaderr.ErrDegenerate, "a form with no flat")
}
