package diameter_test

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPointsKeepsExhaustiveWitness(t *testing.T) {
	t.Parallel()
	check := func(name string, points []r3.Vec) {
		t.Helper()
		t.Run(name, func(t *testing.T) {
			want, wantOK := exhaustivePoints(points)
			got, gotOK := diameter.Points(points)
			require.Equal(t, wantOK, gotOK)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got))
			got, gotOK, err := diameter.PointsContext(t.Context(), points)
			require.NoError(t, err)
			require.Equal(t, wantOK, gotOK)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got))
		})
	}

	coil := make([]r3.Vec, 300)
	for i := range coil {
		angle := 10 * math.Pi * float64(i) / float64(len(coil)-1)
		coil[i] = r3.NewVec(3*math.Cos(angle), 3*math.Sin(angle), 7.5*float64(i)/float64(len(coil)-1))
	}
	check("dense coil", coil)
	longCoil := make([]r3.Vec, 600)
	for i := range longCoil {
		angle := 10 * math.Pi * float64(i) / float64(len(longCoil)-1)
		longCoil[i] = r3.NewVec(3*math.Cos(angle), 3*math.Sin(angle), 7.5*float64(i)/float64(len(longCoil)-1))
	}
	check("parallel coil", longCoil)
	clustered := make([]r3.Vec, 768)
	for i := range clustered {
		clustered[i] = r3.NewVec(
			float64(i/256*100)+float64(i%17)/10,
			float64(i%23)/10,
			float64(i%29)/10,
		)
	}
	check("parallel clustered witnesses", clustered)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, err := diameter.PointsContext(ctx, longCoil)
	require.ErrorIs(t, err, context.Canceled)
	invalidLongCoil := append([]r3.Vec(nil), longCoil...)
	invalidLongCoil[len(invalidLongCoil)-1].X = math.NaN()
	check("parallel-size nonfinite", invalidLongCoil)

	spatialRNG := rand.New(rand.NewPCG(0x382b614d, 0x5246d1c9))
	scattered := make([]r3.Vec, 1024)
	for i := range scattered {
		scattered[i] = r3.NewVec(2*spatialRNG.Float64()-1, 2*spatialRNG.Float64()-1,
			2*spatialRNG.Float64()-1)
	}
	check("parallel scattered witnesses", scattered)
	rng := rand.New(rand.NewPCG(0x1246674d, 0x36e3be91))
	for _, scale := range []float64{1e-120, 1, 1e120} {
		for scene := range 12 {
			points := make([]r3.Vec, 48)
			for i := range points {
				points[i] = r3.NewVec(scale*(2*rng.Float64()-1), scale*(2*rng.Float64()-1), scale*(2*rng.Float64()-1))
			}
			check(fmt.Sprintf("random-%g-%d", scale, scene), points)
		}
	}
	// A slender tilted rod prunes badly under a loose skip margin; a few far
	// pairs sit within an ulp-scale distance of each other.
	rod := make([]r3.Vec, 1200)
	for i := range rod {
		s := float64(i) / float64(len(rod)-1)
		rod[i] = r3.NewVec(40*s+1e-3*rng.Float64(), 20*s+1e-3*rng.Float64(), 10*s+1e-3*rng.Float64())
	}
	rod[len(rod)-2] = rod[len(rod)-1].Add(r3.NewVec(1e-12, 0, 0))
	rod[1] = rod[0].Add(r3.NewVec(-1e-12, 0, 0))
	check("parallel slender rod", rod)
	check("duplicate", []r3.Vec{r3.NewVec(1, 2, 3), r3.NewVec(1, 2, 3)})
	check("overflow", []r3.Vec{r3.NewVec(math.MaxFloat64, 0, 0), r3.NewVec(-math.MaxFloat64, 0, 0)})
	check("nonfinite", []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(math.NaN(), 0, 0)})
}

func TestBudgetedBlocksKeepWitnessAndPollOrder(t *testing.T) {
	t.Parallel()
	points := make([]r3.Vec, 576)
	for i := range points {
		points[i] = r3.NewVec(float64(i), float64(i%7), 0)
	}
	want, wantOK := exhaustivePoints(points)
	count := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error { count++; return nil },
		ErrFn:  func() error { return nil },
	}
	got, gotOK, err := diameter.PointsWithBudget(budget, points)
	require.NoError(t, err)
	require.Equal(t, wantOK, gotOK)
	require.Equal(t, math.Float64bits(want), math.Float64bits(got))
	require.Equal(t, len(points)*(len(points)-1)/2, count)

	stop := errors.New("stop")
	stopAt := count / 2
	count = 0
	budget.StepFn = func() error {
		count++
		if count == stopAt {
			return stop
		}
		return nil
	}
	_, _, err = diameter.PointsWithBudget(budget, points)
	require.ErrorIs(t, err, stop)
	require.Equal(t, stopAt, count)
}

func exhaustivePoints(points []r3.Vec) (float64, bool) {
	if len(points) == 0 {
		return 0, false
	}
	best := 0.0
	bestI, bestJ := 0, 0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			distance := points[i].Sub(points[j]).Len()
			if distance < 0 || math.IsNaN(distance) || math.IsInf(distance, 0) {
				return 0, false
			}
			if distance > best {
				best, bestI, bestJ = distance, i, j
			}
		}
	}
	if best == 0 {
		return 0, true
	}
	return diameter.Points([]r3.Vec{points[bestI], points[bestJ]})
}

func TestAxisExtremesContext(t *testing.T) {
	t.Parallel()
	d, ok, err := diameter.AxisExtremesContext(t.Context(),
		[]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 3, Y: 0, Z: 0}, {X: 0, Y: 4, Z: 0}})
	require.NoError(t, err)
	require.True(t, ok)
	require.GreaterOrEqual(t, d, 5-1e-9)
	require.LessOrEqual(t, d, 5.0)

	d, ok, err = diameter.AxisExtremesContext(t.Context(), nil)
	require.NoError(t, err)
	require.False(t, ok)
	require.Zero(t, d)

	_, ok, err = diameter.AxisExtremesContext(t.Context(),
		[]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: math.NaN(), Y: 0, Z: 0}})
	require.NoError(t, err)
	require.False(t, ok)

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err = diameter.AxisExtremesContext(cancelled, []r3.Vec{{X: 1}, {X: 2}})
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
}
