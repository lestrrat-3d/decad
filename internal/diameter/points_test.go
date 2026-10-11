package diameter_test

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/diameter"
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
		})
	}

	coil := make([]r3.Vec, 300)
	for i := range coil {
		angle := 10 * math.Pi * float64(i) / float64(len(coil)-1)
		coil[i] = r3.NewVec(3*math.Cos(angle), 3*math.Sin(angle), 7.5*float64(i)/float64(len(coil)-1))
	}
	check("dense coil", coil)

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
	check("duplicate", []r3.Vec{r3.NewVec(1, 2, 3), r3.NewVec(1, 2, 3)})
	check("overflow", []r3.Vec{r3.NewVec(math.MaxFloat64, 0, 0), r3.NewVec(-math.MaxFloat64, 0, 0)})
	check("nonfinite", []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(math.NaN(), 0, 0)})
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
