package facetedrecord

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPointConeFactorsMatchSerial(t *testing.T) {
	const n, behind, nan = 1500, 700, 1100
	points := make([]r3.Vec, n)
	for i := range points {
		a := 2 * math.Pi * float64(i) / n
		r := 4 + 2*float64(i%17)/16
		points[i] = r3.Vec{X: 10, Y: r * math.Cos(a), Z: r * math.Sin(a)}
	}
	points[behind] = r3.Vec{X: -1, Y: 1, Z: 1}
	points[nan] = r3.Vec{X: 10, Y: math.NaN(), Z: 1}

	cones := []struct {
		name        string
		apex, slope *big.Rat
	}{
		{"7.5/1", big.NewRat(15, 2), big.NewRat(1, 1)},
		{"12/1", big.NewRat(12, 1), big.NewRat(1, 1)},
		{"3/(1/3)", big.NewRat(3, 1), big.NewRat(1, 3)},
		{"negative apex", big.NewRat(-1, 1), big.NewRat(1, 1)},
	}
	for _, c := range cones {
		t.Run(c.name, func(t *testing.T) {
			newCone := func() *PointConeLimit { return &PointConeLimit{Apex: c.apex, Slope: c.slope} }
			ref := newCone()
			first, firstErr := -1, error(nil)
			for i, p := range points {
				if _, err := PointConeFactor(p, ref); err != nil {
					first, firstErr = i, err
					break
				}
			}
			for _, workers := range []int{1, 4} {
				cone := newCone()
				samples, errs := PointConeFactors(points, cone, workers)
				gotFirst := -1
				for i, err := range errs {
					if err != nil {
						gotFirst = i
						break
					}
				}
				require.Equal(t, first, gotFirst, "workers %d", workers)
				if firstErr != nil {
					require.EqualError(t, errs[gotFirst], firstErr.Error())
				}
				end := n
				if first >= 0 {
					end = first
				}
				for i := range end {
					want, err := PointConeFactor(points[i], ref)
					require.NoError(t, err)
					got := samples[i]
					require.Equal(t, math.Float64bits(want.Q), math.Float64bits(got.Q), "Q %d", i)
					require.Equal(t, math.Float64bits(want.Alpha), math.Float64bits(got.Alpha), "Alpha %d", i)
					require.Equal(t, math.Float64bits(want.QError), math.Float64bits(got.QError), "QError %d", i)
					require.Equal(t, math.Float64bits(want.PointError), math.Float64bits(got.PointError), "PointError %d", i)
				}
			}
		})
	}
}
