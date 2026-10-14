package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// placedCapCentroidBy maps the true centroid by the exact rotation and charges
// only the single rounding of the reported point: the image of the held value
// lies within the reported bound of the reported point, and the bound adds no
// stretch term to the source bound.
func TestPlacedCapCentroidByExactRotation(t *testing.T) {
	source := VecMeasurement{Value: r3.Vec{X: 1.5, Y: -2, Z: 0.25}, Bound: units.Millimeters(0.001)}
	for _, tc := range []struct {
		name         string
		center, axis r3.Vec
		theta        float64
	}{
		{"offset oblique axis", r3.Vec{X: 2, Y: -1, Z: 3}, r3.Vec{X: 1, Y: 2, Z: 2}, 1.1},
		{"X axis through origin", r3.Vec{}, r3.Vec{X: 1}, 2 * math.Pi * 23 / 31},
		{"offset Z axis", r3.Vec{X: 5, Y: 7}, r3.Vec{Z: 1}, math.Pi / 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rec, d, err := exactrotation.New(tc.center, tc.axis, units.Radians(tc.theta))
			require.NoError(t, err)
			got, err := placedCapCentroidBy(source, rec.Linear(), rec.Translation())
			require.NoError(t, err)

			// The exact image Q*c + t, computed here from the record's own
			// accessors.
			m, tr := rec.Linear(), rec.Translation()
			c := [3]*big.Rat{ratOfPointPlane(source.Value.X), ratOfPointPlane(source.Value.Y), ratOfPointPlane(source.Value.Z)}
			var image [3]*big.Rat
			dist2 := new(big.Rat)
			for i := range image {
				image[i] = new(big.Rat).Set(tr[i])
				for j := range c {
					image[i].Add(image[i], new(big.Rat).Mul(m[i][j], c[j]))
				}
			}
			held := [3]*big.Rat{ratOfPointPlane(got.Value.X), ratOfPointPlane(got.Value.Y), ratOfPointPlane(got.Value.Z)}
			for i := range image {
				diff := new(big.Rat).Sub(image[i], held[i])
				dist2.Add(dist2, diff.Mul(diff, diff))
			}
			// Only the point's own rounding is charged on top of the source
			// bound, so the reported point sits far inside the reported bound.
			extra := got.Bound.Base() - source.Bound.Base()
			require.GreaterOrEqual(t, extra, 0.0)
			require.Less(t, extra, 1e-12)
			extraRat := ratOfPointPlane(extra)
			require.LessOrEqual(t, dist2.Cmp(extraRat.Mul(extraRat, extraRat)), 0,
				"the rounded point is within the charged rounding of Q*c + t")

			// An independent reading: r3's float placement agrees to rounding.
			want := d.Apply(source.Value)
			require.InDelta(t, want.X, got.Value.X, 1e-12)
			require.InDelta(t, want.Y, got.Value.Y, 1e-12)
			require.InDelta(t, want.Z, got.Value.Z, 1e-12)
		})
	}
}

// placedCapCentroid is placedCapCentroidBy under the placement's own float
// basis and translation, read as exact rationals.
func TestPlacedCapCentroidReadsPlacementAsExactMap(t *testing.T) {
	source := VecMeasurement{Value: r3.Vec{X: 1.5, Y: -2, Z: 0.25}, Bound: units.Millimeters(0.001)}
	d, err := r3.RotationAround(r3.Vec{X: 2, Y: -1, Z: 3}, r3.Vec{X: 1, Y: 2, Z: 2}, units.Radians(1.1))
	require.NoError(t, err)
	linear, err := massmoment.PlacementRotation(d)
	require.NoError(t, err)
	tr := d.Translation()
	want, err := placedCapCentroidBy(source, linear,
		[3]*big.Rat{ratOfPointPlane(tr.X), ratOfPointPlane(tr.Y), ratOfPointPlane(tr.Z)})
	require.NoError(t, err)
	got, err := placedCapCentroid(source, d)
	require.NoError(t, err)
	require.Equal(t, math.Float64bits(want.Value.X), math.Float64bits(got.Value.X))
	require.Equal(t, math.Float64bits(want.Value.Y), math.Float64bits(got.Value.Y))
	require.Equal(t, math.Float64bits(want.Value.Z), math.Float64bits(got.Value.Z))
	require.Equal(t, math.Float64bits(want.Bound.Base()), math.Float64bits(got.Bound.Base()))
}
