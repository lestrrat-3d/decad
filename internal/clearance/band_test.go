package clearance_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// square is the region of the square [-h, h]², walked counter-clockwise, with
// an optional circular hole of radius hole about the origin.
func square(t *testing.T, h, hole float64) clearance.Region2 {
	t.Helper()
	var elems []survey2d.SurveyElem
	corners := [][2]float64{{-h, -h}, {h, -h}, {h, h}, {-h, h}}
	for i := range corners {
		a, b := corners[i], corners[(i+1)%len(corners)]
		e, ok := survey2d.LineElem(a[0], a[1], b[0], b[1])
		require.True(t, ok)
		elems = append(elems, e)
	}
	if hole > 0 {
		e, ok := survey2d.ArcElem(0, 0, hole, 2*math.Pi, 0, true)
		require.True(t, ok)
		elems = append(elems, e)
	}
	return clearance.NewRegion2(elems)
}

// disk is the region of the disk of radius r about the origin.
func disk(t *testing.T, r float64) clearance.Region2 {
	t.Helper()
	e, ok := survey2d.ArcElem(0, 0, r, 0, 2*math.Pi, true)
	require.True(t, ok)
	return clearance.NewRegion2([]survey2d.SurveyElem{e})
}

// TestAnnulusRegionMiss pins the annulus exclusion against a disk, a hole and
// a rectangle: an annulus wholly outside the region misses it, and one inside
// it, straddling a boundary, or closer to a boundary than its half width plus
// the margin does not.
func TestAnnulusRegionMiss(t *testing.T) {
	t.Parallel()
	const margin = 1e-9
	for _, c := range []struct {
		name         string
		region       clearance.Region2
		cx, cy       float64
		rIn, rOut    float64
		margin, want float64
	}{
		{"annulus inside a hole", square(t, 10, 3.2), 0, 0, 3, 3.0001, margin, 1},
		{"annulus straddling a hole", square(t, 10, 3.2), 0, 0, 3.1, 3.3, margin, 0},
		{"annulus on a hole's rim", square(t, 10, 3), 0, 0, 3, 3.0001, margin, 0},
		{"annulus inside the material", square(t, 10, 3.2), 0, 0, 4, 5, margin, 0},
		{"annulus beyond a square", square(t, 10, 0), 40, 0, 1, 2, margin, 1},
		{"annulus inside a square", square(t, 10, 0), 0, 0, 1, 2, margin, 0},
		{"annulus around a square", square(t, 1, 0), 0, 0, 5, 6, margin, 1},
		{"annulus straddling a square's side", square(t, 10, 0), 9, 0, 0.5, 2, margin, 0},
		{"annulus around a disk", disk(t, 3), 0, 0, 4, 5, margin, 1},
		{"annulus inside a disk", disk(t, 3), 0, 0, 1, 2, margin, 0},
		{"annulus straddling a disk", disk(t, 3), 0, 0, 2, 4, margin, 0},
		{"margin covers the gap", square(t, 10, 3.2), 0, 0, 3, 3.1, 0.2, 0},
		{"margin leaves the gap", square(t, 10, 3.2), 0, 0, 3, 3.1, 0.05, 1},
		{"an inverted annulus", square(t, 10, 3.2), 0, 0, 3.1, 3, margin, 0},
		{"a non-finite centre", square(t, 10, 3.2), math.NaN(), 0, 3, 3.1, margin, 0},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			got := clearance.AnnulusRegionMiss(c.region, c.cx, c.cy, c.rIn, c.rOut, c.margin)
			require.Equal(t, c.want == 1, got)
		})
	}
}

// TestTiltBounds pins the tilt sine and its cosine floor: the sine of two
// equal directions reads only the rounding allowance, the cosine floor of a
// 0.05 rad tilt sits at or below cos 0.05 within 1e-12, and a perpendicular
// pair reads a sine of 1 and a cosine floor of 0.
func TestTiltBounds(t *testing.T) {
	t.Parallel()
	z := r3.NewVec(0, 0, 1)
	require.Less(t, clearance.TiltSine(z, z), 1e-13)
	require.LessOrEqual(t, clearance.TiltCosine(clearance.TiltSine(z, z)), 1.0)
	require.InDelta(t, 1.0, clearance.TiltCosine(clearance.TiltSine(z, z)), 1e-15)
	tilted := r3.NewVec(0, math.Sin(0.05), math.Cos(0.05))
	s := clearance.TiltSine(z, tilted)
	require.GreaterOrEqual(t, s, math.Sin(0.05))
	require.InDelta(t, math.Sin(0.05), s, 1e-12)
	cos := clearance.TiltCosine(s)
	require.LessOrEqual(t, cos, math.Cos(0.05))
	require.InDelta(t, math.Cos(0.05), cos, 1e-12)
	require.Equal(t, 1.0, clearance.TiltSine(z, r3.NewVec(1, 0, 0)))
	require.Equal(t, 0.0, clearance.TiltCosine(1))
}
