package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func torusArcBody(t *testing.T, doc *decad.Document, sweep float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 4)
	s.Fix(c)
	s.CreateCircle(c, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0],
		decad.ConstructionAxis{Dir: r3.Vec{X: 1}},
		decad.AngleExtent{A: units.Radians(sweep), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func radialStemBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(7), Dir: decad.Along})
	require.NoError(t, err)
	tr, err := r3.Translation(r3.Vec{Y: 4})
	require.NoError(t, err)
	moved, err := decad.Transform(t.Context(), body, tr)
	require.NoError(t, err)
	return moved
}

func TestUnionTorusArcWithStemShortSweep(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arc := torusArcBody(t, doc, math.Pi/9)
	stem := radialStemBody(t, doc)
	out, err := decad.Union(t.Context(), arc, stem)
	require.NoError(t, err)
	vol, err := out.Volume()
	require.NoError(t, err)
	arcVol := math.Pi * 4 * (math.Pi / 9)
	stemVol := 7 * math.Pi
	got := vol.Value.Base()
	// The union holds the whole stem plus the part of the arc it does not cover.
	require.Greater(t, got, stemVol)
	require.Less(t, got, arcVol+stemVol)
}

// stemOutsideTorusArcGrid estimates the stem volume outside one torus arc on
// a midpoint grid. The arc spans angles [0, sweep) around the x axis.
func stemOutsideTorusArcGrid(sweep float64, nr, nphi, ny int) float64 {
	outside := 0.0
	for ir := range nr {
		r := (float64(ir) + 0.5) / float64(nr)
		for ip := range nphi {
			phi := 2 * math.Pi * (float64(ip) + 0.5) / float64(nphi)
			x, z := r*math.Cos(phi), r*math.Sin(phi)
			for iy := range ny {
				y := -3 + 7*(float64(iy)+0.5)/float64(ny)
				ring := math.Hypot(y, z)
				theta := math.Atan2(z, y)
				if theta < 0 {
					theta += 2 * math.Pi
				}
				inside := theta < sweep && math.Hypot(ring-4, x) < 1
				if !inside {
					outside += r
				}
			}
		}
	}
	factor := (1.0 / float64(nr)) * (2 * math.Pi / float64(nphi)) * (7.0 / float64(ny))
	return outside * factor
}

func stemOutsideTorusArcs() [3]float64 {
	const nr, nphi, ny = 120, 240, 280
	outside := stemOutsideTorusArcsGridRaw(nr, nphi, ny)
	for i := range outside {
		outside[i] = outside[i] * (1.0 / nr) * (2 * math.Pi / nphi) * (7.0 / ny)
	}
	return outside
}

func torusArcSweeps() [3]float64 {
	degrees := [3]float64{60, 180, 300}
	var sweeps [3]float64
	for i, deg := range degrees {
		sweeps[i] = deg * math.Pi / 180
	}
	return sweeps
}

func stemOutsideTorusArcsGrid(nr, nphi, ny int) [3]float64 {
	outside := stemOutsideTorusArcsGridRaw(nr, nphi, ny)
	factor := (1.0 / float64(nr)) * (2 * math.Pi / float64(nphi)) * (7.0 / float64(ny))
	for i := range outside {
		outside[i] *= factor
	}
	return outside
}

func stemOutsideTorusArcsGridRaw(nr, nphi, ny int) [3]float64 {
	sweeps := torusArcSweeps()
	outside := [3]float64{}
	for ir := range nr {
		r := (float64(ir) + 0.5) / float64(nr)
		for ip := range nphi {
			phi := 2 * math.Pi * (float64(ip) + 0.5) / float64(nphi)
			x, z := r*math.Cos(phi), r*math.Sin(phi)
			for iy := range ny {
				y := -3 + 7*(float64(iy)+0.5)/float64(ny)
				ring := math.Hypot(y, z)
				theta := math.Atan2(z, y)
				if theta < 0 {
					theta += 2 * math.Pi
				}
				insideTube := math.Hypot(ring-4, x) < 1
				for i, sweep := range sweeps {
					if !(theta < sweep && insideTube) {
						outside[i] += r
					}
				}
			}
		}
	}
	return outside
}

func TestStemOutsideTorusArcsMatchesSingleSweepOracle(t *testing.T) {
	const nr, nphi, ny = 7, 11, 13
	sweeps := torusArcSweeps()
	got := stemOutsideTorusArcsGrid(nr, nphi, ny)
	for i, sweep := range sweeps {
		want := stemOutsideTorusArcGrid(sweep, nr, nphi, ny)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got[i]), "%v degrees", (sweep * 180 / math.Pi))
	}
}

func TestUnionTorusArcWithStemCoarsensItsOperandPastTheCeilings(t *testing.T) {
	t.Parallel()
	wants := stemOutsideTorusArcs()
	for i, deg := range []float64{60, 180, 300} {
		sweep := deg * math.Pi / 180
		doc := decad.New()
		out, err := decad.Union(t.Context(), torusArcBody(t, doc, sweep), radialStemBody(t, doc))
		require.NoError(t, err, "%v degrees", deg)
		vol, err := out.Volume()
		require.NoError(t, err)
		want := math.Pi*4*sweep + wants[i]
		require.InDelta(t, want, vol.Value.Base(), vol.Bound.Base()+0.05, "%v degrees", deg)
		// The coarser operand chord is capped at 8x the pair tolerance, so
		// the volume bound stays a small fraction of the volume.
		require.Less(t, vol.Bound.Base(), 0.02*want, "%v degrees", deg)
	}
}
