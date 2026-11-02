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

// stemOutsideTorusArc is the exact volume of the radial stem (radius 1, y from
// -3 to 4) that lies outside a torus arc of the given sweep, by midpoint
// integration over the stem. The arc's tube (radius 1, ring radius 4) spans
// angles [0, sweep) about the x axis, measured from +y toward +z.
func stemOutsideTorusArc(sweep float64) float64 {
	const nr, nphi, ny = 120, 240, 280
	outside := 0.0
	for ir := range nr {
		r := (float64(ir) + 0.5) / nr
		for ip := range nphi {
			phi := 2 * math.Pi * (float64(ip) + 0.5) / nphi
			x, z := r*math.Cos(phi), r*math.Sin(phi)
			for iy := range ny {
				y := -3 + 7*(float64(iy)+0.5)/ny
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
	return outside * (1.0 / nr) * (2 * math.Pi / nphi) * (7.0 / ny)
}

func TestUnionTorusArcWithStemCoarsensItsOperandPastTheCeilings(t *testing.T) {
	t.Parallel()
	for _, deg := range []float64{60, 180, 300} {
		sweep := deg * math.Pi / 180
		doc := decad.New()
		out, err := decad.Union(t.Context(), torusArcBody(t, doc, sweep), radialStemBody(t, doc))
		require.NoError(t, err, "%v degrees", deg)
		vol, err := out.Volume()
		require.NoError(t, err)
		want := math.Pi*4*sweep + stemOutsideTorusArc(sweep)
		require.InDelta(t, want, vol.Value.Base(), vol.Bound.Base()+0.05, "%v degrees", deg)
		// The coarser operand chord is capped at 8x the pair tolerance, so
		// the volume bound stays a small fraction of the volume.
		require.Less(t, vol.Bound.Base(), 0.02*want, "%v degrees", deg)
	}
}
