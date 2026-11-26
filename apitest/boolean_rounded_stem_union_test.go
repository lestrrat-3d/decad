package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// roundedStem extrudes a w by t rectangle about the Z axis, its corners
// rounded by round (none when zero), from z0 up by h.
func roundedStem(t *testing.T, doc *decad.Document, w, th, round, z0, h float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	f, err := r3.NewFrame(r3.Vec{Z: z0}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	hw, ht := w/2, th/2
	p := [4]*sketch.Point{s.CreatePoint(-hw, -ht), s.CreatePoint(hw, -ht), s.CreatePoint(hw, ht), s.CreatePoint(-hw, ht)}
	var e [4]*sketch.Line
	for i := range p {
		s.Fix(p[i])
	}
	for i := range p {
		e[i] = s.CreateLine(p[i], p[(i+1)%4])
	}
	if round > 0 {
		for i := range p {
			_, err := sketch.CreateFillet(s, e[(i+3)%4], e[i], p[i], round)
			require.NoError(t, err)
		}
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// flushArm extrudes poly, drawn in the plane y = y0 with U = +X and V = +Z,
// by depth along +Y.
func flushArm(t *testing.T, doc *decad.Document, y0, depth float64, poly [][2]float64) *decad.Body {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{Y: y0}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	return framePrism(t, doc, f, depth, decad.Against, poly)
}

// TestUnionRoundedStemWithFlushArm joins a stem whose vertical corners carry
// 0.3 mm or 0.8 mm rounds to an arm whose side faces are flush with the stem's flat
// sides. The arm's square corners enclose the stem's rounds over the arm's
// height, so each round's cylinder touches the arm's side planes along its
// tangent lines while its chord facets run deep inside the arm. On the arm's
// thin end, 0.5 mm tall at its tip, that depth lies only across the facets from the
// tangent line, where the union must still find it.
//
// The sharp stem's union is 7 x 3.6 x 18 for the stem plus 108.5 x 3.6 for the
// arm, less their 12.875 x 3.6 overlap. A round takes the corner area
// A = r²(1 - π/4) from the stem wherever the arm does not cover it: the right
// corners lose it over 18 - 2 mm, and the left corners, under the arm's 45
// degree ramp, over 18 - 0.5 - s at distance s from the left face, which sums
// to (17.5 - r)A + r³/6.
func TestUnionRoundedStemWithFlushArm(t *testing.T) {
	t.Parallel()
	const (
		w, th, z0, top = 7.0, 3.6, 42.5, 60.5
		sharpVolume    = w*th*(top-z0) + 108.5*th - 12.875*th
	)
	poly := [][2]float64{{-3.5, 58.5}, {3.5, 58.5}, {3.5, 60.0}, {19.0, 75.5}, {19.0, 76.5}, {14.0, 76.5}, {-3.5, 59.0}}
	rounded := func(r float64) float64 {
		a := r * r * (1 - math.Pi/4)
		return sharpVolume - 2*((17.5-r)*a+r*r*r/6+16*a)
	}
	testcases := []struct {
		Name   string
		Round  float64
		Volume float64
	}{
		{Name: "sharp", Round: 0, Volume: sharpVolume},
		{Name: "rounded 0.3", Round: 0.3, Volume: rounded(0.3)},
		{Name: "rounded 0.8", Round: 0.8, Volume: rounded(0.8)},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			stem := roundedStem(t, doc, w, th, tc.Round, z0, top-z0)
			arm := flushArm(t, doc, -th/2, th, poly)
			got, err := decad.Union(stem, arm).Do(t.Context())
			require.NoError(t, err)
			require.True(t, got.IsSolid())
			require.Len(t, got.Lumps(), 1)
			decadtest.MeasuresVolume(t, got, units.CubicMillimeters(tc.Volume), decadtest.WithinRel(units.Scalar(1e-9)))
		})
	}
}
