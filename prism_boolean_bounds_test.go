package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is design A7's own acceptance fixture: the analytic prism
// Union of two circles (docs/prism-boolean-design.md PR1) merges both
// operands' recorded sections through a private sketch scene, which is what
// turns each operand's WHOLE circle into a TRIMMED CircleSeg in the merged
// result — the fragment moments.go's certified brackets used to refuse.
// Before moments_trig.go's fractional-turn arm, this body's Volume and Area
// bounds were loose by orders of magnitude (a bound proportional to the
// body rather than to the actual error); the closed forms below are the
// test oracle, so nothing here is sampled.

// discBody extrudes a radius-r circle centered at (cx, 0) into an h mm
// prism, on the shared plane every fixture using it draws from. Every caller
// centers its own y at 0 — the pair geometry these fixtures need only ever
// varies along one axis — so this helper takes no separate cy. It takes
// testing.TB rather than *testing.T so a benchmark can build the same
// fixture (boolean_test.go's BenchmarkCutCircularWasher).
func discBody(t testing.TB, doc *decad.Document, cx, r, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cx, 0)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// twoCircleUnionClosedForm is the exact closed form for the union of two
// circles of radius bigR (at the origin) and smallR (at distance centerDist
// along +u), swept to height h: the lens-shaped overlap is subtracted once
// from the sum of the two full disks, with the perimeter read off each
// circle's own majority arc.
type twoCircleUnionClosedForm struct {
	region, perim, volume, area float64
}

func twoCircleUnion(centerDist, bigR, smallR, h float64) twoCircleUnionClosedForm {
	alpha := math.Acos((centerDist*centerDist + bigR*bigR - smallR*smallR) / (2 * centerDist * bigR))
	beta := math.Acos((centerDist*centerDist + smallR*smallR - bigR*bigR) / (2 * centerDist * smallR))
	lens := bigR*bigR*alpha + smallR*smallR*beta -
		0.5*math.Sqrt((-centerDist+bigR+smallR)*(centerDist+bigR-smallR)*(centerDist-bigR+smallR)*(centerDist+bigR+smallR))
	region := math.Pi*bigR*bigR + math.Pi*smallR*smallR - lens
	perim := bigR*(2*math.Pi-2*alpha) + smallR*(2*math.Pi-2*beta)
	return twoCircleUnionClosedForm{
		region: region,
		perim:  perim,
		volume: region * h,
		area:   2*region + perim*h,
	}
}

// TestPrismUnionCoplanarCircleLensBounds is the ask's own acceptance case: a
// coplanar analytic union of two circle prisms must report Area/Volume
// bounds small against their values and a Centroid bound small against the
// body's own size.
// Before the fix, the r=10/r=2 row published a Volume bound of 1.002e+04 on
// a value of 2565.675079 (a bound four times the value) and read Suspect at
// the default tolerance; the r=10/r=3 row's Volume and Area were exact to
// the last bit yet carried a bound four times the value, the clearest sign
// the defect was a loose envelope and not a genuine loss of accuracy.
// wantCentroidX is read off the same closed-form disk-minus-lens
// construction, cross-checked independently against a direct run of the
// evaluator before the fix landed (design A7 §1).
func TestPrismUnionCoplanarCircleLensBounds(t *testing.T) {
	for _, tc := range []struct {
		name          string
		bigR, smallR  float64
		centerDist, h float64
		wantCentroidX float64
	}{
		{"r10 union r2 at x=10", 10, 2, 10, 8, 0.220818},
		{"r10 union r3 at x=9", 10, 3, 9, 8, 0.301357},
	} {
		t.Run(tc.name, func(t *testing.T) {
			want := twoCircleUnion(tc.centerDist, tc.bigR, tc.smallR, tc.h)

			doc := decad.New()
			a := discBody(t, doc, 0, tc.bigR, tc.h)
			b := discBody(t, doc, tc.centerDist, tc.smallR, tc.h)
			got, err := decad.Union(a, b)
			require.NoError(t, err)

			vol, err := got.Volume()
			require.NoError(t, err)
			require.InDelta(t, want.volume, volumeMM(t, vol), 1e-6)
			require.LessOrEqualf(t, boundMM3(t, vol), 1e-9*want.volume,
				"volume bound %g mm^3 is not tight against the value %g mm^3", boundMM3(t, vol), want.volume)

			area, err := got.Area()
			require.NoError(t, err)
			require.InDelta(t, want.area, areaMM2(t, area), 1e-6)
			areaBound, err := area.Bound.In(units.SquareMillimeter)
			require.NoError(t, err)
			require.LessOrEqualf(t, areaBound, 1e-9*want.area,
				"area bound %g mm^2 is not tight against the value %g mm^2", areaBound, want.area)

			c, err := got.Centroid()
			require.NoError(t, err)
			require.InDelta(t, tc.wantCentroidX, c.Value.X, 1e-5)
			require.InDelta(t, 0.0, c.Value.Y, 1e-9)
			cBound, err := c.Bound.In(units.Millimeter)
			require.NoError(t, err)
			require.LessOrEqualf(t, cBound, 1e-6, "centroid bound %g mm is not small", cBound)

			// Verify reads the same tightness the assertions above do. The
			// merged section carries §7's cut displacement, so the clearance
			// kernel's exact carrier model declines it (§12) and the tolerance
			// gate falls back to the body's own recorded section for the
			// reference it anchors each reading against (verification design
			// §3). Bounds this far below their own values then pass that gate.
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
			require.True(t, report.Trustworthy())
			require.Empty(t, report.Diagnostics)
		})
	}
}
