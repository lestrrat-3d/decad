package momentinput

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestCoordinateEnvelopeReadsCircularWalksTightly pins CoordinateEnvelope's
// circular reading, |cu| + |cv| + √2·R: on a slot whose semicircles of radius
// 5 sit at u = ±15, and on a circle of radius 10 about (3, −4), the envelope
// covers the largest |u| + |v| a dense sample of the denoted curves reaches
// and stays within 1e-12 of the closed form, 15 + 5√2 and 7 + 10√2.
//
// Shown to fail first: read through the walks' own CoordUpper, the slot's
// envelope was 85 (an ArcSeg's radiusUpper is its coordinates' L1 sizes) and
// the circle's 27, both past the closed form.
func TestCoordinateEnvelopeReadsCircularWalksTightly(t *testing.T) {
	t.Parallel()
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	slot := Profile{Outer: LoopRecord{Segments: []CurveSegment{
		ArcSeg{Center: p(-15, 0), Start: p(-15, 5), End: p(-15, -5), TStart: 0, TEnd: 1},
		LineSeg{Start: p(-15, -5), End: p(15, -5), TStart: 0, TEnd: 1},
		ArcSeg{Center: p(15, 0), Start: p(15, -5), End: p(15, 5), TStart: 0, TEnd: 1},
		LineSeg{Start: p(15, 5), End: p(-15, 5), TStart: 0, TEnd: 1},
	}}}
	circle := Profile{Outer: LoopRecord{Segments: []CurveSegment{
		CircleSeg{Center: p(3, -4), Radius: units.Millimeters(10), CCW: true, TStart: 0, TEnd: 1},
	}}}
	for _, tc := range []struct {
		name    string
		profile Profile
		closed  float64
		sample  func(th float64) float64
	}{
		{"slot", slot, 15 + 5*math.Sqrt2, func(th float64) float64 {
			return math.Abs(15+5*math.Cos(th)) + math.Abs(5*math.Sin(th))
		}},
		{"circle", circle, 7 + 10*math.Sqrt2, func(th float64) float64 {
			return math.Abs(3+10*math.Cos(th)) + math.Abs(-4+10*math.Sin(th))
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := CoordinateEnvelope(tc.profile, freeform.NewFreeformWork(), nil)
			require.NoError(t, err)
			worst := 0.0
			for i := range 4096 {
				worst = math.Max(worst, tc.sample(2*math.Pi*float64(i)/4096))
			}
			require.LessOrEqual(t, worst, got, "the envelope sits below a point of the denoted curve")
			require.InDelta(t, tc.closed, got, 1e-12)
		})
	}
}

// TestRegionEnvelopeReadsArcsTightly pins the region integrator's coordinate
// envelope for a recorded arc (momentregion's circularL1Upper): F3's right
// semicircle, radius 5 about (15, 0), read against the origin and against an
// anchor at its centre, covers a dense sample of |u − a| + |v| and sits
// within 1e-12 of the closed forms 15 + 5√2 and 5√2.
//
// Shown to fail first: read through the integrator's own |cu| + |cv| +
// 2·radiusUpper, whose radiusUpper is the coordinates' L1 sizes, the two
// envelopes were 85 and 10.
func TestRegionEnvelopeReadsArcsTightly(t *testing.T) {
	t.Parallel()
	arc := ArcSeg{Center: sectionrecord.Point2{U: 15}, Start: sectionrecord.Point2{U: 15, V: -5}, End: sectionrecord.Point2{U: 15, V: 5}, TStart: 0, TEnd: 1}
	for _, tc := range []struct {
		anchor float64
		closed float64
	}{{0, 15 + 5*math.Sqrt2}, {15, 5 * math.Sqrt2}} {
		var ig Integrals
		require.NoError(t, ig.add(arc, Plan{}, sectionrecord.Point2{U: tc.anchor}, freeform.MomentFirstOrder))
		worst := 0.0
		for i := range 4096 {
			th := -math.Pi/2 + math.Pi*float64(i)/4095
			worst = math.Max(worst, math.Abs(15+5*math.Cos(th)-tc.anchor)+math.Abs(5*math.Sin(th)))
		}
		require.LessOrEqual(t, worst, ig.CoordUpper, "anchor %g", tc.anchor)
		require.InDelta(t, tc.closed, ig.CoordUpper, 1e-12, "anchor %g", tc.anchor)
	}
}
