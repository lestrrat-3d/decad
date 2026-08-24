package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file proves end to end the pair of fixes this branch and its parent
// carry: a fit-spline chain's interior joints no longer fold a
// rounding-noise cross product (docs/spline-design.md §6.5), and a prism
// evaluation now resolves each boundary segment's walk once rather than
// eight times. Together they let a real involute gear-tooth flank, drawn as
// a CreateFitSpline profile, extrude and verify through the public API —
// this used to refuse first R19 (the convexity certificate), then R7 (the
// walk-cost budget), and now does neither.

// involuteFlankSketch builds one involute gear-tooth flank as a fit spline:
// module 1, 17 teeth, 20 degree pressure angle — the gear dialog's own
// defaults. The flank runs from the base circle to the tooth tip, sampled at
// 15 points along the involute parametrization, mirrored across +X per the
// gear spec's step 4.2, and closed back to the origin by two straight lines
// so it bounds a single region.
func involuteFlankSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	const (
		module        = 1.0
		toothNumber   = 17.0
		pressureAngle = 20 * math.Pi / 180
		steps         = 15
	)
	pitchR := module * toothNumber / 2
	baseR := pitchR * math.Cos(pressureAngle)
	tipR := (module*toothNumber + 2*module) / 2

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)

	flank := make([]*sketch.Point, 0, steps)
	for i := range steps {
		r := baseR + (tipR-baseR)*float64(i)/float64(steps-1)
		alpha := math.Acos(baseR / r)
		tt := math.Tan(alpha)
		x := baseR * (math.Cos(tt) + tt*math.Sin(tt))
		y := baseR * (math.Sin(tt) - tt*math.Cos(tt))
		p := s.CreatePoint(x, -y) // mirrored across +X, per the gear spec's step 4.2
		s.Fix(p)
		flank = append(flank, p)
	}
	_, err = s.CreateFitSpline(flank...)
	require.NoError(t, err)
	s.CreateLine(flank[len(flank)-1], o)
	s.CreateLine(o, flank[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestExtrudeInvoluteFlankBuildsAndVerifies is the end-to-end proof: the
// involute flank fixture extrudes without error (the whole point — on the
// unfixed evaluator this refused, first R19, then R7), publishes the volume,
// centroid and bounds this branch measured through this exact public path,
// and reports Sound under Verify.
//
// The comparison tolerance is 1e-9, NOT the published Bound. Every input
// coordinate above is built from expressions of the shape a*b + c
// (math.Cos(tt) + tt*math.Sin(tt)), which Go permits the compiler to fuse
// into a single rounding step on some architectures (arm64) and not others
// (amd64). That makes the sampled points themselves — and so every
// measurement derived from them — differ by roughly an ulp between CI hosts.
// The published Bound (on the order of 1e-15) is far tighter than that
// cross-host input difference and would make this test flaky on macOS CI;
// 1e-9 still pins the geometry to ten significant figures while tolerating
// the platform-dependent FMA contraction. Do not tighten this without
// re-reading this comment.
const involuteCompareTolerance = 1e-9

func TestExtrudeInvoluteFlankBuildsAndVerifies(t *testing.T) {
	s, p := involuteFlankSketch(t)

	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err, "a fit-spline involute flank must build: the R19 and R7 refusals this branch fixes")
	require.NotNil(t, body)

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.InDelta(t, 28.386587733941017, vol.Value.Mag(), involuteCompareTolerance)
	// The published Bound moves with the host too, and for the same reason:
	// it is derived from the actual float coordinates, so FMA-contracted
	// inputs give a slightly different bound (measured 1.14e-15 on amd64 and
	// 2.11e-15 on arm64). What is worth asserting is that the bound stays
	// vanishingly small against a ~28 mm^3 volume — a relative error under
	// 1e-13 — not which of those two figures this host produces.
	require.Less(t, vol.Bound.Mag(), 1e-12,
		"the volume's proven error must stay negligible against the volume itself")

	area, err := body.Area()
	require.NoError(t, err)
	require.InDelta(t, 197.10903776598994, area.Value.Mag(), involuteCompareTolerance)

	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5.939041425788393, c.Value.X, involuteCompareTolerance)
	require.InDelta(t, -0.22965573060319747, c.Value.Y, involuteCompareTolerance)
	require.InDelta(t, 5.0, c.Value.Z, involuteCompareTolerance)

	box, err := body.Bounds()
	require.NoError(t, err)
	// Exactness is read off the box's own Bound, which is derived from the
	// same host-dependent coordinates, so assert the bound is negligible
	// rather than pinning which Exactness tier this host lands in.
	require.Less(t, box.Bound.Mag(), 1e-12,
		"the box's proven error must stay negligible against the body's own extent")
	require.InDelta(t, 0.0, box.Min.X, involuteCompareTolerance)
	require.InDelta(t, -0.6817636915234364, box.Min.Y, involuteCompareTolerance)
	require.InDelta(t, 0.0, box.Min.Z, involuteCompareTolerance)
	require.InDelta(t, 9.475505172228038, box.Max.X, involuteCompareTolerance)
	require.InDelta(t, 0.0, box.Max.Y, involuteCompareTolerance)
	require.InDelta(t, 10.0, box.Max.Z, involuteCompareTolerance)

	// The flank wall is free-form: exactly one built face carries the fit
	// spline's surface, tagged KindNURBS.
	wall := freeformWallFace(t, body)
	require.Equal(t, decad.KindNURBS, wall.Surface().Kind())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}
