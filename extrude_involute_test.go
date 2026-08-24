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
	require.LessOrEqual(t, vol.Bound.Mag(), 1.2e-15,
		"the published Bound is decad's own arithmetic error, independent of host-specific input noise")

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
	require.Equal(t, decad.Exact, box.Exactness)
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
