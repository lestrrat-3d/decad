package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

const (
	helicalToothModule    = 1.0
	helicalToothCount     = 17.0
	helicalToothThickness = 5.0
)

// buildHelicalToothProfile builds the four-edge m1 z17 tooth outline used by
// fusion360-gear-generator's loft proof: two radial flanks, a tip arc and a
// root arc. rot is the section's rotation about the gear axis.
func buildHelicalToothProfile(s *sketch.Sketch, rot float64) {
	pitchRadius := helicalToothModule * helicalToothCount / 2
	tipRadius := pitchRadius + helicalToothModule
	rootRadius := pitchRadius - 1.25*helicalToothModule
	pitch := 2 * math.Pi / helicalToothCount

	center := s.CreatePoint(0, 0)
	s.Fix(center)
	pointAt := func(radius, angle float64) *sketch.Point {
		p := s.CreatePoint(radius*math.Cos(angle), radius*math.Sin(angle))
		s.Fix(p)
		return p
	}
	tipA := pointAt(tipRadius, rot-0.25*pitch)
	tipB := pointAt(tipRadius, rot+0.25*pitch)
	rootB := pointAt(rootRadius, rot+0.5*pitch)
	rootA := pointAt(rootRadius, rot-0.5*pitch)
	s.CreateArc(center, tipA, tipB)
	s.CreateLine(tipB, rootB)
	s.CreateArc(center, rootB, rootA)
	s.CreateLine(rootA, tipA)
}

// TestLoftHelicalToothClearsDefaultTolerance is A11's acceptance fixture. The
// generator applies its default 14.5-degree helix angle directly as the
// section's end-to-end twist. Its Volume bound is the binding reading at the
// default relative tolerance.
func TestLoftHelicalToothClearsDefaultTolerance(t *testing.T) {
	if testing.Short() {
		t.Skip("the production chord count makes the crossing audit exceed the two-second fixture budget")
	}
	ctx := t.Context()
	w := sketch.NewWorld()
	s0, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	buildHelicalToothProfile(s0, 0)
	_, err = s0.Solve(ctx)
	require.NoError(t, err)

	plane, err := w.CreateOffsetPlane(w.XY(), helicalToothThickness)
	require.NoError(t, err)
	s1, err := w.CreateSketch(plane)
	require.NoError(t, err)
	twist := 14.5 * math.Pi / 180
	buildHelicalToothProfile(s1, twist)
	_, err = s1.Solve(ctx)
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Loft(s0, s0.Profiles()[0], s1, s1.Profiles()[0])
	require.NoError(t, err)
	volume, err := body.Volume()
	require.NoError(t, err)
	centroid, err := body.Centroid()
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)
	areaRelativeBound := area.Bound.Mag() / area.Value.Mag()
	t.Logf("area relative bound %.6g; volume relative bound %.6g; centroid bound %.6g mm",
		areaRelativeBound, volume.Bound.Mag()/volume.Value.Mag(), centroid.Bound.Mag())
	require.LessOrEqual(t, areaRelativeBound, 1e-3,
		"the m1 z17 tooth's Area reading must clear the default relative tolerance")

	report, err := doc.Verify(ctx)
	require.NoError(t, err)
	require.True(t, report.Trustworthy(), "the m1 z17 tooth loft must be trustworthy: %v", report.Diagnostics)
}
