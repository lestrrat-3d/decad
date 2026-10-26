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

// This file is agent-docs/fillet-chord-design.md §6's and chamfer-angle-design.md §4's
// public-surface tests for a circular edge between a plane and a cylinder: the rims of
// a rod of radius 5 and length 20.

// rimRing is the volume of the ring a triangle or a round takes off a rod's rim: the
// section's area times the path of its centroid. area and centroid are about the rim's
// corner, the centroid's distance inward from the wall given.
func rimRound(radius, r float64) float64 {
	const steps = 200000
	step := r / steps
	total := 0.0
	for i := range steps {
		x := (float64(i) + 0.5) * step
		total += 2 * math.Pi * (radius - x) * (r - math.Sqrt(r*r-(r-x)*(r-x))) * step
	}
	return total
}

func TestFilletChordRoundsTheRimsOfARod(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	// A right angle: a chord of √2 is a radius of 1.
	out, err := decad.FilletChord(t.Context(), rod, decad.Edges(decad.Circular()), units.Millimeters(math.Sqrt2))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == out)
	v, err := out.Volume()
	require.NoError(t, err)
	want := math.Pi*25*20 - 2*rimRound(5, 1)
	require.InDelta(t, want, v.Value.Base(), 1e-3*want+v.Bound.Base())
}

func TestChamferAngleBevelsTheRimsOfARod(t *testing.T) {
	t.Parallel()
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	// 2 mm along the cap and 30 degrees to it: it meets the wall 2·tan 30 down. The triangle's
	// centroid lies a third of the way in from the wall corner along the cap.
	const d = 2.0
	other := d * math.Tan(math.Pi/6)
	out, err := decad.ChamferAngle(t.Context(), rod, decad.Edges(decad.Circular()), units.Millimeters(d),
		decad.Faces(decad.Planar()), units.Degrees(30))
	require.NoError(t, err)
	ring := 2 * math.Pi * (5 - d/3) * 0.5 * d * other
	v, err := out.Volume()
	require.NoError(t, err)
	want := math.Pi*25*20 - 2*ring
	require.InDelta(t, want, v.Value.Base(), 1e-3*want+v.Bound.Base())
}

func TestFilletChordStillRefusesACurvedFaceBesideAStraightEdge(t *testing.T) {
	t.Parallel()
	// A quarter disc prism: its vertical edge at (10, 0) lies between a plane and the cylinder,
	// and their angle there is a right angle at the edge but is not one all along a curved face.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o, a, b := s.CreatePoint(0, 0), s.CreatePoint(10, 0), s.CreatePoint(0, 10)
	for _, p := range []*sketch.Point{o, a, b} {
		s.Fix(p)
	}
	s.CreateLine(o, a)
	s.CreateArc(o, a, b)
	s.CreateLine(b, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	prism, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(10, 0, 0))).Exactly(1)
	_, err = decad.FilletChord(t.Context(), prism, edge, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "planar faces")
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == prism)
}
