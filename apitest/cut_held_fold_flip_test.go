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

// boltHoleDiscBody extrudes a disc of radius outer from z up length, with four
// holes of radius hole on a circle of radius bolt at 45° + k·90°. The holes'
// top chords at 45° and 135° sit within an ulp of one line, which the cap's
// ear clipping can join with a sliver spanning the disc.
func boltHoleDiscBody(t *testing.T, doc *decad.Document, outer, bolt, hole, z, length float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, z), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	s.CreateCircle(s.Origin(), outer)
	for k := range 4 {
		angle := math.Pi/4 + float64(k)*math.Pi/2
		center := s.CreatePoint(bolt*math.Cos(angle), bolt*math.Sin(angle))
		s.Fix(center)
		s.CreateCircle(center, hole)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Holes) == 4 {
			require.Nil(t, profile)
			profile = candidate
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(t.Context(), doc, s, profile,
		decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// toothedToolBody joins a root disc and teeth copies of one fit-spline tooth
// that reaches from inside the root circle to tip, all from z=0 up length.
// The free-form flanks keep a Boolean with it off the analytic prism route.
func toothedToolBody(t *testing.T, doc *decad.Document, teeth int, root, tip, length float64) *decad.Body {
	t.Helper()
	const samples = 5
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	right, left := make([]*sketch.Point, samples), make([]*sketch.Point, samples)
	inner := root - 0.3
	for i := range samples {
		fraction := float64(i) / float64(samples-1)
		radius := inner + fraction*(tip-inner)
		half := 0.03 - fraction*0.015
		right[i] = s.CreatePoint(radius*math.Cos(-half), radius*math.Sin(-half))
		left[i] = s.CreatePoint(radius*math.Cos(half), radius*math.Sin(half))
		s.Fix(right[i])
		s.Fix(left[i])
	}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(s.Origin(), right[samples-1], left[samples-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateArc(s.Origin(), right[0], left[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	tooth, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	disc := circleBodyAtZ(t, doc, 0, root, 0, length)
	copies, err := decad.PatternCopies(t.Context(), tooth, decad.CircularPattern{Axis: r3.Vec{Z: 1}, Count: teeth})
	require.NoError(t, err)
	tool, err := decad.Union(t.Context(), disc, append([]*decad.Body{tooth}, copies...)...)
	require.NoError(t, err)
	return tool
}

// TestCutHoledDiscByToothedToolKeepsEmbeddedMesh cuts a disc with four bolt
// holes by a toothed tool that passes through its whole length. The disc's
// cap holds a sliver between the holes, and the cut leaves a chain of its
// vertices within an ulp of one line, which rounds to collapsed facets the
// float-box search cannot move; the stitch flips them toward the Delaunay
// triangulation of the cap and rounds again (agent-docs/evaluator-design.md
// §9). The ring keeps the disc's volume less the tool's share inside it.
func TestCutHoledDiscByToothedToolKeepsEmbeddedMesh(t *testing.T) {
	t.Parallel()
	const (
		root   = 34.5
		tip    = 36.75
		z0     = 1.0
		length = 24.5
	)
	for _, tc := range []struct {
		name        string
		teeth       int
		wall, holeD float64
	}{
		{name: "wall 7, hole 3.4", teeth: 3, wall: 7, holeD: 3.4},
		{name: "wall 6.5, hole 3.4", teeth: 8, wall: 6.5, holeD: 3.4},
		{name: "wall 7, hole 2.5", teeth: 3, wall: 7, holeD: 2.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			outer := tip + tc.wall
			bolt := (tip + outer) / 2
			blank := boltHoleDiscBody(t, doc, outer, bolt, tc.holeD/2, z0, length)
			tool := toothedToolBody(t, doc, tc.teeth, root, tip, length+2*z0)

			ring, err := decad.Cut(t.Context(), blank, tool)
			require.NoError(t, err)
			require.True(t, ring.IsSolid())
			require.Len(t, ring.Lumps(), 1)

			// The tool spans the disc's length and stays inside its hole-free
			// middle, so the ring is the disc's section less the tool's, over the
			// disc's length.
			toolVolume, err := tool.Volume()
			require.NoError(t, err)
			hole := tc.holeD / 2
			want := length * (math.Pi*(outer*outer-4*hole*hole) - toolVolume.Value.Base()/(length+2*z0))
			volume, err := ring.Volume()
			require.NoError(t, err)
			slack := volume.Bound.Base() + toolVolume.Bound.Base()*length/(length+2*z0)
			require.InDelta(t, want, volume.Value.Base(), slack)
			require.Less(t, slack, 0.002*want)

			mesh, err := ring.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}
