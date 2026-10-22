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

func TestLoftDirectBoreExactLiftRefusals(t *testing.T) {
	for _, tc := range []struct {
		name    string
		origin  r3.Vec
		u, v    r3.Vec
		height  float64
		center  r3.Vec
		top     r3.Vec
		shifted bool
	}{
		{"Y axis lift", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 2,
			r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 2), true},
		{"Z axis lift", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, -1), 2,
			r3.NewVec(0, 0, 0), r3.NewVec(0, 2, 1), true},
		{"rounded center", r3.NewVec(1e9, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 2,
			r3.NewVec(.427, 0, 0), r3.Vec{}, false},
		{"tilted circle map", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 4), r3.NewVec(0, 1, 0), 2,
			r3.NewVec(0, 0, 0), r3.Vec{}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			world := sketch.NewWorld()
			lower, err := world.CreatePlaneFromFrame(frame)
			require.NoError(t, err)
			upper, err := world.CreateOffsetPlane(lower, tc.height)
			if tc.shifted {
				top, frameErr := r3.NewFrame(tc.top, tc.u, tc.v)
				require.NoError(t, frameErr)
				upper, err = world.CreatePlaneFromFrame(top)
			}
			require.NoError(t, err)
			var sections [2]*sketch.Sketch
			var profiles [2]*sketch.Profile
			for i, plane := range []*sketch.Plane{lower, upper} {
				s, err := world.CreateSketch(plane)
				require.NoError(t, err)
				rect := s.CreateRectangle(-5, -5, 5, 5)
				s.Fix(rect.A)
				center := s.CreatePoint(tc.center.X, tc.center.Y)
				s.Fix(center)
				s.CreateCircle(center, 1)
				_, err = s.Solve(t.Context())
				require.NoError(t, err)
				sections[i], profiles[i] = s, s.Profiles()[0]
			}
			body, err := decad.Loft(t.Context(), decad.New(), sections[0], profiles[0], sections[1], profiles[1])
			require.NoError(t, err)
			cylinders, err := decad.Faces(decad.Cylindrical()).SelectFaces(body)
			require.ErrorIs(t, err, decad.ErrNoMatch)
			require.Empty(t, cylinders)
		})
	}
}

func TestLoftCutExactToolCenterRefusesRewrite(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(1e9, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	blank, doc := loftBoreSoundnessBlank(t, frame)
	toolFrame, err := r3.NewFrame(r3.NewVec(1e9, 0, -1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	tool := loftBoreSoundnessTool(t, doc, toolFrame, .427, 0, 4, decad.Along)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	requireRetainedBoreWall(t, cut)
}

func TestLoftCutRoundedYToolCenterKeepsFacetedTopology(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(0, 1e9, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	blank, doc := loftBoreSoundnessBlank(t, frame)
	toolFrame, err := r3.NewFrame(r3.NewVec(0, 1e9, -1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	tool := loftBoreSoundnessTool(t, doc, toolFrame, 0, .427, 4, decad.Along)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	requireLoftCutFacetedVolume(t, cut)
}

func TestLoftCutRoundedZToolCenterKeepsFacetedTopology(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	blank, doc := loftBoreSoundnessBlank(t, frame)
	toolFrame, err := r3.NewFrame(r3.NewVec(0, 0, .427), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	tool := loftBoreSoundnessTool(t, doc, toolFrame, 0, 0, 14, decad.Along)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	requireLoftCutFacetedVolume(t, cut)
}

func TestLoftCutTiltedFrameMapKeepsFacetedTopology(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 4), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	blank, doc := loftBoreSoundnessBlank(t, frame)
	tool := loftBoreSoundnessTool(t, doc, frame, 0, 0, 4, decad.Along)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	// The tilted tool's wall states a tilt from its tag, so A2 declines it
	// (agent-docs/faceted-source-retention-design.md §3.2) and the bore stays
	// Faceted.
	cylinders, err := decad.Faces(decad.Cylindrical()).SelectFaces(cut)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	require.Empty(t, cylinders)
	volume, err := cut.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-2*(100-math.Pi)), volume.Bound.Base())
}

// requireRetainedBoreWall pins what the Cut publishes for a bore whose held
// center rounds: the wall keeps the tool's Cylinder tag as a stand-in, so its
// normal is never an Exact reading
// (agent-docs/faceted-source-retention-design.md §3.3).
func requireRetainedBoreWall(t *testing.T, cut *decad.Body) {
	t.Helper()
	cylinders, err := decad.Faces(decad.Cylindrical()).SelectFaces(cut)
	require.NoError(t, err)
	require.NotEmpty(t, cylinders)
	for _, wall := range cylinders {
		at := wall.Loops()[0].CoEdges()[0].Start().Position().Value
		n, err := wall.NormalAt(at)
		require.NoError(t, err)
		require.Positive(t, n.Bound.Base())
	}
}

func requireLoftCutFacetedVolume(t *testing.T, cut *decad.Body) {
	t.Helper()
	requireRetainedBoreWall(t, cut)
	volume, err := cut.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-2*(100-math.Pi)), volume.Bound.Base())
}

func TestLoftCutExactToolSpanKeepsUncutCap(t *testing.T) {
	for _, tc := range []struct {
		name           string
		originZ, toolZ float64
		dir            decad.Direction
		height         float64
		cap            func(*decad.Body) decad.FeatureRef
	}{
		{"upper cap at base", 1e9, 1e9, decad.Along, math.Nextafter(2, 0), decad.CapEnd},
		{"upper cap", 1e9, 1e9 - 1, decad.Along, math.Nextafter(3, 0), decad.CapEnd},
		{"lower cap", 1e9, 1e9 + 3, decad.Against, math.Nextafter(3, 0), decad.CapStart},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(r3.NewVec(0, 0, tc.originZ), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			blank, doc := loftBoreSoundnessBlank(t, frame, decad.ExactOnly())
			toolFrame, err := r3.NewFrame(r3.NewVec(0, 0, tc.toolZ), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			tool := loftBoreSoundnessTool(t, doc, toolFrame, 0, 0, tc.height, tc.dir)
			uncut := tc.cap(blank)
			cut, err := decad.Cut(t.Context(), blank, tool)
			if err != nil {
				// The general mesh boolean may refuse a rounded cap contact.
				require.ErrorIs(t, err, decad.ErrUnsupported)
				return
			}
			caps, err := decad.Faces(decad.FaceCreatedBy(uncut)).Exactly(1).SelectFaces(cut)
			require.NoError(t, err)
			area, err := caps[0].Area()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(area.Value.Base()-100), area.Bound.Base())
		})
	}
}

func loftBoreSoundnessBlank(t *testing.T, frame r3.Frame, opts ...decad.DocumentOption) (*decad.Body, *decad.Document) {
	t.Helper()
	world := sketch.NewWorld()
	lower, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(lower, 2)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{lower, upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	doc := decad.New(opts...)
	blank, err := decad.Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	return blank, doc
}

func loftBoreSoundnessTool(t *testing.T, doc *decad.Document, frame r3.Frame,
	u, v, height float64, direction decad.Direction) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(u, v)
	s.Fix(center)
	s.CreateCircle(center, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: direction})
	require.NoError(t, err)
	return tool
}
