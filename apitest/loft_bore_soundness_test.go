package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftDirectBoreExactLiftRefusals(t *testing.T) {
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u, v   r3.Vec
		height float64
		center r3.Vec
		top    r3.Vec
	}{
		{"Y axis lift", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 2,
			r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 2)},
		{"Z axis lift", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, -1), 2,
			r3.NewVec(0, 0, 0), r3.NewVec(0, 2, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			world := sketch.NewWorld()
			lower, err := world.CreatePlaneFromFrame(frame)
			require.NoError(t, err)
			top, err := r3.NewFrame(tc.top, tc.u, tc.v)
			require.NoError(t, err)
			upper, err := world.CreatePlaneFromFrame(top)
			require.NoError(t, err)
			body := loftBoredSquares(t, world, lower, upper, tc.center)
			cylinders, err := decad.Faces(decad.Cylindrical()).SelectFaces(body)
			require.ErrorIs(t, err, decad.ErrNoMatch)
			require.Empty(t, cylinders)
		})
	}
}

// TestLoftDirectBoreBoundsRoundedCylinder lofts a square with a radius-1
// bore whose exact axis is proved, but whose held center rounds or whose frame
// map is not exactly orthonormal. The bore publishes its cylinder and rims,
// and the cylinder's normal carries the departure of its tag.
func TestLoftDirectBoreBoundsRoundedCylinder(t *testing.T) {
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u      r3.Vec
		center r3.Vec
	}{
		{"rounded center", r3.NewVec(1e9, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(.427, 0, 0)},
		{"tilted circle map", r3.Vec{}, r3.NewVec(1, 0, 4), r3.Vec{}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			world := sketch.NewWorld()
			lower, err := world.CreatePlaneFromFrame(frame)
			require.NoError(t, err)
			upper, err := world.CreateOffsetPlane(lower, 2)
			require.NoError(t, err)
			body := loftBoredSquares(t, world, lower, upper, tc.center)
			requireBoundedBoreCylinder(t, body)
		})
	}
}

// TestLoftDirectBoreRoundedCenterChargesNormalBound reads the cylinder's
// normal at a point where the tag's own arithmetic is exact, so the whole
// bound NormalAt reports is the wall's departure from its tag.
func TestLoftDirectBoreRoundedCenterChargesNormalBound(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(1e9, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	world := sketch.NewWorld()
	lower, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(lower, 2)
	require.NoError(t, err)
	body := loftBoredSquares(t, world, lower, upper, r3.NewVec(.427, 0, 0))
	walls, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	cylinder, ok := walls[0].Surface().(decad.Cylinder)
	require.True(t, ok)
	require.Equal(t, r3.NewVec(0, 0, 1), cylinder.Axis)
	// 1e9 + 0.427 has no float, so the held axis sits off the true one.
	exact := new(big.Rat).Add(big.NewRat(1e9, 1), proofarith.FloatRat(.427))
	require.NotZero(t, exact.Cmp(proofarith.FloatRat(cylinder.Origin.X)))
	at := cylinder.Origin.Add(r3.NewVec(1, 0, 1))
	normal, err := walls[0].NormalAt(at)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(-1, 0, 0), normal.Value)
	require.Positive(t, normal.Bound.Base())
	require.Less(t, normal.Bound.Base(), 1e-6)
}

// loftBoredSquares lofts a 10 x 10 square with a radius-1 hole at center
// between two planes of world.
func loftBoredSquares(t *testing.T, world *sketch.World, lower, upper *sketch.Plane, at r3.Vec) *decad.Body {
	t.Helper()
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{lower, upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(rect.A)
		center := s.CreatePoint(at.X, at.Y)
		s.Fix(center)
		s.CreateCircle(center, 1)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := decad.Loft(t.Context(), decad.New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	return body
}

// requireBoundedBoreCylinder checks a two-high square loft with a radius-1
// bore: one cylinder rimmed by two whole circles, a normal that is never an
// Exact reading, a wall area of 4π and a volume of 2(100 − π).
func requireBoundedBoreCylinder(t *testing.T, body *decad.Body) {
	t.Helper()
	walls, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	wall := walls[0]
	require.Len(t, wall.Loops(), 2)
	for _, loop := range wall.Loops() {
		require.Len(t, loop.Edges(), 1)
		require.IsType(t, decad.Circle3{}, loop.Edges()[0].Curve())
	}
	at := wall.Loops()[0].CoEdges()[0].Start().Position().Value
	normal, err := wall.NormalAt(at)
	require.NoError(t, err)
	require.Positive(t, normal.Bound.Base())
	area, err := wall.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-4*math.Pi), area.Bound.Base())
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-2*(100-math.Pi)), volume.Bound.Base())
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

func TestLoftCutTiltedFrameMapBoundsCylinder(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 4), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	blank, doc := loftBoreSoundnessBlank(t, frame)
	tool := loftBoreSoundnessTool(t, doc, frame, 0, 0, 4, decad.Along)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	// The certified Cut keeps the Loft, and its bore cylinder charges the
	// tilted frame map's departure to every normal reading.
	requireBoundedBoreCylinder(t, cut)
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
