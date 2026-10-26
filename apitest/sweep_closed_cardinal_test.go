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

// squareSweepSection returns a centred square section of the given side.
func squareSweepSection(t *testing.T, side float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(-side/2, -side/2, side/2, side/2)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// stadiumPathYZ is racetrackPath turned to lie in the YZ plane, so its arcs
// turn about the X axis.
func stadiumPathYZ(t *testing.T, length, major float64) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, length)},
		decad.ArcThrough{Through: r3.NewVec(0, major, length+major),
			End: r3.NewVec(0, 2*major, length)},
		decad.LineTo{End: r3.NewVec(0, 2*major, 0)},
		decad.ArcThrough{Through: r3.NewVec(0, major, -major), End: r3.Vec{}},
	)
	require.NoError(t, err)
	return path
}

func requireClosedSquareTube(t *testing.T, doc *decad.Document, body *decad.Body, side, pathLength float64) {
	t.Helper()
	require.True(t, body.IsSolid())
	require.Len(t, body.Shells(), 1)
	for _, face := range body.Faces() {
		for _, edge := range face.Edges() {
			require.Len(t, edge.Faces(), 2)
		}
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, side*side*pathLength, volume.Value.Base(), 1e-9+volume.Bound.Base())
	area, err := body.Area()
	require.NoError(t, err)
	require.InDelta(t, 4*side*pathLength, area.Value.Base(), 1e-9+area.Bound.Base())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
}

func TestSweepClosedCardinalSquareStadium(t *testing.T) {
	t.Parallel()
	const side, length, major = 2.0, 5.0, 5.0
	section, profile := squareSweepSection(t, side)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, section, profile, racetrackPath(t, length, major))
	require.NoError(t, err)
	require.Len(t, body.Faces(), 16)
	requireClosedSquareTube(t, doc, body, side, 2*length+2*math.Pi*major)
}

func TestSweepClosedCardinalSquareRoundedRectangle(t *testing.T) {
	t.Parallel()
	const side = 1.0
	section, profile := squareSweepSection(t, side)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, section, profile, roundedRectangleSweepPath(t))
	require.NoError(t, err)
	require.Len(t, body.Faces(), 32)
	requireClosedSquareTube(t, doc, body, side, 40+10*math.Pi)
}

func TestSweepClosedCardinalSquareStadiumAboutXAxis(t *testing.T) {
	t.Parallel()
	const side, length, major = 2.0, 5.0, 5.0
	section, profile := squareSweepSection(t, side)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, section, profile, stadiumPathYZ(t, length, major))
	require.NoError(t, err)
	requireClosedSquareTube(t, doc, body, side, 2*length+2*math.Pi*major)
}

func TestSweepClosedCardinalRefusesNonQuarterArcs(t *testing.T) {
	t.Parallel()
	// Six arcs of one radius-5 circle centred at (5,0,0). Every endpoint is a
	// 3-4-5 lattice point, so the loop is exactly closed and tangent, but no
	// arc is a quarter or half turn.
	at := func(dx, dz float64) r3.Vec { return r3.NewVec(5+dx, 0, dz) }
	path, err := decad.NewPath(at(-5, 0),
		decad.ArcThrough{Through: at(-4, 3), End: at(-3, 4)},
		decad.ArcThrough{Through: at(0, 5), End: at(3, 4)},
		decad.ArcThrough{Through: at(4, 3), End: at(5, 0)},
		decad.ArcThrough{Through: at(4, -3), End: at(3, -4)},
		decad.ArcThrough{Through: at(0, -5), End: at(-3, -4)},
		decad.ArcThrough{Through: at(-4, -3), End: at(-5, 0)},
	)
	require.NoError(t, err)
	section, profile := squareSweepSection(t, 0.5)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, section, profile, path)
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "not an exact quarter or half turn")
	require.Empty(t, doc.Bodies())
}
