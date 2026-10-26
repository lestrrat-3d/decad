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

// polylineChain records one open Sketch walk through points on the world XY
// sketch.
func polylineChain(t *testing.T, points ...[2]float64) (*sketch.Sketch, *sketch.Chain) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	var previous *sketch.Point
	for _, p := range points {
		point := s.CreatePoint(p[0], p[1])
		s.Fix(point)
		if previous != nil {
			s.CreateLine(previous, point)
		}
		previous = point
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	return s, s.Chains()[0]
}

// lineQuarterLinePath rises 10 mm on +Z, turns a radius-5 quarter about +Y
// through (1,0,13) to (5,0,15), then runs 8 mm along +X.
func lineQuarterLinePath(t *testing.T) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
		decad.ArcThrough{Through: r3.NewVec(1, 0, 13), End: r3.NewVec(5, 0, 15)},
		decad.LineTo{End: r3.NewVec(13, 0, 15)})
	require.NoError(t, err)
	return path
}

// lineQuarterQuarterPath rises 10 mm on +Z, turns a radius-5 quarter about
// (5,0,10) to (5,0,15), then turns the opposite way through a radius-5
// quarter about (5,0,20) to (10,0,20).
func lineQuarterQuarterPath(t *testing.T) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
		decad.ArcThrough{Through: r3.NewVec(1, 0, 13), End: r3.NewVec(5, 0, 15)},
		decad.ArcThrough{Through: r3.NewVec(8, 0, 16), End: r3.NewVec(10, 0, 20)})
	require.NoError(t, err)
	return path
}

func requireMeasuresWithin(t *testing.T, m decad.Measurement, want float64) {
	t.Helper()
	require.LessOrEqual(t, math.Abs(m.Value.Base()-want), m.Bound.Base()+1e-12,
		"value %g, want %g, bound %g", m.Value.Base(), want, m.Bound.Base())
}

func requireBoxEncloses(t *testing.T, box decad.Box, lo, hi r3.Vec) {
	t.Helper()
	bound := box.Bound.Base() + 1e-12
	for i, pair := range [][2]float64{
		{box.Min.X, lo.X}, {box.Min.Y, lo.Y}, {box.Min.Z, lo.Z},
		{box.Max.X, hi.X}, {box.Max.Y, hi.Y}, {box.Max.Z, hi.Z},
	} {
		require.LessOrEqual(t, math.Abs(pair[0]-pair[1]), bound, "box component %d reads %g, want %g", i, pair[0], pair[1])
	}
}

// TestSweepChainCompositeNSpanSheets sweeps one 2 mm Sketch line, and an L
// of two 2 mm lines, along line-arc-line and line-arc-arc paths. Each wall's
// area is its source length times the path length its centroid travels
// (Pappus), computed here from the path and source alone.
func TestSweepChainCompositeNSpanSheets(t *testing.T) {
	t.Parallel()
	quarter := math.Pi / 2
	for _, tc := range []struct {
		name      string
		points    [][2]float64
		path      func(*testing.T) *decad.Path
		faces     int
		freeEdges int
		area      float64
		lo, hi    r3.Vec
	}{
		{
			name: "line-arc-line", points: [][2]float64{{0, 0}, {2, 0}}, path: lineQuarterLinePath,
			faces: 3, freeEdges: 8,
			// 2 mm over 10 + 8 mm of line, then centroid radius 5-1 over a quarter.
			area: 2*18 + quarter*2*4,
			lo:   r3.Vec{}, hi: r3.NewVec(13, 0, 15),
		},
		{
			name: "line-arc-arc", points: [][2]float64{{0, 0}, {2, 0}}, path: lineQuarterQuarterPath,
			faces: 3, freeEdges: 8,
			// The second quarter turns the other way, so the source sits at
			// radii 5..7 about its axis: centroid radius 6.
			area: 2*10 + quarter*2*4 + quarter*2*6,
			lo:   r3.Vec{}, hi: r3.NewVec(12, 0, 20),
		},
		{
			name: "L walk line-arc-line", points: [][2]float64{{0, 0}, {2, 0}, {2, 2}}, path: lineQuarterLinePath,
			faces: 6, freeEdges: 10,
			// The second segment runs along the arc axis at radius 3.
			area: 4*18 + quarter*2*4 + quarter*2*3,
			lo:   r3.Vec{}, hi: r3.NewVec(13, 2, 15),
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, chain := polylineChain(t, tc.points...)
			doc := decad.New()
			sheet, err := decad.Sweep(t.Context(), doc, s, chain, tc.path(t))
			require.NoError(t, err)
			require.Equal(t, decad.BodySheet, sheet.Kind())
			require.False(t, sheet.IsSolid())
			require.Len(t, sheet.Faces(), tc.faces)
			_, err = decad.Edges(decad.Free()).Exactly(tc.freeEdges).SelectEdges(sheet)
			require.NoError(t, err)
			for _, e := range sheet.Edges() {
				require.LessOrEqual(t, len(e.Faces()), 2)
			}
			area, err := sheet.Area()
			require.NoError(t, err)
			requireMeasuresWithin(t, area, tc.area)
			box, err := sheet.Bounds()
			require.NoError(t, err)
			requireBoxEncloses(t, box, tc.lo, tc.hi)
			_, err = sheet.Volume()
			require.ErrorIs(t, err, decad.ErrNotSolid)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			bodyReport, err := report.ForBody(sheet)
			require.NoError(t, err)
			require.Equal(t, decad.ValidityUndecided, bodyReport.Validity.Outcome)
			_, err = sheet.Tessellate(t.Context(), units.Millimeters(0.1))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Len(t, doc.Bodies(), 1)
		})
	}
}

// TestSweepChainCompositeNSpanThickenSquare thickens the one-line sheets by
// their own 2 mm width, so the solid sweeps a 2 mm square, and an L walk by
// 0.5 mm. Each volume is the section's area times the path length its
// centroid travels (Pappus).
func TestSweepChainCompositeNSpanThickenSquare(t *testing.T) {
	t.Parallel()
	quarter := math.Pi / 2
	// The positive side of the L walk (0,0)-(2,0)-(2,2) is its right: two
	// 2 x 0.5 strips and a quarter disc of radius 0.5 at the convex corner.
	disc := math.Pi / 16
	lArea := 2 + disc
	// Moment about u: strips at u = 1 and u = 2.25, the disc's centroid at
	// u = 2 + 4(0.5)/(3π).
	lMoment := 1 + 2.25 + disc*(2+2/(3*math.Pi))
	for _, tc := range []struct {
		name      string
		points    [][2]float64
		thickness float64
		path      func(*testing.T) *decad.Path
		volume    float64
		faces     int
	}{
		{"line-arc-line", [][2]float64{{0, 0}, {2, 0}}, 2, lineQuarterLinePath, 4*18 + quarter*4*4, 14},
		{"line-arc-arc", [][2]float64{{0, 0}, {2, 0}}, 2, lineQuarterQuarterPath, 4*10 + quarter*4*4 + quarter*4*6, 14},
		// The arc axis sits at u = 5, so the arc span sweeps the moment of
		// (5 - u) over the section.
		{"L walk line-arc-line", [][2]float64{{0, 0}, {2, 0}, {2, 2}}, 0.5, lineQuarterLinePath,
			lArea*18 + quarter*(5*lArea-lMoment), 23},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, chain := polylineChain(t, tc.points...)
			doc := decad.New()
			sheet, err := decad.Sweep(t.Context(), doc, s, chain, tc.path(t))
			require.NoError(t, err)
			solid, err := decad.Thicken(t.Context(), sheet, units.Millimeters(tc.thickness))
			require.NoError(t, err)
			require.Equal(t, decad.BodySolid, solid.Kind())
			require.True(t, solid.IsSolid())
			require.Len(t, solid.Faces(), tc.faces)
			requireManifold(t, solid)
			require.Len(t, doc.Bodies(), 1)
			require.Same(t, solid, doc.Bodies()[0])

			volume, err := solid.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-tc.volume), volume.Bound.Base()+1e-12,
				"volume %g, want %g, bound %g", volume.Value.Base(), tc.volume, volume.Bound.Base())
			mesh, err := solid.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			result, err := report.ForBody(solid)
			require.NoError(t, err)
			require.Equal(t, decad.Sound, result.Status)
		})
	}
}

// TestSweepChainCompositeNonradialHalfCircle sweeps a slanted source line
// along a line and an XZ half-circle. The slant runs along the arc axis, so
// the meridian is the section plane and Pappus gives the arc wall's area.
func TestSweepChainCompositeNonradialHalfCircle(t *testing.T) {
	t.Parallel()
	s, chain := polylineChain(t, [2]float64{0, 0}, [2]float64{4, 1})
	_, _, path := cardinalLineArcChain(t)
	doc := decad.New()
	sheet, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	require.Len(t, sheet.Faces(), 2)
	area, err := sheet.Area()
	require.NoError(t, err)
	length := math.Sqrt(17)
	requireMeasuresWithin(t, area, 20*length+math.Pi*3*length)
	// The certified sheet mesh covers only the radial one-line class.
	_, err = sheet.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "one cardinal line and one positive half-circle")
	// The generated ribbon section needs axis-parallel source lines.
	_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "the open walk is not axis-parallel")
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, sheet, doc.Bodies()[0])
}

// TestSweepChainCompositeRefusals pins the specific refusal of each path a
// composite chain Sweep does not admit; none commits a body.
func TestSweepChainCompositeRefusals(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		segments []decad.PathSegment
		want     string
	}{
		{
			// A quarter about (5,0,0) that leaves the sketch plane at once.
			name: "arc first",
			segments: []decad.PathSegment{
				decad.ArcThrough{Through: r3.NewVec(1, 0, 3), End: r3.NewVec(5, 0, 5)},
				decad.LineTo{End: r3.NewVec(13, 0, 5)},
			},
			want: "the first span must be a line",
		},
		{
			// A radius-5 arc of angle acos(0.6) about +Y: rational points,
			// but no exact sine or cosine.
			name: "non-quadrantal arc",
			segments: []decad.PathSegment{
				decad.LineTo{End: r3.NewVec(0, 0, 10)},
				decad.ArcThrough{Through: r3.NewVec(1, 0, 13), End: r3.NewVec(2, 0, 14)},
				decad.LineTo{End: r3.NewVec(6, 0, 17)},
			},
			want: "span 1 is not an exact quarter or half turn",
		},
		{
			// A half turn about (3,3,10) along (1,-1,0).
			name: "non-coordinate axis",
			segments: []decad.PathSegment{
				decad.LineTo{End: r3.NewVec(0, 0, 10)},
				decad.ArcThrough{Through: r3.NewVec(2, 2, 14), End: r3.NewVec(6, 6, 10)},
				decad.LineTo{End: r3.NewVec(6, 6, 0)},
			},
			want: "span 1 is an arc about a non-coordinate axis",
		},
		{
			// Three quarters about (5,0,10), then back along -X until the
			// last span's section reaches the first span's x = 2 face.
			name: "self-touching",
			segments: []decad.PathSegment{
				decad.LineTo{End: r3.NewVec(0, 0, 10)},
				decad.ArcThrough{Through: r3.NewVec(1, 0, 13), End: r3.NewVec(5, 0, 15)},
				decad.ArcThrough{Through: r3.NewVec(9, 0, 13), End: r3.NewVec(10, 0, 10)},
				decad.ArcThrough{Through: r3.NewVec(9, 0, 7), End: r3.NewVec(5, 0, 5)},
				decad.LineTo{End: r3.NewVec(2, 0, 5)},
			},
			want: "sweep spans 0 and 4 have no strict bounded-box separation",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, chain := polylineChain(t, [2]float64{0, 0}, [2]float64{2, 0})
			path, err := decad.NewPath(r3.Vec{}, tc.segments...)
			require.NoError(t, err)
			doc := decad.New()
			_, err = decad.Sweep(t.Context(), doc, s, chain, path)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
			require.Empty(t, doc.Bodies())
		})
	}
}

// TestSweepChainCompositeThickenRadialGate thickens a sheet swept through a
// quarter turn about +X, whose axis sits at v = 5 in the section. Thickening
// toward +v by 5 mm or more reaches or crosses that axis and refuses at the
// arc span's radial gate; the other cases build with their Pappus volumes.
func TestSweepChainCompositeThickenRadialGate(t *testing.T) {
	t.Parallel()
	quarter := math.Pi / 2
	for _, tc := range []struct {
		name      string
		thickness float64
		side      decad.ThickenSide
		// volume is zero for a refusal.
		volume float64
	}{
		{"negative 5 touches the axis", 5, decad.ThickenNegative, 0},
		{"negative 6 crosses the axis", 6, decad.ThickenNegative, 0},
		// v in [0,2]: area 4, centroid radius 4.
		{"negative 2", 2, decad.ThickenNegative, 4*18 + quarter*4*4},
		// v in [-6,0]: area 12, centroid radius 8.
		{"positive 6", 6, decad.ThickenPositive, 12*18 + quarter*12*8},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, chain := polylineChain(t, [2]float64{0, 0}, [2]float64{2, 0})
			path, err := decad.NewPath(r3.Vec{},
				decad.LineTo{End: r3.NewVec(0, 0, 10)},
				decad.ArcThrough{Through: r3.NewVec(0, 1, 13), End: r3.NewVec(0, 5, 15)},
				decad.LineTo{End: r3.NewVec(0, 13, 15)})
			require.NoError(t, err)
			doc := decad.New()
			sheet, err := decad.Sweep(t.Context(), doc, s, chain, path)
			require.NoError(t, err)
			solid, err := decad.Thicken(t.Context(), sheet, units.Millimeters(tc.thickness),
				decad.WithThickenSide(tc.side))
			if tc.volume == 0 {
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.ErrorContains(t, err, "the swept offset reaches the revolve axis")
				require.Len(t, doc.Bodies(), 1)
				require.Same(t, sheet, doc.Bodies()[0])
				return
			}
			require.NoError(t, err)
			requireManifold(t, solid)
			volume, err := solid.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-tc.volume), volume.Bound.Base()+1e-12,
				"volume %g, want %g, bound %g", volume.Value.Base(), tc.volume, volume.Bound.Base())
		})
	}
}
