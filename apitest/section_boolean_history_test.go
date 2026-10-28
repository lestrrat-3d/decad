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

// This file is agent-docs/section-design.md's public-surface tests for a mesh
// Boolean result sectioned through its operation history.

const (
	historyRefusal  = "exact two-body Boolean history"
	operandRefusal  = "exact section of whole lines, arcs and circles of each Boolean operand"
	junctionRefusal = "junction Sketch does not certify exact"
	curveCutRefusal = "cuts an arc or circle, and Sketch certifies no exact arc junction"
)

// beamAlongX extrudes the rectangle [3, 7] x [z0, z1], drawn on the YZ plane
// offset to x0, by length along +X.
func beamAlongX(t *testing.T, doc *decad.Document, x0, length, z0, z1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), x0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(3, z0, 7, z1)
	s.Fix(rect.A)
	return extrudeSolved(t, doc, s, length)
}

// extrudeSolved solves s and extrudes its one profile by h along its normal.
func extrudeSolved(t *testing.T, doc *decad.Document, s *sketch.Sketch, h float64) *decad.Body {
	t.Helper()
	_, err := s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// historyLevel is the world-XY frame at height z.
func historyLevel(t *testing.T, z float64) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return f
}

// historyCube is the 10 mm cube every fixture combines with a second body.
func historyCube(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
}

// cornerTool extrudes the square [5, 15] x [5, 15] by 10 mm from a Sketch on
// the plane spanned by u then v at height z.
func cornerTool(t *testing.T, doc *decad.Document, z float64, u, v r3.Vec) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{Z: z}, u, v)
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(5, 5, 15, 15)
	s.Fix(rect.A)
	return extrudeSolved(t, doc, s, 10)
}

// requireExactHistorySection sections body on frame and checks the one
// hole-free profile's corners exactly and its area against the closed form.
// A 2 mm Extrude of the result must enclose twice the area and pass VerifyAll.
func requireExactHistorySection(t *testing.T, body *decad.Body, frame r3.Frame, corners [][2]float64, area float64) {
	t.Helper()
	require.True(t, tookMeshPath(body), "the fixture is a mesh Boolean result")
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Empty(t, profiles[0].Holes)
	var got [][2]float64
	for _, edge := range profiles[0].Outer {
		line, isLine := edge.Entity.(*sketch.Line)
		require.True(t, isLine)
		require.False(t, edge.Partial)
		got = append(got, [2]float64{line.Start.X(), line.Start.Y()})
	}
	require.ElementsMatch(t, corners, got)

	reading, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	measured, err := reading.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(measured.Value.Base()-area), measured.Bound.Base())
	require.LessOrEqual(t, measured.Bound.Base(), 1e-9)

	slab, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := slab.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-2*area), volume.Bound.Base())
	mesh, err := slab.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// TestSectionMeshBooleanFromHistory sections real mesh Union, Cut and
// Intersect results of two boxes as the 2D operation of their operands'
// sections. The beam runs along X, so it is sectioned in a frame whose V axis
// is the X axis and mapped back.
func TestSectionMeshBooleanFromHistory(t *testing.T) {
	t.Parallel()
	t.Run("union through the overlap", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		joined, err := decad.Union(t.Context(), historyCube(t, doc), beamAlongX(t, doc, -5, 20, 6, 14))
		require.NoError(t, err)
		requireExactHistorySection(t, joined, historyLevel(t, 8), [][2]float64{
			{-5, 3}, {0, 3}, {0, 0}, {10, 0}, {10, 3}, {15, 3},
			{15, 7}, {10, 7}, {10, 10}, {0, 10}, {0, 7}, {-5, 7},
		}, 100+2*5*4)
	})
	t.Run("cut through the notch", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 5, 5, 15, 15, 5, 10))
		require.NoError(t, err)
		requireExactHistorySection(t, notched, historyLevel(t, 7), [][2]float64{
			{0, 0}, {10, 0}, {10, 5}, {5, 5}, {5, 10}, {0, 10},
		}, 100-25)
	})
	t.Run("intersect through the overlap", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		common, err := decad.Intersect(t.Context(), historyCube(t, doc), beamAlongX(t, doc, -5, 20, 6, 14))
		require.NoError(t, err)
		requireExactHistorySection(t, common, historyLevel(t, 8), [][2]float64{
			{0, 3}, {10, 3}, {10, 7}, {0, 7},
		}, 40)
	})
	t.Run("tool drawn on a plane facing down", func(t *testing.T) {
		t.Parallel()
		// The tool's Sketch plane has U = Y and V = X, so its normal is -Z and
		// it sweeps downward from z = 15. Its section in world XY is read in
		// the swapped frame and re-walked.
		doc := decad.New()
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), cornerTool(t, doc, 15, r3.Vec{Y: 1}, r3.Vec{X: 1}))
		require.NoError(t, err)
		requireExactHistorySection(t, notched, historyLevel(t, 7), [][2]float64{
			{0, 0}, {10, 0}, {10, 5}, {5, 5}, {5, 10}, {0, 10},
		}, 100-25)
	})
	t.Run("tilted sketch plane", func(t *testing.T) {
		t.Parallel()
		// Both squares are drawn on one tilted plane, whose held axes no
		// rebuilt frame reproduces bit for bit, so the section plane is that
		// plane itself. Each box is moved along the normal by a power of two
		// times it, which keeps its frame origin exactly on the normal line:
		// the cube spans levels -4 to 4 and the tool -8 to 2, so no cap is
		// shared. At level 0 the section is the 10 mm square less the tool's
		// 5 mm corner square, in the plane's own coordinates.
		doc := decad.New()
		w := sketch.NewWorld()
		tilted, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 3, Z: -1})
		require.NoError(t, err)
		plane, err := w.CreatePlaneFromFrame(tilted)
		require.NoError(t, err)
		held, err := plane.Frame()
		require.NoError(t, err)
		// Extrude holds the plane's frame as r3.NewFrame rebuilds it from the
		// plane's held axes; the section asks in that same frame.
		frame, err := r3.NewFrame(held.Origin(), held.U(), held.V())
		require.NoError(t, err)
		box := func(lo, hi, length, down float64) *decad.Body {
			s, err := w.CreateSketch(plane)
			require.NoError(t, err)
			rect := s.CreateRectangle(lo, lo, hi, hi)
			s.Fix(rect.A)
			move, err := r3.Translation(frame.N().Scale(-down))
			require.NoError(t, err)
			moved, err := decad.Transform(t.Context(), extrudeSolved(t, doc, s, length), move)
			require.NoError(t, err)
			return moved
		}
		notched, err := decad.Cut(t.Context(), box(0, 10, 8, 4), box(5, 15, 10, 8))
		require.NoError(t, err)
		requireExactHistorySection(t, notched, frame, [][2]float64{
			{0, 0}, {10, 0}, {10, 5}, {5, 5}, {5, 10}, {0, 10},
		}, 100-25)
	})
}

// TestSectionMeshBooleanHistoryRefusals reaches each history gate and checks
// which one refused.
func TestSectionMeshBooleanHistoryRefusals(t *testing.T) {
	t.Parallel()
	refuses := func(t *testing.T, body *decad.Body, z float64, want string) {
		t.Helper()
		require.True(t, tookMeshPath(body), "the fixture is a mesh Boolean result")
		_, _, err := body.Section(t.Context(), historyLevel(t, z))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, want)
	}
	t.Run("gap-shifted result", func(t *testing.T) {
		t.Parallel()
		// The tool's top face is flush with the cube's, so the mesh path
		// shifts it: the result no longer denotes cube minus tool.
		doc := decad.New()
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 5, 2, 15, 8, 5, 5))
		require.NoError(t, err)
		diagnostics := notched.OperationDiagnostics()
		require.Len(t, diagnostics, 1)
		require.Equal(t, decad.OpDiagGapShiftApplied, diagnostics[0].Code)
		refuses(t, notched, 7, historyRefusal)
	})
	t.Run("copy of a mesh result is read from the grid", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 5, 5, 15, 15, 5, 10))
		require.NoError(t, err)
		duplicate, err := decad.Copy(t.Context(), notched)
		require.NoError(t, err)
		// The history gate refuses a copy, but a copy of an axis-square result keeps its exact vertices, so the grid
		// reads the same notch (agent-docs/orthogonal-section-design.md).
		requireExactHistorySection(t, duplicate, historyLevel(t, 7), [][2]float64{
			{0, 0}, {10, 0}, {10, 5}, {5, 5}, {5, 10}, {0, 10},
		}, 75)
	})
	t.Run("plane on the tool's cap", func(t *testing.T) {
		t.Parallel()
		// At z = 5 the result's material is the whole square, not the square
		// minus the tool's footprint, so the identity does not hold here.
		doc := decad.New()
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 5, 5, 15, 15, 5, 10))
		require.NoError(t, err)
		refuses(t, notched, 5, operandRefusal)
	})
	t.Run("plane missing an operand is read from the grid", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		joined, err := decad.Union(t.Context(), historyCube(t, doc), beamAlongX(t, doc, -5, 20, 6, 14))
		require.NoError(t, err)
		// The history gate refuses a plane that misses an operand, but the result is a solid whose faces all square to
		// an axis and a plane square to one, so the grid reads the cut (agent-docs/orthogonal-section-design.md).
		requireExactHistorySection(t, joined, historyLevel(t, 3), [][2]float64{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, 100)
	})
	t.Run("cut circle", func(t *testing.T) {
		t.Parallel()
		// The cylinder about (10, 10) cuts the cube's corner, so its arc is cut
		// at (7, 10) and (10, 7). Sketch names no line-arc crossing as an exact
		// point, so the cut arc refuses.
		doc := decad.New()
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), historyZCylinder(t, doc, 10, 10, 3, 5, 10))
		require.NoError(t, err)
		refuses(t, notched, 7, curveCutRefusal)
	})
	t.Run("inexact line junction", func(t *testing.T) {
		t.Parallel()
		// The slanted side from (8, 20) to (5, -5) crosses the cube's edge
		// y = 10 at x = 6.8 and its edge y = 0 at x = 5.6; binary64 holds
		// neither.
		doc := decad.New()
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), 5)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		a, b, c := s.CreatePoint(5, -5), s.CreatePoint(15, -5), s.CreatePoint(8, 20)
		s.Fix(a)
		s.Fix(b)
		s.Fix(c)
		s.CreateLine(a, b)
		s.CreateLine(b, c)
		s.CreateLine(c, a)
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), extrudeSolved(t, doc, s, 10))
		require.NoError(t, err)
		refuses(t, notched, 7, junctionRefusal)
	})
	t.Run("two separate regions are listed from the grid", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		joined, err := decad.Union(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 12, 3, 15, 7, 5, 10))
		require.NoError(t, err)
		// The history gate refuses two regions, but the grid reads each as a profile of one sketch.
		require.ElementsMatch(t, []float64{100, 12}, sectionAreas(t, joined, historyLevel(t, 7)))
	})
	t.Run("a slot splits the section into two profiles", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		slotted, err := decad.Cut(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 4, -5, 6, 15, 5, 10))
		require.NoError(t, err)
		require.ElementsMatch(t, []float64{40, 40}, sectionAreas(t, slotted, historyLevel(t, 7)))
	})
	t.Run("one section inside the other is read from the grid", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		joined, err := decad.Union(t.Context(), historyCube(t, doc), beamAlongX(t, doc, 2, 6, 2, 6))
		require.NoError(t, err)
		// A beam wholly inside the cube leaves the cube's square: the grid reads it where the history gate does not.
		requireExactHistorySection(t, joined, historyLevel(t, 4), [][2]float64{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, 100)
	})
}

// historyZCylinder extrudes a circle of radius r about (u, v), drawn at height z, by
// h along +Z.
func historyZCylinder(t *testing.T, doc *decad.Document, u, v, r, z, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(u, v)
	s.Fix(center)
	s.CreateCircle(center, r)
	return extrudeSolved(t, doc, s, h)
}

// xCylinder extrudes a circle of radius r about (y, z), drawn on the YZ plane
// offset to x0, by length along +X.
func xCylinder(t *testing.T, doc *decad.Document, x0, length, y, z, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), x0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(y, z)
	s.Fix(center)
	s.CreateCircle(center, r)
	return extrudeSolved(t, doc, s, length)
}

// xBeam extrudes the rectangle [y0, y1] x [z0, z1], drawn on the YZ plane
// offset to x0, by length along +X.
func xBeam(t *testing.T, doc *decad.Document, x0, length, y0, y1, z0, z1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), x0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(y0, z0, y1, z1)
	s.Fix(rect.A)
	return extrudeSolved(t, doc, s, length)
}

// holedPlate is the 10 mm cube with a through hole of radius 2 about (5, 5).
func holedPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	center := s.CreatePoint(5, 5)
	s.Fix(center)
	s.CreateCircle(center, 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, profile := range s.Profiles() {
		if len(profile.Holes) == 1 {
			body, err := decad.Extrude(t.Context(), doc, s, profile,
				decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)
			return body
		}
	}
	t.Fatal("the plate sketch has no holed profile")
	return nil
}

// boredTube revolves the meridian r 4..10, z 0..20 a full turn about world Z.
func boredTube(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	a, b, c, d := s.CreatePoint(4, 0), s.CreatePoint(10, 0), s.CreatePoint(10, 20), s.CreatePoint(4, 20)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{}, End: decad.Point2{V: 1},
	}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// historyRegion is one expected region of a section read through history: its
// net area in closed form and how many holes it has.
type historyRegion struct {
	area  float64
	holes int
}

// requireHistoryRegions sections a mesh Boolean result and checks each region,
// in the order Section lists them, against its closed-form area. A 1 mm
// Extrude of every region must enclose that area and pass VerifyAll.
func requireHistoryRegions(t *testing.T, body *decad.Body, frame r3.Frame, want []historyRegion) {
	t.Helper()
	require.True(t, tookMeshPath(body), "the fixture is a mesh Boolean result")
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, len(want))
	for i, profile := range profiles {
		require.Len(t, profile.Holes, want[i].holes)
		reading, err := decad.MeasureProfile(section, profile)
		require.NoError(t, err)
		area, err := reading.Area()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(area.Value.Base()-want[i].area), area.Bound.Base())
		require.LessOrEqual(t, area.Bound.Base(), 1e-9)

		slab, err := decad.Extrude(t.Context(), decad.New(), section, profile,
			decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
		require.NoError(t, err)
		volume, err := slab.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-want[i].area), volume.Bound.Base())
		require.LessOrEqual(t, volume.Bound.Base(), 1e-9)
		mesh, err := slab.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	}
}

// yzLevel is the frame on the plane x = level with U = +Y and V = +Z.
func yzLevel(t *testing.T, level float64) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{X: level}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	return f
}

// TestSectionMeshBooleanHistoryCurvesAndRegions sections mesh Boolean results
// whose operand sections hold whole circles and holes, and whose results hold
// holes or several regions.
func TestSectionMeshBooleanHistoryCurvesAndRegions(t *testing.T) {
	t.Parallel()
	t.Run("slot splits a holed plate", func(t *testing.T) {
		t.Parallel()
		// The beam y 7.5..9.5 runs through the plate along X. At z = 5 it
		// leaves the part below it, with the round hole, and a 0.5 mm strip.
		doc := decad.New()
		slotted, err := decad.Cut(t.Context(), holedPlate(t, doc), xBeam(t, doc, -5, 20, 7.5, 9.5, -1, 11))
		require.NoError(t, err)
		requireHistoryRegions(t, slotted, historyLevel(t, 5), []historyRegion{
			{area: 75 - 4*math.Pi, holes: 1},
			{area: 5},
		})
	})
	t.Run("beam crosses a holed plate", func(t *testing.T) {
		t.Parallel()
		// The common part is y 1..9 of the plate, and the hole lies inside it.
		doc := decad.New()
		common, err := decad.Intersect(t.Context(), holedPlate(t, doc), xBeam(t, doc, -5, 20, 1, 9, 3, 7))
		require.NoError(t, err)
		requireHistoryRegions(t, common, historyLevel(t, 5), []historyRegion{{area: 80 - 4*math.Pi, holes: 1}})
	})
	t.Run("cylinder apart from the cube", func(t *testing.T) {
		t.Parallel()
		// On the plane x = 5 the cube is a square in its cardinal vertical
		// section, and the cylinder along X is a whole circle in its own
		// parallel section. Neither bounds the other.
		doc := decad.New()
		joined, err := decad.Union(t.Context(), historyCube(t, doc), xCylinder(t, doc, 2, 6, 20, 5, 2))
		require.NoError(t, err)
		requireHistoryRegions(t, joined, yzLevel(t, 5), []historyRegion{{area: 100}, {area: 4 * math.Pi}})
	})
	t.Run("box inside a revolved tube's bore", func(t *testing.T) {
		t.Parallel()
		// At z = 8 the tube is an annulus r 4..10 from its meridian, and the
		// box is a square inside the bore that touches nothing.
		doc := decad.New()
		joined, err := decad.Union(t.Context(), boredTube(t, doc), boxBodyAtZ(t, doc, -2, -2, 2, 2, 5, 10))
		require.NoError(t, err)
		requireHistoryRegions(t, joined, historyLevel(t, 8), []historyRegion{{area: 16}, {area: 84 * math.Pi, holes: 1}})
	})
}

// TestSectionMeshBooleanHistoryRefusesCutCurves crosses two cylinders: the
// section at x = 0 is a disk of radius 6 joined to a 40 x 8 band, whose four
// corners at (±sqrt(20), ±4) no binary64 point holds. It refuses.
func TestSectionMeshBooleanHistoryRefusesCutCurves(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 4)
	alongY, err := r3.Translation(r3.Vec{Y: 20})
	require.NoError(t, err)
	tool, err := decad.Transform(t.Context(), extrudeSolved(t, doc, s, 40), alongY)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), xCylinder(t, doc, -20, 40, 0, 0, 6), tool)
	require.NoError(t, err)
	require.True(t, tookMeshPath(joined), "the fixture is a mesh Boolean result")
	_, _, err = joined.Section(t.Context(), yzLevel(t, 0))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, curveCutRefusal)
}
