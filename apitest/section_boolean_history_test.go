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
	operandRefusal  = "exact line section of each Boolean operand"
	regionRefusal   = "is not one exact region"
	junctionRefusal = "junction Sketch does not certify exact"
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
	t.Run("copy of a mesh result", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 5, 5, 15, 15, 5, 10))
		require.NoError(t, err)
		duplicate, err := decad.Copy(t.Context(), notched)
		require.NoError(t, err)
		refuses(t, duplicate, 7, historyRefusal)
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
	t.Run("plane missing an operand", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		joined, err := decad.Union(t.Context(), historyCube(t, doc), beamAlongX(t, doc, -5, 20, 6, 14))
		require.NoError(t, err)
		refuses(t, joined, 3, operandRefusal)
	})
	t.Run("curved operand section", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), 5)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		center := s.CreatePoint(10, 10)
		s.Fix(center)
		s.CreateCircle(center, 3)
		notched, err := decad.Cut(t.Context(), historyCube(t, doc), extrudeSolved(t, doc, s, 10))
		require.NoError(t, err)
		refuses(t, notched, 7, operandRefusal)
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
	t.Run("two separate regions", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		joined, err := decad.Union(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 12, 3, 15, 7, 5, 10))
		require.NoError(t, err)
		refuses(t, joined, 7, regionRefusal)
	})
	t.Run("a slot splits the section", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		slotted, err := decad.Cut(t.Context(), historyCube(t, doc), boxBodyAtZ(t, doc, 4, -5, 6, 15, 5, 10))
		require.NoError(t, err)
		refuses(t, slotted, 7, regionRefusal)
	})
	t.Run("one section inside the other", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		joined, err := decad.Union(t.Context(), historyCube(t, doc), beamAlongX(t, doc, 2, 6, 2, 6))
		require.NoError(t, err)
		refuses(t, joined, 4, regionRefusal)
	})
}
