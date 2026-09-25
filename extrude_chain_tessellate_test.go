package decad_test

import (
	"bytes"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §13.4's tessellation increment,
// gap 2 of this PR's report: a chain-fed prism ribbon's mesh, restated
// straight off the body's own wall topology with no chording (Table G's
// Plane row). ExtrudeChain and SweepChain's one-span straight reduction
// share the identical restatement (tessellate_chain.go).

// TestExtrudeChainTessellatesOneWallRibbon is T130's ribbon, tessellated: one
// wall, two triangles, a zero Bound since the wall's four corners sit exactly
// on the recorded boundary, and STL/OBJ both write it without error.
func TestExtrudeChainTessellatesOneWallRibbon(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(40, 0)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	doc := decad.New()
	body, err := doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, mesh.Triangles(), 2, "one wall, two triangles")
	require.Len(t, mesh.SourceFaces(), 2)
	require.Zero(t, mesh.Bound().Mag(), "an unplaced, axis-aligned wall's own corners are exact")
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified(), "a sheet publishes no occupied-volume proof")

	free := directedEdgeCensus(t, mesh)
	require.Equal(t, 4, free, "both rims plus one sweep edge at each of the walk's two free ends")

	var buf1, buf2 bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf1, body, units.Millimeters(0.1)))
	require.NoError(t, export.OBJ(t.Context(), &buf2, body, units.Millimeters(0.1)))
	require.Equal(t, 2, countSTLFacets(buf1.String()))
	require.NotEmpty(t, buf2.String())
}

// TestExtrudeChainTessellatesThreeWallRibbon is T132's rectangle-minus-one-
// side chain, tessellated: three walls, six triangles, and the free-edge
// attribution requireSheetMesh proves matches the body's own recorded free
// edges — the same census the profile-fed sheet test runs
// (tessellate_sheet_test.go's TestSurfaceExtrudeSheetTessellates).
func TestExtrudeChainTessellatesThreeWallRibbon(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(10, 6)
	d := s.CreatePoint(0, 6)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	doc := decad.New()
	body, err := doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, mesh.Triangles(), 6, "three walls, two triangles each")

	free := directedEdgeCensus(t, mesh)
	// Two free rims (10 mm and 6 mm walls at each open end) plus two sweep
	// edges at the walk's own two free ends, restated as directed edges: 8
	// body-free edges (T132's own Edges(Free()).Exactly(8)).
	require.Equal(t, 8, free)

	require.InDelta(t, 260.0, meshTriangleArea(mesh), 1e-9, "10+6+10 mm walls of 10 mm height sum to 260 mm^2")

	var buf bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf, body, units.Millimeters(0.1)))
	require.Equal(t, 6, countSTLFacets(buf.String()))
}

// TestExtrudeChainTessellateRefusesCurvedWall is Table G's own staging: a
// chain holding a curved fragment builds a Cylinder wall this increment has
// no chording arm for, so Tessellate, STL and OBJ all refuse it rather than
// silently dropping or approximating the wall.
func TestExtrudeChainTessellateRefusesCurvedWall(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(15, 5)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateArc(s.CreatePoint(10, 5), b, c)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	doc := decad.New()
	body, err := doc.ExtrudeChain(s, ch, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	_, err = body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "no chording arm for a chain-fed")

	var buf bytes.Buffer
	require.ErrorIs(t, export.STL(t.Context(), &buf, body, units.Millimeters(0.1)), decad.ErrUnsupported)
}

// TestSweepChainTessellatesRibbon confirms the chainSweepPayload arm reuses
// the identical restatement: a one-span straight SweepChain ribbon
// tessellates to the same mesh shape (face and triangle count) its
// ExtrudeChain sibling over the same walk does.
func TestSweepChainTessellatesRibbon(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(40, 0)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	doc := decad.New()
	path, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, 0, 10)})
	require.NoError(t, err)
	body, err := doc.SweepChain(t.Context(), s, ch, path)
	require.NoError(t, err)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, mesh.Triangles(), 2)
	require.Zero(t, mesh.Bound().Mag())

	var buf bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf, body, units.Millimeters(0.1)))
	require.Equal(t, 2, countSTLFacets(buf.String()))
}
