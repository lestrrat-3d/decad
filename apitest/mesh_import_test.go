package apitest_test

import (
	"bytes"
	"encoding/binary"
	"math"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-import-design.md's tests for the mesh readers. The
// round trips take the real producer's output, export's STL and OBJ writers,
// through the real reader.

// importBlock builds a 4 x 3 x 2 block, so a volume of 24 mm^3.
func importBlock(t *testing.T) *decad.Body {
	t.Helper()
	return decadtest.NewBlock(t, decad.New(), 0, 0, 4, 3, units.Millimeters(2))
}

func requireImportedBlock(t *testing.T, m *decad.MeshBody) {
	t.Helper()
	report := m.Report()
	require.True(t, report.Closed, "%+v", report)
	require.False(t, report.Inverted)
	require.Equal(t, 8, report.Vertices, "the block's corners weld to eight vertices")
	require.Equal(t, 12, report.Triangles)
	require.Equal(t, 1, report.Shells)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.Equal(t, 24.0, volume.Value.Base())
	require.Equal(t, decad.Exact, volume.Exactness)
	box := m.Bounds()
	require.Equal(t, r3.NewVec(0, 0, 0), box.Min)
	require.Equal(t, r3.NewVec(4, 3, 2), box.Max)
	area, err := m.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-52), area.Bound.Base()+1e-12, "2·(12+8+6) mm^2")
}

func TestMeshImportSTLRoundTripsAnExportedBlock(t *testing.T) {
	t.Parallel()
	var buf bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf, importBlock(t), units.Millimeters(0.1)))
	m, err := decad.ReadSTL(t.Context(), &buf)
	require.NoError(t, err)
	requireImportedBlock(t, m)
}

func TestMeshImportOBJRoundTripsAnExportedBlock(t *testing.T) {
	t.Parallel()
	var buf bytes.Buffer
	require.NoError(t, export.OBJ(t.Context(), &buf, importBlock(t), units.Millimeters(0.1)))
	m, err := decad.ReadOBJ(t.Context(), &buf)
	require.NoError(t, err)
	requireImportedBlock(t, m)
}

// importTetrahedron returns the unit corner importTetrahedron's four outward facets.
func importTetrahedron() [4][3]r3.Vec {
	o, x, y, z := r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	return [4][3]r3.Vec{{o, y, x}, {o, x, z}, {o, z, y}, {x, y, z}}
}

func importBinarySTL(facets [][3]r3.Vec) []byte {
	var buf bytes.Buffer
	buf.Write(make([]byte, 80))
	_ = binary.Write(&buf, binary.LittleEndian, uint32(len(facets)))
	for _, f := range facets {
		_ = binary.Write(&buf, binary.LittleEndian, [3]float32{}) // normal, ignored
		for _, v := range f {
			_ = binary.Write(&buf, binary.LittleEndian, [3]float32{float32(v.X), float32(v.Y), float32(v.Z)})
		}
		buf.Write([]byte{0, 0})
	}
	return buf.Bytes()
}

func TestMeshImportBinarySTLTetrahedron(t *testing.T) {
	t.Parallel()
	facets := importTetrahedron()
	m, err := decad.ReadSTL(t.Context(), bytes.NewReader(importBinarySTL(facets[:])))
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	require.Equal(t, 4, m.Report().Vertices)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-1.0/6), volume.Bound.Base()+1e-18)
	require.Equal(t, decad.Approximate, volume.Exactness, "a sixth is not a float")
	require.Positive(t, volume.Bound.Base())
}

func TestMeshImportASCIISTLWeldsRepeatedCornersAndStoresPositiveZero(t *testing.T) {
	t.Parallel()
	var b strings.Builder
	b.WriteString("solid t\n")
	for _, f := range importTetrahedron() {
		b.WriteString(" facet normal 0 0 0\n  outer loop\n")
		for _, v := range f {
			x := v.X
			if x == 0 {
				x = math.Copysign(0, -1) // -0 must weld with +0
			}
			b.WriteString("   vertex " + importFloat(x) + " " + importFloat(v.Y) + " " + importFloat(v.Z) + "\n")
		}
		b.WriteString("  endloop\n endfacet\n")
	}
	b.WriteString("endsolid t\n")
	m, err := decad.ReadSTL(t.Context(), strings.NewReader(b.String()))
	require.NoError(t, err)
	require.Equal(t, 4, m.Report().Vertices)
	require.True(t, m.Report().Closed)
	for _, v := range m.Vertices() {
		require.False(t, math.Signbit(v.X) || math.Signbit(v.Y) || math.Signbit(v.Z), "vertex %v keeps a signed zero", v)
	}
}

func importFloat(f float64) string { return strconv.FormatFloat(f, 'g', -1, 64) }

func TestMeshImportOBJAcceptsNegativeIndicesQuadsAndExtraParts(t *testing.T) {
	t.Parallel()
	src := `# a unit square split from one quad
v 0 0 0
v 1 0 0
vt 0 0
v 1 1 0
v 0 1 0
vn 0 0 1
f 1/1/1 2/1/1 3/1/1 4/1/1
f -4 -3 -2
`
	m, err := decad.ReadOBJ(t.Context(), strings.NewReader(src))
	require.NoError(t, err)
	require.Equal(t, 4, m.Report().Vertices)
	require.Equal(t, 3, m.Report().Triangles, "the quad fans into two and the negative-index face adds one")
	area, err := m.Area()
	require.NoError(t, err)
	require.Equal(t, 1.5, area.Value.Base())
	_, err = m.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid, "a flat open sheet of triangles encloses nothing")
}

func TestMeshImportReadsAnInvertedMeshAsNotSolid(t *testing.T) {
	t.Parallel()
	facets := importTetrahedron()
	flipped := make([][3]r3.Vec, len(facets))
	for i, f := range facets {
		flipped[i] = [3]r3.Vec{f[0], f[2], f[1]}
	}
	m, err := decad.ReadSTL(t.Context(), bytes.NewReader(importBinarySTL(flipped)))
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	require.True(t, m.Report().Inverted)
	_, err = m.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
}

func TestMeshImportRefusesMalformedInput(t *testing.T) {
	t.Parallel()
	facets := importTetrahedron()
	truncated := importBinarySTL(facets[:])
	truncated = truncated[:len(truncated)-7]
	cases := []struct {
		name string
		read func() error
	}{
		{"unknown STL", func() error { _, err := decad.ReadSTL(t.Context(), strings.NewReader("hello")); return err }},
		{"truncated binary STL", func() error { _, err := decad.ReadSTL(t.Context(), bytes.NewReader(truncated)); return err }},
		{"ASCII STL bad number", func() error {
			_, err := decad.ReadSTL(t.Context(), strings.NewReader("solid x\nfacet normal 0 0 0\nvertex 0 0 abc\nvertex 1 0 0\nvertex 0 1 0\nendfacet\nendsolid x\n"))
			return err
		}},
		{"ASCII STL open facet", func() error {
			_, err := decad.ReadSTL(t.Context(), strings.NewReader("solid x\nvertex 0 0 0\nvertex 1 0 0\nendsolid x\n"))
			return err
		}},
		{"ASCII STL without facets", func() error {
			_, err := decad.ReadSTL(t.Context(), strings.NewReader("solid x\nendsolid x\n"))
			return err
		}},
		{"OBJ index past the vertices", func() error {
			_, err := decad.ReadOBJ(t.Context(), strings.NewReader("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n"))
			return err
		}},
		{"OBJ zero index", func() error {
			_, err := decad.ReadOBJ(t.Context(), strings.NewReader("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n"))
			return err
		}},
		{"OBJ short vertex", func() error { _, err := decad.ReadOBJ(t.Context(), strings.NewReader("v 0 0\n")); return err }},
		{"OBJ without faces", func() error { _, err := decad.ReadOBJ(t.Context(), strings.NewReader("v 0 0 0\n")); return err }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.ErrorIs(t, tc.read(), decad.ErrMalformedMesh)
		})
	}
}

func TestMeshImportLimitsRefuseLargeInput(t *testing.T) {
	t.Parallel()
	facets := importTetrahedron()
	data := importBinarySTL(facets[:])
	_, err := decad.ReadSTL(t.Context(), bytes.NewReader(data), decad.WithMaxMeshTriangles(3))
	require.ErrorIs(t, err, decad.ErrMeshTooLarge)
	_, err = decad.ReadSTL(t.Context(), bytes.NewReader(data), decad.WithMaxMeshBytes(100))
	require.ErrorIs(t, err, decad.ErrMeshTooLarge)
	_, err = decad.ReadOBJ(t.Context(), strings.NewReader("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 3 4\n"), decad.WithMaxMeshTriangles(1))
	require.ErrorIs(t, err, decad.ErrMeshTooLarge)
	_, err = decad.ReadSTL(t.Context(), bytes.NewReader(data), decad.WithMaxMeshTriangles(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestMeshImportNilArgumentsAreDegenerate(t *testing.T) {
	t.Parallel()
	_, err := decad.ReadSTL(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ReadOBJ(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
