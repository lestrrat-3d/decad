package export_test

import (
	"bytes"
	"context"
	"image/png"
	"math"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-import-design.md §6's tests for the export of an
// imported mesh body. The mesh is a tetrahedron with legs of 3, 4 and 5 mm
// along the axes, whose volume is 3·4·5/6 = 10 mm³.

func tetrahedronMesh(t *testing.T) *decad.MeshBody {
	t.Helper()
	m, err := decad.NewMeshBody(t.Context(),
		[]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(3, 0, 0), r3.NewVec(0, 4, 0), r3.NewVec(0, 0, 5)},
		[][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}})
	require.NoError(t, err)
	return m
}

func TestMeshOBJWritesTheMeshAsItIs(t *testing.T) {
	t.Parallel()
	var out bytes.Buffer
	require.NoError(t, export.MeshOBJ(t.Context(), &out, tetrahedronMesh(t)))
	require.Equal(t, "v 0 0 0\nv 3 0 0\nv 0 4 0\nv 0 0 5\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n", out.String())
}

func TestMeshSTLRoundTripsThroughReadSTL(t *testing.T) {
	t.Parallel()
	var out bytes.Buffer
	require.NoError(t, export.MeshSTL(t.Context(), &out, tetrahedronMesh(t)))
	require.Equal(t, 4, strings.Count(out.String(), "facet normal"))
	back, err := decad.ReadSTL(t.Context(), bytes.NewReader(out.Bytes()))
	require.NoError(t, err)
	volume, err := back.Volume()
	require.NoError(t, err)
	require.InDelta(t, 10, volume.Value.Base(), 1e-9)
	// Written again, the read-back mesh gives the same bytes.
	var again bytes.Buffer
	require.NoError(t, export.MeshSTL(t.Context(), &again, back))
	require.Equal(t, out.String(), again.String())
}

func TestMeshGLTFHoldsTheMeshInMetres(t *testing.T) {
	t.Parallel()
	var out bytes.Buffer
	require.NoError(t, export.MeshGLTF(t.Context(), &out, tetrahedronMesh(t)))
	file, vertices, triangles := readGLTF(t, out.Bytes())
	require.Equal(t, 4, file.Accessors[0].Count)
	require.Len(t, triangles, 4)
	require.InDelta(t, 0.005, file.Accessors[0].Max[2], 1e-9, "5 mm is 0.005 m")
	require.InDelta(t, 10e-9, signedVolume(vertices, triangles), 1e-12, "10 mm³ is 1e-8 m³")
}

func TestMeshPNGDrawsTheMesh(t *testing.T) {
	t.Parallel()
	var out bytes.Buffer
	require.NoError(t, export.MeshPNG(t.Context(), &out, tetrahedronMesh(t), export.IsometricView(64, 48)))
	img, err := png.Decode(&out)
	require.NoError(t, err)
	require.Equal(t, 64, img.Bounds().Dx())
	require.Equal(t, 48, img.Bounds().Dy())
	// The corner is background (white); the middle is covered by the solid.
	r, g, b, _ := img.At(0, 0).RGBA()
	require.Equal(t, [3]uint32{0xffff, 0xffff, 0xffff}, [3]uint32{r, g, b})
	r, g, b, _ = img.At(32, 24).RGBA()
	require.NotEqual(t, [3]uint32{0xffff, 0xffff, 0xffff}, [3]uint32{r, g, b})
}

func TestMeshDXFDrawsTheMeshEdges(t *testing.T) {
	t.Parallel()
	var out bytes.Buffer
	// Looking down -Z with +Y up, the base triangle spans x 0 to 3, y 0 to 4.
	view := export.DrawingView{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}
	require.NoError(t, export.MeshDXF(t.Context(), &out, tetrahedronMesh(t), view))
	text := out.String()
	require.GreaterOrEqual(t, strings.Count(text, "LINE\n  8\n"), 6, "a tetrahedron has six edges")
	require.Contains(t, text, "EOF")
	var maxX, maxY float64
	lines := strings.Split(text, "\n")
	for i := 0; i+1 < len(lines); i++ {
		var v float64
		switch strings.TrimSpace(lines[i]) {
		case "10", "11":
			v = parseDXF(t, lines[i+1])
			maxX = math.Max(maxX, math.Abs(v))
		case "20", "21":
			v = parseDXF(t, lines[i+1])
			maxY = math.Max(maxY, math.Abs(v))
		}
	}
	require.InDelta(t, 4, maxY, 1e-9)
	require.InDelta(t, 3, maxX, 1e-9)
}

func TestMeshExportRefusals(t *testing.T) {
	t.Parallel()
	m := tetrahedronMesh(t)
	view := export.IsometricView(64, 48)
	drawing := export.IsometricDrawingView()
	writers := map[string]func(ctx context.Context, w *bytes.Buffer, m *decad.MeshBody) error{
		"STL":  func(ctx context.Context, w *bytes.Buffer, m *decad.MeshBody) error { return export.MeshSTL(ctx, w, m) },
		"OBJ":  func(ctx context.Context, w *bytes.Buffer, m *decad.MeshBody) error { return export.MeshOBJ(ctx, w, m) },
		"glTF": func(ctx context.Context, w *bytes.Buffer, m *decad.MeshBody) error { return export.MeshGLTF(ctx, w, m) },
		"PNG": func(ctx context.Context, w *bytes.Buffer, m *decad.MeshBody) error {
			return export.MeshPNG(ctx, w, m, view)
		},
		"DXF": func(ctx context.Context, w *bytes.Buffer, m *decad.MeshBody) error {
			return export.MeshDXF(ctx, w, m, drawing)
		},
	}
	for name, write := range writers {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			requireMeshWriterRefusals(t, m, write)
		})
	}
	require.ErrorIs(t, export.MeshSTL(t.Context(), nil, m), decad.ErrDegenerate)
}

// parseDXF reads one DXF group value.
func parseDXF(t *testing.T, value string) float64 {
	t.Helper()
	v, err := strconv.ParseFloat(strings.TrimSpace(value), 64)
	require.NoError(t, err)
	return v
}

// requireMeshWriterRefusals checks that write refuses a nil context and a nil
// mesh body, and writes nothing under a cancelled context.
func requireMeshWriterRefusals(t *testing.T, m *decad.MeshBody, write func(context.Context, *bytes.Buffer, *decad.MeshBody) error) {
	t.Helper()
	var nilContext context.Context
	require.ErrorIs(t, write(nilContext, &bytes.Buffer{}, m), decad.ErrDegenerate)
	require.ErrorIs(t, write(t.Context(), &bytes.Buffer{}, nil), decad.ErrDegenerate)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	var out bytes.Buffer
	require.ErrorIs(t, write(canceled, &out, m), context.Canceled)
	require.Zero(t, out.Len(), "a canceled export writes nothing")
}
