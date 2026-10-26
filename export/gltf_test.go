package export_test

import (
	"bytes"
	"context"
	"encoding/base64"
	"encoding/binary"
	"encoding/json"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// decodedGLTF is the part of a glTF file these tests read back.
type decodedGLTF struct {
	Asset struct {
		Version string `json:"version"`
	} `json:"asset"`
	Scene  int `json:"scene"`
	Meshes []struct {
		Primitives []struct {
			Attributes map[string]int `json:"attributes"`
			Indices    int            `json:"indices"`
			Mode       int            `json:"mode"`
		} `json:"primitives"`
	} `json:"meshes"`
	Accessors []struct {
		BufferView    int       `json:"bufferView"`
		ComponentType int       `json:"componentType"`
		Count         int       `json:"count"`
		Type          string    `json:"type"`
		Min           []float64 `json:"min"`
		Max           []float64 `json:"max"`
	} `json:"accessors"`
	BufferViews []struct {
		ByteOffset int `json:"byteOffset"`
		ByteLength int `json:"byteLength"`
	} `json:"bufferViews"`
	Buffers []struct {
		ByteLength int    `json:"byteLength"`
		URI        string `json:"uri"`
	} `json:"buffers"`
}

const gltfDataPrefix = "data:application/octet-stream;base64,"

// readGLTF decodes file and returns it with the positions (metres) and
// triangles its embedded buffer holds.
func readGLTF(t *testing.T, file []byte) (decodedGLTF, [][3]float32, [][3]int) {
	t.Helper()
	var doc decodedGLTF
	require.NoError(t, json.Unmarshal(file, &doc))
	require.Len(t, doc.Buffers, 1)
	require.True(t, strings.HasPrefix(doc.Buffers[0].URI, gltfDataPrefix))
	data, err := base64.StdEncoding.DecodeString(strings.TrimPrefix(doc.Buffers[0].URI, gltfDataPrefix))
	require.NoError(t, err)
	require.Equal(t, doc.Buffers[0].ByteLength, len(data))

	positions := doc.Accessors[0]
	view := doc.BufferViews[positions.BufferView]
	vertices := make([][3]float32, positions.Count)
	for i := range vertices {
		for k := range vertices[i] {
			at := view.ByteOffset + 12*i + 4*k
			vertices[i][k] = math.Float32frombits(binary.LittleEndian.Uint32(data[at:]))
		}
	}
	indices := doc.Accessors[1]
	iview := doc.BufferViews[indices.BufferView]
	triangles := make([][3]int, indices.Count/3)
	for i := range triangles {
		for k := range triangles[i] {
			triangles[i][k] = int(binary.LittleEndian.Uint32(data[iview.ByteOffset+12*i+4*k:]))
		}
	}
	return doc, vertices, triangles
}

// signedVolume is the divergence-theorem volume of a triangle mesh.
func signedVolume(vertices [][3]float32, triangles [][3]int) float64 {
	total := 0.0
	for _, tri := range triangles {
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		cross := [3]float64{
			float64(b[1])*float64(c[2]) - float64(b[2])*float64(c[1]),
			float64(b[2])*float64(c[0]) - float64(b[0])*float64(c[2]),
			float64(b[0])*float64(c[1]) - float64(b[1])*float64(c[0]),
		}
		total += (float64(a[0])*cross[0] + float64(a[1])*cross[1] + float64(a[2])*cross[2]) / 6
	}
	return total
}

func TestGLTFWritesABlockInMetres(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 4, 3, units.Millimeters(2))
	var out bytes.Buffer
	require.NoError(t, export.GLTF(t.Context(), &out, body, units.Millimeters(0.1)))
	doc, vertices, triangles := readGLTF(t, out.Bytes())

	require.Equal(t, "2.0", doc.Asset.Version)
	require.Len(t, doc.Meshes, 1)
	prim := doc.Meshes[0].Primitives[0]
	require.Equal(t, map[string]int{"POSITION": 0}, prim.Attributes)
	require.Equal(t, 4, prim.Mode, "TRIANGLES")
	require.Equal(t, 5126, doc.Accessors[0].ComponentType)
	require.Equal(t, "VEC3", doc.Accessors[0].Type)
	require.Equal(t, 5125, doc.Accessors[1].ComponentType)
	require.Equal(t, "SCALAR", doc.Accessors[1].Type)

	// The 4 x 3 x 2 mm block spans 4 x 3 x 2 thousandths of a metre.
	require.InDeltaSlice(t, []float64{0, 0, 0}, doc.Accessors[0].Min, 1e-12)
	require.InDeltaSlice(t, []float64{0.004, 0.003, 0.002}, doc.Accessors[0].Max, 1e-9)
	require.InDelta(t, 24e-9, signedVolume(vertices, triangles), 24e-9*1e-6, "24 mm^3 in cubic metres")
	for _, tri := range triangles {
		for _, index := range tri {
			require.Less(t, index, len(vertices))
		}
	}
}

func TestGLTFPositionsMatchTheTessellationInMetres(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 1, 2, 5, 7, units.Millimeters(3))
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	var out bytes.Buffer
	require.NoError(t, export.GLTF(t.Context(), &out, body, units.Millimeters(0.1)))
	_, vertices, triangles := readGLTF(t, out.Bytes())
	want := mesh.Vertices()
	require.Len(t, vertices, len(want))
	for i, v := range want {
		require.InDelta(t, v.X*1e-3, float64(vertices[i][0]), 1e-9)
		require.InDelta(t, v.Y*1e-3, float64(vertices[i][1]), 1e-9)
		require.InDelta(t, v.Z*1e-3, float64(vertices[i][2]), 1e-9)
	}
	require.Equal(t, mesh.Triangles(), triangles, "the writer keeps the mesh's indices and winding")
}

func TestGLTFIsDeterministic(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 4, 3, units.Millimeters(2))
	var a, b bytes.Buffer
	require.NoError(t, export.GLTF(t.Context(), &a, body, units.Millimeters(0.1)))
	require.NoError(t, export.GLTF(t.Context(), &b, body, units.Millimeters(0.1)))
	require.Equal(t, a.Bytes(), b.Bytes())
}

func TestGLTFRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 4, 3, units.Millimeters(2))
	var nilContext context.Context
	require.ErrorIs(t, export.GLTF(nilContext, &bytes.Buffer{}, body, units.Millimeters(0.1)), decad.ErrDegenerate)
	require.ErrorIs(t, export.GLTF(t.Context(), nil, body, units.Millimeters(0.1)), decad.ErrDegenerate)
	require.ErrorIs(t, export.GLTF(t.Context(), &bytes.Buffer{}, nil, units.Millimeters(0.1)), decad.ErrDegenerate)
	require.ErrorIs(t, export.GLTF(t.Context(), &bytes.Buffer{}, body, units.Degrees(1)), decad.ErrUnitKind)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	var out bytes.Buffer
	require.ErrorIs(t, export.GLTF(canceled, &out, body, units.Millimeters(0.1)), context.Canceled)
	require.Zero(t, out.Len(), "a canceled export writes nothing")
}
