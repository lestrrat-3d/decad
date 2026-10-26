package export

import (
	"context"
	"encoding/base64"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"io"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

const (
	gltfFloat         = 5126  // FLOAT component type
	gltfUnsignedInt   = 5125  // UNSIGNED_INT component type
	gltfArrayBuffer   = 34962 // ARRAY_BUFFER buffer view target
	gltfElementBuffer = 34963 // ELEMENT_ARRAY_BUFFER buffer view target
	gltfTriangles     = 4     // TRIANGLES primitive mode
	gltfMetersPerMM   = 1e-3
)

// GLTF writes body as a deterministic glTF 2.0 file at the requested chord
// tolerance: one JSON document whose single buffer is embedded as a base64
// data URI (agent-docs/gltf-export-design.md). The file holds one scene, one
// node, one mesh and one triangle primitive, with positions and 32-bit indices
// and no normals, materials or textures. Positions are in metres, as glTF
// requires, so every millimetre coordinate is multiplied by 0.001 and stored as
// a 32-bit float, and no axis is exchanged: decad's +Z stays +Z.
//
// The mesh defaults to decad.VerifyNone; pass decad.WithVerification to demand
// stronger proofs. A sheet writes an open mesh. ctx, w and body MUST NOT be
// nil, and tol is a positive length.
func GLTF(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: glTF: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: glTF: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	doc, err := buildGLTF(ctx, mesh)
	if err != nil {
		return err
	}
	data, err := json.Marshal(doc)
	if err != nil {
		return fmt.Errorf("export: glTF encoding failed: %w", err)
	}
	data = append(data, '\n')
	if n, err := w.Write(data); err != nil {
		return fmt.Errorf("export: glTF write failed: %w", err)
	} else if n != len(data) {
		return fmt.Errorf("export: glTF write failed: %w", io.ErrShortWrite)
	}
	return ctx.Err()
}

type gltfDocument struct {
	Asset       gltfAsset        `json:"asset"`
	Scene       int              `json:"scene"`
	Scenes      []gltfScene      `json:"scenes"`
	Nodes       []gltfNode       `json:"nodes"`
	Meshes      []gltfMesh       `json:"meshes"`
	Accessors   []gltfAccessor   `json:"accessors"`
	BufferViews []gltfBufferView `json:"bufferViews"`
	Buffers     []gltfBuffer     `json:"buffers"`
}

type gltfAsset struct {
	Version   string `json:"version"`
	Generator string `json:"generator"`
}

type gltfScene struct {
	Nodes []int `json:"nodes"`
}

type gltfNode struct {
	Mesh int `json:"mesh"`
}

type gltfMesh struct {
	Primitives []gltfPrimitive `json:"primitives"`
}

type gltfPrimitive struct {
	Attributes map[string]int `json:"attributes"`
	Indices    int            `json:"indices"`
	Mode       int            `json:"mode"`
}

type gltfAccessor struct {
	BufferView    int       `json:"bufferView"`
	ComponentType int       `json:"componentType"`
	Count         int       `json:"count"`
	Type          string    `json:"type"`
	Min           []float64 `json:"min,omitempty"`
	Max           []float64 `json:"max,omitempty"`
}

type gltfBufferView struct {
	Buffer     int `json:"buffer"`
	ByteOffset int `json:"byteOffset"`
	ByteLength int `json:"byteLength"`
	Target     int `json:"target"`
}

type gltfBuffer struct {
	ByteLength int    `json:"byteLength"`
	URI        string `json:"uri"`
}

func buildGLTF(ctx context.Context, mesh *decad.Mesh) (gltfDocument, error) {
	vertices, triangles := mesh.Vertices(), mesh.Triangles()
	if len(vertices) == 0 || len(triangles) == 0 {
		return gltfDocument{}, fmt.Errorf("export: glTF: %w: the mesh is empty", decad.ErrDegenerate)
	}
	positionBytes := 12 * len(vertices)
	indexBytes := 12 * len(triangles)
	buf := make([]byte, 0, positionBytes+indexBytes)
	lo := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
	hi := [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for i, v := range vertices {
		if i%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return gltfDocument{}, err
			}
		}
		for k, c := range [3]float64{v.X, v.Y, v.Z} {
			f := float32(c * gltfMetersPerMM)
			if f == 0 {
				f = 0 // normalize -0
			}
			buf = binary.LittleEndian.AppendUint32(buf, math.Float32bits(f))
			lo[k] = min(lo[k], float64(f))
			hi[k] = max(hi[k], float64(f))
		}
	}
	for _, tri := range triangles {
		for _, index := range tri {
			buf = binary.LittleEndian.AppendUint32(buf, uint32(index))
		}
	}
	return gltfDocument{
		Asset:  gltfAsset{Version: "2.0", Generator: "decad"},
		Scene:  0,
		Scenes: []gltfScene{{Nodes: []int{0}}},
		Nodes:  []gltfNode{{Mesh: 0}},
		Meshes: []gltfMesh{{Primitives: []gltfPrimitive{{Attributes: map[string]int{"POSITION": 0}, Indices: 1, Mode: gltfTriangles}}}},
		Accessors: []gltfAccessor{
			{BufferView: 0, ComponentType: gltfFloat, Count: len(vertices), Type: "VEC3", Min: lo[:], Max: hi[:]},
			{BufferView: 1, ComponentType: gltfUnsignedInt, Count: 3 * len(triangles), Type: "SCALAR"},
		},
		BufferViews: []gltfBufferView{
			{Buffer: 0, ByteOffset: 0, ByteLength: positionBytes, Target: gltfArrayBuffer},
			{Buffer: 0, ByteOffset: positionBytes, ByteLength: indexBytes, Target: gltfElementBuffer},
		},
		Buffers: []gltfBuffer{{
			ByteLength: len(buf),
			URI:        "data:application/octet-stream;base64," + base64.StdEncoding.EncodeToString(buf),
		}},
	}, nil
}
