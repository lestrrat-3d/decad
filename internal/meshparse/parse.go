// Package meshparse reads triangle meshes from STL and Wavefront OBJ bytes
// into plain vertices and triangles (agent-docs/mesh-import-design.md). It
// welds identical vertices and checks nothing about the surface they form;
// decad.NewMeshBody does that. Coordinates are taken as millimetres.
package meshparse

import (
	"errors"
	"fmt"
	"io"

	"github.com/lestrrat-3d/r3"
)

// ErrMalformed marks input that is not a well-formed STL or OBJ file: a bad
// header, a truncated record, a number that does not parse or an index that
// names no vertex.
var ErrMalformed = errors.New("decad: malformed mesh file")

// ErrTooLarge marks input that holds more triangles or bytes than the limits
// allow.
var ErrTooLarge = errors.New("decad: mesh file too large")

// Default limits.
const (
	DefaultMaxTriangles = 5_000_000
	DefaultMaxBytes     = 512 << 20
)

// Limits caps what a read accepts. Both fields are positive.
type Limits struct {
	Triangles int
	Bytes     int64
}

// Mesh is the vertices and triangles a file holds, after welding.
type Mesh struct {
	Vertices  []r3.Vec
	Triangles [][3]int
}

// readAll reads r in full, refusing input past the byte limit.
func readAll(r io.Reader, l Limits) ([]byte, error) {
	data, err := io.ReadAll(io.LimitReader(r, l.Bytes+1))
	if err != nil {
		return nil, fmt.Errorf("mesh read failed: %w", err)
	}
	if int64(len(data)) > l.Bytes {
		return nil, fmt.Errorf("%w: more than %d bytes", ErrTooLarge, l.Bytes)
	}
	return data, nil
}

// welder merges vertices with identical coordinates, so a file that repeats a
// corner for every triangle yields a connected mesh. Only exact equality
// merges; two corners that differ in any bit stay apart.
type welder struct {
	index    map[[3]float64]int
	vertices []r3.Vec
}

func newWelder() *welder { return &welder{index: make(map[[3]float64]int)} }

func (w *welder) add(v r3.Vec) int {
	// A map key already treats -0 and +0 as one; adding zero stores +0 whichever came first.
	key := [3]float64{v.X + 0, v.Y + 0, v.Z + 0}
	if i, ok := w.index[key]; ok {
		return i
	}
	i := len(w.vertices)
	w.index[key] = i
	w.vertices = append(w.vertices, r3.Vec{X: key[0], Y: key[1], Z: key[2]})
	return i
}

func (w *welder) mesh(triangles [][3]int) Mesh {
	return Mesh{Vertices: w.vertices, Triangles: triangles}
}
