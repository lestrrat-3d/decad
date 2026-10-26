package meshparse

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"math"
	"strconv"

	"github.com/lestrrat-3d/r3"
)

// OBJ reads a Wavefront OBJ file into a welded mesh. Only the "v" vertex and "f"
// face lines count; texture coordinates, normals, groups and materials are
// ignored. A face index may be positive (counted from the first vertex, which
// is 1) or negative (counted back from the latest vertex), and an index may
// carry "/" parts, of which the first is the vertex. A face of more than three
// vertices is cut into a fan from its first vertex, which is right for a convex
// face and wrong for any other.
//
// Vertices with identical coordinates are welded, as [STL] does. ctx and r
// MUST NOT be nil. Malformed input is [ErrMalformed] and input past a limit is
// [ErrTooLarge].
func OBJ(ctx context.Context, r io.Reader, l Limits) (Mesh, error) {
	data, err := readAll(r, l)
	if err != nil {
		return Mesh{}, err
	}
	w := newWelder()
	var fileVertices []int // each "v" line's welded index, in file order
	var triangles [][3]int
	line := 0
	for len(data) > 0 {
		var text []byte
		text, data, _ = bytes.Cut(data, []byte("\n"))
		line++
		if line%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return Mesh{}, err
			}
		}
		fields := bytes.Fields(text)
		if len(fields) == 0 {
			continue
		}
		switch string(fields[0]) {
		case "v":
			if len(fields) < 4 {
				return Mesh{}, fmt.Errorf("OBJ: %w: line %d: a vertex needs three coordinates", ErrMalformed, line)
			}
			var c [3]float64
			for k := range c {
				f, err := strconv.ParseFloat(string(fields[1+k]), 64)
				if err != nil || math.IsNaN(f) || math.IsInf(f, 0) {
					return Mesh{}, fmt.Errorf("OBJ: %w: line %d: %q is not a finite number", ErrMalformed, line, fields[1+k])
				}
				c[k] = f
			}
			fileVertices = append(fileVertices, w.add(r3.Vec{X: c[0], Y: c[1], Z: c[2]}))
		case "f":
			if len(fields) < 4 {
				return Mesh{}, fmt.Errorf("OBJ: %w: line %d: a face needs three vertices", ErrMalformed, line)
			}
			face := make([]int, 0, len(fields)-1)
			for _, f := range fields[1:] {
				index, err := objIndex(f, len(fileVertices))
				if err != nil {
					return Mesh{}, fmt.Errorf("OBJ: %w: line %d: %s", ErrMalformed, line, err)
				}
				face = append(face, fileVertices[index])
			}
			for i := 1; i+1 < len(face); i++ {
				if len(triangles) >= l.Triangles {
					return Mesh{}, fmt.Errorf("OBJ: %w: more than %d triangles", ErrTooLarge, l.Triangles)
				}
				triangles = append(triangles, [3]int{face[0], face[i], face[i+1]})
			}
		}
	}
	if len(triangles) == 0 {
		return Mesh{}, fmt.Errorf("OBJ: %w: no face", ErrMalformed)
	}
	return w.mesh(triangles), nil
}

// objIndex resolves one face element to a zero-based position in the file's
// vertex list, given how many vertices the file has defined so far.
func objIndex(element []byte, defined int) (int, error) {
	head, _, _ := bytes.Cut(element, []byte("/"))
	n, err := strconv.Atoi(string(head))
	if err != nil || n == 0 {
		return 0, fmt.Errorf("%q is not a vertex index", element)
	}
	if n < 0 {
		n += defined + 1
	}
	if n < 1 || n > defined {
		return 0, fmt.Errorf("index %q names no vertex of the %d defined so far", element, defined)
	}
	return n - 1, nil
}
