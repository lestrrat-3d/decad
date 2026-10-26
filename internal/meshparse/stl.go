package meshparse

import (
	"bytes"
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"strconv"

	"github.com/lestrrat-3d/r3"
)

const (
	stlHeaderBytes = 80
	stlRecordBytes = 50
)

// STL reads a binary or ASCII STL file into a welded mesh. A file is binary when
// its length is exactly 84 bytes plus 50 for each triangle its count names, and
// ASCII when it starts with "solid". The facet normals and attributes a file
// carries are ignored: a triangle's side comes from its vertex order.
//
// Vertices with identical coordinates are welded. ctx and r MUST NOT be nil.
// Malformed input is [ErrMalformed] and input past a limit is [ErrTooLarge].
func STL(ctx context.Context, r io.Reader, l Limits) (Mesh, error) {
	data, err := readAll(r, l)
	if err != nil {
		return Mesh{}, err
	}
	if isBinarySTL(data) {
		return binarySTL(ctx, data, l)
	}
	if bytes.HasPrefix(bytes.TrimLeft(data, " \t\r\n"), []byte("solid")) {
		return asciiSTL(ctx, data, l)
	}
	return Mesh{}, fmt.Errorf("STL: %w: neither a binary STL of the length its count names nor an ASCII STL starting with solid",
		ErrMalformed)
}

func isBinarySTL(data []byte) bool {
	if len(data) < stlHeaderBytes+4 {
		return false
	}
	count := int64(binary.LittleEndian.Uint32(data[stlHeaderBytes:]))
	return int64(len(data)) == stlHeaderBytes+4+count*stlRecordBytes
}

func binarySTL(ctx context.Context, data []byte, l Limits) (Mesh, error) {
	count := int(binary.LittleEndian.Uint32(data[stlHeaderBytes:]))
	if count > l.Triangles {
		return Mesh{}, fmt.Errorf("STL: %w: %d triangles, limit %d", ErrTooLarge, count, l.Triangles)
	}
	w := newWelder()
	triangles := make([][3]int, 0, count)
	record := data[stlHeaderBytes+4:]
	for i := range count {
		if i%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return Mesh{}, err
			}
		}
		rec := record[i*stlRecordBytes : (i+1)*stlRecordBytes]
		var tri [3]int
		for k := range tri {
			at := 12 + 12*k // the 12-byte normal comes first
			v := r3.Vec{
				X: float64(math.Float32frombits(binary.LittleEndian.Uint32(rec[at:]))),
				Y: float64(math.Float32frombits(binary.LittleEndian.Uint32(rec[at+4:]))),
				Z: float64(math.Float32frombits(binary.LittleEndian.Uint32(rec[at+8:]))),
			}
			if math.IsNaN(v.X) || math.IsNaN(v.Y) || math.IsNaN(v.Z) || math.IsInf(v.X, 0) || math.IsInf(v.Y, 0) || math.IsInf(v.Z, 0) {
				return Mesh{}, fmt.Errorf("STL: %w: triangle %d has a non-finite coordinate", ErrMalformed, i)
			}
			tri[k] = w.add(v)
		}
		triangles = append(triangles, tri)
	}
	return w.mesh(triangles), nil
}

func asciiSTL(ctx context.Context, data []byte, l Limits) (Mesh, error) {
	w := newWelder()
	var triangles [][3]int
	var pending []int
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
		if len(fields) == 0 || !bytes.EqualFold(fields[0], []byte("vertex")) {
			continue
		}
		if len(fields) != 4 {
			return Mesh{}, fmt.Errorf("STL: %w: line %d: a vertex needs three coordinates", ErrMalformed, line)
		}
		var c [3]float64
		for k := range c {
			f, err := strconv.ParseFloat(string(fields[1+k]), 64)
			if err != nil || math.IsNaN(f) || math.IsInf(f, 0) {
				return Mesh{}, fmt.Errorf("STL: %w: line %d: %q is not a finite number", ErrMalformed, line, fields[1+k])
			}
			c[k] = f
		}
		pending = append(pending, w.add(r3.Vec{X: c[0], Y: c[1], Z: c[2]}))
		if len(pending) == 3 {
			if len(triangles) >= l.Triangles {
				return Mesh{}, fmt.Errorf("STL: %w: more than %d triangles", ErrTooLarge, l.Triangles)
			}
			triangles = append(triangles, [3]int{pending[0], pending[1], pending[2]})
			pending = pending[:0]
		}
	}
	if len(pending) != 0 {
		return Mesh{}, fmt.Errorf("STL: %w: a facet ends after %d of its 3 vertices", ErrMalformed, len(pending))
	}
	if len(triangles) == 0 {
		return Mesh{}, fmt.Errorf("STL: %w: no facet", ErrMalformed)
	}
	return w.mesh(triangles), nil
}
