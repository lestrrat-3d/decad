package export

import (
	"context"
	"fmt"
	"io"
	"strconv"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// STL writes body as deterministic ASCII STL at the requested chord tolerance.
// The mesh defaults to decad.VerifyNone; pass decad.WithVerification to demand
// stronger proofs. A sheet produces an open STL file even though STL uses the
// words "solid" and "endsolid". ctx, w, and body must not be nil.
func STL(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: STL: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: STL: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	return writeSTL(ctx, w, mesh)
}

// meshSource is a triangle mesh a writer reads: a body's tessellation or an
// imported mesh body.
type meshSource interface {
	Vertices() []r3.Vec
	Triangles() [][3]int
}

// writeSTL writes the mesh as deterministic ASCII STL.
func writeSTL(ctx context.Context, w io.Writer, mesh meshSource) error {
	vertices, triangles := mesh.Vertices(), mesh.Triangles()
	if err := ctx.Err(); err != nil {
		return err
	}
	sw := stickyWriter{w: w}
	line := make([]byte, 0, 128)
	line = append(line, "solid decad\n"...)
	sw.write(line)
	for i, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return err
		}
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		normal, ok := b.Sub(a).Cross(c.Sub(a)).Normalize()
		if !ok {
			return fmt.Errorf("export: STL: %w: facet %d has no normal", decad.ErrDegenerate, i)
		}
		line = append(line[:0], "  facet normal "...)
		line = appendCoord(line, normal.X)
		line = append(line, ' ')
		line = appendCoord(line, normal.Y)
		line = append(line, ' ')
		line = appendCoord(line, normal.Z)
		line = append(line, '\n')
		sw.write(line)
		line = append(line[:0], "    outer loop\n"...)
		sw.write(line)
		for _, index := range tri {
			v := vertices[index]
			line = append(line[:0], "      vertex "...)
			line = appendCoord(line, v.X)
			line = append(line, ' ')
			line = appendCoord(line, v.Y)
			line = append(line, ' ')
			line = appendCoord(line, v.Z)
			line = append(line, '\n')
			sw.write(line)
		}
		line = append(line[:0], "    endloop\n"...)
		sw.write(line)
		line = append(line[:0], "  endfacet\n"...)
		sw.write(line)
	}
	line = append(line[:0], "endsolid decad\n"...)
	sw.write(line)
	if sw.err != nil {
		return fmt.Errorf("export: STL write failed: %w", sw.err)
	}
	return nil
}

// OBJ writes body as deterministic Wavefront OBJ at the requested chord
// tolerance. The mesh defaults to decad.VerifyNone; pass
// decad.WithVerification to demand stronger proofs. ctx, w, and body must not
// be nil.
func OBJ(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: OBJ: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: OBJ: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	return writeOBJ(ctx, w, mesh)
}

// writeOBJ writes the mesh as deterministic Wavefront OBJ.
func writeOBJ(ctx context.Context, w io.Writer, mesh meshSource) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	sw := stickyWriter{w: w}
	line := make([]byte, 0, 128)
	for _, v := range mesh.Vertices() {
		if err := ctx.Err(); err != nil {
			return err
		}
		line = append(line[:0], "v "...)
		line = appendCoord(line, v.X)
		line = append(line, ' ')
		line = appendCoord(line, v.Y)
		line = append(line, ' ')
		line = appendCoord(line, v.Z)
		line = append(line, '\n')
		sw.write(line)
	}
	for _, tri := range mesh.Triangles() {
		if err := ctx.Err(); err != nil {
			return err
		}
		line = append(line[:0], "f "...)
		line = strconv.AppendInt(line, int64(tri[0]+1), 10)
		line = append(line, ' ')
		line = strconv.AppendInt(line, int64(tri[1]+1), 10)
		line = append(line, ' ')
		line = strconv.AppendInt(line, int64(tri[2]+1), 10)
		line = append(line, '\n')
		sw.write(line)
	}
	if sw.err != nil {
		return fmt.Errorf("export: OBJ write failed: %w", sw.err)
	}
	return nil
}

func tessellateForExport(ctx context.Context, body *decad.Body, tol units.Value, opts []decad.TessellateOption) (*decad.Mesh, error) {
	if body == nil {
		return nil, fmt.Errorf("export: %w: nil body", decad.ErrDegenerate)
	}
	options := make([]decad.TessellateOption, 0, len(opts)+1)
	options = append(options, decad.WithVerification(decad.VerifyNone))
	options = append(options, opts...)
	return body.Tessellate(ctx, tol, options...)
}

type stickyWriter struct {
	w   io.Writer
	err error
}

func (sw *stickyWriter) printf(format string, args ...any) {
	if sw.err != nil {
		return
	}
	_, sw.err = fmt.Fprintf(sw.w, format, args...)
}

func (sw *stickyWriter) write(p []byte) {
	if sw.err != nil {
		return
	}
	_, sw.err = sw.w.Write(p)
}

func appendCoord(dst []byte, x float64) []byte {
	if x == 0 {
		x = 0
	}
	return strconv.AppendFloat(dst, x, 'g', -1, 64)
}

func fmtCoord(x float64) string {
	if x == 0 {
		x = 0
	}
	return strconv.FormatFloat(x, 'g', -1, 64)
}
