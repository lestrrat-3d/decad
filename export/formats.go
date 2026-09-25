package export

import (
	"context"
	"fmt"
	"io"
	"strconv"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// STL writes body as deterministic ASCII STL at the requested chord tolerance.
// The mesh defaults to decad.VerifyNone; pass decad.WithVerification to demand
// stronger proofs. A sheet produces an open STL file even though STL uses the
// words "solid" and "endsolid". ctx, w, and body must not be nil.
func STL(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, opts ...decad.TessellateOption) error {
	if w == nil {
		return fmt.Errorf("export: STL: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	vertices, triangles := mesh.Vertices(), mesh.Triangles()
	if err := ctx.Err(); err != nil {
		return err
	}
	sw := stickyWriter{w: w}
	sw.printf("solid decad\n")
	for i, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return err
		}
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		normal, ok := b.Sub(a).Cross(c.Sub(a)).Normalize()
		if !ok {
			return fmt.Errorf("export: STL: %w: facet %d has no normal", decad.ErrDegenerate, i)
		}
		sw.printf("  facet normal %s %s %s\n", fmtCoord(normal.X), fmtCoord(normal.Y), fmtCoord(normal.Z))
		sw.printf("    outer loop\n")
		for _, index := range tri {
			v := vertices[index]
			sw.printf("      vertex %s %s %s\n", fmtCoord(v.X), fmtCoord(v.Y), fmtCoord(v.Z))
		}
		sw.printf("    endloop\n")
		sw.printf("  endfacet\n")
	}
	sw.printf("endsolid decad\n")
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
	if w == nil {
		return fmt.Errorf("export: OBJ: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	sw := stickyWriter{w: w}
	for _, v := range mesh.Vertices() {
		if err := ctx.Err(); err != nil {
			return err
		}
		sw.printf("v %s %s %s\n", fmtCoord(v.X), fmtCoord(v.Y), fmtCoord(v.Z))
	}
	for _, tri := range mesh.Triangles() {
		if err := ctx.Err(); err != nil {
			return err
		}
		sw.printf("f %d %d %d\n", tri[0]+1, tri[1]+1, tri[2]+1)
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

func fmtCoord(x float64) string {
	if x == 0 {
		x = 0
	}
	return strconv.FormatFloat(x, 'g', -1, 64)
}
