package decad

import (
	"fmt"
	"io"
	"math"
	"strconv"

	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the writer half of the export slice (core §11): ASCII STL and
// Wavefront OBJ over Body.Tessellate. Both writers are deterministic — the
// same body and tolerance produce identical bytes.

// STLOption configures Body.STL.
type STLOption interface {
	option.Interface
	stlOption()
}

// OBJOption configures Body.OBJ.
type OBJOption interface {
	option.Interface
	objOption()
}

// STLOBJOption is an option both exporters accept: one value flows into
// Body.STL and Body.OBJ alike.
type STLOBJOption interface {
	STLOption
	OBJOption
}

type stlObjOption struct{ option.Interface }

func (stlObjOption) stlOption() {}
func (stlObjOption) objOption() {}

type identChordTolerance struct{}

// WithChordTolerance sets the chord tolerance the exporter tessellates at —
// the tol of [Body.Tessellate], validated identically. Without it the
// exporter uses the documented default: 1/1000 of the body's own
// bounding-box diagonal, so facet density is scale-invariant.
func WithChordTolerance(tol units.Value) STLOBJOption {
	return stlObjOption{option.New(identChordTolerance{}, tol)}
}

// STL writes the body as ASCII STL — chosen over binary so the output is a
// deterministic, diffable text record (core §11). Facet normals are the
// triangles' own outward normals; the mesh comes from [Body.Tessellate] at
// the [WithChordTolerance] tolerance, or the documented size-derived default
// without one, and every Tessellate rejection surfaces unchanged — including
// the [ErrUnsupported] a revolve body returns, which cannot be exported.
func (b *Body) STL(w io.Writer, opts ...STLOption) error {
	folded := make([]option.Interface, len(opts))
	for i, o := range opts {
		folded[i] = o
	}
	mesh, err := b.exportMesh(folded)
	if err != nil {
		return err
	}
	sw := stickyWriter{w: w}
	sw.printf("solid decad\n")
	for i, tri := range mesh.triangles {
		a, bb, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
		nrm, ok := bb.Sub(a).Cross(c.Sub(a)).Normalize()
		if !ok {
			return fmt.Errorf(`%w: facet %d has no normal`, ErrDegenerate, i)
		}
		sw.printf("  facet normal %s %s %s\n", fmtCoord(nrm.X), fmtCoord(nrm.Y), fmtCoord(nrm.Z))
		sw.printf("    outer loop\n")
		for _, vi := range tri {
			v := mesh.vertices[vi]
			sw.printf("      vertex %s %s %s\n", fmtCoord(v.X), fmtCoord(v.Y), fmtCoord(v.Z))
		}
		sw.printf("    endloop\n")
		sw.printf("  endfacet\n")
	}
	sw.printf("endsolid decad\n")
	if sw.err != nil {
		return fmt.Errorf(`decad: STL write failed: %w`, sw.err)
	}
	return nil
}

// OBJ writes the body as Wavefront OBJ — the shared vertex table first, then
// the triangles as 1-based index triples, both in mesh order (core §11). The
// mesh comes from [Body.Tessellate] at the [WithChordTolerance] tolerance, or
// the documented size-derived default without one, and every Tessellate
// rejection surfaces unchanged — including the [ErrUnsupported] a revolve body
// returns, which cannot be exported.
func (b *Body) OBJ(w io.Writer, opts ...OBJOption) error {
	folded := make([]option.Interface, len(opts))
	for i, o := range opts {
		folded[i] = o
	}
	mesh, err := b.exportMesh(folded)
	if err != nil {
		return err
	}
	sw := stickyWriter{w: w}
	for _, v := range mesh.vertices {
		sw.printf("v %s %s %s\n", fmtCoord(v.X), fmtCoord(v.Y), fmtCoord(v.Z))
	}
	for _, tri := range mesh.triangles {
		sw.printf("f %d %d %d\n", tri[0]+1, tri[1]+1, tri[2]+1)
	}
	if sw.err != nil {
		return fmt.Errorf(`decad: OBJ write failed: %w`, sw.err)
	}
	return nil
}

// exportMesh folds the exporter options and tessellates the body at the
// chosen — or default — chord tolerance.
func (b *Body) exportMesh(opts []option.Interface) (*Mesh, error) {
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	var tol units.Value
	chosen := false
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identChordTolerance:
			v, ok := option.Get[units.Value](o)
			if !ok {
				return nil, fmt.Errorf(`%w: WithChordTolerance carries no tolerance`, ErrDegenerate)
			}
			tol = v
			chosen = true
		}
	}
	if !chosen {
		var err error
		tol, err = b.defaultChordTolerance()
		if err != nil {
			return nil, err
		}
	}
	return b.Tessellate(tol)
}

// defaultChordTolerance derives the exporter's default from the body's own
// size: 1/1000 of its bounding-box diagonal. A body this evaluator did not
// build has no payload to tessellate anyway; the staging error is
// Tessellate's to report, so a zero diagonal only guards the impossible.
func (b *Body) defaultChordTolerance() (units.Value, error) {
	if b.payload == nil {
		return units.Value{}, fmt.Errorf(`%w: this evaluator cannot tessellate a body it did not build`, ErrUnsupported)
	}
	diag := b.bounds.Max.Sub(b.bounds.Min).Len()
	if diag <= 0 || math.IsInf(diag, 0) || math.IsNaN(diag) {
		return units.Value{}, fmt.Errorf(`%w: the body has no extent to derive a chord tolerance from`, ErrDegenerate)
	}
	return units.Millimeters(diag / 1000), nil
}

// stickyWriter carries the first write error forward so each emitted line
// stays a single checked expression.
type stickyWriter struct {
	w   io.Writer
	err error
}

// printf writes one formatted line unless an earlier write already failed.
func (sw *stickyWriter) printf(format string, args ...any) {
	if sw.err != nil {
		return
	}
	_, sw.err = fmt.Fprintf(sw.w, format, args...)
}

// fmtCoord renders a coordinate in its shortest round-trip decimal form,
// normalizing negative zero, so writer output is deterministic byte for byte.
func fmtCoord(x float64) string {
	if x == 0 {
		x = 0
	}
	return strconv.FormatFloat(x, 'g', -1, 64)
}
