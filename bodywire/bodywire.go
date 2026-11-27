// Package bodywire exchanges read-only body geometry in a binary frame.
package bodywire

import (
	"context"
	"io"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/stepwire"
	"github.com/lestrrat-3d/units"
)

// Body holds received geometry. It cannot be used as a modeling operand.
type Body struct{ source *stepwire.Body }

// Write sends analytic STEP geometry when available, or a bounded faceted output.
// verificationTolerance controls the sender's boundary verification, not
// receiver tessellation. Unsupported exports are refused. ctx, w and body
// must not be nil.
func Write(ctx context.Context, w io.Writer, body *decad.Body, verificationTolerance units.Value) error {
	return stepwire.Write(ctx, w, body, verificationTolerance)
}

// IsSheet reports whether the source was a sheet body.
func (b *Body) IsSheet() bool { return b != nil && b.source.IsSheet() }

// Read receives one frame and leaves later frames unread. ctx and r must not be nil.
func Read(ctx context.Context, r io.Reader) (*Body, error) {
	source, err := stepwire.Read(ctx, r)
	if err != nil {
		return nil, err
	}
	return &Body{source: source}, nil
}

// SourceBoundaryBound returns the sender's claimed bound for faceted output.
// It returns false for analytic records; the receiver does not prove it.
func (b *Body) SourceBoundaryBound() (units.Value, bool) {
	if b == nil {
		return units.Millimeters(0), false
	}
	return b.source.SourceBoundaryBound()
}

// Tessellate returns a mesh at the receiver's requested circular chord tolerance.
// The returned MeshBody describes its own polygons and carries no bound to the
// original body. ctx must not be nil.
func (b *Body) Tessellate(ctx context.Context, tolerance units.Value) (*decad.MeshBody, error) {
	if b == nil {
		return nil, decad.ErrDegenerate
	}
	mesh, err := b.source.Tessellate(ctx, tolerance)
	if err != nil {
		return nil, err
	}
	return decad.NewMeshBody(ctx, mesh.Vertices, mesh.Triangles)
}
