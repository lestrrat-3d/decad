package export

import (
	"context"
	"fmt"
	"io"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PNGBodies draws several bodies in one PNG image, as [PNG] draws one
// (agent-docs/png-export-design.md §5). Each body is tessellated at the chord
// tolerance and the meshes are drawn together, so a nearer body hides a farther
// one and the whole scene is centred and scaled to fill the image with the
// margin [PNG] leaves. The bodies share the one colour scheme: a face toward the
// camera is blue and one looking away is orange. A pixel no face covers is
// opaque white.
//
// The meshes default to decad.VerifyNone; pass decad.WithVerification to demand
// stronger proofs. The output is deterministic for a platform: equal bodies in
// equal order, tolerance, view and options give equal bytes. ctx and w MUST NOT
// be nil, bodies MUST hold at least one body and no nil, and view is valid as
// for [PNG], each otherwise decad.ErrDegenerate.
func PNGBodies(ctx context.Context, w io.Writer, bodies []*decad.Body, tol units.Value, view View, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: PNG: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: PNG: %w: nil writer", decad.ErrDegenerate)
	}
	if len(bodies) == 0 {
		return fmt.Errorf("export: PNG: %w: no body to draw", decad.ErrDegenerate)
	}
	var scene mergedMesh
	for _, body := range bodies {
		mesh, err := tessellateForExport(ctx, body, tol, opts)
		if err != nil {
			return err
		}
		offset := len(scene.vertices)
		scene.vertices = append(scene.vertices, mesh.Vertices()...)
		for _, tri := range mesh.Triangles() {
			scene.triangles = append(scene.triangles, [3]int{tri[0] + offset, tri[1] + offset, tri[2] + offset})
		}
	}
	return writePNG(ctx, w, scene, view)
}

// mergedMesh is the meshes of several bodies as one: each body's triangles keep
// their own vertices, shifted past those of the bodies before it.
type mergedMesh struct {
	vertices  []r3.Vec
	triangles [][3]int
}

func (m mergedMesh) Vertices() []r3.Vec  { return m.vertices }
func (m mergedMesh) Triangles() [][3]int { return m.triangles }
