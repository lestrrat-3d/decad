package export

import (
	"context"
	"fmt"
	"io"

	"github.com/lestrrat-3d/decad"
)

// This file writes an imported [decad.MeshBody] in the formats [STL], [OBJ],
// [GLTF], [PNG] and [DXF] write a body in (agent-docs/mesh-import-design.md §6).
// The mesh is written as it is: no tolerance, no tessellation and no proof, so
// a closed mesh read from an STL file and written back as STL holds the same
// triangles. Each function is its body counterpart's writer over the mesh body's
// own vertices and triangles, with the same layout, determinism and context
// rules.

// checkMeshWriter refuses a nil context, writer or mesh body.
func checkMeshWriter(ctx context.Context, w io.Writer, m *decad.MeshBody, format string) error {
	if ctx == nil {
		return fmt.Errorf("export: %s: %w: nil context", format, decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: %s: %w: nil writer", format, decad.ErrDegenerate)
	}
	if m == nil {
		return fmt.Errorf("export: %s: %w: nil mesh body", format, decad.ErrDegenerate)
	}
	return nil
}

// MeshSTL writes m as deterministic ASCII STL, as [STL] writes a body. A
// triangle of zero area has no normal and is ErrDegenerate. ctx, w and m MUST
// NOT be nil.
func MeshSTL(ctx context.Context, w io.Writer, m *decad.MeshBody) error {
	if err := checkMeshWriter(ctx, w, m, "STL"); err != nil {
		return err
	}
	return writeSTL(ctx, w, m)
}

// MeshOBJ writes m as deterministic Wavefront OBJ, as [OBJ] writes a body. ctx,
// w and m MUST NOT be nil.
func MeshOBJ(ctx context.Context, w io.Writer, m *decad.MeshBody) error {
	if err := checkMeshWriter(ctx, w, m, "OBJ"); err != nil {
		return err
	}
	return writeOBJ(ctx, w, m)
}

// MeshGLTF writes m as a deterministic glTF 2.0 file in metres, as [GLTF]
// writes a body. ctx, w and m MUST NOT be nil.
func MeshGLTF(ctx context.Context, w io.Writer, m *decad.MeshBody) error {
	if err := checkMeshWriter(ctx, w, m, "glTF"); err != nil {
		return err
	}
	return writeGLTF(ctx, w, m)
}

// MeshPNG draws m as a PNG image, as [PNG] draws a body, so a mesh read from a
// file can be looked at. A mesh with a face looking away from the camera shows
// it in the back colour, which is how an open or inside-out mesh shows. ctx, w
// and m MUST NOT be nil, and view MUST be valid for [PNG].
func MeshPNG(ctx context.Context, w io.Writer, m *decad.MeshBody, view View) error {
	if err := checkMeshWriter(ctx, w, m, "PNG"); err != nil {
		return err
	}
	return writePNG(ctx, w, m, view)
}

// MeshDXF writes the feature edges of m as a 2D DXF drawing, as [DXF] writes a
// body's. ctx, w and m MUST NOT be nil, and view MUST be valid for [DXF].
func MeshDXF(ctx context.Context, w io.Writer, m *decad.MeshBody, view DrawingView) error {
	if err := checkMeshWriter(ctx, w, m, "DXF"); err != nil {
		return err
	}
	return writeDXFDrawing(ctx, w, m, view)
}
