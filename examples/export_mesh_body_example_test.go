package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
)

// MeshOBJ, MeshSTL, MeshGLTF, MeshPNG and MeshDXF write an imported mesh in the
// formats a body is written in. The mesh is written as it is, so use them to
// convert a mesh file or to draw one before building around it.
func Example_export_mesh_body() {
	// A tetrahedron as an OBJ file: one corner at the origin and three more
	// 3, 4 and 5 mm along the axes.
	src := `v 0 0 0
v 3 0 0
v 0 4 0
v 0 0 5
f 1 3 2
f 1 2 4
f 1 4 3
f 2 3 4
`
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(src))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}

	// Write it as STL, then read that back and measure it.
	var stl bytes.Buffer
	if err := export.MeshSTL(context.Background(), &stl, mesh); err != nil {
		fmt.Printf("failed to write STL: %s\n", err)
		return
	}
	back, err := decad.ReadSTL(context.Background(), &stl)
	if err != nil {
		fmt.Printf("failed to read STL: %s\n", err)
		return
	}
	volume, err := back.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())

	// Draw it as an isometric PNG.
	var img bytes.Buffer
	if err := export.MeshPNG(context.Background(), &img, mesh, export.IsometricView(64, 64)); err != nil {
		fmt.Printf("failed to draw PNG: %s\n", err)
		return
	}
	fmt.Printf("png: %t\n", bytes.HasPrefix(img.Bytes(), []byte("\x89PNG")))
	// Output:
	// volume: 10 mm^3
	// png: true
}
