package examples_test

import (
	"archive/zip"
	"bytes"
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
)

// ReadThreeMF loads a 3MF package, the format slicers and print services use,
// into a MeshBody. The model's unit is converted to millimetres and each build
// item's placement is applied. Here a package holds one triangle-based tetrahedron
// drawn in centimetres.
func Example_mesh_import_3mf() {
	model := `<?xml version="1.0"?>
<model unit="centimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
<resources><object id="1" type="model"><mesh>
<vertices><vertex x="0" y="0" z="0"/><vertex x="3" y="0" z="0"/><vertex x="0" y="4" z="0"/><vertex x="0" y="0" z="5"/></vertices>
<triangles><triangle v1="0" v2="2" v3="1"/><triangle v1="0" v2="1" v3="3"/><triangle v1="0" v2="3" v3="2"/><triangle v1="1" v2="2" v3="3"/></triangles>
</mesh></object></resources>
<build><item objectid="1"/></build></model>`

	// A 3MF package is a zip file holding the model part.
	var pkg bytes.Buffer
	zw := zip.NewWriter(&pkg)
	part, err := zw.Create("3D/3dmodel.model")
	if err != nil {
		fmt.Printf("failed to build the package: %s\n", err)
		return
	}
	if _, err := part.Write([]byte(model)); err != nil {
		fmt.Printf("failed to build the package: %s\n", err)
		return
	}
	if err := zw.Close(); err != nil {
		fmt.Printf("failed to build the package: %s\n", err)
		return
	}

	mesh, err := decad.ReadThreeMF(context.Background(), &pkg)
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	volume, err := mesh.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The tetrahedron has legs of 30, 40 and 50 mm: 30 x 40 x 50 / 6 = 10000 mm^3.
	fmt.Printf("closed: %t, volume: %.0f mm^3\n", mesh.Report().Closed, volume.Value.Base())
	// Output:
	// closed: true, volume: 10000 mm^3
}
