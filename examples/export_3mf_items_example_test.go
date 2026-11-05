package examples_test

import (
	"archive/zip"
	"bytes"
	"context"
	"fmt"
	"io"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// export.ThreeMFItems writes several bodies as one 3MF file, each a named
// object moved by a rigid placement. Use it for a whole build, assembled or
// pulled apart: the file keeps every part separate.
func Example_export_3mf_items() {
	ctx := context.Background()
	doc := decad.New()
	plate, err := extrudeBlock(doc, 0, 0, 20, 20, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	pin, err := extrudeCircle(doc, r3.Vec{X: 10, Y: 10}, 3, 0, 15)
	if err != nil {
		fmt.Printf("failed to build the pin: %s\n", err)
		return
	}
	// The plate stays where it was built; the pin lifts 30 mm clear of it.
	lift, err := r3.Translation(r3.Vec{Z: 30})
	if err != nil {
		fmt.Printf("failed to build the placement: %s\n", err)
		return
	}
	stay := r3.Identity()

	var out bytes.Buffer
	err = export.ThreeMFItems(ctx, &out, []export.ThreeMFItem{
		{Name: partNamePlate, Body: plate, Placement: stay},
		{Name: partNamePin, Body: pin, Placement: lift},
	}, units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}

	// A 3MF file is a zip archive; read its model part back.
	zr, err := zip.NewReader(bytes.NewReader(out.Bytes()), int64(out.Len()))
	if err != nil {
		fmt.Printf("failed to open the archive: %s\n", err)
		return
	}
	for _, f := range zr.File {
		if f.Name != "3D/3dmodel.model" {
			continue
		}
		r, err := f.Open()
		if err != nil {
			fmt.Printf("failed to open the model: %s\n", err)
			return
		}
		model, err := io.ReadAll(r)
		r.Close()
		if err != nil {
			fmt.Printf("failed to read the model: %s\n", err)
			return
		}
		fmt.Printf("objects: %d, build items: %d\n",
			strings.Count(string(model), "<object "), strings.Count(string(model), "<item "))
		fmt.Printf("named plate and pin: %v\n",
			strings.Contains(string(model), `name="`+partNamePlate+`"`) && strings.Contains(string(model), `name="`+partNamePin+`"`))
	}
	// Output:
	// objects: 2, build items: 2
	// named plate and pin: true
}
