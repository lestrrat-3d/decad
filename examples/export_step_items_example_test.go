package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"image/color"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// export.STEPItems writes several bodies as one STEP assembly. Each distinct body becomes a named part with its
// exact surfaces, and each item places it in the assembly by a rigid motion, so a receiving CAD system opens
// named components in their positions, with the colours given. Several items may place one body.
func Example_export_step_items() {
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
	lift, err := r3.Translation(r3.Vec{Z: 30})
	if err != nil {
		fmt.Printf("failed to build the placement: %s\n", err)
		return
	}

	// The pin is placed twice from one body: in place and lifted clear of the plate.
	var out bytes.Buffer
	err = export.STEPItems(ctx, &out, []export.STEPItem{
		{Name: partNamePlate, Body: plate, Placement: r3.Identity(), Color: &color.RGBA{R: 160, G: 160, B: 170, A: 255}},
		{Name: partNamePin, Body: pin, Placement: r3.Identity()},
		{Name: partNamePin + "-lifted", Body: pin, Placement: lift},
	}, units.Millimeters(0.1),
		export.WithSTEPName("assembly.step"),
		export.WithSTEPAuthor("Example"),
		export.WithSTEPOrganization("Example"))
	if err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	text := out.String()
	fmt.Printf("products: %d\n", strings.Count(text, "=PRODUCT("))
	fmt.Printf("occurrences: %d\n", strings.Count(text, "=NEXT_ASSEMBLY_USAGE_OCCURRENCE("))
	fmt.Printf("coloured solids: %d\n", strings.Count(text, "=STYLED_ITEM("))
	// Output:
	// products: 3
	// occurrences: 3
	// coloured solids: 1
}
