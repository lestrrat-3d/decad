package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
)

// Views poses each part alone for a part sheet: centred at the origin in the
// orientation it was modelled in, framed by its bounds, with a port at each
// joint. A renderer draws one tile per part and a callout at each port.
func Example_parts_views() {
	ctx := context.Background()
	doc := decad.New()
	plate, err := extrudeBlock(doc, 0, 0, 20, 20, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	bore, err := extrudeCircle(doc, r3.Vec{X: 10, Y: 10}, 3.2, -1, 12)
	if err != nil {
		fmt.Printf("failed to build the bore tool: %s\n", err)
		return
	}
	if plate, err = decad.Cut(ctx, plate, bore); err != nil {
		fmt.Printf("failed to bore the plate: %s\n", err)
		return
	}
	pin, err := extrudeCircle(doc, r3.Vec{X: 10, Y: 10}, 3, -2, 17)
	if err != nil {
		fmt.Printf("failed to build the pin: %s\n", err)
		return
	}
	set := parts.NewSet(doc)
	for _, p := range []parts.Part{
		{Name: partNamePlate, Kind: parts.Manufactured, Body: plate},
		{Name: partNamePin, Kind: parts.Manufactured, Body: pin},
	} {
		if err := set.Add(p); err != nil {
			fmt.Printf("failed to add %s: %s\n", p.Name, err)
			return
		}
	}
	if err := set.Join(parts.Joint{Kind: parts.PinInBore, A: partNamePin, B: partNamePlate}); err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}
	views, err := set.Views(ctx)
	if err != nil {
		fmt.Printf("failed to build the views: %s\n", err)
		return
	}
	for _, v := range views.Parts {
		fmt.Printf("%s: z %g to %g, label at z %g\n", v.Part, v.Bounds.Min.Z, v.Bounds.Max.Z, v.Anchor.Z)
		for _, p := range v.Ports {
			fmt.Printf("  port %s with %s at z %g, along (%g, %g, %g)\n", p.Joint, p.Other, p.Point.Z, p.Dir.X, p.Dir.Y, p.Dir.Z)
		}
	}
	// Output:
	// plate: z -5 to 5, label at z 5
	//   port pin~plate with pin at z 0, along (0, 0, 1)
	// pin: z -8.5 to 8.5, label at z 8.5
	//   port pin~plate with plate at z -1.5, along (0, 0, 1)
}
