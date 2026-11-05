package examples_test

import (
	"context"
	"fmt"
	"os"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
)

// A Set names the parts of a build and the joints between them. Explode pulls
// each part away from its neighbour along the joint that holds it, and Graph
// draws which part joins which, as a Mermaid diagram.
func Example_parts_explode() {
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
	if err := set.Join(parts.Joint{Kind: parts.PinInBore, A: partNamePin, B: partNamePlate, Motion: parts.Turns}); err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}
	ex, err := set.Explode(ctx, parts.WithExplodeGround(partNamePlate))
	if err != nil {
		fmt.Printf("failed to explode: %s\n", err)
		return
	}
	for _, p := range ex.Parts {
		fmt.Printf("%s: offset (%g, %g, %g), pulled by %s\n", p.Name, p.Offset.X, p.Offset.Y, p.Offset.Z, p.Pull)
	}
	if err := set.Graph(nil).WriteMermaid(os.Stdout); err != nil {
		fmt.Printf("failed to write the diagram: %s\n", err)
		return
	}
	// Output:
	// plate: offset (0, 0, 0), pulled by none
	// pin: offset (0, 0, 22), pulled by axis from faces
	// flowchart LR
	//   plate["plate"]
	//   pin["pin"]
	//   pin -- "pin in bore, turns" --> plate
}
