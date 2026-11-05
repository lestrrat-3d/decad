package examples_test

import (
	"context"
	"fmt"
	"os"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/units"
)

// Summary writes the build as one Markdown page an agent can read: the bill of
// materials, the joints, the exploded layout and the connection diagram. Parts
// that share a Spec are one row.
func Example_parts_summary() {
	ctx := context.Background()
	doc := decad.New()
	plate, err := extrudeBlock(doc, 0, 0, 20, 20, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	petg := parts.Material{Name: "PETG", Density: units.GramsPerCubicCentimeter(1.27)}
	set := parts.NewSet(doc)
	if err := set.Add(parts.Part{Name: partNamePlate, Kind: parts.Manufactured, Body: plate, Material: petg}); err != nil {
		fmt.Printf("failed to add the plate: %s\n", err)
		return
	}
	// The screws are bought and only listed: they have no body.
	if err := set.Add(parts.Part{Name: "screw", Kind: parts.Purchased, Quantity: 4, Spec: "M3 x 10 screw"}); err != nil {
		fmt.Printf("failed to add the screws: %s\n", err)
		return
	}
	if err := set.Join(parts.Joint{Kind: parts.Tie, A: "screw", B: partNamePlate}); err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}
	ex, err := set.Explode(ctx)
	if err != nil {
		fmt.Printf("failed to explode: %s\n", err)
		return
	}
	if err := (parts.Summary{Set: set, Exploded: ex}).WriteMarkdown(os.Stdout); err != nil {
		fmt.Printf("failed to write the summary: %s\n", err)
		return
	}
	// Output:
	// # Build summary
	//
	// ## Bill of materials
	//
	// | Spec | Kind | Quantity | Material | Volume | Mass | Parts |
	// |---|---|---|---|---|---|---|
	// | plate | manufactured | 1 | PETG | 4000.00 mm³ | 5.08 g | plate |
	// | M3 x 10 screw | purchased | 4 | - | - | - | screw |
	//
	// Volume is that of one part. Mass is the solid body's mass: infill lowers it.
	//
	// ## Exploded view
	//
	// Ground: plate.
	//
	// | Part | Depth | Pull | Stage | Offset (mm) | Moved (mm) | Clearance (mm) | Path |
	// |---|---|---|---|---|---|---|---|
	// | plate | 0 | none | - | (0.00, 0.00, 0.00) | 0.00 | 0.00 | none |
	//
	// ## Connection diagram
	//
	// ```mermaid
	// flowchart LR
	//   plate["plate"]
	//   screw("screw")
	//   screw -- "tie" --> plate
	// ```
}
