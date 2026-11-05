package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Check compares each declared joint with the faces of its two parts. It never
// runs a pair proof, so it is quick on a whole build, and a joint it cannot read
// is reported as undecided with a cause rather than guessed.
func Example_parts_joints() {
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
	// The pin turns in its bore, with 0.2 mm all round.
	err = set.Join(parts.Joint{Kind: parts.PinInBore, A: partNamePin, B: partNamePlate, Motion: parts.Turns,
		Clearance: units.Millimeters(0.2)})
	if err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}
	report, err := set.Check(ctx)
	if err != nil {
		fmt.Printf("failed to check: %s\n", err)
		return
	}
	for _, r := range report.Joints {
		fmt.Printf("%s: %s, radial clearance %.2f mm\n", r.Joint.Name, r.Status, r.Measured.Value.Base())
	}
	fmt.Printf("%d unexplained contacts\n", len(report.Unexplained))
	// Output:
	// pin~plate: confirmed, radial clearance 0.20 mm
	// 0 unexplained contacts
}
