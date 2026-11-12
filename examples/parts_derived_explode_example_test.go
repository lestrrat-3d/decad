package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
)

// WithDerivedExplode reads the removal order from the tessellated bodies. The
// shaft's collar is wider than the bearing's bore and the carrier's bore, so the
// bearing leaves first, downward, and the shaft follows it the same way.
func Example_parts_derived_explode() {
	ctx := context.Background()
	doc := decad.New()
	axis := r3.Vec{X: 10, Y: 10}
	carrier, err := extrudeBlock(doc, 0, 0, 20, 20, 16, 4)
	if err != nil {
		fmt.Printf("failed to build the carrier: %s\n", err)
		return
	}
	carrierBore, err := extrudeCircle(doc, axis, 3, 15, 6)
	if err != nil {
		fmt.Printf("failed to build the carrier bore: %s\n", err)
		return
	}
	if carrier, err = decad.Cut(ctx, carrier, carrierBore); err != nil {
		fmt.Printf("failed to bore the carrier: %s\n", err)
		return
	}
	stem, err := extrudeCircle(doc, axis, 2, 0, 30)
	if err != nil {
		fmt.Printf("failed to build the stem: %s\n", err)
		return
	}
	collar, err := extrudeCircle(doc, axis, 5, 12, 2)
	if err != nil {
		fmt.Printf("failed to build the collar: %s\n", err)
		return
	}
	shaft, err := decad.Union(ctx, stem, collar)
	if err != nil {
		fmt.Printf("failed to join the collar: %s\n", err)
		return
	}
	bearing, err := extrudeCircle(doc, axis, 6, 8, 3)
	if err != nil {
		fmt.Printf("failed to build the bearing: %s\n", err)
		return
	}
	bearingBore, err := extrudeCircle(doc, axis, 2.5, 7, 5)
	if err != nil {
		fmt.Printf("failed to build the bearing bore: %s\n", err)
		return
	}
	if bearing, err = decad.Cut(ctx, bearing, bearingBore); err != nil {
		fmt.Printf("failed to bore the bearing: %s\n", err)
		return
	}

	set := parts.NewSet(doc)
	for _, p := range []parts.Part{
		{Name: "carrier", Kind: parts.Manufactured, Body: carrier},
		{Name: "shaft", Kind: parts.Manufactured, Body: shaft},
		{Name: "bearing", Kind: parts.Purchased, Body: bearing},
	} {
		if err := set.Add(p); err != nil {
			fmt.Printf("failed to add %s: %s\n", p.Name, err)
			return
		}
	}
	ex, err := set.Explode(ctx, parts.WithDerivedExplode(), parts.WithExplodeGround("carrier"))
	if err != nil {
		fmt.Printf("failed to explode: %s\n", err)
		return
	}
	for _, p := range ex.Parts {
		fmt.Printf("%s: %s, direction (%g, %g, %g)\n", p.Name, p.Stage, p.Dir.X, p.Dir.Y, p.Dir.Z)
	}
	// The report: the layout is derived, so it is layout-grade, and it lists the
	// parts in removal order and the pairs that forced that order.
	fmt.Printf("derived: %t\n", ex.Derived)
	byOrder := make([]string, len(ex.Parts))
	for _, p := range ex.Parts {
		if p.Order > 0 {
			byOrder[p.Order-1] = fmt.Sprintf("%s (round %d, %s)", p.Name, p.Round, p.Pull)
		}
	}
	for i, line := range byOrder {
		if line != "" {
			fmt.Printf("order %d: %s\n", i+1, line)
		}
	}
	for _, b := range ex.Blocked {
		fmt.Printf("%s is blocked by %s along (%g, %g, %g)\n", b.Mover, b.Blocker, b.Dir.X, b.Dir.Y, b.Dir.Z)
	}
	// Output:
	// carrier: , direction (0, 0, 0)
	// shaft: removal 2, direction (0, 0, -1)
	// bearing: removal 1, direction (0, 0, -1)
	// derived: true
	// order 1: bearing (round 1, world axis)
	// order 2: shaft (round 2, world axis)
	// shaft is blocked by bearing along (0, 0, -1)
}
