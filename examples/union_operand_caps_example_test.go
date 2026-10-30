package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
)

// After a Union, a face that lies inside exactly one operand face still answers
// to that operand's roles, so CapEnd(boss) keeps naming the boss's top and
// CapEnd(plate) the plate's top around it. A face both operands share, such as
// a floor two flush blocks stand on, names neither operand.
func Example_union_operandCaps() {
	doc := decad.New()
	// An 80 x 60 x 10 mm plate with a 20 x 20 mm boss standing 15 mm on it
	// (extrudeBlock is in helpers_test.go).
	plate, err := extrudeBlock(doc, 0, 0, 80, 60, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	boss, err := extrudeBlock(doc, 30, 20, 50, 40, 10, 15)
	if err != nil {
		fmt.Printf("failed to build the boss: %s\n", err)
		return
	}
	part, err := decad.Union(context.Background(), plate, boss)
	if err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}

	for _, c := range []struct {
		name string
		ref  decad.FeatureRef
	}{{"boss top", decad.CapEnd(boss)}, {"plate top", decad.CapEnd(plate)}} {
		faces, err := decad.Faces(decad.FaceCreatedBy(c.ref)).Exactly(1).SelectFaces(part)
		if err != nil {
			fmt.Printf("failed to select the %s: %s\n", c.name, err)
			return
		}
		area, err := faces[0].Area()
		if err != nil {
			fmt.Printf("failed to measure the %s: %s\n", c.name, err)
			return
		}
		fmt.Printf("%s: %.0f mm^2\n", c.name, area.Value.Base())
	}

	// The boss top's outer loop is the four edges a chamfer of its rim takes.
	rim, err := decad.Edges(decad.OuterLoopOf(decad.CapEnd(boss))).Exactly(4).SelectEdges(part)
	if err != nil {
		fmt.Printf("failed to select the rim: %s\n", err)
		return
	}
	fmt.Printf("rim edges: %d at z = %.0f\n", len(rim), rim[0].Start().Position().Value.Z)
	// Output:
	// boss top: 400 mm^2
	// plate top: 4400 mm^2
	// rim edges: 4 at z = 25
}
