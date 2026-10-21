package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Web adds a set of crossing ribs to a part. Ribs stiffen thin walls and
// pockets without adding much weight. You draw each rib as a line, and Web
// gives it thickness and height and joins it to the part. Use it for stiffening
// ribs in a housing or a bracket.
func Example_web() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()

	// The part is a 40 x 40 x 20 mm block with a pocket in it, which is what
	// the ribs will cross. The pocket is 30 x 30 mm and 15 mm deep, opening
	// at the top. The block and the cutting tool are made the same way: see
	// Example_cut_blind_pocket for how the pocket is cut.
	block, err := extrudeBlock(doc, 0, 0, 40, 40, 0, 20)
	if err != nil {
		fmt.Printf("failed to build block: %s\n", err)
		return
	}
	pocketTool, err := extrudeBlock(doc, 5, 5, 35, 35, 5, 15)
	if err != nil {
		fmt.Printf("failed to build pocket: %s\n", err)
		return
	}
	part, err := decad.Cut(ctx, block, pocketTool)
	if err != nil {
		fmt.Printf("failed to cut pocket: %s\n", err)
		return
	}

	// Draw the rib lines on a plane 4 mm up, which is 1 mm below the pocket
	// floor. That makes each rib start buried in the floor, so it fuses with
	// it. Each line runs from 3 mm into the wall at one side to 3 mm into the
	// wall at the other, so the rib ends are fused with the walls too. One
	// line runs along X and the other along Y, so the ribs cross.
	plane, err := w.CreateOffsetPlane(w.XY(), 4)
	if err != nil {
		fmt.Printf("failed to create rib plane: %s\n", err)
		return
	}
	ribs, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create rib sketch: %s\n", err)
		return
	}
	var lines []*sketch.Line
	for _, ends := range [][4]float64{{3, 20, 37, 20}, {20, 3, 20, 37}} {
		a := ribs.CreatePoint(ends[0], ends[1])
		b := ribs.CreatePoint(ends[2], ends[3])
		ribs.Fix(a)
		ribs.Fix(b)
		lines = append(lines, ribs.CreateLine(a, b))
	}
	if _, err := ribs.Solve(ctx); err != nil {
		fmt.Printf("failed to solve rib sketch: %s\n", err)
		return
	}

	// Make each rib 2 mm thick and 11 mm tall (reaching z = 15, which is 5 mm
	// below the top of the pocket).
	ribbed, err := decad.Web(ctx, part, ribs, lines, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to add web: %s\n", err)
		return
	}
	volume, err := ribbed.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 19660 mm^3
}
