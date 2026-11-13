package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A derived explode of a thin assembly keeps every part on the axis it assembles
// on, in removal order along it. When a part would cover a part it is joined to,
// its layer steps sideways along Spread.Shear, just far enough to show the
// covered part. Guides gives each part a line from where it assembles on its
// joint partner; Inline marks the lines a reader does not need.
func Example_parts_explode_spread() {
	doc := decad.New()
	cut := func(b, tool *decad.Body, err error) (*decad.Body, error) {
		if err != nil {
			return nil, err
		}
		return decad.Cut(context.Background(), b, tool)
	}
	cyl := func(x, y, r, z0, h float64) (*decad.Body, error) {
		return extrudeCircle(doc, r3.Vec{X: x, Y: y}, r, z0, h)
	}
	// drill cuts a hole of the given centre and radius from z0, h tall, through body.
	type hole struct{ x, y, r float64 }
	drill := func(body *decad.Body, z0, h float64, holes ...hole) (*decad.Body, error) {
		for _, ho := range holes {
			tool, err := cyl(ho.x, ho.y, ho.r, z0, h)
			if body, err = cut(body, tool, err); err != nil {
				return nil, err
			}
		}
		return body, nil
	}

	// A back plate with an arbor behind it, a wheel on the arbor, a bridge over the wheel and one screw
	// that holds the bridge, off the arbor's axis.
	plate, err := extrudeBlock(doc, -60, -40, 60, 40, 0, 6)
	if err == nil {
		plate, err = drill(plate, -1, 8, hole{0, 0, 2.5})
	}
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	bridge, err := extrudeBlock(doc, -50, -20, 50, 20, 15, 3)
	if err == nil {
		bridge, err = drill(bridge, 14, 5, hole{0, 0, 2.5}, hole{45, 15, 1.6})
	}
	if err != nil {
		fmt.Printf("failed to build the bridge: %s\n", err)
		return
	}
	arbor, err := cyl(0, 0, 2, -10, 32)
	if err != nil {
		fmt.Printf("failed to build the arbor: %s\n", err)
		return
	}
	collar, err := cyl(0, 0, 4, -10, 4)
	if err != nil {
		fmt.Printf("failed to build the collar: %s\n", err)
		return
	}
	if arbor, err = decad.Union(context.Background(), arbor, collar); err != nil {
		fmt.Printf("failed to join the collar: %s\n", err)
		return
	}
	wheel, err := cyl(0, 0, 12, 9, 3)
	if err == nil {
		wheel, err = drill(wheel, 8, 5, hole{0, 0, 2.5})
	}
	if err != nil {
		fmt.Printf("failed to build the wheel: %s\n", err)
		return
	}
	shank, err := cyl(45, 15, 1.5, 6, 12)
	if err != nil {
		fmt.Printf("failed to build the screw: %s\n", err)
		return
	}
	head, err := cyl(45, 15, 3, 18, 2)
	if err != nil {
		fmt.Printf("failed to build the screw head: %s\n", err)
		return
	}
	screw, err := decad.Union(context.Background(), shank, head)
	if err != nil {
		fmt.Printf("failed to join the screw head: %s\n", err)
		return
	}

	// A dial over the bridge, wider than it.
	dial, err := extrudeBlock(doc, -60, -40, 60, 40, 23, 2)
	if err != nil {
		fmt.Printf("failed to build the dial: %s\n", err)
		return
	}

	set := parts.NewSet(doc)
	for _, p := range []parts.Part{
		{Name: "plate", Kind: parts.Manufactured, Body: plate},
		{Name: "arbor", Kind: parts.Manufactured, Body: arbor},
		{Name: "wheel", Kind: parts.Manufactured, Body: wheel},
		{Name: "bridge", Kind: parts.Manufactured, Body: bridge},
		{Name: "screw", Kind: parts.Purchased, Body: screw},
		{Name: "dial", Kind: parts.Manufactured, Body: dial},
	} {
		if err := set.Add(p); err != nil {
			fmt.Printf("failed to add %s: %s\n", p.Name, err)
			return
		}
	}

	// Each part is joined by a pin whose axis runs along Z through the part's centre.
	zAxis := func(x, y float64) *parts.Axis { return &parts.Axis{Point: r3.Vec{X: x, Y: y}, Dir: r3.Vec{Z: 1}} }
	for _, j := range []parts.Joint{
		{Name: "arbor in plate", Kind: parts.PinInBore, A: "arbor", B: "plate", Axis: zAxis(0, 0)},
		{Name: "wheel on arbor", Kind: parts.PinInBore, A: "wheel", B: "arbor", Axis: zAxis(0, 0)},
		{Name: "arbor in bridge", Kind: parts.PinInBore, A: "arbor", B: "bridge", Axis: zAxis(0, 0)},
		{Name: "screw in bridge", Kind: parts.PinInBore, A: "screw", B: "bridge", Axis: zAxis(45, 15)},
		{Name: "dial on bridge", Kind: parts.Contact, A: "dial", B: "bridge"},
	} {
		if err := set.Join(j); err != nil {
			fmt.Printf("failed to join %s: %s\n", j.Name, err)
			return
		}
	}

	// The wheel and the dial are fixed to +Z: with the arbor gone nothing stands beside the wheel, so the
	// search would let it slide out sideways too.
	ex, err := set.Explode(context.Background(), parts.WithDerivedExplode(), parts.WithExplodeGround("plate"),
		parts.WithExplodeGap(units.Millimeters(5)),
		parts.WithRemoval("wheel", parts.Removal{Direction: r3.Vec{Z: 1}}),
		parts.WithRemoval("dial", parts.Removal{Direction: r3.Vec{Z: 1}}))
	if err != nil {
		fmt.Printf("failed to explode: %s\n", err)
		return
	}
	sp := ex.Spread
	fmt.Printf("spread ran: %t, axis (%g, %g, %g), %d layers, steps along (%g, %g, %g)\n", sp.Ran, sp.Axis.X, sp.Axis.Y,
		sp.Axis.Z, sp.Layers, sp.Shear.X, sp.Shear.Y, sp.Shear.Z)

	// rounded keeps the printed numbers stable to the hundredth of a millimetre.
	rounded := func(v r3.Vec) string {
		r := func(x float64) float64 { return math.Round(x*100)/100 + 0 }
		return fmt.Sprintf("(%g, %g, %g)", r(v.X), r(v.Y), r(v.Z))
	}
	for _, p := range ex.Parts {
		fmt.Printf("%s: layer %d, slide %s\n", p.Name, p.Layer, rounded(p.Slide))
	}
	for _, g := range ex.Guides {
		if g.Inline {
			continue
		}
		// The guide runs from where the dial assembles on its bridge, as the bridge sits in the exploded
		// layout, to the dial's own place: across the axis, because the dial's layer stepped.
		fmt.Printf("%s guide: to %s along %q, from %s to %s, size %g mm\n", g.Part, g.Partner, g.Joint, rounded(g.From),
			rounded(g.To), math.Round(g.Size.Base()*100)/100)
	}
	// Output:
	// spread ran: true, axis (0, 0, 1), 6 layers, steps along (0, 1, 0)
	// plate: layer 0, slide (0, 0, 0)
	// arbor: layer -1, slide (0, 0, 0)
	// wheel: layer 1, slide (0, 0, 0)
	// bridge: layer 2, slide (0, 0, 0)
	// screw: layer 3, slide (0, 0, 0)
	// dial: layer 4, slide (0, 25, 0)
	// dial guide: to bridge along "dial on bridge", from (0, 0, 28) to (0, 25, 47), size 120 mm
}
