package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Stages take an assembly apart in order. Each stage's parts move along their
// joints, or along a stated direction, and a ClearAll stage moves them until they
// clear every part still standing, so a mechanism comes out of the housing that
// encloses it instead of staying inside it.
func Example_parts_explode_stages() {
	ctx := context.Background()
	doc := decad.New()
	centre := r3.Vec{X: 20, Y: 10}
	// block is a box with an optional pocket and a Ø4.2 bore cut into it.
	block := func(x0, y0, x1, y1, z0, h float64, pocket [6]float64, bz0, bh float64) (*decad.Body, error) {
		b, err := extrudeBlock(doc, x0, y0, x1, y1, z0, h)
		if err != nil {
			return nil, err
		}
		if pocket != ([6]float64{}) {
			tool, err := extrudeBlock(doc, pocket[0], pocket[1], pocket[2], pocket[3], pocket[4], pocket[5])
			if err != nil {
				return nil, err
			}
			if b, err = decad.Cut(context.Background(), b, tool); err != nil {
				return nil, err
			}
		}
		bore, err := extrudeCircle(doc, centre, 2.1, bz0, bh)
		if err != nil {
			return nil, err
		}
		return decad.Cut(context.Background(), b, bore)
	}
	bottom, err := block(0, 0, 40, 20, -10, 10, [6]float64{10, 4, 30, 16, -6, 7}, -11, 12)
	if err != nil {
		fmt.Printf("failed to build the bottom half: %s\n", err)
		return
	}
	top, err := block(0, 0, 40, 20, 0, 10, [6]float64{10, 4, 30, 16, -1, 7}, -1, 12)
	if err != nil {
		fmt.Printf("failed to build the top half: %s\n", err)
		return
	}
	lever, err := block(14, 7, 26, 13, -4, 8, [6]float64{}, -5, 10)
	if err != nil {
		fmt.Printf("failed to build the lever: %s\n", err)
		return
	}
	pin, err := extrudeCircle(doc, centre, 2, -12, 24)
	if err != nil {
		fmt.Printf("failed to build the pin: %s\n", err)
		return
	}

	set := parts.NewSet(doc)
	for _, p := range []parts.Part{
		{Name: "housing-bottom", Kind: parts.Manufactured, Body: bottom},
		{Name: "housing-top", Kind: parts.Manufactured, Body: top},
		{Name: "lever", Kind: parts.Manufactured, Body: lever},
		{Name: partNamePin, Kind: parts.Manufactured, Body: pin},
	} {
		if err := set.Add(p); err != nil {
			fmt.Printf("failed to add %s: %s\n", p.Name, err)
			return
		}
	}
	for _, j := range []parts.Joint{
		{Kind: parts.FaceContact, A: "housing-top", B: "housing-bottom"},
		{Kind: parts.PinInBore, A: partNamePin, B: "housing-top"},
		{Kind: parts.PinInBore, A: partNamePin, B: "housing-bottom"},
		{Kind: parts.PinInBore, A: partNamePin, B: "lever", Motion: parts.Turns},
	} {
		if err := set.Join(j); err != nil {
			fmt.Printf("failed to join: %s\n", err)
			return
		}
	}

	// One stage: the lever comes off its pin but stays in the bottom half's box.
	flat, err := set.Explode(ctx, parts.WithExplodeGround("housing-bottom"))
	if err != nil {
		fmt.Printf("failed to explode: %s\n", err)
		return
	}
	fmt.Printf("one stage: %d overlap(s) %v\n", len(flat.Overlaps), flat.Overlaps)

	staged, err := set.Explode(ctx, parts.WithExplodeGround("housing-bottom"), parts.WithExplodeStages(
		parts.Stage{Name: "shell", Parts: []string{"housing-top"}},
		parts.Stage{Name: "mechanism", Parts: []string{"lever"}, Clear: parts.ClearAll, Direction: r3.Vec{Z: 1}},
		parts.Stage{Name: "pins", Parts: []string{partNamePin}, Clear: parts.ClearAll, Gap: units.Millimeters(10)},
	))
	if err != nil {
		fmt.Printf("failed to explode in stages: %s\n", err)
		return
	}
	for _, p := range staged.Parts {
		path := p.Path.String()
		if p.Path == parts.PathEnclosed {
			path += " by " + p.Blocker
		}
		fmt.Printf("%s: stage %q, offset (%.0f, %.0f, %.0f), path %s\n",
			p.Name, p.Stage, p.Offset.X, p.Offset.Y, p.Offset.Z, path)
	}
	fmt.Printf("staged: %d overlap(s), %d short\n", len(staged.Overlaps), len(staged.Short))
	// Output:
	// one stage: 1 overlap(s) [[housing-bottom lever]]
	// housing-bottom: stage "", offset (0, 0, 0), path none
	// housing-top: stage "shell", offset (0, 0, 10), path enclosed by lever
	// lever: stage "mechanism", offset (0, 0, 34), path enclosed by housing-bottom
	// pin: stage "pins", offset (0, 0, -32), path clear
	// staged: 0 overlap(s), 0 short
}
