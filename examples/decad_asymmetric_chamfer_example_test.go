package examples_test

import (
	"context"
	"fmt"
	"sort"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithAsymmetricChamfer sets a chamfer back by two different distances
// (docs/modify-reach-design.md §6): the positional distance across the
// reference face and the option's other distance across the face beside it.
// The reference is a face query resolved against the body, so the call names
// which wall takes which setback without depending on how the edge is walked.
func Example_decad_asymmetric_chamfer() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 40 x 20 plate, extruded 10 mm.
	rect := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	plate, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The vertical edge at (40, 20): 3 mm down the x = 40 wall, which the
	// reference names, and 5 mm along the y = 20 wall.
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.NewVec(40, 20, 0))).Exactly(1)
	wall := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
	body, err := plate.Chamfer(context.Background(), edge, units.Millimeters(3),
		decad.WithAsymmetricChamfer(wall, units.Millimeters(5)))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}

	// The bevel's two feet on the z = 0 cap.
	var feet []string
	for _, f := range body.Faces() {
		bevel := false
		for _, o := range f.Origins() {
			bevel = bevel || strings.HasPrefix(o.Role, "chamfer(")
		}
		if !bevel {
			continue
		}
		seen := map[r3.Vec]bool{}
		for _, e := range f.Edges() {
			for _, v := range []*decad.Vertex{e.Start(), e.End()} {
				p := v.Position().Value
				if p.Z == 0 && !seen[p] {
					seen[p] = true
					feet = append(feet, fmt.Sprintf("(%g, %g)", p.X, p.Y))
				}
			}
		}
	}
	sort.Strings(feet)

	fmt.Printf("volume: %g mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("feet: %s\n", strings.Join(feet, " "))
	// Output:
	// volume: 7925 mm^3 (Exact)
	// feet: (35, 20) (40, 17)
}
