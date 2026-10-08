package examples_test

import (
	"context"
	"fmt"
	"sort"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithAsymmetricChamfer also sets a complete cap loop back by two distances
// (docs/modify-reach-design.md §8.3.1). The reference face picks which one
// runs where: naming the cap face puts the positional distance across the cap
// and the other distance down the side walls; naming the side walls swaps
// them.
func Example_decad_asymmetric_cap_chamfer() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate, extruded 20 mm.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The end cap's rim loop: 3 mm across the end cap, which the reference
	// names, and 6 mm down the side walls.
	loop := decad.Edges(decad.CreatedBy(decad.CapEnd(box)))
	capFace := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box)))
	body, err := box.Chamfer(context.Background(), loop, units.Millimeters(3),
		decad.WithAsymmetricChamfer(capFace, units.Millimeters(6)))
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

	// The cap face's corners sit 3 mm in from the rim; the side walls stop
	// 6 mm below the cap.
	caps, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).SelectFaces(body)
	if err != nil {
		fmt.Printf("failed to select the cap: %s\n", err)
		return
	}
	var corners []string
	for _, ce := range caps[0].Loops()[0].CoEdges() {
		p := ce.Start().Position().Value
		corners = append(corners, fmt.Sprintf("(%g, %g)", p.X, p.Y))
	}
	sort.Strings(corners)
	levels := map[float64]bool{}
	for _, v := range body.Vertices() {
		levels[v.Position().Value.Z] = true
	}
	var zs []float64
	for z := range levels {
		zs = append(zs, z)
	}
	sort.Float64s(zs)

	fmt.Printf("volume: %g mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("cap corners: %s\n", strings.Join(corners, " "))
	fmt.Printf("vertex levels: %v\n", zs)
	// Output:
	// volume: 117192 mm^3 (Exact)
	// cap corners: (3, 3) (3, 57) (97, 3) (97, 57)
	// vertex levels: [0 14 20]
}
