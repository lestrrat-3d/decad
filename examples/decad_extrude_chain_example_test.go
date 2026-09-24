package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ExtrudeChain sweeps an open sketch chain — an ordered run of boundary
// edges whose two ends are free, sketch.Sketch.Chains' own counterpart of a
// closed Profile — into a ribbon: one wall face per recorded segment, with
// no cap and no closing face. The result is always a sheet body, since an
// open walk encloses no region to close. A line-arc-line chain here builds a
// three-wall ribbon; the arc wall's area carries its own evaluation bound,
// while the two straight walls stay exact.
func Example_decad_extrude_chain() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(15, 5)
	d := s.CreatePoint(25, 5)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateArc(s.CreatePoint(10, 5), b, c)
	s.CreateLine(c, d)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The chain is the sketch's one open connected run; the closed-region
	// path (Profiles) sees nothing here.
	chains := s.Chains()
	fmt.Printf("profiles: %d, chains: %d, edges: %d\n", len(s.Profiles()), len(chains), len(chains[0].Edges))

	doc := decad.New()
	body, err := doc.ExtrudeChain(s, chains[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude chain: %s\n", err)
		return
	}

	area, err := body.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	surface, err := area.Value.In(units.SquareMillimeter)
	if err != nil {
		fmt.Printf("failed to convert area: %s\n", err)
		return
	}

	free, err := decad.Edges(decad.Free()).SelectEdges(body)
	if err != nil {
		fmt.Printf("failed to select free edges: %s\n", err)
		return
	}

	fmt.Printf("sheet: %v, solid: %v, faces: %d\n", body.Kind() == decad.BodySheet, body.IsSolid(), len(body.Faces()))
	fmt.Printf("free edges: %d\n", len(free))
	fmt.Printf("area: %.4f mm² (%s)\n", surface, area.Exactness)
	// Output:
	// profiles: 0, chains: 1, edges: 3
	// sheet: true, solid: false, faces: 3
	// free edges: 8
	// area: 278.5398 mm² (Approximate)
}
