package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Sweep moves an open sketch chain along a spatial Path, and publishes
// the ribbon it sweeps out: one wall face per recorded segment per span, no cap and no
// closing face, so the result is always a sheet body. This evaluator sweeps
// one straight path span, which needs no join between two spans. One exact
// cardinal line/arc path over one radial source line also builds. The
// multi-segment composite path below and a standalone arc still refuse.
//
// The height comes from the path's own two points rather than from an Extent,
// which is the whole difference from chain Extrude over the same walk.
func Example_decad_sweep_chain() {
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
	chain := s.Chains()[0]

	// The path starts in the sketch plane and leaves it along that plane's
	// positive normal, which for the XY plane is +Z.
	path, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, 0, 10)})
	if err != nil {
		fmt.Printf("failed to build path: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := decad.Sweep(context.Background(), doc, s, chain, path)
	if err != nil {
		fmt.Printf("failed to sweep chain: %s\n", err)
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

	// This multi-segment chain falls outside the admitted one-line composite.
	composite, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 20)},
		decad.ArcThrough{Through: r3.NewVec(5, 0, 25), End: r3.NewVec(10, 0, 20)},
	)
	if err != nil {
		fmt.Printf("failed to build composite path: %s\n", err)
		return
	}
	_, err = decad.Sweep(context.Background(), doc, s, chain, composite)
	fmt.Printf("composite path: %v\n", err)
	fmt.Printf("bodies: %d\n", len(doc.Bodies()))

	// Output:
	// sheet: true, solid: false, faces: 3
	// free edges: 8
	// area: 278.5398 mm² (Approximate)
	// composite path: decad: not supported by the current evaluator: composite chain Sweep requires one XY line and a cardinal two-span path
	// bodies: 1
}
