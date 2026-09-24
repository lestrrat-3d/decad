package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// LoftChain rules a sheet between two OPEN sketch chains: two flat triangles
// per chord cell, no cap and no closing face, so the result is always a sheet
// body. Segment j of the first walk pairs with segment j of the second — an
// open walk has a first and a last segment, so there is no alignment offset to
// rotate and none to pass.
//
// The two sketch planes must be exactly parallel with the second on the first
// plane's positive side. That is what states which way the ribbon faces: each
// wall publishes the normal an ExtrudeChain wall over the same recorded
// segment publishes. Swapping the two arguments puts the planes the wrong way
// round, and the call says so instead of quietly building a ribbon facing the
// other way.
func Example_decad_loft_chain() {
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to offset plane: %s\n", err)
		return
	}

	lower, lowerChain, err := openWalk(w.XY(), w, [][2]float64{{0, 0}, {20, 0}, {20, 15}})
	if err != nil {
		fmt.Printf("failed to build the lower walk: %s\n", err)
		return
	}
	upper, upperChain, err := openWalk(top, w, [][2]float64{{0, 0}, {20, 0}, {20, 15}})
	if err != nil {
		fmt.Printf("failed to build the upper walk: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.LoftChain(context.Background(), lower, lowerChain, upper, upperChain)
	if err != nil {
		fmt.Printf("failed to loft chain: %s\n", err)
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

	// The same two walks the other way round: the second plane is then below
	// the first, which states no side for the ribbon between them.
	_, err = doc.LoftChain(context.Background(), upper, upperChain, lower, lowerChain)
	fmt.Printf("reversed argument order: %v\n", err)
	fmt.Printf("bodies: %d\n", len(doc.Bodies()))

	// Output:
	// sheet: true, solid: false, faces: 4
	// free edges: 6
	// area: 350.0000 mm² (Approximate)
	// reversed argument order: decad: not supported by the current evaluator: the second chain's plane does not lie on the first plane's positive side, so this evaluator has no stated positive side for the ribbon between them (docs/loft-design.md §16.2)
	// bodies: 1
}

// openWalk draws one open polyline on plane and returns its sketch with the
// single chain sketch publishes for it.
func openWalk(plane *sketch.Plane, w *sketch.World, pts [][2]float64) (*sketch.Sketch, *sketch.Chain, error) {
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, nil, err
	}
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range len(points) - 1 {
		s.CreateLine(points[i], points[i+1])
	}
	if _, err := s.Solve(context.Background()); err != nil {
		return nil, nil, err
	}
	chains := s.Chains()
	if len(chains) != 1 {
		return nil, nil, fmt.Errorf("expected one chain, got %d", len(chains))
	}
	return s, chains[0], nil
}
