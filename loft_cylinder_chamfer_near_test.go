package decad

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

func TestChamferSplitRingGridContainsEveryExactReachCandidate(t *testing.T) {
	point := func(x, y float64) proofarith.Xpt {
		return proofarith.XptOf(r3.Vec{X: x, Y: y})
	}
	edges := []loftCylinderChamferSplitEdge{
		{a: point(-2, -1), b: point(-1, -1)},
		{a: point(-0.25, 0), b: point(0.25, 0)},
		{a: point(1, -0.5), b: point(1, 0.5)},
		{a: point(-1, 1), b: point(1, 1)},
	}
	reach := big.NewRat(1, 8)
	grid, err := loftCylinderChamferEdgeGridOf(edges, reach)
	if err != nil {
		t.Fatal(err)
	}
	reach2 := new(big.Rat).Mul(reach, reach)
	for ix := -16; ix <= 16; ix++ {
		for iy := -16; iy <= 16; iy++ {
			x, y := float64(ix)/8, float64(iy)/8
			tri := [3]proofarith.Xpt{
				point(x, y), point(x+1.0/128, y), point(x, y+1.0/128),
			}
			candidates := grid.candidates(tri)
			for index, edge := range edges {
				if !loftCylinderChamferPointEdgeWithin(tri[0], edge, reach2) ||
					!loftCylinderChamferPointEdgeWithin(tri[1], edge, reach2) ||
					!loftCylinderChamferPointEdgeWithin(tri[2], edge, reach2) {
					continue
				}
				found := false
				for _, candidate := range candidates {
					found = found || candidate == index
				}
				if !found {
					t.Fatalf("grid omitted reachable edge %d at (%g,%g)", index, x, y)
				}
			}
		}
	}
}
