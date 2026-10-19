package loftmesh

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestChamferSplitRingGridContainsEveryExactReachCandidate(t *testing.T) {
	point := func(x, y float64) proofarith.Xpt {
		return proofarith.XptOf(r3.Vec{X: x, Y: y})
	}
	edges := []ChamferSplitEdge{
		{A: point(-2, -1), B: point(-1, -1)},
		{A: point(-0.25, 0), B: point(0.25, 0)},
		{A: point(1, -0.5), B: point(1, 0.5)},
		{A: point(-1, 1), B: point(1, 1)},
	}
	reach := big.NewRat(1, 8)
	grid, err := ChamferEdgeGridOf(edges, reach)
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
				if !chamferPointEdgeWithin(tri[0], edge, reach2) ||
					!chamferPointEdgeWithin(tri[1], edge, reach2) ||
					!chamferPointEdgeWithin(tri[2], edge, reach2) {
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

func TestChamferMiddleGridKeepsEveryOverlappingBox(t *testing.T) {
	boxes := [][2]r3.Vec{
		{{X: -2, Y: -1, Z: 0}, {X: -1, Y: 0, Z: 2}},
		{{X: -0.5, Y: -0.5, Z: 0}, {X: 0.5, Y: 0.5, Z: 2}},
		{{X: 1, Y: 0, Z: 0}, {X: 2, Y: 1, Z: 2}},
		{{X: -100, Y: -100, Z: 0}, {X: 100, Y: 100, Z: 0}},
	}
	grid, err := ChamferBoxGridOf(boxes, 0, 2)
	if err != nil {
		t.Fatal(err)
	}
	for ix := -8; ix <= 8; ix++ {
		for iy := -8; iy <= 8; iy++ {
			x, y := float64(ix)/4, float64(iy)/4
			query := [2]r3.Vec{{X: x, Y: y, Z: 0.5}, {X: x + 0.5, Y: y + 0.5, Z: 1.5}}
			candidates, err := grid.candidates(query)
			if err != nil {
				t.Fatal(err)
			}
			seen := make(map[int]bool)
			for _, index := range candidates {
				if seen[index] {
					t.Fatalf("duplicate candidate %d", index)
				}
				seen[index] = true
			}
			for index, box := range boxes {
				if meshbool.BoxesOverlap(query, box) && !seen[index] {
					t.Fatalf("grid omitted overlapping box %d at (%g,%g)", index, x, y)
				}
			}
		}
	}
}

func TestChamferFarTriangles(t *testing.T) {
	xverts := []proofarith.Xpt{
		proofarith.XptOf(r3.Vec{X: 0, Y: 0, Z: 0}),
		proofarith.XptOf(r3.Vec{X: 1, Y: 0, Z: 0}),
		proofarith.XptOf(r3.Vec{X: 0, Y: 0, Z: 1}),
		proofarith.XptOf(r3.Vec{X: 1, Y: 0, Z: 1}),
	}
	capLevel := big.NewRat(1, 1)
	half := big.NewRat(1, 2)
	vecs := func(tris [][3]proofarith.Xpt) [][3]r3.Vec {
		out := make([][3]r3.Vec, len(tris))
		for i, tri := range tris {
			out[i] = [3]r3.Vec{tri[0].Vec(), tri[1].Vec(), tri[2].Vec()}
		}
		return out
	}

	t.Run("one cap vertex", func(t *testing.T) {
		got, err := ChamferFarTriangles([3]int{0, 1, 2}, xverts, capLevel, half)
		require.NoError(t, err)
		require.Equal(t, [][3]r3.Vec{
			{{X: 0, Y: 0, Z: 1}, {X: 0, Y: 0, Z: 0.5}, {X: 0.5, Y: 0, Z: 0.5}},
		}, vecs(got))
	})
	t.Run("two cap vertices", func(t *testing.T) {
		got, err := ChamferFarTriangles([3]int{0, 2, 3}, xverts, capLevel, half)
		require.NoError(t, err)
		require.Equal(t, [][3]r3.Vec{
			{{X: 0, Y: 0, Z: 1}, {X: 1, Y: 0, Z: 1}, {X: 0.5, Y: 0, Z: 0.5}},
			{{X: 0, Y: 0, Z: 1}, {X: 0.5, Y: 0, Z: 0.5}, {X: 0, Y: 0, Z: 0.5}},
		}, vecs(got))
	})
	for _, tc := range []struct {
		name     string
		tri      [3]int
		fraction *big.Rat
		text     string
	}{
		{"invalid vertex", [3]int{0, 1, 4}, half, "cap band split has an invalid vertex"},
		{"no side vertex", [3]int{2, 3, 2}, half, "cap band split lacks two axial levels"},
		{"zero fraction", [3]int{0, 1, 2}, big.NewRat(0, 1), "cap band has no interior split level"},
		{"unit fraction", [3]int{0, 1, 2}, big.NewRat(1, 1), "cap band has no interior split level"},
		{"nil fraction", [3]int{0, 1, 2}, nil, "cap band has no interior split level"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := ChamferFarTriangles(tc.tri, xverts, capLevel, tc.fraction)
			require.Nil(t, got)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.text)
		})
	}
}
