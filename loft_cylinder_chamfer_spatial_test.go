package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
)

func TestChamferMiddleGridKeepsEveryOverlappingBox(t *testing.T) {
	boxes := [][2]r3.Vec{
		{{X: -2, Y: -1, Z: 0}, {X: -1, Y: 0, Z: 2}},
		{{X: -0.5, Y: -0.5, Z: 0}, {X: 0.5, Y: 0.5, Z: 2}},
		{{X: 1, Y: 0, Z: 0}, {X: 2, Y: 1, Z: 2}},
		{{X: -100, Y: -100, Z: 0}, {X: 100, Y: 100, Z: 0}},
	}
	grid, err := loftCylinderChamferBoxGridOf(boxes, 0, 2)
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
