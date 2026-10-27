package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B14's public-surface test: a circular pattern of a faceted solid
// that is a whole number of quarter turns about a world axis keeps every instance exact, so the joined pattern reads
// exactly.

// turnedArea counts the unit squares of the plane covered by the rectangle [x0,x1) x [y0,y1) and its copies turned
// about the origin by 4/count quarter turns each, the oracle the pattern's volume is checked against.
func turnedArea(x0, y0, x1, y1, count int) int {
	covered := map[[2]int]bool{}
	for x := x0; x < x1; x++ {
		for y := y0; y < y1; y++ {
			cx, cy := x, y
			for range count {
				covered[[2]int{cx, cy}] = true
				for range 4 / count {
					// A quarter turn sends the cell with lower corner (x, y) to the one with lower corner (-y-1, x).
					cx, cy = -cy-1, cx
				}
			}
		}
	}
	return len(covered)
}

func TestCircularPatternOfAMeshSolidReadsExactly(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                  string
		x0, y0, x1, y1, count int
	}{
		{"pinwheel of four bars", 0, 0, 10, 4, 4},
		{"four overlapping bars", -2, -2, 10, 4, 4},
		{"two overlapping bars", -2, -2, 10, 4, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			b := blockIn(t, doc, r3.NewVec(float64(tc.x0), float64(tc.y0), 0), r3.NewVec(float64(tc.x1), float64(tc.y1), 10))
			out, err := decad.Patterned(t.Context(), b, decad.CircularPattern{
				Center: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1), Count: tc.count})
			require.NoError(t, err)
			requireVolume(t, out, float64(10*turnedArea(tc.x0, tc.y0, tc.x1, tc.y1, tc.count)))
		})
	}
}
