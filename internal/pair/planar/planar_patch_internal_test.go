package planar

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestLocateRayAndBoundary(t *testing.T) {
	t.Parallel()
	point := func(x, y int64) Point2 { return Point2{X: big.NewRat(x, 1), Y: big.NewRat(y, 1)} }
	square := []Point2{point(0, 0), point(8, 0), point(8, 8), point(0, 8)}
	hole := []Point2{point(2, 2), point(6, 2), point(6, 6), point(2, 6)}
	triangle := []Point2{point(0, 0), point(4, 4), point(8, 0)}
	tests := []struct {
		name  string
		at    Point2
		loops [][]Point2
		want  int
	}{
		{name: "inside", at: point(1, 3), loops: [][]Point2{square}, want: 1},
		{name: "outside", at: point(9, 3), loops: [][]Point2{square}, want: -1},
		{name: "bottom edge", at: point(4, 0), loops: [][]Point2{square}, want: 0},
		{name: "top edge", at: point(4, 8), loops: [][]Point2{square}, want: 0},
		{name: "left edge", at: point(0, 4), loops: [][]Point2{square}, want: 0},
		{name: "right edge", at: point(8, 4), loops: [][]Point2{square}, want: 0},
		{name: "upper vertex", at: point(8, 8), loops: [][]Point2{square}, want: 0},
		{name: "horizontal extension", at: point(9, 8), loops: [][]Point2{square}, want: -1},
		{name: "rising edge", at: point(2, 2), loops: [][]Point2{triangle}, want: 0},
		{name: "falling edge", at: point(6, 2), loops: [][]Point2{triangle}, want: 0},
		{name: "triangle inside", at: point(4, 1), loops: [][]Point2{triangle}, want: 1},
		{name: "triangle outside", at: point(4, 5), loops: [][]Point2{triangle}, want: -1},
		{name: "inside hole", at: point(4, 4), loops: [][]Point2{square, hole}, want: -1},
		{name: "hole edge", at: point(2, 4), loops: [][]Point2{square, hole}, want: 0},
		{name: "outside hole", at: point(1, 1), loops: [][]Point2{square, hole}, want: 1},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			require.Equal(t, tt.want, locate(tt.at, tt.loops))
		})
	}
}
