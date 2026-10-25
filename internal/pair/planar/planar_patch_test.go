package planar

import (
	"math/big"
	"slices"
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

// TestClipConvexPreservesExactAffineCoordinates exercises the point primitives
// through clipping, area and point location. The untransformed intersection
// is [(0,0), (1,0), (1,2/3), (0,1)], with double area 5/3. Exact affine maps
// preserve that intersection, including under reflection and either winding.
func TestClipConvexPreservesExactAffineCoordinates(t *testing.T) {
	t.Parallel()
	pow := func(e int) *big.Rat {
		if e >= 0 {
			return new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(e)))
		}
		return new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(-e)))
	}
	for _, exponent := range []int{-600, -31, 0, 67, 600} {
		for _, reflected := range []bool{false, true} {
			sx := new(big.Rat).Mul(pow(exponent), big.NewRat(5, 7))
			sy := new(big.Rat).Mul(pow(-exponent), big.NewRat(3, 11))
			if reflected {
				sx.Neg(sx)
			}
			tx, ty := pow(900), big.NewRat(-13, 17)
			point := func(x, y *big.Rat) Point2 {
				return Point2{
					X: new(big.Rat).Add(tx, new(big.Rat).Mul(sx, x)),
					Y: new(big.Rat).Add(ty, new(big.Rat).Mul(sy, y)),
				}
			}
			p := func(x, y int64) Point2 { return point(big.NewRat(x, 1), big.NewRat(y, 1)) }
			clip := []Point2{p(0, 0), p(1, 0), p(1, 1), p(0, 1)}
			// Repeated coordinates deliberately share storage across vertices.
			clip[3].X, clip[2].X = clip[0].X, clip[1].X
			clip[1].Y, clip[3].Y = clip[0].Y, clip[2].Y
			subject := []Point2{clip[0], p(3, 0), p(0, 1)}
			want := []Point2{p(0, 0), p(1, 0), point(big.NewRat(1, 1), big.NewRat(2, 3)), p(0, 1)}
			area := new(big.Rat).Mul(new(big.Rat).Abs(new(big.Rat).Mul(sx, sy)), big.NewRat(5, 3))
			var before []*big.Rat
			for _, polygon := range [][]Point2{subject, clip} {
				for _, vertex := range polygon {
					before = append(before, new(big.Rat).Set(vertex.X), new(big.Rat).Set(vertex.Y))
				}
			}
			for winding := range 4 {
				s, c := slices.Clone(subject), slices.Clone(clip)
				if winding&1 != 0 {
					slices.Reverse(s)
				}
				if winding&2 != 0 {
					slices.Reverse(c)
				}
				got, err := ClipConvex(s, c, noPollInternal)
				require.NoError(t, err)
				require.Len(t, got, len(want))
				require.Zero(t, area.Cmp(DoubleArea(got)), "exponent %d winding %d", exponent, winding)
				for index, expected := range want {
					matched := false
					for _, actual := range got {
						if expected.X.Cmp(actual.X) != 0 || expected.Y.Cmp(actual.Y) != 0 {
							continue
						}
						matched = true
						require.Zero(t, expected.X.Num().Cmp(actual.X.Num()), "x numerator, vertex %d", index)
						require.Zero(t, expected.X.Denom().Cmp(actual.X.Denom()), "x denominator, vertex %d", index)
						require.Zero(t, expected.Y.Num().Cmp(actual.Y.Num()), "y numerator, vertex %d", index)
						require.Zero(t, expected.Y.Denom().Cmp(actual.Y.Denom()), "y denominator, vertex %d", index)
					}
					require.True(t, matched, "missing vertex %d, exponent %d winding %d", index, exponent, winding)
				}
				require.Equal(t, 1, locate(point(big.NewRat(1, 4), big.NewRat(1, 4)), [][]Point2{got}))
				require.Equal(t, 0, locate(point(big.NewRat(1, 1), big.NewRat(1, 3)), [][]Point2{got}))
				require.Equal(t, -1, locate(p(2, 2), [][]Point2{got}))
			}
			index := 0
			for _, polygon := range [][]Point2{subject, clip} {
				for _, vertex := range polygon {
					require.Zero(t, before[index].Cmp(vertex.X), "input x changed at %d", index)
					require.Zero(t, before[index+1].Cmp(vertex.Y), "input y changed at %d", index+1)
					index += 2
				}
			}
		}
	}
}
