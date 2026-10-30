package brepfilletmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// twistRings is TwistRingCount's count once the band's spread is known. The
// expected counts are worked by hand: ⌈spread·π/(4·chord)⌉ against the
// sagitta count n, capped at 4096.
func TestTwistRings(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name          string
		n             int
		spread, chord float64
		want          int
	}{
		// 0.1·π/0.04 = 7.854.
		{"twist raises the count", 2, 0.1, 0.01, 8},
		{"sagitta count already covers the twist", 10, 0.1, 0.01, 10},
		{"no spread keeps the sagitta count", 4, 0, 0.1, 4},
		// 4·π/4e-9 ≈ 3.1e9 rings.
		{"cap", 2, 4, 1e-9, 4096},
		{"zero chord", 2, 4, 0, 2},
		{"negative chord", 2, 4, -0.1, 2},
		{"NaN chord", 2, 4, math.NaN(), 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, twistRings(tc.n, tc.spread, tc.chord))
		})
	}
}

// cellOffsetUpper charges a cell the smaller of |T|/4 and the height bound,
// and the height bound only where the cell projects strictly convex. The
// expected values are worked by hand from the corners, with
// T = vLo − vHi − wLo + wHi.
func TestCellOffsetUpper(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name               string
		floor              float64
		vLo, vHi, wLo, wHi r3.Vec
		want               float64
	}{
		// Chords of lengths 4 and 2 in the plane z = y: |T|/4 = 1/2, all of
		// it within the plane, and the fourth corner lies on the plane.
		{"planar trapezoid", 0, r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(1, 1, 1), r3.NewVec(3, 1, 1), 0},
		// The unit square with one corner lifted by 1/2: |T|/4 = 1/8 is
		// below the corner's height 1/2, so the twist bound stays.
		{"lifted corner", 0, r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(1, 1, 0.5), 0.125},
		// A planar cell whose boundary crosses itself is not a graph over
		// its plane, so it keeps |T|/4 = 1/2.
		{"bowtie", 0, r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 0), 0.5},
		// A |T|/4 of 1/2 at or below the running maximum 1 cannot raise it.
		{"below the floor", 1, r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(1, 1, 1), r3.NewVec(3, 1, 1), 0.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, cellOffsetUpper(tc.floor, tc.vLo, tc.vHi, tc.wLo, tc.wHi))
		})
	}
}
