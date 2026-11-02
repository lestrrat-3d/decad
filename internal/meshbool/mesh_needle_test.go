package meshbool

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestFlipNeedles holds the needle pass to surface-preserving flips: a facet
// whose apex sits 2^-50 off its longest edge, beside a coplanar facet of the
// same source, is replaced by the two facets over the other diagonal, each
// with the larger bound. A neighbour out of the plane, from another source, or
// one whose flip would leave needles is left alone.
//
// Leg shown to fail: dropping the coplanarity test flips the out-of-plane pair
// and changes the surface.
func TestFlipNeedles(t *testing.T) {
	t.Parallel()
	tiny := math.Ldexp(1, -50)
	points := func(d r3.Vec) []proof.Xpt {
		var out []proof.Xpt
		for _, p := range []r3.Vec{{}, {X: 4}, {X: 2, Y: tiny}, d} {
			out = append(out, proof.XptOf(p))
		}
		return out
	}
	for _, tc := range []struct {
		name string
		d    r3.Vec
		src  []int
		flip bool
	}{
		{"coplanar convex", r3.Vec{X: 2, Y: -3}, []int{7, 7}, true},
		{"out of the plane", r3.Vec{X: 2, Y: -3, Z: 1}, []int{7, 7}, false},
		{"another source", r3.Vec{X: 2, Y: -3}, []int{7, 8}, false},
		{"new facets are needles too", r3.Vec{X: 3, Y: -tiny}, []int{7, 7}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			const u, w, b, d = 0, 1, 2, 3
			tris := [][3]int{{u, w, b}, {w, u, d}}
			bounds := []float64{0.25, 0.5}
			n, err := FlipNeedles(t.Context(), points(tc.d), tris, tc.src, bounds)
			require.NoError(t, err)
			if !tc.flip {
				require.Zero(t, n)
				require.Equal(t, [][3]int{{u, w, b}, {w, u, d}}, tris)
				require.Equal(t, []float64{0.25, 0.5}, bounds)
				return
			}
			require.Equal(t, 1, n)
			require.Equal(t, [][3]int{{u, d, b}, {d, w, b}}, tris)
			require.Equal(t, []float64{0.5, 0.5}, bounds)
		})
	}
}
