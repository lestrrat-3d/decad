package flushface_test

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/flushface"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestDeepNeedsTheMarginInside(t *testing.T) {
	t.Parallel()
	// The square [0, 2] × [0, 2] in the plane y = 0; its centre lies 1 from every edge.
	o := square(0, 0, 0, true)
	for _, tc := range []struct {
		name   string
		p      r3.Vec
		margin float64
		want   bool
	}{
		{"centre, margin below its distance", r3.Vec{X: 1, Z: 1}, 0.999, true},
		{"centre, margin at its distance", r3.Vec{X: 1, Z: 1}, 1, false},
		{"near an edge", r3.Vec{X: 1, Z: 0.25}, 0.5, false},
		{"on an edge", r3.Vec{X: 1, Z: 0}, 0, false},
		{"outside, far from every edge", r3.Vec{X: 5, Z: 1}, 0.5, false},
		{"off the plane, inside", r3.Vec{X: 1, Y: 7, Z: 1}, 0.5, true},
	} {
		got, err := flushface.Deep(budget(t), o, tc.p, tc.margin)
		require.NoError(t, err)
		require.Equal(t, tc.want, got, tc.name)
	}
}
