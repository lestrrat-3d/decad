package extent_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestExactlyParallel(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		a, b r3.Vec
		want bool
	}{
		{name: "same", a: r3.NewVec(0, 0, 1), b: r3.NewVec(0, 0, 1), want: true},
		{name: "opposite", a: r3.NewVec(0, 0, 1), b: r3.NewVec(0, 0, -1), want: true},
		{name: "scaled", a: r3.NewVec(1, 2, 3), b: r3.NewVec(-2, -4, -6), want: true},
		{name: "one ulp tilt", a: r3.NewVec(0, 0, 1), b: r3.NewVec(math.SmallestNonzeroFloat64, 0, 1)},
		{name: "rounded flip", a: r3.NewVec(0, 0, 1), b: r3.NewVec(0, -1.2246467991473532e-16, -1)},
		{name: "zero", a: r3.NewVec(0, 0, 0), b: r3.NewVec(0, 0, 1)},
		{name: "non finite", a: r3.NewVec(math.Inf(1), 0, 0), b: r3.NewVec(1, 0, 0)},
		{name: "perpendicular", a: r3.NewVec(1, 0, 0), b: r3.NewVec(0, 0, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, extent.ExactlyParallel(tc.a, tc.b))
		})
	}
}
