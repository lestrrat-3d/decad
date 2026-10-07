package offset2d_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/stretchr/testify/require"
)

func TestCarrierIntersectionsKeepHeldBits(t *testing.T) {
	tests := []struct {
		a, b offset2d.Curve
		want [2]uint64
	}{
		{
			a:    offset2d.Curve{IsLine: true, PX: 0.1, PY: 0.2, DX: 1},
			b:    offset2d.Curve{IsLine: true, PX: 1.3, PY: 1.4, DY: 1},
			want: [2]uint64{0x3ff4cccccccccccd, 0x3fc999999999999a},
		},
		{
			a:    offset2d.Curve{IsLine: true, PX: 0.1, PY: 0.2, DX: 1},
			b:    offset2d.Curve{CX: 1.3, CY: 0.2, Radius: 0.7},
			want: [2]uint64{0x3fe3333333333333, 0x3fc999999999999a},
		},
		{
			a:    offset2d.Curve{CX: 0.1, CY: 0.2, Radius: 1.5},
			b:    offset2d.Curve{CX: 1.3, CY: 0.2, Radius: 1.1},
			want: [2]uint64{0x3ff2222222222222, 0xbfec64c3de5df40a},
		},
	}
	for _, tt := range tests {
		x, y, ok := offset2d.Intersect(tt.a, tt.b, 0.7, 0.1)
		require.True(t, ok)
		require.Equal(t, tt.want, [2]uint64{math.Float64bits(x), math.Float64bits(y)})
	}
	_, _, ok := offset2d.Intersect(
		offset2d.Curve{IsLine: true, DX: 1}, offset2d.Curve{IsLine: true, PY: 1, DX: 1}, 0, 0)
	require.False(t, ok)
}
