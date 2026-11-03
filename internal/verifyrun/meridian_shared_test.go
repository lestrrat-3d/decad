package verifyrun

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestCommonAxesSharedPlacement compares two meridians whose frames differ
// (one in XY about the y axis, one in XZ about −y) under placements: one
// inexact rotation shared bit for bit places both axes on one line before
// it; the same rotation composed with a translation on one side does not
// count as shared, and neither lift is orthonormal, so the axes are not
// compared at all.
func TestCommonAxesSharedPlacement(t *testing.T) {
	t.Parallel()
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(math.Pi/3))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{Y: 1})
	require.NoError(t, err)
	moved, err := shift.Then(turn)
	require.NoError(t, err)
	xy, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	xz, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	zero, one := new(big.Rat), big.NewRat(1, 1)
	mk := func(f r3.Frame, xf r3.Transform, dir [3]*big.Rat) meridian {
		l, tr, _ := frameLift(f, xf)
		fl, ft, _ := frameLift(f, r3.Identity())
		return meridian{l: l, t: tr, xform: xf, fl: fl, ft: ft,
			anchor: [3]*big.Rat{zero, zero, zero}, dir: dir}
	}
	spindle := mk(xy, turn, [3]*big.Rat{zero, one, zero})
	for _, tc := range []struct {
		name   string
		place  r3.Transform
		shared bool
	}{
		{"one rotation shared", turn, true},
		{"rotation after a translation", moved, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			bearing := mk(xz, tc.place, [3]*big.Rat{zero, zero, one})
			aAnchor, aDir, bAnchor, bDir, ok := commonAxes(spindle, bearing)
			require.Equal(t, tc.shared, ok)
			if ok {
				require.True(t, isZero(crossVec(aDir, bDir)), "one axis direction")
				require.True(t, isZero(crossVec(subVec(bAnchor, aAnchor), aDir)), "one axis line")
			}
		})
	}
}
