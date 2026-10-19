package denotation

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSameCurveRequiresMatchingMotion(t *testing.T) {
	t.Parallel()
	token := NewCurve(1)
	require.True(t, SameCurve(token, token))

	axis, ok := r3.NewVec(3, -1, 2).Normalize()
	require.True(t, ok)
	motion, err := r3.RotationAround(r3.NewVec(41, -17, 9), axis, units.Degrees(37))
	require.NoError(t, err)
	moved := token.Compose(motion)
	require.Equal(t, token.ID, moved.ID)
	require.NotEqual(t, token.Xform, moved.Xform)
	require.False(t, SameCurve(token, moved))
	require.False(t, SameCurve(moved, token))
}

func TestSameCurveDeclinesZeroToken(t *testing.T) {
	t.Parallel()
	zero := CurveToken{}
	require.False(t, SameCurve(zero, zero))
	token := NewCurve(1)
	require.False(t, SameCurve(zero, token))
	require.False(t, SameCurve(token, zero))
}

func TestComposeKeepsZeroTokenAsDecline(t *testing.T) {
	t.Parallel()
	axis, ok := r3.NewVec(1, 2, 3).Normalize()
	require.True(t, ok)
	motion, err := r3.RotationAround(r3.NewVec(5, 6, 7), axis, units.Degrees(37))
	require.NoError(t, err)
	require.Zero(t, (CurveToken{}).Compose(motion))
}
