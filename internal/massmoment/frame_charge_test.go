package massmoment_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMapChargeContainsFrameDefect(t *testing.T) {
	frame, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	linear, err := massmoment.PlaneMap(frame, r3.Identity())
	require.NoError(t, err)

	charge, err := massmoment.MapChargeOf(linear)
	require.NoError(t, err)
	defect := massmoment.OrthonormalityDefect(linear)
	determinant := massmoment.Determinant(linear)
	volumeDefect := new(big.Rat).Sub(new(big.Rat).Abs(determinant), big.NewRat(1, 1))
	volumeDefect.Abs(volumeDefect)

	gotVolume := new(big.Rat).SetFloat64(charge.Volume)
	gotStretch := new(big.Rat).SetFloat64(charge.Stretch)
	require.NotNil(t, gotVolume)
	require.NotNil(t, gotStretch)
	require.GreaterOrEqual(t, gotVolume.Cmp(volumeDefect), 0)
	require.GreaterOrEqual(t, gotStretch.Cmp(defect), 0)
}
