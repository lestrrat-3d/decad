package revolvemass_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolvemass"
	"github.com/stretchr/testify/require"
)

func TestAdmitBandChargesScaleWithTheRadialAllowance(t *testing.T) {
	const axialExtent = 100.0
	smallBand := revolvemass.AdmitBandCharge(1e-5, axialExtent)
	largeBand := revolvemass.AdmitBandCharge(2e-5, axialExtent)
	smallVolume := revolvemass.AdmitVolumeCharge(1e-5, axialExtent)
	largeVolume := revolvemass.AdmitVolumeCharge(2e-5, axialExtent)

	require.Positive(t, smallBand)
	require.GreaterOrEqual(t, largeBand, 2*smallBand)
	require.Positive(t, smallVolume)
	require.GreaterOrEqual(t, largeVolume, 4*smallVolume)
	require.Zero(t, revolvemass.AdmitBandCharge(0, axialExtent))
	require.Zero(t, revolvemass.AdmitVolumeCharge(0, axialExtent))
}
