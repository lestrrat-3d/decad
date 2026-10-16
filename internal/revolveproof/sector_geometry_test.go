package revolveproof_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestRevolveSectorSourceScalars(t *testing.T) {
	a := sectionrecord.Point2{}
	b := sectionrecord.Point2{U: 1, V: 1}
	c := sectionrecord.Point2{U: 2, V: 2}
	require.True(t, revolveproof.RevolveForwardCollinear(a, b, c))
	require.False(t, revolveproof.RevolveForwardCollinear(a, c, b))
	require.False(t, revolveproof.RevolveForwardCollinear(a, b,
		sectionrecord.Point2{U: 2, V: 3}))
	require.Equal(t, float64(0), revolveproof.RevolveMeridianResidual(b, 1, 0))
	require.Equal(t, float64(1), revolveproof.RevolveMeridianResidual(
		sectionrecord.Point2{U: 1, V: 2}, 1, 0))
}

func TestRevolveSectorRootAngles(t *testing.T) {
	path := []facetedrecord.RevolveSectorNode{
		{RootBoundary: true, Angle: 0.4},
		{RootBoundary: true, Angle: 0.1},
	}
	lo, hi, ok := revolveproof.RevolveRootAngles(path)
	require.True(t, ok)
	require.Equal(t, 0.1, lo)
	require.Equal(t, 0.4, hi)
	path[0].RootBoundary = false
	_, _, ok = revolveproof.RevolveRootAngles(path)
	require.False(t, ok)
}

func TestRevolveExteriorPathIncompleteFan(t *testing.T) {
	_, _, err := revolveproof.RevolveExteriorPath(
		facetedrecord.Payload{}, facetedrecord.RevolveSectorBoundary{}, nil,
		facetedrecord.RevolveSectionCertificate{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "point-loft fan is incomplete")
}
