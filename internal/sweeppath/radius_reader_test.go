package sweeppath_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestRotationalSweepRadiusKeepsMemoOnItsNode checks that a refused box
// caches its refusal without affecting a distinct node with the same key.
func TestRotationalSweepRadiusKeepsMemoOnItsNode(t *testing.T) {
	axis := motionbound.RatVec{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(1, 1)}
	invalid := &topo.Body{Bounds: measurement.Box{
		Min: r3.Vec{X: 2}, Max: r3.Vec{X: 1}, Bound: units.Millimeters(0),
	}}
	for range 2 {
		radius, ok := sweeppath.RotationalSweepRadius(invalid, r3.Identity(), r3.Vec{}, axis)
		require.False(t, ok)
		require.Nil(t, radius)
	}
	invalid.SweepRadii.Mu.Lock()
	misses, hits, entries := invalid.SweepRadii.Misses, invalid.SweepRadii.Hits, len(invalid.SweepRadii.Entries)
	invalid.SweepRadii.Mu.Unlock()
	require.Equal(t, 1, misses)
	require.Equal(t, 1, hits)
	require.Equal(t, 1, entries)

	valid := &topo.Body{Bounds: measurement.Box{
		Min: r3.Vec{X: -1}, Max: r3.Vec{X: 1, Y: 1, Z: 1}, Bound: units.Millimeters(0),
	}}
	radius, ok := sweeppath.RotationalSweepRadius(valid, r3.Identity(), r3.Vec{}, axis)
	require.True(t, ok)
	require.Positive(t, radius.Sign())
}
