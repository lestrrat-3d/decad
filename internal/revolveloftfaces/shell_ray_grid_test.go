package revolveloftfaces_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCurvedMultiRunShellRayOwnersRefuseCollisions(t *testing.T) {
	station := revolveloftfaces.ArcSeamVertex{
		Exact: loftface.RatVec{big.NewRat(0, 1), big.NewRat(2, 1), big.NewRat(1, 1)},
		Point: r3.Vec{Y: 2, Z: 1}, LoftBound: 0.001,
	}
	seams := [][]revolveloftfaces.ArcSeamVertex{{station}, {station}}
	anchors := revolveloftfaces.ShellAnchors{Seams: seams}
	rays := revolveloftfaces.ShellRays(16, anchors)
	require.NotNil(t, rays)
	require.True(t, revolveloftfaces.ShellRayOrderBounded(rays, anchors))
	shared := 0
	for _, ray := range rays {
		if len(ray.SeamRefs) == 2 {
			shared++
		}
	}
	require.Equal(t, 1, shared)
	require.Nil(t, revolveloftfaces.ShellRays(16,
		revolveloftfaces.ShellAnchors{Seams: [][]revolveloftfaces.ArcSeamVertex{{station, station}}}))
	require.Nil(t, revolveloftfaces.ShellRays(16,
		revolveloftfaces.ShellAnchors{Seams: seams,
			Caps: []revolveloftfaces.ShellRayCap{{Point: r3.Vec{Y: 2, Z: 1}}}}))
}

func TestCurvedCapRayRejectsGridCollapse(t *testing.T) {
	require.Nil(t, revolveloftfaces.ShellRays(16,
		revolveloftfaces.ShellAnchors{Caps: []revolveloftfaces.ShellRayCap{{Point: r3.Vec{Y: 1}}}}))
}
