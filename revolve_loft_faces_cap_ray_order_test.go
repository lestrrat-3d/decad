package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCurvedCapRayOrderRequiresSeparatedSourceBoxes(t *testing.T) {
	point := func(y, z int64, id int) revolveloftfaces.RayOrderRay {
		return revolveloftfaces.RayOrderRay{Y: lfF(float64(y)), Z: lfF(float64(z)),
			SeamIndex: -1, CapCount: 1, CapID: id}
	}
	box := func(y0, y1, z0, z1 int64) revolveloftfaces.RayBox {
		return revolveloftfaces.RayBox{
			Y: loftFaceInterval{Lo: lfF(float64(y0)), Hi: lfF(float64(y1))},
			Z: loftFaceInterval{Lo: lfF(float64(z0)), Hi: lfF(float64(z1))},
		}
	}
	rays := []revolveloftfaces.RayOrderRay{
		{Y: lfOne(), Z: lfZero(), SeamIndex: -1},
		point(2, 1, 0), point(1, 2, 1),
		{Y: lfZero(), Z: lfOne(), SeamIndex: -1},
		{Y: lfSub(lfZero(), lfOne()), Z: lfZero(), SeamIndex: -1},
		{Y: lfZero(), Z: lfSub(lfZero(), lfOne()), SeamIndex: -1},
	}
	boxes := []revolveloftfaces.RayBox{box(2, 2, 1, 1), box(1, 1, 2, 2)}
	require.True(t, revolveloftfaces.CapRayOrderBounded(rays, nil, boxes))
	boxes[0] = box(2, 2, 1, 5)
	require.False(t, revolveloftfaces.CapRayOrderBounded(rays, nil, boxes),
		"overlapping source boxes cannot inherit rounded ray order")
	boxes[0] = box(2, 2, 1, 1)
	boxes[1] = box(0, 1, 2, 2)
	require.False(t, revolveloftfaces.CapRayOrderBounded(rays, nil, boxes),
		"a source box touching the next grid ray must refuse")
}

func TestCurvedCapRayRejectsGridCollapse(t *testing.T) {
	vertex := revolveLoftFacesCapSeamVertex{point: r3.Vec{Y: 1}}
	require.Nil(t, revolveLoftFacesShellRays(16, nil,
		[]revolveLoftFacesCapSeamVertex{vertex}))
}
