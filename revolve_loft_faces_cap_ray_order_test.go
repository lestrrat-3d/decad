package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCurvedCapRayOrderRequiresSeparatedSourceBoxes(t *testing.T) {
	point := func(y, z int64, id int) revolveLoftFacesRay {
		return revolveLoftFacesRay{y: lfF(float64(y)), z: lfF(float64(z)),
			seamIndex: -1, capIDs: []int{id}}
	}
	box := func(y0, y1, z0, z1 int64) revolveLoftFacesCapSeamVertex {
		var vertex revolveLoftFacesCapSeamVertex
		vertex.boundary.box[1] = loftFaceInterval{Lo: lfF(float64(y0)), Hi: lfF(float64(y1))}
		vertex.boundary.box[2] = loftFaceInterval{Lo: lfF(float64(z0)), Hi: lfF(float64(z1))}
		return vertex
	}
	rays := []revolveLoftFacesRay{
		{y: lfOne(), z: lfZero(), seamIndex: -1},
		point(2, 1, 0), point(1, 2, 1),
		{y: lfZero(), z: lfOne(), seamIndex: -1},
		{y: lfSub(lfZero(), lfOne()), z: lfZero(), seamIndex: -1},
		{y: lfZero(), z: lfSub(lfZero(), lfOne()), seamIndex: -1},
	}
	vertices := []revolveLoftFacesCapSeamVertex{box(2, 2, 1, 1), box(1, 1, 2, 2)}
	require.True(t, revolveLoftFacesCapRayOrderBounded(rays, nil, vertices))
	vertices[0] = box(2, 2, 1, 5)
	require.False(t, revolveLoftFacesCapRayOrderBounded(rays, nil, vertices),
		"overlapping source boxes cannot inherit rounded ray order")
	vertices[0] = box(2, 2, 1, 1)
	vertices[1] = box(0, 1, 2, 2)
	require.False(t, revolveLoftFacesCapRayOrderBounded(rays, nil, vertices),
		"a source box touching the next grid ray must refuse")
}

func TestCurvedCapRayRejectsGridCollapse(t *testing.T) {
	vertex := revolveLoftFacesCapSeamVertex{point: r3.Vec{Y: 1}}
	require.Nil(t, revolveLoftFacesShellRays(16, nil,
		[]revolveLoftFacesCapSeamVertex{vertex}))
}
