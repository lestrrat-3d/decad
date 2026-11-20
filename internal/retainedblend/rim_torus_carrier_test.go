package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRoundedRimTorusChargesCenterAndMajor(t *testing.T) {
	z := r3.Vec{Z: 1}
	frame, err := r3.NewFrame(z, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	plane := &topo.Face{Surface: surfacegeom.Plane{Frame: frame}, RetainedTag: true}
	wall := &topo.Face{Surface: surfacegeom.Cylinder{Axis: z, Radius: units.Millimeters(1)}, RetainedTag: true}
	plane.CapRecord = &topo.CapRecord{Frame: frame, Xform: r3.Identity()}
	wall.FilletCarrier = &topo.FilletCarrier{Surface: wall.Surface}
	r := rim{planeFace: plane, wallFace: wall, radius: 1, kappa: 1, side: 1}
	small := math.Ldexp(1, -54)
	tag := surfacegeom.Torus{Center: z, Axis: z, Major: units.Millimeters(1), Minor: units.Millimeters(small)}
	c := roundedRimTorus(r, small, tag)
	require.NotNil(t, c)
	require.GreaterOrEqual(t, c.Distance, 2*small)
	// The native cap level can also round away from its published plane.
	plane.CapRecord.Z = small
	c = roundedRimTorus(r, small, tag)
	require.NotNil(t, c)
	require.GreaterOrEqual(t, c.Distance, 3*small)
	plane.CapRecord.Z = 0
	proof := wall.FilletCarrier
	wall.FilletCarrier = nil
	require.Nil(t, roundedRimTorus(r, small, tag))
	wall.FilletCarrier = proof
	plane.AxialDelta = small
	require.Nil(t, roundedRimTorus(r, small, tag))
	plane.AxialDelta = 0
	wall.Surface = surfacegeom.Sphere{}
	require.Nil(t, roundedRimTorus(r, small, tag))
}
