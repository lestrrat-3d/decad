package breprecord

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStraightFilletCarrier(t *testing.T) {
	small := math.Ldexp(1, -54)
	f, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	pl := prismrecord.Payload{Frame: f, Xform: r3.Identity()}
	b := LoopBand{Sigma: 1, Kind: BandFillet, Setback: capblend.Setback{Dc: small, Ds: small}, Orig: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.LineSeg{}}}}
	face := Face{Frame: f, Outward: true, Z0: 1}
	w := survey2d.SideWalk{Segs: []int{0}, StartV: 1, EndU: 1, EndV: 1}
	z, _ := b.SideLevel(face)
	c := surfacegeom.Cylinder{Axis: r3.Vec{X: 1}, Origin: r3.Vec{Y: 1, Z: z}, Radius: units.Millimeters(small)}
	proof := straightFilletCarrier(b, face, pl, w, c)
	require.NotNil(t, proof)
	require.GreaterOrEqual(t, proof.Distance, small)
	w.EndV = math.Nextafter(1, 2)
	require.Nil(t, straightFilletCarrier(b, face, pl, w, c))
	w.EndV = 1
	w.StartBound.U = small
	require.Nil(t, straightFilletCarrier(b, face, pl, w, c))
}
