package retainedblend

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPlaneRecordWithBooleanRim distinguishes a rim cut into a retained plane
// from an original revolve rim. The cut rim need not be a meridian endpoint;
// neither kind of rim can make a mismatched plane record exact.
func TestPlaneRecordWithBooleanRim(t *testing.T) {
	point := func(n int64) proofbound.RatInterval { return proofbound.PointInterval(big.NewRat(n, 1)) }
	vec := func(x, y, z int64) proofbound.IvVec3 { return proofbound.IvVec3{point(x), point(y), point(z)} }
	record := &surfacenormal.Revolved{
		Origin: vec(0, 0, 0), Basis: [3]proofbound.IvVec3{vec(1, 0, 0), vec(0, 1, 0), vec(0, 0, 1)},
		Ends: [][2][2]proofbound.RatInterval{{{point(10), point(0)}, {point(10), point(20)}}}, Valid: true,
	}
	frame, err := r3.NewFrame(r3.Vec{Z: 10}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	rim := &topo.Edge{RetainedCurve: true, Curve: surfacegeom.Circle3{
		Center: r3.Vec{Z: 10}, Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(4),
	}}
	face := &topo.Face{RetainedTag: true, Surface: surfacegeom.Plane{Frame: frame}, Denoted: record,
		Loops: []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: rim}}}}}
	require.True(t, tagIsRecord(face), "a Boolean-cut rim does not change the denoted plane")

	face.RetainedTag = false
	require.False(t, tagIsRecord(face), "an unretained face cannot carry a Boolean rim")
	face.RetainedTag = true
	rim.RetainedCurve = false
	require.False(t, tagIsRecord(face), "an original rim still must match the recorded meridian")
	rim.RetainedCurve = true
	moved, err := r3.NewFrame(r3.Vec{Z: math.Nextafter(10, 11)}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	face.Surface = surfacegeom.Plane{Frame: moved}
	require.False(t, tagIsRecord(face), "a Boolean rim cannot make a shifted plane exact")
	face.Surface = surfacegeom.Plane{Frame: frame}
	record.Valid = false
	require.False(t, tagIsRecord(face), "a Boolean rim cannot make an invalid record exact")
}
