package capplane_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capplane"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

var ex, ey, ez = r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)

func frame(t *testing.T, u, v r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, u, v)
	require.NoError(t, err)
	return f
}

// record is the plane of the XY frame at level z along its normal: +z, or -z
// when flipped, so a flipped record at level z is the plane at height -z.
func record(t *testing.T, z float64, flipped bool) *topo.CapRecord {
	t.Helper()
	f := frame(t, ex, ey)
	if flipped {
		f = frame(t, ey, ex)
	}
	return &topo.CapRecord{Frame: f, Xform: r3.Identity(), Z: z}
}

func ofRecord(t *testing.T, rec *topo.CapRecord) capplane.Plane {
	t.Helper()
	p, ok := capplane.OfRecord(rec)
	require.True(t, ok)
	return p
}

func TestOfRecord(t *testing.T) {
	t.Parallel()
	p := ofRecord(t, record(t, 10, false))
	require.True(t, p.Holds(r3.NewVec(3, -7, 10)))
	require.False(t, p.Holds(r3.NewVec(3, -7, math.Nextafter(10, 11))), "one ulp off the level is off the plane")
	shift, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	moved := ofRecord(t, &topo.CapRecord{Frame: frame(t, ex, ey), Xform: shift, Z: 10})
	require.True(t, moved.Holds(r3.NewVec(0, 0, 15)), "the placement carries the plane")
	_, ok := capplane.OfRecord(&topo.CapRecord{Frame: frame(t, ex, ey), Xform: r3.Identity(), Z: math.Inf(1)})
	require.False(t, ok, "a non-finite level reads no plane")
}

func TestOfFace(t *testing.T) {
	t.Parallel()
	face := func(rec *topo.CapRecord, has bool, delta float64) *topo.Face {
		return &topo.Face{CapRecord: rec, HasAxialDelta: has, AxialDelta: delta}
	}
	p, ok := capplane.OfFace(face(record(t, 0.1, false), true, 0))
	require.True(t, ok, "a face with a stated zero displacement lies on its held level")
	require.True(t, p.Holds(r3.NewVec(0, 0, 0.1)))

	denoted := record(t, 0.1, false)
	denoted.ZDenoted = big.NewRat(1, 10)
	p, ok = capplane.OfFace(face(denoted, true, 1e-17))
	require.True(t, ok, "a displaced face with a denoted level lies on that level")
	require.False(t, p.Holds(r3.NewVec(0, 0, 0.1)), "the float 0.1 is not the denoted 1/10")
	require.True(t, p.Same(p))

	for name, f := range map[string]*topo.Face{
		"no record":               {},
		"displaced, not denoted":  face(record(t, 10, false), true, 1e-9),
		"no displacement stated":  face(record(t, 10, false), false, 0),
		"non-finite record level": face(&topo.CapRecord{Frame: frame(t, ex, ey), Xform: r3.Identity(), Z: math.NaN()}, true, 0),
	} {
		_, ok := capplane.OfFace(f)
		require.False(t, ok, name)
	}
}

func TestSameAndParallel(t *testing.T) {
	t.Parallel()
	p := ofRecord(t, record(t, 10, false))
	require.True(t, p.Same(ofRecord(t, record(t, -10, true))), "a reversed normal names the same plane")
	require.False(t, p.Same(ofRecord(t, record(t, 10, true))), "level 10 along -z is the plane at height -10")
	ulp := ofRecord(t, record(t, math.Nextafter(10, 11), false))
	require.False(t, p.Same(ulp), "planes one ulp apart are two planes")
	require.True(t, p.ParallelTo(ulp))
	offset := ofRecord(t, record(t, 20, false))
	require.False(t, p.Same(offset))
	require.True(t, p.ParallelTo(offset))
	side := ofRecord(t, &topo.CapRecord{Frame: frame(t, ey, ez), Xform: r3.Identity(), Z: 10})
	require.False(t, p.Same(side))
	require.False(t, p.ParallelTo(side))
}

func TestSense(t *testing.T) {
	t.Parallel()
	up := ofRecord(t, record(t, 10, false))
	down := ofRecord(t, record(t, -10, true))
	require.Equal(t, 1, up.Sense(ez))
	require.Equal(t, -1, up.Sense(ez.Scale(-1)))
	require.Equal(t, -1, down.Sense(ez), "the reversed record's normal points down")
	require.Equal(t, 0, up.Sense(ex), "a direction in the plane has no side")
	require.Equal(t, 1, up.Sense(r3.NewVec(1, 0, 0x1p-60)), "a tilt of 2^-60 is still a side")
	require.Equal(t, 0, up.Sense(r3.NewVec(math.NaN(), 0, 1)))
}

func TestAxisAndCrosses(t *testing.T) {
	t.Parallel()
	shift, err := r3.Translation(r3.NewVec(4, 5, 6))
	require.NoError(t, err)
	zAxis, ok := capplane.Axis(frame(t, ex, ey), shift)
	require.True(t, ok)
	xAxis, ok := capplane.Axis(frame(t, ey, ez), r3.Identity())
	require.True(t, ok)
	p := ofRecord(t, record(t, 10, false))
	require.True(t, p.Crosses(zAxis), "a sweep along the normal crosses the plane")
	require.False(t, p.Crosses(xAxis), "a sweep along the plane does not")
	downAxis, ok := capplane.Axis(frame(t, ey, ex), r3.Identity())
	require.True(t, ok)
	require.True(t, p.Crosses(downAxis), "a sweep against the normal crosses the plane too")
	tilted, ok := capplane.Axis(frame(t, ey, r3.NewVec(-0x1p-60, 0, 1)), r3.Identity())
	require.True(t, ok)
	require.True(t, p.Crosses(tilted), "a sweep tilted 2^-60 out of the plane crosses it")
	side := ofRecord(t, &topo.CapRecord{Frame: frame(t, ey, ez), Xform: r3.Identity(), Z: 10})
	require.True(t, side.Crosses(xAxis))
	require.False(t, side.Crosses(zAxis))
}
