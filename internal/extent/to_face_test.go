package extent_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// toFaceCap is an exact planar cap at world level z: a Plane tag whose
// origin sits at z, and a cap record of the canonical XY frame at level z.
func toFaceCap(t *testing.T, z, axialDelta float64) *topo.Face {
	t.Helper()
	tag, err := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	local, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return &topo.Face{Surface: surfacegeom.Plane{Frame: tag},
		CapRecord:  &topo.CapRecord{Frame: local, Xform: r3.Identity(), Z: z},
		AxialDelta: axialDelta, HasAxialDelta: true}
}

func TestToFaceStop(t *testing.T) {
	t.Parallel()
	sketch, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	t.Run("the face supplies the sense", func(t *testing.T) {
		stop, delta, err := extent.ToFaceStop(toFaceCap(t, 5, 0), sketch, 0, 0, 0, "the extrude")
		require.NoError(t, err)
		require.Equal(t, 5.0, stop)
		require.Zero(t, delta, "an exact cap level held exactly charges nothing")
		stop, _, err = extent.ToFaceStop(toFaceCap(t, -5, 0), sketch, 0, 0, 0, "the extrude")
		require.NoError(t, err)
		require.Equal(t, -5.0, stop)
	})
	t.Run("the offset and both displacements are charged", func(t *testing.T) {
		stop, delta, err := extent.ToFaceStop(toFaceCap(t, 5, 0.5), sketch, 1, 1, 0.25, "the extrude")
		require.NoError(t, err)
		require.Equal(t, 6.0, stop)
		require.GreaterOrEqual(t, delta, 0.75, "the offset conversion and the face's axial displacement")
		require.InDelta(t, 0.75, delta, 1e-12)
	})
	refusals := []struct {
		name   string
		face   func(t *testing.T) *topo.Face
		travel float64
		offset float64
		target error
		want   string
	}{
		{name: "faceted surface", target: decaderr.ErrUnsupported, want: "this face's surface is decad.Faceted",
			face: func(t *testing.T) *topo.Face {
				f := toFaceCap(t, 5, 0)
				f.Surface = surfacegeom.Faceted{}
				return f
			}},
		{name: "tilted plane", target: decaderr.ErrUnsupported, want: "normal exactly parallel",
			face: func(t *testing.T) *topo.Face {
				f := toFaceCap(t, 5, 0)
				tilted, err := r3.NewFrame(r3.Vec{Z: 5}, r3.Vec{X: 1}, r3.Vec{Y: 1, Z: 1e-6})
				require.NoError(t, err)
				f.Surface = surfacegeom.Plane{Frame: tilted}
				return f
			}},
		{name: "stated normal departure", target: decaderr.ErrUnsupported, want: "departs from its planar tag",
			face: func(t *testing.T) *topo.Face {
				f := toFaceCap(t, 5, 0)
				f.NormalBound = 1e-9
				return f
			}},
		{name: "no cap record", target: decaderr.ErrUnsupported, want: "states no record of the exact plane",
			face: func(t *testing.T) *topo.Face {
				f := toFaceCap(t, 5, 0)
				f.CapRecord = nil
				return f
			}},
		{name: "no axial displacement", target: decaderr.ErrUnsupported, want: "states no record of the exact plane",
			face: func(t *testing.T) *topo.Face {
				f := toFaceCap(t, 5, 0)
				f.HasAxialDelta = false
				return f
			}},
		{name: "record plane not perpendicular", target: decaderr.ErrUnsupported, want: "exactly perpendicular to the sweep",
			face: func(t *testing.T) *topo.Face {
				f := toFaceCap(t, 5, 0)
				side, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
				require.NoError(t, err)
				f.CapRecord.Frame = side
				return f
			}},
		{name: "coplanar with the sketch", target: decaderr.ErrDegenerate, want: "coplanar with the sketch plane",
			face: func(t *testing.T) *topo.Face { return toFaceCap(t, 0, 0) }},
		{name: "behind the swept side", target: decaderr.ErrDegenerate, travel: -1,
			want: "other side of the sketch plane than the extrude sweeps",
			face: func(t *testing.T) *topo.Face { return toFaceCap(t, 5, 0) }},
		{name: "offset behind the sketch", target: decaderr.ErrDegenerate, offset: -6,
			want: "offset pulls the stop behind the sketch plane",
			face: func(t *testing.T) *topo.Face { return toFaceCap(t, 5, 0) }},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			stop, delta, err := extent.ToFaceStop(tc.face(t), sketch, tc.travel, tc.offset, 0, "the extrude")
			require.ErrorIs(t, err, tc.target)
			require.ErrorContains(t, err, tc.want)
			require.Zero(t, stop)
			require.Zero(t, delta)
		})
	}
}
