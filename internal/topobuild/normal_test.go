package topobuild_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestNormalAtReadsOutwardNormals checks the computed direction of a planar
// face and of a cylindrical (non-planar curve) face, with and without the
// face's reversed flag.
func TestNormalAtReadsOutwardNormals(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	cyl := surfacegeom.Cylinder{Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(2)}
	tests := []struct {
		name     string
		face     topo.Face
		at       r3.Vec
		expected r3.Vec
	}{
		{"plane", topo.Face{Surface: surfacegeom.Plane{Frame: frame}}, r3.Vec{X: 3, Y: 4}, r3.Vec{Z: 1}},
		{"plane reversed", topo.Face{Surface: surfacegeom.Plane{Frame: frame}, Reversed: true}, r3.Vec{X: 3}, r3.Vec{Z: -1}},
		{"cylinder", topo.Face{Surface: cyl}, r3.Vec{X: 2, Z: 5}, r3.Vec{X: 1}},
		{"cylinder diagonal", topo.Face{Surface: cyl}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: math.Sqrt2 / 2, Y: math.Sqrt2 / 2}},
		{"cylinder reversed", topo.Face{Surface: cyl, Reversed: true}, r3.Vec{Y: 2}, r3.Vec{Y: -1}},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_ = t.Context()
			m, err := topobuild.NormalAt(&tc.face, tc.at)
			require.NoError(t, err)
			require.InDelta(t, tc.expected.X, m.Value.X, 1e-12)
			require.InDelta(t, tc.expected.Y, m.Value.Y, 1e-12)
			require.InDelta(t, tc.expected.Z, m.Value.Z, 1e-12)
		})
	}
}

// TestNormalAtRefusals checks the two refusal classes: a point with no
// direction and a surface that is not one of the five analytic variants.
func TestNormalAtRefusals(t *testing.T) {
	t.Parallel()
	cyl := topo.Face{Surface: surfacegeom.Cylinder{Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(2)}}
	_, err := topobuild.NormalAt(&cyl, r3.Vec{Z: 3})
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	_, err = topobuild.NormalAt(&topo.Face{}, r3.Vec{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
