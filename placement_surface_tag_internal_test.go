package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// curvedLoftRadiusFace returns the Cylinder-tagged face of radius 8 mm.
func curvedLoftRadiusFace(t *testing.T, body *Body) *Face {
	t.Helper()
	var found *Face
	for _, f := range body.Faces() {
		c, ok := f.Surface().(Cylinder)
		if !ok {
			continue
		}
		if r, _ := c.Radius.In(units.Millimeter); math.Abs(r-8) < 1e-9 {
			found = f
		}
	}
	require.NotNil(t, found)
	return found
}

// TestFacetedPlacementMovesCurvedTags places a smooth LoftSections body whose
// Cylinder tag carries no denoted patch. The moved tag must be the placed
// source surface exactly, and the published NormalAt bound must cover the true
// rotated normal for a rotation and for a pure translation.
func TestFacetedPlacementMovesCurvedTags(t *testing.T) {
	body := curvedLoftBody(t)
	src := curvedLoftRadiusFace(t, body)
	srcCyl := src.Surface().(Cylinder)
	p0 := r3.NewVec(8, 0, 5)
	n0, err := src.NormalAt(p0)
	require.NoError(t, err)

	rotY, err := r3.Rotation(r3.NewVec(0, 1, 0), units.Degrees(30))
	require.NoError(t, err)
	trans, err := r3.Translation(r3.NewVec(0, 5, 0))
	require.NoError(t, err)
	for _, tc := range []struct {
		name string
		move r3.Transform
	}{{"rotate30Y", rotY}, {"translateY5", trans}} {
		for _, mode := range []string{"Transform", "Copy"} {
			t.Run(tc.name+"/"+mode, func(t *testing.T) {
				var placed *Body
				if mode == "Transform" {
					// Transform consumes its input; copy first so the source stays live.
					cp, cerr := Copy(t.Context(), body)
					require.NoError(t, cerr)
					placed, err = Transform(t.Context(), cp, tc.move)
				} else {
					placed, err = Copy(t.Context(), body, WithCopyPlacement(tc.move))
				}
				require.NoError(t, err)
				mv := curvedLoftRadiusFace(t, placed)
				require.IsType(t, Cylinder{}, mv.Surface())
				cyl := mv.Surface().(Cylinder)
				require.Equal(t, tc.move.Apply(srcCyl.Origin), cyl.Origin)
				require.Equal(t, tc.move.ApplyDir(srcCyl.Axis), cyl.Axis)

				// The cylinder is rigidly moved, so its moved normal is the rotated source normal.
				want := tc.move.ApplyDir(n0.Value)
				got, err := mv.NormalAt(tc.move.Apply(p0))
				require.NoError(t, err)
				diff := got.Value.Sub(want).Len()
				// A moved tag is a float-image stand-in, charged the largest distance
				// of two unit normals (2), the limit of any enclosure of its departure.
				require.GreaterOrEqual(t, got.Bound.Base(), 2.0)
				require.LessOrEqual(t, diff, got.Bound.Base(),
					"published normal bound does not cover the true error")
			})
		}
	}
}
