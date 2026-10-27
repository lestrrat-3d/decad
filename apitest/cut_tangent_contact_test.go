package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures below cut a 20×20×10 plate, x and y in [-10, 10], whose
// square through hole spans x ∈ [-4, 0], y ∈ [-2, 2]: 4000 − 160 = 3840 mm³.
// A round tool whose wall touches a target wall at one point without crossing
// it matches the clean-nesting cut whole, since sketch's arrangement does not
// cut a tangency. The cut section would hold two loops meeting at that point,
// which no cap mesh can separate (agent-docs/prism-boolean-design.md §4.2).

// touchHoledPlate is the plate with its square through hole.
func touchHoledPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	hole := boxBody(t, doc, -4, -2, 0, 2, 10)
	holed, err := decad.Cut(t.Context(), plate, hole)
	require.NoError(t, err)
	return holed
}

// touchStackedPlate adds a 4×4 blind pocket from the top, z ∈ [6, 10], away
// from the hole, so the target is a stacked prism: 3840 − 64 = 3776 mm³.
func touchStackedPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	pocket := boxBodyAtZ(t, doc, 4, 4, 8, 8, 6, 4)
	stacked, err := decad.Cut(t.Context(), touchHoledPlate(t, doc), pocket)
	require.NoError(t, err)
	require.False(t, tookMeshPath(stacked))
	return stacked
}

// TestCutTangentPocketTakesMeshPath cuts round tools whose wall touches the
// hole wall, the plate's side wall, an enclosed round hole or a cylinder's
// wall at one point. Each takes the mesh path, so the result meshes and its
// volume covers the closed form.
func TestCutTangentPocketTakesMeshPath(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name   string
		target func(t *testing.T, doc *decad.Document) *decad.Body
		tool   func(t *testing.T, doc *decad.Document) *decad.Body
		volume float64
	}{
		{
			// The reported case: r = 2 at (2, 0) touches the hole wall x = 0
			// at (0, 0), from the top.
			name:   "hole wall from the top",
			target: touchHoledPlate,
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 2, 2, 6, 4)
			},
			volume: 3840 - 16*math.Pi,
		},
		{
			name:   "hole wall from the bottom",
			target: touchHoledPlate,
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 2, 2, 0, 4)
			},
			volume: 3840 - 16*math.Pi,
		},
		{
			// r = 2 at (8, 0) touches the side wall x = 10 from inside.
			name: "outer wall",
			target: func(t *testing.T, doc *decad.Document) *decad.Body {
				return boxBody(t, doc, -10, -10, 10, 10, 10)
			},
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 8, 2, 6, 4)
			},
			volume: 4000 - 16*math.Pi,
		},
		{
			// r = 3 at (-1, 0) encloses the r = 2 through hole at (-2, 0)
			// and touches it at (-4, 0): 4000 − 40π − (9π − 4π)·4.
			name: "enclosed round hole",
			target: func(t *testing.T, doc *decad.Document) *decad.Body {
				plate := boxBody(t, doc, -10, -10, 10, 10, 10)
				holed, err := decad.Cut(t.Context(), plate, circleBodyAtZ(t, doc, -2, 2, 0, 10))
				require.NoError(t, err)
				return holed
			},
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, -1, 3, 6, 4)
			},
			volume: 4000 - 60*math.Pi,
		},
		{
			// A through tool r = 2 at (8, 0) touches the wall of an r = 10
			// cylinder from inside at (10, 0); the mesh path builds this
			// contact rather than refusing it: 1000π − 40π.
			name: "through a cylinder's wall",
			target: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 0, 10, 0, 10)
			},
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 8, 2, 0, 10)
			},
			volume: 960 * math.Pi,
		},
		{
			name:   "stacked target, split slab",
			target: touchStackedPlate,
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 2, 2, 7, 3)
			},
			volume: 3776 - 12*math.Pi,
		},
		{
			name:   "stacked target, existing interface",
			target: touchStackedPlate,
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 2, 2, 6, 4)
			},
			volume: 3776 - 16*math.Pi,
		},
		{
			name:   "stacked target, from the bottom",
			target: touchStackedPlate,
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 2, 2, 0, 3)
			},
			volume: 3776 - 12*math.Pi,
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			got, err := decad.Cut(t.Context(), tc.target(t, doc), tc.tool(t, doc))
			require.NoError(t, err)
			require.True(t, tookMeshPath(got))
			volume, err := got.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, volume.Exactness)
			require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-tc.volume), boundMM3(t, volume))
			mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.5))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// TestCutTangentThroughToolRefuses runs round tools through the whole target
// touching a wall at one point. The mesh path refuses that contact.
func TestCutTangentThroughToolRefuses(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name   string
		target func(t *testing.T, doc *decad.Document) *decad.Body
		tool   func(t *testing.T, doc *decad.Document) *decad.Body
	}{
		{
			name:   "hole wall",
			target: touchHoledPlate,
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 2, 2, 0, 10)
			},
		},
		{
			name: "outer wall",
			target: func(t *testing.T, doc *decad.Document) *decad.Body {
				return boxBody(t, doc, -10, -10, 10, 10, 10)
			},
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 8, 2, 0, 10)
			},
		},
		{
			name:   "stacked target",
			target: touchStackedPlate,
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				return circleBodyAtZ(t, doc, 2, 2, 0, 10)
			},
		},
		{
			// Two disjoint circles at one level union into a prism group,
			// which takes A5's group cut; the first touches the hole wall.
			name:   "prism group tool",
			target: touchHoledPlate,
			tool: func(t *testing.T, doc *decad.Document) *decad.Body {
				group, err := decad.Union(t.Context(), circleBodyAtZ(t, doc, 2, 2, 0, 10),
					circleBodyAtZ(t, doc, 7, 1, 0, 10))
				require.NoError(t, err)
				require.False(t, tookMeshPath(group))
				return group
			},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			_, err := decad.Cut(t.Context(), tc.target(t, doc), tc.tool(t, doc))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			var be *decad.BooleanError
			require.ErrorAs(t, err, &be)
			require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
			require.Len(t, doc.Bodies(), 2)
		})
	}
}

// TestCutLineTouchPocket cuts square pockets from the top that touch the
// hole along its x = 0 wall or at its (0, 2) corner. The arrangement splits
// a shared span and two meeting corners, so these never match the
// clean-nesting cut whole; each result meshes and its volume is the closed
// form.
func TestCutLineTouchPocket(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name           string
		x0, y0, x1, y1 float64
		volume         float64
		exact          bool
	}{
		{name: "whole wall", x0: 0, y0: -2, x1: 4, y1: 2, volume: 3840 - 64, exact: true},
		{name: "part of the wall", x0: 0, y0: -1, x1: 4, y1: 1, volume: 3840 - 32, exact: true},
		{name: "corner", x0: 0, y0: 2, x1: 4, y1: 6, volume: 3840 - 64},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			tool := boxBodyAtZ(t, doc, tc.x0, tc.y0, tc.x1, tc.y1, 6, 4)
			got, err := decad.Cut(t.Context(), touchHoledPlate(t, doc), tool)
			require.NoError(t, err)
			volume, err := got.Volume()
			require.NoError(t, err)
			if tc.exact {
				require.Equal(t, decad.Exact, volume.Exactness)
				require.Equal(t, tc.volume, volumeMM(t, volume))
			} else {
				require.True(t, tookMeshPath(got))
				require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-tc.volume), boundMM3(t, volume))
			}
			mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.5))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}
