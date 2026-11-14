package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/section-outline-design.md §6's tests of the local
// bound: a section's OffSurface and tolerance check read the facets near the
// plane, not the whole held mesh.

// blindPocketPlate is a 20 x 20 x 8 plate with a radius-2 blind pocket from its
// bottom face up to z = 5. The pocket's cylinder chords carry the mesh bound
// the rest of the plate does not.
func blindPocketPlate(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 10, 10, 2), 0, 0, -15)
	body, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	return body
}

func TestSectionOutlineLocalBoundBelowTheGlobalFloor(t *testing.T) {
	t.Parallel()
	body := blindPocketPlate(t)
	held, err := body.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	floor := held.Bound().Mag()
	require.Positive(t, floor)
	tol := units.Millimeters(math.Nextafter(floor, 0))

	// Through the pocket's wall the plane crosses the coarse facets: the whole
	// floor is charged, and a tolerance below it is refused.
	_, err = body.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(0, 0, 2), r3.NewVec(0, 0, 1)), tol)
	var boundErr *decad.MeshBoundError
	require.ErrorAs(t, err, &boundErr)
	require.Equal(t, floor, boundErr.Floor().Mag())

	// Far above the pocket only the plate's own facets lie near the plane.
	// The tolerance that the whole-mesh bound refused now succeeds, and the
	// reported bound is far below the floor.
	out, err := body.SectionOutline(t.Context(), outlinePlane(t, r3.NewVec(0, 0, 7), r3.NewVec(0, 0, 1)), tol)
	require.NoError(t, err)
	require.Len(t, out.Loops, 1)
	require.InDelta(t, 400, out.PolygonArea().Base(), 1e-9)
	require.Less(t, out.OffSurface.Base(), 1e-9)
	for _, p := range out.Loops[0] {
		require.InDelta(t, 7, p.Z, 1e-12)
	}
}

func TestSectionOutlineLocalBoundStillChargesFacetsInTheSlab(t *testing.T) {
	t.Parallel()
	body := blindPocketPlate(t)
	held, err := body.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	floor := held.Bound().Mag()
	tol := units.Millimeters(math.Nextafter(floor, 0))

	cases := []struct {
		name string
		z    float64
		// charged is whether the plane's slab holds a coarse pocket facet.
		charged bool
	}{
		{name: "through the wall", z: 2, charged: true},
		{name: "through the pocket's top face", z: 4.99, charged: true},
		{name: "just beside the rim, within the floor", z: 5 + floor/2, charged: true},
		{name: "beyond the floor from the rim", z: 5 + 4*floor, charged: false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			plane := outlinePlane(t, r3.NewVec(0, 0, tc.z), r3.NewVec(0, 0, 1))
			// At the floor the global value is read in every case.
			atFloor, err := body.SectionOutline(t.Context(), plane, units.Millimeters(floor))
			require.NoError(t, err)
			if tc.charged {
				require.GreaterOrEqual(t, atFloor.OffSurface.Base(), floor)
				_, err = body.SectionOutline(t.Context(), plane, tol)
				var boundErr *decad.MeshBoundError
				require.ErrorAs(t, err, &boundErr)
				require.GreaterOrEqual(t, boundErr.Floor().Mag(), floor)
				return
			}
			require.Less(t, atFloor.OffSurface.Base(), floor/2)
			_, err = body.SectionOutline(t.Context(), plane, tol)
			require.NoError(t, err)
		})
	}
}
