package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const capTagBlank provenance.Producer = 7

func capTagPlane(t *testing.T, z, normal float64) surfacegeom.Plane {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: normal})
	require.NoError(t, err)
	require.Equal(t, r3.Vec{Z: normal}, frame.N())
	return surfacegeom.Plane{Frame: frame}
}

// capTagFixture returns a blank whose start cap is the plane z=0 with an
// outward -Z normal and whose end cap is the plane z=2 with +Z, plus a mesh
// with one facet on each cap level (groups 0 and 1) and one side facet
// (group 2).
func capTagFixture(t *testing.T) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	bottom := &topo.Face{Surface: capTagPlane(t, 0, -1), NormalBound: 0.25, Reversed: true,
		Origins: []provenance.Ref{{Producer: capTagBlank, Role: provenance.RoleCapStart}}}
	top := &topo.Face{Surface: capTagPlane(t, 2, 1), NormalBound: 0.5,
		Origins: []provenance.Ref{{Producer: capTagBlank, Role: provenance.RoleCapEnd}}}
	side := &topo.Face{Origins: []provenance.Ref{{Producer: capTagBlank, Role: "side"}}}
	blank := &topo.Body{Origin: provenance.Ref{Producer: capTagBlank},
		Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{bottom, top, side}}}}}}
	pp := facetedrecord.Payload{
		Verts: []r3.Vec{
			{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0, Y: 1, Z: 0},
			{X: 0, Y: 0, Z: 2}, {X: 1, Y: 0, Z: 2}, {X: 0, Y: 1, Z: 2},
		},
		Tris: [][3]int{{0, 2, 1}, {3, 4, 5}, {0, 1, 4}},
		Src:  []int{0, 1, 2},
		Groups: []facetedrecord.Group{
			{Origins: []provenance.Ref{{Producer: 3, Role: "held"}}},
			{},
			{Origins: []provenance.Ref{{Producer: 3, Role: "wall"}}},
		},
	}
	return blank, pp
}

func TestTagCaps(t *testing.T) {
	const ref provenance.Producer = 11
	levels := [2]float64{0, 2}
	t.Run("tags both caps from the blank", func(t *testing.T) {
		blank, pp := capTagFixture(t)
		heldOrigins := pp.Groups[0].Origins
		require.NoError(t, loftcylinder.TagCaps(&pp, blank, [2]int{0, 1}, levels, ref))

		require.Equal(t, capTagPlane(t, 0, -1), pp.Groups[0].Surface)
		require.True(t, pp.Groups[0].Planar)
		require.Equal(t, 0.25, pp.Groups[0].NormalBound)
		require.True(t, pp.Groups[0].Reversed)
		require.Equal(t, []provenance.Ref{{Producer: 3, Role: "held"},
			{Producer: ref, Role: provenance.RoleCapStart}}, pp.Groups[0].Origins)
		require.Len(t, heldOrigins, 1, "the source group's origin slice is not appended in place")

		require.Equal(t, capTagPlane(t, 2, 1), pp.Groups[1].Surface)
		require.True(t, pp.Groups[1].Planar)
		require.Equal(t, 0.5, pp.Groups[1].NormalBound)
		require.False(t, pp.Groups[1].Reversed)
		require.Equal(t, []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}}, pp.Groups[1].Origins)

		require.Nil(t, pp.Groups[2].Surface)
		require.False(t, pp.Groups[2].Planar)
		require.Equal(t, []provenance.Ref{{Producer: 3, Role: "wall"}}, pp.Groups[2].Origins)
	})

	refusals := []struct {
		name   string
		groups [2]int
		levels [2]float64
		edit   func(blank **topo.Body, pp *facetedrecord.Payload)
	}{
		{name: "no blank", groups: [2]int{0, 1}, levels: levels,
			edit: func(blank **topo.Body, _ *facetedrecord.Payload) { *blank = nil }},
		{name: "one group for both caps", groups: [2]int{0, 0}, levels: levels},
		{name: "group out of range", groups: [2]int{0, 3}, levels: levels},
		{name: "source count mismatch", groups: [2]int{0, 1}, levels: levels,
			edit: func(_ **topo.Body, pp *facetedrecord.Payload) { pp.Src = pp.Src[:2] }},
		{name: "duplicate cap role", groups: [2]int{0, 1}, levels: levels,
			edit: func(blank **topo.Body, _ *facetedrecord.Payload) {
				shell := (*blank).Lumps[0].Shells[0]
				twin := *shell.Faces[1]
				shell.Faces = append(shell.Faces, &twin)
			}},
		{name: "cap role of another producer", groups: [2]int{0, 1}, levels: levels,
			edit: func(blank **topo.Body, _ *facetedrecord.Payload) {
				(*blank).Lumps[0].Shells[0].Faces[1].Origins[0].Producer = capTagBlank + 1
			}},
		{name: "cap is not a plane", groups: [2]int{0, 1}, levels: levels,
			edit: func(blank **topo.Body, _ *facetedrecord.Payload) {
				(*blank).Lumps[0].Shells[0].Faces[0].Surface = surfacegeom.Cylinder{}
			}},
		{name: "cap plane at another level", groups: [2]int{0, 1}, levels: levels,
			edit: func(blank **topo.Body, _ *facetedrecord.Payload) {
				(*blank).Lumps[0].Shells[0].Faces[1].Surface = capTagPlane(t, 3, 1)
			}},
		{name: "cap normal points inward", groups: [2]int{0, 1}, levels: levels,
			edit: func(blank **topo.Body, _ *facetedrecord.Payload) {
				(*blank).Lumps[0].Shells[0].Faces[0].Surface = capTagPlane(t, 0, 1)
			}},
		{name: "facet leaves its cap plane", groups: [2]int{0, 1}, levels: levels,
			edit: func(_ **topo.Body, pp *facetedrecord.Payload) { pp.Verts[4].Z = 1.5 }},
		{name: "facet names no vertex", groups: [2]int{0, 1}, levels: levels,
			edit: func(_ **topo.Body, pp *facetedrecord.Payload) { pp.Tris[1][2] = len(pp.Verts) }},
		{name: "cap group has no facets", groups: [2]int{0, 1}, levels: levels,
			edit: func(_ **topo.Body, pp *facetedrecord.Payload) { pp.Src[1] = 2 }},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			blank, pp := capTagFixture(t)
			if tc.edit != nil {
				tc.edit(&blank, &pp)
			}
			err := loftcylinder.TagCaps(&pp, blank, tc.groups, tc.levels, ref)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
}

func TestTagBridgedTop(t *testing.T) {
	tagged := func(t *testing.T) facetedrecord.Payload {
		t.Helper()
		blank, pp := capTagFixture(t)
		require.NoError(t, loftcylinder.TagCaps(&pp, blank, [2]int{0, 1}, [2]float64{0, 2}, 11))
		return pp
	}
	t.Run("no positive shift keeps the plane", func(t *testing.T) {
		pp := tagged(t)
		loftcylinder.TagBridgedTop(&pp, 1, []float64{0, -1})
		require.Equal(t, capTagPlane(t, 2, 1), pp.Groups[1].Surface)
		require.True(t, pp.Groups[1].Planar)
	})
	t.Run("a positive shift drops the top plane only", func(t *testing.T) {
		pp := tagged(t)
		loftcylinder.TagBridgedTop(&pp, 1, []float64{0, 1e-9})
		require.Nil(t, pp.Groups[1].Surface)
		require.False(t, pp.Groups[1].Planar)
		require.Equal(t, []provenance.Ref{{Producer: 11, Role: provenance.RoleCapEnd}}, pp.Groups[1].Origins)
		require.Equal(t, capTagPlane(t, 0, -1), pp.Groups[0].Surface)
		require.True(t, pp.Groups[0].Planar)
	})
}
