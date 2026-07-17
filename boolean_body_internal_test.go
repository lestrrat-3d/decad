package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFacetedPlacementRebuildsCachedDiameter(t *testing.T) {
	doc := New()
	body, err := buildFacetedBody(doc, StepRef(0), facetedPayload{
		verts: []r3.Vec{
			r3.NewVec(0, 0, 0),
			r3.NewVec(3, 0, 0),
			r3.NewVec(0, 4, 0),
			r3.NewVec(0, 0, 5),
		},
		tris: [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
		src:  []int{0, 1, 2, 3},
		groups: []facetGroup{
			{planar: true}, {planar: true}, {planar: true}, {planar: true},
		},
		dPair: 10,
		xform: r3.Identity(),
	})
	require.NoError(t, err)

	before := body.payload.(facetedPayload)
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	translation, err := r3.Translation(r3.NewVec(1e12, -2e12, 3e12))
	require.NoError(t, err)
	placement, err := rotation.Then(translation)
	require.NoError(t, err)

	placed, err := before.placed(doc, StepRef(1), placement)
	require.NoError(t, err)
	after := placed.payload.(facetedPayload)
	want, ok := pointSetDiameter(after.verts)
	require.True(t, ok)
	require.NotEqual(t, before.diameter, want, "the placement must make a stale cached diameter observable")
	require.Equal(t, want, after.diameter)
}
