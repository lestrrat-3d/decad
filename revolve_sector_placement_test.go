package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These vertices are the two axial ends of a triangular prism. Quarter-turn
// copies overlap in each coordinate range but have a separating YZ diagonal.
func TestRevolveSectorProjectionSeparatesRotatedSources(t *testing.T) {
	first, second := revolveSectorProjectionTestSources(t, 90)
	require.True(t, revolveSectorSourcesSeparate([]facetedPayload{first, second}))
	second.meshBound = 2
	require.False(t, revolveSectorSourcesSeparate([]facetedPayload{first, second}))
}

func TestRevolveSectorProjectionRefusesOverlap(t *testing.T) {
	first, second := revolveSectorProjectionTestSources(t, 5)
	require.False(t, revolveSectorSourcesSeparate([]facetedPayload{first, second}))
}

func revolveSectorProjectionTestSources(t *testing.T, degrees float64) (facetedPayload, facetedPayload) {
	t.Helper()
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(degrees))
	require.NoError(t, err)
	vertices := make([]r3.Vec, 0, 6)
	for _, x := range []float64{4, 6} {
		vertices = append(vertices,
			r3.Vec{X: x, Y: 1, Z: 0},
			r3.Vec{X: x, Y: 4, Z: -2},
			r3.Vec{X: x, Y: 4, Z: 2})
	}
	placed := make([]r3.Vec, len(vertices))
	for i, vertex := range vertices {
		placed[i] = turn.Apply(vertex)
	}
	first := facetedPayload{verts: vertices, meshBound: 0.01, xform: r3.Identity()}
	second := facetedPayload{verts: placed, meshBound: 0.01, xform: turn,
		revolveLoftSector: &revolveLoftSectorPlacement{motion: turn}}
	return first, second
}
