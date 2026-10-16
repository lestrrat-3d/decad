package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRevolveSectorNearUnitAxisPlacement(t *testing.T) {
	turn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(2*math.Pi*5/16))
	require.NoError(t, err)
	require.GreaterOrEqual(t, turn.Basis().EX.X, math.Nextafter(1, 0))
	require.LessOrEqual(t, turn.Basis().EX.X, math.Nextafter(1, math.Inf(1)))
	require.True(t, massmoment.XAxisRotation(turn))
	charge, err := massmoment.XAxisPlacementCharge(turn)
	require.NoError(t, err)
	require.GreaterOrEqual(t, charge.Stretch, math.Abs(1-turn.Basis().EX.X))
	require.Greater(t, massmoment.PlacedBound(1, charge), 1.0)

	tilted, err := r3.Rotation(r3.NewVec(1, 0.1, 0), units.Radians(2*math.Pi*5/16))
	require.NoError(t, err)
	require.False(t, massmoment.XAxisRotation(tilted))
	_, err = massmoment.XAxisPlacementCharge(tilted)
	require.Error(t, err)

	shifted, err := r3.Translation(r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	require.False(t, massmoment.XAxisRotation(shifted))
	_, err = massmoment.XAxisPlacementCharge(shifted)
	require.Error(t, err)

	mirrorFrame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	reflected, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	require.False(t, massmoment.XAxisRotation(reflected))
	_, err = massmoment.XAxisPlacementCharge(reflected)
	require.Error(t, err)
}

// These vertices are the two axial ends of a triangular prism. Quarter-turn
// copies overlap in each coordinate range but have a separating YZ diagonal.
func TestRevolveSectorProjectionSeparatesRotatedSources(t *testing.T) {
	first, second := revolveSectorProjectionTestSources(t, 90)
	require.True(t, revolveshell.RevolveSectorSourcesSeparate([]facetedPayload{first, second}))
	second.MeshBound = 2
	require.False(t, revolveshell.RevolveSectorSourcesSeparate([]facetedPayload{first, second}))
}

func TestRevolveSectorProjectionRefusesOverlap(t *testing.T) {
	first, second := revolveSectorProjectionTestSources(t, 5)
	require.False(t, revolveshell.RevolveSectorSourcesSeparate([]facetedPayload{first, second}))
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
	first := facetedPayload{Verts: vertices, MeshBound: 0.01, Xform: r3.Identity()}
	second := facetedPayload{Verts: placed, MeshBound: 0.01, Xform: turn,
		RevolveLoftSector: &revolveLoftSectorPlacement{Motion: turn}}
	return first, second
}
