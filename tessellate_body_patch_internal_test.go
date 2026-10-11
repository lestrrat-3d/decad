package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBodyPatchMeshChargesTheFilledCapsAxialBound(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := New()
	walls, err := doc.Extrude(s, s.Profiles()[0], Symmetric{D: units.Inches(2.5)}, WithSurfaceResult())
	require.NoError(t, err)
	filled, err := walls.Patch(t.Context(), Edges(Free()).Exactly(8))
	require.NoError(t, err)
	mesh, err := filled.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.Greater(t, mesh.bound, 0.0)
	require.Greater(t, mesh.areaSlack, 0.0)
	require.False(t, mesh.VolumeVerified())

	patches := 0
	for _, f := range filled.Faces() {
		if len(f.origins) != 1 || f.origins[0].Role != rolePatch {
			continue
		}
		patches++
		require.True(t, f.hasAxialDelta)
		require.Greater(t, f.axialDelta, 0.0)
		bound, ok := mesh.sourceBound(f)
		require.True(t, ok)
		require.GreaterOrEqual(t, bound, f.axialDelta)
	}
	require.Equal(t, 2, patches)
	heldArea := 0.0
	for _, tri := range mesh.triangles {
		a, b, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
		heldArea += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	require.LessOrEqual(t, math.Abs(heldArea-52640), mesh.areaSlack)
}
