package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCurvedLoftFacesRealCopySeparation(t *testing.T) {
	_, source, trim := neutralArcSeamSource(t)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	image, err := Copy(t.Context(), source, WithCopyPlacement(turn))
	require.NoError(t, err)
	original := source.n.Payload.(facetedPayload)
	placed := image.n.Payload.(facetedPayload)
	require.Same(t, trim, original.LoftFacesTrim)
	require.NotNil(t, placed.CopyLineage)
	require.Same(t, source, placed.CopyLineage.Source)
	require.Equal(t, turn, placed.CopyLineage.Motion)

	shell := func(p facetedPayload) shellseparation.Shell {
		return shellseparation.Shell{Verts: p.Verts, Tris: p.Tris,
			VertexBound: p.VertexBound, MeshBound: p.MeshBound}
	}
	moves := []r3.Transform{r3.Identity(), turn}
	require.True(t, shellseparation.Separate(t.Context(),
		[]shellseparation.Shell{shell(original), shell(placed)}, moves,
		nil, shellseparation.SourceShell,
		shellseparation.AxisX, true))
	held, err := shellseparation.PlacedHeldShells(t.Context(), []shellseparation.HeldCopy{
		{Verts: original.Verts, Tris: original.Tris, Linked: true},
		{Verts: placed.Verts, Tris: placed.Tris, Linked: true},
	}, moves)
	require.NoError(t, err)
	require.True(t, shellseparation.Separate(t.Context(), held, moves,
		nil, shellseparation.HeldCopyShell,
		shellseparation.AxisX, true))

	overlap, err := Copy(t.Context(), source)
	require.NoError(t, err)
	require.False(t, shellseparation.Separate(t.Context(), []shellseparation.Shell{
		shell(original), shell(overlap.n.Payload.(facetedPayload)),
	}, []r3.Transform{r3.Identity(), r3.Identity()}, nil, shellseparation.SourceShell,
		shellseparation.AxisX, true))
}
