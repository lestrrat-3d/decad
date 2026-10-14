package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRealCurvedCopiedToolLineage(t *testing.T) {
	blank, source, _ := neutralArcSeamSource(t)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	image, err := Copy(t.Context(), source, WithCopyPlacement(turn))
	require.NoError(t, err)

	set, err := certifyFacetedCopies(t.Context(), []*Body{source, image})
	require.NoError(t, err)
	require.Same(t, source, set.source)
	require.Equal(t, turn, set.moves[1])
	require.Len(t, set.held, 2)
	require.Len(t, set.faceOwner[1], len(source.Faces()))
	proxy, _, err := tessellateBooleanOperand(t.Context(), source, 0.1)
	require.NoError(t, err)
	placed, err := set.placedProxyShells(t.Context(), proxy)
	require.NoError(t, err)
	require.Len(t, placed, 2)
	for _, face := range source.Faces() {
		owner := set.faceOwner[1][face]
		require.NotNil(t, owner)
		require.Same(t, image, owner.body)
	}
	require.NotNil(t, blank, "the source is a real Revolve and LoftFaces pair")

	overlap, err := Copy(t.Context(), source)
	require.NoError(t, err)
	before := source.doc.Bodies()
	_, err = certifyFacetedCopies(t.Context(), []*Body{source, overlap})
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "not separate")
	require.Equal(t, before, source.doc.Bodies())

	quarter, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	third, err := Copy(t.Context(), image, WithCopyPlacement(quarter))
	require.NoError(t, err)
	thirdPayload := third.payload.(facetedPayload)
	require.Nil(t, thirdPayload.loftFacesTrim)
	require.Same(t, source, thirdPayload.copyLineage.source)
	set, err = certifyFacetedCopies(t.Context(), []*Body{source, image, third})
	require.NoError(t, err)
	require.Len(t, set.faceOwner[2], len(source.Faces()))

	mirrorFrame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	mirrored, err := Copy(t.Context(), source, WithCopyPlacement(reflection))
	require.NoError(t, err)
	set, err = certifyFacetedCopies(t.Context(), []*Body{source, mirrored})
	require.NoError(t, err)
	require.Equal(t, reflection, set.moves[1])
	placed, err = set.placedProxyShells(t.Context(), proxy)
	require.NoError(t, err)
	require.Len(t, placed, 2)

	part := image.payload.(facetedPayload)
	changed := *part.copyLineage
	changed.motion = r3.Identity()
	part.copyLineage = &changed
	image.payload = part
	_, err = certifyFacetedCopies(t.Context(), []*Body{source, image})
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "changed its recorded motion")
}

func TestFacetedCopyLineageDirectTransfer(t *testing.T) {
	_, source, _ := neutralArcSeamSource(t)
	rotate := func(degrees float64) r3.Transform {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(degrees))
		require.NoError(t, err)
		return turn
	}
	a, err := Copy(t.Context(), source, WithCopyPlacement(rotate(120)))
	require.NoError(t, err)
	b, err := Copy(t.Context(), source, WithCopyPlacement(rotate(240)))
	require.NoError(t, err)
	require.True(t, a.payload.(facetedPayload).copyLineage.direct)
	require.True(t, b.payload.(facetedPayload).copyLineage.direct)

	c, err := Copy(t.Context(), a, WithCopyPlacement(rotate(90)))
	require.NoError(t, err)
	cLineage := c.payload.(facetedPayload).copyLineage
	require.Same(t, source, cLineage.source)
	require.False(t, cLineage.direct)

	set, err := certifyFacetedCopies(t.Context(), []*Body{source, a, b})
	require.NoError(t, err)
	require.Equal(t, shellseparation.AxisX, set.axis)

	// Transfer and the direct pairwise check must agree on the direct set.
	tools := []*Body{source, a, b}
	shells := make([]shellseparation.Shell, len(tools))
	for i, tool := range tools {
		shells[i] = facetedShell(tool.payload.(facetedPayload))
	}
	for _, linked := range []bool{true, false} {
		require.True(t, shellseparation.Separate(t.Context(), shells, set.moves,
			shellseparation.SourceShell, shellseparation.AxisX, linked), "linked=%v", linked)
	}

	_, err = certifyFacetedCopies(t.Context(), []*Body{source, a, c})
	require.NoError(t, err)

	moved, err := Transform(t.Context(), a, rotate(30))
	require.NoError(t, err)
	require.False(t, moved.payload.(facetedPayload).copyLineage.direct)
}
