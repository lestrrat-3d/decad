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
		require.Same(t, image, bodyHandle(owner.n.Body))
	}
	require.NotNil(t, blank, "the source is a real Revolve and LoftFaces pair")

	overlap, err := Copy(t.Context(), source)
	require.NoError(t, err)
	before := documentHandle(source.n.Doc).Bodies()
	_, err = certifyFacetedCopies(t.Context(), []*Body{source, overlap})
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "not separate")
	require.Equal(t, before, documentHandle(source.n.Doc).Bodies())

	quarter, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	third, err := Copy(t.Context(), image, WithCopyPlacement(quarter))
	require.NoError(t, err)
	thirdPayload := third.n.Payload.(facetedPayload)
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

	part := image.n.Payload.(facetedPayload)
	changed := *part.copyLineage
	changed.motion = r3.Identity()
	part.copyLineage = &changed
	image.n.Payload = part
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
	require.True(t, a.n.Payload.(facetedPayload).copyLineage.direct)
	require.True(t, b.n.Payload.(facetedPayload).copyLineage.direct)

	c, err := Copy(t.Context(), a, WithCopyPlacement(rotate(90)))
	require.NoError(t, err)
	cLineage := c.n.Payload.(facetedPayload).copyLineage
	require.Same(t, source, cLineage.source)
	require.False(t, cLineage.direct)

	set, err := certifyFacetedCopies(t.Context(), []*Body{source, a, b})
	require.NoError(t, err)
	require.Equal(t, shellseparation.AxisX, set.axis)

	// Transfer and the direct pairwise check must agree on the direct set.
	tools := []*Body{source, a, b}
	shells := make([]shellseparation.Shell, len(tools))
	for i, tool := range tools {
		shells[i] = facetedShell(tool.n.Payload.(facetedPayload))
	}
	for _, linked := range []bool{true, false} {
		require.True(t, shellseparation.Separate(t.Context(), shells, set.moves,
			nil, shellseparation.SourceShell, shellseparation.AxisX, linked), "linked=%v", linked)
	}

	_, err = certifyFacetedCopies(t.Context(), []*Body{source, a, c})
	require.NoError(t, err)

	moved, err := Transform(t.Context(), a, rotate(30))
	require.NoError(t, err)
	require.False(t, moved.n.Payload.(facetedPayload).copyLineage.direct)
}

// looseBounds widens every held bound of body by half a millimetre. A wider
// bound is still a valid upper bound, so the held placement check accepts it,
// but a direct pairwise slab check must charge it.
func looseBounds(body *Body) {
	const delta = 0.5
	part := body.n.Payload.(facetedPayload)
	part.vertexBound = append([]float64(nil), part.vertexBound...)
	for i := range part.vertexBound {
		part.vertexBound[i] += delta
	}
	part.meshBound += delta
	body.n.Payload = part
}

func TestFacetedCopyTransferSeparation(t *testing.T) {
	_, source, _ := neutralArcSeamSource(t)
	rotate := func(degrees float64) r3.Transform {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(degrees))
		require.NoError(t, err)
		return turn
	}
	copyOf := func(body *Body, degrees float64) *Body {
		image, err := Copy(t.Context(), body, WithCopyPlacement(rotate(degrees)))
		require.NoError(t, err)
		return image
	}
	// Source plus two copies is the smallest set for which shell separation
	// enters reference-gap transfer. The tooth clears its neighbour from about
	// 13 degrees, so 20 degree steps leave real clearance.
	a := copyOf(source, 20)
	direct := copyOf(source, 40)
	nested := copyOf(a, 20)
	require.True(t, direct.n.Payload.(facetedPayload).copyLineage.direct)
	require.False(t, nested.n.Payload.(facetedPayload).copyLineage.direct)
	// Only the second copy carries the wider bound, and both copies sit at the
	// same 40 degree placement, so lineage is the only difference between sets.
	looseBounds(direct)
	looseBounds(nested)

	t.Run("direct set takes transfer", func(t *testing.T) {
		// Transfer ignores the second copy's own bound because a direct copy's
		// true solid is the source's image. A pairwise check must charge it and
		// refuses, so certification succeeding proves transfer replaced it. The mixed
		// lineage subtest shows that refusal on the same geometry.
		tools := []*Body{source, a, direct}
		set, err := certifyFacetedCopies(t.Context(), tools)
		require.NoError(t, err)
		require.Equal(t, shellseparation.AxisX, set.axis)
	})
	t.Run("mixed lineage refuses", func(t *testing.T) {
		// Same geometry, but one copy is a copy of a copy. sourceLinked stays
		// false, so only pairwise checks run and they refuse the wider bound.
		_, err := certifyFacetedCopies(t.Context(), []*Body{source, a, nested})
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "not separate")
	})
	t.Run("uneven spacing refuses near clearance", func(t *testing.T) {
		// The 20 to 30 degree pair is 10 degrees apart, inside the clearance,
		// while the reference gap from the 20 degree copy is positive. Transfer
		// must charge the 10 degree mismatch and decline, then the pairwise
		// check refuses. One axis keeps the exact slab work small.
		tools := []*Body{source, a, copyOf(source, 30)}
		shells := make([]shellseparation.Shell, len(tools))
		moves := make([]r3.Transform, len(tools))
		moves[0] = r3.Identity()
		for i, tool := range tools {
			part := tool.n.Payload.(facetedPayload)
			shells[i] = facetedShell(part)
			if i > 0 {
				moves[i] = part.copyLineage.motion
			}
		}
		require.False(t, shellseparation.Separate(t.Context(), shells, moves,
			nil, shellseparation.SourceShell, shellseparation.AxisX, true))
	})
}
