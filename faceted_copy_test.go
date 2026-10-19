package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func exactCopyOf(t *testing.T, body *Body, degrees float64) *Body {
	t.Helper()
	image, err := Copy(t.Context(), body, WithCopyRotation(r3.Vec{}, r3.Vec{X: 1}, units.Degrees(degrees)))
	require.NoError(t, err)
	require.NotNil(t, image.n.Payload.(facetedPayload).CopyLineage.Exact)
	return image
}

func ordinaryCopyOf(t *testing.T, body *Body, degrees float64) *Body {
	t.Helper()
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(degrees))
	require.NoError(t, err)
	image, err := Copy(t.Context(), body, WithCopyPlacement(turn))
	require.NoError(t, err)
	return image
}

func TestCertifyFacetedCopiesPassesExactMaps(t *testing.T) {
	_, source, _ := neutralArcSeamSource(t)
	// As TestFacetedCopyTransferSeparation: the widened bound of the second
	// copy is chargeable only by a direct pairwise check, so a certified set
	// shows that reference-gap transfer ran.
	a := exactCopyOf(t, source, 20)
	b := exactCopyOf(t, source, 40)
	looseBounds(b)

	set, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, a, b}))
	require.NoError(t, err)
	require.Len(t, set.Exact, 3)
	require.Len(t, set.Maps, 3)
	require.Nil(t, set.Exact[0])
	for i, tool := range set.Tools[1:] {
		record := tool.Payload.(facetedPayload).CopyLineage.Exact
		require.Same(t, record, set.Exact[i+1])
		require.Equal(t, record.Linear(), set.Maps[i+1])
	}
	require.Nil(t, set.Maps[0][0][0])

	t.Run("ordinary placement passes no maps", func(t *testing.T) {
		c := ordinaryCopyOf(t, source, 20)
		d := ordinaryCopyOf(t, source, 40)
		looseBounds(d)
		set, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, c, d}))
		require.NoError(t, err)
		require.Nil(t, set.Maps)
		require.Equal(t, []*exactrotation.Rotation{nil, nil, nil}, set.Exact)
	})
	t.Run("mixed set maps only the exact copy", func(t *testing.T) {
		c := ordinaryCopyOf(t, source, 20)
		d := exactCopyOf(t, source, 40)
		looseBounds(d)
		set, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, c, d}))
		require.NoError(t, err)
		require.Nil(t, set.Maps[1][0][0])
		require.NotNil(t, set.Maps[2][0][0])
	})
	t.Run("nonzero exact translation turns transfer off", func(t *testing.T) {
		// Same linear map as the origin rotation, with the translation of a
		// rotation about another center. The float move still has no
		// translation, so only the exact record can switch transfer off.
		shifted, _, err := exactrotation.New(r3.Vec{Y: 1, Z: 1}, r3.Vec{X: 1}, units.Degrees(40))
		require.NoError(t, err)
		moved := false
		for _, entry := range shifted.Translation() {
			moved = moved || entry.Sign() != 0
		}
		require.True(t, moved)
		c := exactCopyOf(t, source, 20)
		d := exactCopyOf(t, source, 40)
		looseBounds(d)
		part := d.n.Payload.(facetedPayload)
		lineage := *part.CopyLineage
		lineage.Exact = shifted
		part.CopyLineage = &lineage
		d.n.Payload = part
		_, err = facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, c, d}))
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "not separate")
	})
	t.Run("transfer composes the recorded exact maps", func(t *testing.T) {
		// The second copy's record names a 70 degree rotation while its float
		// move is 40 degrees. Exact maps do not compose (20 + 20 != 70), so
		// transfer declines and the widened bound fails the pairwise check.
		// Float moves would compose and certify the set.
		c := exactCopyOf(t, source, 20)
		d := exactCopyOf(t, source, 40)
		looseBounds(d)
		other, _, err := exactrotation.New(r3.Vec{}, r3.Vec{X: 1}, units.Degrees(70))
		require.NoError(t, err)
		part := d.n.Payload.(facetedPayload)
		lineage := *part.CopyLineage
		lineage.Exact = other
		part.CopyLineage = &lineage
		d.n.Payload = part
		_, err = facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, c, d}))
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "not separate")
	})
	t.Run("exact record needs a direct placement", func(t *testing.T) {
		c := exactCopyOf(t, source, 20)
		part := c.n.Payload.(facetedPayload)
		lineage := *part.CopyLineage
		lineage.Direct = false
		part.CopyLineage = &lineage
		c.n.Payload = part
		_, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, c}))
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "without a direct placement")
	})
}

func TestExactRotationAtTinyAngleHasHiddenTranslation(t *testing.T) {
	// The review's case: the float placement of a 2^-60 rotation about
	// (0,1,1) carries no translation, while the exact rotation does.
	center, axis, angle := r3.Vec{Y: 1, Z: 1}, r3.Vec{X: 1}, units.Radians(0x1p-60)
	record, float, err := exactrotation.New(center, axis, angle)
	require.NoError(t, err)
	require.Equal(t, r3.Vec{}, float.Translation())
	nonzero := false
	for _, entry := range record.Translation() {
		nonzero = nonzero || entry.Sign() != 0
	}
	require.True(t, nonzero)
	require.False(t, record.FixesXAxisAtOrigin())
}

func TestPlacedProxyShellsChargeExactCopies(t *testing.T) {
	_, source, _ := neutralArcSeamSource(t)
	proxy, _, err := tessellateBooleanOperand(t.Context(), source, 0.1)
	require.NoError(t, err)
	image := exactCopyOf(t, source, 40)
	twin := ordinaryCopyOf(t, source, 40)

	set, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, image}))
	require.NoError(t, err)
	placed, err := facetedrecord.PlacedProxyShells(t.Context(), set, meshNode(proxy))
	require.NoError(t, err)
	twinSet, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, twin}))
	require.NoError(t, err)
	legacy, err := facetedrecord.PlacedProxyShells(t.Context(), twinSet, meshNode(proxy))
	require.NoError(t, err)
	require.Equal(t, legacy[1].Verts, placed[1].Verts, "the held placement is the same float transform")

	record := set.Exact[1]
	linearGap, translationGap, ok := record.Gap(set.Moves[1])
	require.True(t, ok)
	want, err := facetproof.PlaceMeshDenoting(t.Context(), proofbound.NewWorkBudget(t.Context()),
		proxy.n.Vertices, proxy.n.Triangles, proxy.n.VertexBound, proxy.n.VolSymDiff,
		proxy.n.AreaSlack, set.Moves[1], linearGap, translationGap)
	require.NoError(t, err)
	require.Equal(t, want, placed[1], "an exact copy's proxy pays the exact-rotation charge")
	require.NotEqual(t, legacy[1].VertexBound, placed[1].VertexBound,
		"the float matrix's stretch charge is not paid")

	// Every placed vertex lies within its bound of the exact image Q*v + t.
	linear, translation := record.Linear(), record.Translation()
	for i, v := range proxy.n.Vertices {
		src := [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y),
			new(big.Rat).SetFloat64(v.Z)}
		got := [3]float64{placed[1].Verts[i].X, placed[1].Verts[i].Y, placed[1].Verts[i].Z}
		limit := new(big.Rat).SetFloat64(placed[1].VertexBound[i])
		for row := range linear {
			image := new(big.Rat).Set(translation[row])
			for col := range src {
				image.Add(image, new(big.Rat).Mul(linear[row][col], src[col]))
			}
			miss := new(big.Rat).Sub(image, new(big.Rat).SetFloat64(got[row]))
			require.LessOrEqual(t, miss.Abs(miss).Cmp(limit), 0, "vertex %d row %d", i, row)
		}
	}

	t.Run("a swapped record is refused", func(t *testing.T) {
		part := image.n.Payload.(facetedPayload)
		lineage := *part.CopyLineage
		lineage.Exact = nil
		part.CopyLineage = &lineage
		image.n.Payload = part
		_, err := facetedrecord.PlacedProxyShells(t.Context(), set, meshNode(proxy))
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "changed its exact rotation")
	})
}

func TestRealCurvedCopiedToolLineage(t *testing.T) {
	blank, source, _ := neutralArcSeamSource(t)
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
	require.NoError(t, err)
	image, err := Copy(t.Context(), source, WithCopyPlacement(turn))
	require.NoError(t, err)

	set, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, image}))
	require.NoError(t, err)
	require.Same(t, source, bodyHandle(set.Source))
	require.Equal(t, turn, set.Moves[1])
	require.Len(t, set.Held, 2)
	require.Len(t, set.FaceOwner[1], len(source.Faces()))
	proxy, _, err := tessellateBooleanOperand(t.Context(), source, 0.1)
	require.NoError(t, err)
	placed, err := facetedrecord.PlacedProxyShells(t.Context(), set, meshNode(proxy))
	require.NoError(t, err)
	require.Len(t, placed, 2)
	for _, face := range source.Faces() {
		owner := set.FaceOwner[1][faceNode(face)]
		require.NotNil(t, owner)
		require.Same(t, image, bodyHandle(owner.Body))
	}
	require.NotNil(t, blank, "the source is a real Revolve and LoftFaces pair")

	overlap, err := Copy(t.Context(), source)
	require.NoError(t, err)
	before := documentHandle(source.n.Doc).Bodies()
	_, err = facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, overlap}))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "not separate")
	require.Equal(t, before, documentHandle(source.n.Doc).Bodies())

	quarter, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	third, err := Copy(t.Context(), image, WithCopyPlacement(quarter))
	require.NoError(t, err)
	thirdPayload := third.n.Payload.(facetedPayload)
	require.Nil(t, thirdPayload.LoftFacesTrim)
	require.Same(t, source, bodyHandle(thirdPayload.CopyLineage.Source))
	set, err = facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, image, third}))
	require.NoError(t, err)
	require.Len(t, set.FaceOwner[2], len(source.Faces()))

	mirrorFrame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflection, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	mirrored, err := Copy(t.Context(), source, WithCopyPlacement(reflection))
	require.NoError(t, err)
	set, err = facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, mirrored}))
	require.NoError(t, err)
	require.Equal(t, reflection, set.Moves[1])
	placed, err = facetedrecord.PlacedProxyShells(t.Context(), set, meshNode(proxy))
	require.NoError(t, err)
	require.Len(t, placed, 2)

	part := image.n.Payload.(facetedPayload)
	changed := *part.CopyLineage
	changed.Motion = r3.Identity()
	part.CopyLineage = &changed
	image.n.Payload = part
	_, err = facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, image}))
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
	require.True(t, a.n.Payload.(facetedPayload).CopyLineage.Direct)
	require.True(t, b.n.Payload.(facetedPayload).CopyLineage.Direct)

	c, err := Copy(t.Context(), a, WithCopyPlacement(rotate(90)))
	require.NoError(t, err)
	cLineage := c.n.Payload.(facetedPayload).CopyLineage
	require.Same(t, source, bodyHandle(cLineage.Source))
	require.False(t, cLineage.Direct)

	set, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, a, b}))
	require.NoError(t, err)
	require.Equal(t, shellseparation.AxisX, set.Axis)

	// Transfer and the direct pairwise check must agree on the direct set.
	tools := []*Body{source, a, b}
	shells := make([]shellseparation.Shell, len(tools))
	for i, tool := range tools {
		shells[i] = facetedrecord.SeparationShell(tool.n.Payload.(facetedPayload))
	}
	for _, linked := range []bool{true, false} {
		require.True(t, shellseparation.Separate(t.Context(), shells, set.Moves,
			nil, shellseparation.SourceShell, shellseparation.AxisX, linked), "linked=%v", linked)
	}

	_, err = facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, a, c}))
	require.NoError(t, err)

	moved, err := Transform(t.Context(), a, rotate(30))
	require.NoError(t, err)
	require.False(t, moved.n.Payload.(facetedPayload).CopyLineage.Direct)
}

// looseBounds widens every held bound of body by half a millimetre. A wider
// bound is still a valid upper bound, so the held placement check accepts it,
// but a direct pairwise slab check must charge it.
func looseBounds(body *Body) {
	const delta = 0.5
	part := body.n.Payload.(facetedPayload)
	part.VertexBound = append([]float64(nil), part.VertexBound...)
	for i := range part.VertexBound {
		part.VertexBound[i] += delta
	}
	part.MeshBound += delta
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
	require.True(t, direct.n.Payload.(facetedPayload).CopyLineage.Direct)
	require.False(t, nested.n.Payload.(facetedPayload).CopyLineage.Direct)
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
		set, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes(tools))
		require.NoError(t, err)
		require.Equal(t, shellseparation.AxisX, set.Axis)
	})
	t.Run("mixed lineage refuses", func(t *testing.T) {
		// Same geometry, but one copy is a copy of a copy. sourceLinked stays
		// false, so only pairwise checks run and they refuse the wider bound.
		_, err := facetedrecord.CertifyCopies(t.Context(), bodyNodes([]*Body{source, a, nested}))
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
			shells[i] = facetedrecord.SeparationShell(part)
			if i > 0 {
				moves[i] = part.CopyLineage.Motion
			}
		}
		require.False(t, shellseparation.Separate(t.Context(), shells, moves,
			nil, shellseparation.SourceShell, shellseparation.AxisX, true))
	})
}
