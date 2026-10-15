package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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

	set, err := certifyFacetedCopies(t.Context(), []*Body{source, a, b})
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
		set, err := certifyFacetedCopies(t.Context(), []*Body{source, c, d})
		require.NoError(t, err)
		require.Nil(t, set.Maps)
		require.Equal(t, []*exactrotation.Rotation{nil, nil, nil}, set.Exact)
	})
	t.Run("mixed set maps only the exact copy", func(t *testing.T) {
		c := ordinaryCopyOf(t, source, 20)
		d := exactCopyOf(t, source, 40)
		looseBounds(d)
		set, err := certifyFacetedCopies(t.Context(), []*Body{source, c, d})
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
		_, err = certifyFacetedCopies(t.Context(), []*Body{source, c, d})
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
		_, err = certifyFacetedCopies(t.Context(), []*Body{source, c, d})
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
		_, err := certifyFacetedCopies(t.Context(), []*Body{source, c})
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

	set, err := certifyFacetedCopies(t.Context(), []*Body{source, image})
	require.NoError(t, err)
	placed, err := placedProxyShells(t.Context(), set, proxy)
	require.NoError(t, err)
	twinSet, err := certifyFacetedCopies(t.Context(), []*Body{source, twin})
	require.NoError(t, err)
	legacy, err := placedProxyShells(t.Context(), twinSet, proxy)
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
		_, err := placedProxyShells(t.Context(), set, proxy)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "changed its exact rotation")
	})
}
