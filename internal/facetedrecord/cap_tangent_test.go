package facetedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/stretchr/testify/require"
)

func TestLoftCapTangentToHeelCone(t *testing.T) {
	apex, slope := big.NewRat(8, 1), big.NewRat(3, 4)
	end, err := loftface.NewTangentEnd(apex, slope, 0, 1)
	require.NoError(t, err)
	final := end.Map()
	normal := loftface.VCross(final.QU, final.QV)
	previous := final
	previous.Constant = loftface.VAdd(final.Constant, loftface.VScale(normal, big.NewRat(1, 8)))
	box := [4]*big.Rat{big.NewRat(-4, 1), big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(1, 1)}
	cone := &facetedrecord.PointConeLimit{Axis: coneaxis.X(), Apex: apex, Far: big.NewRat(-4, 1), Slope: slope}
	record := func() *facetedrecord.LoftFacesCapRecord {
		return &facetedrecord.LoftFacesCapRecord{Tangent: &end, Final: final, Maps: []loftface.Map{previous, previous, final}}
	}

	plane, err := facetedrecord.CapTangentToHeelCone(record(), box, cone)
	require.NoError(t, err)
	require.NotNil(t, plane)
	require.Equal(t, 1, plane.Side)
	require.False(t, facetedrecord.CapOutsideHeelCone(final, box, cone), "the tangent cap is not on the strict path")

	plane, err = facetedrecord.CapTangentToHeelCone(record(), box, aboutY(cone))
	require.NoError(t, err)
	require.Nil(t, plane, "a cone about world Y has no tangent scope")

	plain := record()
	plain.Tangent = nil
	plane, err = facetedrecord.CapTangentToHeelCone(plain, box, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "a plain LoftFaces cap has no tangent scope")

	other := &facetedrecord.PointConeLimit{Axis: coneaxis.X(), Apex: apex, Far: cone.Far, Slope: big.NewRat(1, 1)}
	plane, err = facetedrecord.CapTangentToHeelCone(record(), box, other)
	require.NoError(t, err)
	require.Nil(t, plane, "the end was built for another cone")

	changed := record()
	changed.Final.Q0 = loftface.RatVec{big.NewRat(0, 1), big.NewRat(1, 1<<40), big.NewRat(0, 1)}
	plane, err = facetedrecord.CapTangentToHeelCone(changed, box, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "the recorded final map is not the constructed one")

	straddling := [4]*big.Rat{big.NewRat(-1, 1), big.NewRat(1, 1), big.NewRat(-1, 1), big.NewRat(1, 1)}
	plane, err = facetedrecord.CapTangentToHeelCone(record(), straddling, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "the box contains the apex line u = 0")

	wrongSide := record()
	wrongSide.Maps = []loftface.Map{final, previous, final}
	wrongSide.Maps[1].Constant = loftface.VSub(final.Constant, loftface.VScale(normal, big.NewRat(1, 8)))
	plane, err = facetedrecord.CapTangentToHeelCone(wrongSide, box, cone)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the heel cone is not provably on the loft side")
	require.Nil(t, plane)

	short := record()
	short.Maps = []loftface.Map{final}
	plane, err = facetedrecord.CapTangentToHeelCone(short, box, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "a tangent cap with no earlier cap is not scoped")

	earlier := record()
	earlier.Maps = []loftface.Map{previous, previous, final}
	earlier.Maps[0].Constant = loftface.VSub(final.Constant, loftface.VScale(normal, big.NewRat(1, 8)))
	plane, err = facetedrecord.CapTangentToHeelCone(earlier, box, cone)
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "G2 refuses an earlier cap across the plane")
	require.ErrorContains(t, err, "an earlier cap is not provably on the loft side")
	require.Nil(t, plane)
}
