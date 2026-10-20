package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftFacesCapRecordChecksVariableSourceShape(t *testing.T) {
	source := &pointSectionRecord{SourceStation: make([]planePoint, 4)}
	record := &loftFacesCapRecord{Source: source, StationGroup: make([]int, 4)}
	for _, count := range []int{2, 3, 9, maxLoftFacesSections} {
		record.Maps = make([]loftFaceMap, count)
		record.Bounds = make([]loftFaceCapBounds, count)
		require.True(t, loftFacesCapRecordReady(record), "cap count %d", count)
	}
	record.Bounds = record.Bounds[:len(record.Bounds)-1]
	require.False(t, loftFacesCapRecordReady(record))
	record.Maps = make([]loftFaceMap, maxLoftFacesSections+1)
	record.Bounds = make([]loftFaceCapBounds, len(record.Maps))
	require.False(t, loftFacesCapRecordReady(record))
	record.Maps = make([]loftFaceMap, 3)
	record.Bounds = make([]loftFaceCapBounds, 3)
	record.StationGroup = record.StationGroup[:3]
	require.False(t, loftFacesCapRecordReady(record))
}

func TestLoftCapConeRecordDoesNotSurviveRoundTripPlacement(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 20, 20, 8)
	tool := internalDiscBody(t, doc, 2, 30)
	lift, err := r3.Translation(r3.NewVec(10, 10, -10))
	require.NoError(t, err)
	tool, err = Transform(t.Context(), tool, lift)
	require.NoError(t, err)
	drilled, err := Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	p := drilled.n.Payload.(facetedPayload)
	p.LoftFacesCap = &loftFacesCapRecord{}
	p.LoftFacesToe = &loftFacesToeRecord{}
	p.LoftFacesTrim = &loftFacesTrimRecord{}
	drilled.n.Payload = p
	move, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	moved, err := Transform(t.Context(), drilled, move)
	require.NoError(t, err)
	back, err := move.Inverse()
	require.NoError(t, err)
	roundTrip, err := Transform(t.Context(), moved, back)
	require.NoError(t, err)
	final := roundTrip.n.Payload.(facetedPayload)
	require.Nil(t, final.LoftFacesCap)
	require.Nil(t, final.LoftFacesToe)
	require.Nil(t, final.LoftFacesTrim)
}

func TestLoftCapTangentToHeelCone(t *testing.T) {
	apex, slope := big.NewRat(8, 1), big.NewRat(3, 4)
	end, err := loftface.NewTangentEnd(apex, slope, 0, 1)
	require.NoError(t, err)
	final := end.Map()
	normal := loftface.VCross(final.QU, final.QV)
	previous := final
	previous.Constant = loftface.VAdd(final.Constant, loftface.VScale(normal, big.NewRat(1, 8)))
	box := [4]*big.Rat{big.NewRat(-4, 1), big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(1, 1)}
	cone := &pointConeLimit{Apex: apex, Far: big.NewRat(-4, 1), Slope: slope}
	record := func() *loftFacesCapRecord {
		return &loftFacesCapRecord{Tangent: &end, Final: final, Maps: []loftFaceMap{previous, previous, final}}
	}

	plane, err := loftCapTangentToHeelCone(record(), box, cone)
	require.NoError(t, err)
	require.NotNil(t, plane)
	require.Equal(t, 1, plane.Side)
	require.False(t, facetedrecord.CapOutsideHeelCone(final, box, cone), "the tangent cap is not on the strict path")

	plain := record()
	plain.Tangent = nil
	plane, err = loftCapTangentToHeelCone(plain, box, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "a plain LoftFaces cap has no tangent scope")

	other := &pointConeLimit{Apex: apex, Far: cone.Far, Slope: big.NewRat(1, 1)}
	plane, err = loftCapTangentToHeelCone(record(), box, other)
	require.NoError(t, err)
	require.Nil(t, plane, "the end was built for another cone")

	changed := record()
	changed.Final.Q0 = loftface.RatVec{big.NewRat(0, 1), big.NewRat(1, 1<<40), big.NewRat(0, 1)}
	plane, err = loftCapTangentToHeelCone(changed, box, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "the recorded final map is not the constructed one")

	straddling := [4]*big.Rat{big.NewRat(-1, 1), big.NewRat(1, 1), big.NewRat(-1, 1), big.NewRat(1, 1)}
	plane, err = loftCapTangentToHeelCone(record(), straddling, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "the box contains the apex line u = 0")

	wrongSide := record()
	wrongSide.Maps = []loftFaceMap{final, previous, final}
	wrongSide.Maps[1].Constant = loftface.VSub(final.Constant, loftface.VScale(normal, big.NewRat(1, 8)))
	plane, err = loftCapTangentToHeelCone(wrongSide, box, cone)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, plane)

	earlier := record()
	earlier.Maps = []loftFaceMap{previous, previous, final}
	earlier.Maps[0].Constant = loftface.VSub(final.Constant, loftface.VScale(normal, big.NewRat(1, 8)))
	plane, err = loftCapTangentToHeelCone(earlier, box, cone)
	require.ErrorIs(t, err, ErrUnsupported, "G2 refuses an earlier cap across the plane")
	require.Nil(t, plane)
}
