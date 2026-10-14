package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftCapConeProofRequiresStrictWholeCapSeparation(t *testing.T) {
	zero, one := lfZero(), lfOne()
	near := big.NewRat(10000000000000001, 10000000000000000)
	m := loftFaceMap{
		Constant: loftFaceRatVec{zero, zero, zero},
		Q0:       loftFaceRatVec{big.NewRat(10, 1), zero, zero},
		QU:       loftFaceRatVec{one, near, zero},
		QV:       loftFaceRatVec{zero, zero, big.NewRat(3, 1)},
		G0:       one, GU: zero, GV: zero,
	}
	box := [4]*big.Rat{big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(-1, 1), one}
	cone := &pointConeLimit{apex: big.NewRat(10, 1), far: zero, slope: one}
	if !loftCapOutsideHeelCone(m, box, cone) {
		t.Fatal("the exact positive quadratic with a small radial margin was refused")
	}
	changed := m
	changed.QU[1] = one
	if loftCapOutsideHeelCone(changed, box, cone) {
		t.Fatal("a cap that touches the cone at v=0 was admitted")
	}
	changed = m
	changed.QV[1] = one
	if loftCapOutsideHeelCone(changed, box, cone) {
		t.Fatal("an uncertified cross term was admitted")
	}
	box[1] = one
	if loftCapOutsideHeelCone(m, box, cone) {
		t.Fatal("a source rectangle containing the quadratic zero was admitted")
	}
}

func TestLoftCapToeConeProofUsesWholeSourceBox(t *testing.T) {
	zero, one := lfZero(), lfOne()
	m := loftFaceMap{
		Constant: loftFaceRatVec{zero, zero, zero},
		Q0:       loftFaceRatVec{big.NewRat(5, 1), big.NewRat(10, 1), zero},
		QU:       loftFaceRatVec{zero, zero, zero},
		QV:       loftFaceRatVec{zero, zero, zero},
		G0:       one, GU: zero, GV: zero,
	}
	box := [4]*big.Rat{big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(-1, 1), one}
	cone := &pointConeLimit{apex: big.NewRat(8, 1), far: zero, slope: one}
	if !loftCapOutsideToeCone(m, box, cone) {
		t.Fatal("the cap strictly outside the toe cone was refused")
	}
	m.Q0[1] = big.NewRat(2, 1)
	if loftCapOutsideToeCone(m, box, cone) {
		t.Fatal("the cap inside the toe cone was admitted")
	}
}

func TestLoftFacesCapRecordChecksVariableSourceShape(t *testing.T) {
	source := &pointSectionRecord{sourceStation: make([]planePoint, 4)}
	record := &loftFacesCapRecord{source: source, stationGroup: make([]int, 4)}
	for _, count := range []int{2, 3, 9, maxLoftFacesSections} {
		record.maps = make([]loftFaceMap, count)
		record.bounds = make([]loftFaceCapBounds, count)
		require.True(t, loftFacesCapRecordReady(record), "cap count %d", count)
	}
	record.bounds = record.bounds[:len(record.bounds)-1]
	require.False(t, loftFacesCapRecordReady(record))
	record.maps = make([]loftFaceMap, maxLoftFacesSections+1)
	record.bounds = make([]loftFaceCapBounds, len(record.maps))
	require.False(t, loftFacesCapRecordReady(record))
	record.maps = make([]loftFaceMap, 3)
	record.bounds = make([]loftFaceCapBounds, 3)
	record.stationGroup = record.stationGroup[:3]
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
	p.loftFacesCap = &loftFacesCapRecord{}
	p.loftFacesToe = &loftFacesToeRecord{}
	p.loftFacesTrim = &loftFacesTrimRecord{}
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
	require.Nil(t, final.loftFacesCap)
	require.Nil(t, final.loftFacesToe)
	require.Nil(t, final.loftFacesTrim)
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
	cone := &pointConeLimit{apex: apex, far: big.NewRat(-4, 1), slope: slope}
	record := func() *loftFacesCapRecord {
		return &loftFacesCapRecord{tangent: &end, final: final, maps: []loftFaceMap{previous, previous, final}}
	}

	plane, err := loftCapTangentToHeelCone(record(), box, cone)
	require.NoError(t, err)
	require.NotNil(t, plane)
	require.Equal(t, 1, plane.Side)
	require.False(t, loftCapOutsideHeelCone(final, box, cone), "the tangent cap is not on the strict path")

	plain := record()
	plain.tangent = nil
	plane, err = loftCapTangentToHeelCone(plain, box, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "a plain LoftFaces cap has no tangent scope")

	other := &pointConeLimit{apex: apex, far: cone.far, slope: big.NewRat(1, 1)}
	plane, err = loftCapTangentToHeelCone(record(), box, other)
	require.NoError(t, err)
	require.Nil(t, plane, "the end was built for another cone")

	changed := record()
	changed.final.Q0 = loftface.RatVec{big.NewRat(0, 1), big.NewRat(1, 1<<40), big.NewRat(0, 1)}
	plane, err = loftCapTangentToHeelCone(changed, box, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "the recorded final map is not the constructed one")

	straddling := [4]*big.Rat{big.NewRat(-1, 1), big.NewRat(1, 1), big.NewRat(-1, 1), big.NewRat(1, 1)}
	plane, err = loftCapTangentToHeelCone(record(), straddling, cone)
	require.NoError(t, err)
	require.Nil(t, plane, "the box contains the apex line u = 0")

	wrongSide := record()
	wrongSide.maps = []loftFaceMap{final, previous, final}
	wrongSide.maps[1].Constant = loftface.VSub(final.Constant, loftface.VScale(normal, big.NewRat(1, 8)))
	plane, err = loftCapTangentToHeelCone(wrongSide, box, cone)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, plane)

	earlier := record()
	earlier.maps = []loftFaceMap{previous, previous, final}
	earlier.maps[0].Constant = loftface.VSub(final.Constant, loftface.VScale(normal, big.NewRat(1, 8)))
	plane, err = loftCapTangentToHeelCone(earlier, box, cone)
	require.ErrorIs(t, err, ErrUnsupported, "G2 refuses an earlier cap across the plane")
	require.Nil(t, plane)
}
