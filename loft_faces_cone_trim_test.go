package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftCapConeProofRequiresStrictWholeCapSeparation(t *testing.T) {
	zero, one := lfZero(), lfOne()
	near := big.NewRat(10000000000000001, 10000000000000000)
	m := loftFaceMap{
		constant: loftFaceRatVec{zero, zero, zero},
		q0:       loftFaceRatVec{big.NewRat(10, 1), zero, zero},
		qu:       loftFaceRatVec{one, near, zero},
		qv:       loftFaceRatVec{zero, zero, big.NewRat(3, 1)},
		g0:       one, gu: zero, gv: zero,
	}
	box := [4]*big.Rat{big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(-1, 1), one}
	cone := &pointConeLimit{apex: big.NewRat(10, 1), far: zero, slope: one}
	if !loftCapOutsideHeelCone(m, box, cone) {
		t.Fatal("the exact positive quadratic with a small radial margin was refused")
	}
	changed := m
	changed.qu[1] = one
	if loftCapOutsideHeelCone(changed, box, cone) {
		t.Fatal("a cap that touches the cone at v=0 was admitted")
	}
	changed = m
	changed.qv[1] = one
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
		constant: loftFaceRatVec{zero, zero, zero},
		q0:       loftFaceRatVec{big.NewRat(5, 1), big.NewRat(10, 1), zero},
		qu:       loftFaceRatVec{zero, zero, zero},
		qv:       loftFaceRatVec{zero, zero, zero},
		g0:       one, gu: zero, gv: zero,
	}
	box := [4]*big.Rat{big.NewRat(-3, 1), big.NewRat(-1, 1), big.NewRat(-1, 1), one}
	cone := &pointConeLimit{apex: big.NewRat(8, 1), far: zero, slope: one}
	if !loftCapOutsideToeCone(m, box, cone) {
		t.Fatal("the cap strictly outside the toe cone was refused")
	}
	m.q0[1] = big.NewRat(2, 1)
	if loftCapOutsideToeCone(m, box, cone) {
		t.Fatal("the cap inside the toe cone was admitted")
	}
}

func TestLoftFacesCapRecordChecksVariableSourceShape(t *testing.T) {
	source := &pointSectionRecord{sourceStation: make([]Point2, 4)}
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
	p := drilled.payload.(facetedPayload)
	p.loftFacesCap = &loftFacesCapRecord{}
	p.loftFacesToe = &loftFacesToeRecord{}
	p.loftFacesTrim = &loftFacesTrimRecord{}
	drilled.payload = p
	move, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	moved, err := Transform(t.Context(), drilled, move)
	require.NoError(t, err)
	back, err := move.Inverse()
	require.NoError(t, err)
	roundTrip, err := Transform(t.Context(), moved, back)
	require.NoError(t, err)
	final := roundTrip.payload.(facetedPayload)
	require.Nil(t, final.loftFacesCap)
	require.Nil(t, final.loftFacesToe)
	require.Nil(t, final.loftFacesTrim)
}
