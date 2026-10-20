package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftfacestrim"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftFacesCapRecordChecksVariableSourceShape(t *testing.T) {
	source := &pointSectionRecord{SourceStation: make([]planePoint, 4)}
	record := &loftFacesCapRecord{Source: source, StationGroup: make([]int, 4)}
	for _, count := range []int{2, 3, 9, maxLoftFacesSections} {
		record.Maps = make([]loftFaceMap, count)
		record.Bounds = make([]loftFaceCapBounds, count)
		require.True(t, loftfacestrim.CapRecordReady(record), "cap count %d", count)
	}
	record.Bounds = record.Bounds[:len(record.Bounds)-1]
	require.False(t, loftfacestrim.CapRecordReady(record))
	record.Maps = make([]loftFaceMap, maxLoftFacesSections+1)
	record.Bounds = make([]loftFaceCapBounds, len(record.Maps))
	require.False(t, loftfacestrim.CapRecordReady(record))
	record.Maps = make([]loftFaceMap, 3)
	record.Bounds = make([]loftFaceCapBounds, 3)
	record.StationGroup = record.StationGroup[:3]
	require.False(t, loftfacestrim.CapRecordReady(record))
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
