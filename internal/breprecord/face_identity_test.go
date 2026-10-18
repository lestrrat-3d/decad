package breprecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestMatchRecordFaceRequiresReceiverProducerAndRole(t *testing.T) {
	t.Parallel()
	receiver := &topo.Body{Origin: provenance.Ref{Producer: 7}}
	face := &topo.Face{Origins: []provenance.Ref{{Producer: 7, Role: "wall(2)"}}}
	require.True(t, breprecord.MatchRecordFace(receiver, face, nil, 2))
	require.False(t, breprecord.MatchRecordFace(receiver, face, nil, 1))
	require.False(t, breprecord.MatchRecordFace(receiver, nil, nil, 2))

	face.Origins[0].Producer = 8
	require.False(t, breprecord.MatchRecordFace(receiver, face, nil, 2))
	face.Origins[0] = provenance.Ref{Producer: 7, Role: "cap-top"}
	require.True(t, breprecord.MatchRecordFace(receiver, face, []string{"side", "cap-top"}, 1))
	require.False(t, breprecord.MatchRecordFace(receiver, face, []string{"side"}, 1))
}
