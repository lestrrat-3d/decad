package loftrecord

import (
	"reflect"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// A cloned pairing shares no slice with its source, so a build that writes to
// its copy leaves the held pairing as it was.
func TestRecordPairingCloneSharesNothing(t *testing.T) {
	t.Parallel()
	// clone copies these fields by name; a new LoopPair field must join it.
	require.Equal(t, 9, reflect.TypeFor[loftmesh.LoopPair]().NumField())
	src := recordPairing{
		pairs: []loftmesh.LoopPair{{
			V: []sectionrecord.Point2{{U: 1}}, W: []sectionrecord.Point2{{V: 2}},
			ArcUpperV: []float64{3}, ArcUpperW: []float64{4}, MatchedDelta: []float64{5},
			TangentEnergyV: []float64{6}, TangentEnergyW: []float64{7},
			Segment: []int{8}, Faceted: []bool{true},
		}},
		sectionDelta: 9, sectionMatchedDelta: 10, stationRound: 11,
	}
	out := src.clone()
	require.True(t, sectionrecord.IdenticalRecord(src.pairs, out.pairs))
	require.Equal(t, [3]float64{9, 10, 11}, [3]float64{out.sectionDelta, out.sectionMatchedDelta, out.stationRound})

	p := &out.pairs[0]
	p.V[0].U, p.W[0].V = -1, -1
	p.ArcUpperV[0], p.ArcUpperW[0], p.MatchedDelta[0] = -1, -1, -1
	p.TangentEnergyV[0], p.TangentEnergyW[0] = -1, -1
	p.Segment[0], p.Faceted[0] = -1, false
	s := src.pairs[0]
	require.Equal(t, []float64{1, 2, 3, 4, 5, 6, 7}, []float64{s.V[0].U, s.W[0].V, s.ArcUpperV[0], s.ArcUpperW[0],
		s.MatchedDelta[0], s.TangentEnergyV[0], s.TangentEnergyW[0]})
	require.Equal(t, 8, s.Segment[0])
	require.True(t, s.Faceted[0])
}
