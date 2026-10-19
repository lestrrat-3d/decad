package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestChamferSourceOf reads the cap Chamfer source from each loft/cylinder
// record and refuses a payload that is unaudited, placed, or has no record.
func TestChamferSourceOf(t *testing.T) {
	t.Parallel()
	blank := &topo.Body{}
	teeth := []*topo.Body{{}, {}}
	proof := []*topo.Body{{}, {}}
	join := &facetedrecord.LoftCylinderJoinRecord{
		Blank: blank, Teeth: teeth, ProofTeeth: proof, AxialBridge: []float64{0, 1e-15},
		CapOutlines: [2][]int{{0, 1, 2}, {3, 4, 5}}, CapGroups: [2]int{1, 2},
		Radius: 10, InnerInset: 0.25, OuterExcess: 0.5, RootCollar: 0.75, Z0: -1, Z1: 4,
	}
	bore := &facetedrecord.LoftCylinderBoreRecord{
		Blank: blank, Teeth: teeth, ProofTeeth: proof, AxialBridge: []float64{0, 0},
		Outer: [2][]int{{0, 1, 2}, {3, 4, 5}}, Inner: [2][]int{{6, 7, 8}, {9, 10, 11}},
		CapGroups: [2]int{3, 4}, WallGroup: 5,
		BlankRadius: 10, BoreRadius: 2, BoreBound: 0.125,
		InnerInset: 0.25, OuterExcess: 0.5, RootCollar: 0.75, Z0: 0, Z1: 3,
	}
	audited := facetedrecord.Payload{ContactAudited: true, Xform: r3.Identity()}
	placed, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err, "translation must build")

	t.Run("join", func(t *testing.T) {
		t.Parallel()
		fp := audited
		fp.LoftCylinderJoin = join
		got, ok := loftcylinder.ChamferSourceOf(fp)
		require.True(t, ok, "audited join record must give a source")
		require.Equal(t, loftcylinder.ChamferSource{
			Blank: blank, Teeth: proof, OriginalTeeth: teeth, AxialBridge: join.AxialBridge,
			Outer: join.CapOutlines, CapGroups: join.CapGroups, BoreGroup: -1,
			BlankRadius: 10, InnerInset: 0.25, OuterExcess: 0.5, RootCollar: 0.75, Z0: -1, Z1: 4,
		}, got)
	})
	t.Run("bore wins over join", func(t *testing.T) {
		t.Parallel()
		fp := audited
		fp.LoftCylinderJoin = join
		fp.LoftCylinderBore = bore
		got, ok := loftcylinder.ChamferSourceOf(fp)
		require.True(t, ok, "audited bore record must give a source")
		require.Equal(t, loftcylinder.ChamferSource{
			Blank: blank, Teeth: proof, OriginalTeeth: teeth, AxialBridge: bore.AxialBridge,
			Outer: bore.Outer, Inner: bore.Inner, CapGroups: bore.CapGroups, BoreGroup: 5,
			BlankRadius: 10, BoreRadius: 2, BoreBound: 0.125,
			InnerInset: 0.25, OuterExcess: 0.5, RootCollar: 0.75, Z0: 0, Z1: 3,
		}, got)
	})
	for _, tc := range []struct {
		name string
		fp   facetedrecord.Payload
	}{
		{"no record", audited},
		{"unaudited", facetedrecord.Payload{Xform: r3.Identity(), LoftCylinderJoin: join}},
		{"placed", facetedrecord.Payload{ContactAudited: true, Xform: placed, LoftCylinderJoin: join}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ok := loftcylinder.ChamferSourceOf(tc.fp)
			require.False(t, ok, "payload must not give a chamfer source")
		})
	}
}

// TestChamferSourceBindBridgeRefusal leaves the source's bridge fields unset
// when its tooth lists cannot be paired.
func TestChamferSourceBindBridgeRefusal(t *testing.T) {
	t.Parallel()
	source := loftcylinder.ChamferSource{
		Teeth: []*topo.Body{{}}, OriginalTeeth: []*topo.Body{{}, {}},
		AxialBridge: []float64{0}, BridgeBound: 0.5, Z0: 0, Z1: 1,
	}
	err := source.BindBridge()
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Equal(t, 0.5, source.BridgeBound, "a refused bind must keep the recorded bridge bound")
	require.Nil(t, source.TopSourceLevel, "a refused bind must not set a source level")
}
