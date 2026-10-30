package facetedrecord

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/stretchr/testify/require"
)

// TestChargeAreaSlack holds a charged body's area bounds to the old bounds
// plus the charge, exactly, and to those of the body assembled with the larger
// slack up to the rounding of the sums. The payload carries the larger slack.
func TestChargeAreaSlack(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	assemble := func(slack float64) *topo.Body {
		m := torusMesh(24, 12)
		m.AreaSlack = slack
		body, _, _, err := AssembleBody(t.Context(), env, m, nil, measurement.Measurement{}, nil)
		require.NoError(t, err)
		body.Payload = Payload{AreaSlack: slack}
		return body
	}
	const slack, extra = 1e-7, 0.25
	old := assemble(slack)
	charged := assemble(slack)
	require.NoError(t, ChargeAreaSlack(charged, extra))
	want := assemble(proofbound.AbsSumUpper(slack, extra))

	got, ok := charged.Payload.(Payload)
	require.True(t, ok)
	require.Equal(t, want.Payload.(Payload).AreaSlack, got.AreaSlack)
	rat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	// covers reports whether bound reaches past before + extra, and lies
	// within a few ulps of the reassembled bound.
	covers := func(bound, before, reassembled float64) bool {
		floor := new(big.Rat).Add(rat(before), rat(extra))
		gap := new(big.Rat).Abs(new(big.Rat).Sub(rat(bound), rat(reassembled)))
		return rat(bound).Cmp(floor) >= 0 && gap.Cmp(new(big.Rat).Mul(rat(reassembled), big.NewRat(1, 1<<48))) <= 0
	}
	oldFaces, gotFaces, wantFaces := topobuild.Faces(old), topobuild.Faces(charged), topobuild.Faces(want)
	require.Len(t, gotFaces, len(wantFaces))
	bad := 0
	for i := range gotFaces {
		if !covers(gotFaces[i].AreaBound, oldFaces[i].AreaBound, wantFaces[i].AreaBound) {
			bad++
		}
	}
	require.Zero(t, bad, "faces whose charged bound misses the old bound plus the charge")
	require.True(t, covers(charged.Area.Bound.Base(), old.Area.Bound.Base(), want.Area.Bound.Base()))
	require.Equal(t, want.Area.Exactness, charged.Area.Exactness)

	t.Run("refusals", func(t *testing.T) {
		t.Parallel()
		body := assemble(slack)
		before := body.Area
		require.NoError(t, ChargeAreaSlack(body, 0))
		require.Equal(t, before, body.Area)
		require.ErrorIs(t, ChargeAreaSlack(body, -1), decaderr.ErrUnsupported)
		body.Payload = nil
		require.ErrorIs(t, ChargeAreaSlack(body, extra), decaderr.ErrUnsupported)
	})
}
