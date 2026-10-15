package compositesweep

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestEndpointSupportsStopsAtHalfTurn(t *testing.T) {
	t.Parallel()
	body := &topo.Body{Payload: revolveplan.Payload{Den: revolveangle.Sweep{
		Phi0: revolveangle.Angle{Rad: new(big.Rat), Turn: new(big.Rat)},
		Phi1: revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(1, 2)},
	}}}
	require.True(t, endpointSupports(body))

	payload := body.Payload.(revolveplan.Payload)
	payload.Den.Phi1.Turn = big.NewRat(3, 4)
	body.Payload = payload
	require.False(t, endpointSupports(body))

	payload.Full = true
	payload.Den.Phi1.Turn = big.NewRat(1, 2)
	body.Payload = payload
	require.False(t, endpointSupports(body))

	require.True(t, endpointSupports(&topo.Body{Payload: prismrecord.Payload{}}))
	require.False(t, endpointSupports(&topo.Body{}))
}
