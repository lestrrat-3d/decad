package tessellation

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestBlindContactMaximumWidthCrossing(t *testing.T) {
	var p, q blindContactVector
	for i := range p {
		p[i].limbs[0], p[i].limbs[1], p[i].limbs[2] = ^uint64(0), ^uint64(0), (1<<20)-1
		q[i].limbs[0] = 1
		q[i].negative = true
	}
	var dp, dq blindContactInt
	for i := range 7 {
		dp.limbs[i] = ^uint64(0)
	}
	dp.limbs[7] = 3
	dq = dp
	dp.negative = true
	budget := proofbound.NewReplayTrialBudget()
	a := blindContactArithmetic{t.Context(), budget}
	var forward, reverse [4]blindContactInt
	var products [2]blindContactInt
	var zero blindContactInt
	err := a.crossingContactEndpoint(&p, &q, &dp, &dq, &forward, &products, &zero)
	require.NoError(t, err)
	err = a.crossingContactEndpoint(&q, &p, &dq, &dp, &reverse, &products, &zero)
	require.NoError(t, err)
	for i := range 3 {
		order, err := a.compareContactFractions(&forward[i], &forward[3], &reverse[i], &reverse[3])
		require.NoError(t, err)
		require.Zero(t, order)
	}
	work, _ := budget.Usage()
	short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
	require.NoError(t, err)
	b := blindContactArithmetic{t.Context(), short}
	forward, reverse = [4]blindContactInt{}, [4]blindContactInt{}
	products = [2]blindContactInt{}
	err = b.crossingContactEndpoint(&p, &q, &dp, &dq, &forward, &products, &zero)
	require.NoError(t, err)
	err = b.crossingContactEndpoint(&q, &p, &dq, &dp, &reverse, &products, &zero)
	require.NoError(t, err)
	for i := range 3 {
		_, err = b.compareContactFractions(&forward[i], &forward[3], &reverse[i], &reverse[3])
		if err != nil {
			break
		}
	}
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
