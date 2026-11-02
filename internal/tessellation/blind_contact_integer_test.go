package tessellation

import (
	"context"
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

const replayTestCaseExact = "exact"

func contactBig(x blindContactInt) *big.Int {
	value := new(big.Int)
	for i := len(x.limbs) - 1; i >= 0; i-- {
		value.Lsh(value, 64)
		value.Or(value, new(big.Int).SetUint64(x.limbs[i]))
	}
	if x.negative {
		value.Neg(value)
	}
	return value
}

func TestBlindContactInteger(t *testing.T) {
	rng := rand.New(rand.NewSource(8675309))
	for range 128 {
		var x, y blindContactInt
		for i := range 16 {
			x.limbs[i], y.limbs[i] = rng.Uint64(), rng.Uint64()
		}
		x.negative, y.negative = rng.Intn(2) == 0, rng.Intn(2) == 0
		a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
		sum, err := a.add(&x, &y)
		require.NoError(t, err)
		require.Zero(t, contactBig(sum).Cmp(new(big.Int).Add(contactBig(x), contactBig(y))))
		product, err := a.mul(&x, &y)
		require.NoError(t, err)
		require.Zero(t, contactBig(product).Cmp(new(big.Int).Mul(contactBig(x), contactBig(y))))
		order, err := a.compare(&x, &y)
		require.NoError(t, err)
		require.Equal(t, contactBig(x).Cmp(contactBig(y)), order)
	}
	var x, y blindContactInt
	x.limbs[31], y.limbs[0] = 1<<63, 2
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	_, err := a.mul(&x, &y)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	x.limbs[31], y.limbs[0] = ^uint64(0), 1
	for i := range 31 {
		x.limbs[i] = ^uint64(0)
	}
	_, err = a.add(&x, &y)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestBlindContactNormalizeEndpoint(t *testing.T) {
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	var p [4]blindContactInt
	p[0].limbs[0], p[0].negative = 3, true
	p[1].limbs[0] = 2
	p[3].limbs[0], p[3].negative = 7, true
	require.NoError(t, a.normalizeContactEndpoint(&p))
	require.Equal(t, int64(3), contactBig(p[0]).Int64())
	require.Equal(t, int64(-2), contactBig(p[1]).Int64())
	require.Equal(t, int64(7), contactBig(p[3]).Int64())
	require.False(t, p[2].negative)
	var q [4]blindContactInt
	q[0].limbs[0], q[3].limbs[0] = 1, 2
	order, err := a.compareContactFractions(&p[0], &p[3], &q[0], &q[3])
	require.NoError(t, err)
	require.Equal(t, -1, order)
	order, err = a.compareContactFractions(&q[0], &q[3], &p[0], &p[3])
	require.NoError(t, err)
	require.Equal(t, 1, order)
	p[3] = blindContactInt{}
	require.ErrorIs(t, a.normalizeContactEndpoint(&p), decaderr.ErrUnsupported)
	_, err = a.compareContactFractions(&p[0], &p[3], &q[0], &q[3])
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	var negativeZero, zero blindContactInt
	negativeZero.negative = true
	order, err = a.compare(&negativeZero, &zero)
	require.NoError(t, err)
	require.Zero(t, order)
	var endpoint [4]blindContactInt
	endpoint[0] = negativeZero
	endpoint[3].limbs[0] = 1
	require.NoError(t, a.normalizeContactEndpoint(&endpoint))
	require.False(t, endpoint[0].negative)
	endpoint[0] = negativeZero
	endpoint[1].limbs[0] = 2
	endpoint[3].negative = true
	original := endpoint
	short, err := proofbound.NewReplayBudgetLimits(159, 1, 1)
	require.NoError(t, err)
	require.ErrorIs(t, (blindContactArithmetic{t.Context(), short}).normalizeContactEndpoint(&endpoint), decaderr.ErrUnsupported)
	require.Equal(t, original, endpoint)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, (blindContactArithmetic{ctx, proofbound.NewReplayTrialBudget()}).normalizeContactEndpoint(&endpoint), context.Canceled)
	require.Equal(t, original, endpoint)
}

func TestBlindContactIntegerWorkLimit(t *testing.T) {
	var x, y blindContactInt
	x.limbs[0], y.limbs[0] = 5, 7
	for _, tc := range []struct {
		name string
		work uint64
		pass bool
	}{
		{replayTestCaseExact, 67, true},
		{"one short", 66, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, 1, 1)
			require.NoError(t, err)
			out, err := (blindContactArithmetic{t.Context(), budget}).add(&x, &y)
			if tc.pass {
				require.NoError(t, err)
				require.Equal(t, int64(12), contactBig(out).Int64())
			} else {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := (blindContactArithmetic{ctx, proofbound.NewReplayTrialBudget()}).mul(&x, &y)
	require.ErrorIs(t, err, context.Canceled)
}

func TestBlindContactVectorArithmetic(t *testing.T) {
	var x, y blindContactVector
	x[0].limbs[0], x[1].limbs[0], x[2].limbs[0] = 2, 3, 4
	y[0].limbs[0], y[1].limbs[0], y[2].limbs[0] = 5, 6, 7
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	difference, err := a.difference(&x, &y)
	require.NoError(t, err)
	for i := range difference {
		require.Equal(t, int64(-3), contactBig(difference[i]).Int64())
	}
	cross, err := a.cross(&x, &y)
	require.NoError(t, err)
	for i, want := range []int64{-3, 6, -3} {
		require.Equal(t, want, contactBig(cross[i]).Int64())
	}
	dot, err := a.dot(&x, &y)
	require.NoError(t, err)
	require.Equal(t, int64(56), contactBig(dot).Int64())
}

func TestBlindContactSignedVectorArithmetic(t *testing.T) {
	var x, y blindContactVector
	x[0].limbs[0], x[1].limbs[0], x[2].limbs[0] = 2, 3, 4
	x[1].negative = true
	y[0].limbs[0], y[1].limbs[0], y[2].limbs[0] = 5, 6, 7
	y[0].negative, y[2].negative = true, true
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	difference, err := a.difference(&x, &y)
	require.NoError(t, err)
	for i, want := range []int64{7, -9, 11} {
		require.Equal(t, want, contactBig(difference[i]).Int64())
	}
	cross, err := a.cross(&x, &y)
	require.NoError(t, err)
	for i, want := range []int64{-3, -6, -3} {
		require.Equal(t, want, contactBig(cross[i]).Int64())
	}
	dot, err := a.dot(&x, &y)
	require.NoError(t, err)
	require.Equal(t, int64(-56), contactBig(dot).Int64())
}

func TestBlindContactVectorLateWorkLimit(t *testing.T) {
	var x, y blindContactVector
	for i := range x {
		x[i].limbs[0], y[i].limbs[0] = uint64(i+2), uint64(i+5)
	}
	for _, tc := range []struct {
		name string
		run  func(blindContactArithmetic) error
	}{
		{"difference", func(a blindContactArithmetic) error { _, err := a.difference(&x, &y); return err }},
		{"cross", func(a blindContactArithmetic) error { _, err := a.cross(&x, &y); return err }},
		{"dot", func(a blindContactArithmetic) error { _, err := a.dot(&x, &y); return err }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			full := proofbound.NewReplayTrialBudget()
			require.NoError(t, tc.run(blindContactArithmetic{t.Context(), full}))
			work, records := full.Usage()
			require.Zero(t, records)
			require.Positive(t, work)
			short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
			require.NoError(t, err)
			require.ErrorIs(t, tc.run(blindContactArithmetic{t.Context(), short}), decaderr.ErrUnsupported)
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			require.ErrorIs(t, tc.run(blindContactArithmetic{ctx, proofbound.NewReplayTrialBudget()}), context.Canceled)
		})
	}
}

func TestBlindContactCrossingEndpoint(t *testing.T) {
	var p, q blindContactVector
	p[0].limbs[0], q[0].limbs[0] = 1, 7
	var dp, dq blindContactInt
	dp.limbs[0], dp.negative = 2, true
	dq.limbs[0] = 1
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	var first, second [4]blindContactInt
	var products [2]blindContactInt
	var zero blindContactInt
	err := a.crossingContactEndpoint(&p, &q, &dp, &dq, &first, &products, &zero)
	require.NoError(t, err)
	err = a.crossingContactEndpoint(&q, &p, &dq, &dp, &second, &products, &zero)
	require.NoError(t, err)
	require.Equal(t, int64(15), contactBig(first[0]).Int64())
	require.Equal(t, int64(3), contactBig(first[3]).Int64())
	order, err := a.compareContactFractions(&first[0], &first[3], &second[0], &second[3])
	require.NoError(t, err)
	require.Zero(t, order)
	dq = blindContactInt{}
	err = a.crossingContactEndpoint(&p, &q, &dp, &dq, &first, &products, &zero)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestBlindContactMultiplyCarry(t *testing.T) {
	var x, y blindContactInt
	for i := range 16 {
		x.limbs[i], y.limbs[i] = ^uint64(0), ^uint64(0)
	}
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	got, err := a.mul(&x, &y)
	require.NoError(t, err)
	require.Zero(t, contactBig(got).Cmp(new(big.Int).Mul(contactBig(x), contactBig(y))))
}

func TestBlindContactClipTriangle(t *testing.T) {
	var p [3]blindContactVector
	p[0][0].limbs[0], p[1][0].limbs[0], p[2][0].limbs[0] = 1, 4, 7
	var d [3]blindContactInt
	d[0].limbs[0], d[0].negative = 1, true
	d[1].limbs[0], d[2].limbs[0] = 1, 2
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	var endpoints [2][4]blindContactInt
	var products [2]blindContactInt
	var zero blindContactInt
	count, err := a.clipContactTriangle(&p, &d, &endpoints, &products, &zero)
	require.NoError(t, err)
	require.Equal(t, 2, count)
	var five, two blindContactInt
	five.limbs[0], two.limbs[0] = 5, 2
	order, err := a.compareContactFractions(&endpoints[0][0], &endpoints[0][3], &five, &two)
	require.NoError(t, err)
	require.Zero(t, order)
	var direction blindContactVector
	direction[0].limbs[0] = 1
	projection, err := a.projectContactEndpoint(&endpoints[0], &direction)
	require.NoError(t, err)
	order, err = a.compareContactFractions(&projection, &endpoints[0][3], &five, &two)
	require.NoError(t, err)
	require.Zero(t, order)
	d[0] = blindContactInt{}
	count, err = a.clipContactTriangle(&p, &d, &endpoints, &products, &zero)
	require.NoError(t, err)
	require.Equal(t, 1, count)
	d[1], d[2] = blindContactInt{}, blindContactInt{}
	_, err = a.clipContactTriangle(&p, &d, &endpoints, &products, &zero)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestBlindContactNoncoplanarIntervals(t *testing.T) {
	var a, b [2][4]blindContactInt
	for i := range 2 {
		a[i][3].limbs[0], b[i][3].limbs[0] = 1, 1
	}
	a[0][0].limbs[0], a[1][0].limbs[0] = 1, 3
	b[0][0].limbs[0], b[1][0].limbs[0] = 4, 5
	var direction blindContactVector
	direction[0].limbs[0] = 1
	arithmetic := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	var scratch blindContactScratch
	allowed, err := arithmetic.noncoplanarContactAllowed(&a, &b, 2, 2, 0, &direction, &scratch)
	require.NoError(t, err)
	require.True(t, allowed)
	b[0][0].limbs[0] = 3
	allowed, err = arithmetic.noncoplanarContactAllowed(&a, &b, 2, 2, 0, &direction, &scratch)
	require.NoError(t, err)
	require.False(t, allowed)
	allowed, err = arithmetic.noncoplanarContactAllowed(&a, &b, 2, 2, 1, &direction, &scratch)
	require.NoError(t, err)
	require.True(t, allowed)
	b[0][0].limbs[0] = 2
	allowed, err = arithmetic.noncoplanarContactAllowed(&a, &b, 2, 2, 1, &direction, &scratch)
	require.NoError(t, err)
	require.False(t, allowed)
}
