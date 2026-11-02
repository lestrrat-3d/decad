package tessellation

import (
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestBlindContactArenaArithmetic(t *testing.T) {
	rng := rand.New(rand.NewSource(4523))
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	for range 128 {
		var x, y, out blindContactInt
		for i := range 16 {
			x.limbs[i], y.limbs[i] = rng.Uint64(), rng.Uint64()
		}
		x.negative, y.negative = rng.Intn(2) == 0, rng.Intn(2) == 0
		require.NoError(t, a.addContactInto(&out, &x, &y, false))
		require.Zero(t, contactBig(out).Cmp(new(big.Int).Add(contactBig(x), contactBig(y))))
		requireBlindContactWidth(t, &out)
		require.NoError(t, a.addContactInto(&out, &x, &y, true))
		require.Zero(t, contactBig(out).Cmp(new(big.Int).Sub(contactBig(x), contactBig(y))))
		requireBlindContactWidth(t, &out)
		require.NoError(t, a.multiplyContactInto(&out, &x, &y))
		require.Zero(t, contactBig(out).Cmp(new(big.Int).Mul(contactBig(x), contactBig(y))))
		requireBlindContactWidth(t, &out)
	}
}

func requireBlindContactWidth(t *testing.T, x *blindContactInt) {
	t.Helper()
	width := len(x.limbs)
	for width > 0 && x.limbs[width-1] == 0 {
		width--
	}
	require.True(t, x.known)
	require.Equal(t, width, int(x.used))
}

func TestBlindContactArenaArithmeticLimits(t *testing.T) {
	var x, y, out blindContactInt
	x.limbs[0], y.limbs[0] = 5, 7
	for _, tc := range []struct {
		name string
		run  func(blindContactArithmetic) error
	}{
		{"sum", func(a blindContactArithmetic) error { return a.addContactInto(&out, &x, &y, false) }},
		{"difference", func(a blindContactArithmetic) error { return a.addContactInto(&out, &x, &y, true) }},
		{"product", func(a blindContactArithmetic) error { return a.multiplyContactInto(&out, &x, &y) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			out = blindContactInt{}
			budget := proofbound.NewReplayTrialBudget()
			require.NoError(t, tc.run(blindContactArithmetic{t.Context(), budget}))
			work, _ := budget.Usage()
			short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
			require.NoError(t, err)
			out = blindContactInt{}
			require.ErrorIs(t, tc.run(blindContactArithmetic{t.Context(), short}), decaderr.ErrUnsupported)
		})
	}
	var zero blindContactInt
	zero.negative = true
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	require.ErrorIs(t, a.addContactInto(&x, &x, &y, false), decaderr.ErrUnsupported)
	require.ErrorIs(t, a.multiplyContactInto(&y, &x, &y), decaderr.ErrUnsupported)
	require.NoError(t, a.addContactInto(&out, &zero, &zero, true))
	require.False(t, out.negative)
	for i := range x.limbs {
		x.limbs[i] = ^uint64(0)
	}
	y = blindContactInt{}
	y.limbs[0] = 1
	require.ErrorIs(t, a.addContactInto(&out, &x, &y, false), decaderr.ErrUnsupported)
	require.False(t, out.known)
	y.limbs[0] = 2
	require.ErrorIs(t, a.multiplyContactInto(&out, &x, &y), decaderr.ErrUnsupported)
	require.False(t, out.known)
	require.NoError(t, a.addContactInto(&out, &y, &y, false))
	require.Equal(t, int64(4), contactBig(out).Int64())
	requireBlindContactWidth(t, &out)
}

func TestBlindContactCachedWidthChaining(t *testing.T) {
	var x, y, product, next blindContactInt
	require.True(t, blindContactCoordinate(0x1.fffffffffffffp31, &x))
	require.True(t, blindContactCoordinate(-0x1.1p-64, &y))
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	require.NoError(t, a.multiplyContactInto(&product, &x, &y))
	requireBlindContactWidth(t, &product)
	require.Zero(t, contactBig(product).Cmp(new(big.Int).Mul(contactBig(x), contactBig(y))))
	require.NoError(t, a.addContactInto(&next, &product, &x, true))
	requireBlindContactWidth(t, &next)
	require.Zero(t, contactBig(next).Cmp(new(big.Int).Sub(contactBig(product), contactBig(x))))
}

func TestBlindContactArenaVectors(t *testing.T) {
	var x, y, out blindContactVector
	for i := range x {
		x[i].limbs[0], y[i].limbs[0] = uint64(i+2), uint64(i+5)
	}
	x[1].negative, y[2].negative = true, true
	a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	var products [2]blindContactInt
	var dotSlots [3]blindContactInt
	require.ErrorIs(t, a.differenceContactInto(&x, &x, &y), decaderr.ErrUnsupported)
	require.ErrorIs(t, a.crossContactInto(&y, &x, &y, &products), decaderr.ErrUnsupported)
	require.ErrorIs(t, a.dotContactInto(&x[0], &x, &y, &dotSlots), decaderr.ErrUnsupported)
	require.ErrorIs(t, a.dotContactInto(&dotSlots[0], &x, &y, &dotSlots), decaderr.ErrUnsupported)
	require.NoError(t, a.differenceContactInto(&out, &x, &y))
	for i, want := range []int64{-3, -9, 11} {
		require.Equal(t, want, contactBig(out[i]).Int64())
	}
	require.NoError(t, a.crossContactInto(&out, &x, &y, &products))
	for i, want := range []int64{-3, 34, 27} {
		require.Equal(t, want, contactBig(out[i]).Int64())
	}
	var dot blindContactInt
	require.NoError(t, a.dotContactInto(&dot, &x, &y, &dotSlots))
	require.Equal(t, int64(-36), contactBig(dot).Int64())
}
