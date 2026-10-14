package revolvemesh_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/stretchr/testify/require"
)

func sameRat(x, y *big.Rat) bool {
	return x.Num().Cmp(y.Num()) == 0 && x.Denom().Cmp(y.Denom()) == 0
}

// angularHomotopyBulgesRaw is AngularHomotopyBulges written with big.Rat's own
// Mul, Add and Quo.
func angularHomotopyBulgesRaw(d proofbound.RatInterval, n int64) (*big.Rat, *big.Rat) {
	dh := new(big.Rat).Set(d.Hi)
	sq := new(big.Rat).Mul(dh, dh)
	sinB := proofbound.RatMin(big.NewRat(1, 1), dh)
	versB := proofbound.RatMin(big.NewRat(2, 1), new(big.Rat).Mul(sq, big.NewRat(1, 2)))
	p := new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), dh), new(big.Rat).Add(
		new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), dh), sinB),
		sq,
	))
	q := new(big.Rat).Mul(new(big.Rat).Mul(sq, sinB), new(big.Rat).Add(versB, big.NewRat(1, 1)))
	denom := new(big.Rat).SetInt64(8 * n * n)
	return new(big.Rat).Quo(p, denom), new(big.Rat).Quo(q, denom)
}

// angularHomotopyFactorRaw is the reading revolveAngularHomotopyFactorUncached
// takes, written with big.Rat's own Cmp, Sub, Add and Mul.
func angularHomotopyFactorRaw(step proofbound.RatInterval) (*big.Rat, error) {
	d := revolvemesh.IntervalAbsSpan(step)
	if d.Lo.Sign() < 0 || d.Hi.Cmp(d.Lo) < 0 {
		return nil, revolvemesh.ErrRevolveAngularHomotopy
	}
	one := proofbound.PointInterval(big.NewRat(1, 1))
	sinD, cosD, ok := proofbound.RadSinCosSpan(d)
	if !ok {
		return nil, revolvemesh.ErrRevolveAngularHomotopy
	}
	versD := proofbound.IntervalSub(one, cosD)
	n := int64(revolvemesh.RevolveAngularIntegralSteps)
	pAt := make([]*big.Rat, n+1)
	qAt := make([]*big.Rat, n+1)
	sinAt := make([]proofbound.RatInterval, n+1)
	cosAt := make([]proofbound.RatInterval, n+1)
	for i := int64(0); i <= n; i++ {
		sin, cos, ok := proofbound.RadSinCosSpan(proofbound.IntervalScale(d, big.NewRat(i, n)))
		if !ok {
			return nil, revolvemesh.ErrRevolveAngularHomotopy
		}
		sinAt[i], cosAt[i] = sin, cos
	}
	for i := int64(0); i <= n; i++ {
		u := big.NewRat(i, n)
		co := new(big.Rat).Sub(big.NewRat(1, 1), u)
		sinA, cosA, cosB := sinAt[i], cosAt[i], cosAt[n-i]
		versA := proofbound.IntervalSub(one, cosA)
		p := proofbound.IntervalMul(d, proofbound.IntervalAdd(
			proofbound.IntervalScale(versA, co),
			proofbound.IntervalScale(proofbound.IntervalSub(one, cosB), u),
		))
		q := proofbound.IntervalSub(proofbound.IntervalMul(sinA, versD), proofbound.IntervalMul(sinD, versA))
		pAt[i], qAt[i] = proofbound.IntervalAbsUpper(p), proofbound.IntervalAbsUpper(q)
	}
	bulgeP, bulgeQ := angularHomotopyBulgesRaw(d, n)
	total := new(big.Rat)
	width := big.NewRat(1, n)
	for i := range n {
		piece := new(big.Rat).Add(proofbound.RatMax(pAt[i], pAt[i+1]), bulgeP)
		piece.Add(piece, new(big.Rat).Add(proofbound.RatMax(qAt[i], qAt[i+1]), bulgeQ))
		total.Add(total, new(big.Rat).Mul(piece, width))
	}
	return total.Mul(total, big.NewRat(1, 2)), nil
}

// TestAngularHomotopyMatchesRawRat compares the homotopy factor, its bulges
// and the swept cell volume with the same readings taken through big.Rat's own
// operations, over steps that are exact fractions, float lifts, wide intervals,
// straddling zero, negative and refused.
func TestAngularHomotopyMatchesRawRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(61, 67))
	steps := []proofbound.RatInterval{
		proofbound.PointInterval(new(big.Rat)),
		proofbound.PointInterval(big.NewRat(1, 8)),
		proofbound.PointInterval(proofarith.FloatRat(0.3)),
		proofbound.Interval(big.NewRat(-1, 9), big.NewRat(1, 7)),
		proofbound.Interval(big.NewRat(-1, 4), big.NewRat(-1, 11)),
		proofbound.Interval(big.NewRat(1, 10), big.NewRat(1, 9)),
		proofbound.Interval(big.NewRat(2, 1), big.NewRat(5, 2)),
	}
	for range 5 {
		lo := big.NewRat(rng.Int64N(1000)-500, 997)
		steps = append(steps, proofbound.Interval(lo, new(big.Rat).Add(lo, big.NewRat(rng.Int64N(50), 100003))))
	}
	for _, step := range steps {
		got, gotErr := revolvemesh.RevolveAngularHomotopyFactorUncached(step)
		want, wantErr := angularHomotopyFactorRaw(step)
		require.Equal(t, wantErr, gotErr)
		if wantErr != nil {
			continue
		}
		require.True(t, sameRat(got, want), "step [%v, %v]: got %v, want %v", step.Lo, step.Hi, got, want)

		d := revolvemesh.IntervalAbsSpan(step)
		for _, n := range []int64{1, 6, 24} {
			gp, gq := revolvemesh.AngularHomotopyBulges(d, n)
			wp, wq := angularHomotopyBulgesRaw(d, n)
			require.True(t, sameRat(gp, wp) && sameRat(gq, wq), "bulges of step [%v, %v] at %d", step.Lo, step.Hi, n)
		}
	}
	require.ErrorIs(t, func() error {
		_, err := revolvemesh.RevolveAngularHomotopyFactorUncached(proofbound.Interval(big.NewRat(1, 1), new(big.Rat)))
		return err
	}(), revolvemesh.ErrRevolveAngularHomotopy)
}

// TestRevolveCellSweptVolumeMatchesRawRat compares the swept volume of one
// cell with the product the raw big.Rat operations give.
func TestRevolveCellSweptVolumeMatchesRawRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(71, 73))
	interval := func() proofbound.RatInterval {
		lo := big.NewRat(rng.Int64N(400)-200, 37)
		return proofbound.Interval(lo, new(big.Rat).Add(lo, big.NewRat(rng.Int64N(60), 53)))
	}
	for range 200 {
		lo := revolvemesh.RevMeridian{RhoIv: interval(), ZIv: interval()}
		hi := revolvemesh.RevMeridian{RhoIv: interval(), ZIv: interval()}
		angular := big.NewRat(rng.Int64N(500)+1, 211)
		got := revolvemesh.RevolveCellSweptVolume(lo, hi, angular)

		third := big.NewRat(1, 3)
		quad := proofbound.IntervalScale(proofbound.IntervalAdd(
			proofbound.IntervalAdd(proofbound.IntervalSquare(lo.RhoIv), proofbound.IntervalSquare(hi.RhoIv)),
			proofbound.IntervalMul(lo.RhoIv, hi.RhoIv),
		), third)
		axial := proofbound.IntervalAbsUpper(proofbound.IntervalSub(hi.ZIv, lo.ZIv))
		want := new(big.Rat).Mul(new(big.Rat).Mul(axial, proofbound.IntervalAbsUpper(quad)), angular)
		require.True(t, sameRat(got, want), "got %v, want %v", got, want)
	}
}
