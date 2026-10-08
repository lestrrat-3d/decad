package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// TestTransformVolumeMomentsIsTheImage carries a box's moments through
// f = (1 + ε)·R, R an exact rational rotation and ε = 2⁻²⁰, and requires the
// image's own moments exactly: V·|det f|, |det f|·f·P and |det f|·f·Q·fᵀ,
// computed here from the box directly. A held sweep frame is orthonormal to
// within an ulp, too close for a float reading to see the scaling, so the
// scaled matrix makes each factor visible.
//
// Leg shown to fail (deleted in massmoment.Transform, this fixture watched go
// red, then restored): the |det f| factor, whose absence the volume missed.
func TestTransformVolumeMomentsIsTheImage(t *testing.T) {
	r := [3][3]*big.Rat{
		{big.NewRat(3, 5), big.NewRat(-4, 5), new(big.Rat)},
		{big.NewRat(4, 5), big.NewRat(3, 5), new(big.Rat)},
		{new(big.Rat), new(big.Rat), big.NewRat(1, 1)},
	}
	scale := new(big.Rat).Add(big.NewRat(1, 1), big.NewRat(1, 1<<20))
	var f [3][3]*big.Rat
	for i := range f {
		for j := range f[i] {
			f[i][j] = new(big.Rat).Mul(scale, r[i][j])
		}
	}
	det := new(big.Rat).Mul(scale, new(big.Rat).Mul(scale, scale))

	// The box [-1, 1] × [-2, 2] × [-3, 3] moved to (1, 2, 3), so every first
	// and second moment is nonzero.
	box := massmoment.Moments{Volume: proofbound.PointInterval(big.NewRat(48, 1))}
	for i := range box.First {
		box.First[i] = proofbound.PointInterval(new(big.Rat))
		for j := range box.Second[i] {
			box.Second[i][j] = proofbound.PointInterval(new(big.Rat))
		}
	}
	for i, half := range []int64{1, 2, 3} {
		box.Second[i][i] = proofbound.PointInterval(big.NewRat(48*half*half, 3))
	}
	local := massmoment.Shift(box, [3]*big.Rat{big.NewRat(1, 1), big.NewRat(2, 1), big.NewRat(3, 1)})
	exact := func(iv proofbound.RatInterval) *big.Rat {
		require.Zero(t, iv.Lo.Cmp(iv.Hi))
		return iv.Lo
	}

	got := massmoment.Transform(local, f)
	requireIntervalContains(t, got.Volume, new(big.Rat).Mul(det, exact(local.Volume)))
	for i := range 3 {
		want := new(big.Rat)
		for k := range 3 {
			want.Add(want, new(big.Rat).Mul(f[i][k], exact(local.First[k])))
		}
		requireIntervalContains(t, got.First[i], want.Mul(want, det))
		for j := range 3 {
			want := new(big.Rat)
			for k := range 3 {
				for l := range 3 {
					term := new(big.Rat).Mul(f[i][k], f[j][l])
					want.Add(want, term.Mul(term, exact(local.Second[k][l])))
				}
			}
			requireIntervalContains(t, got.Second[i][j], want.Mul(want, det))
		}
	}
}

func requireIntervalContains(t *testing.T, iv proofbound.RatInterval, value *big.Rat) {
	t.Helper()
	require.True(t, iv.Lo.Cmp(value) <= 0 && value.Cmp(iv.Hi) <= 0,
		"[%s, %s] misses %s", iv.Lo.FloatString(20), iv.Hi.FloatString(20), value.FloatString(20))
}
