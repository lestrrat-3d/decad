package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/stretchr/testify/require"
)

// TestRotateVolumeMomentsChargesDefect carries a box's moments through
// f = (1 + ε)·R, R an exact rational rotation and ε = 2⁻²⁰. The rotation
// nearest f is R itself, so the true rotated moments are R·P and R·Q·Rᵀ,
// computed here exactly; both must lie in the widened intervals. A held
// sweep frame is orthonormal to within an ulp, too close for a float
// reading to see the widening, so the scaled matrix makes each leg visible.
//
// Legs shown to fail (each deleted in rotateVolumeMoments, this fixture
// watched go red, then restored):
//   - the first-moment widening d·‖P‖₁: R·P escapes f·P;
//   - the second-moment widening 3·d·(2+d)·m: R·Q·Rᵀ escapes f·Q·fᵀ.
func TestRotateVolumeMomentsChargesDefect(t *testing.T) {
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

	// The box [-1, 1] × [-2, 2] × [-3, 3] moved to (1, 2, 3), so every first
	// and second moment is nonzero.
	box := volumeMoments{volume: proofbound.PointInterval(big.NewRat(48, 1))}
	for i := range box.first {
		box.first[i] = proofbound.PointInterval(new(big.Rat))
		for j := range box.second[i] {
			box.second[i][j] = proofbound.PointInterval(new(big.Rat))
		}
	}
	for i, half := range []int64{1, 2, 3} {
		box.second[i][i] = proofbound.PointInterval(big.NewRat(48*half*half, 3))
	}
	local := shiftVolumeMoments(box, [3]*big.Rat{big.NewRat(1, 1), big.NewRat(2, 1), big.NewRat(3, 1)})
	exact := func(iv proofbound.RatInterval) *big.Rat {
		require.Zero(t, iv.Lo.Cmp(iv.Hi))
		return iv.Lo
	}

	got := rotateVolumeMoments(local, f)
	require.Zero(t, got.volume.Lo.Cmp(exact(local.volume)))
	for i := range 3 {
		want := new(big.Rat)
		for k := range 3 {
			want.Add(want, new(big.Rat).Mul(r[i][k], exact(local.first[k])))
		}
		requireIntervalContains(t, got.first[i], want)
		for j := range 3 {
			want := new(big.Rat)
			for k := range 3 {
				for l := range 3 {
					term := new(big.Rat).Mul(r[i][k], r[j][l])
					want.Add(want, term.Mul(term, exact(local.second[k][l])))
				}
			}
			requireIntervalContains(t, got.second[i][j], want)
		}
	}
}

func requireIntervalContains(t *testing.T, iv proofbound.RatInterval, value *big.Rat) {
	t.Helper()
	require.True(t, iv.Lo.Cmp(value) <= 0 && value.Cmp(iv.Hi) <= 0,
		"[%s, %s] misses %s", iv.Lo.FloatString(20), iv.Hi.FloatString(20), value.FloatString(20))
}
