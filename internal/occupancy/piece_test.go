package occupancy

import (
	"fmt"
	"math"
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

// prefilterCase is one half-plane ux·x + uy·y ≤ c and an exact corner P.
type prefilterCase struct {
	ux, uy, c float64
	x, y      *big.Rat
}

// exactLess reports whether the exact ux·P.x + uy·P.y is strictly below c.
func (pc prefilterCase) exactLess() bool {
	s := new(big.Rat).Mul(ratOf(pc.ux), pc.x)
	s.Add(s, new(big.Rat).Mul(ratOf(pc.uy), pc.y))
	return s.Cmp(ratOf(pc.c)) < 0
}

func (pc prefilterCase) prefilter() bool {
	px, _ := pc.x.Float64()
	py, _ := pc.y.Float64()
	return cornerClearlyInside(pc.ux, pc.uy, pc.c, px, py)
}

// scaled returns a random float whose magnitude spans 1e-300 to 1e300.
func scaled(rng *rand.Rand) float64 {
	v := math.Pow(10, rng.Float64()*600-300) * (1 + rng.Float64())
	if rng.Intn(4) == 0 { // the subnormal range, where products lose digits
		v = math.Pow(10, -324+rng.Float64()*24)
	}
	if rng.Intn(2) == 0 {
		v = -v
	}
	return v
}

// ratFloor returns the largest float not above the exact value s.
func ratFloor(s *big.Rat) float64 {
	f, _ := s.Float64()
	if ratOf(f).Cmp(s) > 0 {
		f = math.Nextafter(f, math.Inf(-1))
	}
	return f
}

// genCase builds a case whose boundary sits at, just inside or just outside
// the exact corner, with the two products cancelling in half of them.
func genCase(rng *rand.Rand) prefilterCase {
	th := rng.Float64() * 2 * math.Pi
	ux, uy := math.Cos(th), math.Sin(th)
	mag := scaled(rng)
	x := new(big.Rat).SetFloat64(mag)
	y := new(big.Rat)
	switch rng.Intn(3) {
	case 0: // y cancels ux·x, up to the rounding of a float
		y.SetFloat64(-ux * mag / uy)
	case 1:
		y.SetFloat64(scaled(rng))
	default:
		y.SetFloat64(mag * rng.Float64())
	}
	// A corner is a rational: nudge it off the float grid by a few parts in 2^k.
	nudge := new(big.Rat).SetFrac64(int64(rng.Intn(1<<20)+1), 1<<(40+rng.Intn(20)))
	x.Add(x, new(big.Rat).Mul(x, nudge))
	if rng.Intn(2) == 0 {
		y.Add(y, new(big.Rat).Mul(y, nudge))
	}
	s := new(big.Rat).Mul(ratOf(ux), x)
	s.Add(s, new(big.Rat).Mul(ratOf(uy), y))
	c := ratFloor(s)
	for range rng.Intn(5) { // 0..4 ulps inside or outside
		if rng.Intn(2) == 0 {
			c = math.Nextafter(c, math.Inf(1))
		} else {
			c = math.Nextafter(c, math.Inf(-1))
		}
	}
	return prefilterCase{ux, uy, c, x, y}
}

func TestCornerClearlyInsideNeverSkipsAFailingPair(t *testing.T) {
	rng := rand.New(rand.NewSource(20261103))
	skipped := 0
	const n = 120000
	for range n {
		pc := genCase(rng)
		if pc.prefilter() {
			skipped++
			require.True(t, pc.exactLess(),
				"skipped a pair outside the half-plane: ux=%v uy=%v c=%v x=%v y=%v",
				pc.ux, pc.uy, pc.c, pc.x.FloatString(30), pc.y.FloatString(30))
		}
	}
	// The generator reaches both outcomes, so the property is not vacuous.
	require.Positive(t, skipped)
	require.Less(t, skipped, n)
}

func TestCornerClearlyInsideNonFinite(t *testing.T) {
	nan, inf := math.NaN(), math.Inf(1)
	for _, tc := range []struct{ ux, uy, c, px, py float64 }{
		{1, 0, 1, inf, 0},
		{1, 0, 1, -inf, 0},
		{0, 1, 1, inf, 0},
		{0, 1, 1, 0, -inf},
		{0.6, 0.8, 1, nan, 0},
		{0.6, 0.8, 1, 0, nan},
		{0.6, 0.8, 1, inf, -inf},
		{0.6, 0.8, 1, -inf, inf},
		{1, 1, 1, math.MaxFloat64, -math.MaxFloat64},
		{1, 1, 1, -math.MaxFloat64, -math.MaxFloat64},
		{1, 1, 1, math.MaxFloat64, math.MaxFloat64},
	} {
		require.False(t, cornerClearlyInside(tc.ux, tc.uy, tc.c, tc.px, tc.py), "%+v", tc)
	}
	// Plainly inside, far from the boundary, is cleared.
	require.True(t, cornerClearlyInside(1, 0, 1, 0.5, 0))
	// On the boundary is not.
	require.False(t, cornerClearlyInside(1, 0, 1, 1, 0))
}

// circlePrismRefusal is CirclePrism's corner check with every pair compared
// exactly: the same lines, corners, comparison order and error values.
func circlePrismRefusal(r float64, sides int) error {
	type line struct{ ux, uy, c *big.Rat }
	r2 := new(big.Rat).Mul(ratOf(r), ratOf(r))
	lines := make([]line, sides)
	for k := range lines {
		th := 2 * math.Pi * float64(k) / float64(sides)
		ux, uy := math.Cos(th), math.Sin(th)
		c := r * math.Hypot(ux, uy)
		rx, ry := ratOf(ux), ratOf(uy)
		need := new(big.Rat).Mul(rx, rx)
		need.Add(need, new(big.Rat).Mul(ry, ry))
		need.Mul(need, r2)
		for steps := 0; ; steps++ {
			rc := ratOf(c)
			if new(big.Rat).Mul(rc, rc).Cmp(need) >= 0 {
				break
			}
			if steps > 8 {
				return fmt.Errorf("%w: offset for side %d did not converge", decaderr.ErrUnsupported, k)
			}
			c = math.Nextafter(c, math.Inf(1))
		}
		lines[k] = line{rx, ry, ratOf(c)}
	}
	corners := make([][2]*big.Rat, sides)
	for k := range corners {
		a, b := lines[k], lines[(k+1)%sides]
		det := new(big.Rat).Sub(new(big.Rat).Mul(a.ux, b.uy), new(big.Rat).Mul(a.uy, b.ux))
		if det.Sign() <= 0 {
			return fmt.Errorf("%w: sides %d and %d do not turn left", decaderr.ErrUnsupported, k, (k+1)%sides)
		}
		x := new(big.Rat).Sub(new(big.Rat).Mul(a.c, b.uy), new(big.Rat).Mul(a.uy, b.c))
		y := new(big.Rat).Sub(new(big.Rat).Mul(a.ux, b.c), new(big.Rat).Mul(a.c, b.ux))
		corners[k] = [2]*big.Rat{x.Quo(x, det), y.Quo(y, det)}
	}
	for k, p := range corners {
		for j, l := range lines {
			s := new(big.Rat).Mul(l.ux, p[0])
			s.Add(s, new(big.Rat).Mul(l.uy, p[1]))
			if s.Cmp(l.c) > 0 {
				return fmt.Errorf("%w: corner %d violates side %d", decaderr.ErrUnsupported, k, j)
			}
		}
	}
	return nil
}

func TestCirclePrismMatchesExactOnlyCheck(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	for range 40 {
		sides := 3 + rng.Intn(62)
		r := math.Abs(scaled(rng))
		if r > 1e300 || r < 1e-300 {
			continue
		}
		want := circlePrismRefusal(r, sides)
		p, err := CirclePrism(nil, r, 0, 1, sides)
		if want == nil {
			require.NoError(t, err, "r=%v sides=%d", r, sides)
			require.NotNil(t, p)
		} else {
			require.EqualError(t, err, want.Error(), "r=%v sides=%d", r, sides)
		}
	}
}
