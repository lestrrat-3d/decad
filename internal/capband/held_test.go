package capband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/stretchr/testify/require"
)

const heldRefPrec = 256

const heldRefPi = `3.14159265358979323846264338327950288419716939937510582097494459230781640628620899`

func heldBig(x float64) *big.Float { return new(big.Float).SetPrec(heldRefPrec).SetFloat64(x) }

// heldRefAtan is atan(z) for |z| <= 1 in 256-bit arithmetic: three argument
// halvings atan(z) = 2·atan(z/(1+√(1+z²))) bring |z| under 0.13, where the
// Taylor series converges past the working precision in 80 terms.
func heldRefAtan(z *big.Float) *big.Float {
	one := heldBig(1)
	x := new(big.Float).SetPrec(heldRefPrec).Set(z)
	for range 3 {
		root := new(big.Float).SetPrec(heldRefPrec).Sqrt(new(big.Float).SetPrec(heldRefPrec).Add(one, new(big.Float).SetPrec(heldRefPrec).Mul(x, x)))
		x.Quo(x, new(big.Float).SetPrec(heldRefPrec).Add(one, root))
	}
	sum := heldBig(0)
	term := new(big.Float).SetPrec(heldRefPrec).Set(x)
	x2 := new(big.Float).SetPrec(heldRefPrec).Mul(x, x)
	for k := range 80 {
		piece := new(big.Float).SetPrec(heldRefPrec).Quo(term, heldBig(float64(2*k+1)))
		if k%2 == 0 {
			sum.Add(sum, piece)
		} else {
			sum.Sub(sum, piece)
		}
		term.Mul(term, x2)
	}
	return sum.Mul(sum, heldBig(8))
}

// heldRefAtan2 is atan2(y, x) in (−π, π] for exact float64 differences.
func heldRefAtan2(y, x *big.Float) *big.Float {
	pi, _ := new(big.Float).SetPrec(heldRefPrec).SetString(heldRefPi)
	half := new(big.Float).SetPrec(heldRefPrec).Quo(pi, heldBig(2))
	ax, ay := new(big.Float).SetPrec(heldRefPrec).Abs(x), new(big.Float).SetPrec(heldRefPrec).Abs(y)
	var base *big.Float
	if ay.Cmp(ax) <= 0 {
		base = heldRefAtan(new(big.Float).SetPrec(heldRefPrec).Quo(ay, ax))
	} else {
		base = new(big.Float).SetPrec(heldRefPrec).Sub(half, heldRefAtan(new(big.Float).SetPrec(heldRefPrec).Quo(ax, ay)))
	}
	if x.Sign() < 0 {
		base = new(big.Float).SetPrec(heldRefPrec).Sub(pi, base)
	}
	if y.Sign() < 0 {
		base.Neg(base)
	}
	return base
}

func heldRefDelta(a, b float64) *big.Float {
	return new(big.Float).SetPrec(heldRefPrec).Sub(heldBig(a), heldBig(b))
}

// TestAngleAllowCoversTheExactAngle checks AngleAllow against the exact angle
// of the point it names, computed in 256-bit arithmetic: centres from the
// sketch origin out to 2^40 mm, held angles off their principal branch by a
// whole turn, and a point displaced by its stated reach.
//
// Shown to fail: with the Atan2Interval enclosure replaced by the point held
// value, every row reads zero against a nonzero gap; with the reach widening
// dropped, the displaced rows fall short.
func TestAngleAllowCoversTheExactAngle(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		cU, cV float64
		du, dv float64
		turns  float64
		reach  float64
	}{
		{name: `near the origin`, cU: 3, cV: -2, du: 1, dv: 2},
		{name: `a third quadrant point`, cU: 0.1, cV: 0.7, du: -3.25, dv: -0.5},
		{name: `a far centre`, cU: 1 << 40, cV: -(1 << 38), du: -4, dv: 7},
		{name: `unwrapped a turn up`, cU: 5, cV: 5, du: -1, dv: 1e-3, turns: 1},
		{name: `unwrapped a turn down`, cU: -9, cV: 2, du: 2, dv: -3, turns: -1},
		{name: `a displaced point`, cU: 2, cV: 1, du: 3, dv: 4, reach: 1e-3},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			p := capband.Point{U: tc.cU + tc.du, V: tc.cV + tc.dv}
			held := math.Atan2(p.V-tc.cV, p.U-tc.cU) + tc.turns*2*math.Pi
			allow, ok := capband.AngleAllow(tc.cU, tc.cV, p, tc.reach, held)
			require.True(t, ok)
			// The points the allowance must answer for: p itself, and where a
			// reach is stated, p moved that far across its own radius.
			targets := [][2]*big.Float{{heldRefDelta(p.U, tc.cU), heldRefDelta(p.V, tc.cV)}}
			if tc.reach > 0 {
				ru, rv := heldRefDelta(p.U, tc.cU), heldRefDelta(p.V, tc.cV)
				norm := math.Hypot(p.U-tc.cU, p.V-tc.cV)
				for _, s := range []float64{-1, 1} {
					nu := new(big.Float).SetPrec(heldRefPrec).Add(ru, heldBig(-s*tc.reach*(p.V-tc.cV)/norm*0.999))
					nv := new(big.Float).SetPrec(heldRefPrec).Add(rv, heldBig(s*tc.reach*(p.U-tc.cU)/norm*0.999))
					targets = append(targets, [2]*big.Float{nu, nv})
				}
			}
			pi, _ := new(big.Float).SetPrec(heldRefPrec).SetString(heldRefPi)
			turn := new(big.Float).SetPrec(heldRefPrec).Mul(pi, heldBig(2*tc.turns))
			worst := 0.0
			for _, target := range targets {
				exact := new(big.Float).SetPrec(heldRefPrec).Add(heldRefAtan2(target[1], target[0]), turn)
				gap, _ := new(big.Float).SetPrec(heldRefPrec).Sub(heldBig(held), exact).Float64()
				worst = math.Max(worst, math.Abs(gap))
			}
			require.Positive(t, worst, `the held angle is not the exact one`)
			require.GreaterOrEqual(t, allow, worst, `the allowance covers the exact angle`)
		})
	}
}

// TestRadiusAllowCoversTheExactRadius checks RadiusAllow against each point's
// exact distance from the centre, in 256-bit arithmetic: reach must cover the
// gap from the held radius to every point's, and spread the gap between any
// two points'.
//
// Shown to fail: with the square roots read through math.Hypot rather than
// their outward-rounded brackets, the irrational radius row reads a zero
// reach and the two-feet row a spread short of the exact gap.
func TestRadiusAllowCoversTheExactRadius(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		cU, cV float64
		pts    []capband.Point
	}{
		{name: `an irrational radius`, cU: 0, cV: 0, pts: []capband.Point{{U: 1, V: 2}, {U: -2, V: 1}}},
		{name: `two feet apart`, cU: 1.5, cV: -0.25, pts: []capband.Point{{U: 4.75, V: 3.1}, {U: -1.3, V: 3.7}}},
		{name: `a far centre`, cU: 1 << 30, cV: 1 << 29, pts: []capband.Point{{U: (1 << 30) + 3, V: (1 << 29) - 7}, {U: (1 << 30) - 5, V: (1 << 29) + 5.5}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			held := math.Hypot(tc.pts[0].U-tc.cU, tc.pts[0].V-tc.cV)
			reach, spread, ok := capband.RadiusAllow(tc.cU, tc.cV, held, tc.pts...)
			require.True(t, ok)
			radii := make([]*big.Float, len(tc.pts))
			for i, p := range tc.pts {
				du, dv := heldRefDelta(p.U, tc.cU), heldRefDelta(p.V, tc.cV)
				sq := new(big.Float).SetPrec(heldRefPrec).Add(new(big.Float).SetPrec(heldRefPrec).Mul(du, du), new(big.Float).SetPrec(heldRefPrec).Mul(dv, dv))
				radii[i] = new(big.Float).SetPrec(heldRefPrec).Sqrt(sq)
				gap, _ := new(big.Float).SetPrec(heldRefPrec).Sub(heldBig(held), radii[i]).Float64()
				require.GreaterOrEqual(t, reach, math.Abs(gap), `reach covers point %d`, i)
			}
			for i := range radii {
				for j := range radii {
					gap, _ := new(big.Float).SetPrec(heldRefPrec).Sub(radii[i], radii[j]).Float64()
					require.GreaterOrEqual(t, spread, math.Abs(gap), `spread covers points %d and %d`, i, j)
				}
			}
		})
	}
}
