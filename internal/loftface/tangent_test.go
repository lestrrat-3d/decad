package loftface_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/stretchr/testify/require"
)

func tangentSlopes() []*big.Rat {
	// The third slope is a ratio of two non-dyadic floats, so it is not a
	// float itself.
	odd := new(big.Rat).Quo(new(big.Rat).SetFloat64(0.7), new(big.Rat).SetFloat64(3.1))
	return []*big.Rat{rat(1, 1), rat(7, 5), odd}
}

func TestTangentEndPullbackIsExactRuling(t *testing.T) {
	apex := new(big.Rat).SetFloat64(8.3)
	directions := [][2]float64{{1, 0}, {0.6, 0.8}, {-0.6, 0.8}, {-0.6, -0.8}, {0.6, -0.8},
		{0, 1}, {0, -1}, {-1, 0}, {3, 1e-9}, {-3, -1e-9}}
	for _, slope := range tangentSlopes() {
		for _, dir := range directions {
			end, err := loftface.NewTangentEnd(apex, slope, dir[0], dir[1])
			require.NoError(t, err)
			m := end.Map()
			q := loftface.ConePullback(m, apex, slope)
			k2 := new(big.Rat).Mul(slope, slope)
			for i := range 5 {
				require.Zero(t, q[i].Sign(), "slope %s direction %v: coefficient %d", slope, dir, i)
			}
			require.Zero(t, q[5].Cmp(k2), "slope %s direction %v: v^2 coefficient", slope, dir)
			require.False(t, loftface.ConeStrictOutsideForm(q))

			// e is read back from the map: QV = (0, -e_z, e_y).
			ey, ez := m.QV[2], new(big.Rat).Neg(m.QV[1])
			unit := new(big.Rat).Add(new(big.Rat).Mul(ey, ey), new(big.Rat).Mul(ez, ez))
			require.Zero(t, unit.Cmp(rat(1, 1)), "e is not an exact unit vector")
			// The chosen direction is the radial one, up to float rounding.
			n := math.Hypot(dir[0], dir[1])
			gotY, _ := ey.Float64()
			gotZ, _ := ez.Float64()
			require.InDelta(t, dir[0]/n, gotY, 1e-12)
			require.InDelta(t, dir[1]/n, gotZ, 1e-12)

			// x + k*(e_y*y + e_z*z) = apex at sample (u, v), and
			// k^2(y^2+z^2) - (apex-x)^2 = k^2 v^2.
			for _, uv := range [][2]*big.Rat{{rat(0, 1), rat(0, 1)}, {rat(-3, 1), rat(1, 7)},
				{rat(5, 3), rat(-9, 4)}, {new(big.Rat).SetFloat64(-0.125), new(big.Rat).SetFloat64(2.5)}} {
				p := m.At(uv[0], uv[1])
				dot := new(big.Rat).Add(new(big.Rat).Mul(ey, p[1]), new(big.Rat).Mul(ez, p[2]))
				tangent := new(big.Rat).Add(p[0], new(big.Rat).Mul(slope, dot))
				require.Zero(t, tangent.Cmp(apex), "sample %v", uv)
				r2 := new(big.Rat).Add(new(big.Rat).Mul(p[1], p[1]), new(big.Rat).Mul(p[2], p[2]))
				gap := new(big.Rat).Sub(apex, p[0])
				lhs := new(big.Rat).Sub(new(big.Rat).Mul(k2, r2), new(big.Rat).Mul(gap, gap))
				rhs := new(big.Rat).Mul(k2, new(big.Rat).Mul(uv[1], uv[1]))
				require.Zero(t, lhs.Cmp(rhs), "sample %v", uv)
			}
		}
	}
}

func TestTangentEndMapIsPlanarWithBoundedStretch(t *testing.T) {
	apex := rat(8, 1)
	end, err := loftface.NewTangentEnd(apex, rat(3, 4), 0, 1)
	require.NoError(t, err)
	m := end.Map()
	require.Zero(t, m.G0.Cmp(rat(1, 1)))
	require.Zero(t, m.GU.Sign())
	require.Zero(t, m.GV.Sign())
	frob2 := loftface.VDot(m.QU, m.QU)
	frob2.Add(frob2, loftface.VDot(m.QV, m.QV))
	stretch := new(big.Rat).SetFloat64(m.Stretch)
	require.GreaterOrEqual(t, stretch.Cmp(new(big.Rat).Mul(stretch, big.NewRat(0, 1))), 0)
	require.GreaterOrEqual(t, new(big.Rat).Mul(stretch, stretch).Cmp(frob2), 0,
		"Stretch must not be below the Frobenius norm")
	require.Less(t, m.Stretch, 1.5)
}

func TestTangentEndRefuses(t *testing.T) {
	apex, slope := rat(8, 1), rat(1, 1)
	for name, dir := range map[string][2]float64{
		"zero": {0, 0}, "nan": {math.NaN(), 1}, "inf": {1, math.Inf(1)}} {
		_, err := loftface.NewTangentEnd(apex, slope, dir[0], dir[1])
		require.Error(t, err, name)
	}
	_, err := loftface.NewTangentEnd(apex, rat(0, 1), 1, 0)
	require.Error(t, err)
	_, err = loftface.NewTangentEnd(apex, rat(-1, 1), 1, 0)
	require.Error(t, err)
	_, err = loftface.NewTangentEnd(nil, slope, 1, 0)
	require.Error(t, err)
}

func TestTangentEndEqualAndMatch(t *testing.T) {
	apex, slope := rat(8, 1), rat(7, 5)
	end, err := loftface.NewTangentEnd(apex, slope, 0.6, -0.8)
	require.NoError(t, err)
	// The end holds copies of its inputs.
	apex.Add(apex, rat(1, 1))
	require.True(t, end.MatchesCone(rat(8, 1), rat(7, 5)))
	require.False(t, end.MatchesCone(apex, rat(7, 5)))
	require.False(t, end.MatchesCone(rat(8, 1), rat(7, 5+1)))

	same, err := loftface.NewTangentEnd(rat(8, 1), rat(7, 5), 0.6, -0.8)
	require.NoError(t, err)
	require.True(t, end.Equal(same))
	require.True(t, loftface.MapEqual(end.Map(), same.Map()))

	other, err := loftface.NewTangentEnd(rat(8, 1), rat(7, 5), 0.6, -0.79)
	require.NoError(t, err)
	require.False(t, end.Equal(other))
	require.False(t, loftface.MapEqual(end.Map(), other.Map()))

	flipped, err := loftface.NewTangentEnd(rat(8, 1), rat(7, 5), -0.6, -0.8)
	require.NoError(t, err)
	require.False(t, end.Equal(flipped))

	changed := end.Map()
	changed.Q0[2] = rat(1, 1<<40)
	require.False(t, loftface.MapEqual(end.Map(), changed))
	changed = end.Map()
	changed.Stretch = math.Nextafter(changed.Stretch, 0)
	require.False(t, loftface.MapEqual(end.Map(), changed))
}

func TestConeStrictOutsideForm(t *testing.T) {
	zero := func() *big.Rat { return new(big.Rat) }
	good := [6]*big.Rat{zero(), zero(), zero(), rat(1, 1<<30), zero(), rat(2, 1)}
	require.True(t, loftface.ConeStrictOutsideForm(good))
	for i, v := range []*big.Rat{rat(1, 1<<60), rat(1, 1<<60), rat(1, 1<<60), zero(), rat(1, 1<<60), zero()} {
		bad := good
		bad[i] = v
		require.False(t, loftface.ConeStrictOutsideForm(bad), "coefficient %d", i)
	}
	bad := good
	bad[3] = rat(-1, 1<<60)
	require.False(t, loftface.ConeStrictOutsideForm(bad))
}

func TestRoundPoint(t *testing.T) {
	third := rat(1, 3)
	p := loftface.RoundPoint(loftface.RatVec{third, rat(-5, 2), rat(0, 1)})
	want, _ := third.Float64()
	require.Equal(t, want, p.X)
	require.Equal(t, -2.5, p.Y)
	require.Equal(t, 0.0, p.Z)
}

// A radial direction of extreme magnitude only picks the direction. Huge,
// tiny and subnormal inputs must give the requested quadrant and direction,
// with the exact unit-vector and pullback identities intact.
func TestTangentEndScaleSafeDirection(t *testing.T) {
	apex, slope := rat(8, 1), rat(7, 5)
	const sub = 5e-324
	for _, dir := range [][2]float64{
		{1e308, 1e308}, {-1e308, 1e308}, {-1e308, -1e308}, {1e308, -1e308}, {1.7e308, 1e300},
		{1e-308, 1e-308}, {-1e-308, 1e-308}, {1e-308, -1e-308},
		{sub, sub}, {-sub, sub}, {-sub, -sub}, {sub, -sub}, {3 * sub, -4 * sub}, {-4 * sub, 3 * sub},
		{math.MaxFloat64, math.MaxFloat64}, {-math.MaxFloat64, math.MaxFloat64},
	} {
		end, err := loftface.NewTangentEnd(apex, slope, dir[0], dir[1])
		require.NoError(t, err, "%v", dir)
		m := end.Map()
		ey, ez := m.QV[2], new(big.Rat).Neg(m.QV[1])
		unit := new(big.Rat).Add(new(big.Rat).Mul(ey, ey), new(big.Rat).Mul(ez, ez))
		require.Zero(t, unit.Cmp(rat(1, 1)), "%v: e is not an exact unit vector", dir)

		q := loftface.ConePullback(m, apex, slope)
		for i := range 5 {
			require.Zero(t, q[i].Sign(), "%v: coefficient %d", dir, i)
		}
		require.Zero(t, q[5].Cmp(new(big.Rat).Mul(slope, slope)), "%v: v^2 coefficient", dir)

		// Compare with the direction scaled into a safe range.
		scale := math.Max(math.Abs(dir[0]), math.Abs(dir[1]))
		wantY, wantZ := dir[0]/scale, dir[1]/scale
		n := math.Hypot(wantY, wantZ)
		gotY, _ := ey.Float64()
		gotZ, _ := ez.Float64()
		require.InDelta(t, wantY/n, gotY, 1e-12, "%v", dir)
		require.InDelta(t, wantZ/n, gotZ, 1e-12, "%v", dir)
		require.Equal(t, dir[0] < 0, ey.Sign() < 0, "%v: y quadrant", dir)
		require.Equal(t, dir[1] < 0, ez.Sign() < 0, "%v: z quadrant", dir)
	}
}
