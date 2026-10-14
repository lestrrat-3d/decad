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

// The nestedMap cap P(u, v) = (8 + qux*u, quy*u, v) has the pullback
// (quy^2 - qux^2)*u^2 + v^2 against the cone x + r = 8.
func TestConeTangentAlongRuling(t *testing.T) {
	apex, slope := rat(8, 1), rat(1, 1)
	box := [4]*big.Rat{rat(-4, 1), rat(-3, 1), rat(-1, 1), rat(1, 1)}
	eps := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 80))
	half := rat(1, 2)
	// (1+eps)/2 and (1-eps)/2 differ by eps and sum to 1, so their squares
	// differ by exactly eps.
	plus := new(big.Rat).Mul(half, new(big.Rat).Add(rat(1, 1), eps))
	minus := new(big.Rat).Mul(half, new(big.Rat).Sub(rat(1, 1), eps))

	exact := nestedMap(rat(1, 1), rat(1, 1))
	require.True(t, loftface.ConeTangentAlongRuling(exact, box, apex, slope), "q3 = 0 is the tangent form")
	require.False(t, loftface.ConeStrictOutsideForm(loftface.ConePullback(exact, apex, slope)))
	require.True(t, loftface.ConeTangentAlongRuling(tangentFixture(t), box, apex, slope),
		"the issued tangent map is accepted")

	over := nestedMap(minus, plus) // q3 = +eps
	require.Zero(t, loftface.ConePullback(over, apex, slope)[3].Cmp(eps))
	require.False(t, loftface.ConeTangentAlongRuling(over, box, apex, slope),
		"q3 = +2^-80 is not tangent")
	require.True(t, loftface.ConeStrictOutsideForm(loftface.ConePullback(over, apex, slope)),
		"q3 = +2^-80 belongs to the strict path alone")

	under := nestedMap(plus, minus) // q3 = -eps
	require.Zero(t, loftface.ConePullback(under, apex, slope)[3].Cmp(new(big.Rat).Neg(eps)))
	require.False(t, loftface.ConeTangentAlongRuling(under, box, apex, slope))
	require.False(t, loftface.ConeStrictOutsideForm(loftface.ConePullback(under, apex, slope)),
		"q3 = -2^-80 is outside the cone and admitted by neither form")

	cross := nestedMap(rat(1, 1), rat(1, 1))
	cross.QV = loftface.RatVec{rat(0, 1), eps, rat(1, 1)}
	require.False(t, loftface.ConeTangentAlongRuling(cross, box, apex, slope), "a 2^-80 cross term")

	offset := nestedMap(rat(1, 1), rat(1, 1))
	offset.Constant = loftface.RatVec{rat(8, 1), eps, rat(0, 1)}
	require.False(t, loftface.ConeTangentAlongRuling(offset, box, apex, slope), "a 2^-80 constant term")

	flat := nestedMap(rat(1, 1), rat(1, 1))
	flat.QV = loftface.RatVec{rat(0, 1), rat(0, 1), rat(0, 1)}
	require.False(t, loftface.ConeTangentAlongRuling(flat, box, apex, slope), "no v^2 term")

	for name, straddle := range map[string][4]*big.Rat{
		"straddles":  {rat(-1, 1), rat(1, 1), rat(-1, 1), rat(1, 1)},
		"touches":    {rat(-1, 1), rat(0, 1), rat(-1, 1), rat(1, 1)},
		"touches hi": {rat(0, 1), rat(1, 1), rat(-1, 1), rat(1, 1)},
	} {
		require.False(t, loftface.ConeTangentAlongRuling(exact, straddle, apex, slope), "box %s u = 0", name)
	}
	require.True(t, loftface.ConeTangentAlongRuling(exact, [4]*big.Rat{rat(3, 1), rat(4, 1), rat(-1, 1), rat(1, 1)},
		apex, slope), "the other side of u = 0")
	require.False(t, loftface.ConeTangentAlongRuling(exact, box, nil, slope))
}

// sideFixture is the cone x + 3r/4 = 8 (far = -4) with the tangent end map
// along +Y. Its plane has phi = rho*(6 - (x - 8)) > 0 at the axis midpoint
// (2, 0, 0), so the cone lies on the positive side.
func sideFixture(t *testing.T) (loftface.Map, [4]*big.Rat) {
	t.Helper()
	end, err := loftface.NewTangentEnd(rat(8, 1), rat(3, 4), 0, 1)
	require.NoError(t, err)
	return end.Map(), [4]*big.Rat{rat(-4, 1), rat(-3, 1), rat(-1, 1), rat(1, 1)}
}

// movedAlongNormal returns m shifted by c*N, where N = QU x QV.
func movedAlongNormal(m loftface.Map, c *big.Rat) loftface.Map {
	n := loftface.VCross(m.QU, m.QV)
	m.Constant = loftface.VAdd(m.Constant, loftface.VScale(n, c))
	return m
}

func TestConeOnLoftSideRequiresSharedSide(t *testing.T) {
	final, box := sideFixture(t)
	apex, slope, far := rat(8, 1), rat(3, 4), rat(-4, 1)

	plane, ok := loftface.NewTangentPlane(final, 1)
	require.True(t, ok)
	require.Positive(t, plane.Phi(loftface.RatVec{rat(2, 1), rat(0, 1), rat(0, 1)}).Sign(),
		"the axis midpoint is on the positive side of the end plane")

	axisSide := movedAlongNormal(final, rat(1, 8))
	side, ok := loftface.ConeOnLoftSide(final, axisSide, box, apex, slope, far)
	require.True(t, ok, "a preceding cap on the cone's side is accepted")
	require.Equal(t, 1, side)

	reflected := movedAlongNormal(final, rat(-1, 8))
	_, ok = loftface.ConeOnLoftSide(final, reflected, box, apex, slope, far)
	require.False(t, ok, "a preceding cap reflected across the plane is refused")

	straddling := movedAlongNormal(final, rat(1, 8))
	straddling.QV = loftface.VAdd(straddling.QV, loftface.VScale(loftface.VCross(final.QU, final.QV), rat(10, 1)))
	_, ok = loftface.ConeOnLoftSide(final, straddling, box, apex, slope, far)
	require.False(t, ok, "a preceding cap that crosses the plane has no side")

	_, ok = loftface.ConeOnLoftSide(final, final, box, apex, slope, far)
	require.False(t, ok, "a preceding cap on the plane has no strict side")

	_, ok = loftface.ConeOnLoftSide(final, axisSide, box, apex, slope, apex)
	require.False(t, ok, "far = apex puts the axis midpoint on the plane")
	_, ok = loftface.ConeOnLoftSide(final, axisSide, box, apex, slope, rat(9, 1))
	require.False(t, ok, "far above the apex")
	_, ok = loftface.ConeOnLoftSide(final, axisSide, box, apex, rat(1, 2), far)
	require.False(t, ok, "a plane that does not touch this cone")

	// Flipping the plane orientation flips both phi signs, so the reported
	// side is -1 and a preceding cap on the cone side is still accepted.
	flipped := final
	flipped.QV = loftface.VScale(final.QV, rat(-1, 1))
	side, ok = loftface.ConeOnLoftSide(flipped, movedAlongNormal(flipped, rat(1, 8)), box, apex, slope, far)
	require.False(t, ok, "the preceding cap is on the negative side of the flipped plane, away from m")
	require.Zero(t, side)
	side, ok = loftface.ConeOnLoftSide(flipped, movedAlongNormal(flipped, rat(-1, 8)), box, apex, slope, far)
	require.True(t, ok)
	require.Equal(t, -1, side)

	negative := axisSide
	negative.G0 = rat(-1, 1)
	_, ok = loftface.ConeOnLoftSide(final, negative, box, apex, slope, far)
	require.False(t, ok, "a preceding cap with a nonpositive denominator")
}

func TestNewTangentPlaneRefuses(t *testing.T) {
	final, _ := sideFixture(t)
	_, ok := loftface.NewTangentPlane(final, 0)
	require.False(t, ok)
	collapsed := final
	collapsed.QV = loftface.RatVec{rat(0, 1), rat(0, 1), rat(0, 1)}
	_, ok = loftface.NewTangentPlane(collapsed, 1)
	require.False(t, ok)
	projective := final
	projective.GU = rat(1, 1)
	_, ok = loftface.NewTangentPlane(projective, 1)
	require.False(t, ok)
}

func TestCapsOnSideRefusesEarlierCapAcrossPlane(t *testing.T) {
	final, box := sideFixture(t)
	near := movedAlongNormal(final, rat(1, 8))
	far := movedAlongNormal(final, rat(1, 4))
	require.True(t, loftface.CapsOnSide(final, []loftface.Map{far, near}, box, 1),
		"earlier caps strictly on the cone's side are accepted")
	require.False(t, loftface.CapsOnSide(final, []loftface.Map{far, near}, box, -1), "the wrong side")
	require.False(t, loftface.CapsOnSide(final, []loftface.Map{far, near}, box, 0), "no side")
	require.False(t, loftface.CapsOnSide(final, nil, box, 1), "no earlier cap")

	// A corner exactly on the plane, or 2^-60 beyond it, is refused in any
	// position of the list, including the first, which a last-cap-only check
	// would not read.
	// Shift so that phi = 0 at the corner (u, v) = (box[0], box[2]) and phi > 0 at the others.
	normal := loftface.VCross(final.QU, final.QV)
	shift := func(m loftface.Map, delta *big.Rat) loftface.Map {
		out := m
		plane, ok := loftface.NewTangentPlane(final, 1)
		require.True(t, ok)
		phi := plane.Phi(m.At(box[0], box[2]))
		nn := loftface.VDot(normal, normal)
		// Move along the normal so phi at the corner becomes delta.
		step := new(big.Rat).Quo(new(big.Rat).Sub(delta, phi), nn)
		out.Constant = loftface.VAdd(m.Constant, loftface.VScale(normal, step))
		return out
	}
	tilted := far
	tilted.QU = loftface.VAdd(far.QU, loftface.VScale(normal, rat(1, 4)))
	tilted.QV = loftface.VAdd(far.QV, loftface.VScale(normal, rat(1, 4)))
	eps := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 60))
	for name, delta := range map[string]*big.Rat{"on the plane": rat(0, 1), "2^-60 beyond": new(big.Rat).Neg(eps)} {
		corner := shift(tilted, delta)
		require.False(t, loftface.CapsOnSide(final, []loftface.Map{corner, near}, box, 1), "%s, first of the list", name)
		require.False(t, loftface.CapsOnSide(final, []loftface.Map{near, corner}, box, 1), "%s, last of the list", name)
	}
	require.True(t, loftface.CapsOnSide(final, []loftface.Map{shift(tilted, eps), near}, box, 1),
		"2^-60 inside the plane is strictly on the side")
	negative := near
	negative.G0 = rat(-1, 1)
	require.False(t, loftface.CapsOnSide(final, []loftface.Map{far, negative}, box, 1), "nonpositive denominator")
}

// affineMap is the cap map P(u, v) = constant + u*qu + v*qv with denominator 1.
func affineMap(constant, qu, qv [3]int64) loftface.Map {
	vec := func(a [3]int64) loftface.RatVec { return loftface.RatVec{rat(a[0], 1), rat(a[1], 1), rat(a[2], 1)} }
	return loftface.Map{
		Constant: vec(constant), Q0: vec([3]int64{}), QU: vec(qu), QV: vec(qv),
		G0: rat(1, 1), GU: rat(0, 1), GV: rat(0, 1),
	}
}

// With a negative slope the squared pullback is still C*v^2, but the
// inequality x + slope*r >= apex that G1 and non-intrusion need is lost. The
// final cap P = (8+u, u, v) over u in [-4,-3] reaches (4,-4,0), which
// satisfies x - r = 0 < 8, and the cone set K contains (4,-8,0) with phi = 4
// on the other side of the plane from the preceding cap.
func TestConeHelpersRefuseNonpositiveSlope(t *testing.T) {
	apex, far := rat(8, 1), rat(0, 1)
	box := [4]*big.Rat{rat(-4, 1), rat(-3, 1), rat(-1, 1), rat(1, 1)}
	final := affineMap([3]int64{8, 0, 0}, [3]int64{1, 1, 0}, [3]int64{0, 0, 1})
	previous := affineMap([3]int64{7, 1, 0}, [3]int64{1, 1, 0}, [3]int64{0, 0, 1})

	// The pullback is exactly v^2 for slope 1 and slope -1 alike.
	for _, slope := range []*big.Rat{rat(1, 1), rat(-1, 1)} {
		q := loftface.ConePullback(final, apex, slope)
		for i := range 5 {
			require.Zero(t, q[i].Sign(), "slope %s coefficient %d", slope, i)
		}
		require.Zero(t, q[5].Cmp(rat(1, 1)), "slope %s", slope)
	}
	_, ok := loftface.ConeOnLoftSide(final, previous, box, apex, rat(1, 1), far)
	require.True(t, ok, "the positive slope is the valid counterpart")
	require.True(t, loftface.ConeTangentAlongRuling(final, box, apex, rat(1, 1)))

	for _, slope := range []*big.Rat{rat(-1, 1), rat(0, 1)} {
		require.False(t, loftface.ConeTangentAlongRuling(final, box, apex, slope), "ruling, slope %s", slope)
		side, ok := loftface.ConeOnLoftSide(final, previous, box, apex, slope, far)
		require.False(t, ok, "loft side, slope %s", slope)
		require.Zero(t, side)
	}
}

// Each pullback term other than v^2 is isolated by a map that violates only
// that one coefficient. Deleting any single check then admits the map.
func TestConeTangentAlongRulingIsolatesEachTerm(t *testing.T) {
	apex, slope := rat(8, 1), rat(1, 1)
	box := [4]*big.Rat{rat(-4, 1), rat(-3, 1), rat(-1, 1), rat(1, 1)}
	for _, tc := range []struct {
		name  string
		m     loftface.Map
		index int
		want  int64
	}{
		// P = (5, 4, v): y^2 - (8-x)^2 = 7.
		{"q0", affineMap([3]int64{5, 4, 0}, [3]int64{}, [3]int64{0, 0, 1}), 0, 7},
		// P = (9+u, 1-u, v): (1-u)^2 + v^2 - (1+u)^2 = -4u + v^2.
		{"q1", affineMap([3]int64{9, 1, 0}, [3]int64{1, -1, 0}, [3]int64{0, 0, 1}), 1, -4},
		// P = (7-v, 1, 2v): 1 + 4v^2 - (1+v)^2 = 3v^2 - 2v.
		{"q2", affineMap([3]int64{7, 1, 0}, [3]int64{}, [3]int64{-1, 0, 2}), 2, -2},
	} {
		q := loftface.ConePullback(tc.m, apex, slope)
		for i := range 5 {
			if i == tc.index {
				require.Zero(t, q[i].Cmp(rat(tc.want, 1)), "%s: coefficient %d", tc.name, i)
				continue
			}
			require.Zero(t, q[i].Sign(), "%s: coefficient %d", tc.name, i)
		}
		require.Positive(t, q[5].Sign(), tc.name)
		require.False(t, loftface.ConeTangentAlongRuling(tc.m, box, apex, slope), tc.name)
	}
}

func TestTangentPlanePsiHasBothSigns(t *testing.T) {
	final, _ := sideFixture(t)
	// Moving along N = QU x QV by c gives phi = c*|N|^2 at the shifted cap.
	above := movedAlongNormal(final, rat(1, 8)).At(rat(-3, 1), rat(1, 2))
	below := movedAlongNormal(final, rat(-1, 8)).At(rat(-3, 1), rat(1, 2))
	for _, side := range []int{1, -1} {
		plane, ok := loftface.NewTangentPlane(final, side)
		require.True(t, ok)
		require.Equal(t, side, plane.Side)
		require.Positive(t, plane.Phi(above).Sign())
		require.Negative(t, plane.Phi(below).Sign())
		require.Equal(t, side, plane.Psi(above).Sign(), "side %d above", side)
		require.Equal(t, -side, plane.Psi(below).Sign(), "side %d below", side)
		require.Zero(t, plane.Psi(above).Cmp(new(big.Rat).Mul(rat(int64(side), 1), plane.Phi(above))))
		require.Zero(t, plane.Psi(final.At(rat(-3, 1), rat(1, 2))).Sign(), "a point of the plane")
	}
}

// The map P(u, v) = (1,2,3) + ((2,4,-2) + u*(1,2,0) + v*(0,1,1)) / 2 has its
// (u,v) = (0,0) point at (2,4,2) and tangent directions (1/2,1,0) and
// (0,1/2,1/2). The plane through (2,4,2) with normal (1,2,0) x (0,1,1) =
// (2,-1,1) is 2x - y + z = 2, found by hand and independent of capPlane.
func TestNewTangentPlaneWithNonzeroOriginAndDenominator(t *testing.T) {
	m := affineMap([3]int64{1, 2, 3}, [3]int64{1, 2, 0}, [3]int64{0, 1, 1})
	m.Q0 = loftface.RatVec{rat(2, 1), rat(4, 1), rat(-2, 1)}
	m.G0 = rat(2, 1)
	plane, ok := loftface.NewTangentPlane(m, -1)
	require.True(t, ok)
	for i, want := range []int64{2, -1, 1} {
		require.Zero(t, plane.N[i].Cmp(rat(want, 1)), "N[%d] = %s", i, plane.N[i])
	}
	require.Zero(t, plane.D.Cmp(rat(2, 1)), "D = %s", plane.D)
	require.Equal(t, -1, plane.Side)
	for _, uv := range [][2]*big.Rat{{rat(0, 1), rat(0, 1)}, {rat(3, 1), rat(-5, 7)}, {rat(-1, 3), rat(4, 1)}} {
		require.Zero(t, plane.Phi(m.At(uv[0], uv[1])).Sign(), "point %v is on the plane", uv)
	}
	// The origin (2,4,2) is on the plane; the unshifted-origin plane through
	// the constant (1,2,3) would give phi = 2-2+3-2 = 1 there instead.
	require.Zero(t, plane.Phi(loftface.RatVec{rat(2, 1), rat(4, 1), rat(2, 1)}).Sign())
	require.Zero(t, plane.Phi(loftface.RatVec{rat(1, 1), rat(2, 1), rat(3, 1)}).Cmp(rat(1, 1)))
}
