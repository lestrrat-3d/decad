package revolvemesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// filterSpecials are the float ends the differential tests combine: signed
// zeros, subnormals, both sides of RevUp's 2^-969 threshold, powers of two
// (the ties a one-product step has to avoid) and their neighbours, huge
// magnitudes, the range ends, infinities and NaN.
func filterSpecials() []float64 {
	var out []float64
	for _, x := range []float64{
		0, math.SmallestNonzeroFloat64, 0x1p-1050, 0x1p-1022, 0x1p-969, 0x1p-500,
		0.1, 1, 1.5, 3, 0x1p52, 1e300, 0x1p1023, math.MaxFloat64, math.Inf(1),
	} {
		out = append(out, x, -x, math.Nextafter(x, 0), -math.Nextafter(x, 0),
			math.Nextafter(x, math.Inf(1)), -math.Nextafter(x, math.Inf(1)))
	}
	return append(out, math.NaN(), -math.NaN())
}

func requireSameIv(t *testing.T, want, got RevIv, what string, args ...any) {
	t.Helper()
	if math.Float64bits(want.Lo) != math.Float64bits(got.Lo) || math.Float64bits(want.Hi) != math.Float64bits(got.Hi) {
		require.Failf(t, what, "want [%x, %x], got [%x, %x] for %v", math.Float64bits(want.Lo), math.Float64bits(want.Hi),
			math.Float64bits(got.Lo), math.Float64bits(got.Hi), args)
	}
}

// TestRevStepMatchesProof compares RevUp and RevDown with proof.NextUp and
// proof.NextDown by bit pattern on the specials, every power of two with its
// neighbours, and random bit patterns.
func TestRevStepMatchesProof(t *testing.T) {
	t.Parallel()
	check := func(x float64) {
		if math.Float64bits(RevUp(x)) != math.Float64bits(proof.NextUp(x)) {
			require.Failf(t, "RevUp", "x = %x", math.Float64bits(x))
		}
		if math.Float64bits(RevDown(x)) != math.Float64bits(proof.NextDown(x)) {
			require.Failf(t, "RevDown", "x = %x", math.Float64bits(x))
		}
	}
	for _, x := range filterSpecials() {
		check(x)
	}
	for e := -1074; e <= 1023; e++ {
		p := math.Ldexp(1, e)
		for _, x := range []float64{p, math.Nextafter(p, 0), math.Nextafter(p, math.Inf(1))} {
			check(x)
			check(-x)
		}
	}
	r := rand.New(rand.NewPCG(3, 5))
	for range 200_000 {
		check(math.Float64frombits(r.Uint64()))
	}
}

// TestRevIvMulMatchesLegacy compares RevIvMul, revIvScale and revIvPointDot
// with the four-product math.Min/math.Max form, end for end by bit pattern:
// every pair of intervals built from the specials' main values, and random
// pairs over every special, straddling zero, degenerate, reversed, infinite
// and NaN ends alike.
func TestRevIvMulMatchesLegacy(t *testing.T) {
	t.Parallel()
	sp := filterSpecials()
	intervals := func(ends []float64) []RevIv {
		var out []RevIv
		for _, lo := range ends {
			for _, hi := range ends {
				out = append(out, RevIv{lo, hi})
			}
		}
		return out
	}
	// Every sixth special is a main value or its negation, NaN aside.
	var main []float64
	for i := 0; i+1 < len(sp)-2; i += 6 {
		main = append(main, sp[i], sp[i+1])
	}
	main = append(main, math.NaN())
	small := intervals(main)
	for _, a := range small {
		for _, b := range small {
			requireSameIv(t, legacyRevIvMul(a, b), RevIvMul(a, b), "RevIvMul", a, b)
		}
	}
	all := intervals(sp)
	for _, q := range sp {
		for _, g := range all {
			requireSameIv(t, legacyRevIvMul(RevIv{q, q}, g), revIvScale(q, g), "revIvScale", q, g)
		}
	}
	r := rand.New(rand.NewPCG(9, 13))
	pick := func() RevIv { return all[r.IntN(len(all))] }
	rounds := 100_000
	if raceDetector {
		rounds = 30_000
	}
	for range rounds {
		a, b := pick(), pick()
		requireSameIv(t, legacyRevIvMul(a, b), RevIvMul(a, b), "RevIvMul", a, b)
		p := r3.NewVec(sp[r.IntN(len(sp))], sp[r.IntN(len(sp))], sp[r.IntN(len(sp))])
		g, h := RevIvVec{pick(), pick(), pick()}, RevIvVec{pick(), pick(), pick()}
		requireSameIv(t, legacyRevIvDot(RevIvPoint(p), g), revIvPointDot(p, g), "revIvPointDot", p, g)
		requireSameIv(t, legacyRevIvDot(g, h), RevIvDot(g, h), "RevIvDot", g, h)
	}
}

// filterDiffTriangles is a varied set of float triangles: unit-scale, huge,
// subnormal, mixed-magnitude, flat and repeated-corner ones, with powers of
// two and signed zeros among the coordinates.
func filterDiffTriangles(r *rand.Rand, n int) [][3]r3.Vec {
	sp := filterSpecials()
	coord := func(kind int) float64 {
		switch kind {
		case 0:
			return (r.Float64() - 0.5) * 8
		case 1:
			return (r.Float64() - 0.5) * 1e300
		case 2:
			return (r.Float64() - 0.5) * 0x1p-1060
		case 3:
			return math.Ldexp(1, r.IntN(40)-20) * float64(r.IntN(3)-1)
		default:
			if s := sp[r.IntN(len(sp))]; !math.IsInf(s, 0) && !math.IsNaN(s) {
				return s
			}
			return math.Copysign(0, -1)
		}
	}
	var out [][3]r3.Vec
	for range n {
		kind := r.IntN(5)
		base := r3.NewVec(coord(kind), coord(kind), coord(kind))
		var tri [3]r3.Vec
		tri[0] = base
		for k := 1; k < 3; k++ {
			k2 := kind
			if r.IntN(4) == 0 {
				k2 = r.IntN(5)
			}
			tri[k] = base.Add(r3.NewVec(coord(k2), coord(k2), coord(k2)))
		}
		if r.IntN(10) == 0 {
			tri[2] = tri[1]
		}
		out = append(out, tri)
	}
	return out
}

// TestRevolveSeparatedFloatMatchesLegacy compares the pre-test's own-normal
// projection, its projections onto other axes, and its separation verdict
// with the legacy form on varied triangle pairs.
func TestRevolveSeparatedFloatMatchesLegacy(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(17, 19))
	n := 220
	if raceDetector {
		n = 90
	}
	var data []RevolveAuditTri
	for _, tri := range filterDiffTriangles(r, n) {
		d, ok := NewRevolveAuditTri(tri[:], [3]int{0, 1, 2})
		if !ok {
			continue
		}
		data = append(data, d)
		lo, hi := legacyRevIvProject(d.Fp, d.Fn)
		requireSameIv(t, RevIv{lo, hi}, RevIv{d.FnProj[0], d.FnProj[1]}, "FnProj", tri)
		lo, hi = legacyRevIvProject(d.Fp, d.Fu)
		gotLo, gotHi := RevIvProject(d.Fp, d.Fu)
		requireSameIv(t, RevIv{lo, hi}, RevIv{gotLo, gotHi}, "RevIvProject", tri)
	}
	require.Greater(t, len(data), n/2)
	separated := 0
	for i := range data {
		for j := range data {
			for _, delta := range []float64{1e-9, 0.5} {
				want := legacyRevolveSeparatedFloat(data[i], data[j], delta)
				if got := RevolveSeparatedFloat(data[i], data[j], delta); got != want {
					require.Failf(t, "RevolveSeparatedFloat", "pair %d, %d, delta %g: want %v", i, j, delta, want)
				}
				if want {
					separated++
				}
			}
		}
	}
	require.Positive(t, separated)
}
