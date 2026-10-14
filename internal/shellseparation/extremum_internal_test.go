package shellseparation

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// edgeFloats are the fixed operands every random operand set is mixed with.
func edgeFloats() []float64 {
	base := []float64{
		0, math.Copysign(0, -1), math.SmallestNonzeroFloat64, math.MaxFloat64, 1,
		0x1p-1022, 0x1p-1023,
	}
	var out []float64
	for _, b := range base {
		out = append(out, b, -b, math.Nextafter(b, math.Inf(1)), math.Nextafter(b, math.Inf(-1)))
	}
	finite := out[:0]
	for _, v := range out {
		if !math.IsInf(v, 0) {
			finite = append(finite, v)
		}
	}
	return finite
}

// randFloat returns a random finite float with a binary exponent in [minExp, maxExp].
func randFloat(r *rand.Rand, minExp, maxExp int) float64 {
	v := math.Ldexp(0.5+r.Float64()/2, minExp+r.IntN(maxExp-minExp+1))
	if r.IntN(2) == 0 {
		return -v
	}
	return v
}

func wideFloat(r *rand.Rand, edges []float64) float64 {
	if r.IntN(8) == 0 {
		return edges[r.IntN(len(edges))]
	}
	return randFloat(r, -1070, 1000)
}

func requireEncloses(t *testing.T, lo, hi float64, exact *big.Rat, what string) {
	t.Helper()
	if !math.IsInf(lo, -1) {
		require.LessOrEqual(t, ratF(lo).Cmp(exact), 0, "%s: lower end %g above exact", what, lo)
	}
	if !math.IsInf(hi, 1) {
		require.GreaterOrEqual(t, ratF(hi).Cmp(exact), 0, "%s: upper end %g below exact", what, hi)
	}
}

func TestOutwardStepsEncloseExactResults(t *testing.T) {
	r := rand.New(rand.NewPCG(1, 2))
	edges := edgeFloats()
	type op struct {
		name     string
		down, up func(a, b float64) float64
		exact    func(a, b *big.Rat) *big.Rat
		skip     func(b *big.Rat) bool
	}
	ops := []op{
		{"add", fAddDown, fAddUp, ratAdd, nil},
		{"sub", fSubDown, fSubUp, ratSub, nil},
		{"mul", fMulDown, fMulUp, ratMul, nil},
		{"quo", fQuoDown, fQuoUp, func(a, b *big.Rat) *big.Rat { return new(big.Rat).Quo(a, b) },
			func(b *big.Rat) bool { return b.Sign() == 0 }},
	}
	for range 20000 {
		a, b := wideFloat(r, edges), wideFloat(r, edges)
		if r.IntN(4) == 0 { // operands of similar size make cancellation and ties likely
			b = math.Nextafter(a, math.Inf(1-2*r.IntN(2)))
		}
		if math.IsInf(b, 0) {
			continue
		}
		ra, rb := ratF(a), ratF(b)
		for _, o := range ops {
			if o.skip != nil && o.skip(rb) {
				continue
			}
			exact := o.exact(ra, rb)
			requireEncloses(t, o.down(a, b), o.up(a, b), exact, o.name)
		}
	}
}

func TestDirectionValueEnclosesExactDot(t *testing.T) {
	r := rand.New(rand.NewPCG(3, 4))
	for i := range 5000 {
		var ny, nz *big.Rat
		if i%2 == 0 {
			ny, nz = ratF(randFloat(r, -40, 40)), ratF(randFloat(r, -40, 40))
		} else {
			a, b := randFloat(r, 0, 10), randFloat(r, -80, -60)
			ny = ratSub(ratF(a), ratF(b))
			ny = ratSub(ny, ratF(randFloat(r, -90, -70)))
			nz = ratSub(ratF(randFloat(r, -3, 3)), ratF(randFloat(r, -75, -55)))
		}
		if i%11 == 0 {
			ny = ratZero()
		}
		if i%13 == 0 {
			nz = ratZero()
		}
		d := newDirection(ny, nz)
		y, z := randFloat(r, -40, 40), randFloat(r, -40, 40)
		if i%7 == 0 {
			y = math.Copysign(0, -1)
		}
		lo, hi := d.value(y, z)
		requireEncloses(t, lo, hi, d.exact(y, z), "value")
	}
}

func TestRatEnclosureHoldsNonFloatRationals(t *testing.T) {
	r := rand.New(rand.NewPCG(11, 12))
	for range 5000 {
		a, b := randFloat(r, -5, 5), randFloat(r, -80, -55)
		q := ratSub(ratF(a), ratF(b))
		lo, hi := ratEnclosure(q)
		require.Equal(t, -1, ratF(lo).Cmp(q), "%g not strictly below the rational", lo)
		require.Equal(t, 1, ratF(hi).Cmp(q), "%g not strictly above the rational", hi)
		f := randFloat(r, -60, 60)
		lo, hi = ratEnclosure(ratF(f))
		require.Equal(t, f, lo)
		require.Equal(t, f, hi)
	}
	huge := new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(3), 1100), big.NewInt(1))
	lo, hi := ratEnclosure(huge)
	require.Equal(t, math.MaxFloat64, lo)
	require.True(t, math.IsInf(hi, 1))
}

func TestEnclosureCoversFusedAndUnfusedEvaluation(t *testing.T) {
	// A float direction evaluated by a separate multiply and add, by a fused
	// multiply-add, and with the two products swapped must each lie inside the
	// enclosure, and the ends must lie strictly outside the unfused value.
	r := rand.New(rand.NewPCG(5, 6))
	for range 5000 {
		ny, nz := randFloat(r, -30, 30), randFloat(r, -30, 30)
		y, z := randFloat(r, -30, 30), randFloat(r, -30, 30)
		d := newDirection(ratF(ny), ratF(nz))
		lo, hi := d.value(y, z)
		unfused := float64(ny*y) + float64(nz*z)
		for _, v := range []float64{unfused, math.FMA(ny, y, float64(nz*z)), math.FMA(nz, z, float64(ny*y))} {
			require.LessOrEqual(t, lo, v)
			require.GreaterOrEqual(t, hi, v)
		}
		if unfused != 0 {
			require.Less(t, lo, unfused)
			require.Greater(t, hi, unfused)
		}
	}
}

func TestCrossingEnclosureEnclosesExactCrossing(t *testing.T) {
	r := rand.New(rand.NewPCG(7, 8))
	edges := edgeFloats()
	done := 0
	for done < 10000 {
		var xk, xn float64
		switch r.IntN(4) {
		case 0:
			xk, xn = wideFloat(r, edges), wideFloat(r, edges)
		case 1:
			xk = randFloat(r, -20, 20)
			xn = math.Nextafter(xk, math.Inf(1-2*r.IntN(2)))
			xn = math.Nextafter(xn, math.Inf(1-2*r.IntN(2)))
		default:
			xk, xn = randFloat(r, -20, 20), randFloat(r, -20, 20)
		}
		if xk == xn || math.IsInf(xn-xk, 0) {
			continue
		}
		limit := xk + (xn-xk)*r.Float64()
		if r.IntN(3) == 0 {
			limit = math.Nextafter(xk, xn)
		}
		if !(limit > min(xk, xn) && limit < max(xk, xn)) {
			continue
		}
		done++
		vk, vn := randFloat(r, -30, 30), randFloat(r, -30, 30)
		kLo, kHi := vk, vk
		nLo, nHi := vn, vn
		if done%2 == 0 {
			kLo, kHi = fDown(vk), fUp(vk)
			nLo, nHi = fDown(vn), fUp(vn)
		}
		tExact := new(big.Rat).Quo(ratSub(ratF(limit), ratF(xk)), ratSub(ratF(xn), ratF(xk)))
		exact := ratAdd(ratF(vk), ratMul(tExact, ratSub(ratF(vn), ratF(vk))))
		tLo, tHi := crossingParamEnclosure(xk, xn, limit)
		requireEncloses(t, tLo, tHi, tExact, "crossing parameter")
		lo, hi := crossingEnclosure(xk, xn, limit, kLo, kHi, nLo, nHi)
		// Deleting the [0, 1] clamps in crossingEnclosure only loosens the
		// enclosure and does not fail this test.
		requireEncloses(t, lo, hi, exact, "crossing")
	}
}

// slabProjectionReference is the exact-only slabProjection before the float filter.
func slabProjectionReference(p Shell, facets []int, slabLo, slabHi float64, ny, nz *big.Rat,
	norm, axialExtra float64) (*big.Rat, *big.Rat, bool) {
	var lo, hi *big.Rat
	if len(p.VertexBound) != len(p.Verts) {
		return nil, nil, false
	}
	include := func(value, charge *big.Rat) {
		lower, upper := ratSub(value, charge), ratAdd(value, charge)
		if lo == nil || lower.Cmp(lo) < 0 {
			lo = lower
		}
		if hi == nil || upper.Cmp(hi) > 0 {
			hi = upper
		}
	}
	for _, i := range facets {
		tri := p.Tris[i]
		beta := max(p.VertexBound[tri[0]], p.VertexBound[tri[1]], p.VertexBound[tri[2]])
		if axialExtra > 0 {
			beta = proofbound.AbsSumUpper(beta, axialExtra)
		}
		chargeBound := proofbound.ProductUpper(norm, beta)
		lower := math.Nextafter(slabLo-beta, math.Inf(-1))
		upper := math.Nextafter(slabHi+beta, math.Inf(1))
		if proofbound.IsNonFinite(beta) || beta < 0 ||
			proofbound.IsNonFinite(chargeBound) ||
			proofbound.IsNonFinite(lower) || proofbound.IsNonFinite(upper) {
			return nil, nil, false
		}
		charge := ratF(chargeBound)
		xLo := ratF(lower)
		xHi := ratF(upper)
		var x, value [3]*big.Rat
		for k, vi := range tri {
			v := p.Verts[vi]
			x[k] = ratF(v.X)
			value[k] = ratAdd(ratMul(ny, ratF(v.Y)), ratMul(nz, ratF(v.Z)))
			if x[k].Cmp(xLo) >= 0 && x[k].Cmp(xHi) <= 0 {
				include(value[k], charge)
			}
		}
		for k := range 3 {
			next := (k + 1) % 3
			if x[k].Cmp(x[next]) == 0 {
				continue
			}
			for _, limit := range [...]*big.Rat{xLo, xHi} {
				if (x[k].Cmp(limit) < 0 && x[next].Cmp(limit) > 0) ||
					(x[k].Cmp(limit) > 0 && x[next].Cmp(limit) < 0) {
					t := new(big.Rat).Quo(ratSub(limit, x[k]), ratSub(x[next], x[k]))
					include(ratAdd(value[k], ratMul(t, ratSub(value[next], value[k]))), charge)
				}
			}
		}
	}
	return lo, hi, true
}

// projectionReference is the exact-only projection before the float filter.
func projectionReference(p Shell, ny, nz *big.Rat, norm float64) (*big.Rat, *big.Rat, bool) {
	if len(p.Verts) == 0 || proofbound.IsNonFinite(p.MeshBound) || p.MeshBound < 0 {
		return nil, nil, false
	}
	var lo, hi *big.Rat
	for _, v := range p.Verts {
		if !proofbound.FiniteVec(v) {
			return nil, nil, false
		}
		value := ratAdd(ratMul(ny, ratF(v.Y)), ratMul(nz, ratF(v.Z)))
		if lo == nil || value.Cmp(lo) < 0 {
			lo = value
		}
		if hi == nil || value.Cmp(hi) > 0 {
			hi = value
		}
	}
	chargeBound := proofbound.ProductUpper(norm, p.MeshBound)
	if proofbound.IsNonFinite(chargeBound) {
		return nil, nil, false
	}
	charge := ratF(chargeBound)
	return ratSub(lo, charge), ratAdd(hi, charge), true
}

func randDirection(r *rand.Rand, nonFloat bool) (*big.Rat, *big.Rat) {
	comp := func() *big.Rat {
		switch r.IntN(6) {
		case 0:
			return ratZero()
		case 1:
			return ratF([]float64{1, -1, 0.5, 2}[r.IntN(4)])
		}
		if nonFloat {
			return ratSub(ratF(randFloat(r, -2, 2)), ratF(randFloat(r, -70, -60)))
		}
		return ratF(randFloat(r, -3, 3))
	}
	return comp(), comp()
}

// cancelShell has vertices whose Y and Z cancel under the direction (1, sigma):
// large operands give a wide enclosure around a tiny exact positive value, next
// to vertices (0, 0) with a narrow enclosure around the exact value 0. A filter
// that trusts a float end instead of the exact value picks the wrong candidate.
func cancelShell(r *rand.Rand, sigma float64) Shell {
	s := randShell(r)
	scale := math.Ldexp(1, 20+r.IntN(30))
	for i := range s.Verts {
		if r.IntN(3) == 0 {
			s.Verts[i].Y, s.Verts[i].Z = 0, 0
		} else {
			y := scale * (1 + r.Float64())
			s.Verts[i].Y = y
			s.Verts[i].Z = math.Nextafter(-sigma*y, math.Inf(int(sigma)))
		}
		s.VertexBound[i] = 0
	}
	s.MeshBound = 0
	return s
}

func randShell(r *rand.Rand) Shell {
	values := []float64{-2, -1, -0.5, 0, math.Copysign(0, -1), 0.25, 1, 3}
	scale := math.Ldexp(1, r.IntN(7)-3)
	coord := func() float64 {
		if r.IntN(5) == 0 {
			return randFloat(r, -4, 4)
		}
		return values[r.IntN(len(values))] * scale
	}
	n := 12 + r.IntN(29)
	s := Shell{Verts: make([]r3.Vec, n), VertexBound: make([]float64, n)}
	for i := range s.Verts {
		s.Verts[i] = r3.Vec{X: coord(), Y: coord(), Z: coord()}
		if r.IntN(3) != 0 {
			s.VertexBound[i] = 0.05 * r.Float64()
		}
	}
	for range 8 + r.IntN(53) {
		s.Tris = append(s.Tris, [3]int{r.IntN(n), r.IntN(n), r.IntN(n)})
	}
	s.MeshBound = 0.05 * r.Float64()
	return s
}

func requireSameRat(t *testing.T, want, got *big.Rat, what string) {
	t.Helper()
	require.Equal(t, want == nil, got == nil, "%s nil-ness", what)
	if want == nil {
		return
	}
	require.Zero(t, want.Num().Cmp(got.Num()), "%s numerator", what)
	require.Zero(t, want.Denom().Cmp(got.Denom()), "%s denominator", what)
}

func TestSlabProjectionMatchesReference(t *testing.T) {
	r := rand.New(rand.NewPCG(1, 2))
	for i := range 3000 {
		s := randShell(r)
		sigma := []float64{1, -1}[r.IntN(2)]
		if i%4 == 3 {
			s = cancelShell(r, sigma)
		}
		xs := make([]float64, len(s.Verts))
		for k, v := range s.Verts {
			xs[k] = v.X
		}
		slabLo, slabHi := xs[r.IntN(len(xs))], xs[r.IntN(len(xs))]
		if slabLo > slabHi {
			slabLo, slabHi = slabHi, slabLo
		}
		if slabLo == slabHi {
			slabHi = slabLo + 0.5
		}
		axialExtra := []float64{0, 0.01}[r.IntN(2)]
		if i%3 == 0 {
			// Put one vertex of the first facet exactly on a slab limit.
			tri := s.Tris[0]
			beta := max(s.VertexBound[tri[0]], s.VertexBound[tri[1]], s.VertexBound[tri[2]])
			if axialExtra > 0 {
				beta = proofbound.AbsSumUpper(beta, axialExtra)
			}
			if r.IntN(2) == 0 {
				s.Verts[tri[r.IntN(3)]].X = math.Nextafter(slabLo-beta, math.Inf(-1))
			} else {
				s.Verts[tri[r.IntN(3)]].X = math.Nextafter(slabHi+beta, math.Inf(1))
			}
		}
		facets := make([]int, 0, len(s.Tris))
		for k := range s.Tris {
			if i%5 != 0 || r.IntN(2) == 0 {
				facets = append(facets, k)
			}
		}
		ny, nz := randDirection(r, i%2 == 1)
		if i%4 == 3 {
			ny, nz = ratOne(), ratF(sigma)
		}
		norm := []float64{0, 1, 0.5, 3}[r.IntN(4)]
		wantLo, wantHi, wantOK := slabProjectionReference(s, facets, slabLo, slabHi, ny, nz, norm, axialExtra)
		gotLo, gotHi, gotOK := slabProjection(s, facets, slabLo, slabHi, ny, nz, norm, axialExtra)
		require.Equal(t, wantOK, gotOK, "case %d validity", i)
		requireSameRat(t, wantLo, gotLo, "slab lo")
		requireSameRat(t, wantHi, gotHi, "slab hi")
	}
}

func TestProjectionMatchesReference(t *testing.T) {
	r := rand.New(rand.NewPCG(9, 10))
	for i := range 3000 {
		s := randShell(r)
		sigma := []float64{1, -1}[r.IntN(2)]
		if i%4 == 3 {
			s = cancelShell(r, sigma)
		}
		if i%7 == 0 {
			s.Verts = s.Verts[:1+r.IntN(2)] // ties: every vertex equal
			for k := range s.Verts {
				s.Verts[k] = s.Verts[0]
			}
		}
		ny, nz := randDirection(r, i%2 == 1)
		if i%4 == 3 {
			ny, nz = ratOne(), ratF(sigma)
		}
		norm := []float64{0, 1, 0.5, 3}[r.IntN(4)]
		wantLo, wantHi, wantOK := projectionReference(s, ny, nz, norm)
		gotLo, gotHi, gotOK := projection(s, ny, nz, norm)
		require.Equal(t, wantOK, gotOK, "case %d validity", i)
		requireSameRat(t, wantLo, gotLo, "projection lo")
		requireSameRat(t, wantHi, gotHi, "projection hi")
	}
	empty := Shell{}
	_, _, ok := projection(empty, ratOne(), ratZero(), 1)
	require.False(t, ok)
	bad := Shell{Verts: []r3.Vec{{Y: math.Inf(1)}}}
	_, _, ok = projection(bad, ratOne(), ratZero(), 1)
	require.False(t, ok)
}

func TestRatEnclosureHoldsUnderflowingRationals(t *testing.T) {
	// big.Rat.Float64 reports exact for a nonzero value that underflows to 0.
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1200))
	for _, q := range []*big.Rat{tiny, new(big.Rat).Neg(tiny)} {
		lo, hi := ratEnclosure(q)
		require.LessOrEqual(t, ratF(lo).Cmp(q), 0)
		require.GreaterOrEqual(t, ratF(hi).Cmp(q), 0)
	}
	// The overflow side does not have the quirk: it reports inexact.
	for _, sign := range []int64{1, -1} {
		huge := new(big.Rat).SetFrac(new(big.Int).Mul(big.NewInt(sign), new(big.Int).Lsh(big.NewInt(1), 1100)),
			big.NewInt(1))
		lo, hi := ratEnclosure(huge)
		if sign > 0 {
			require.Equal(t, math.MaxFloat64, lo)
			require.True(t, math.IsInf(hi, 1))
			continue
		}
		require.True(t, math.IsInf(lo, -1))
		require.Equal(t, -math.MaxFloat64, hi)
	}
}

func TestSlabProjectionNaNEnclosureKeepsEveryCandidate(t *testing.T) {
	// Direction (1, 0), Y = MaxFloat64 and a crossing at a limit one
	// subnormal from xk make the crossing enclosure [NaN, NaN] (0 * Inf), so
	// lowCut becomes NaN. Every candidate must then stay for exact evaluation.
	// Dropping the NaN fallback (`lower < lowCut`) loses the exact result.
	lo, hi := crossingEnclosure(0, 1, math.SmallestNonzeroFloat64, math.MaxFloat64, math.MaxFloat64, 0, 0)
	require.True(t, math.IsNaN(lo) && math.IsNaN(hi), "fixture must give a NaN enclosure: %g %g", lo, hi)
	s := Shell{
		Verts:       []r3.Vec{{X: 0, Y: math.MaxFloat64}, {X: 1}, {X: 0, Y: -1}, {X: 0.5, Y: 2}, {X: 0.5, Y: 3}},
		Tris:        [][3]int{{0, 1, 2}, {3, 4, 2}},
		VertexBound: make([]float64, 5),
	}
	// A slab whose lower limit is the smallest subnormal makes the first edge cross it.
	for _, slab := range [][2]float64{{math.SmallestNonzeroFloat64, 2}, {-1, math.SmallestNonzeroFloat64}} {
		wantLo, wantHi, wantOK := slabProjectionReference(s, []int{0, 1}, slab[0], slab[1], ratOne(), ratZero(), 0, 0)
		gotLo, gotHi, gotOK := slabProjection(s, []int{0, 1}, slab[0], slab[1], ratOne(), ratZero(), 0, 0)
		require.Equal(t, wantOK, gotOK)
		requireSameRat(t, wantLo, gotLo, "lo")
		requireSameRat(t, wantHi, gotHi, "hi")
	}
}
