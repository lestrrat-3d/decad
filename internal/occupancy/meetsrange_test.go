package occupancy

import (
	"cmp"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// oldKeeps is the decision the candidate filter made before meetsRange: the
// axis is kept exactly when the gap from the full projRange folds is positive.
func oldKeeps(a r3.Vec, vs []r3.Vec, tlo, thi float64) bool {
	plo, phi := mathProjRange(a, vs)
	return math.Max(plo-thi, tlo-phi) > 0
}

// newKeeps is the decision with the early exit in front.
func newKeeps(a r3.Vec, vs []r3.Vec, tlo, thi float64) bool {
	if meetsRange(a, vs, tlo, thi) {
		return false
	}
	return oldKeeps(a, vs, tlo, thi)
}

var meetsRangeValues = []float64{
	math.NaN(), math.Inf(1), math.Inf(-1), 0, math.Copysign(0, -1), 1, -1, 2, -2,
	5e-324, -5e-324, math.MaxFloat64, -math.MaxFloat64,
}

// TestMeetsRangeKeepsOldDecision walks every sequence of up to four vertex
// projections and every bound pair drawn from NaN, both infinities, both
// zeros and the extremes, and requires the early exit never to change the
// keep decision, and never to fire on a kept axis.
func TestMeetsRangeKeepsOldDecision(t *testing.T) {
	t.Parallel()
	a := r3.Vec{X: 1}
	negZero := math.Copysign(0, -1)
	mismatches, fired, cases := 0, 0, 0
	vs := make([]r3.Vec, 0, 4)
	var walk func()
	walk = func() {
		for _, tlo := range meetsRangeValues {
			for _, thi := range meetsRangeValues {
				cases++
				if meetsRange(a, vs, tlo, thi) {
					fired++
					if oldKeeps(a, vs, tlo, thi) {
						mismatches++
					}
				}
				if oldKeeps(a, vs, tlo, thi) != newKeeps(a, vs, tlo, thi) {
					mismatches++
				}
			}
		}
		if len(vs) == cap(vs) {
			return
		}
		for _, d := range meetsRangeValues {
			vs = append(vs, r3.Vec{X: d, Y: negZero, Z: negZero})
			walk()
			vs = vs[:len(vs)-1]
		}
	}
	walk()
	require.Zero(t, mismatches)
	require.Greater(t, fired, 1000, "the early exit never fired in %d cases", cases)
}

// TestMeetsRangeRandomOracle compares the old and new keep decision on
// seeded random axes, vertex sets and bounds, with special values mixed in
// and with touching ranges.
func TestMeetsRangeRandomOracle(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 7))
	coord := func() float64 {
		switch rng.IntN(6) {
		case 0:
			return meetsRangeValues[rng.IntN(len(meetsRangeValues))]
		case 1:
			return float64(rng.IntN(5) - 2)
		default:
			return (rng.Float64() - 0.5) * math.Pow(10, float64(rng.IntN(8)-3))
		}
	}
	vec := func() r3.Vec { return r3.Vec{X: coord(), Y: coord(), Z: coord()} }
	mismatches, fired, kept := 0, 0, 0
	for range 300_000 {
		a := vec()
		vs := make([]r3.Vec, rng.IntN(8))
		for i := range vs {
			vs[i] = vec()
		}
		tlo, thi := coord(), coord()
		switch rng.IntN(4) {
		case 0: // touching: a bound equals an exact vertex projection
			if len(vs) > 0 {
				d := a.Dot(vs[rng.IntN(len(vs))])
				if rng.IntN(2) == 0 {
					tlo = d
				} else {
					thi = d
				}
			}
		case 1: // a real triangle range
			tri := []r3.Vec{vec(), vec(), vec()}
			tlo, thi = mathProjRange(a, tri)
		}
		early := meetsRange(a, vs, tlo, thi)
		old := oldKeeps(a, vs, tlo, thi)
		if early {
			fired++
		}
		if old {
			kept++
		}
		if (early && old) || old != newKeeps(a, vs, tlo, thi) {
			mismatches++
		}
	}
	require.Zero(t, mismatches)
	require.Positive(t, fired)
	require.Positive(t, kept)
}

// oldMisses is Misses as it stood before meetsRange: every candidate axis
// projects the whole piece through the full fold.
func oldMisses(tri [3]r3.Vec, shift r3.Vec, p *Piece, budget *Budget) (bool, error) {
	var ft [3]r3.Vec
	for i, v := range tri {
		ft[i] = v.Add(shift)
	}
	ok, err := missesBySide(tri, shift, ft, p, budget)
	if ok || err != nil {
		return ok, err
	}
	type cand struct {
		axis r3.Vec
		gap  float64
	}
	var cands []cand
	try := func(a r3.Vec) {
		l := a.Len()
		if !(l > 0) || math.IsInf(l, 0) {
			return
		}
		a = a.Scale(1 / l)
		tlo, thi := projRange(a, ft[:])
		plo, phi := projRange(a, p.fverts)
		if g := math.Max(plo-thi, tlo-phi); g > 0 {
			cands = append(cands, cand{a, g})
		}
	}
	try(ft[1].Sub(ft[0]).Cross(ft[2].Sub(ft[0])))
	for _, f := range p.faces {
		try(f.n)
	}
	if len(cands) == 0 {
		for i := range 3 {
			te := ft[(i+1)%3].Sub(ft[i])
			for _, e := range p.edges {
				try(te.Cross(p.fverts[e[1]].Sub(p.fverts[e[0]])))
			}
		}
	}
	if len(cands) == 0 {
		return false, nil
	}
	slices.SortFunc(cands, func(a, b cand) int { return cmp.Compare(b.gap, a.gap) })
	sx := xvecOf(shift)
	var xt [3]xvec
	for i, v := range tri {
		x := xvecOf(v)
		for k := range 3 {
			x[k].Add(x[k], sx[k])
		}
		xt[i] = x
	}
	for _, c := range cands[:min(len(cands), maxExactTries)] {
		if err := budget.charge(int64(3 + len(p.verts))); err != nil {
			return false, err
		}
		if separated(c.axis, xt, p.verts) {
			return true, nil
		}
	}
	return false, nil
}

// TestMissesMatchesOldCandidateFilter runs Misses and the pre-change copy on
// seeded triangles around small circle prisms and requires the same answer,
// error and exact-work charge. The charge counts the candidates tried, so it
// pins which axes were kept and their order.
func TestMissesMatchesOldCandidateFilter(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(11, 13))
	trues, falses, edgeEdge := 0, 0, 0
	for _, sides := range []int{3, 4, 5, 8} {
		axis := r3.Vec{X: 1, Y: 2, Z: 3}
		k, _ := axis.Normalize()
		c, s := math.Cos(0.7), math.Sin(0.7)
		m := [3][3]float64{
			{c + k.X*k.X*(1-c), k.X*k.Y*(1-c) - k.Z*s, k.X*k.Z*(1-c) + k.Y*s},
			{k.Y*k.X*(1-c) + k.Z*s, c + k.Y*k.Y*(1-c), k.Y*k.Z*(1-c) - k.X*s},
			{k.Z*k.X*(1-c) - k.Y*s, k.Z*k.Y*(1-c) + k.X*s, c + k.Z*k.Z*(1-c)},
		}
		piece, err := CirclePrism(Chain{{M: m, T: [3]float64{3.5, -2.25, 7.125}}}, 1, 0, 2, sides)
		require.NoError(t, err)
		ctr := r3.Vec{X: 3.5, Y: -2.25, Z: 7.125}
		pt := func(spread float64) r3.Vec {
			return ctr.Add(r3.Vec{
				X: (rng.Float64() - 0.5) * spread, Y: (rng.Float64() - 0.5) * spread, Z: (rng.Float64() - 0.5) * spread,
			})
		}
		for range 4000 {
			spread := math.Pow(10, float64(rng.IntN(3))) // 1, 10, 100
			tri := [3]r3.Vec{pt(spread), pt(spread), pt(spread)}
			shift := r3.Vec{X: (rng.Float64() - 0.5) * 2, Y: (rng.Float64() - 0.5) * 2, Z: (rng.Float64() - 0.5) * 2}
			var nb, ob Budget
			nb.Limit, ob.Limit = 1<<40, 1<<40
			if rng.IntN(8) == 0 {
				nb.Limit, ob.Limit = 20, 20 // small ceiling: the same refusal
			}
			nok, nerr := Misses(tri, shift, piece, &nb)
			ook, oerr := oldMisses(tri, shift, piece, &ob)
			require.Equal(t, ook, nok)
			require.Equal(t, oerr == nil, nerr == nil)
			require.Equal(t, ob.Used(), nb.Used())
			if nok {
				trues++
			} else {
				falses++
			}
			if nb.Used() > 0 {
				edgeEdge++
			}
		}
	}
	require.Positive(t, trues)
	require.Positive(t, falses)
	require.Positive(t, edgeEdge)
}
