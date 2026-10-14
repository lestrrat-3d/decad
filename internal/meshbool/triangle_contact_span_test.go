package meshbool_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestOrient2FloatUnderflowFallsBack(t *testing.T) {
	m := math.SmallestNonzeroFloat64
	a := r3.Vec{X: -0x1p-52}
	b := r3.Vec{X: 0x1.e124924924925p+3, Y: 421 * m}
	c := r3.Vec{X: 2.5, Y: 70 * m}
	xp := func(p r3.Vec) meshbool.Xp2 { return meshbool.NewXP2FromXpt(proof.XptOf(p), 0, 1) }
	require.Equal(t, -1, meshbool.Cross2x(xp(a), xp(b), xp(c)).Sign())
	require.Zero(t, meshbool.Orient2Float([2]float64{a.X, a.Y}, [2]float64{b.X, b.Y}, [2]float64{c.X, c.Y}))
}

func TestTriTriContactSpanUnderflowKeepsCoplanarRegion(t *testing.T) {
	m := math.SmallestNonzeroFloat64
	a := r3.Vec{X: -0x1p-52}
	b := r3.Vec{X: 0x1.e124924924925p+3, Y: 421 * m}
	c := r3.Vec{X: 2.5, Y: 70 * m}
	ta := [3]r3.Vec{a, b, {Y: -m}}
	tb := [3]r3.Vec{c, {X: 2.5, Y: 71 * m}, {X: 3, Y: 85 * m}}
	var xa, xb [3]proof.Xpt
	for i := range 3 {
		xa[i], xb[i] = proof.XptOf(ta[i]), proof.XptOf(tb[i])
	}
	na := proof.Xcross(proof.Xsub(xa[1], xa[0]), proof.Xsub(xa[2], xa[0]))
	nb := proof.Xcross(proof.Xsub(xb[1], xb[0]), proof.Xsub(xb[2], xb[0]))
	require.False(t, meshbool.CoplanarFloatSeparated(ta, tb, na))
	span, err := meshbool.TriTriContactSpanCore(ta, tb, xa, xb, na, nb, nil, nil, nil, nil,
		true, r3.Vec{}, r3.Vec{}, false)
	require.NoError(t, err)
	require.Equal(t, meshbool.ContactRegion, span.Kind)
}

func spanTestPointsEqual(a, b proof.Xpt) bool {
	same := func(x, wx, y, wy *big.Int) bool {
		return new(big.Int).Mul(x, wy).Cmp(new(big.Int).Mul(y, wx)) == 0
	}
	return same(a.X, a.W, b.X, b.W) && same(a.Y, a.W, b.Y, b.W) && same(a.Z, a.W, b.Z, b.W)
}

func TestTriTriContactSpanMatchesClassify(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(67, 71))
	corner := func() r3.Vec {
		return r3.NewVec(float64(rng.IntN(7)-3), float64(rng.IntN(7)-3), float64(rng.IntN(7)-3))
	}
	lift := func(c [3]r3.Vec) [3]proof.Xpt {
		return [3]proof.Xpt{proof.XptOf(c[0]), proof.XptOf(c[1]), proof.XptOf(c[2])}
	}
	normal := func(x [3]proof.Xpt) proof.Xpt {
		return proof.Xcross(proof.Xsub(x[1], x[0]), proof.Xsub(x[2], x[0]))
	}
	crossings, pairs := 0, 0
	for pairs < 2000 {
		ta := [3]r3.Vec{corner(), corner(), corner()}
		tb := [3]r3.Vec{corner(), corner(), corner()}
		xta, xtb := lift(ta), lift(tb)
		na, nb := normal(xta), normal(xtb)
		if proof.XdotSign(na, na) == 0 || proof.XdotSign(nb, nb) == 0 {
			continue
		}
		pairs++
		var signsA, signsB [3]int
		for i := range 3 {
			signsB[i] = proof.OrientSignExact(xta[0], xta[1], xta[2], xtb[i])
			signsA[i] = proof.OrientSignExact(xtb[0], xtb[1], xtb[2], xta[i])
		}
		for _, exact := range []bool{false, true} {
			var sa, sb *[3]int
			if exact {
				sa, sb = &signsA, &signsB
			}
			full, ferr := meshbool.TriTriClassifyCore(ta, tb, xta, xtb, na, nb, nil, nil, sa, sb,
				!exact, r3.Vec{}, r3.Vec{}, false)
			span, serr := meshbool.TriTriContactSpanCore(ta, tb, xta, xtb, na, nb, nil, nil, sa, sb,
				!exact, r3.Vec{}, r3.Vec{}, false)
			require.Equal(t, ferr == nil, serr == nil)
			if ferr != nil {
				require.EqualError(t, serr, ferr.Error())
			}
			require.Equal(t, full.Kind, span.Kind)
			if full.Kind != meshbool.ContactPoint && full.Kind != meshbool.ContactSegment {
				continue
			}
			crossings++
			require.True(t, spanTestPointsEqual(full.P0, span.P0))
			require.True(t, spanTestPointsEqual(full.P1, span.P1))
		}
	}
	require.GreaterOrEqual(t, crossings, 200)
}
