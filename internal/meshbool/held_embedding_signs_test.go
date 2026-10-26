package meshbool

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestHeldFacetNormalMatchesHomogeneousArithmetic(t *testing.T) {
	t.Parallel()
	check := func(points [3]r3.Vec, sample int) {
		t.Helper()
		reference := func() proof.Xpt {
			a, b, c := proof.XptOf(points[0]), proof.XptOf(points[1]), proof.XptOf(points[2])
			return proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
		}
		same := func(got, want proof.Xpt) {
			t.Helper()
			for axis, pair := range [4][2]*big.Int{{got.X, want.X}, {got.Y, want.Y}, {got.Z, want.Z}, {got.W, want.W}} {
				require.Zero(t, pair[0].Cmp(pair[1]), "sample %d coordinate %d", sample, axis)
			}
		}
		e := newHeldEmbedding(HeldRounding{Verts: append([]r3.Vec(nil), points[:]...), Tris: [][3]int{{0, 1, 2}}},
			proofbound.NewWorkBudget(t.Context()), false)
		want := reference()
		first := e.normal(0)
		same(first, want)
		for axis := range 3 {
			require.False(t, e.xvOK[axis], "sample %d: normal materialized vertex %d", sample, axis)
		}
		_, _, cached := e.facet(0)
		same(cached, want)
		points[1] = points[1].Scale(-1)
		e.setVert(1, points[1])
		changed := e.normal(0)
		same(changed, reference())
		same(first, want)
	}
	for i, points := range [][3]r3.Vec{
		{},
		{{}, {X: 1}, {X: 2}},
		{{X: 0.1, Y: -2.25, Z: 3.5}, {X: -1.75, Y: 0.375, Z: 4.125}, {Y: 2.5}},
		{{X: math.SmallestNonzeroFloat64}, {Y: math.MaxFloat64}, {Z: -1}},
	} {
		check(points, i)
	}
	rng := rand.New(rand.NewPCG(229, 257))
	for sample := range 200 {
		var points [3]r3.Vec
		finite := func() float64 {
			if sample%2 != 0 {
				return math.Ldexp(rng.Float64()-0.5, rng.IntN(41)-20)
			}
			bits := rng.Uint64() &^ (uint64(0x7ff) << 52)
			return math.Float64frombits(bits | (rng.Uint64()%0x7ff)<<52)
		}
		for i := range points {
			points[i] = r3.NewVec(finite(), finite(), finite())
		}
		check(points, sample+4)
	}
}

func TestHeldOrient2MatchesRationalCross(t *testing.T) {
	t.Parallel()
	tiny := math.Ldexp(1, -50)
	cases := [][3]r3.Vec{
		{{}, {X: 1, Y: 1, Z: 1}, {X: 1 + tiny, Y: 1 + 2*tiny, Z: 1 - tiny}},
		{{X: 0.5, Y: -2, Z: 3}, {X: 2, Y: 1, Z: -4}, {X: -1, Y: 0.25, Z: 5}},
		{{X: 1, Y: 1, Z: 1}, {X: 2, Y: 2, Z: 2}, {X: 3, Y: 3, Z: 3}},
	}
	r := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	for _, points := range cases {
		e := newHeldEmbedding(HeldRounding{Verts: points[:]}, proofbound.NewWorkBudget(t.Context()), true)
		for _, axes := range [][2]int{{0, 1}, {0, 2}, {1, 2}} {
			u, v := axes[0], axes[1]
			a, b, c := points[0], points[1], points[2]
			bu := new(big.Rat).Sub(r(AxisCoord(b, u)), r(AxisCoord(a, u)))
			bv := new(big.Rat).Sub(r(AxisCoord(b, v)), r(AxisCoord(a, v)))
			cu := new(big.Rat).Sub(r(AxisCoord(c, u)), r(AxisCoord(a, u)))
			cv := new(big.Rat).Sub(r(AxisCoord(c, v)), r(AxisCoord(a, v)))
			want := new(big.Rat).Sub(new(big.Rat).Mul(bu, cv), new(big.Rat).Mul(bv, cu)).Sign()
			require.Equal(t, want, e.orient2(0, 1, 2, u, v), "points=%v axes=%v", points, axes)
			require.Equal(t, -want, e.orient2(2, 1, 0, u, v), "reversed points=%v axes=%v", points, axes)
		}
	}
}

func TestHeldOrient2UnderflowUsesExactSign(t *testing.T) {
	m := math.SmallestNonzeroFloat64
	verts := []r3.Vec{{X: -0x1p-52}, {X: 0x1.e124924924925p+3, Y: 421 * m}, {X: 2.5, Y: 70 * m}}
	e := newHeldEmbedding(HeldRounding{Verts: verts}, proofbound.NewWorkBudget(t.Context()), true)
	require.Equal(t, -1, e.orient2(0, 1, 2, 0, 1))
}

func TestPairSeparatedReusesExactClassifierSigns(t *testing.T) {
	crossingsByOrient := func(xt, xo [3]proof.Xpt, signs [3]int) []proof.Xpt {
		var points []proof.Xpt
		for i := range 3 {
			if signs[i] == 0 {
				points = append(points, xt[i])
			}
			j := (i + 1) % 3
			if signs[i]*signs[j] >= 0 {
				continue
			}
			ni, di := proof.OrientNum(xo[0], xo[1], xo[2], xt[i])
			nj, dj := proof.OrientNum(xo[0], xo[1], xo[2], xt[j])
			tn := new(big.Int).Mul(ni, dj)
			td := new(big.Int).Sub(tn, new(big.Int).Mul(nj, di))
			points = append(points, proof.Xlerp(xt[i], xt[j], tn, td))
		}
		return points
	}
	checkCrossings := func(t *testing.T, xt, xo [3]proof.Xpt, normal proof.Xpt, signs [3]int) {
		t.Helper()
		want := crossingsByOrient(xt, xo, signs)
		for _, got := range [][]proof.Xpt{
			planeCrossingsPrepared(xt, xo[0], normal, signs),
			PlaneCrossings(xt, xo, signs),
		} {
			require.Len(t, got, len(want))
			for i := range got {
				require.Equal(t, want[i].Key(), got[i].Key())
			}
		}
	}
	base := []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}, {X: 0, Y: 2, Z: 0}}
	cases := map[string]struct {
		other [3]r3.Vec
		tri   [3]int
	}{
		"crossing": {
			other: [3]r3.Vec{{X: 0.5, Y: 0.5, Z: -1}, {X: 0.5, Y: 0.5, Z: 1}, {X: 1, Y: 1, Z: 0}},
			tri:   [3]int{3, 4, 5},
		},
		"shared vertex": {
			other: [3]r3.Vec{{}, {X: 1, Y: 1, Z: 1}, {X: 1, Y: 1, Z: -1}},
			tri:   [3]int{0, 4, 5},
		},
		"coplanar overlap": {
			other: [3]r3.Vec{{X: 0.5, Y: 0.5, Z: 0}, {X: 1.5, Y: 0.5, Z: 0}, {X: 0.5, Y: 1.5, Z: 0}},
			tri:   [3]int{3, 4, 5},
		},
	}
	for name, test := range cases {
		t.Run(name, func(t *testing.T) {
			h := HeldRounding{Verts: append(append([]r3.Vec{}, base...), test.other[:]...),
				Tris: [][3]int{{0, 1, 2}, test.tri}}
			e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			var sharedA, sharedB []int
			for a, va := range h.Tris[0] {
				for b, vb := range h.Tris[1] {
					if va == vb {
						sharedA, sharedB = append(sharedA, a), append(sharedB, b)
					}
				}
			}
			_, decided, signs := e.pairSeparated(0, 1, sharedA, sharedB)
			require.False(t, decided)
			fa, xa, na := e.facet(0)
			fb, xb, nb := e.facet(1)
			checkCrossings(t, xa, xb, nb, signs.a)
			checkCrossings(t, xb, xa, na, signs.b)
			for _, factors := range [][6]int64{{1, 3, 5, 7, 11, 13}, {3, 3, 3, 3, 3, 3}, {3, 3, 3, 5, 5, 5}} {
				scale := func(p proof.Xpt, factor int64) proof.Xpt {
					f := big.NewInt(factor)
					return proof.Xpt{X: new(big.Int).Mul(p.X, f), Y: new(big.Int).Mul(p.Y, f),
						Z: new(big.Int).Mul(p.Z, f), W: new(big.Int).Mul(p.W, f)}
				}
				var sa, sb [3]proof.Xpt
				for i := range 3 {
					sa[i] = scale(xa[i], factors[i])
					sb[i] = scale(xb[i], factors[i+3])
				}
				checkCrossings(t, sa, sb, nb, signs.a)
				checkCrossings(t, sb, sa, na, signs.b)
			}
			for k := range 3 {
				require.Equal(t, proof.OrientSignPrepared(fb[0], fb[1], fb[2], fa[k], xb[0], xa[k], nb), signs.a[k])
				require.Equal(t, proof.OrientSignPrepared(fa[0], fa[1], fa[2], fb[k], xa[0], xb[k], na), signs.b[k])
			}
			got, gotErr := TriTriClassifyCore(fa, fb, xa, xb, na, nb, nil, nil,
				&signs.a, &signs.b, true, e.fnorm[0], e.fnorm[1], true)
			want, wantErr := TriTriClassifyPrepared(fa, fb, xa, xb, na, nb, e.fnorm[0], e.fnorm[1])
			require.Equal(t, wantErr, gotErr)
			require.Equal(t, want, got)
		})
	}
}
