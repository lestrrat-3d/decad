package facetproof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFirstMomentsMatchExactFacetFormula(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x57dfe12a, 0x4d75b3af))
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}}
	for range 50 {
		coordinate := func() float64 {
			return math.Ldexp(2*rng.Float64()-1, rng.IntN(80)-40)
		}
		verts = append(verts, r3.Vec{X: coordinate(), Y: coordinate(), Z: coordinate()})
	}
	xverts := make([]proof.Xpt, len(verts))
	for i, v := range verts {
		xverts[i] = proof.XptOf(v)
	}
	tris := [][3]int{{0, 1, 2}}
	for range 100 {
		a, b, c := rng.IntN(len(verts)), rng.IntN(len(verts)), rng.IntN(len(verts))
		if a != b && b != c && c != a {
			tris = append(tris, [3]int{a, b, c})
		}
	}
	want := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, tri := range tris {
		a, b, c := xverts[tri[0]], xverts[tri[1]], xverts[tri[2]]
		det := proof.XdotRat(a, proof.Xcross(b, c))
		ax, ay, az := proof.XhpRat(proof.Xhp(a))
		bx, by, bz := proof.XhpRat(proof.Xhp(b))
		cx, cy, cz := proof.XhpRat(proof.Xhp(c))
		for axis, coords := range [3][3]*big.Rat{{ax, bx, cx}, {ay, by, cy}, {az, bz, cz}} {
			sum := new(big.Rat).Add(coords[0], coords[1])
			sum.Add(sum, coords[2])
			want[axis].Add(want[axis], new(big.Rat).Mul(det, sum))
		}
	}
	got, err := facetproof.FirstMoments(proofbound.NewWorkBudget(t.Context()), xverts, tris)
	require.NoError(t, err)
	for axis := range got {
		require.Zero(t, got[axis].Cmp(want[axis]), "axis %d", axis)
	}
	scaled := append([]proof.Xpt(nil), xverts...)
	three := big.NewInt(3)
	p := scaled[3]
	scaled[3] = proof.Xpt{
		X: new(big.Int).Mul(p.X, three), Y: new(big.Int).Mul(p.Y, three),
		Z: new(big.Int).Mul(p.Z, three), W: new(big.Int).Mul(p.W, three),
	}
	got, err = facetproof.FirstMoments(proofbound.NewWorkBudget(t.Context()), scaled, tris)
	require.NoError(t, err)
	for axis := range got {
		require.Zero(t, got[axis].Cmp(want[axis]), "non-dyadic axis %d", axis)
	}
}
