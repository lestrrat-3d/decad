package proofbound_test

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// legacyTriangleParts is the per-triangle body the mesh functions used before
// they lifted each vertex once: HeldDelta lifts, the exact cross product, and
// the proven lengths.
func legacyTriangleParts(a, b, c r3.Vec, delta float64) (area, allow float64) {
	u, v := proofbound.HeldDelta(b, a), proofbound.HeldDelta(c, a)
	n := proofarith.DvCross(u, v)
	area = proofarith.DySqrtUp(proofarith.DyShift(proofarith.DvDot(n, n), -2))
	allow = proofbound.AbsSumUpper(
		proofbound.ProductUpper(delta, proofbound.AbsSumUpper(proofbound.DvLenUpper(u), proofbound.DvLenUpper(v))),
		proofbound.ProductUpper(2, proofbound.ProductUpper(delta, delta)),
	)
	return area, allow
}

func legacyTriangleTerm(a, b, c r3.Vec, delta float64) (term, allow float64) {
	if !proofbound.FiniteVec(a) || !proofbound.FiniteVec(b) || !proofbound.FiniteVec(c) {
		return math.Inf(1), math.Inf(1)
	}
	area, allow := legacyTriangleParts(a, b, c, delta)
	return proofbound.AbsSumUpper(area, allow), allow
}

func legacyAreaUpper(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	total := 0.0
	for _, t := range tris {
		term, _ := legacyTriangleTerm(verts[t[0]], verts[t[1]], verts[t[2]], delta)
		total += term
	}
	return proofbound.UpRound(total + proofbound.SumSlop(len(tris), total))
}

func randomMesh(r *rand.Rand, nVerts, nTris int) ([]r3.Vec, [][3]int) {
	coord := func() float64 {
		scale := math.Pow(10, float64(r.IntN(7)-3))
		return (r.Float64()*2 - 1) * scale
	}
	verts := make([]r3.Vec, nVerts)
	for i := range verts {
		verts[i] = r3.Vec{X: coord(), Y: coord(), Z: coord()}
	}
	tris := make([][3]int, nTris)
	for i := range tris {
		tris[i] = [3]int{r.IntN(nVerts), r.IntN(nVerts), r.IntN(nVerts)}
	}
	return verts, tris
}

func requireBits(t *testing.T, want, got float64, msg string) {
	t.Helper()
	require.Equal(t, math.Float64bits(want), math.Float64bits(got), msg)
}

func TestPerturbedAreaUpperMatchesPerTriangleForm(t *testing.T) {
	r := rand.New(rand.NewPCG(11, 12))
	for _, mesh := range []string{"finite", "nan-vertex", "unreferenced-inf"} {
		verts, tris := randomMesh(r, 120, 200)
		switch mesh {
		case "nan-vertex":
			verts[tris[3][1]] = r3.Vec{X: math.NaN()}
		case "unreferenced-inf":
			verts = append(verts, r3.Vec{Y: math.Inf(1)})
		}
		const d1, d2 = 0.0, 3e-4
		t.Run(mesh, func(t *testing.T) {
			want1, want2 := legacyAreaUpper(verts, tris, d1), legacyAreaUpper(verts, tris, d2)

			got1, err := proofbound.PerturbedAreaUpperWithBudget(nil, verts, tris, d1)
			require.NoError(t, err)
			requireBits(t, want1, got1, "budget d1")
			got2, err := proofbound.PerturbedAreaUpperContext(t.Context(), verts, tris, d2)
			require.NoError(t, err)
			requireBits(t, want2, got2, "context d2")

			p1, p2, err := proofbound.PerturbedAreaUpperPairContext(t.Context(), verts, tris, d1, d2)
			require.NoError(t, err)
			requireBits(t, want1, p1, "pair d1")
			requireBits(t, want2, p2, "pair d2")

			total, allows, err := proofbound.PerturbedAreaUpperAllowsContext(t.Context(), verts, tris, d2)
			require.NoError(t, err)
			requireBits(t, want2, total, "allows total")
			require.Len(t, allows, len(tris))
			for i, tri := range tris {
				_, allow := legacyTriangleTerm(verts[tri[0]], verts[tri[1]], verts[tri[2]], d2)
				requireBits(t, allow, allows[i], "allow")
			}

			switch mesh {
			case "unreferenced-inf":
				require.False(t, math.IsInf(want1, 1))
			case "nan-vertex":
				require.True(t, math.IsInf(want1, 1))
			}
		})
	}
}

func TestPerturbedAreaUpperHonoursCancellation(t *testing.T) {
	verts, tris := randomMesh(rand.New(rand.NewPCG(13, 14)), 30, 3*proofbound.WorkPollInterval)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := proofbound.PerturbedAreaUpperContext(ctx, verts, tris, 0)
	require.Error(t, err)
	_, _, err = proofbound.PerturbedAreaUpperPairContext(ctx, verts, tris, 0, 1e-3)
	require.Error(t, err)
	_, _, err = proofbound.PerturbedAreaUpperAllowsContext(ctx, verts, tris, 1e-3)
	require.Error(t, err)
}
