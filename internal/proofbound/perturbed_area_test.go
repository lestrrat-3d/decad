package proofbound_test

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"strings"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// legacyTriangleParts is the per-triangle body the mesh functions used before
// they lifted each vertex once: HeldDelta lifts, the exact cross product, and
// the proven lengths.
// meshUnreferencedInf names the mesh variant with an unreferenced +Inf vertex.
const meshUnreferencedInf = "unreferenced-inf"

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
	for _, mesh := range []string{
		"finite", "nan-vertex", meshUnreferencedInf,
		"large-finite", "large-nan-vertex", "large-unreferenced-inf",
	} {
		verts, tris := randomMesh(r, 120, 200)
		if strings.HasPrefix(mesh, "large-") {
			verts, tris = randomMesh(r, 1500, 3000)
			mesh = strings.TrimPrefix(mesh, "large-")
		}
		switch mesh {
		case "nan-vertex":
			verts[tris[3][1]] = r3.Vec{X: math.NaN()}
		case meshUnreferencedInf:
			verts = append(verts, r3.Vec{Y: math.Inf(1)})
		}
		const d1, d2 = 0.0, 3e-4
		t.Run(fmt.Sprintf("%s/%d", mesh, len(tris)), func(t *testing.T) {
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
			case meshUnreferencedInf:
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

	verts, tris = randomMesh(rand.New(rand.NewPCG(15, 16)), 900, 3000)
	_, err = proofbound.PerturbedAreaUpperContext(ctx, verts, tris, 0)
	require.Error(t, err)
	_, _, err = proofbound.PerturbedAreaUpperPairContext(ctx, verts, tris, 0, 1e-3)
	require.Error(t, err)
	_, _, err = proofbound.PerturbedAreaUpperAllowsContext(ctx, verts, tris, 1e-3)
	require.Error(t, err)
}

func TestPerturbedTriangleTermsParallelMatchesSerial(t *testing.T) {
	r := rand.New(rand.NewPCG(21, 22))
	build := func(name string) ([]r3.Vec, [][3]int) {
		verts, tris := randomMesh(r, 1500, 3000)
		switch name {
		case "nan-corner":
			verts[tris[1700][2]] = r3.Vec{Z: math.NaN()}
		case meshUnreferencedInf:
			verts = append(verts, r3.Vec{X: math.Inf(1)})
		case "repeated":
			for i := range 50 {
				tris[2000+i] = tris[7]
			}
		}
		return verts, tris
	}
	for _, name := range []string{"finite", "nan-corner", meshUnreferencedInf, "repeated"} {
		t.Run(name, func(t *testing.T) {
			verts, tris := build(name)
			wantV, wantOK, err := proofbound.PerturbedTriangleTermsForTest(nil, verts, tris, 1)
			require.NoError(t, err)
			gotV, gotOK, err := proofbound.PerturbedTriangleTermsForTest(nil, verts, tris, 4)
			require.NoError(t, err)
			require.Equal(t, wantOK, gotOK)
			for i := range wantV {
				for k := range wantV[i] {
					requireBits(t, wantV[i][k], gotV[i][k], "term")
				}
			}
			if name == "nan-corner" {
				require.Contains(t, wantOK, false)
			}
		})
	}
}

func TestPerturbedTriangleTermsBudget(t *testing.T) {
	verts, tris := randomMesh(rand.New(rand.NewPCG(23, 24)), 1500, 3000)
	count := func(workers int) int {
		steps := 0
		budget := &proofbound.WorkBudget{
			StepFn: func() error { steps++; return nil },
			ErrFn:  func() error { return nil },
		}
		_, _, err := proofbound.PerturbedTriangleTermsForTest(budget, verts, tris, workers)
		require.NoError(t, err)
		return steps
	}
	require.Equal(t, len(tris), count(1))
	require.Equal(t, len(tris), count(4))

	for _, limit := range []uint64{0, 1, 255, 1000, 2999, 3000} {
		_, _, wantErr := proofbound.PerturbedTriangleTermsForTest(
			proofbound.NewWallWorkBudget(limit), verts, tris, 1)
		_, _, gotErr := proofbound.PerturbedTriangleTermsForTest(
			proofbound.NewWallWorkBudget(limit), verts, tris, 4)
		require.Equal(t, wantErr == nil, gotErr == nil, "limit %d", limit)
		if limit < uint64(len(tris)) {
			require.ErrorIs(t, gotErr, proofbound.ErrWallWorkBudget)
			require.ErrorIs(t, wantErr, proofbound.ErrWallWorkBudget)
			require.Equal(t, wantErr.Error(), gotErr.Error(), "limit %d", limit)
			var wantLimit, gotLimit *proofbound.WorkLimitError
			require.True(t, errors.As(wantErr, &wantLimit))
			require.True(t, errors.As(gotErr, &gotLimit))
			require.Equal(t, "proof_steps", wantLimit.Resource)
			require.Equal(t, wantLimit.Resource, gotLimit.Resource)
			require.Equal(t, limit, wantLimit.Maximum)
			require.Equal(t, wantLimit.Maximum, gotLimit.Maximum)
			require.True(t, wantLimit.Observed == limit || wantLimit.Observed == limit+1)
			require.True(t, gotLimit.Observed == limit || gotLimit.Observed == limit+1)
		} else {
			require.NoError(t, gotErr)
		}
	}
}
