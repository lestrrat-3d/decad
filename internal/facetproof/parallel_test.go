package facetproof_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// uvSphere is a closed outward (or, with flip, inward) sphere of
// slices*(2*stacks-2) facets, rotated about (1, 2, 3) by 0.3 rad and moved to
// centre, with float-rounded vertices.
func uvSphere(t *testing.T, centre r3.Vec, radius float64, stacks, slices int, flip bool) ([]r3.Vec, [][3]int) {
	t.Helper()
	rot, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(0.3))
	require.NoError(t, err)
	place := func(v r3.Vec) r3.Vec {
		return rot.Apply(v).Add(centre)
	}
	verts := []r3.Vec{place(r3.Vec{Z: radius})}
	for i := 1; i < stacks; i++ {
		phi := math.Pi * float64(i) / float64(stacks)
		for j := range slices {
			th := 2 * math.Pi * float64(j) / float64(slices)
			verts = append(verts, place(r3.Vec{
				X: radius * math.Sin(phi) * math.Cos(th), Y: radius * math.Sin(phi) * math.Sin(th),
				Z: radius * math.Cos(phi)}))
		}
	}
	verts = append(verts, place(r3.Vec{Z: -radius}))
	ring := func(i, j int) int { return 1 + (i-1)*slices + j%slices }
	south := len(verts) - 1
	var tris [][3]int
	for j := range slices {
		tris = append(tris, [3]int{0, ring(1, j), ring(1, j+1)})
	}
	for i := 1; i < stacks-1; i++ {
		for j := range slices {
			a, b, c, d := ring(i, j), ring(i, j+1), ring(i+1, j+1), ring(i+1, j)
			tris = append(tris, [3]int{a, d, c}, [3]int{a, c, b})
		}
	}
	for j := range slices {
		tris = append(tris, [3]int{south, ring(stacks-1, j+1), ring(stacks-1, j)})
	}
	if flip {
		for i, tr := range tris {
			tris[i] = [3]int{tr[0], tr[2], tr[1]}
		}
	}
	return verts, tris
}

func joinMeshes(av []r3.Vec, at [][3]int, bv []r3.Vec, bt [][3]int) ([]r3.Vec, [][3]int) {
	verts := append(append([]r3.Vec{}, av...), bv...)
	tris := append([][3]int{}, at...)
	for _, tr := range bt {
		tris = append(tris, [3]int{tr[0] + len(av), tr[1] + len(av), tr[2] + len(av)})
	}
	return verts, tris
}

type auditCase struct {
	name  string
	verts []r3.Vec
	tris  [][3]int
}

func auditCases(t *testing.T) []auditCase {
	t.Helper()
	v1, t1 := uvSphere(t, r3.Vec{}, 10, 40, 32, false)
	v2, t2 := uvSphere(t, r3.Vec{X: 40}, 7, 40, 32, false)
	vi, ti := uvSphere(t, r3.Vec{}, 3, 12, 12, true)
	vo, to := uvSphere(t, r3.Vec{}, 10, 40, 32, false)
	twoV, twoT := joinMeshes(v1, t1, v2, t2)
	voidV, voidT := joinMeshes(vo, to, vi, ti)
	return []auditCase{
		{"sphere", v1, t1},
		{"two spheres", twoV, twoT},
		{"void", voidV, voidT},
	}
}

func requireAuditsEqual(t *testing.T, want, got *facetproof.MeshAudit) {
	t.Helper()
	require.Equal(t, want.Comp, got.Comp)
	require.Equal(t, want.Adj, got.Adj)
	require.Equal(t, want.Members, got.Members)
	require.Equal(t, want.Contains, got.Contains)
	require.Len(t, got.CompVol, len(want.CompVol))
	for i := range want.CompVol {
		require.Zero(t, want.CompVol[i].Cmp(got.CompVol[i]))
	}
	require.Equal(t, want.Det == nil, got.Det == nil)
	require.Len(t, got.Det, len(want.Det))
	for i := range want.Det {
		require.Zero(t, proof.DyCmp(want.Det[i], got.Det[i]))
		require.Equal(t, want.Det[i].Exp(), got.Det[i].Exp())
	}
	require.Len(t, got.XVerts, len(want.XVerts))
	for i := range want.XVerts {
		w, g := want.XVerts[i], got.XVerts[i]
		require.Zero(t, w.X.Cmp(g.X))
		require.Zero(t, w.Y.Cmp(g.Y))
		require.Zero(t, w.Z.Cmp(g.Z))
		require.Zero(t, w.W.Cmp(g.W))
	}
}

func TestAuditFacetedMeshParallelMatchesSerial(t *testing.T) {
	t.Parallel()
	for _, c := range auditCases(t) {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			want, err := facetproof.AuditFacetedMeshWorkers(t.Context(), c.verts, c.tris, 1)
			require.NoError(t, err)
			require.NotNil(t, want.Det)
			got, err := facetproof.AuditFacetedMeshWorkers(t.Context(), c.verts, c.tris, 4)
			require.NoError(t, err)
			requireAuditsEqual(t, want, got)
		})
	}
}

func TestAuditFacetedMeshParallelKeepsErrors(t *testing.T) {
	t.Parallel()
	verts, tris := uvSphere(t, r3.Vec{}, 10, 40, 32, false)
	t.Run("open mesh", func(t *testing.T) {
		t.Parallel()
		open := tris[1:]
		_, want := facetproof.AuditFacetedMeshWorkers(t.Context(), verts, open, 1)
		_, got := facetproof.AuditFacetedMeshWorkers(t.Context(), verts, open, 4)
		require.Error(t, want)
		require.EqualError(t, got, want.Error())
	})
	t.Run("inverted mesh", func(t *testing.T) {
		t.Parallel()
		flip := make([][3]int, len(tris))
		for i, tr := range tris {
			flip[i] = [3]int{tr[0], tr[2], tr[1]}
		}
		_, want := facetproof.AuditFacetedMeshWorkers(t.Context(), verts, flip, 1)
		_, got := facetproof.AuditFacetedMeshWorkers(t.Context(), verts, flip, 4)
		require.Error(t, want)
		require.EqualError(t, got, want.Error())
	})
	t.Run("cancelled", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, want := facetproof.AuditFacetedMeshWorkers(ctx, verts, tris, 1)
		_, got := facetproof.AuditFacetedMeshWorkers(ctx, verts, tris, 4)
		require.ErrorIs(t, want, context.Canceled)
		require.ErrorIs(t, got, context.Canceled)
	})
	t.Run("NaN vertex takes the serial path", func(t *testing.T) {
		t.Parallel()
		bad := append([]r3.Vec{}, verts...)
		bad[7].Y = math.NaN()
		require.Panics(t, func() { _, _ = facetproof.AuditFacetedMeshWorkers(t.Context(), bad, tris, 1) })
		require.Panics(t, func() { _, _ = facetproof.AuditFacetedMeshWorkers(t.Context(), bad, tris, 4) })
	})
}

func countingBudget(steps *int) *proofbound.WorkBudget {
	return &proofbound.WorkBudget{
		StepFn: func() error { *steps++; return nil },
		ErrFn:  func() error { return nil },
	}
}

func TestFirstMomentsFromAuditChecksExhaustionBeforeWork(t *testing.T) {
	verts, tris := uvSphere(t, r3.Vec{}, 1, 4, 8, false)
	audit, err := facetproof.AuditFacetedMeshWorkers(t.Context(), verts, tris, 1)
	require.NoError(t, err)
	for _, workers := range []int{1, 4} {
		wall := proofbound.NewWallWorkBudget(0)
		polled, steps := false, 0
		budget := &proofbound.WorkBudget{
			ErrFn: func() error {
				polled = true
				return wall.Err()
			},
			StepFn: func() error {
				steps++
				if !polled {
					return errors.New("moment work began before checking the exhausted budget")
				}
				return wall.Step()
			},
		}
		_, err := facetproof.FirstMomentsFromAuditWorkers(budget, audit, tris, workers)
		require.ErrorIs(t, err, proofbound.ErrWallWorkBudget, "workers %d", workers)
		require.Equal(t, 1, steps, "workers %d", workers)
		var limit *proofbound.WorkLimitError
		require.ErrorAs(t, err, &limit)
		require.Equal(t, "proof_steps", limit.Resource)
		require.Equal(t, uint64(1), limit.Observed)
		require.Zero(t, limit.Maximum)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	for _, workers := range []int{1, 4} {
		_, err := facetproof.FirstMomentsFromAuditWorkers(proofbound.NewWorkBudget(ctx), audit, tris, workers)
		require.ErrorIs(t, err, context.Canceled, "workers %d", workers)
	}
}

func TestFirstMomentsFromAuditParallelMatchesSerial(t *testing.T) {
	t.Parallel()
	for _, c := range auditCases(t) {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			audit, err := facetproof.AuditFacetedMeshWorkers(t.Context(), c.verts, c.tris, 1)
			require.NoError(t, err)
			require.NotNil(t, audit.Det)
			var wantSteps, gotSteps int
			want, err := facetproof.FirstMomentsFromAuditWorkers(countingBudget(&wantSteps), audit, c.tris, 1)
			require.NoError(t, err)
			got, err := facetproof.FirstMomentsFromAuditWorkers(countingBudget(&gotSteps), audit, c.tris, 4)
			require.NoError(t, err)
			for axis := range want {
				require.Zero(t, want[axis].Cmp(got[axis]), "axis %d", axis)
				require.Equal(t, want[axis].Num(), got[axis].Num())
				require.Equal(t, want[axis].Denom(), got[axis].Denom())
			}
			require.Equal(t, len(c.tris), wantSteps)
			require.Equal(t, wantSteps, gotSteps)
			for _, k := range []uint64{0, 1, 600, uint64(len(c.tris))} {
				_, wantErr := facetproof.FirstMomentsFromAuditWorkers(proofbound.NewWallWorkBudget(k), audit, c.tris, 1)
				_, gotErr := facetproof.FirstMomentsFromAuditWorkers(proofbound.NewWallWorkBudget(k), audit, c.tris, 4)
				require.Equal(t, wantErr, gotErr, "limit %d", k)
				if k < uint64(len(c.tris)) {
					require.ErrorIs(t, gotErr, proofbound.ErrWallWorkBudget)
				}
			}
		})
	}
}

func TestPlaceMeshParallelMatchesSerial(t *testing.T) {
	t.Parallel()
	verts, tris := uvSphere(t, r3.Vec{}, 10, 40, 32, false)
	rot, err := r3.Rotation(r3.Vec{X: 0.3, Y: 1, Z: -0.2}, units.Degrees(37))
	require.NoError(t, err)
	fr, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	mirror, err := r3.Reflection(fr)
	require.NoError(t, err)
	reflected, err := rot.Then(mirror)
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 1.5, Y: -2.25, Z: 0.125})
	require.NoError(t, err)
	huge, err := r3.Translation(r3.Vec{X: 1e308})
	require.NoError(t, err)
	rigid, err := shift.Then(rot)
	require.NoError(t, err)
	mirrored, err := shift.Then(reflected)
	require.NoError(t, err)
	far := append([]r3.Vec{}, verts...)
	far[700] = r3.Vec{X: 1e308}
	cases := []struct {
		name  string
		verts []r3.Vec
		xf    r3.Transform
	}{
		{"rigid", verts, rigid},
		{"reflection", verts, mirrored},
		{"overflow", far, huge},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			run := func(workers int) (facetproof.PlacedMesh, int, error) {
				steps := 0
				got, err := facetproof.PlaceMeshWorkers(t.Context(), countingBudget(&steps), c.verts, tris,
					make([]float64, len(c.verts)), 0, 0, 0, 0, c.xf, workers)
				return got, steps, err
			}
			want, wantSteps, wantErr := run(1)
			got, gotSteps, gotErr := run(4)
			require.Equal(t, wantSteps, gotSteps)
			if wantErr != nil {
				require.EqualError(t, gotErr, wantErr.Error())
				return
			}
			require.NoError(t, gotErr)
			require.Equal(t, want.Tris, got.Tris)
			require.Len(t, got.Verts, len(want.Verts))
			for i := range want.Verts {
				require.Equal(t, math.Float64bits(want.Verts[i].X), math.Float64bits(got.Verts[i].X))
				require.Equal(t, math.Float64bits(want.Verts[i].Y), math.Float64bits(got.Verts[i].Y))
				require.Equal(t, math.Float64bits(want.Verts[i].Z), math.Float64bits(got.Verts[i].Z))
				require.Equal(t, math.Float64bits(want.VertexBound[i]), math.Float64bits(got.VertexBound[i]))
			}
			for _, p := range [][2]float64{{want.MeshBound, got.MeshBound}, {want.VolSymDiff, got.VolSymDiff},
				{want.AreaSlack, got.AreaSlack}} {
				require.Equal(t, math.Float64bits(p[0]), math.Float64bits(p[1]))
			}
		})
	}
	t.Run("overflow refuses", func(t *testing.T) {
		t.Parallel()
		_, err := facetproof.PlaceMeshWorkers(t.Context(), proofbound.NewWorkBudget(t.Context()), far, tris,
			make([]float64, len(far)), 0, 0, 0, 0, huge, 4)
		require.ErrorContains(t, err, "not representable")
	})
}
