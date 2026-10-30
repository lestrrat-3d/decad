package revolvemesh

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// sweepMesh is a vertex list, its triangles, and the audit data built from
// them.
type sweepMesh struct {
	verts []r3.Vec
	tris  [][3]int
	data  []RevolveAuditTri
}

func newSweepMesh(t *testing.T, verts []r3.Vec, tris [][3]int) sweepMesh {
	t.Helper()
	m := sweepMesh{verts: verts}
	for _, tri := range tris {
		d, ok := NewRevolveAuditTri(verts, tri)
		if !ok {
			continue
		}
		m.tris = append(m.tris, tri)
		m.data = append(m.data, d)
	}
	require.NotEmpty(t, m.tris)
	return m
}

// revolveLikeMesh is a closed solid of revolution around the X axis: 5 rings
// of 24 points, a planar pole fan at each end.
func revolveLikeMesh(t *testing.T) sweepMesh {
	t.Helper()
	const steps, rings = 24, 5
	var verts []r3.Vec
	for k := range rings {
		for s := range steps {
			a := 2 * math.Pi * float64(s) / steps
			verts = append(verts, r3.Vec{X: float64(k), Y: 2 * math.Cos(a), Z: 2 * math.Sin(a)})
		}
	}
	poleA, poleB := len(verts), len(verts)+1
	verts = append(verts, r3.Vec{X: 0}, r3.Vec{X: rings - 1})
	var tris [][3]int
	for s := range steps {
		n := (s + 1) % steps
		tris = append(tris, [3]int{poleA, n, s})
		last := (rings - 1) * steps
		tris = append(tris, [3]int{poleB, last + s, last + n})
	}
	for k := range rings - 1 {
		for s := range steps {
			n := (s + 1) % steps
			a, b, c, d := k*steps+s, k*steps+n, (k+1)*steps+n, (k+1)*steps+s
			tris = append(tris, [3]int{a, b, c}, [3]int{a, c, d})
		}
	}
	return newSweepMesh(t, verts, tris)
}

// soupMesh is 200 random triangles; some reuse earlier vertices, a few are
// flat on one axis.
func soupMesh(t *testing.T) sweepMesh {
	t.Helper()
	rng := rand.New(rand.NewPCG(1, 2))
	coord := func() float64 { return rng.Float64()*20 - 10 }
	var verts []r3.Vec
	var tris [][3]int
	add := func(v r3.Vec) int {
		verts = append(verts, v)
		return len(verts) - 1
	}
	near := func(p r3.Vec) r3.Vec {
		l := 1e-3 + rng.Float64()*5
		return r3.Vec{X: p.X + (rng.Float64()-0.5)*l, Y: p.Y + (rng.Float64()-0.5)*l, Z: p.Z + (rng.Float64()-0.5)*l}
	}
	for n := range 200 {
		var tri [3]int
		if n > 10 && n%3 == 0 {
			// Share one or two corners with an earlier triangle.
			prev := tris[rng.IntN(len(tris))]
			tri[0] = prev[rng.IntN(3)]
			tri[1] = add(near(verts[tri[0]]))
			tri[2] = prev[rng.IntN(3)]
			if n%6 == 0 || tri[2] == tri[0] {
				tri[2] = add(near(verts[tri[0]]))
			}
		} else {
			p := r3.Vec{X: coord(), Y: coord(), Z: coord()}
			tri = [3]int{add(p), add(near(p)), add(near(p))}
		}
		if n%25 == 0 {
			flat := verts[tri[0]].Z
			for _, v := range tri {
				verts[v].Z = flat
			}
		}
		tris = append(tris, tri)
	}
	return newSweepMesh(t, verts, tris)
}

func crossingMesh(t *testing.T) sweepMesh {
	t.Helper()
	m := revolveLikeMesh(t)
	base := len(m.verts)
	m.verts = append(m.verts,
		r3.Vec{X: 1.5, Y: -0.5, Z: -0.5}, r3.Vec{X: 1.5, Y: 0.5, Z: -0.5}, r3.Vec{X: 1.5, Y: 0, Z: 0.5},
		r3.Vec{X: 1.2, Y: 0, Z: 0}, r3.Vec{X: 1.8, Y: 0, Z: 0}, r3.Vec{X: 1.5, Y: 0, Z: -0.3},
	)
	return newSweepMesh(t, m.verts, append(append([][3]int{}, m.tris...), [3]int{base, base + 1, base + 2},
		[3]int{base + 3, base + 4, base + 5}))
}

func sweepMeshes(t *testing.T) map[string]sweepMesh {
	return map[string]sweepMesh{"revolve": revolveLikeMesh(t), "soup": soupMesh(t), "crossing": crossingMesh(t)}
}

var sweepDeltas = []float64{0, 1e-9, 0.25}

func TestRevolveBoundedContactAuditMatchesExisting(t *testing.T) {
	for name, mesh := range sweepMeshes(t) {
		for _, delta := range sweepDeltas {
			oldErr := RevolveContactAudit(proofbound.NewWorkBudget(t.Context()), mesh.data, mesh.tris, delta)
			newErr := RevolveContactAuditBounded(proofbound.NewWorkBudget(t.Context()), mesh.data, mesh.tris, delta)
			require.Equal(t, oldErr == nil, newErr == nil, "%s delta %g", name, delta)
		}
	}
}

func TestRevolveContactCandidatesMatchBruteForce(t *testing.T) {
	for name, m := range sweepMeshes(t) {
		for _, delta := range sweepDeltas {
			margin := proofbound.ProductUpper(2, delta)
			rows, err := revolveContactCandidates(proofbound.NewWorkBudget(t.Context()), m.data, m.tris, margin)
			require.NoError(t, err, name)
			var want, got [][2]int
			for i := range m.tris {
				for j := i + 1; j < len(m.tris); j++ {
					_, count := tessellation.SharedVertexIndices(m.tris[i], m.tris[j])
					if count > 0 || !BoxGapExceeds(m.data[i].Box, m.data[j].Box, margin) {
						want = append(want, [2]int{i, j})
					}
				}
			}
			for i, row := range rows {
				for k, j := range row {
					require.Greater(t, int(j), i, name)
					if k > 0 {
						require.Greater(t, j, row[k-1], name)
					}
					_, count := tessellation.SharedVertexIndices(m.tris[i], m.tris[j])
					if count == 0 && BoxGapExceeds(m.data[i].Box, m.data[j].Box, margin) {
						continue
					}
					got = append(got, [2]int{i, int(j)})
				}
			}
			require.Equal(t, want, got, "%s delta %v", name, delta)
		}
	}
}

func TestRevolveVertexIsolatedCachedMatchesUncached(t *testing.T) {
	for name, m := range sweepMeshes(t) {
		for _, delta := range sweepDeltas {
			cache := newRevolveAxisCache(len(m.tris))
			for i := range m.tris {
				for j := i + 1; j < len(m.tris); j++ {
					shared, count := tessellation.SharedVertexIndices(m.tris[i], m.tris[j])
					if count != 1 {
						continue
					}
					a, b := m.data[i], m.data[j]
					require.Equal(t, RevolveVertexIsolated(a, m.tris[i], b, m.tris[j], shared[0], delta),
						revolveVertexIsolated(a, m.tris[i], b, m.tris[j], shared[0], delta, cache, i), "%s %d,%d", name, i, j)
					require.Equal(t, RevolveVertexIsolated(b, m.tris[j], a, m.tris[i], shared[0], delta),
						revolveVertexIsolated(b, m.tris[j], a, m.tris[i], shared[0], delta, cache, j), "%s %d,%d", name, j, i)
				}
			}
		}
	}
}

// allPairsAudit is the all-pairs walk the sweep replaces.
func allPairsAudit(data []RevolveAuditTri, tris [][3]int, delta float64) error {
	margin := proofbound.ProductUpper(2, delta)
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
			if count == 0 && BoxGapExceeds(data[i].Box, data[j].Box, margin) {
				continue
			}
			if err := AuditRevolvePair(data, tris, shared, count, i, j, delta); err != nil {
				return err
			}
		}
	}
	return nil
}

func TestRevolveContactAuditMatchesAllPairs(t *testing.T) {
	for name, m := range sweepMeshes(t) {
		for _, delta := range sweepDeltas {
			want := allPairsAudit(m.data, m.tris, delta)
			got := RevolveContactAudit(proofbound.NewWorkBudget(t.Context()), m.data, m.tris, delta)
			if want == nil {
				require.NoError(t, got, "%s delta %v", name, delta)
				continue
			}
			require.Error(t, got, "%s delta %v", name, delta)
			require.Equal(t, want.Error(), got.Error(), "%s delta %v", name, delta)
		}
	}
}

// TestRevolveAuditRowsParallelMatchesSerial runs the parallel audit on the
// sweep meshes, including the ones whose audit fails, and compares the
// outcome and the number of budget steps with the serial walk.
func TestRevolveAuditRowsParallelMatchesSerial(t *testing.T) {
	failed := 0
	for name, m := range sweepMeshes(t) {
		for _, delta := range sweepDeltas {
			margin := proofbound.ProductUpper(2, delta)
			rows, err := revolveContactCandidates(proofbound.NewWorkBudget(t.Context()), m.data, m.tris, margin)
			require.NoError(t, err, name)
			run := func(parallel bool) (int, error) {
				budget := proofbound.NewWorkBudget(t.Context())
				steps := 0
				inner := budget.StepFn
				budget.StepFn = func() error { steps++; return inner() }
				err := revolveAuditRows(budget, m.data, m.tris, rows, delta, margin, parallel)
				return steps, err
			}
			wantSteps, want := run(false)
			gotSteps, got := run(true)
			if want == nil {
				require.NoError(t, got, "%s delta %v", name, delta)
			} else {
				failed++
				require.Error(t, got, "%s delta %v", name, delta)
				require.Equal(t, want.Error(), got.Error(), "%s delta %v", name, delta)
			}
			require.Equal(t, wantSteps, gotSteps, "%s delta %v", name, delta)
		}
	}
	require.NotZero(t, failed, "no sweep mesh exercises the error path")
}

// TestRevolveAuditRowsParallelCancelled checks that a cancelled context gives
// the parallel audit the outcome the serial walk has.
func TestRevolveAuditRowsParallelCancelled(t *testing.T) {
	m := soupMesh(t)
	margin := proofbound.ProductUpper(2, 0.25)
	rows, err := revolveContactCandidates(proofbound.NewWorkBudget(t.Context()), m.data, m.tris, margin)
	require.NoError(t, err)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	serialBudget := proofbound.NewWorkBudget(ctx)
	wantSteps := 0
	serialStep := serialBudget.StepFn
	serialBudget.StepFn = func() error { wantSteps++; return serialStep() }
	want := revolveAuditRows(serialBudget, m.data, m.tris, rows, 0.25, margin, false)
	got := revolveAuditRows(proofbound.NewWorkBudget(ctx), m.data, m.tris, rows, 0.25, margin, true)
	require.Error(t, want)
	require.ErrorContains(t, want, "revolve facets 0 and 60")
	require.Equal(t, want.Error(), got.Error())

	// Hold the workers until the caller observes cancellation. The serial
	// walk must still reach its earlier facet error before its first poll.
	release := make(chan struct{})
	poll := make(chan time.Time, 1)
	poll <- time.Time{}
	budget := proofbound.NewWorkBudget(ctx)
	gotSteps := 0
	parallelStep := budget.StepFn
	budget.StepFn = func() error { gotSteps++; return parallelStep() }
	budget.ErrFn = func() error {
		close(release)
		return ctx.Err()
	}
	got = revolveAuditRowsParallelWithPoll(budget, m.data, m.tris, rows, 0.25, margin, poll,
		func() { <-release })
	require.Error(t, got)
	require.Equal(t, want.Error(), got.Error())
	require.Equal(t, wantSteps, gotSteps)
}
