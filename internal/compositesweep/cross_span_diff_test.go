package compositesweep_test

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// spanMesh is a triangle set with each triangle's span and its audit data.
type spanMesh struct {
	verts  []r3.Vec
	tris   [][3]int
	spanOf []int
	data   []revolvemesh.RevolveAuditTri
}

func newSpanMesh(t testing.TB, verts []r3.Vec, tris [][3]int, spanOf []int) spanMesh {
	t.Helper()
	m := spanMesh{verts: verts, tris: tris, spanOf: spanOf, data: make([]revolvemesh.RevolveAuditTri, len(tris))}
	for i, tri := range tris {
		var ok bool
		m.data[i], ok = revolvemesh.NewRevolveAuditTri(verts, tri)
		require.True(t, ok)
	}
	return m
}

// twistedPrismMesh is the closed mesh of a twisted sweep held as cells: a
// rectangle of half sides hx, hy turned by angle
// radians over height, one ring of corners per station, two wall triangles
// per side per cell, and a fan on each cap. A wall's span is its cell; a cap
// belongs to the cell it closes, as CompositeTwistMesh records them.
func twistedPrismMesh(t testing.TB, hx, hy, angle, height float64, cells int) spanMesh {
	t.Helper()
	corners := [][2]float64{{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}}
	n := len(corners)
	var verts []r3.Vec
	for s := range cells + 1 {
		sin, cos := math.Sincos(angle * float64(s) / float64(cells))
		z := height * float64(s) / float64(cells)
		for _, c := range corners {
			verts = append(verts, r3.NewVec(c[0]*cos-c[1]*sin, c[0]*sin+c[1]*cos, z))
		}
	}
	var tris [][3]int
	var spanOf []int
	for c := range cells {
		for j := range n {
			a, b := c*n+j, c*n+(j+1)%n
			tris = append(tris, [3]int{a, b, b + n}, [3]int{a, b + n, a + n})
			spanOf = append(spanOf, c, c)
		}
	}
	top := cells * n
	for j := 1; j+1 < n; j++ {
		tris = append(tris, [3]int{0, j + 1, j}, [3]int{top, top + j, top + j + 1})
		spanOf = append(spanOf, 0, cells-1)
	}
	return newSpanMesh(t, verts, tris, spanOf)
}

// randomSpanMesh is a soup of small triangles in the unit cube, spread over
// four spans, that shares vertices between neighbouring triangles. Its
// crossings and contacts make the audit refuse at some first pair.
func randomSpanMesh(t testing.TB, rng *rand.Rand, count int, size float64) spanMesh {
	t.Helper()
	var verts []r3.Vec
	tris := make([][3]int, count)
	spanOf := make([]int, count)
	for i := range tris {
		base := r3.NewVec(rng.Float64(), rng.Float64(), rng.Float64())
		for k := range 3 {
			if k == 0 && i > 0 && rng.IntN(3) == 0 {
				tris[i][k] = tris[i-1][rng.IntN(3)]
				continue
			}
			verts = append(verts, base.Add(r3.NewVec(size*rng.Float64(), size*rng.Float64(), size*rng.Float64())))
			tris[i][k] = len(verts) - 1
		}
		spanOf[i] = rng.IntN(4)
	}
	return newSpanMesh(t, verts, tris, spanOf)
}

func requireSameCrossSpanVerdict(t *testing.T, label string, m spanMesh, delta float64) {
	t.Helper()
	want := legacyCrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()), m.data, m.tris, m.spanOf, delta)
	got := compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()), m.data, m.tris, m.spanOf, delta)
	if want == nil {
		require.NoError(t, got, label)
		return
	}
	require.Error(t, got, label)
	require.Equal(t, want.Error(), got.Error(), label)
}

// TestCrossSpanContactAuditMatchesAllPairs requires the swept audit to return
// the all-pairs walk's verdict, error text included, over twisted cell meshes
// of several turns, cell counts and displacements, over the same meshes with
// one station corner pushed through a neighbouring wall, and over random
// soups that refuse at different first pairs.
func TestCrossSpanContactAuditMatchesAllPairs(t *testing.T) {
	refused := 0
	for _, deg := range []float64{30, 87, 120, -75, 200} {
		for _, cells := range []int{2, 15, 81} {
			for _, delta := range []float64{0, 1e-9, 1e-3, 0.05, 0.4, math.NaN()} {
				label := fmt.Sprintf("deg=%v cells=%d delta=%v", deg, cells, delta)
				m := twistedPrismMesh(t, 7.7, 2.375, deg*math.Pi/180, 6, cells)
				requireSameCrossSpanVerdict(t, label, m, delta)

				// Push one corner of the middle station inward across the
				// opposite wall, so later cells cross earlier ones.
				dented := twistedPrismMesh(t, 7.7, 2.375, deg*math.Pi/180, 6, cells)
				corner := (cells / 2) * 4
				dented.verts[corner] = dented.verts[corner].Scale(-0.6)
				dented = newSpanMesh(t, dented.verts, dented.tris, dented.spanOf)
				requireSameCrossSpanVerdict(t, label+" dented", dented, delta)
				if compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
					dented.data, dented.tris, dented.spanOf, delta) != nil {
					refused++
				}
			}
		}
	}
	rng := rand.New(rand.NewPCG(7, 11))
	for trial := range 40 {
		m := randomSpanMesh(t, rng, 20+10*trial, 0.02+0.004*float64(trial))
		for _, delta := range []float64{0, 1e-6, 0.01} {
			label := fmt.Sprintf("soup trial=%d delta=%v", trial, delta)
			requireSameCrossSpanVerdict(t, label, m, delta)
			if compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
				m.data, m.tris, m.spanOf, delta) != nil {
				refused++
			}
		}
	}
	// The sweep must have met refusals, not only clean meshes.
	require.Greater(t, refused, 50)
}

// BenchmarkCrossSpanContactAudit audits the 233-cell mesh a refined
// 87-degree twist of the tube-wall cutter holds, with the swept audit and the
// all-pairs reference.
func BenchmarkCrossSpanContactAudit(b *testing.B) {
	m := twistedPrismMesh(b, 7.7, 2.375, 87*math.Pi/180, 6, 233)
	const delta = 1e-9
	for _, bench := range []struct {
		name  string
		audit func(*proofbound.WorkBudget, []revolvemesh.RevolveAuditTri, [][3]int, []int, float64) error
	}{
		{"swept", compositesweep.CrossSpanContactAudit},
		{"all-pairs", legacyCrossSpanContactAudit},
	} {
		b.Run(bench.name, func(b *testing.B) {
			for b.Loop() {
				require.NoError(b, bench.audit(proofbound.NewWorkBudget(b.Context()), m.data, m.tris, m.spanOf, delta))
			}
		})
	}
}
