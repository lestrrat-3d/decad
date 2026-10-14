package meshbool

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// oldSharedEdgeSeparation is the plane-key decision used for a pair with two
// shared indices before pairSeparated checks the other corner's plane side.
func oldSharedEdgeSeparation(e *heldEmbedding, i, j int, sharedI, sharedJ []int) bool {
	pi, pj := e.planeOf(i), e.planeOf(j)
	if pi == -2 || pj == -2 {
		return false
	}
	if pi != pj {
		return true
	}
	ti, tj := e.h.Tris[i], e.h.Tris[j]
	a, b := ti[sharedI[0]], ti[sharedI[1]]
	u, v, _ := e.planeAxes(i)
	return e.orient2(a, b, unshared(ti, sharedI).verts[0], u, v)*
		e.orient2(a, b, unshared(tj, sharedJ).verts[0], u, v) < 0
}

func TestHeldSharedEdgeSeparationMatchesPlaneKeys(t *testing.T) {
	t.Parallel()
	subnormal := math.SmallestNonzeroFloat64
	cases := []struct {
		name string
		c, d r3.Vec
	}{
		{"opposite coplanar", r3.Vec{Y: 1}, r3.Vec{Y: -1}},
		{"overlapping coplanar", r3.Vec{Y: 1}, r3.Vec{Y: 2}},
		{"coincident other corners", r3.Vec{Y: 1}, r3.Vec{Y: 1}},
		{"signed zero", r3.Vec{Y: 1, Z: math.Copysign(0, -1)}, r3.Vec{Y: -1}},
		{"opposite winding", r3.Vec{Y: 1}, r3.Vec{Y: -2}},
		{"noncoplanar", r3.Vec{Y: 1}, r3.Vec{Y: -1, Z: 1}},
		{"near coplanar", r3.Vec{Y: 1}, r3.Vec{Y: -1, Z: subnormal}},
		{"collapsed second", r3.Vec{Y: 1}, r3.Vec{X: 0.5}},
		{"collapsed first", r3.Vec{X: 0.5}, r3.Vec{Y: -1}},
		{"both collapsed", r3.Vec{X: 0.5}, r3.Vec{X: 0.75}},
		{"large exponent", r3.Vec{Y: math.Ldexp(1, 900)}, r3.Vec{Y: -1, Z: subnormal}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			for _, reverse := range []bool{false, true} {
				tris := [][3]int{{0, 1, 2}, {0, 1, 3}}
				if reverse {
					tris[1] = [3]int{1, 0, 3}
				}
				h := floatCertMesh([]r3.Vec{{}, {X: 1}, tc.c, tc.d}, tris)
				for _, ij := range [][2]int{{0, 1}, {1, 0}} {
					i, j := ij[0], ij[1]
					originalVerts := append([]r3.Vec(nil), h.Verts...)
					originalTris := append([][3]int(nil), h.Tris...)
					var sharedI, sharedJ []int
					for a, va := range h.Tris[i] {
						for b, vb := range h.Tris[j] {
							if va == vb {
								sharedI, sharedJ = append(sharedI, a), append(sharedJ, b)
							}
						}
					}
					wantE := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
					want := oldSharedEdgeSeparation(wantE, i, j, sharedI, sharedJ)
					gotE := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
					got, decided, signs := gotE.pairSeparated(i, j, sharedI, sharedJ)
					require.True(t, decided)
					require.Equal(t, pairSideSigns{}, signs)
					require.Equal(t, want, got)
					require.Equal(t, originalVerts, h.Verts)
					require.Equal(t, originalTris, h.Tris)
				}
			}
		})
	}
}

type heldEdgePair struct {
	i, j   int
	si, sj []int
}

func heldSharedEdgePairs(h HeldRounding) []heldEdgePair {
	var pairs []heldEdgePair
	edges := make(map[[2]int]int)
	for i, tri := range h.Tris {
		for k := range 3 {
			a, c := tri[k], tri[(k+1)%3]
			if a > c {
				a, c = c, a
			}
			key := [2]int{a, c}
			j, ok := edges[key]
			if !ok {
				edges[key] = i
				continue
			}
			var si, sj []int
			for a, va := range h.Tris[j] {
				for bb, vb := range tri {
					if va == vb {
						si, sj = append(si, a), append(sj, bb)
					}
				}
			}
			if len(si) == 2 {
				pairs = append(pairs, heldEdgePair{j, i, si, sj})
			}
		}
	}
	return pairs
}

func TestHeldSharedEdgeSeparationMatchesPlaneKeysOnMeshes(t *testing.T) {
	t.Parallel()
	for name, h := range map[string]HeldRounding{
		"sphere": floatCertSphere(), "grid": floatCertGrid(), "fan": floatCertFan(),
	} {
		t.Run(name, func(t *testing.T) {
			pairs := heldSharedEdgePairs(h)
			require.NotEmpty(t, pairs)
			old := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			current := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			for _, p := range pairs {
				want := oldSharedEdgeSeparation(old, p.i, p.j, p.si, p.sj)
				got, decided, signs := current.pairSeparated(p.i, p.j, p.si, p.sj)
				require.True(t, decided)
				require.Equal(t, pairSideSigns{}, signs)
				require.Equal(t, want, got, "facets %d and %d", p.i, p.j)
			}
		})
	}
}

func BenchmarkHeldSharedEdgeSeparation(b *testing.B) {
	h := floatCertSphere()
	pairs := heldSharedEdgePairs(h)
	if len(pairs) < 256 {
		b.Fatal("sphere did not supply 256 adjacent pairs")
	}
	pairs = pairs[:256]
	b.ReportAllocs()
	for b.Loop() {
		e := newHeldEmbedding(h, nil, false)
		for _, p := range pairs {
			e.pairSeparated(p.i, p.j, p.si, p.sj)
		}
	}
}
