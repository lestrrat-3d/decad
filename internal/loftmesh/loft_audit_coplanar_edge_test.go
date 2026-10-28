package loftmesh

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// triTriCoplanarSharedEdgeReference is TriTriCoplanarSharedEdge as the pair
// classifier called it before coplanarSharedEdge, kept verbatim so a later
// edit to the helper cannot move the reference.
func triTriCoplanarSharedEdgeReference(xta, xtb [3]proof.Xpt, n proof.Xpt, edgeA, edgeB proof.Xpt) bool {
	edgeAKey, edgeBKey := edgeA.Key(), edgeB.Key()
	var apexA, apexB proof.Xpt
	for _, p := range xta {
		key := p.Key()
		if key != edgeAKey && key != edgeBKey {
			apexA = p
			break
		}
	}
	for _, p := range xtb {
		key := p.Key()
		if key != edgeAKey && key != edgeBKey {
			apexB = p
			break
		}
	}
	sA := meshbool.PlaneSide(edgeA, edgeB, apexA, n)
	sB := meshbool.PlaneSide(edgeA, edgeB, apexB, n)
	return sA != 0 && sB != 0 && sA != sB
}

// coplanarEdgeOutcome is one call's result: its bool, or that it panicked.
type coplanarEdgeOutcome struct {
	value    bool
	panicked bool
}

func coplanarEdgeCall(fn func() bool) (out coplanarEdgeOutcome) {
	defer func() {
		if recover() != nil {
			out = coplanarEdgeOutcome{panicked: true}
		}
	}()
	return coplanarEdgeOutcome{value: fn()}
}

// coplanarEdgeTally counts the outcomes one comparison saw.
type coplanarEdgeTally struct {
	trues, falses, panics, zeroNormals, pairs int
}

// requireCoplanarEdgeMatches compares coplanarSharedEdge with the reference
// for every ordered pair of d's triangles sharing two vertex indices, in both
// edge orders.
func requireCoplanarEdgeMatches(t *testing.T, name string, d *LoftAuditData, tally *coplanarEdgeTally) {
	t.Helper()
	for i := range d.tris {
		for j := range d.tris {
			if i == j {
				continue
			}
			shared, count := tessellation.SharedVertexIndices(d.tris[i], d.tris[j])
			if count != 2 {
				continue
			}
			for _, edge := range [][2]int{{shared[0], shared[1]}, {shared[1], shared[0]}} {
				a, b := edge[0], edge[1]
				want := coplanarEdgeCall(func() bool {
					return triTriCoplanarSharedEdgeReference(d.xtri(i), d.xtri(j), d.Norms[i], d.Xverts[a], d.Xverts[b])
				})
				got := coplanarEdgeCall(func() bool { return d.coplanarSharedEdge(i, j, a, b) })
				if want != got {
					require.Failf(t, "coplanarSharedEdge differs from the reference",
						"%s: pair %d,%d edge %d,%d: want %+v, got %+v", name, i, j, a, b, want, got)
				}
				tally.pairs++
				switch {
				case want.panicked:
					tally.panics++
				case want.value:
					tally.trues++
				default:
					tally.falses++
				}
				if !xptNonzeroVector(d.Norms[i]) || !xptNonzeroVector(d.Norms[j]) {
					tally.zeroNormals++
				}
			}
		}
	}
}

// gridCubeSurface is the boundary of [0, k]^3 split into unit squares, two
// triangles each. rng picks each square's diagonal and flips some triangles'
// winding, so both orientations of a shared edge occur.
func gridCubeSurface(k int, rng *rand.Rand) ([]r3.Vec, [][3]int) {
	index := map[[3]int]int{}
	var verts []r3.Vec
	vertex := func(p [3]int) int {
		if v, ok := index[p]; ok {
			return v
		}
		index[p] = len(verts)
		verts = append(verts, r3.Vec{X: float64(p[0]), Y: float64(p[1]), Z: float64(p[2])})
		return index[p]
	}
	var tris [][3]int
	for axis := range 3 {
		u, w := (axis+1)%3, (axis+2)%3
		for _, level := range []int{0, k} {
			for s := range k {
				for r := range k {
					corner := func(ds, dr int) int {
						var p [3]int
						p[axis], p[u], p[w] = level, s+ds, r+dr
						return vertex(p)
					}
					c00, c10, c11, c01 := corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)
					quad := [][3]int{{c00, c10, c11}, {c00, c11, c01}}
					if rng.IntN(2) == 0 {
						quad = [][3]int{{c00, c10, c01}, {c10, c11, c01}}
					}
					for _, tri := range quad {
						if rng.IntN(4) == 0 {
							tri[1], tri[2] = tri[2], tri[1]
						}
						tris = append(tris, tri)
					}
				}
			}
		}
	}
	return verts, tris
}

func mapVerts(verts []r3.Vec, fn func(r3.Vec) r3.Vec) []r3.Vec {
	out := make([]r3.Vec, len(verts))
	for i, v := range verts {
		out[i] = fn(v)
	}
	return out
}

// edgePairFamily lists every corner order of two triangles on the edge (0, 1)
// with apexes 2 and 3.
func edgePairFamily() [][3]int {
	var tris [][3]int
	for _, apex := range []int{2, 3} {
		tris = append(tris,
			[3]int{0, 1, apex}, [3]int{1, apex, 0}, [3]int{apex, 0, 1},
			[3]int{1, 0, apex}, [3]int{0, apex, 1}, [3]int{apex, 1, 0})
	}
	return tris
}

// dyadicStep returns a random dyadic in [-4, 4] with at most six fraction bits.
func dyadicStep(rng *rand.Rand) float64 { return float64(rng.IntN(513)-256) / 64 }

// edgePairPoints returns an edge and two apexes. Kind 0 puts the second apex
// exactly in the first triangle's plane, kind 1 puts it off the plane, kind 2
// puts an apex on the edge's line (a zero-area triangle) and kind 3 makes a
// sliver whose apex is one unit in the last place off the edge's line.
func edgePairPoints(rng *rand.Rand, kind int, scale float64) []r3.Vec {
	a := r3.Vec{X: dyadicStep(rng), Y: dyadicStep(rng), Z: dyadicStep(rng)}
	b := a.Add(r3.Vec{X: dyadicStep(rng) + 5, Y: dyadicStep(rng), Z: dyadicStep(rng)})
	p := a.Add(r3.Vec{X: dyadicStep(rng), Y: dyadicStep(rng) + 5, Z: dyadicStep(rng)})
	combo := func(s, u float64) r3.Vec { return a.Add(b.Sub(a).Scale(s)).Add(p.Sub(a).Scale(u)) }
	var q r3.Vec
	switch kind {
	case 0:
		q = combo(dyadicStep(rng)/4, dyadicStep(rng)/4)
	case 1:
		q = combo(dyadicStep(rng)/4, dyadicStep(rng)/4).Add(r3.Vec{Z: 1.0 / 128})
	case 2:
		p = combo(dyadicStep(rng)/4, 0)
		q = combo(dyadicStep(rng)/4, -1)
	default:
		mid := combo(0.5, 0)
		p = r3.Vec{X: mid.X, Y: math.Nextafter(mid.Y, math.Inf(1)), Z: mid.Z}
		q = combo(dyadicStep(rng)/4, dyadicStep(rng)/4)
	}
	return mapVerts([]r3.Vec{a, b, p, q}, func(v r3.Vec) r3.Vec { return v.Scale(scale) })
}

// coplanarEdgeBuilders are the table layouts the pair classifier reads.
var coplanarEdgeBuilders = map[string]func([]r3.Vec, [][3]int) *LoftAuditData{
	"table":    NewLoftAuditData,
	"streamed": newCompactLoftAuditData,
	"legacy": func(verts []r3.Vec, tris [][3]int) *LoftAuditData {
		return legacyLoftAuditDataSelected(verts, liftLoftVertices(verts, nil, 1), tris, nil, false)
	},
	"exact": func(verts []r3.Vec, tris [][3]int) *LoftAuditData {
		return NewLoftAuditDataWithExact(verts, liftLoftVertices(verts, nil, 1), tris)
	},
}

func TestCoplanarSharedEdgeMatchesTheHelper(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	type mesh struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
	}
	var meshes []mesh
	for _, k := range []int{1, 2, 3} {
		verts, tris := gridCubeSurface(k, rng)
		meshes = append(meshes,
			mesh{fmt.Sprintf("cube%d", k), verts, tris},
			mesh{fmt.Sprintf("cube%d-tenths", k), mapVerts(verts, func(v r3.Vec) r3.Vec { return v.Scale(0.1) }), tris},
			mesh{fmt.Sprintf("cube%d-swapped", k), mapVerts(verts, func(v r3.Vec) r3.Vec {
				return r3.Vec{X: v.Z, Y: -v.X, Z: v.Y}
			}), tris},
			mesh{fmt.Sprintf("cube%d-near-2^63", k), mapVerts(verts, func(v r3.Vec) r3.Vec {
				return v.Scale(0x1p60).Add(r3.Vec{X: 0x1p62, Y: -0x1p62, Z: 0x1p61})
			}), tris},
			mesh{fmt.Sprintf("cube%d-word-edge", k), mapVerts(verts, func(v r3.Vec) r3.Vec {
				return v.Add(r3.Vec{X: 0x1p52, Y: 0x1p53 - 4, Z: -0x1p52})
			}), tris},
			mesh{fmt.Sprintf("cube%d-tiny", k), mapVerts(verts, func(v r3.Vec) r3.Vec { return v.Scale(0x1p-1000) }), tris},
		)
		// Turned faces are close to planar but rarely exactly planar.
		for turn := range 3 {
			axis := r3.Vec{X: rng.Float64() - 0.5, Y: rng.Float64() - 0.5, Z: rng.Float64() - 0.5}
			rotation, err := r3.Rotation(axis, units.Radians(rng.Float64()*math.Pi))
			require.NoError(t, err)
			meshes = append(meshes, mesh{fmt.Sprintf("cube%d-turned%d", k, turn), mapVerts(verts, rotation.Apply), tris})
		}
	}
	for n := range 64 {
		kind := n % 4
		scale := []float64{1, 0x1p40, 0x1p-40, 0x1p58}[(n/4)%4]
		meshes = append(meshes, mesh{fmt.Sprintf("pair%d-kind%d", n, kind),
			edgePairPoints(rng, kind, scale), edgePairFamily()})
	}
	// Two indices with the same coordinates make zero-area triangles.
	meshes = append(meshes, mesh{"repeated-point",
		[]r3.Vec{{}, {X: 1}, {Y: 1}, {X: 1}}, edgePairFamily()})

	var tally coplanarEdgeTally
	for _, m := range meshes {
		for name, build := range coplanarEdgeBuilders {
			requireCoplanarEdgeMatches(t, m.name+"/"+name, build(m.verts, m.tris), &tally)
		}
	}
	require.Positive(t, tally.trues)
	require.Positive(t, tally.falses)
	require.Positive(t, tally.zeroNormals)
}

// TestCoplanarSharedEdgeMatchesTheHelperOnHandBuiltTables covers tables no
// constructor builds: rational lifts, negative weight spellings, vertices
// sharing one point's integers, and missing normals.
func TestCoplanarSharedEdgeMatchesTheHelperOnHandBuiltTables(t *testing.T) {
	t.Parallel()
	frac := func(x, y, z, w int64) proof.Xpt {
		return proof.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(z), W: big.NewInt(w)}
	}
	huge := new(big.Int).Lsh(big.NewInt(1), 130)
	hugePoint := func(x, y, z int64) proof.Xpt {
		scaled := func(v int64) *big.Int { return new(big.Int).Mul(huge, big.NewInt(v)) }
		return proof.Xpt{X: scaled(x), Y: scaled(y), Z: scaled(z), W: big.NewInt(3)}
	}
	tables := map[string][]proof.Xpt{
		"thirds":     {frac(0, 0, 0, 3), frac(3, 1, 0, 3), frac(1, 3, 0, 3), frac(2, -2, 0, 3)},
		"thirds-off": {frac(0, 0, 0, 3), frac(3, 1, 0, 3), frac(1, 3, 0, 3), frac(2, -2, 1, 7)},
		"mixed":      {frac(0, 0, 0, 5), frac(4, 0, 0, 1), frac(1, 1, 0, 2), frac(1, -1, 0, 9)},
		"negative":   {frac(0, 0, 0, -1), frac(-3, 0, 0, -1), frac(0, -3, 0, -1), frac(1, 1, 0, 1)},
		"negative-n": {frac(0, 0, 0, 1), frac(-3, 0, 0, -1), frac(0, 3, 0, 1), frac(2, -2, 0, 1)},
		"negative-p": {frac(0, 0, 0, 1), frac(3, 0, 0, 1), frac(0, -3, 0, -1), frac(2, -2, 0, 1)},
		"negative-q": {frac(0, 0, 0, 1), frac(3, 0, 0, 1), frac(0, 3, 0, 1), frac(-2, 2, 0, -1)},
		"huge":       {hugePoint(0, 0, 0), hugePoint(5, 1, 2), hugePoint(1, 5, 2), hugePoint(3, -4, 2)},
		"huge-off":   {hugePoint(0, 0, 0), hugePoint(5, 1, 2), hugePoint(1, 5, 2), hugePoint(3, -4, 3)},
	}
	var tally coplanarEdgeTally
	for name, exact := range tables {
		verts := make([]r3.Vec, len(exact))
		for i, p := range exact {
			verts[i] = p.Vec()
		}
		requireCoplanarEdgeMatches(t, name, legacyLoftAuditDataSelected(verts, exact, edgePairFamily(), nil, false), &tally)
		requireCoplanarEdgeMatches(t, name+"/exact", NewLoftAuditDataWithExact(verts, exact, edgePairFamily()), &tally)
	}

	square := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 1, Y: -1}}
	shared := liftLoftVertices(square, nil, 1)
	shared[3] = shared[2]
	requireCoplanarEdgeMatches(t, "shared-apex-pointers",
		legacyLoftAuditDataSelected(square, shared, edgePairFamily(), nil, false), &tally)

	missing := NewLoftAuditData(square, edgePairFamily())
	missing.Norms[0] = proof.Xpt{}
	missing.Norms[7] = proof.Xpt{X: missing.Norms[7].X, Y: missing.Norms[7].Y, Z: missing.Norms[7].Z}
	requireCoplanarEdgeMatches(t, "missing-normals", missing, &tally)

	partial := newLoftAuditDataForCandidates(square, edgePairFamily(), loftCandidates{
		starts: []int{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, js: []int32{3},
	})
	requireCoplanarEdgeMatches(t, "partial-table", partial, &tally)

	require.Positive(t, tally.trues)
	require.Positive(t, tally.falses)
	require.Positive(t, tally.panics)
}

// BenchmarkCoplanarSharedEdge times the coplanar shared-edge reading over
// every coplanar edge-sharing pair of a split cube's surface: helper is
// TriTriCoplanarSharedEdge on the table's points, table is coplanarSharedEdge.
func BenchmarkCoplanarSharedEdge(b *testing.B) {
	verts, tris := gridCubeSurface(8, rand.New(rand.NewPCG(3, 5)))
	d := NewLoftAuditData(verts, tris)
	type pair struct{ i, j, a, b int }
	var pairs []pair
	for i := range tris {
		for j := range tris {
			shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
			if i == j || count != 2 {
				continue
			}
			apex := d.Xverts[tessellation.TriangleApexIndex(tris[j], shared[0], shared[1])]
			if proof.XdotSign(d.Norms[i], proof.Xsub(apex, d.Xverts[shared[0]])) == 0 {
				pairs = append(pairs, pair{i, j, shared[0], shared[1]})
			}
		}
	}
	require.NotEmpty(b, pairs)
	b.Run("helper", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			for _, p := range pairs {
				TriTriCoplanarSharedEdge(d.xtri(p.i), d.xtri(p.j), d.Norms[p.i], d.Xverts[p.a], d.Xverts[p.b])
			}
		}
	})
	b.Run("table", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			for _, p := range pairs {
				d.coplanarSharedEdge(p.i, p.j, p.a, p.b)
			}
		}
	})
}
