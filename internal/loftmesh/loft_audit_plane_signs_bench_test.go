package loftmesh

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

var planeSignsBenchSink int

// jitterMesh moves every vertex by up to amount on each axis.
func jitterMesh(m planeSignMesh, amount float64, rng *rand.Rand) planeSignMesh {
	for k := range m.verts {
		d := r3.Vec{X: rng.Float64() - 0.5, Y: rng.Float64() - 0.5, Z: rng.Float64() - 0.5}
		m.verts[k] = m.verts[k].Add(d.Scale(2 * amount))
	}
	return m
}

// broadPhasePairs is the crossing audit's candidate list for m filtered to
// the pairs its broad phase hands loftPlaneSeparatedSigns: no shared vertex
// and overlapping boxes, in the audit's row order.
func broadPhasePairs(b *testing.B, m planeSignMesh) [][2]int {
	cands, err := LoftSweepCandidates(proofbound.NewWorkBudget(b.Context()), m.verts, m.tris)
	if err != nil {
		b.Fatal(err)
	}
	var pairs [][2]int
	for _, p := range cands {
		_, shared := tessellation.SharedVertexIndices(m.tris[p[0]], m.tris[p[1]])
		if shared == 0 && meshbool.BoxesOverlap(meshbool.TriBox(m.verts, m.tris[p[0]]),
			meshbool.TriBox(m.verts, m.tris[p[1]])) {
			pairs = append(pairs, p)
		}
	}
	if len(pairs) == 0 {
		b.Fatal("no candidate pairs")
	}
	return pairs
}

// BenchmarkLoftPlaneSeparatedSigns runs the broad phase's plane test over the
// pairs broadPhasePairs selects, every pair once per iteration, against the
// full and the compact table. The torus has a few partners per row; the
// twisted band's long slanted triangles have hundreds, as the sector join's
// audit does. The vertices move by up to 1e-3 so the quads are not planar
// trapezoids: the float plane then decides nearly every sign, as it does in
// the sector join's audit. "reference" is the identity-first plane signs,
// "lifted" loftPlaneSeparatedSigns, and "memo" loftPlaneSeparatedSignsMemo
// with one new memo per iteration, as each parallelLoftPairs worker has.
func BenchmarkLoftPlaneSeparatedSigns(b *testing.B) {
	rng := rand.New(rand.NewPCG(0x77, 0x13))
	for _, m := range []planeSignMesh{
		jitterMesh(torusMesh("torus", 251, 97, 10, 3, 1, r3.Vec{X: 0.125, Y: -0.375}, false), 1e-3, rng),
		jitterMesh(twistedBandMesh(600, 6, 10, 4, 0.6), 1e-3, rng),
	} {
		pairs := broadPhasePairs(b, m)
		for _, layout := range []struct {
			name string
			data *LoftAuditData
		}{
			{tableFull, NewLoftAuditData(m.verts, m.tris)},
			{tableCompact, newCompactLoftAuditData(m.verts, m.tris)},
		} {
			for _, impl := range []struct {
				name string
				fn   func(*LoftAuditData, *loftPlaneSignMemo, int, int) (bool, [3]int, [3]int)
			}{
				{"reference", func(d *LoftAuditData, _ *loftPlaneSignMemo, i, j int) (bool, [3]int, [3]int) {
					return referenceLoftPlaneSeparatedSigns(d, i, j)
				}},
				{"lifted", func(d *LoftAuditData, _ *loftPlaneSignMemo, i, j int) (bool, [3]int, [3]int) {
					return loftPlaneSeparatedSigns(d, i, j)
				}},
				{"memo", loftPlaneSeparatedSignsMemo},
			} {
				b.Run(m.name+"/"+layout.name+"/"+impl.name, func(b *testing.B) {
					b.ReportAllocs()
					for b.Loop() {
						var memo *loftPlaneSignMemo
						if impl.name == "memo" {
							memo = newLoftPlaneSignMemo(layout.data)
						}
						for _, p := range pairs {
							separated, sa, sb := impl.fn(layout.data, memo, p[0], p[1])
							if separated {
								planeSignsBenchSink++
							}
							planeSignsBenchSink += sa[0] + sb[2]
						}
					}
					b.ReportMetric(float64(len(pairs)), "pairs/op")
				})
			}
		}
	}
}
