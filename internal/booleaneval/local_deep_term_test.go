package booleaneval

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/decad/internal/occupancyleaf"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// oldLocalDeepTerm is localDeepTerm as it read before it took the facets'
// pokes: it recomputes each contact facet's poke through LocalBulge.
func oldLocalDeepTerm(op meshbool.OperationKind, ra, rb *occupancy.Record, ma, mb *topo.Mesh, contacts []meshbool.ContactSite) (float64, [][2]r3.Vec) {
	da, db := occupancyleaf.Deltas(ma), occupancyleaf.Deltas(mb)
	corners := func(m *topo.Mesh, t int) [3]r3.Vec {
		tri := m.Triangles[t]
		return [3]r3.Vec{m.Vertices[tri[0]], m.Vertices[tri[1]], m.Vertices[tri[2]]}
	}
	deep := 0.0
	sites := make([][2]r3.Vec, 0, len(contacts))
	for _, c := range contacts {
		sites = append(sites, c.Box)
		la := ra.LocalBulge(corners(ma, c.I), da[c.I], c.Box)
		lb := db[c.J]
		if op != meshbool.OpCut {
			lb = rb.LocalBulge(corners(mb, c.J), db[c.J], c.Box)
		}
		if b := max(la, lb); b > 0 {
			deep = max(deep, proofbound.DivUpper(proofbound.AbsSumUpper(b, b), c.Sin))
		}
	}
	if !(deep < occupancy.SiteReach) {
		return math.Inf(1), sites
	}
	return deep, sites
}

// pokeMesh is a ring of facets around a unit-radius cylinder along Z, some
// vertices a little outside it and some far from it, with a per-vertex bound.
func pokeMesh(rng *rand.Rand, outside float64, outliers bool) *topo.Mesh {
	const n = 24
	m := &topo.Mesh{}
	for i := range n {
		a := 2 * math.Pi * float64(i) / n
		for k := range 2 {
			r := 1 + outside*rng.Float64()*float64(rng.IntN(3)-1)
			if outliers && rng.IntN(9) == 0 {
				r += 40
			}
			m.Vertices = append(m.Vertices, r3.Vec{X: r * math.Cos(a), Y: r * math.Sin(a), Z: 0.2 + 1.6*float64(k)})
		}
		m.VertexBound = append(m.VertexBound, 1e-4, 1e-4)
	}
	for i := range n {
		j := (i + 1) % n
		m.Triangles = append(m.Triangles, [3]int{2 * i, 2 * j, 2*i + 1}, [3]int{2 * j, 2*j + 1, 2*i + 1})
	}
	return m
}

func TestLocalDeepTermMatchesRecomputedPokes(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(5, 9))
	leaf, err := occupancy.CircleLeaf(occupancy.Chain{{M: [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}},
		0, 0, 1, 0, 2, 0, 0, 0, 1e-9, 64, 1e-4)
	require.NoError(t, err)
	rec := occupancy.LeafRecord(leaf)
	finite := 0
	for _, op := range []meshbool.OperationKind{meshbool.OpUnion, meshbool.OpCut} {
		for trial := range 6 {
			// Even trials mix far facets and shallow pokes; odd trials keep
			// one operand inside the leaf so the other's poke alone decides.
			ma, mb := pokeMesh(rng, 1e-5, trial%2 == 0), pokeMesh(rng, 1e-5, trial%2 == 0)
			if trial%2 == 1 {
				ma, mb = pokeMesh(rng, 1e-5*float64(trial%4/2), false), pokeMesh(rng, 1e-5*float64(1-trial%4/2), false)
			}
			var contacts []meshbool.ContactSite
			for range 60 {
				i, j := rng.IntN(len(ma.Triangles)), rng.IntN(len(mb.Triangles))
				tri := ma.Triangles[i]
				lo, hi := ma.Vertices[tri[0]], ma.Vertices[tri[0]]
				for _, v := range tri {
					p := ma.Vertices[v]
					lo = r3.Vec{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
					hi = r3.Vec{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
				}
				contacts = append(contacts, meshbool.ContactSite{I: i, J: j, Sin: 0.2 + rng.Float64(), Box: [2]r3.Vec{lo, hi}})
			}
			bulgeA, pokesA := occupancyleaf.BulgesPokes(rec, ma)
			require.Len(t, bulgeA, len(ma.Triangles))
			var pokesB []float64
			if op != meshbool.OpCut {
				_, pokesB = occupancyleaf.BulgesPokes(rec, mb)
			}
			wantDeep, wantSites := oldLocalDeepTerm(op, rec, rec, ma, mb, contacts)
			gotDeep, gotSites := localDeepTerm(op, rec, rec, pokesA, pokesB, ma, mb, contacts)
			require.Equal(t, math.Float64bits(wantDeep), math.Float64bits(gotDeep), "%v trial %d", op, trial)
			require.Equal(t, wantSites, gotSites)
			if !math.IsInf(gotDeep, 1) {
				finite++
			}
		}
	}
	require.Positive(t, finite, "some deep terms must be finite")
}
