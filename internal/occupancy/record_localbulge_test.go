package occupancy

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// oldLocalBulge is LocalBulge as it read before it took the facet's poke as
// input: it computes the least poke over the near leaves facet by facet, from
// oldCylinderPoke rather than from meshPokes or the split helpers.
func oldLocalBulge(r *Record, tri [3]r3.Vec, delta float64, site [2]r3.Vec) float64 {
	lo, hi := triBox(tri)
	poke := math.Inf(1)
	for _, l := range r.Leaves {
		if !l.near(lo, hi, r.Cross+l.Delta) {
			continue
		}
		if c, ok := l.shape.(cylinderShape); ok {
			poke = math.Min(poke, oldCylinderPoke(c, tri))
			continue
		}
		poke = math.Min(poke, l.shape.poke(tri))
	}
	lam := poke
	for _, l := range r.Holes {
		if l.nearHoleWall(site[0], site[1], SiteReach) {
			lam = math.Max(lam, l.Delta)
		}
	}
	if r.Cross > 0 && r.sites().near(site[0], site[1], SiteReach) {
		lam = math.Max(lam, r.Cross)
	}
	return math.Min(delta, lam)
}

// randLocalBulgeRecord is randBulgeRecord, then optionally cut by a tool leaf
// or joined with a crossing site so the hole and crossing readings are live.
func randLocalBulgeRecord(t *testing.T, rng *rand.Rand, scale float64, kind int) *Record {
	t.Helper()
	rec := randBulgeRecord(t, rng, scale, kind == 0)
	switch kind {
	case 2:
		tool, err := CircleLeaf(randChain(rng, scale), 0, 0, 0.3+rng.Float64()*scale, 0, 1+rng.Float64()*scale,
			rng.Float64()*0.01, 0, 0, 1e-9, 64, 0.01)
		require.NoError(t, err)
		out := CutCompose(rec, LeafRecord(tool), 0, 0, nil)
		require.NotNil(t, out)
		return out
	case 3:
		sites := [][2]r3.Vec{{
			{X: -scale / 4, Y: -scale / 4, Z: -scale / 4}, {X: scale / 4, Y: scale / 4, Z: scale / 4},
		}}
		out := Compose(rec, rec, 0, 0.02, sites)
		require.NotNil(t, out)
		return out
	}
	return rec
}

// TestLocalBulgeAtMatchesOldLocalBulge compares, bit for bit, the reading the
// Boolean takes from BulgesPokes and LocalBulgeAt, and LocalBulge itself, with
// the old per-facet LocalBulge, on cylinder, mixed, hole and crossing records,
// including facets far from every leaf (poke +Inf).
func TestLocalBulgeAtMatchesOldLocalBulge(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(13, 17))
	finite, infinite, holeHit, crossHit := 0, 0, 0, 0
	for trial := range 16 {
		scale := []float64{3, 20, 1e-3, 1e5}[trial%4]
		rec := randLocalBulgeRecord(t, rng, scale, trial/4)
		verts, tris := randBulgeMesh(rng, scale)
		delta := make([]float64, len(tris))
		for i := range delta {
			delta[i] = []float64{0, 1e-4, 0.05, 1, math.Inf(1)}[rng.IntN(5)]
		}
		lam, pokes := rec.BulgesPokes(verts, tris, delta)
		require.Equal(t, len(tris), len(pokes))
		wantLam := rec.Bulges(verts, tris, delta)
		for i := range lam {
			require.True(t, sameFloat(wantLam[i], lam[i]), "trial %d facet %d: Bulges", trial, i)
		}
		for ti, tri := range tris {
			tv := [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
			lo, hi := triBox(tv)
			sites := [][2]r3.Vec{
				{lo, hi},
				{lo.Sub(r3.Vec{X: 0.1, Y: 0.1, Z: 0.1}), hi.Add(r3.Vec{X: 0.1, Y: 0.1, Z: 0.1})},
				{{X: -scale / 4, Y: -scale / 4, Z: -scale / 4}, {X: scale / 4, Y: scale / 4, Z: scale / 4}},
			}
			if math.IsInf(pokes[ti], 1) {
				infinite++
			} else {
				finite++
			}
			for _, site := range sites {
				want := oldLocalBulge(rec, tv, delta[ti], site)
				got := rec.LocalBulgeAt(pokes[ti], delta[ti], site)
				require.True(t, sameFloat(want, got), "trial %d facet %d: want %v got %v", trial, ti, want, got)
				wrap := rec.LocalBulge(tv, delta[ti], site)
				require.True(t, sameFloat(want, wrap), "trial %d facet %d: LocalBulge want %v got %v", trial, ti, want, wrap)
				if len(rec.Holes) > 0 && want > pokes[ti] {
					holeHit++
				}
				if rec.Cross > 0 && want >= math.Min(delta[ti], rec.Cross) && want > pokes[ti] {
					crossHit++
				}
			}
		}
	}
	require.Positive(t, finite, "some facets must reach a leaf")
	require.Positive(t, infinite, "some facets must be far from every leaf")
	require.Positive(t, holeHit, "some readings must come from a hole's bound")
	require.Positive(t, crossHit, "some readings must come from a crossing depth")
}
