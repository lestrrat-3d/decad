package meshbool

import (
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// parityAxisPlanes lists the (u, v) projection plane of each swept axis.
var parityAxisPlanes = [3][2]int{{1, 2}, {0, 2}, {0, 1}}

// parityGridTry is the outcome of placing one mesh on one grid with no cap.
type parityGridTry struct {
	inner  int
	total  int // memberships; zero when refused
	status int
}

// parityGridChain places pm on every grid the retry can visit, from the first
// grid down to the floor, with a limit nothing reaches. It reports false when
// the axis gets no index before any grid (too small, range guard).
func parityGridChain(pm *ParityMesh, axis int) ([]parityGridTry, bool) {
	n := len(pm.Tris)
	if n < ParityIndexMinFacets {
		return nil, false
	}
	u, v := parityAxisPlanes[axis][0], parityAxisPlanes[axis][1]
	boxes, umin, umax, vmin, vmax, ok := pm.parityBoxBounds(axis, u, v)
	if !ok {
		return nil, false
	}
	var chain []parityGridTry
	for inner := parityGridInner(n); inner >= parityGridMinInner; inner /= 2 {
		_, pl, status := pm.placeParityGrid(u, v, boxes, umin, umax, vmin, vmax, inner, math.MaxInt32/2)
		try := parityGridTry{inner: inner, status: status}
		if pl != nil {
			try.total = pl.memberships
		}
		chain = append(chain, try)
		if status == parityPlaceRefused {
			break
		}
	}
	return chain, true
}

// parityExpectedGrid is the inner cell count the retry must settle on under
// limit, or zero when it must give no index.
func parityExpectedGrid(chain []parityGridTry, limit int) int {
	for _, try := range chain {
		if try.status == parityPlaceRefused {
			return 0
		}
		if try.total <= limit {
			return try.inner
		}
	}
	return 0
}

// parityGridCells reports the inner cells per side of idx, zero for nil.
func parityGridCells(idx *ParityIndex) int {
	if idx == nil {
		return 0
	}
	return len(idx.u.lo) - 2
}

// parityRing is a closed toothed ring along z of k segments and b bands of
// wall, 10 tall: with b = 1 every wall facet is as tall as the ring, so its
// projection along x or y covers every row of the grid.
func parityRing(k, b int) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	at := func(level, j, side int) int { return (level*k+j)*2 + side }
	for level := 0; level <= b; level++ {
		z := 10 * float64(level) / float64(b)
		for j := range k {
			a := 2 * math.Pi * float64(j) / float64(k)
			r1 := 40.0
			if j%8 < 4 {
				r1 = 42
			}
			verts = append(verts, r3.NewVec(r1*math.Cos(a), r1*math.Sin(a), z),
				r3.NewVec(50*math.Cos(a), 50*math.Sin(a), z))
		}
	}
	var tris [][3]int
	for j := range k {
		n := (j + 1) % k
		tris = append(tris, [3]int{at(0, j, 0), at(0, n, 0), at(0, j, 1)}, [3]int{at(0, n, 0), at(0, n, 1), at(0, j, 1)})
		tris = append(tris, [3]int{at(b, j, 0), at(b, j, 1), at(b, n, 0)}, [3]int{at(b, n, 0), at(b, j, 1), at(b, n, 1)})
		for level := range b {
			for side := range 2 {
				p, q := at(level, j, side), at(level, n, side)
				pu, qu := at(level+1, j, side), at(level+1, n, side)
				tris = append(tris, [3]int{p, q, pu}, [3]int{q, qu, pu})
			}
		}
	}
	return verts, tris
}

// parityRingQueries returns count points around a parityRing: random in its
// box, on a vertex's y and z line with a random x, at a vertex, and in the wall.
func parityRingQueries(rng *rand.Rand, verts []r3.Vec, count int) []proof.Xpt {
	var qs []proof.Xpt
	for i := range count {
		switch i % 4 {
		case 0:
			qs = append(qs, proof.XptOf(r3.NewVec(rng.Float64()*120-60, rng.Float64()*120-60, rng.Float64()*14-2)))
		case 1:
			v := verts[rng.IntN(len(verts))]
			qs = append(qs, proof.XptOf(r3.NewVec(rng.Float64()*120-60, v.Y, v.Z)))
		case 2:
			qs = append(qs, proof.XptOf(verts[rng.IntN(len(verts))]))
		default:
			a := rng.Float64() * 2 * math.Pi
			r := 43 + 6*rng.Float64()
			qs = append(qs, proof.XptOf(r3.NewVec(r*math.Cos(a), r*math.Sin(a), rng.Float64()*10)))
		}
	}
	return qs
}

// parityEffectiveLimit is the membership limit a mesh with the test override
// maxMemberships works under.
func parityEffectiveLimit(override int) int {
	return (&ParityMesh{maxMemberships: override}).membershipLimit()
}

// parityCapsFor lists the test caps for a mesh whose first grid lists total
// memberships: the default, the total and one less, fractions of it and one,
// and, when exact is set, the boundary of every grid the retry can visit.
func parityCapsFor(total int, chain []parityGridTry, exact bool) []int {
	caps := []int{0, total, total - 1, total / 2, total / 8, 1}
	for _, try := range chain {
		if exact {
			caps = append(caps, try.total, try.total-1)
		}
	}
	slices.Sort(caps)
	caps = slices.Compact(caps)
	return slices.DeleteFunc(caps, func(c int) bool { return c < 0 })
}

// parityCheckCoarseRetry checks, for every cap in parityCapsFor (exact: with the grid boundaries), that the
// index of axis 0 answers every query like the plain unfiltered scan, that the
// grid it settled on is the first one that fits, and that its memberships stay
// within the cap. A failure names the fixture, cap and query index only.
func parityCheckCoarseRetry(t *testing.T, fx parityIndexFixture, qs []proof.Xpt, exact bool) {
	t.Helper()
	chain, ok := parityGridChain(NewParityMesh(fx.verts, fx.tris), 0)
	require.True(t, ok, `%s has an index grid`, fx.name)
	all := parityAllFacets(len(fx.tris))
	plain := NewParityMesh(fx.verts, fx.tris)
	plain.Unfiltered = true
	want := make([]parityAnswer, len(qs))
	for qi, p := range qs {
		want[qi] = answerOf(MeshParityPreparedContext(t.Context(), p, plain, all))
	}
	for _, limit := range parityCapsFor(chain[0].total, chain, exact) {
		got := NewParityMesh(fx.verts, fx.tris)
		got.maxMemberships = limit
		for qi, p := range qs {
			have := answerOf(MeshParityAllContext(t.Context(), p, got))
			if have != want[qi] {
				require.Failf(t, `index differs from the plain scan`, `%s cap %d query %d: got %+v want %+v`,
					fx.name, limit, qi, have, want[qi])
			}
		}
		require.True(t, got.indexBuilt[0], `%s cap %d: the index build was attempted`, fx.name, limit)
		effective := parityEffectiveLimit(limit)
		require.Equal(t, parityExpectedGrid(chain, effective), parityGridCells(got.index[0]),
			`%s cap %d: grid`, fx.name, limit)
		if got.index[0] != nil {
			require.LessOrEqual(t, len(got.index[0].ids), effective, `%s cap %d: memberships`, fx.name, limit)
		}
	}
}

// parityCoarseQueries returns the queries for a fixture: the shared ones for
// small fixtures and ring queries for the large ones.
func parityCoarseQueries(fx parityIndexFixture, ring int) []proof.Xpt {
	rng := rand.New(rand.NewPCG(0x61, 0x62))
	if ring > 0 {
		return parityRingQueries(rng, fx.verts, ring)
	}
	return parityIndexQueries(rng, fx)
}
