package meshbool

// CI: isolated-tests

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestParityCoarseRetryMatchesPlainScanOnFixtures(t *testing.T) {
	t.Parallel()
	for _, fx := range parityIndexFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			parityCheckCoarseRetry(t, fx, parityCoarseQueries(fx, 0), true)
		})
	}
}

func TestParityCoarseRetryMatchesPlainScanOnSmallRings(t *testing.T) {
	t.Parallel()
	v, tr := parityRing(256, 1)
	tall := parityIndexFixture{`tall ring`, v, tr}
	v, tr = parityRing(128, 6)
	banded := parityIndexFixture{`banded ring`, v, tr}
	for _, fx := range []parityIndexFixture{tall, banded} {
		t.Run(fx.name, func(t *testing.T) {
			parityCheckCoarseRetry(t, fx, parityCoarseQueries(fx, 600), true)
		})
	}
}

// TestParityCoarseRetryNeedsOneHalving uses a ring of 163 thousand facets
// whose first grid is over the default cap and whose second fits.
func TestParityCoarseRetryNeedsOneHalving(t *testing.T) {
	t.Parallel()
	v, tr := parityRing(10203, 3)
	fx := parityIndexFixture{`ring 163k, one halving`, v, tr}
	chain, ok := parityGridChain(NewParityMesh(v, tr), 0)
	require.True(t, ok)
	require.Greater(t, chain[0].total, ParityIndexMaxMemberships)
	require.LessOrEqual(t, chain[1].total, ParityIndexMaxMemberships)
	require.Equal(t, chain[1].inner, parityExpectedGrid(chain, ParityIndexMaxMemberships))
	parityCheckCoarseRetry(t, fx, parityCoarseQueries(fx, 60), false)
}

// TestParityCoarseRetryNeedsTwoHalvings uses a ring of 163 thousand full-height
// facets, which list three times the cap on the first grid and fit on the third.
func TestParityCoarseRetryNeedsTwoHalvings(t *testing.T) {
	t.Parallel()
	v, tr := parityRing(20406, 1)
	fx := parityIndexFixture{`ring 163k, full height`, v, tr}
	chain, ok := parityGridChain(NewParityMesh(v, tr), 0)
	require.True(t, ok)
	require.Greater(t, chain[1].total, ParityIndexMaxMemberships)
	require.LessOrEqual(t, chain[2].total, ParityIndexMaxMemberships)
	require.Equal(t, chain[2].inner, parityExpectedGrid(chain, ParityIndexMaxMemberships))
	parityCheckCoarseRetry(t, fx, parityCoarseQueries(fx, 60), false)
}

// TestParityCoarseRetryNeedsSeveralHalvings uses a larger ring of full-height
// facets, which fits only on the fourth grid.
func TestParityCoarseRetryNeedsSeveralHalvings(t *testing.T) {
	t.Parallel()
	v, tr := parityRing(50000, 1)
	fx := parityIndexFixture{`ring 400k, full height`, v, tr}
	chain, ok := parityGridChain(NewParityMesh(v, tr), 0)
	require.True(t, ok)
	require.Greater(t, chain[2].total, ParityIndexMaxMemberships)
	require.LessOrEqual(t, chain[3].total, ParityIndexMaxMemberships)
	require.Equal(t, chain[3].inner, parityExpectedGrid(chain, ParityIndexMaxMemberships))
	parityCheckCoarseRetry(t, fx, parityCoarseQueries(fx, 40), false)
}

func TestParityCoarseRetryNeverExceedsTheCap(t *testing.T) {
	t.Parallel()
	v, tr := parityRing(20406, 1)
	fixtures := append(parityIndexFixtures(), parityIndexFixture{`ring 163k, full height`, v, tr})
	for _, fx := range fixtures {
		exact := len(fx.tris) < 1000
		for axis, plane := range parityAxisPlanes {
			pm := NewParityMesh(fx.verts, fx.tris)
			chain, ok := parityGridChain(pm, axis)
			if !ok {
				continue
			}
			for _, limit := range parityCapsFor(chain[0].total, chain, exact) {
				got := NewParityMesh(fx.verts, fx.tris)
				got.maxMemberships = limit
				idx := got.buildParityIndex(axis, plane[0], plane[1])
				effective := parityEffectiveLimit(limit)
				require.Equal(t, parityExpectedGrid(chain, effective), parityGridCells(idx),
					`%s axis %d cap %d: grid`, fx.name, axis, limit)
				if idx != nil {
					require.LessOrEqual(t, len(idx.ids), effective, `%s axis %d cap %d: memberships`,
						fx.name, axis, limit)
				}
			}
		}
	}
}

func TestParityCoarseRetryGivesUpBelowTheFloorGrid(t *testing.T) {
	t.Parallel()
	for _, fx := range parityIndexFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			chain, ok := parityGridChain(NewParityMesh(fx.verts, fx.tris), 0)
			if !ok {
				t.Skip(`no index grid`)
			}
			floor := chain[len(chain)-1]
			require.Equal(t, parityPlaceOK, floor.status)
			require.Less(t, floor.inner/2, parityGridMinInner, `the chain ends at the floor grid`)
			fits := NewParityMesh(fx.verts, fx.tris)
			fits.maxMemberships = floor.total
			require.Equal(t, floor.inner, parityGridCells(fits.buildParityIndex(0, 1, 2)), `a cap at the floor total fits`)

			// One membership under the floor total fits no grid: the axis
			// gets no index and still answers like the plain scan.
			none := NewParityMesh(fx.verts, fx.tris)
			none.maxMemberships = floor.total - 1
			require.True(t, none.buildParityIndex(0, 1, 2) == nil, `no grid fits`)
			all := parityAllFacets(len(fx.tris))
			plain := NewParityMesh(fx.verts, fx.tris)
			plain.Unfiltered = true
			served := NewParityMesh(fx.verts, fx.tris)
			served.maxMemberships = floor.total - 1
			rng := rand.New(rand.NewPCG(0x63, 0x64))
			for qi, p := range parityIndexQueries(rng, fx) {
				want := answerOf(MeshParityPreparedContext(t.Context(), p, plain, all))
				have := answerOf(MeshParityAllContext(t.Context(), p, served))
				if have != want {
					require.Failf(t, `scan differs`, `%s query %d: got %+v want %+v`, fx.name, qi, have, want)
				}
			}
			require.True(t, served.indexBuilt[0], `the build was attempted`)
			require.True(t, served.index[0] == nil, `and no grid fit`)
		})
	}
}

// TestParityCoarseRetryStopsAtARefusedFacet checks that a facet the grid cannot
// place ends the search with no index, whatever the cap: it is not an over-cap
// outcome, so no coarser grid is tried.
func TestParityCoarseRetryStopsAtARefusedFacet(t *testing.T) {
	t.Parallel()
	fx := parityIndexFixtures()[0]
	// Two vertices above one of the cylinder's, so a facet projects to a
	// single point along the x axis.
	verts := append([]r3.Vec(nil), fx.verts...)
	tris := append([][3]int(nil), fx.tris...)
	base := verts[tris[0][0]]
	verts = append(verts, base, r3.NewVec(base.X+1, base.Y, base.Z), r3.NewVec(base.X+2, base.Y, base.Z))
	tris = append(tris, [3]int{len(verts) - 3, len(verts) - 2, len(verts) - 1})

	for _, limit := range []int{0, 1} {
		pm := NewParityMesh(verts, tris)
		pm.maxMemberships = limit
		boxes, umin, umax, vmin, vmax, ok := pm.parityBoxBounds(0, 1, 2)
		require.True(t, ok)
		inner := parityGridInner(len(tris))
		_, _, status := pm.placeParityGrid(1, 2, boxes, umin, umax, vmin, vmax, inner, pm.membershipLimit())
		require.Equal(t, parityPlaceRefused, status, `cap %d: the first grid refuses`, limit)
		require.True(t, pm.buildParityIndex(0, 1, 2) == nil, `cap %d: no index`, limit)
	}

	// Without that facet the same cap is an over-cap outcome.
	control := NewParityMesh(fx.verts, fx.tris)
	control.maxMemberships = 1
	boxes, umin, umax, vmin, vmax, ok := control.parityBoxBounds(0, 1, 2)
	require.True(t, ok)
	_, _, status := control.placeParityGrid(1, 2, boxes, umin, umax, vmin, vmax, parityGridInner(len(fx.tris)), 1)
	require.Equal(t, parityPlaceOverCap, status)

	// The refusal is per axis, and the refused axis answers like the plain scan.
	all := parityAllFacets(len(tris))
	plain := NewParityMesh(verts, tris)
	plain.Unfiltered = true
	served := NewParityMesh(verts, tris)
	rng := rand.New(rand.NewPCG(0x65, 0x66))
	for qi, p := range parityIndexQueries(rng, parityIndexFixture{fx.name, verts, tris}) {
		want := answerOf(MeshParityPreparedContext(t.Context(), p, plain, all))
		have := answerOf(MeshParityAllContext(t.Context(), p, served))
		if have != want {
			require.Failf(t, `scan differs`, `query %d: got %+v want %+v`, qi, have, want)
		}
	}
}
