package meshbool

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// requireHeldProjectedEdgesMatch compares every cached edge certificate with
// the original FloatEdgeSeparates computation, including its abstentions.
func requireHeldProjectedEdgesMatch(t *testing.T, e *heldEmbedding) {
	t.Helper()
	a, b := e.projection(0), e.projection(1)
	for axis := range 3 {
		if !a.stable[axis] || !b.stable[axis] {
			continue
		}
		for edge := range 3 {
			for order, pair := range [][2]*heldProjectedFacet{{a, b}, {b, a}} {
				p, q := pair[0], pair[1]
				want := FloatEdgeSeparates(p.points[axis][edge], p.points[axis][(edge+1)%3],
					p.points[axis][(edge+2)%3], q.points[axis])
				require.Equal(t, want, projectedEdgeSeparates(p, q, axis, edge),
					"axis %d edge %d order %d", axis, edge, order)
			}
		}
	}
}

func TestHeldProjectedEdgesMatchUncachedCertificates(t *testing.T) {
	rng := rand.New(rand.NewPCG(281, 283))
	for sample := range 1000 {
		scale := math.Ldexp(1, rng.IntN(701)-350)
		verts := make([]r3.Vec, 6)
		for i := range verts {
			coord := func() float64 { return scale * float64(rng.IntN(65)-32) / 16 }
			verts[i] = r3.NewVec(coord(), coord(), coord())
		}
		if sample%4 == 0 {
			// Coplanar, collinear and coincident projections must abstain
			// at exactly the same edges as the original filter.
			for i := range verts {
				verts[i].Z = 0
			}
		}
		if sample%7 == 0 {
			verts[2] = verts[1]
		}
		h := HeldRounding{Verts: verts, Tris: [][3]int{{0, 1, 2}, {3, 4, 5}}}
		e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
		requireHeldProjectedEdgesMatch(t, e)
	}

	// The limits bracket Orient2Float's admitted differences; translated
	// triples exercise cancellation near those limits and near collinearity.
	for _, exponent := range []int{-1074, -1000, -301, -300, -299, -1, 0, 249, 250, 251, 1023} {
		scale := math.Ldexp(1, exponent)
		for _, offset := range []float64{0, scale * 16} {
			verts := []r3.Vec{
				{X: offset, Y: offset}, {X: offset + scale, Y: offset + scale},
				{X: offset + scale, Y: math.Nextafter(offset+scale, math.Inf(1))},
				{X: offset - scale, Y: offset}, {X: offset, Y: offset - scale},
				{X: offset - scale, Y: offset - scale},
			}
			h := HeldRounding{Verts: verts, Tris: [][3]int{{0, 1, 2}, {3, 4, 5}}}
			e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			requireHeldProjectedEdgesMatch(t, e)
		}
	}
}

func TestHeldProjectedEdgesInvalidateWithVertex(t *testing.T) {
	h := HeldRounding{
		Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 2, Y: 2}, {X: 3, Y: 2}, {X: 2, Y: 3}},
		Tris:  [][3]int{{0, 1, 2}, {3, 4, 5}},
	}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	requireHeldProjectedEdgesMatch(t, e)
	other := *e.projection(1)
	for _, position := range []r3.Vec{{X: 4, Y: 1, Z: 2}, {X: 1e200}, {X: 1}} {
		e.setVert(1, position)
		require.False(t, e.projectedOK[0])
		require.True(t, e.projectedOK[1])
		require.Equal(t, other, e.projected[1])
		requireHeldProjectedEdgesMatch(t, e)
		fresh := newHeldEmbedding(e.h, proofbound.NewWorkBudget(t.Context()), true)
		require.Equal(t, fresh.projectionsSeparated(0, 1), e.projectionsSeparated(0, 1))
	}
}

func TestHeldSharedProjectionsInvalidateAfterFailedScan(t *testing.T) {
	h := HeldRounding{
		Verts:   []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 0.2, Y: 0.2}, {X: 0.8, Y: 0.2}, {X: 0.2, Y: 0.8}},
		Tris:    [][3]int{{0, 1, 2}, {3, 4, 5}},
		Movable: make([]bool, 6),
	}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	require.NoError(t, e.buildGrid())
	clean, err := e.parallelClean(t.Context(), []int{0, 1})
	require.NoError(t, err)
	require.False(t, clean, "the two coplanar facets overlap")
	for i := range h.Tris {
		require.True(t, e.projectedOK[i], "facet %d was prepared before workers started", i)
	}
	local := e.readOnlyScanCopy(t.Context(), 1)
	require.True(t, &local.projected[0] == &e.projected[0], "workers share the prepared certificates")
	require.True(t, &local.projectedOK[0] == &e.projectedOK[0], "workers share the read-only readiness flags")
	other := e.projected[1]
	e.setVert(1, r3.Vec{X: 4, Y: 1, Z: 2})
	require.False(t, e.projectedOK[0], "repair invalidates the moved vertex's facet")
	require.True(t, e.projectedOK[1], "repair preserves the other facet")
	require.Equal(t, other, e.projected[1])
	requireHeldProjectedEdgesMatch(t, e)
	fresh := newHeldEmbedding(e.h, proofbound.NewWorkBudget(t.Context()), true)
	require.Equal(t, fresh.projectionsSeparated(0, 1), e.projectionsSeparated(0, 1))
}
