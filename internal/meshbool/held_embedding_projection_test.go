package meshbool

import (
	"math"
	"sync"
	"testing"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Frozen pre-change representation and loops provide an independent scalar oracle.
type originalProjectedFacet struct {
	points [3][3][2]float64
	signs  [3][3]int8
	stable [3]bool
}

func (p *originalProjectedFacet) refresh(h HeldRounding, i int) {
	tri := h.Tris[i]
	for axis, axes := range [...][2]int{{0, 1}, {2, 0}, {1, 2}} {
		p.stable[axis] = true
		for k, v := range tri {
			point := h.Verts[v]
			p.points[axis][k] = [2]float64{AxisCoord(point, axes[0]), AxisCoord(point, axes[1])}
			for _, coord := range p.points[axis][k] {
				magnitude := proof.Abs(coord)
				if !(magnitude == 0 || magnitude >= 1e-100 && magnitude <= 1e100) {
					p.stable[axis] = false
				}
			}
		}
		// Keep every subtraction and product in Orient2Float away from
		// overflow and underflow; other scales use the exact pair checks.
		if !p.stable[axis] {
			continue
		}
		for k := range 3 {
			p.signs[axis][k] = int8(Orient2Float(p.points[axis][k],
				p.points[axis][(k+1)%3], p.points[axis][(k+2)%3]))
		}
	}
}

func originalProjectedEdgeSeparates(a, b *originalProjectedFacet, axis, edge int) bool {
	sign := int(a.signs[axis][edge])
	if sign == 0 {
		return false
	}
	start := a.points[axis][edge]
	end := a.points[axis][(edge+1)%3]
	for _, point := range b.points[axis] {
		if Orient2Float(start, end, point) != -sign {
			return false
		}
	}
	return true
}

func originalProjectionsSeparated(a, b *originalProjectedFacet) bool {
	for axis := range 3 {
		if !a.stable[axis] || !b.stable[axis] {
			continue
		}
		for k := range 3 {
			if originalProjectedEdgeSeparates(a, b, axis, k) || originalProjectedEdgeSeparates(b, a, axis, k) {
				return true
			}
		}
	}
	return false
}

func requireProjectionMatches(t *testing.T, got *heldProjectedFacet, want *originalProjectedFacet) {
	t.Helper()
	for axis := range 3 {
		require.Equal(t, want.stable[axis], got.stable[axis], "stable axis %d", axis)
		for k := range 3 {
			require.Equal(t, want.signs[axis][k], got.signs[axis][k], "sign axis %d edge %d", axis, k)
			point := got.point(axis, k)
			for c := range 2 {
				require.Equal(t, math.Float64bits(want.points[axis][k][c]), math.Float64bits(point[c]),
					"coordinate axis %d vertex %d component %d", axis, k, c)
			}
		}
	}
}

func projectionFixtures() [][3]r3.Vec {
	negativeZero := math.Copysign(0, -1)
	payloadNaN := math.Float64frombits(0x7ff8000000000123)
	out := [][3]r3.Vec{
		{{}, {X: 1}, {Y: 1}},
		{{}, {Y: 1}, {X: 1}},
		{{X: negativeZero}, {Y: negativeZero}, {Z: negativeZero}},
		{{X: 1, Y: 2, Z: 3}, {X: 2, Y: 4, Z: 6}, {X: 3, Y: 6, Z: 9}},
		{{X: 1}, {X: 1, Y: math.SmallestNonzeroFloat64}, {X: 1, Y: 1}},
		{{X: 1e-100}, {Y: 1e-100}, {Z: 1e-100}},
		{{X: math.Nextafter(1e-100, 0)}, {Y: 1}, {Z: 1}},
		{{X: 1e100}, {Y: -1e100}, {Z: 1e100}},
		{{X: math.Nextafter(1e100, math.Inf(1))}, {Y: 1}, {Z: 1}},
		{{X: math.Inf(1)}, {Y: math.Inf(-1)}, {Z: payloadNaN}},
	}
	for i := 1; i <= 12; i++ {
		x := float64(i)
		out = append(out, [3]r3.Vec{{X: x, Y: -x, Z: x / 2}, {X: x + 1, Y: x / 2, Z: -x}, {X: x, Y: math.Nextafter(x, math.Inf(1)), Z: x + 1}})
	}
	return out
}

func TestHeldProjectionStorageMatchesOriginal(t *testing.T) {
	fixtures := projectionFixtures()
	h := HeldRounding{}
	for _, tri := range fixtures {
		n := len(h.Verts)
		h.Verts = append(h.Verts, tri[:]...)
		h.Tris = append(h.Tris, [3]int{n, n + 1, n + 2})
	}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	want := make([]originalProjectedFacet, len(h.Tris))
	for i := range h.Tris {
		want[i].refresh(h, i)
		requireProjectionMatches(t, e.projection(i), &want[i])
	}
	for i := range h.Tris {
		for j := range h.Tris {
			require.Equal(t, originalProjectionsSeparated(&want[i], &want[j]), e.projectionsSeparated(i, j), "pair %d %d", i, j)
			for axis := range 3 {
				for edge := range 3 {
					require.Equal(t, originalProjectedEdgeSeparates(&want[i], &want[j], axis, edge),
						projectedEdgeSeparates(e.projection(i), e.projection(j), axis, edge), "pair %d %d axis %d edge %d", i, j, axis, edge)
				}
			}
		}
	}
}

func TestHeldProjectionStorageInvalidation(t *testing.T) {
	h := HeldRounding{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 2}, {X: 1, Z: 2}, {Y: 1, Z: 2}}, Tris: [][3]int{{0, 1, 2}, {3, 4, 5}}}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	want := [2]originalProjectedFacet{}
	for i := range want {
		want[i].refresh(h, i)
		e.projection(i)
	}
	// A cached projection is a snapshot until its incident facet is invalidated.
	h.Verts[0] = r3.Vec{X: 4}
	requireProjectionMatches(t, e.projection(0), &want[0])
	h.Verts[0] = r3.Vec{}
	for _, point := range []r3.Vec{{X: 4}, {X: math.Inf(1)}, {Y: math.Copysign(0, -1)}, {Z: 2}} {
		e.setVert(0, point)
		require.False(t, e.projectedOK[0])
		require.True(t, e.projectedOK[1])
		want[0].refresh(h, 0)
		requireProjectionMatches(t, e.projection(0), &want[0])
		requireProjectionMatches(t, e.projection(1), &want[1])
	}
}

func TestHeldProjectionStorageWorkerIsolation(t *testing.T) {
	h := HeldRounding{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 2}, {X: 1, Z: 2}, {Y: 1, Z: 2}}, Tris: [][3]int{{0, 1, 2}, {3, 4, 5}}}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	workers := []*heldEmbedding{e.readOnlyScanCopy(t.Context(), 2), e.readOnlyScanCopy(t.Context(), 2)}
	var wg sync.WaitGroup
	for _, worker := range workers {
		wg.Go(func() {
			for range 64 {
				worker.projectionsSeparated(0, 1)
			}
		})
	}
	wg.Wait()
	for i := range h.Tris {
		require.False(t, e.projectedOK[i], "parent cache %d", i)
		var want originalProjectedFacet
		want.refresh(h, i)
		for _, worker := range workers {
			requireProjectionMatches(t, worker.projection(i), &want)
		}
	}
	workers[0].projected[0].points[0].X = 99
	var want originalProjectedFacet
	want.refresh(h, 0)
	requireProjectionMatches(t, workers[1].projection(0), &want)
	requireProjectionMatches(t, e.projection(0), &want)
}

func TestHeldProjectionStorageSize(t *testing.T) {
	// Sizeof measures layout only; no unsafe memory access is used.
	before, after := unsafe.Sizeof(originalProjectedFacet{}), unsafe.Sizeof(heldProjectedFacet{})
	require.Equal(t, uintptr(72), before-after)
	t.Logf("projected facet bytes: original=%d compact=%d", before, after)
}
