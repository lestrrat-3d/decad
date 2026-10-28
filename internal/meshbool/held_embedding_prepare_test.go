package meshbool

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// heldPrepareGrid is a tilted side by side grid of 2·side² facets; 48 gives
// 4,608, enough for parallelClean to prepare its caches on several
// goroutines. extra appends one facet: "crossing" pierces the first cell,
// "collapsed" lies far off the grid with three collinear corners, so only the
// collapse check refuses it.
func heldPrepareGrid(t *testing.T, side int, extra string) HeldRounding {
	t.Helper()
	tilt, err := r3.NewFrame(r3.NewVec(0.25, -0.5, 0.125),
		r3.NewVec(math.Cos(0.3), math.Sin(0.3), 0.1), r3.NewVec(-math.Sin(0.3), math.Cos(0.3), 0.2))
	require.NoError(t, err)
	var h HeldRounding
	for y := range side + 1 {
		for x := range side + 1 {
			h.Verts = append(h.Verts, tilt.ToWorld(r3.Vec{X: float64(x) / 3, Y: float64(y) / 3}))
		}
	}
	for y := range side {
		for x := range side {
			a := y*(side+1) + x
			b := a + side + 1
			h.Tris = append(h.Tris, [3]int{a, a + 1, b}, [3]int{a + 1, b + 1, b})
		}
	}
	n := len(h.Verts)
	switch extra {
	case "crossing":
		h.Verts = append(h.Verts, tilt.ToWorld(r3.Vec{X: 0.05, Y: 0.05, Z: -1}),
			tilt.ToWorld(r3.Vec{X: 0.25, Y: 0.05, Z: 1}), tilt.ToWorld(r3.Vec{X: 0.05, Y: 0.25, Z: 1}))
		h.Tris = append(h.Tris, [3]int{n, n + 1, n + 2})
	case "collapsed":
		h.Verts = append(h.Verts, r3.Vec{X: 100, Y: 100, Z: 100},
			r3.Vec{X: 101, Y: 100, Z: 100}, r3.Vec{X: 102, Y: 100, Z: 100})
		h.Tris = append(h.Tris, [3]int{n, n + 1, n + 2})
	}
	h.Movable = make([]bool, len(h.Verts))
	return h
}

// TestParallelCleanPreparedCachesMatchSerialScan checks that parallelClean,
// which prepares every facet's projection, world box, openness and (for a
// self scan) corner cones on several goroutines and builds an oriented tree
// beside them, answers as the serial scan does, and leaves each prepared
// cache holding the value a fresh lazy reading gives.
func TestParallelCleanPreparedCachesMatchSerialScan(t *testing.T) {
	t.Parallel()
	for _, extra := range []string{"none", "crossing", "collapsed"} {
		for _, axes := range [][3]r3.Vec{{}, {{X: 2, Y: 3, Z: 5}, {X: -3, Y: 2}, {X: 1, Z: -2}}} {
			// every queries each facet, a self scan; otherwise the odd facets
			// and the extra one are queried.
			for _, every := range []bool{true, false} {
				t.Run(fmt.Sprintf("%s/oriented=%v/every=%v", extra, axes != [3]r3.Vec{}, every), func(t *testing.T) {
					h := heldPrepareGrid(t, 48, extra)
					h.ScanAxes = axes
					var query []int
					mask := make([]bool, len(h.Tris))
					for i := range h.Tris {
						if every || i%2 == 1 || i == len(h.Tris)-1 {
							query = append(query, i)
							mask[i] = true
						}
					}
					serial := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
					require.NoError(t, serial.buildGrid())
					want, err := serial.serialClean(query, mask)
					require.NoError(t, err)
					require.Equal(t, extra == "none", want, "the serial scan reads the fixture")

					e := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), true)
					got, err := e.parallelClean(t.Context(), query)
					require.NoError(t, err)
					require.Equal(t, want, got)

					fresh := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), true)
					for i := range h.Tris {
						require.True(t, e.projectedOK[i], "facet %d projection was prepared", i)
						if e.projected[i] != *fresh.projection(i) {
							require.Failf(t, "prepared projection differs", "facet %d", i)
						}
						require.True(t, e.boxOK[i], "facet %d box was prepared", i)
						require.Equal(t, TriBox(h.Verts, h.Tris[i]), e.box[i], "facet %d box", i)
						if !every {
							continue
						}
						for k := range 3 {
							require.True(t, e.coneOK[i][k], "facet %d corner %d cone was prepared", i, k)
							require.True(t, heldConeBoxEncloses(e.coneBox(i, k), fresh.coneBox(i, k)),
								"facet %d corner %d cone bounds", i, k)
						}
					}
				})
			}
		}
	}
}

func heldConeBoxEncloses(outer, inner [2]r3.Vec) bool {
	for _, axis := range [][4]float64{
		{outer[0].X, inner[0].X, inner[1].X, outer[1].X},
		{outer[0].Y, inner[0].Y, inner[1].Y, outer[1].Y},
		{outer[0].Z, inner[0].Z, inner[1].Z, outer[1].Z},
	} {
		if axis[0] > axis[1] || axis[2] > axis[3] {
			return false
		}
	}
	return true
}
