package loftmesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests gridScan directly, which no exported entry reaches on
// demand: newPairScan hands the audit the grid only where it does less work
// than the sweep. FALSIFICATION LOG. Each leg was broken in
// loft_audit_grid.go, this test re-run, and the change reverted; each went
// RED:
//
//   - visit reported a pair from every cell it shared, without the
//     lower-corner cell check: every fixture with a box spanning two cells
//     reported pairs more than once.
//   - the check read boxA's own lower corner instead of the two corners'
//     maximum: pairs whose second box starts in a later cell went missing.

// gridFixture is a named set of boxes.
type gridFixture struct {
	name  string
	boxes [][2]r3.Vec
}

func gridBox(a, b r3.Vec) [2]r3.Vec {
	return [2]r3.Vec{
		r3.NewVec(math.Min(a.X, b.X), math.Min(a.Y, b.Y), math.Min(a.Z, b.Z)),
		r3.NewVec(math.Max(a.X, b.X), math.Max(a.Y, b.Y), math.Max(a.Z, b.Z)),
	}
}

func gridFixtures() []gridFixture {
	rng := rand.New(rand.NewPCG(0x6a1d, 0x3c55))
	vec := func(scale float64) r3.Vec {
		return r3.NewVec(rng.Float64()*scale, rng.Float64()*scale, rng.Float64()*scale)
	}
	var out []gridFixture

	var mixed [][2]r3.Vec
	for i := range 600 {
		size := 0.05
		if i%50 == 0 {
			size = 4 // a few boxes spanning most of the grid
		}
		lo := vec(10)
		mixed = append(mixed, gridBox(lo, lo.Add(vec(size))))
	}
	out = append(out, gridFixture{"mixed sizes", mixed})

	// A tall tube's facets: every box spans the full height.
	var tube [][2]r3.Vec
	const n = 400
	for k := range n {
		a := 2 * math.Pi * float64(k) / n
		b := 2 * math.Pi * float64(k+1) / n
		tube = append(tube, gridBox(r3.NewVec(math.Cos(a), math.Sin(a), 0), r3.NewVec(math.Cos(b), math.Sin(b), 100)))
	}
	out = append(out, gridFixture{"tall tube", tube})

	// A helical strip: radial rulings from radius 2 to 3 over four turns.
	var helix [][2]r3.Vec
	k := 1.5 / (2 * math.Pi)
	for j := range 1024 {
		a := 2 * math.Pi * float64(j) / 256
		b := 2 * math.Pi * float64(j+1) / 256
		p := r3.NewVec(2*math.Cos(a), 2*math.Sin(a), k*a)
		q := r3.NewVec(3*math.Cos(b), 3*math.Sin(b), k*b)
		helix = append(helix, gridBox(p, q))
	}
	out = append(out, gridFixture{"helical strip", helix})

	// Boxes on an exact lattice, touching at shared faces, edges and
	// corners, and flat in Z: zero span on one axis.
	var lattice [][2]r3.Vec
	for x := range 12 {
		for y := range 12 {
			lattice = append(lattice, gridBox(r3.NewVec(float64(x), float64(y), 0), r3.NewVec(float64(x+1), float64(y+1), 0)))
		}
	}
	out = append(out, gridFixture{"flat touching lattice", lattice})

	// Repeated identical boxes.
	var same [][2]r3.Vec
	for range 30 {
		same = append(same, gridBox(r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1)))
	}
	out = append(out, gridFixture{"identical boxes", same})
	return out
}

func TestGridScanEnumeratesEveryOverlappingPairOnce(t *testing.T) {
	t.Parallel()
	for _, fx := range gridFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			members := make([]int, len(fx.boxes))
			for i := range members {
				members[i] = i
			}
			g, ok := newGridScan(fx.boxes, members, math.MaxUint64)
			require.True(t, ok)
			seen := map[[2]int]int{}
			scanned, exceeded, err := g.visit(proofbound.NewWorkBudget(t.Context()), math.MaxUint64, func(i, j int) {
				require.Less(t, i, j)
				seen[[2]int{i, j}]++
			})
			require.NoError(t, err)
			require.False(t, exceeded)
			require.Equal(t, g.work, scanned, "a complete visit counts exactly the work the grid states")

			want := 0
			for i := range fx.boxes {
				for j := i + 1; j < len(fx.boxes); j++ {
					if !meshbool.BoxesOverlap(fx.boxes[i], fx.boxes[j]) {
						continue
					}
					want++
					require.Equal(t, 1, seen[[2]int{i, j}], "pair (%d, %d) must be reported exactly once", i, j)
				}
			}
			require.Len(t, seen, want, "no pair whose boxes are apart may be reported")

			if g.work > 0 {
				_, exceeded, err = g.visit(proofbound.NewWorkBudget(t.Context()), g.work-1, func(int, int) {})
				require.NoError(t, err)
				require.True(t, exceeded, "a limit one below the work refuses")
			}
		})
	}
}

// TestPairScanPicksTheCheaperEnumeration pins newPairScan's choice on the
// two shapes it was built for: a tall tube and a helical strip both overlap
// on every axis a sweep could take, so the sweep scans nearly every pair;
// the grid's work is a small share of that and is the one handed back.
func TestPairScanPicksTheCheaperEnumeration(t *testing.T) {
	t.Parallel()
	for _, fx := range gridFixtures() {
		if fx.name != "tall tube" && fx.name != "helical strip" {
			continue
		}
		t.Run(fx.name, func(t *testing.T) {
			members := make([]int, len(fx.boxes))
			for i := range members {
				members[i] = i
			}
			sweep := newSweepOrder(fx.boxes, members)
			g, ok := newGridScan(fx.boxes, members, math.MaxUint64)
			require.True(t, ok)
			require.Less(t, g.work*3, sweep.work())
			_, isGrid := newPairScan(fx.boxes, members, math.MaxUint64).(gridScan)
			require.True(t, isGrid)
			t.Logf("%s: sweep scans %d pairs, the grid works %d", fx.name, sweep.work(), g.work)
		})
	}
}
