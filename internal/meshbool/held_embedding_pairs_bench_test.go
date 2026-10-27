package meshbool

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// heldRevolvedTube is a closed ring of long thin facets, the shape a revolved
// section holds: an eight-corner profile in (x, r) revolved about the x axis in
// segments steps, then turned 60° about x so its coordinates round. With
// scanAxes the first scan indexes the revolve frame, as a revolve supplies it.
func heldRevolvedTube(segments int, scanAxes bool) HeldRounding {
	profile := [...][2]float64{{0, 2.5}, {1, 2}, {2, 2}, {3, 2}, {4, 2.5}, {3, 3}, {2, 3}, {1, 3}}
	c, s := math.Cos(math.Pi/3), math.Sin(math.Pi/3)
	var h HeldRounding
	for k := range segments {
		for i, p := range profile {
			// Odd profile corners sit half a step on, so no quad is planar.
			theta := 2 * math.Pi * (float64(k) + float64(i%2)/2) / float64(segments)
			y, z := p[1]*math.Cos(theta), p[1]*math.Sin(theta)
			h.Verts = append(h.Verts, r3.NewVec(p[0], c*y-s*z, s*y+c*z))
		}
	}
	n := len(profile)
	for k := range segments {
		next := (k + 1) % segments
		for p := range n {
			q := (p + 1) % n
			a, b, d, e := k*n+p, k*n+q, next*n+q, next*n+p
			h.Tris = append(h.Tris, [3]int{a, b, d}, [3]int{a, d, e})
		}
	}
	if scanAxes {
		h.ScanAxes = [3]r3.Vec{{X: 1}, {Y: c, Z: s}, {Y: -s, Z: c}}
	}
	return h
}

// heldCoarseRevolve is a closed elliptical profile of corners points in
// (x, r) revolved about the x axis in only segments steps, turned like
// heldRevolvedTube. Its facets are long chords around the axis and short
// along the profile, so a facet's box meets few of the boxes in a leaf whose
// hull it meets: in eight steps of 1158 corners about 4% of the leaf-by-leaf
// box tests pass, against 13% on the revolved tooth meshes the bevel sector
// joins hold and about half on heldRevolvedTube.
func heldCoarseRevolve(segments, corners int, scanAxes bool) HeldRounding {
	c, s := math.Cos(math.Pi/3), math.Sin(math.Pi/3)
	var h HeldRounding
	for k := range segments {
		for i := range corners {
			phi := 2 * math.Pi * float64(i) / float64(corners)
			x, r := 2*math.Cos(phi), 3+math.Sin(phi)
			// The profile twists a whole step about the axis and back, so no
			// quad is planar and each facet runs slantwise across its leaf.
			theta := 2 * math.Pi * (float64(k) + math.Sin(phi)) / float64(segments)
			y, z := r*math.Cos(theta), r*math.Sin(theta)
			h.Verts = append(h.Verts, r3.NewVec(x, c*y-s*z, s*y+c*z))
		}
	}
	for k := range segments {
		next := (k + 1) % segments
		for p := range corners {
			q := (p + 1) % corners
			a, b, d, e := k*corners+p, k*corners+q, next*corners+q, next*corners+p
			h.Tris = append(h.Tris, [3]int{a, b, d}, [3]int{a, d, e})
		}
	}
	if scanAxes {
		h.ScanAxes = [3]r3.Vec{{X: 1}, {Y: c, Z: s}, {Y: -s, Z: c}}
	}
	return h
}

// BenchmarkHeldEmbeddingSelfPairs runs the first read-only scan over a mesh
// whose every facet is a query, so parallelClean takes the self-pair path.
func BenchmarkHeldEmbeddingSelfPairs(b *testing.B) {
	for _, bench := range []struct {
		name     string
		scanAxes bool
	}{{"world", false}, {"oriented", true}} {
		h := heldRevolvedTube(1160, bench.scanAxes)
		query := make([]int, len(h.Tris))
		for i := range query {
			query[i] = i
		}
		b.Run(bench.name, func(b *testing.B) {
			for b.Loop() {
				e := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(b.Context()), true)
				clean, err := e.parallelClean(b.Context(), query)
				require.NoError(b, err)
				require.True(b, clean)
			}
		})
	}
}

// BenchmarkHeldEmbeddingSelfPairScan runs cleanSelfPairs alone, on one
// goroutine, over caches and a tree prepared as parallelClean prepares them.
// The tube's leaf-by-leaf box tests mostly pass; the coarse revolve's mostly
// miss.
func BenchmarkHeldEmbeddingSelfPairScan(b *testing.B) {
	for _, bench := range []struct {
		name string
		h    HeldRounding
	}{
		{"tube", heldRevolvedTube(1160, true)},
		{"coarse-world", heldCoarseRevolve(8, 1158, false)},
		{"coarse-oriented", heldCoarseRevolve(8, 1158, true)},
	} {
		e, tree, tasks := heldSelfScan(b, bench.h)
		e.pollErr = b.Context().Err
		e.pollInterval, e.pollRemaining = proofbound.WorkPollInterval/4, proofbound.WorkPollInterval/4
		b.Run(bench.name, func(b *testing.B) {
			for b.Loop() {
				clean, err := e.cleanSelfPairs(tasks, &tree)
				require.NoError(b, err)
				require.True(b, clean)
			}
		})
	}
}
