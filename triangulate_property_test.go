package decad

import (
	"math"
	"math/big"
	"math/rand"
	"testing"

	"github.com/stretchr/testify/require"
)

// This file is randomized property testing plus a native fuzz target for the
// shared cap triangulator (triangulate.go: triangulate2D and its helpers
// bridgeHole / earClip). It is package-internal — like triangulate_internal_test.go
// — because triangulate2D and Point2 are unexported; it reuses that
// file's loopSignedArea2. This treatment matches the randomized suites revolve,
// the boolean and the fillet already carry.
//
// The triangulator is shared by every Tessellate (prism, cup, boolean), so it
// sees a broad geometric input space, yet PR #39 fixed two real faults in it
// (a hole bridge landing collinear with a hole edge; a genuine boundary vertex
// dropped as a zero-area corner) that had no property test. This suite samples
// the input space the triangulator is SPECIFIED to handle and asserts the
// contract that must hold whichever polygon-with-holes is drawn.
//
// GENERATOR — the guaranteed-valid family (triBuild + triGridRects). An
// axis-aligned outer rectangle [0,w]×[0,h] whose RIGHT edge is densely
// subdivided at each integer v (the other three edges single segments), plus a
// ROW GRID of k ≥ 0 axis-aligned integer rectangular holes: holes are grouped
// into rows, every hole in a row sharing that row's v-band, holes within a row
// separated by a ≥1 u-gap. Consecutive bands may be v-separated OR v-adjacent
// (sharing a v-line), each band carrying a horizontal phase so holes across a
// shared v-line stay u-clear. Each hole is strictly inside the outer (a ≥1 margin
// off every edge). This is provably a valid simple polygon-with-holes: the outer
// is a convex rectangle walked once (collinear samples on one edge break neither
// simplicity nor its positive/CCW area), each hole is a simple rectangle walked
// CW (negative area), and triBuild rejects any draw where a hole touches the
// outer or another hole, so the region outer − Σholes is exactly the region
// triangulate2D promises to reduce.
//
// One deliberate choice keeps the family inside the triangulator's exercised
// domain: the right edge alone carries the collinear samples. Real Tessellate
// boundaries (chordLoop) chord CURVES and leave a straight edge a single
// segment, so subdividing every straight edge to long collinear runs would push
// past that domain (it makes the ear clip stall on a fully-collinear boundary).
// One densely-sampled edge is the honest degenerate-alignment probe without
// leaving the domain.
//
// Because the holes and the right-edge samples share one integer grid, the
// DEGENERATE ALIGNMENTS the triangulator handles occur by construction:
//   - every hole's maximum-u vertex sits at an integer v, so its +u visibility
//     ray lands EXACTLY on an outer vertex (w, v) — the ray-hits-a-vertex /
//     bridge-collinear-with-a-boundary-edge case #39's bridgeHole fix targets;
//   - two holes sharing a v-band make the left hole's bridge run along the top
//     or bottom edge of the right hole — the same-band bridge-collinear-with-a-
//     hole-edge case, whose splice mints genuine collinear corners;
//   - two holes in v-ADJACENT bands share a horizontal v-line, so a +u bridge
//     ray from the lower hole runs collinear THROUGH the upper hole's edge (or
//     vice versa) — the hole-vs-hole shared-v-line case, whose splice makes a
//     genuine boundary corner of one hole coincide with the other hole's bridge
//     anchor; collapsing it would drop that boundary edge and crack the band, so
//     earClip must defer it (a distinct-index collinear corner), not collapse it;
//   - the right edge's collinear run exercises earClip's genuine-collinear-vertex
//     deferral directly, holes or none.
//   - the generator does NOT produce two holes sharing one bridge anchor: its
//     densely-sampled right edge gives every hole its own outer vertex to bridge
//     to. That class is pinned deterministically by
//     TestTriangulate2DHolesSharingOneBridgeAnchor
//     (triangulate_internal_test.go) instead.
//
// PROPERTY ASSERTIONS (triAssert), all on computed geometry:
//  P1 A valid input MUST NOT error and MUST yield at least one triangle.
//  P2 Every triangle is wound CCW with STRICTLY positive area — no zero-area
//     facet shipped. This is one half of the #39 bite: the pre-fix code
//     dissolved a collinear bridge sliver / dropped a collinear corner, either
//     shipping a degenerate facet or losing one. The area/orientation oracle is
//     triSignedArea2, a test-only exact determinant — NEVER the production
//     cross2 the triangulator itself uses to accept ears, which would make the
//     check circular (a facet cross2 wrongly blesses as positive would be
//     blessed here too).
//  P3 Coverage: the triangle areas sum to (outer area − Σ hole areas) within a
//     tight tolerance — no gap, no double cover. The per-facet areas summed are
//     triSignedArea2's, independent of production; the region area is
//     loopSignedArea2, the test's own shoelace.
//  P4 Boundary preservation: every directed boundary edge of every input loop
//     (outer + holes, each unit sub-edge of the dense runs included) appears as
//     a triangle edge exactly once, and its reverse never. This is the sharp
//     #39 bite: dropping a genuine boundary vertex replaces two consecutive
//     boundary sub-edges a→b, b→c with a single a→c that is not a boundary edge,
//     so the b-incident sub-edges vanish from the map and P4 fails — the exact
//     cracked-band fault.
//  P5 Conforming / non-overlapping: no directed edge repeats, and every directed
//     edge is either matched by its reverse (a shared interior edge) or is an
//     input boundary edge (never a dangling interior edge — a crack).
//
// All randomness is drawn from fixed, logged seeds so any failure replays.

// triPropSeed seeds the randomized property test. Fixed and logged for
// reproducibility.
const triPropSeed int64 = 0x7213A6

// triRect is an axis-aligned integer rectangle [x0,x1]×[y0,y1], a hole
// candidate.
type triRect struct{ x0, y0, x1, y1 int }

// triRectsClear reports whether rects a and b are separated by at least gap on
// some axis — no touching, no overlap.
func triRectsClear(a, b triRect, gap int) bool {
	return a.x1+gap <= b.x0 || b.x1+gap <= a.x0 ||
		a.y1+gap <= b.y0 || b.y1+gap <= a.y0
}

// triBuild assembles the guaranteed-valid polygon-with-holes: the outer
// rectangle [0,w]×[0,h] as loops[0] (CCW, its right edge subdivided at each
// integer v) and the accepted holes as CW loops. It returns ok=false — no
// panic, no error — when w/h are too small or a hole is out of bounds or not
// clear of the outer / another hole, so both the random generator and the fuzz
// decoder can hand it arbitrary candidates and keep only the valid family.
func triBuild(w, h int, holeRects []triRect) (pts []Point2, outer []int, holes [][]int, ok bool) {
	if w < 4 || h < 4 {
		return nil, nil, nil, false
	}
	add := func(u, v int) int {
		pts = append(pts, Point2{U: float64(u), V: float64(v)})
		return len(pts) - 1
	}

	// Outer walked CCW: bottom (0,0)→(w,0) single segment, right edge subdivided
	// (w,0),(w,1),…,(w,h), top (w,h)→(0,h) single, left (0,h)→(0,0) single. Only
	// the right edge carries collinear samples — every hole's +u ray hits one.
	outer = append(outer, add(0, 0))
	for v := 0; v <= h; v++ {
		outer = append(outer, add(w, v))
	}
	outer = append(outer, add(0, h))

	// Validate and place each hole: strictly inside (≥1 margin off every outer
	// edge) and clear of every already-placed hole by ≥1.
	var placed []triRect
	for _, r := range holeRects {
		if r.x1 <= r.x0 || r.y1 <= r.y0 {
			return nil, nil, nil, false
		}
		if r.x0 < 1 || r.y0 < 1 || r.x1 > w-1 || r.y1 > h-1 {
			return nil, nil, nil, false
		}
		for _, q := range placed {
			if !triRectsClear(r, q, 1) {
				return nil, nil, nil, false
			}
		}
		placed = append(placed, r)
		// CW order (negative signed area): (x0,y0),(x0,y1),(x1,y1),(x1,y0).
		hole := []int{
			add(r.x0, r.y0), add(r.x0, r.y1), add(r.x1, r.y1), add(r.x1, r.y0),
		}
		holes = append(holes, hole)
	}
	return pts, outer, holes, true
}

// triGridRects builds the row-grid hole set inside [0,w]×[0,h]: rows stacked in
// v, each row a band of holes that share the band and are separated in u by ≥1
// gaps. Consecutive bands may be v-SEPARATED (a ≥1 v-gap) OR v-ADJACENT (gap 0),
// so a hole's top edge sits at the very v of the hole below it in the next band
// — the shared-v-line configuration that makes a +u bridge ray run collinear
// THROUGH another hole's edge (the fu6 fault). To keep such configurations valid
// yet exercised, each band carries a horizontal PHASE offset so a hole and the
// hole below it, sharing a v-line, are offset in u and stay u-clear rather than
// touching; any residual overlap triBuild rejects, so the family stays the
// guaranteed-valid one. next(n) returns a value in [0,n) and is the sole entropy
// source, so the very same construction serves the seeded generator (next
// drawing from an rng) and the fuzz decoder (next drawing from the input bytes).
func triGridRects(next func(n int) int, w, h int) []triRect {
	var rects []triRect
	v := 1
	for v <= h-2 && len(rects) < 7 {
		bandH := 1 + next(2) // 1..2
		if v+bandH > h-1 {
			break
		}
		y0, y1 := v, v+bandH
		u := 1 + next(3) // band phase 0..2: staggers holes across a shared v-line
		for u <= w-2 {
			holeW := 1 + next(3) // 1..3
			if u+holeW > w-1 {
				break
			}
			if next(3) != 0 { // place ~2/3 of the time, else leave a gap column
				rects = append(rects, triRect{u, y0, u + holeW, y1})
			}
			u = u + holeW + 1 // step past the hole and a ≥1 gap
		}
		v = y1 + next(3) // next band 0..2 above: gap 0 shares this band's v-line
	}
	return rects
}

// triSignedArea2 is the property suite's OWN area/orientation oracle: an exact
// math/big.Rat determinant ½·((b−a)×(c−a)) over the point coordinates. It is
// deliberately independent of the production cross2 that triangulate2D uses to
// decide which ears are valid — judging a facet with cross2 would be circular,
// so a facet the triangulator wrongly accepted as positive-area could never be
// caught by P2/P3. The polygon-with-holes family is drawn on an integer grid,
// so SetFloat64 is exact and the sign is decided with zero float ambiguity: a
// truly degenerate facet reads exactly 0, never a near-zero positive.
func triSignedArea2(a, b, c Point2) *big.Rat {
	ax := new(big.Rat).SetFloat64(a.U)
	ay := new(big.Rat).SetFloat64(a.V)
	bx := new(big.Rat).SetFloat64(b.U)
	by := new(big.Rat).SetFloat64(b.V)
	cx := new(big.Rat).SetFloat64(c.U)
	cy := new(big.Rat).SetFloat64(c.V)
	t1 := new(big.Rat).Mul(new(big.Rat).Sub(bx, ax), new(big.Rat).Sub(cy, ay))
	t2 := new(big.Rat).Mul(new(big.Rat).Sub(cx, ax), new(big.Rat).Sub(by, ay))
	d := new(big.Rat).Sub(t1, t2)
	return d.Mul(d, big.NewRat(1, 2))
}

// triAssert runs the whole property contract (P1..P5) on a triangulation of the
// given valid polygon-with-holes.
func triAssert(t *testing.T, pts []Point2, outer []int, holes [][]int) {
	t.Helper()

	// Sanity on the generated input itself: outer CCW, every hole CW. A generator
	// bug would otherwise mask a triangulator bug.
	require.Positive(t, loopSignedArea2(pts, outer), `generated outer is CCW`)
	for i, hole := range holes {
		require.Negative(t, loopSignedArea2(pts, hole), `generated hole %d is CW`, i)
	}

	loops := append([][]int{outer}, holes...)
	tris, err := triangulate2DContext(t.Context(), pts, loops)

	// P1: a valid input must triangulate into at least one facet.
	require.NoError(t, err, `valid polygon-with-holes must triangulate`)
	require.NotEmpty(t, tris)

	// P2 + P3: every facet CCW and strictly non-degenerate; areas sum to the
	// region area (outer − Σ holes). Both use triSignedArea2 (test-only exact
	// determinant), NEVER the production cross2 the triangulator accepts ears
	// with — so the oracle is not circular.
	areaSum := new(big.Rat)
	for _, tr := range tris {
		s := triSignedArea2(pts[tr[0]], pts[tr[1]], pts[tr[2]])
		require.Positive(t, s.Sign(), `facet %v is CCW and non-degenerate`, tr)
		areaSum.Add(areaSum, s)
	}
	area, _ := areaSum.Float64()
	want := loopSignedArea2(pts, outer) // positive
	for _, hole := range holes {
		want += loopSignedArea2(pts, hole) // negative
	}
	require.InDelta(t, want, area, 1e-9*(1+math.Abs(want)), `the fan covers exactly the region`)

	// Directed-edge multiset of the triangulation.
	de := map[[2]int]int{}
	for _, tr := range tris {
		for k := range 3 {
			de[[2]int{tr[k], tr[(k+1)%3]}]++
		}
	}

	// The set of input boundary directed edges (outer + every hole, walked in the
	// loop's own sense).
	boundary := map[[2]int]bool{}
	addBoundary := func(loop []int) {
		n := len(loop)
		for i := range loop {
			boundary[[2]int{loop[i], loop[(i+1)%n]}] = true
		}
	}
	addBoundary(outer)
	for _, hole := range holes {
		addBoundary(hole)
	}

	// P4: every input boundary edge kept exactly once, never reversed.
	for e := range boundary {
		require.Equal(t, 1, de[e], `boundary edge %v kept once`, e)
		require.Equal(t, 0, de[[2]int{e[1], e[0]}], `boundary edge %v not reversed`, e)
	}

	// P5: no directed edge repeats (consistent orientation, non-overlapping), and
	// every directed edge is either shared by two triangles (interior, its
	// reverse present) or an input boundary edge — never a dangling interior edge.
	for e, c := range de {
		require.Equal(t, 1, c, `directed edge %v appears once`, e)
		if de[[2]int{e[1], e[0]}] == 0 {
			require.Truef(t, boundary[e], `directed edge %v is unpaired yet not a boundary edge (a crack)`, e)
		}
	}
}

// TestTriangulate2DProperty samples the guaranteed-valid family at scale from a
// fixed seed and asserts the full contract on every draw.
func TestTriangulate2DProperty(t *testing.T) {
	t.Logf("triPropSeed=%#x", triPropSeed)
	rng := rand.New(rand.NewSource(triPropSeed))

	const cases = 800
	built, withHoles := 0, 0
	for range cases {
		w := 6 + rng.Intn(11) // 6..16
		h := 6 + rng.Intn(11) // 6..16
		rects := triGridRects(rng.Intn, w, h)
		pts, outer, holes, ok := triBuild(w, h, rects)
		if !ok {
			continue
		}
		built++
		if len(holes) > 0 {
			withHoles++
		}
		triAssert(t, pts, outer, holes)
	}
	// The generator must actually exercise the triangulator — most draws build,
	// and a healthy share carry holes (the interesting bridging path).
	require.Greater(t, built, cases/2, `most draws build a valid polygon`)
	require.Greater(t, withHoles, cases/4, `many draws carry holes`)
}

// TestTriangulate2DBridgeCollinearFamily pins deterministic members of the exact
// class PR #39 fixed. Each is built through triBuild (collinear right edge), so
// the degenerate alignment is present by construction; the comment on each states
// how P4/P2 bite if the bridgeHole / earClip fix were reverted.
func TestTriangulate2DBridgeCollinearFamily(t *testing.T) {
	cases := []struct {
		name  string
		w, h  int
		holes []triRect
	}{
		{
			// Centred hole: its max-u vertices at v=3 and v=7 send +u rays onto the
			// outer right-edge vertices (10,3),(10,7). Reverting earClip's genuine-
			// collinear deferral would collapse an outer collinear vertex and drop a
			// boundary sub-edge — P4 fails; reverting bridgeHole's max-u-endpoint
			// rule would dissolve a bridge sliver into a zero-area facet — P2 fails.
			name: "centred-hole-ray-hits-outer-vertex", w: 10, h: 10,
			holes: []triRect{{3, 3, 7, 7}},
		},
		{
			// Two holes sharing the v-band 3..5: bridging the left hole runs its
			// bridge collinear with the right hole's top/bottom edge — the same-band
			// bridge-collinear-with-a-hole-edge case. A dissolved sliver or dropped
			// boundary vertex there cracks the band (P4/P2).
			name: "two-holes-shared-v-band", w: 16, h: 8,
			holes: []triRect{{2, 3, 4, 5}, {8, 3, 11, 5}},
		},
		{
			// No holes: the right edge's collinear run alone exercises earClip's
			// deferral. Dropping any genuine collinear corner drops a boundary
			// sub-edge (P4).
			name: "no-holes-collinear-right-edge", w: 8, h: 6,
		},
		{
			// A two-row grid: each row a shared v-band (2..4 and 8..10), the rows
			// separated by a v-gap so no hole shares a v-line with a hole in the other
			// row. Every hole's +u ray still lands on an outer right-edge vertex.
			name: "two-row-grid", w: 16, h: 14,
			holes: []triRect{{2, 2, 4, 4}, {8, 2, 11, 4}, {2, 8, 4, 10}, {8, 8, 11, 10}},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			pts, outer, holes, ok := triBuild(tc.w, tc.h, tc.holes)
			require.True(t, ok, `case builds a valid polygon`)
			triAssert(t, pts, outer, holes)
		})
	}
}

// TestTriangulate2DHoleHoleCollinear pins the hole-vs-hole shared-v-line fault
// (the #40 fuzzer's calibration repro): an outer 13×13 rectangle with two
// rectangular holes {2,1,3,2} and {4,2,7,3} whose bands are v-ADJACENT — the
// first hole's top edge and the second's bottom edge both at v=2. The first
// hole's maximum-u corner (3,2) casts its +u bridge ray collinear THROUGH the
// second hole's bottom edge, and the bridge lands on the second hole's reflex
// corner (4,2). That corner is a genuine boundary vertex of the second hole AND
// the first hole's bridge anchor; collapsing it as a zero-area corner would drop
// the second hole's boundary edge (7,2)→(4,2) and overlap triangles. earClip
// must instead DEFER it — a distinct-index collinear corner is not a bridge stub
// — so every boundary edge survives and the fan is non-overlapping. This is
// distinct from #39's hole-vs-outer collinear bridge. triAssert judges with the
// test-only exact oracles (triSignedArea2, the P4/P5 edge multiset), never the
// production cross2.
func TestTriangulate2DHoleHoleCollinear(t *testing.T) {
	pts, outer, holes, ok := triBuild(13, 13, []triRect{{2, 1, 3, 2}, {4, 2, 7, 3}})
	require.True(t, ok, `the hole-hole shared-v-line repro builds a valid polygon`)
	triAssert(t, pts, outer, holes)
}

// FuzzTriangulate2D decodes fuzz bytes into a member of the guaranteed-valid
// row-grid family (via triGridRects driven by a byte cursor) and asserts the
// same P1..P5 contract. Any input that cannot form a valid polygon is skipped
// (return), so the target only ever judges valid input — the triangulator's
// specified domain.
func FuzzTriangulate2D(f *testing.F) {
	// Seed corpus: a spread of w/h and enough trailing entropy for a few rows of
	// holes. Layout: [w, h, then bytes consumed by triGridRects].
	f.Add([]byte{10, 10, 3, 3, 4, 4})
	f.Add([]byte{16, 8, 1, 2, 1, 2, 1, 2, 1, 2})
	f.Add([]byte{8, 6})
	f.Add([]byte{16, 14, 1, 2, 2, 1, 2, 2, 1, 2, 2, 1, 2, 2, 1, 2, 2})

	f.Fuzz(func(t *testing.T, data []byte) {
		if len(data) < 2 {
			return
		}
		w := 6 + int(data[0]%11) // 6..16
		h := 6 + int(data[1]%11) // 6..16

		// next(n) walks the remaining bytes; past the end it yields 0, which
		// triGridRects reads as "smallest choice / stop", so short inputs simply
		// make fewer holes.
		cur := 2
		next := func(n int) int {
			if n <= 0 {
				return 0
			}
			var b int
			if cur < len(data) {
				b = int(data[cur])
				cur++
			}
			return b % n
		}
		rects := triGridRects(next, w, h)

		pts, outer, holes, ok := triBuild(w, h, rects)
		if !ok {
			return
		}
		triAssert(t, pts, outer, holes)
	})
}
