package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// loopSignedArea2 is the shoelace area of a loop of point indices, positive
// when the loop winds counter-clockwise.
func loopSignedArea2(pts []Point2, loop []int) float64 {
	a := 0.0
	n := len(loop)
	for i := range loop {
		p, q := pts[loop[i]], pts[loop[(i+1)%n]]
		a += p.U*q.V - q.U*p.V
	}
	return a / 2
}

// TestTriangulate2DBridgeCollinear pins the bridge-collinear rim band: a rounded
// square (a 20×20 tunnel grown outward by 5, its corners arcs of radius 5) as
// the outer boundary, with the sharp 20×20 tunnel as a clockwise hole — the
// post-rim band of a rectangular inward cup. The tunnel's maximum-u corners sit
// at exactly the v of the outer's vertical right edge, so the +u visibility ray
// lands on an outer vertex and the natural bridge runs collinear with a tunnel
// edge. The triangulator must produce a valid counter-clockwise fan that covers
// the whole band and keeps every boundary edge, rather than dropping a boundary
// vertex into a cracked band.
func TestTriangulate2DBridgeCollinear(t *testing.T) {
	var pts []Point2
	add := func(u, v float64) int { pts = append(pts, Point2{U: u, V: v}); return len(pts) - 1 }

	// Outer: the tunnel (40,20)-(60,40) offset outward by 5. Straight edges at
	// u=35/65 and v=15/45; corner arcs of radius 5 centred on the tunnel
	// corners, chorded into four segments each. Walked counter-clockwise.
	const r = 5.0
	// arc appends the interior + final samples of a corner arc (k = 1..seg),
	// excluding the start point, which the previous vertex already supplied.
	arc := func(cu, cv, a0, a1 float64, seg int) []int {
		var out []int
		for k := 1; k <= seg; k++ {
			a := a0 + (a1-a0)*float64(k)/float64(seg)
			out = append(out, add(cu+r*math.Cos(a), cv+r*math.Sin(a)))
		}
		return out
	}
	var outer []int
	outer = append(outer, add(65, 20), add(65, 40))                // right edge
	outer = append(outer, arc(60, 40, 0, math.Pi/2, 4)...)         // top-right corner -> (60,45)
	outer = append(outer, add(40, 45))                             // top edge
	outer = append(outer, arc(40, 40, math.Pi/2, math.Pi, 4)...)   // top-left corner -> (35,40)
	outer = append(outer, add(35, 20))                             // left edge
	outer = append(outer, arc(40, 20, math.Pi, 3*math.Pi/2, 4)...) // bottom-left -> (40,15)
	outer = append(outer, add(60, 15))                             // bottom edge
	// bottom-right corner, its final sample dropped: it would coincide with the
	// start vertex (65,20) and close the loop with a zero-length edge.
	br := arc(60, 20, 3*math.Pi/2, 2*math.Pi, 4)
	outer = append(outer, br[:len(br)-1]...)

	// Hole: the sharp 20×20 tunnel, clockwise.
	hole := []int{add(60, 20), add(40, 20), add(40, 40), add(60, 40)}

	require.Positive(t, loopSignedArea2(pts, outer), `outer is counter-clockwise`)
	require.Negative(t, loopSignedArea2(pts, hole), `tunnel hole is clockwise`)

	tris, err := triangulate2DContext(t.Context(), pts, [][]int{outer, hole})
	require.NoError(t, err)
	require.NotEmpty(t, tris)

	// Every triangle is wound counter-clockwise (no inverted or zero-area
	// facet), and the triangles together cover exactly the band area. The facet
	// oracle is triSignedArea2 (a test-only exact math/big.Rat determinant),
	// NEVER the production cross2 that triangulate2D accepts ears with — judging
	// a facet with cross2 would be circular, so a zero-area facet a cross2
	// regression let through could never be caught here.
	areaSum := new(big.Rat)
	for _, tr := range tris {
		s := triSignedArea2(pts[tr[0]], pts[tr[1]], pts[tr[2]])
		require.Positive(t, s.Sign(), `facet %v is counter-clockwise and non-degenerate`, tr)
		areaSum.Add(areaSum, s)
	}
	area, _ := areaSum.Float64()
	wantArea := loopSignedArea2(pts, outer) + loopSignedArea2(pts, hole) // hole area is negative
	require.InDelta(t, wantArea, area, 1e-9, `the fan covers exactly the band`)

	// Boundary preservation: every directed boundary edge of the input loops
	// appears exactly once among the triangle edges (no boundary vertex was
	// dropped), and its reverse never does.
	de := map[[2]int]int{}
	for _, tr := range tris {
		for k := range 3 {
			de[[2]int{tr[k], tr[(k+1)%3]}]++
		}
	}
	boundary := func(loop []int) {
		n := len(loop)
		for i := range loop {
			e := [2]int{loop[i], loop[(i+1)%n]}
			require.Equal(t, 1, de[e], `boundary edge %v kept once`, e)
			require.Equal(t, 0, de[[2]int{e[1], e[0]}], `boundary edge %v not traversed backward`, e)
		}
	}
	boundary(outer)
	boundary(hole)
}

// TestTriangulate2DHolesSharingOneBridgeAnchor is the shared-anchor case the
// property generator cannot draw: an outer rectangle whose right edge carries NO
// interior samples, and four holes in two v-bands. Every hole's +u visibility ray
// reaches that one right edge, so two bridges land on the same outer corner, and
// the splice must put each one in that corner's own angular order — otherwise the
// merged polygon crosses itself there and ear clipping stalls.
func TestTriangulate2DHolesSharingOneBridgeAnchor(t *testing.T) {
	var pts []Point2
	add := func(u, v float64) int { pts = append(pts, Point2{U: u, V: v}); return len(pts) - 1 }

	outer := []int{add(0, 0), add(60, 0), add(60, 40), add(0, 40)}
	// Each hole is walked clockwise, starting at its own maximum-u corner.
	sq := func(u0, v0, u1, v1 float64) []int {
		return []int{add(u1, v1), add(u1, v0), add(u0, v0), add(u0, v1)}
	}
	holes := [][]int{
		sq(11, 9, 19, 15),
		sq(41, 9, 49, 15),
		sq(11, 25, 19, 31),
		sq(41, 25, 49, 31),
	}

	require.Positive(t, loopSignedArea2(pts, outer))
	for i, h := range holes {
		require.Negative(t, loopSignedArea2(pts, h), `hole %d is clockwise`, i)
	}
	triAssert(t, pts, outer, holes)
}
