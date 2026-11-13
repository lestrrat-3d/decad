package retainedblend

import (
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// closeVerticesFace is a plane face, a square outer loop and one 64-gon hole,
// whose hole loop holds a pair of neighbouring vertices a hair apart.
type closeVerticesFace struct {
	verts       []r3.Vec
	outer, hole []int
	u, v        r3.Vec
}

// planeArea2 is the signed double area of loop in the face's winding frame.
func (f *closeVerticesFace) planeArea2(loop []int) float64 {
	ps := make([]pt2, len(loop))
	idx := make([]int, len(loop))
	for i, vi := range loop {
		ps[i] = pt2{f.verts[vi].Dot(f.u), f.verts[vi].Dot(f.v)}
		idx[i] = i
	}
	return signedArea2(ps, idx)
}

// newCloseVerticesFace builds the face of normal n. at maps plane coordinates
// to world; centre is the hole's centre in plane coordinates; pairA and pairB
// replace the hole's angle-0 vertex.
func newCloseVerticesFace(n r3.Vec, at func(a, b float64) r3.Vec, centre [2]float64, pairA, pairB r3.Vec) *closeVerticesFace {
	f := &closeVerticesFace{}
	f.u, _ = perpendicular(n)
	f.v = n.Cross(f.u)
	for _, p := range [][2]float64{{-10, -10}, {10, -10}, {10, 10}, {-10, 10}} {
		f.outer = append(f.outer, len(f.verts))
		f.verts = append(f.verts, at(centre[0]+p[0], centre[1]+p[1]))
	}
	const sides = 64
	for k := range sides {
		a := 2 * math.Pi * float64(k) / sides
		f.hole = append(f.hole, len(f.verts))
		if k == 0 {
			f.verts = append(f.verts, pairA)
			f.hole = append(f.hole, len(f.verts))
			f.verts = append(f.verts, pairB)
			continue
		}
		f.verts = append(f.verts, at(centre[0]+3*math.Cos(a), centre[1]+3*math.Sin(a)))
	}
	if f.planeArea2(f.outer) < 0 {
		slices.Reverse(f.outer)
	}
	if f.planeArea2(f.hole) > 0 {
		slices.Reverse(f.hole)
	}
	return f
}

// TestRetriangulateKeepsCloseBoundaryVertices retriangulates plane faces whose
// hole loop holds two vertices closer than the rounding of a frame centred far
// from the hole: a mesh Boolean leaves such a pair where a facet edge crosses
// a faceted cylinder beside its seam vertex. Both must stay on the boundary.
func TestRetriangulateKeepsCloseBoundaryVertices(t *testing.T) {
	tests := []struct {
		name         string
		n            r3.Vec
		at           func(a, b float64) r3.Vec
		centre       [2]float64
		pairA, pairB r3.Vec
	}{
		{
			name:   "normal -z, offset far below the rounding",
			n:      r3.NewVec(0, 0, -1),
			at:     func(a, b float64) r3.Vec { return r3.NewVec(a, b, 10) },
			centre: [2]float64{0, 0},
			pairA:  r3.NewVec(3, 0, 10),
			pairB:  r3.NewVec(3, -4e-16, 10),
		},
		{
			name:   "normal +z, offset of one ulp",
			n:      r3.NewVec(0, 0, 1),
			at:     func(a, b float64) r3.Vec { return r3.NewVec(a, b, -2) },
			centre: [2]float64{0, -20},
			pairA:  r3.NewVec(3, -20, -2),
			pairB:  r3.NewVec(3, math.Nextafter(-20, -30), -2),
		},
		{
			name:   "normal +x",
			n:      r3.NewVec(1, 0, 0),
			at:     func(a, b float64) r3.Vec { return r3.NewVec(5, b, a) },
			centre: [2]float64{0, 0},
			pairA:  r3.NewVec(5, 0, 3),
			pairB:  r3.NewVec(5, -3e-16, 3),
		},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			f := newCloseVerticesFace(tc.n, tc.at, tc.centre, tc.pairA, tc.pairB)
			w := &work{verts: f.verts}
			require.NoError(t, w.retriangulate(tc.n, [][]int{f.outer, f.hole}, 0))

			count := map[[2]int]int{}
			area := 0.0
			for _, tr := range w.out {
				for k := range 3 {
					count[[2]int{tr[k], tr[(k+1)%3]}]++
				}
				a, b, c := f.verts[tr[0]], f.verts[tr[1]], f.verts[tr[2]]
				area += b.Sub(a).Cross(c.Sub(a)).Dot(tc.n) / 2
			}
			onLoop := map[[2]int]struct{}{}
			for _, loop := range [][]int{f.outer, f.hole} {
				for k, vi := range loop {
					e := [2]int{vi, loop[(k+1)%len(loop)]}
					onLoop[e] = struct{}{}
					require.Equal(t, 1, count[e], "loop edge %v appears once", e)
					require.Zero(t, count[[2]int{e[1], e[0]}], "loop edge %v appears reversed", e)
				}
			}
			for e, c := range count {
				if _, ok := onLoop[e]; ok {
					continue
				}
				require.Equal(t, c, count[[2]int{e[1], e[0]}], "interior edge %v pairs with its reverse", e)
			}
			want := (math.Abs(f.planeArea2(f.outer)) - math.Abs(f.planeArea2(f.hole))) / 2
			require.InDelta(t, want, area, 1e-9, "the triangles cover the face")
		})
	}
}

// TestRetriangulateRefusesCoincidentProjection refuses two neighbouring loop
// vertices that differ only along the plane normal.
func TestRetriangulateRefusesCoincidentProjection(t *testing.T) {
	n := r3.NewVec(0, 0, -1)
	at := func(a, b float64) r3.Vec { return r3.NewVec(a, b, 10) }
	f := newCloseVerticesFace(n, at, [2]float64{0, 0}, r3.NewVec(3, 0, 10), r3.NewVec(3, 0, 10+1e-12))
	w := &work{verts: f.verts}
	err := w.retriangulate(n, [][]int{f.outer, f.hole}, 0)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "read one point in its plane")
}

// TestSortRingOrdersEqualAngles sorts a wall's cut ring whose two vertices,
// 2e-16 mm apart, read one float angle. Their order must follow their exact
// turn about the rim centre and never the order the ring was collected in.
func TestSortRingOrdersEqualAngles(t *testing.T) {
	for _, n := range []r3.Vec{r3.NewVec(0, 0, -1), r3.NewVec(0, 0, 1)} {
		u, ok := perpendicular(n)
		require.True(t, ok)
		rm := rim{center: r3.NewVec(0, 0, 4), n: n, u: u, w: n.Cross(u)}
		const radius = 3.0
		verts := []r3.Vec{r3.NewVec(radius, 0, 4), r3.NewVec(radius, -2e-16, 4)}
		for k := 1; k < 16; k++ {
			a := 2 * math.Pi * float64(k) / 16
			verts = append(verts, r3.NewVec(radius*math.Cos(a), radius*math.Sin(a), 4))
		}
		for _, dir := range []float64{1, -1} {
			w := &work{r: rm, verts: verts, dir: dir}
			require.Equal(t, w.angleOf(verts[0]), w.angleOf(verts[1]), "the pair reads one float angle")
			var want []int
			for trial := range 20 {
				ring := make([]ringPt, len(verts))
				for i, p := range verts {
					ring[i] = ringPt{idx: i, ang: w.angleOf(p)}
				}
				rng := rand.New(rand.NewPCG(uint64(trial), 7))
				rng.Shuffle(len(ring), func(i, j int) { ring[i], ring[j] = ring[j], ring[i] })
				w.sortRing(ring)
				got := make([]int, len(ring))
				for i, p := range ring {
					got[i] = p.idx
				}
				if want == nil {
					want = got
				}
				require.Equal(t, want, got, "normal %v dir %g trial %d", n, dir, trial)
			}
			i0, i1 := slices.Index(want, 0), slices.Index(want, 1)
			require.True(t, i0-i1 == 1 || i1-i0 == 1, "the close pair stays adjacent")
			// Vertex 1 lies just clockwise of vertex 0 about +Z, so it comes first when the ring turns
			// counter-clockwise about +Z.
			ccwAboutZ := w.r.u.Cross(w.r.w).Dot(r3.NewVec(0, 0, 1))*dir > 0
			require.Equal(t, ccwAboutZ, i1 < i0, "normal %v dir %g: exact turn order", n, dir)
		}
	}
}
