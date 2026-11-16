package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// capScene is an arc end on a cylinder G (radius 2 about the axis through (9, 0), the rim of radius 7.5 about the
// z axis in the plane z = 0, a fillet of radius 0.5 beside it) with G's held facets along its top circle, the rows'
// end vertices on G and the wall's cut up the generator through V, in the form capFacets reads.
type capScene struct {
	a                 arcRim
	w                 *work
	g                 faceLoops
	chainA, remA      []int
	endIDs, wallRun   []int
	v0                int
	psiV, step, bound float64
}

const (
	capR  = 7.5
	capD  = 9.0
	capRG = 2.0
)

// onG is the point of G at the angle psi from c→O, at height z.
func onG(psi, z float64) r3.Vec {
	return r3.NewVec(capD-capRG*math.Cos(psi), capRG*math.Sin(psi), z)
}

// psiOfRow is the angle at G's axis of the meeting of the circle of radius rho with G's circle.
func psiOfRow(rho float64) float64 {
	return math.Acos((capD*capD + capRG*capRG - rho*rho) / (2 * capD * capRG))
}

// newCapScene builds the scene with step the angle between G's top vertices and m rows.
func newCapScene(t *testing.T, step float64, m int) *capScene {
	t.Helper()
	sc := &capScene{step: step, bound: 1e-4}
	sc.psiV = psiOfRow(capR)
	rm := rim{center: r3.Vec{}, n: r3.NewVec(0, 0, 1), u: r3.NewVec(1, 0, 0), w: r3.NewVec(0, 1, 0),
		radius: capR, kappa: 1, side: 1, scale: 16}
	a := arcRim{rim: rm, sign: 1}
	c := r3.NewVec(capD, 0, 0)
	// A's chain along G runs with ψ growing, the angle about c shrinking: dir −1.
	ce := &cylEnd{c: c, axis: r3.NewVec(0, 0, 1), rG: capRG, d: capD, sigma: 1, dir: -1}
	a.cyl[0] = ce

	var verts []r3.Vec
	var tris [][3]int
	add := func(p r3.Vec) int {
		verts = append(verts, p)
		return len(verts) - 1
	}
	// G's strip: top circle vertices T_k at height 0 and bottom ones B_k at −1.
	top, bottom := []int{add(onG(sc.psiV, 0))}, []int{add(onG(sc.psiV, -1))}
	for k := 1; sc.psiV+float64(k-1)*step < psiOfRow(8)+3*step; k++ {
		top = append(top, add(onG(sc.psiV+float64(k)*step, 0)))
		bottom = append(bottom, add(onG(sc.psiV+float64(k)*step, -1)))
	}
	for k := range len(top) - 1 {
		tris = append(tris, [3]int{top[k], bottom[k], top[k+1]}, [3]int{top[k+1], bottom[k], bottom[k+1]})
	}
	sc.v0 = top[0]
	a.ends[0] = lineEnd{exact: verts[sc.v0], v: sc.v0}
	pa := psiOfRow(8)
	sc.chainA = top[1:]
	for k, v := range sc.chainA {
		if sc.psiV+float64(k+1)*step < pa-1e-9 {
			sc.remA = append(sc.remA, v)
		}
	}
	// The rows' ends: the torus rows' meeting with G, from P_A to the band's corner.
	for j := 0; j <= m; j++ {
		tt := math.Pi / 2 * float64(j) / float64(m)
		rho, z := capR+0.5-0.5*math.Sin(tt), 0.5*(1-math.Cos(tt))
		sc.endIDs = append(sc.endIDs, add(onG(psiOfRow(rho), z)))
	}
	sc.wallRun = []int{add(onG(sc.psiV, 0.25))}
	fp := facetedrecord.Payload{Verts: verts, VertexBound: make([]float64, len(verts)), Tris: tris}
	for i := range fp.VertexBound {
		fp.VertexBound[i] = sc.bound
	}
	sc.w = newWork(fp, rm, blendSpec{fillet: true, r: 0.5})
	sc.a = a
	for f := range tris {
		sc.g.facets = append(sc.g.facets, f)
	}
	return sc
}

func (sc *capScene) capFacets() ([2]int, error) {
	return sc.a.capFacets(sc.w, 0, sc.g, sc.chainA, sc.remA, sc.endIDs, sc.wallRun)
}

// TestCapFacetsChargeCoversSagitta builds a cap whose triangles are wide enough that a plane triangle's departure
// from G exceeds the corners' own bound, and checks sample points of every cap triangle against the exact cylinder
// in float64: each lies within the largest corner bound of its triangle.
func TestCapFacetsChargeCoversSagitta(t *testing.T) {
	t.Parallel()
	sc := newCapScene(t, 0.2, 12)
	r, err := sc.capFacets()
	require.NoError(t, err)
	require.Greater(t, r[1]-r[0], 10)
	c := sc.a.cyl[0].c
	raised := 0
	for _, tri := range sc.w.out[r[0]:r[1]] {
		bound := max(sc.w.beta[tri[0]], sc.w.beta[tri[1]], sc.w.beta[tri[2]])
		if bound > 10*sc.bound {
			raised++
		}
		worst := 0.0
		const n = 12
		for i := 0; i <= n; i++ {
			for j := 0; i+j <= n; j++ {
				l0, l1 := float64(i)/n, float64(j)/n
				p := sc.w.verts[tri[0]].Scale(l0).Add(sc.w.verts[tri[1]].Scale(l1)).Add(sc.w.verts[tri[2]].Scale(1 - l0 - l1))
				q := p.Sub(c)
				worst = max(worst, math.Abs(math.Hypot(q.X, q.Y)-capRG))
			}
		}
		require.LessOrEqual(t, worst, bound, "triangle %v", tri)
	}
	// The wide triangles' sagitta is what the charge carries: without it their bound would stay at the corners'.
	require.Positive(t, raised)
}

// TestCapFacetsRefusesFoldingChain moves one row end below A's plane, so no triangle that joins it to the bottom
// chain is wound the way the cap's loop runs: the zip has no positive move and refuses.
func TestCapFacetsRefusesFoldingChain(t *testing.T) {
	t.Parallel()
	sc := newCapScene(t, 0.05, 12)
	sc.w.verts[sc.endIDs[5]] = onG(psiOfRow(sc.rowRho(5)), -0.3)
	_, err := sc.capFacets()
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "folds")
}

func (sc *capScene) rowRho(j int) float64 {
	tt := math.Pi / 2 * float64(j) / float64(len(sc.endIDs)-1)
	return capR + 0.5 - 0.5*math.Sin(tt)
}

// TestSplitChainEdge checks the split of a held facet that keeps a chain edge the cap's columns fall on: feet in
// order split it into a fan with a bound that covers the points' distance from the edge, and feet out of order,
// equal or beyond the edge's end refuse.
func TestSplitChainEdge(t *testing.T) {
	t.Parallel()
	type pts = []r3.Vec
	cases := []struct {
		name string
		at   pts
		bad  bool
	}{
		{"in order", pts{onG(0.7, 0), onG(0.74, 0)}, false},
		{"reversed listing", pts{onG(0.74, 0), onG(0.7, 0)}, false},
		{"equal feet", pts{onG(0.72, 0), onG(0.72, 0)}, true},
		{"beyond the end", pts{onG(0.7, 0), onG(0.9, 0)}, true},
		{"before the start", pts{onG(0.5, 0), onG(0.7, 0)}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sc := newCapScene(t, 0.1, 8)
			// The edge T₀T₁ of the first facet, at ψ from psiV = 0.646 to 0.746.
			sc.a.cyl[0].dir = -1
			var ids []int
			for _, p := range tc.at {
				ids = append(ids, sc.w.vertex(p, 0))
			}
			u, v := 0, 2
			e := &splitEdge{u: u, v: v, pts: ids}
			facet := -1
			for f, tr := range sc.w.fp.Tris {
				if (tr[0] == u || tr[1] == u || tr[2] == u) && (tr[0] == v || tr[1] == v || tr[2] == v) {
					facet = f
					break
				}
			}
			require.GreaterOrEqual(t, facet, 0)
			outFrom := len(sc.w.out)
			err := sc.a.splitChainEdge(sc.w, sc.g, facet, e, 1e-12)
			if tc.bad {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				return
			}
			require.NoError(t, err)
			require.Len(t, sc.w.out[outFrom:], len(ids)+1)
			require.Equal(t, []int{facet}, sc.w.removed)
			for _, id := range ids {
				d := distToSegment(sc.w.verts[id], sc.w.verts[u], sc.w.verts[v])
				require.GreaterOrEqual(t, sc.w.beta[id], sc.bound+d)
			}
			require.Positive(t, sc.w.splitVol)
		})
	}
}

// TestCapFacetsNoiseVertexBesideV puts a rim vertex n beside V, 8·10⁻⁴ along G's circle from it and 6·10⁻⁴ off G,
// with every bound at 10⁻⁹, so that only the raise of n's bound (input bounds plus |n − V|) can cover it:
// cylBandEnd reads it as on the radial plane, and the cap closes with the closing triangle (n, V, b₁). The facets
// of the cap with the closing triangles must leave one closed boundary loop and use every other edge twice, once
// each way; sample points of the closing triangle must lie within its facet bound of the exact arc of A ∩ G, G's
// circle at z = 0, from V to b₁; and n's bound must cover its distance from the true V.
func TestCapFacetsNoiseVertexBesideV(t *testing.T) {
	t.Parallel()
	sc := newCapScene(t, 0.2, 12)
	const tiny = 1e-9
	for i := range sc.w.fp.VertexBound {
		sc.w.fp.VertexBound[i], sc.w.beta[i] = tiny, tiny
	}
	v := sc.w.verts[sc.v0]
	c := sc.a.cyl[0].c
	p := onG(sc.psiV+0.0004, 0)
	radial, ok := p.Sub(c).Normalize()
	require.True(t, ok)
	n := sc.w.vertex(p.Add(radial.Scale(6e-4)), tiny)
	sc.a.cyl[0].noise = []int{n}
	// The wall's kept facet beside V runs V→n.
	sc.w.emit([3]int{sc.wallRun[0], sc.v0, n}, 0)
	r, err := sc.capFacets()
	require.NoError(t, err)

	// n lies within its raised bound of the true V.
	dist := sc.w.verts[n].Sub(v).Len()
	require.LessOrEqual(t, dist, sc.w.beta[n])
	require.GreaterOrEqual(t, sc.w.beta[n], dist)

	// The closing triangle is the first of the cap's range; its points lie within its facet bound of the arc of
	// A ∩ G from V to b₁.
	closing := sc.w.out[r[0]]
	require.Contains(t, closing[:], n)
	require.Contains(t, closing[:], sc.v0)
	bound := max(sc.w.beta[closing[0]], sc.w.beta[closing[1]], sc.w.beta[closing[2]])
	b1 := -1
	for _, id := range closing {
		if id != n && id != sc.v0 {
			b1 = id
		}
	}
	angle := func(q r3.Vec) float64 { return math.Atan2(q.Y-c.Y, q.X-c.X) }
	a0, a1 := angle(v), angle(sc.w.verts[b1])
	lo, hi := min(a0, a1), max(a0, a1)
	require.Less(t, hi-lo, 1.0)
	onArc := func(q r3.Vec) float64 {
		at := math.Max(lo, math.Min(hi, angle(q)))
		nearest := r3.NewVec(c.X+capRG*math.Cos(at), c.Y+capRG*math.Sin(at), 0)
		return q.Sub(nearest).Len()
	}
	worst := 0.0
	const samples = 12
	for i := 0; i <= samples; i++ {
		for j := 0; i+j <= samples; j++ {
			l0, l1 := float64(i)/samples, float64(j)/samples
			q := sc.w.verts[closing[0]].Scale(l0).Add(sc.w.verts[closing[1]].Scale(l1)).
				Add(sc.w.verts[closing[2]].Scale(1 - l0 - l1))
			worst = max(worst, onArc(q))
		}
	}
	require.Greater(t, worst, 1e-4, "the closing triangle departs from the arc by more than the tiny bounds")
	require.LessOrEqual(t, worst, bound)

	// The cap's edges: one closed boundary loop, every other edge used twice, once each way.
	type edge struct{ a, b int }
	count := map[edge]int{}
	for _, tri := range sc.w.out[r[0]:r[1]] {
		for k := range 3 {
			count[edge{tri[k], tri[(k+1)%3]}]++
		}
	}
	next := map[int]int{}
	for e, k := range count {
		require.Equal(t, 1, k, "edge %v is used twice the same way", e)
		if count[edge{e.b, e.a}] == 0 {
			_, dup := next[e.a]
			require.False(t, dup, "boundary vertex %d leaves twice", e.a)
			next[e.a] = e.b
		}
	}
	require.NotEmpty(t, next)
	start := -1
	for a := range next {
		start = a
		break
	}
	steps := 0
	for at := start; ; {
		at = next[at]
		steps++
		require.LessOrEqual(t, steps, len(next))
		if at == start {
			break
		}
	}
	require.Equal(t, len(next), steps, "the boundary is one loop")
	// The held facet that keeps V→n lies outside the range, so the closing triangle's edge n→V is on the loop,
	// with V→b₁ after it, which the held facets of G beside the bottom chain pair.
	require.Equal(t, sc.v0, next[n])
	require.Equal(t, b1, next[sc.v0])
}
