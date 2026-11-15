package retainedblend

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// cylinderBand is the arc route's band on the cylinder R = 3 about z with the lean
// from its tag: base z = 0, setback plane z = 1 (the main plane), end plane x = h with
// the band on side·(x − h) > 0, and the sine and slant the route reads.
func cylinderBand(t *testing.T, h, side, beta float64, verts []r3.Vec, tris [][3]int) (*work, *wallBand, faceLoops) {
	t.Helper()
	bs := make([]float64, len(verts))
	for i := range bs {
		bs[i] = beta
	}
	facets := make([]int, len(tris))
	for i := range facets {
		facets[i] = i
	}
	w := &work{
		fp:    facetedrecord.Payload{Tris: tris, FaceOf: make([]int, len(tris)), VertexBound: slices.Clone(bs)},
		verts: slices.Clone(verts), beta: bs, tau: 0.001, cuts: map[[2]int]int{},
	}
	z := r3.NewVec(0, 0, 1)
	b := &wallBand{
		main:     &bandEnd{g: plane{n: z, o: r3.NewVec(0, 0, 1)}, sign: -1},
		others:   []*bandEnd{{g: plane{n: z, o: r3.Vec{}}, sign: 1}},
		noteCuts: true,
	}
	g := plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(h, 0, 0)}
	cyl := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: z, Radius: units.Millimeters(3)}
	ww := wallWork(tagged(cyl))
	sine, planar := ww.wallSine(0, g)
	require.NotNil(t, sine)
	slant := ww.wallSlant(0, g)
	require.NotNil(t, slant)
	b.ends[0] = &bandEnd{g: g, sign: side, sine: sine, planarWall: planar, slanted: true, slant: slant}
	var ok bool
	b.lean, ok = ww.wallTilt(0, z)
	require.True(t, ok)
	b.leanOK = true
	return w, b, faceLoops{facets: facets}
}

// keptWallDistance is the distance from p to the closure of the kept wall of the
// cylinder R = 3: the wall above the setback plane z = 1, and the wall past G (x = h,
// the band on side·(x − h) > 0). It is a lower bound on the distance to the true result
// for every blend, which only adds surface inside the band.
func keptWallDistance(p r3.Vec, h, side float64) float64 {
	const radius = 3.0
	rho := math.Hypot(p.X, p.Y)
	foot := radius * math.Cos(math.Atan2(p.Y, p.X))
	if side*(foot-h) <= 0 || p.Z >= 1 {
		return math.Abs(rho - radius)
	}
	yG := math.Copysign(math.Sqrt(radius*radius-h*h), p.Y)
	return math.Min(math.Hypot(rho-radius, p.Z-1), math.Hypot(p.X-h, p.Y-yG))
}

// TestHeldVertexKeepsSlantBoundBesideTheSetbackPlane keeps one held facet whole: its
// corner A lies just outside G (within its bound, so it is read by sign) and a few
// hundredths below the setback plane, inside the wall by less than β, and its other two
// corners lie on the base row clear of G. The facet runs down the wall from A, and its
// points beside A lie inside the band, farther from every true surface than A is from
// the setback line. A held vertex therefore keeps β + c/s_lo, and every sampled point
// of the facet lies within the facet bound of the exact kept wall. Cap the held vertex
// by its distance to the setback line and the rows at sine 0.31 fail by up to 1.09.
func TestHeldVertexKeepsSlantBoundBesideTheSetbackPlane(t *testing.T) {
	t.Parallel()

	const radius = 3.0
	for _, beta := range []float64{0.002, 0.017} {
		tol := beta + 0.001
		for _, hr := range []float64{0.89, 0.95, 0.98} {
			if beta > 0.01 && hr > 0.97 {
				continue
			}
			for _, d := range []float64{tol + 0.0001, tol + 0.0006, tol + 0.002} {
				h := hr * radius
				s := math.Sqrt(1 - hr*hr)
				eta := beta - math.Max(0.00015, 0.06*beta)
				ax, ar := h-0.0001, radius-eta
				a := r3.NewVec(ax, math.Sqrt(ar*ar-ax*ax), 1-d)
				thOut := math.Acos(hr) + (tol+0.0005)/(radius*s)
				b0 := r3.NewVec(ar*math.Cos(thOut), ar*math.Sin(thOut), 0.55)
				c0 := r3.NewVec(ar*math.Cos(thOut+0.01), ar*math.Sin(thOut+0.01), 0.55)
				w, b, f := cylinderBand(t, h, 1, beta, []r3.Vec{a, b0, c0}, [][3]int{{0, 1, 2}})
				_, err := w.cutBand(f, b)
				require.NoError(t, err, "beta %v hr %v d %v", beta, hr, d)
				slant, ok := slantBound(proofbound.AbsSumUpper(beta, 0.0001), b.ends[0].sine)
				require.True(t, ok)
				require.InDelta(t, proofbound.AbsSumUpper(beta, slant), w.beta[0], 1e-9,
					"A keeps the slant bound, not its distance to the line")
				for _, tr := range w.out {
					p0, p1, p2 := w.verts[tr[0]], w.verts[tr[1]], w.verts[tr[2]]
					delta := max(w.beta[tr[0]], w.beta[tr[1]], w.beta[tr[2]])
					const n = 200
					for i := 0; i <= n; i++ {
						for j := 0; i+j <= n; j += 2 {
							l1, l2 := float64(i)/n, float64(j)/n
							p := p0.Scale(1 - l1 - l2).Add(p1.Scale(l1)).Add(p2.Scale(l2))
							require.LessOrEqual(t, keptWallDistance(p, h, 1), delta*(1+1e-9),
								"beta %v hr %v s %.3f d %v facet point %v", beta, hr, s, d, p)
						}
					}
				}
			}
		}
	}
}

// TestCappedCutPointOnTheSetbackPlane reads a cut point of the setback plane that lies
// 0.0015 off the end plane, whose edge runs from inside the band to outside it, with a
// third corner within its bound of the end plane so that the end plane's clip waits for the
// sign reading and the cut is read by it. Read
// inside, it is a vertex of a kept facet that lies above the setback plane, and its bound
// is its own (0.002, not the slant bound 0.0117). Read outside, the cut is dropped: the
// kept part of the facet runs to the end plane's own corner and no output vertex lies
// on the setback plane in the outside region.
func TestCappedCutPointOnTheSetbackPlane(t *testing.T) {
	t.Parallel()

	const s = 0.3
	cut := func(off float64, lean bool) (*work, int) {
		y := off / s
		w, b, f := setbackBand(s, 0, r3.NewVec(0.5, 0.2, 0), r3.NewVec(1.5, 2*y-0.2, 0), r3.NewVec(0.3, 0.001, 0))
		b.leanOK = lean
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		for id := 3; id < len(w.verts); id++ {
			if p := w.verts[id]; math.Abs(p.X-1) < 1e-12 && math.Abs(p.Y-y) < 1e-12 {
				return w, id
			}
		}
		return w, -1
	}

	t.Run("read inside the end plane", func(t *testing.T) {
		t.Parallel()
		w, id := cut(0.0015, true)
		require.GreaterOrEqual(t, id, 0)
		require.InDelta(t, 0.002, w.beta[id], 1e-9, "the cut carries its own bound")
		seen := false
		for _, tr := range w.out {
			if !slices.Contains(tr[:], id) {
				continue
			}
			seen = true
			for _, v := range tr {
				require.GreaterOrEqual(t, w.verts[v].X, 1-1e-12, "a facet that has the cut is above the setback plane")
			}
		}
		require.True(t, seen)
		w, id = cut(0.0015, false)
		require.InDelta(t, (0.002+0.0015)/s, w.beta[id], 1e-9, "no lean from the tags: the slant bound")
	})
	t.Run("read outside the end plane", func(t *testing.T) {
		t.Parallel()
		w, _ := cut(-0.0015, true)
		for _, tr := range w.out {
			for _, v := range tr {
				p := w.verts[v]
				require.False(t, math.Abs(p.X-1) < 1e-12 && p.Y < -1e-6, "no output vertex on the setback plane outside G: %v", p)
			}
		}
	})
}

// TestOnlyCutPointsOnTheSetbackPlaneAreCapped reads, in one band 0.01 thick between the
// base plane x = 0.99 and the setback plane x = 1, the cuts of a facet edge by both planes
// where each lies within its bound of the end plane. The cut of the setback plane takes
// the cap (0.002); the cut of the base plane keeps its slant bound, which is the same
// with and without a lean from the tags, although the setback bound would be lower.
func TestOnlyCutPointsOnTheSetbackPlaneAreCapped(t *testing.T) {
	t.Parallel()

	build := func(lean bool) *work {
		w, b, f := setbackBand(0.2, 0, r3.NewVec(0.5, 0.2, 0), r3.NewVec(1.5, -0.198, 0), r3.NewVec(0.995, 0.001, 0))
		b.others[0].g.o = r3.NewVec(0.99, 0, 0)
		b.leanOK = lean
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		return w
	}
	find := func(w *work, x float64) int {
		for id := 3; id < len(w.verts); id++ {
			if math.Abs(w.verts[id].X-x) < 1e-12 && w.verts[id].Y > 1e-6 {
				return id
			}
		}
		return -1
	}
	with, without := build(true), build(false)
	base, main := find(with, 0.99), find(with, 1)
	require.GreaterOrEqual(t, base, 0)
	require.GreaterOrEqual(t, main, 0)
	require.InDelta(t, 0.002, with.beta[main], 1e-9, "the cut of the setback plane is capped")
	require.Greater(t, without.beta[main], 0.01, "and has the slant bound without a lean")
	require.Greater(t, with.beta[base], 0.011+1e-9, "the cut of the base plane is not capped to β plus its distance to the setback plane")
	require.Equal(t, without.beta[base], with.beta[base])
}
