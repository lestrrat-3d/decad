package decad

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file checks agent-docs/retained-edge-blend-design.md §6.3's strip
// triangles facet by facet. A held mesh states a bound per vertex, and every
// facet claims its true piece within the largest bound of its three corners
// (agent-docs/faceted-vertex-bounds-design.md §2). Mesh.Bound is the largest
// over the whole mesh, so a source's own charges can cover a strip facet that
// its own corners understate; this test reads each blend facet's own bound
// from the mesh node's VertexBound, which no exported API publishes, so it
// lives in the root package rather than apitest/.
// The exact surfaces come from each fixture's construction in closed form.

// stripProfile is a blend's profile in the meridian half-plane about the axis
// through o along +z: the arc of radius r about (rc, zc) whose directions run
// from a0 counter-clockwise through span. sphere set measures from the sphere
// of radius r about o instead, a nonzero line from the cylinder of radius r
// about the line through o along it, and seg from the cone the meridian
// segment p0 p1 sweeps about the axis.
type stripProfile struct {
	o         r3.Vec
	rc, zc, r float64
	a0, span  float64
	sphere    bool
	line      r3.Vec
	seg       bool
	p0, p1    [2]float64
}

func (p stripProfile) dist(x r3.Vec) float64 {
	q := x.Sub(p.o)
	if p.sphere {
		return math.Abs(q.Len() - p.r)
	}
	if p.line != (r3.Vec{}) {
		return math.Abs(q.Sub(p.line.Scale(q.Dot(p.line))).Len() - p.r)
	}
	if p.seg {
		rho := math.Hypot(q.X, q.Y)
		ux, uz := p.p1[0]-p.p0[0], p.p1[1]-p.p0[1]
		s := math.Min(1, math.Max(0, ((rho-p.p0[0])*ux+(q.Z-p.p0[1])*uz)/(ux*ux+uz*uz)))
		return math.Hypot(rho-p.p0[0]-s*ux, q.Z-p.p0[1]-s*uz)
	}
	dx, dz := math.Hypot(q.X, q.Y)-p.rc, q.Z-p.zc
	th := math.Atan2(dz, dx) - p.a0
	th -= 2 * math.Pi * math.Floor(th/(2*math.Pi))
	if th <= p.span {
		return math.Abs(math.Hypot(dx, dz) - p.r)
	}
	end := func(a float64) float64 { return math.Hypot(dx-p.r*math.Cos(a), dz-p.r*math.Sin(a)) }
	return math.Min(end(p.a0), end(p.a0+p.span))
}

// requireFacetBoundsHold samples every facet of out's mesh whose source face
// keep accepts on a 12×12 barycentric grid and requires every sample within
// the facet's own bound, the largest of its corners' vertex bounds, of the
// surface p.
func requireFacetBoundsHold(t *testing.T, out *Body, keep func(*Face) bool, p stripProfile) {
	t.Helper()
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyNone))
	require.NoError(t, err)
	vb := mesh.n.VertexBound
	require.Len(t, vb, len(mesh.n.Vertices), "the blend's mesh states its vertex bounds")
	src := mesh.SourceFaces()
	worst, facets := 0.0, 0
	for i, tri := range mesh.n.Triangles {
		if !keep(src[i]) {
			continue
		}
		facets++
		a, b, c := mesh.n.Vertices[tri[0]], mesh.n.Vertices[tri[1]], mesh.n.Vertices[tri[2]]
		own := max(vb[tri[0]], vb[tri[1]], vb[tri[2]])
		far := 0.0
		const g = 12
		for x := 0; x <= g; x++ {
			for y := 0; x+y <= g; y++ {
				far = math.Max(far, p.dist(a.Add(b.Sub(a).Scale(float64(x)/g)).Add(c.Sub(a).Scale(float64(y)/g))))
			}
		}
		worst = math.Max(worst, far/own)
		require.LessOrEqual(t, far, own, "facet %d at %v", i, a)
	}
	t.Logf("%d blend facets, the farthest at %.3f of its own bound", facets, worst)
	require.Positive(t, facets)
}

// freshFaces returns a predicate that accepts the faces of a body none of
// whose origins appear on a face of before.
func freshFaces(before *Body) func(after *Body) func(*Face) bool {
	old := map[FeatureRef]bool{}
	for _, f := range before.Faces() {
		for _, o := range f.Origins() {
			old[o] = true
		}
	}
	return func(after *Body) func(*Face) bool {
		made := map[*Face]bool{}
		for _, f := range after.Faces() {
			fresh := true
			for _, o := range f.Origins() {
				fresh = fresh && !old[o]
			}
			made[f] = fresh
		}
		return func(f *Face) bool { return made[f] }
	}
}

type stripScene struct {
	w   *sketch.World
	doc *Document
}

func newStripScene() *stripScene { return &stripScene{w: sketch.NewWorld(), doc: New()} }

func (sc *stripScene) rect(t *testing.T, x0, y0, x1, y1, h float64) *Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := Extrude(t.Context(), sc.doc, s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return b
}

// disc extrudes the circle of radius r about (cx, 10) by h along dir.
func (sc *stripScene) disc(t *testing.T, cx, r, h float64, dir Direction) *Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(cx, 10)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := Extrude(t.Context(), sc.doc, s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: dir})
	require.NoError(t, err)
	return b
}

func moved(t *testing.T, b *Body, d r3.Vec) *Body {
	t.Helper()
	tr, err := r3.Translation(d)
	require.NoError(t, err)
	out, err := Transform(t.Context(), b, tr)
	require.NoError(t, err)
	return out
}

// lineEdgeAt selects the one straight edge along dir with an end within
// 5e-3 of p.
func lineEdgeAt(t *testing.T, b *Body, p, dir r3.Vec) EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(Line3); !ok {
			continue
		}
		for _, v := range []*Vertex{e.Start(), e.End()} {
			run := e.End().Position().Value.Sub(e.Start().Position().Value)
			if v.Position().Value.Sub(p).Len() < 5e-3 && run.Cross(dir).Len() < 1e-6*run.Len() {
				return Edges(EndpointAt(v.Position().Value), ParallelTo(dir)).Exactly(1)
			}
		}
	}
	require.Fail(t, "no such edge", "near %v along %v", p, dir)
	return nil
}

// circularAt selects the one circular edge (an arc when arc is set, else a
// full circle) whose centre lies at height z.
func circularAt(t *testing.T, b *Body, z float64, arc bool) EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		var centre r3.Vec
		switch c := e.Curve().(type) {
		case Arc3:
			if !arc {
				continue
			}
			centre = c.Center
		case Circle3:
			if arc {
				continue
			}
			centre = c.Center
		default:
			continue
		}
		if math.Abs(centre.Z-z) < 1e-9 {
			return Edges(Circular(), EndpointAt(e.Start().Position().Value)).Exactly(1)
		}
	}
	require.Fail(t, "no circular edge at that height")
	return nil
}

func TestRetainedBlendStripFacetBounds(t *testing.T) {
	t.Parallel()

	// A boss of radius 3 on a plate, its top rim at z = 30.001 filleted (R2,
	// κ = −1): the tube's sagitta and the azimuth's add toward the axis.
	for _, r := range []float64{1, 2.5} {
		t.Run(fmt.Sprintf("a boss's top rim fillet, r = %g", r), func(t *testing.T) {
			t.Parallel()
			const top = 30.001
			sc := newStripScene()
			joined, err := Union(t.Context(), sc.rect(t, 0, 0, 40, 20, 20),
				moved(t, sc.disc(t, 20, 3, 10.5, Against), r3.Vec{Z: top}))
			require.NoError(t, err)
			fresh := freshFaces(joined)
			out, err := Fillet(t.Context(), joined, circularAt(t, joined, top, false), units.Millimeters(r))
			require.NoError(t, err)
			requireFacetBoundsHold(t, out, fresh(out),
				stripProfile{o: r3.NewVec(20, 10, 0), rc: 3 - r, zc: top - r, r: r, span: math.Pi / 2})
		})
	}

	// A boss of radius 9 with its top rim chamfered at d = 0.003, just inside
	// the 4,096 columns a strip takes on the rim (a chamfer's columns take all
	// of tau, so it reaches the limit below d/R of about 3.0·10⁻⁴ and keeps one
	// row gap, where a fillet just inside it builds some 200,000 facets): the
	// facets still lie within their own bounds.
	t.Run("a boss's top rim chamfer just inside the column limit", func(t *testing.T) {
		t.Parallel()
		const top, R, d = 30.001, 9.0, 0.003
		sc := newStripScene()
		joined, err := Union(t.Context(), sc.rect(t, 0, 0, 40, 20, 20),
			moved(t, sc.disc(t, 20, R, 10.5, Against), r3.Vec{Z: top}))
		require.NoError(t, err)
		fresh := freshFaces(joined)
		out, err := Chamfer(t.Context(), joined, circularAt(t, joined, top, false), units.Millimeters(d))
		require.NoError(t, err)
		requireFacetBoundsHold(t, out, fresh(out),
			stripProfile{o: r3.NewVec(20, 10, 0), seg: true, p0: [2]float64{R - d, top}, p1: [2]float64{R, top - d}})
	})

	// The rounded box corner cut back so its plane's row meets the cut at the
	// sine 0.4, filleted along the tangent chain (the chain route's fan).
	t.Run("a boss corner's chain fillet", func(t *testing.T) {
		t.Parallel()
		const r, q, R = 2.5, 0.4, 3.0
		gx := R - (R-r)*math.Sqrt(1-q*q)
		sc := newStripScene()
		box, err := Fillet(t.Context(), sc.rect(t, 0, 0, 40, 20, 20),
			Edges(ParallelTo(r3.NewVec(0, 0, 1)), EndpointAt(r3.Vec{})).Exactly(1), units.Millimeters(R))
		require.NoError(t, err)
		trimmed, err := Cut(t.Context(), box, moved(t, sc.rect(t, -5, -5, gx, 25, 30), r3.Vec{Z: -5}))
		require.NoError(t, err)
		fresh := freshFaces(trimmed)
		out, err := Fillet(t.Context(), trimmed, circularAt(t, trimmed, 20, true), units.Millimeters(r),
			WithTangentChain())
		require.NoError(t, err)
		made := fresh(out)
		corner := func(f *Face) bool {
			p := f.Loops()[0].CoEdges()[0].Start().Position().Value
			return made(f) && p.X < R+1e-6 && p.Y < R+1e-6
		}
		requireFacetBoundsHold(t, out, corner, stripProfile{o: r3.NewVec(R, R, 0), rc: R - r, zc: 20 - r, r: r,
			span: math.Pi / 2})
	})

	// A Ø8 bore whose axis lies h from x = 0 breaks out through it, its top arc
	// filleted with d = 4 (the arc route): the wall's row meets x = 0 at the
	// sine 0.2.
	t.Run("a hole mouth's arc fillet", func(t *testing.T) {
		t.Parallel()
		const bore, d, c = 4.0, 4.0, 0.2
		h := bore * math.Sqrt(1-c*c)
		sc := newStripScene()
		bored, err := Cut(t.Context(), sc.rect(t, 0, 0, 40, 20, 20),
			moved(t, sc.disc(t, h, bore, 60, Along), r3.Vec{Z: -30 + 0.001}))
		require.NoError(t, err)
		fresh := freshFaces(bored)
		out, err := Fillet(t.Context(), bored, circularAt(t, bored, 20, true), units.Millimeters(d))
		require.NoError(t, err)
		requireFacetBoundsHold(t, out, fresh(out), stripProfile{o: r3.NewVec(h, 10, 0), rc: bore + d, zc: 20 - d, r: d,
			a0: math.Pi / 2, span: math.Pi / 2})
	})

	// The same hole mouth at the sine 0.4, chamfered with d = 2: a cone's rows
	// take their columns from each gap's whole allowance, less the shift of
	// their ends on x = 0.
	t.Run("a hole mouth's arc chamfer", func(t *testing.T) {
		t.Parallel()
		const bore, d, c = 4.0, 2.0, 0.4
		h := bore * math.Sqrt(1-c*c)
		sc := newStripScene()
		bored, err := Cut(t.Context(), sc.rect(t, 0, 0, 40, 20, 20),
			moved(t, sc.disc(t, h, bore, 60, Along), r3.Vec{Z: -30 + 0.001}))
		require.NoError(t, err)
		fresh := freshFaces(bored)
		out, err := Chamfer(t.Context(), bored, circularAt(t, bored, 20, true), units.Millimeters(d))
		require.NoError(t, err)
		requireFacetBoundsHold(t, out, fresh(out), stripProfile{o: r3.NewVec(h, 10, 0), seg: true,
			p0: [2]float64{bore + d, 20}, p1: [2]float64{bore, 20 - d}})
	})

	// The boss corner at the sine 0.2, chamfered with d = 2.5 along the
	// tangent chain: the rows near the axis sweep the widest angles, and the
	// fan takes as many columns as its most demanding row needs.
	t.Run("a boss corner's chain chamfer", func(t *testing.T) {
		t.Parallel()
		const d, q, R = 2.5, 0.2, 3.0
		gx := R - (R-d)*math.Sqrt(1-q*q)
		sc := newStripScene()
		box, err := Fillet(t.Context(), sc.rect(t, 0, 0, 40, 20, 20),
			Edges(ParallelTo(r3.NewVec(0, 0, 1)), EndpointAt(r3.Vec{})).Exactly(1), units.Millimeters(R))
		require.NoError(t, err)
		trimmed, err := Cut(t.Context(), box, moved(t, sc.rect(t, -5, -5, gx, 25, 30), r3.Vec{Z: -5}))
		require.NoError(t, err)
		fresh := freshFaces(trimmed)
		out, err := Chamfer(t.Context(), trimmed, circularAt(t, trimmed, 20, true), units.Millimeters(d),
			WithTangentChain())
		require.NoError(t, err)
		made := fresh(out)
		corner := func(f *Face) bool {
			p := f.Loops()[0].CoEdges()[0].Start().Position().Value
			return made(f) && p.X < R+1e-6 && p.Y < R+1e-6
		}
		requireFacetBoundsHold(t, out, corner, stripProfile{o: r3.NewVec(R, R, 0), seg: true,
			p0: [2]float64{R - d, 20}, p1: [2]float64{R, 20 - d}})
	})

	// The cap of a ball of radius 2 below z = −1, its rim filleted with
	// r = 0.4 (R6): the band's cut vertices lie on the sphere's held facets.
	t.Run("a sphere rim's fillet", func(t *testing.T) {
		t.Parallel()
		const R, cutZ, r = 2.0, -1.0, 0.4
		sc := newStripScene()
		s, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		o := s.CreatePoint(-R, 0)
		s.Fix(o)
		end := s.CreatePoint(R, 0)
		s.CreateLine(o, end)
		s.CreateArc(s.CreatePoint(0, 0), end, o)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		ball, err := Revolve(t.Context(), sc.doc, s, s.Profiles()[0],
			SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}, FullRevolution{})
		require.NoError(t, err)
		capped, err := Cut(t.Context(), ball, moved(t, sc.rect(t, -100, -100, 100, 100, 20), r3.Vec{Z: cutZ}))
		require.NoError(t, err)
		fresh := freshFaces(capped)
		out, err := Fillet(t.Context(), capped, circularAt(t, capped, cutZ, false), units.Millimeters(r))
		require.NoError(t, err)
		zc := cutZ - r
		rc := math.Sqrt((R-r)*(R-r) - zc*zc)
		a0 := math.Atan2(zc, rc)
		requireFacetBoundsHold(t, out, fresh(out), stripProfile{rc: rc, zc: zc, r: r, a0: a0, span: math.Pi/2 - a0})
	})

	// The straight routes cut curved walls too, and their cut vertices take
	// the held facet's bound: a groove of radius 4 along x about (y, z) =
	// (10, 22) whose near top edge is filleted with r = 1 (R3, the ball at
	// (6, 19)), and the ridge between two bores of radius 4 about (14, 10) and
	// (21, 10), filleted with r = 1 (R5, the ball outside both).
	t.Run("a groove's top edge fillet", func(t *testing.T) {
		t.Parallel()
		sc := newStripScene()
		s, err := sc.w.CreateSketch(sc.w.YZ())
		require.NoError(t, err)
		c := s.CreatePoint(10, 22)
		s.Fix(c)
		s.CreateCircle(c, 4)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		tool, err := Extrude(t.Context(), sc.doc, s, s.Profiles()[0], Symmetric{D: units.Millimeters(30)})
		require.NoError(t, err)
		grooved, err := Cut(t.Context(), sc.rect(t, 0, 0, 40, 20, 20), moved(t, tool, r3.Vec{X: 20}))
		require.NoError(t, err)
		fresh := freshFaces(grooved)
		k := 10 - math.Sqrt(16-4)
		out, err := Fillet(t.Context(), grooved, lineEdgeAt(t, grooved, r3.NewVec(0, k, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(1))
		require.NoError(t, err)
		requireFacetBoundsHold(t, out, fresh(out), stripProfile{o: r3.NewVec(0, 6, 19), r: 1, line: r3.NewVec(1, 0, 0)})
	})
	t.Run("a bore pair's ridge fillet", func(t *testing.T) {
		t.Parallel()
		sc := newStripScene()
		b := sc.rect(t, 0, 0, 40, 20, 20)
		for _, x := range []float64{14, 21} {
			var err error
			b, err = Cut(t.Context(), b, moved(t, sc.disc(t, x, 4, 40, Along), r3.Vec{Z: -10 + 0.001}))
			require.NoError(t, err)
		}
		fresh := freshFaces(b)
		ridge := r3.NewVec(17.5, 10-math.Sqrt(16-3.5*3.5), 20)
		out, err := Fillet(t.Context(), b, lineEdgeAt(t, b, ridge, r3.NewVec(0, 0, 1)), units.Millimeters(1))
		require.NoError(t, err)
		requireFacetBoundsHold(t, out, fresh(out),
			stripProfile{o: r3.NewVec(17.5, 10-math.Sqrt(25-3.5*3.5), 0), r: 1, line: r3.NewVec(0, 0, 1)})
	})

	// Three edges at a mesh box's corner (§4.8): the sphere octant between the
	// strips' end rows.
	t.Run("a box corner's sphere patch", func(t *testing.T) {
		t.Parallel()
		const r = 2.0
		lo, hi := r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10)
		v := []r3.Vec{
			{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: hi.Y, Z: lo.Z},
			{X: lo.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z}, {X: lo.X, Y: hi.Y, Z: hi.Z},
		}
		var tris [][3]int
		for _, q := range [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}} {
			tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
		}
		m, err := NewMeshBody(t.Context(), v, tris)
		require.NoError(t, err)
		box, err := SolidFromMesh(t.Context(), New(), m)
		require.NoError(t, err)
		corner := r3.NewVec(0, 0, 10)
		three := Edges(EndpointAt(corner), ParallelTo(r3.NewVec(1, 0, 0))).
			Or(EndpointAt(corner), ParallelTo(r3.NewVec(0, 1, 0))).Or(EndpointAt(corner), ParallelTo(r3.NewVec(0, 0, 1)))
		out, err := Fillet(t.Context(), box, three, units.Millimeters(r))
		require.NoError(t, err)
		centre := r3.NewVec(r, r, 10-r)
		octant := func(f *Face) bool {
			_, ok := f.Surface().(Sphere)
			return ok
		}
		requireFacetBoundsHold(t, out, octant, stripProfile{o: centre, r: r, sphere: true})
	})
}

// triangleDist is the distance from p to the triangle abc: the closest point
// by the triangle's Voronoi regions (Ericson, Real-Time Collision Detection
// §5.1.5).
func triangleDist(p, a, b, c r3.Vec) float64 {
	ab, ac, ap := b.Sub(a), c.Sub(a), p.Sub(a)
	d1, d2 := ab.Dot(ap), ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		return ap.Len()
	}
	bp := p.Sub(b)
	d3, d4 := ab.Dot(bp), ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		return bp.Len()
	}
	if vc := d1*d4 - d3*d2; vc <= 0 && d1 >= 0 && d3 <= 0 {
		return p.Sub(a.Add(ab.Scale(d1 / (d1 - d3)))).Len()
	}
	cp := p.Sub(c)
	d5, d6 := ab.Dot(cp), ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		return cp.Len()
	}
	if vb := d5*d2 - d1*d6; vb <= 0 && d2 >= 0 && d6 <= 0 {
		return p.Sub(a.Add(ac.Scale(d2 / (d2 - d6)))).Len()
	}
	if va := d3*d6 - d5*d4; va <= 0 && d4-d3 >= 0 && d5-d6 >= 0 {
		return p.Sub(b.Add(c.Sub(b).Scale((d4 - d3) / ((d4 - d3) + (d5 - d6))))).Len()
	}
	n := ab.Cross(ac)
	return math.Abs(ap.Dot(n)) / n.Len()
}

// requirePatchCovered requires every point of pts, exact points of the blend
// surface, within some facet's own bound of that facet, over the facets of
// out's mesh whose source face keep accepts: the patch side of §6.3's two-way
// claim. Facets are bucketed on a grid of their bounding boxes widened by
// their bounds, so each point meets only the facets that could hold it.
func requirePatchCovered(t *testing.T, out *Body, keep func(*Face) bool, pts []r3.Vec) {
	t.Helper()
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyNone))
	require.NoError(t, err)
	vb := mesh.n.VertexBound
	src := mesh.SourceFaces()
	const cell = 0.25
	key := func(x, y, z float64) [3]int {
		return [3]int{int(math.Floor(x / cell)), int(math.Floor(y / cell)), int(math.Floor(z / cell))}
	}
	type facet struct {
		a, b, c r3.Vec
		own     float64
	}
	var facets []facet
	grid := map[[3]int][]int{}
	for i, tri := range mesh.n.Triangles {
		if !keep(src[i]) {
			continue
		}
		f := facet{mesh.n.Vertices[tri[0]], mesh.n.Vertices[tri[1]], mesh.n.Vertices[tri[2]],
			max(vb[tri[0]], vb[tri[1]], vb[tri[2]])}
		lo := key(min(f.a.X, f.b.X, f.c.X)-f.own, min(f.a.Y, f.b.Y, f.c.Y)-f.own, min(f.a.Z, f.b.Z, f.c.Z)-f.own)
		hi := key(max(f.a.X, f.b.X, f.c.X)+f.own, max(f.a.Y, f.b.Y, f.c.Y)+f.own, max(f.a.Z, f.b.Z, f.c.Z)+f.own)
		for x := lo[0]; x <= hi[0]; x++ {
			for y := lo[1]; y <= hi[1]; y++ {
				for z := lo[2]; z <= hi[2]; z++ {
					grid[[3]int{x, y, z}] = append(grid[[3]int{x, y, z}], len(facets))
				}
			}
		}
		facets = append(facets, f)
	}
	require.NotEmpty(t, facets)
	worst := 0.0
	for _, p := range pts {
		best := math.Inf(1)
		for _, i := range grid[key(p.X, p.Y, p.Z)] {
			f := facets[i]
			best = math.Min(best, triangleDist(p, f.a, f.b, f.c)/f.own)
		}
		worst = math.Max(worst, best)
		require.LessOrEqual(t, best, 1.0, "the patch at %v lies past every facet's own bound", p)
	}
	t.Logf("%d patch points against %d blend facets, the farthest at %.3f of the nearest facet's own bound",
		len(pts), len(facets), worst)
}

// torusPatch samples the torus a blendProfile-like arc of radius r about
// (rc, zc), directions from a0 through span, sweeps about the vertical axis
// through o, at n profile angles strictly inside the arc and every 2π/k of
// azimuth from phi0 that inside accepts.
func torusPatch(o r3.Vec, rc, zc, r, a0, span float64, n, k int, phi0 float64, inside func(r3.Vec) bool) []r3.Vec {
	var out []r3.Vec
	for i := 1; i < n; i++ {
		th := a0 + span*float64(i)/float64(n)
		rho, z := rc+r*math.Cos(th), zc+r*math.Sin(th)
		for j := range k {
			phi := phi0 + 2*math.Pi*float64(j)/float64(k)
			p := r3.NewVec(o.X+rho*math.Cos(phi), o.Y+rho*math.Sin(phi), z)
			if inside(p) {
				out = append(out, p)
			}
		}
	}
	return out
}

func TestRetainedBlendStripPatchCovered(t *testing.T) {
	t.Parallel()

	// The hole mouth's arc fillet of TestRetainedBlendStripFacetBounds: d = 4
	// on the Ø8 bore whose wall row meets x = 0 at the sine 0.2. Its patch
	// lies inside the plate, x > 0.
	t.Run("a hole mouth's arc fillet", func(t *testing.T) {
		t.Parallel()
		const bore, d, c = 4.0, 4.0, 0.2
		h := bore * math.Sqrt(1-c*c)
		sc := newStripScene()
		bored, err := Cut(t.Context(), sc.rect(t, 0, 0, 40, 20, 20),
			moved(t, sc.disc(t, h, bore, 60, Along), r3.Vec{Z: -30 + 0.001}))
		require.NoError(t, err)
		fresh := freshFaces(bored)
		out, err := Fillet(t.Context(), bored, circularAt(t, bored, 20, true), units.Millimeters(d))
		require.NoError(t, err)
		pts := torusPatch(r3.NewVec(h, 10, 0), bore+d, 20-d, d, math.Pi/2, math.Pi/2, 48, 1200, 0,
			func(p r3.Vec) bool { return p.X > 0 })
		requirePatchCovered(t, out, fresh(out), pts)
	})

	// The boss corner's chain fillet: r = 2.5 with the plane's row meeting the
	// cut x = gx at the sine 0.4. Its patch is the corner's quarter turn,
	// x ≥ gx, short of the junction meridian at y = 0.
	t.Run("a boss corner's chain fillet", func(t *testing.T) {
		t.Parallel()
		const r, q, R = 2.5, 0.4, 3.0
		gx := R - (R-r)*math.Sqrt(1-q*q)
		sc := newStripScene()
		box, err := Fillet(t.Context(), sc.rect(t, 0, 0, 40, 20, 20),
			Edges(ParallelTo(r3.NewVec(0, 0, 1)), EndpointAt(r3.Vec{})).Exactly(1), units.Millimeters(R))
		require.NoError(t, err)
		trimmed, err := Cut(t.Context(), box, moved(t, sc.rect(t, -5, -5, gx, 25, 30), r3.Vec{Z: -5}))
		require.NoError(t, err)
		fresh := freshFaces(trimmed)
		out, err := Fillet(t.Context(), trimmed, circularAt(t, trimmed, 20, true), units.Millimeters(r),
			WithTangentChain())
		require.NoError(t, err)
		made := fresh(out)
		corner := func(f *Face) bool {
			p := f.Loops()[0].CoEdges()[0].Start().Position().Value
			return made(f) && p.X < R+1e-6 && p.Y < R+1e-6
		}
		pts := torusPatch(r3.NewVec(R, R, 0), R-r, 20-r, r, 0, math.Pi/2, 48, 4800, math.Pi,
			func(p r3.Vec) bool { return p.X > gx && p.Y < R && p.X < R })
		requirePatchCovered(t, out, corner, pts)
	})
}
