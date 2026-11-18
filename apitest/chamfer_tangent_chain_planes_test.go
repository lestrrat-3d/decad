package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// tcSeg is one outline segment in sketch coordinates: a line from a to b, or
// with arc set a counter-clockwise arc about c from a to b.
type tcSeg struct {
	arc     bool
	a, b, c [2]float64
}

func tcLine(ax, ay, bx, by float64) tcSeg { return tcSeg{a: [2]float64{ax, ay}, b: [2]float64{bx, by}} }

func tcArc(cx, cy, ax, ay, bx, by float64) tcSeg {
	return tcSeg{arc: true, a: [2]float64{ax, ay}, b: [2]float64{bx, by}, c: [2]float64{cx, cy}}
}

// tcScene is a document and sketch world shared by one test's bodies.
type tcScene struct {
	doc *decad.Document
	w   *sketch.World
}

func newTCScene() *tcScene {
	return &tcScene{doc: decad.New(decad.WithChainTolerance(units.Millimeters(0.01))), w: sketch.NewWorld()}
}

// prism sketches segs on the plane at origin spanned by u and v and extrudes
// the one profile by ext.
func (sc *tcScene) prism(t *testing.T, origin, u, v r3.Vec, segs []tcSeg, ext decad.Extent) *decad.Body {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	pl, err := sc.w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := sc.w.CreateSketch(pl)
	require.NoError(t, err)
	pts := map[[2]float64]*sketch.Point{}
	pt := func(p [2]float64) *sketch.Point {
		if q, ok := pts[p]; ok {
			return q
		}
		q := s.CreatePoint(p[0], p[1])
		s.Fix(q)
		pts[p] = q
		return q
	}
	for _, sg := range segs {
		if sg.arc {
			s.CreateArc(pt(sg.c), pt(sg.a), pt(sg.b))
			continue
		}
		s.CreateLine(pt(sg.a), pt(sg.b))
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	b, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], ext)
	require.NoError(t, err)
	return b
}

var (
	tcX = r3.NewVec(1, 0, 0)
	tcY = r3.NewVec(0, 1, 0)
	tcZ = r3.NewVec(0, 0, 1)
)

func tcAlong(d float64) decad.Extent {
	return decad.Distance{D: units.Millimeters(d), Dir: decad.Along}
}

// tcFacesEdges returns the outer-loop edges of the planar faces with normal
// ±normal at the given offsets, for which keep is true.
func tcFacesEdges(t *testing.T, b *decad.Body, normal r3.Vec, offsets []float64, keep func(*decad.Edge) bool) []*decad.Edge {
	t.Helper()
	var out []*decad.Edge
	for _, f := range b.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(math.Abs(pl.Frame.N().Dot(normal))-1) > 1e-9 {
			continue
		}
		d := pl.Frame.Origin().Dot(normal)
		hit := false
		for _, o := range offsets {
			hit = hit || math.Abs(d-o) < 1e-6
		}
		if !hit {
			continue
		}
		for _, l := range f.Loops() {
			if !l.IsOuter() {
				continue
			}
			for _, e := range l.Edges() {
				if keep(e) {
					out = append(out, e)
				}
			}
		}
	}
	return out
}

// tcSelect names edges by start point and kind.
func tcSelect(t *testing.T, edges []*decad.Edge) decad.EdgeSelector {
	t.Helper()
	require.NotEmpty(t, edges)
	var q *decad.EdgeQuery
	for _, e := range edges {
		start := e.Start().Position().Value
		var preds []decad.EdgePredicate
		switch e.Curve().(type) {
		case decad.Line3:
			preds = []decad.EdgePredicate{decad.EndpointAt(start), decad.ParallelTo(e.End().Position().Value.Sub(start))}
		default:
			preds = []decad.EdgePredicate{decad.EndpointAt(start), decad.Circular()}
		}
		if q == nil {
			q = decad.Edges(preds...)
			continue
		}
		q = q.Or(preds...)
	}
	return q.Exactly(len(edges))
}

// tcSkipAlong is true for a line whose ends differ along axis only.
func tcSkipAlong(axis r3.Vec) func(*decad.Edge) bool {
	return func(e *decad.Edge) bool {
		if _, ok := e.Curve().(decad.Line3); !ok {
			return false
		}
		d := e.End().Position().Value.Sub(e.Start().Position().Value)
		return d.Sub(axis.Scale(d.Dot(axis))).Len() < 1e-9
	}
}

func tcNot(f func(*decad.Edge) bool) func(*decad.Edge) bool {
	return func(e *decad.Edge) bool { return !f(e) }
}

func tcIsArc(e *decad.Edge) bool { _, ok := e.Curve().(decad.Arc3); return ok }

// tcBar intersects a plan prism, a half-circle end of radius r about the z axis, with an elevation prism, a
// half-circle end of radius r about the line x = l, z = r along y: a bar 2r square rounded about z at one end and
// about y at the other. Its rim runs over both caps and both sides as one closed tangent chain on four planes. The
// prisms overreach the bar by 1 so that no face of one lies in a plane of the other.
func tcBar(t *testing.T, sc *tcScene, r, l float64) *decad.Body {
	t.Helper()
	plan := sc.prism(t, r3.NewVec(0, 0, -1), tcX, tcY, []tcSeg{
		tcArc(0, 0, 0, r, 0, -r), tcLine(0, -r, l+r+1, -r), tcLine(l+r+1, -r, l+r+1, r), tcLine(l+r+1, r, 0, r),
	}, tcAlong(2*r+2))
	elev := sc.prism(t, r3.NewVec(0, r+1, 0), tcX, tcZ, []tcSeg{
		tcLine(-r-1, 0, l, 0), tcArc(l, r, l, 0, l, 2*r), tcLine(l, 2*r, -r-1, 2*r), tcLine(-r-1, 2*r, -r-1, 0),
	}, tcAlong(2*r+2))
	b, err := decad.Intersect(t.Context(), plan, elev)
	require.NoError(t, err)
	return b
}

// tcStepTool cuts away z > h for x < x1 from a bar of half-width r, with a round of radius q where the step's floor
// meets its wall at x = x1, so the floor's rim runs on round the round and up the wall.
func tcStepTool(t *testing.T, sc *tcScene, r, h, x1, q float64) *decad.Body {
	top := 2*r + 1
	return sc.prism(t, r3.NewVec(0, r+2, 0), tcX, tcZ, []tcSeg{
		tcLine(-r-2, h, x1-q, h), tcArc(x1-q, h+q, x1-q, h, x1, h+q), tcLine(x1, h+q, x1, top),
		tcLine(x1, top, -r-2, top), tcLine(-r-2, top, -r-2, h),
	}, tcAlong(2*r+4))
}

// tcSlotTool is the round-ended slot |y| < w from x = xEnd back to a half circle about x = cx, from z0 up h.
func tcSlotTool(t *testing.T, sc *tcScene, w, cx, xEnd, z0, h float64) *decad.Body {
	return sc.prism(t, r3.NewVec(0, 0, z0), tcX, tcY, []tcSeg{
		tcLine(xEnd, w, cx, w), tcArc(cx, 0, cx, w, cx, -w), tcLine(cx, -w, xEnd, -w), tcLine(xEnd, -w, xEnd, w),
	}, tcAlong(h))
}

// tcTongueCutter cuts |y| > 4 on side s for x < 20, from z = -11 up 22, with a radius-4 round where the tongue's
// face meets the shoulder at x = 20.
func tcTongueCutter(t *testing.T, sc *tcScene, s float64) *decad.Body {
	segs := []tcSeg{
		tcLine(-11, 4, 16, 4), tcArc(16, 8, 16, 4, 20, 8), tcLine(20, 8, 20, 11), tcLine(20, 11, -11, 11),
		tcLine(-11, 11, -11, 4),
	}
	if s < 0 {
		segs = []tcSeg{
			tcLine(-11, -11, 20, -11), tcLine(20, -11, 20, -8), tcArc(16, -8, 20, -8, 16, -4),
			tcLine(16, -4, -11, -4), tcLine(-11, -4, -11, -11),
		}
	}
	return sc.prism(t, r3.NewVec(0, 0, -11), tcX, tcY, segs, tcAlong(22))
}

func tcCut(t *testing.T, b *decad.Body, tools ...*decad.Body) *decad.Body {
	t.Helper()
	for _, tool := range tools {
		var err error
		b, err = decad.Cut(t.Context(), b, tool)
		require.NoError(t, err)
	}
	return b
}

const tcD = 0.5

func tcChamfer(t *testing.T, b *decad.Body, edges []*decad.Edge, opts ...decad.ChamferOption) (*decad.Body, error) {
	t.Helper()
	return tcChamferBy(t, b, edges, tcD, opts...)
}

func tcChamferBy(t *testing.T, b *decad.Body, edges []*decad.Edge, d float64, opts ...decad.ChamferOption) (*decad.Body, error) {
	t.Helper()
	return decad.Chamfer(t.Context(), b, tcSelect(t, edges), units.Millimeters(d), opts...)
}

// tcEndPiece is the volume a chamfer of setback tcD removes beyond its full wedge where its straight end runs onto an
// earlier 45° bevel that meets the edge's wall square: the wedge's section shrinks as (h)²/2 over the last tcD of
// height h, which adds tcD³/6.
const tcEndPiece = tcD * tcD * tcD / 6

// tcSurfaceKinds counts b's faces by surface: oblique planes (a normal off every axis), cones and faceted faces.
func tcSurfaceKinds(b *decad.Body) (oblique, cones, faceted int) {
	for _, f := range b.Faces() {
		switch s := f.Surface().(type) {
		case decad.Plane:
			n := s.Frame.N()
			axis := 0
			for _, c := range []float64{n.X, n.Y, n.Z} {
				if math.Abs(math.Abs(c)-1) < 1e-12 {
					axis++
				}
			}
			if axis == 0 {
				oblique++
			}
		case decad.Cone:
			cones++
		case decad.Faceted:
			faceted++
		}
	}
	return oblique, cones, faceted
}

// tcRequireExact checks that out bevels each edge of chain with an exact surface, as the chain rewrite builds it
// (agent-docs/retained-modify-reach-design.md §15b): one new oblique plane per line, one new cone per arc, and no new
// faceted face, which a Boolean Cut would leave.
func tcRequireExact(t *testing.T, before, out *decad.Body, chain []*decad.Edge) {
	t.Helper()
	lines, arcs := 0, 0
	for _, e := range chain {
		if tcIsArc(e) {
			arcs++
			continue
		}
		lines++
	}
	o0, c0, f0 := tcSurfaceKinds(before)
	o1, c1, f1 := tcSurfaceKinds(out)
	require.Equal(t, lines, o1-o0, "new oblique plane faces")
	require.Equal(t, arcs, c1-c0, "new cone faces")
	require.Equal(t, f0, f1, "faceted faces")
}

// tcRequireVertexAt checks that p is a vertex of b, to the tolerance the held mesh reads it with.
func tcRequireVertexAt(t *testing.T, b *decad.Body, p r3.Vec) {
	t.Helper()
	best := math.Inf(1)
	for _, e := range b.Edges() {
		for _, q := range []r3.Vec{e.Start().Position().Value, e.End().Position().Value} {
			best = math.Min(best, q.Sub(p).Len())
		}
	}
	require.Less(t, best, 1e-6, "no vertex at %v", p)
}

func tcAll(*decad.Edge) bool { return true }

// tcLinkLength is the distance between a stadium link's end axes.
const tcLinkLength = 120

// tcLink is a stadium link of length tcLinkLength between its end axes, 20 thick along y, with both y caps chamfered
// when capped, and cut to a tongue |y| <= 4 at x < 20 and a fork slot |y| < 4.26 at its far end.
func tcLink(t *testing.T, sc *tcScene, capped bool) *decad.Body {
	t.Helper()
	const l = tcLinkLength
	b := sc.prism(t, r3.Vec{}, tcX, tcZ, []tcSeg{
		tcLine(0, -10, l, -10), tcArc(l, 0, l, -10, l, 10), tcLine(l, 10, 0, 10), tcArc(0, 0, 0, 10, 0, -10),
	}, decad.Symmetric{D: units.Millimeters(10)})
	if capped {
		var err error
		b, err = tcChamfer(t, b, tcFacesEdges(t, b, tcY, []float64{-10, 10}, tcAll))
		require.NoError(t, err)
	}
	return tcCut(t, b, tcTongueCutter(t, sc, 1), tcTongueCutter(t, sc, -1),
		tcSlotTool(t, sc, 4.26, l-21.74, l+11, -11, 22))
}

// tcWedge is the volume a chamfer of setback d removes along a right-angled edge: d²/2 times the path of the
// wedge's centroid (Pappus). An arc of radius r and sweep θ contributes θ(r − d/3) when the material lies inside its
// circle and θ(r + d/3) when it lies outside.
func tcWedge(lines float64, inside, outside [][2]float64) float64 {
	path := lines
	for _, a := range inside {
		path += a[1] * (a[0] - tcD/3)
	}
	for _, a := range outside {
		path += a[1] * (a[0] + tcD/3)
	}
	return tcD * tcD / 2 * path
}

// tcRequireChamfer checks that out is before less a chamfer of setback tcD along every edge of chain: the volume it
// removed is want within the two readings' bounds, no edge of out still runs between the two ends of a chain edge,
// and every vertex the chamfer made lies on a bevel's rim, tcD from the line or circle of a chain edge, within the
// receiver's chord bound.
func tcRequireChamfer(t *testing.T, before, out *decad.Body, chain []*decad.Edge, want float64) {
	t.Helper()
	require.True(t, out.IsSolid())
	v0, err := before.Volume()
	require.NoError(t, err)
	v1, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, v0.Value.Base()-v1.Value.Base(), v0.Bound.Base()+v1.Bound.Base())
	same := func(a, b r3.Vec) bool { return a.Sub(b).Len() < 1e-9 }
	for _, e := range chain {
		p0, p1 := e.Start().Position().Value, e.End().Position().Value
		for _, o := range out.Edges() {
			q0, q1 := o.Start().Position().Value, o.End().Position().Value
			require.False(t, same(p0, q0) && same(p1, q1) || same(p0, q1) && same(p1, q0),
				"the edge from %v to %v is not chamfered", p0, p1)
		}
	}
	old := map[r3.Vec]struct{}{}
	for _, e := range before.Edges() {
		old[e.Start().Position().Value] = struct{}{}
		old[e.End().Position().Value] = struct{}{}
	}
	made := 0
	for _, e := range out.Edges() {
		for _, q := range []r3.Vec{e.Start().Position().Value, e.End().Position().Value} {
			if _, ok := old[q]; ok {
				continue
			}
			made++
			best := math.Inf(1)
			for _, c := range chain {
				best = math.Min(best, math.Abs(tcCurveDistance(c, q)-tcD))
			}
			require.Less(t, best, 0.02, "the vertex %v lies on no bevel's rim", q)
		}
	}
	require.Positive(t, made)
	tcRequireExact(t, before, out, chain)
}

// tcCurveDistance is the distance from q to the line or the circle that carries e.
func tcCurveDistance(e *decad.Edge, q r3.Vec) float64 {
	p0, p1 := e.Start().Position().Value, e.End().Position().Value
	if arc, ok := e.Curve().(decad.Arc3); ok {
		axis, _ := arc.Axis.Normalize()
		rel := q.Sub(arc.Center)
		h := rel.Dot(axis)
		rho := rel.Sub(axis.Scale(h)).Len()
		return math.Hypot(h, rho-arc.Radius.Base())
	}
	dir, _ := p1.Sub(p0).Normalize()
	rel := q.Sub(p0)
	return rel.Sub(dir.Scale(rel.Dot(dir))).Len()
}

// tcMidpoint is the point halfway along a line or an arc edge.
func tcMidpoint(e *decad.Edge) r3.Vec {
	p0, p1 := e.Start().Position().Value, e.End().Position().Value
	arc, ok := e.Curve().(decad.Arc3)
	if !ok {
		return p0.Add(p1).Scale(0.5)
	}
	axis, _ := arc.Axis.Normalize()
	a, b := p0.Sub(arc.Center), p1.Sub(arc.Center)
	sweep := math.Atan2(axis.Dot(a.Cross(b)), a.Dot(b))
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	half := sweep / 2
	return arc.Center.Add(a.Scale(math.Cos(half))).Add(axis.Cross(a).Scale(math.Sin(half)))
}

// tcEdgesWhere returns the edges of b whose midpoint keep accepts.
func tcEdgesWhere(b *decad.Body, keep func(e *decad.Edge, mid r3.Vec) bool) []*decad.Edge {
	var out []*decad.Edge
	for _, e := range b.Edges() {
		if _, line := e.Curve().(decad.Line3); !line && !tcIsArc(e) {
			continue
		}
		if keep(e, tcMidpoint(e)) {
			out = append(out, e)
		}
	}
	return out
}

func tcNear(a, b float64) bool { return math.Abs(a-b) < 1e-6 }

// TestChamferTangentChainAcrossPlanes chamfers tangent chains of mesh Boolean results whose edges share no one base
// plane (agent-docs/retained-modify-reach-design.md §15b), each exactly: every bevel is a plane or a cone.
func TestChamferTangentChainAcrossPlanes(t *testing.T) {
	t.Parallel()

	for _, size := range []struct{ r, l float64 }{{10, 70}, {6, 40}} {
		// The bar's rim: both caps' half circles and straight edges, and both sides' arcs about the far end's axis.
		barWant := tcWedge(4*size.l, [][2]float64{{size.r, math.Pi}, {size.r, math.Pi}, {size.r, math.Pi}, {size.r, math.Pi}}, nil)
		rim := func(b *decad.Body) []*decad.Edge {
			return tcEdgesWhere(b, func(e *decad.Edge, m r3.Vec) bool {
				if tcSkipAlong(tcY)(e) {
					return false
				}
				onCap := tcNear(m.Z, 0) || tcNear(m.Z, 2*size.r)
				return onCap || (tcNear(math.Abs(m.Y), size.r) && tcIsArc(e))
			})
		}
		t.Run(fmt.Sprintf("a bar of radius %g from its caps' edges", size.r), func(t *testing.T) {
			t.Parallel()
			b := tcBar(t, newTCScene(), size.r, size.l)
			chain := rim(b)
			require.Len(t, chain, 8)
			// The caps' edges end at the far end's round, whose arcs the chamfer takes as their tangent continuation.
			out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcZ, []float64{0, 2 * size.r}, tcNot(tcSkipAlong(tcY))))
			require.NoError(t, err)
			tcRequireChamfer(t, b, out, chain, barWant)
		})
		t.Run(fmt.Sprintf("a bar of radius %g from the arcs on its sides", size.r), func(t *testing.T) {
			t.Parallel()
			b := tcBar(t, newTCScene(), size.r, size.l)
			out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcY, []float64{-size.r, size.r}, tcIsArc))
			require.NoError(t, err)
			tcRequireChamfer(t, b, out, rim(b), barWant)
		})
		t.Run(fmt.Sprintf("a bar of radius %g from one seed under WithTangentChain", size.r), func(t *testing.T) {
			t.Parallel()
			b := tcBar(t, newTCScene(), size.r, size.l)
			seed := tcEdgesWhere(b, func(e *decad.Edge, m r3.Vec) bool {
				return tcIsArc(e) && tcNear(m.Z, 2*size.r) && m.X < 0
			})
			require.Len(t, seed, 1)
			out, err := tcChamfer(t, b, seed, decad.WithTangentChain())
			require.NoError(t, err)
			tcRequireChamfer(t, b, out, rim(b), barWant)
		})
	}

	// A step cut into the bar leaves a floor whose rim runs round the bar's end, along both sides, round the step's
	// round and up its wall to the top, an open chain whose ends meet the top face square.
	stepped := func(t *testing.T, sc *tcScene) *decad.Body {
		return tcCut(t, tcBar(t, sc, 10, 70), tcStepTool(t, sc, 10, 8, 30, 4))
	}
	stepRim := func(b *decad.Body) []*decad.Edge {
		return tcEdgesWhere(b, func(e *decad.Edge, m r3.Vec) bool {
			switch {
			case tcNear(m.Z, 8):
				return !tcSkipAlong(tcY)(e)
			case tcNear(math.Abs(m.Y), 10):
				return m.X > 25 && m.X <= 30+1e-9 && m.Z > 8 && m.Z < 20
			}
			return false
		})
	}
	t.Run("a step's floor rim open to the top face", func(t *testing.T) {
		t.Parallel()
		b := stepped(t, newTCScene())
		chain := stepRim(b)
		require.Len(t, chain, 7)
		out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcZ, []float64{8}, tcNot(tcSkipAlong(tcY))))
		require.NoError(t, err)
		want := tcWedge(2*26+2*8, [][2]float64{{10, math.Pi}}, [][2]float64{{4, math.Pi / 2}, {4, math.Pi / 2}})
		tcRequireChamfer(t, b, out, chain, want)
	})
	// The step's wall edges end on the bevels of the bar's top rim, which an earlier chamfer made. Each wall's bevel
	// ends on the old bevel's plane, which meets the wall square.
	for _, earlier := range []struct {
		name string
		d    float64
	}{
		// With the same setback the new bevel passes exactly through the old bevel's far corner on the top face.
		{"of the same setback", tcD},
		// A wider earlier bevel leaves part of its edge on the wall, beyond the new bevel's end.
		{"wider than the step's", 0.8},
	} {
		t.Run("a step's floor rim ending on an earlier bevel "+earlier.name, func(t *testing.T) {
			t.Parallel()
			sc := newTCScene()
			bar := tcBar(t, sc, 10, 70)
			capped, err := tcChamferBy(t, bar, tcFacesEdges(t, bar, tcZ, []float64{0, 20}, tcNot(tcSkipAlong(tcY))), earlier.d)
			require.NoError(t, err)
			b := tcCut(t, capped, tcStepTool(t, sc, 10, 8, 30, 4))
			chain := stepRim(b)
			require.Len(t, chain, 7)
			out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcZ, []float64{8}, tcNot(tcSkipAlong(tcY))))
			require.NoError(t, err)
			// Each wall edge runs from the round at z = 12 to the old bevel at z = 20 − earlier.d.
			wall := 20 - earlier.d - 12
			want := tcWedge(2*26+2*wall, [][2]float64{{10, math.Pi}}, [][2]float64{{4, math.Pi / 2}, {4, math.Pi / 2}}) +
				2*tcEndPiece
			tcRequireChamfer(t, b, out, chain, want)
			for _, s := range []float64{1, -1} {
				// The bevel's end on the old bevel: its corner on the side face, and its corner on the wall, which the
				// old bevel's far corner is when the setbacks agree.
				tcRequireVertexAt(t, out, r3.NewVec(30+tcD, 10*s, 20-earlier.d))
				tcRequireVertexAt(t, out, r3.NewVec(30, (10-tcD)*s, 20-earlier.d+tcD))
			}
		})
	}

	// A slot's rim runs along its sides, round the bar's far end on both sides, and round the slot's end on the top
	// and bottom faces.
	slotWant := tcWedge(4*21.74, [][2]float64{{10, math.Pi}, {10, math.Pi}}, [][2]float64{{4.26, math.Pi}, {4.26, math.Pi}})
	slotRim := func(b *decad.Body, end float64) []*decad.Edge {
		return tcEdgesWhere(b, func(e *decad.Edge, m r3.Vec) bool {
			if tcSkipAlong(tcZ)(e) {
				return false
			}
			side := tcNear(math.Abs(m.Y), 4.26) && m.X > end
			slotEnd := tcIsArc(e) && m.X < end && m.X > end-5
			return side || slotEnd
		})
	}
	t.Run("a slot through the bar's rounded end", func(t *testing.T) {
		t.Parallel()
		sc := newTCScene()
		b := tcCut(t, tcBar(t, sc, 10, 70), tcSlotTool(t, sc, 4.26, 70-21.74, 81, -1, 22))
		chain := slotRim(b, 70-21.74)
		require.Len(t, chain, 8)
		out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcY, []float64{-4.26, 4.26}, tcNot(tcSkipAlong(tcZ))))
		require.NoError(t, err)
		tcRequireChamfer(t, b, out, chain, slotWant)
	})
	t.Run("a slot through the bar's rounded end after its rim was chamfered", func(t *testing.T) {
		t.Parallel()
		sc := newTCScene()
		bar := tcBar(t, sc, 10, 70)
		capped, err := tcChamfer(t, bar, tcFacesEdges(t, bar, tcZ, []float64{0, 20}, tcNot(tcSkipAlong(tcY))))
		require.NoError(t, err)
		b := tcCut(t, capped, tcSlotTool(t, sc, 4.26, 70-21.74, 81, -1, 22))
		chain := slotRim(b, 70-21.74)
		require.Len(t, chain, 8)
		out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcY, []float64{-4.26, 4.26}, tcNot(tcSkipAlong(tcZ))))
		require.NoError(t, err)
		tcRequireChamfer(t, b, out, chain, slotWant)
	})
	t.Run("a slot through a link's end after its caps were chamfered", func(t *testing.T) {
		t.Parallel()
		const l = tcLinkLength
		b := tcLink(t, newTCScene(), true)
		chain := tcEdgesWhere(b, func(e *decad.Edge, m r3.Vec) bool {
			return m.X > l-30 && !tcSkipAlong(tcZ)(e) &&
				(tcNear(math.Abs(m.Y), 4.26) || (tcIsArc(e) && m.X < l-21.74 && m.X > l-27))
		})
		require.Len(t, chain, 8)
		out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcY, []float64{-4.26, 4.26}, tcNot(tcSkipAlong(tcZ))))
		require.NoError(t, err)
		tcRequireChamfer(t, b, out, chain, slotWant)
	})

	// A tongue's face rim runs round the link's end, along the top and bottom, round the shoulder's round and across
	// the shoulder to the cap: an open chain on three planes.
	t.Run("a tongue's rim open to uncut caps", func(t *testing.T) {
		t.Parallel()
		b := tcLink(t, newTCScene(), false)
		chain := tcEdgesWhere(b, func(e *decad.Edge, m r3.Vec) bool {
			if m.Y < 0 || m.X > 20+1e-9 || tcSkipAlong(tcZ)(e) || (tcSkipAlong(tcY)(e) && m.X < 1) {
				return false
			}
			return tcNear(m.Y, 4) || tcNear(math.Abs(m.Z), 10)
		})
		require.Len(t, chain, 7)
		out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcY, []float64{4}, tcNot(tcSkipAlong(tcZ))))
		require.NoError(t, err)
		want := tcWedge(2*16+2*2, [][2]float64{{10, math.Pi}}, [][2]float64{{4, math.Pi / 2}, {4, math.Pi / 2}})
		tcRequireChamfer(t, b, out, chain, want)
	})
	t.Run("a tongue's rim ending on an earlier cap bevel of the same setback", func(t *testing.T) {
		t.Parallel()
		b := tcLink(t, newTCScene(), true)
		chain := tcEdgesWhere(b, func(e *decad.Edge, m r3.Vec) bool {
			if m.Y < 0 || m.X > 20+1e-9 || tcSkipAlong(tcZ)(e) || (tcSkipAlong(tcY)(e) && m.X < 1) {
				return false
			}
			return tcNear(m.Y, 4) || (tcNear(math.Abs(m.Z), 10) && m.Y < 10-tcD)
		})
		require.Len(t, chain, 7)
		out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcY, []float64{4}, tcNot(tcSkipAlong(tcZ))))
		require.NoError(t, err)
		// The shoulder's edges run from the round at y = 8 to the cap's bevel at y = 9.5.
		want := tcWedge(2*16+2*1.5, [][2]float64{{10, math.Pi}}, [][2]float64{{4, math.Pi / 2}, {4, math.Pi / 2}}) +
			2*tcEndPiece
		tcRequireChamfer(t, b, out, chain, want)
		for _, s := range []float64{1, -1} {
			// The old bevel's far corner on the cap is the new bevel's corner on the shoulder.
			tcRequireVertexAt(t, out, r3.NewVec(20, 10, (10-tcD)*s))
			tcRequireVertexAt(t, out, r3.NewVec(20+tcD, 10-tcD, 10*s))
		}
	})
	t.Run("a link's tongue and fork faces in one call after its caps were chamfered", func(t *testing.T) {
		t.Parallel()
		b := tcLink(t, newTCScene(), true)
		out, err := tcChamfer(t, b, tcFacesEdges(t, b, tcY, []float64{-4.26, -4, 4, 4.26}, tcNot(tcSkipAlong(tcZ))))
		require.NoError(t, err)
		tongue := tcWedge(2*16+2*1.5, [][2]float64{{10, math.Pi}}, [][2]float64{{4, math.Pi / 2}, {4, math.Pi / 2}}) +
			2*tcEndPiece
		v0, err := b.Volume()
		require.NoError(t, err)
		v1, err := out.Volume()
		require.NoError(t, err)
		// Both fork faces' rims are one closed chain, joined round the slot's end on the top and bottom faces.
		require.InDelta(t, 2*tongue+slotWant, v0.Value.Base()-v1.Value.Base(), v0.Bound.Base()+v1.Bound.Base())
		require.True(t, out.IsSolid())
		_, _, f0 := tcSurfaceKinds(b)
		_, _, f1 := tcSurfaceKinds(out)
		require.Equal(t, f0, f1, "faceted faces")
	})
}
