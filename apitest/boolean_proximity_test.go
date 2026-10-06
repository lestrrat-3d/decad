package apitest_test

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// placedPrism is a regular n-gon prism placed by a rigid motion: the polygon of
// circumradius r, turned by phase radians, sits in the local XY plane and is
// extruded l along local +Z, and frame carries local to world.
type placedPrism struct {
	frame        r3.Transform
	l, r, phase  float64
	n            int
	bottom, top  []r3.Vec
	centroidHint r3.Vec
}

// axisFrame is the rigid motion taking local +Z to dir and the origin to a,
// built the way the mtilt hand-off places its prisms.
func axisFrame(t *testing.T, a, dir r3.Vec) r3.Transform {
	t.Helper()
	d, ok := dir.Normalize()
	require.True(t, ok)
	ref := r3.Vec{Y: 1}
	if math.Abs(d.Y) > 0.9 {
		ref = r3.Vec{X: 1}
	}
	p, ok := d.Cross(ref).Normalize()
	require.True(t, ok)
	tr, err := r3.FromBasis(r3.Basis{EX: p.Cross(d), EY: p, EZ: d}, a)
	require.NoError(t, err)
	return tr
}

func newPlacedPrism(frame r3.Transform, l, r float64, n int, phase float64) placedPrism {
	pp := placedPrism{frame: frame, l: l, r: r, n: n, phase: phase}
	for i := range n {
		th := 2*math.Pi*float64(i)/float64(n) + phase
		x, y := r*math.Cos(th), r*math.Sin(th)
		pp.bottom = append(pp.bottom, frame.Apply(r3.Vec{X: x, Y: y}))
		pp.top = append(pp.top, frame.Apply(r3.Vec{X: x, Y: y, Z: l}))
	}
	pp.centroidHint = frame.Apply(r3.Vec{Z: l / 2})
	return pp
}

// body builds the prism in doc: the polygon drawn on an XY sketch, extruded,
// then placed by the prism's frame.
func (pp placedPrism) body(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := make([]*sketch.Point, pp.n)
	for i := range pp.n {
		th := 2*math.Pi*float64(i)/float64(pp.n) + pp.phase
		pts[i] = s.CreatePoint(pp.r*math.Cos(th), pp.r*math.Sin(th))
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(pp.l), Dir: decad.Along})
	require.NoError(t, err)
	body, err = body.Placed(t.Context(), pp.frame)
	require.NoError(t, err)
	return body
}

// faces lists the prism's boundary polygons in world coordinates.
func (pp placedPrism) faces() [][]r3.Vec {
	out := [][]r3.Vec{append([]r3.Vec(nil), pp.bottom...), append([]r3.Vec(nil), pp.top...)}
	for i := range pp.n {
		j := (i + 1) % pp.n
		out = append(out, []r3.Vec{pp.bottom[i], pp.bottom[j], pp.top[j], pp.top[i]})
	}
	return out
}

// clipConvexPolytope keeps the part of a convex polytope, given by its face
// polygons, on the side of the plane (point o, normal n) that holds inside. It
// is the test's own independent reference for an intersection volume: plain
// float half-space clipping, sharing no code with the evaluator.
func clipConvexPolytope(faces [][]r3.Vec, o, n, inside r3.Vec) [][]r3.Vec {
	side := func(p r3.Vec) float64 { return n.Dot(p.Sub(o)) }
	if side(inside) > 0 {
		n = n.Scale(-1)
	}
	var out [][]r3.Vec
	var cut []r3.Vec
	for _, f := range faces {
		var kept []r3.Vec
		for i := range f {
			p, q := f[i], f[(i+1)%len(f)]
			sp, sq := side(p), side(q)
			if sp <= 0 {
				kept = append(kept, p)
			}
			if (sp < 0 && sq > 0) || (sp > 0 && sq < 0) {
				x := p.Add(q.Sub(p).Scale(sp / (sp - sq)))
				kept = append(kept, x)
				cut = append(cut, x)
			}
		}
		if len(kept) >= 3 {
			out = append(out, kept)
		}
	}
	if len(cut) < 3 {
		return out
	}
	var g r3.Vec
	for _, p := range cut {
		g = g.Add(p)
	}
	g = g.Scale(1 / float64(len(cut)))
	u, _ := cut[0].Sub(g).Normalize()
	v := n.Cross(u)
	sort.Slice(cut, func(i, j int) bool {
		a, b := cut[i].Sub(g), cut[j].Sub(g)
		return math.Atan2(a.Dot(v), a.Dot(u)) < math.Atan2(b.Dot(v), b.Dot(u))
	})
	return append(out, cut)
}

// convexPolytopeVolume sums one pyramid per face from a point inside the
// polytope: (1/3)·area·height.
func convexPolytopeVolume(faces [][]r3.Vec) float64 {
	var c r3.Vec
	count := 0
	for _, f := range faces {
		for _, p := range f {
			c = c.Add(p)
			count++
		}
	}
	c = c.Scale(1 / float64(count))
	vol := 0.0
	for _, f := range faces {
		var area r3.Vec
		for i := 1; i+1 < len(f); i++ {
			area = area.Add(f[i].Sub(f[0]).Cross(f[i+1].Sub(f[0])))
		}
		n, ok := area.Normalize()
		if !ok {
			continue
		}
		vol += area.Len() / 2 * math.Abs(n.Dot(f[0].Sub(c))) / 3
	}
	return vol
}

// intersectionVolume clips a by every face plane of b.
func intersectionVolume(a, b placedPrism) float64 {
	poly := a.faces()
	for _, f := range b.faces() {
		n := f[1].Sub(f[0]).Cross(f[2].Sub(f[0]))
		poly = clipConvexPolytope(poly, f[0], n, b.centroidHint)
	}
	return convexPolytopeVolume(poly)
}

// TestUnionOfTiltedPrismsCrossingMidWall unions two placed octagonal prisms
// whose walls cross far from every facet corner: B leans 30 degrees off A's
// axis and its 11.5 mm walls cross A's 11 mm walls near their middles. The
// placement rounding gives every face a positive displacement bound, so the
// hidden-tangency gate must prove each close face pair crosses deeper than that
// bound. The seven fixed sample points of those long wall facets all lie
// outside the other prism; the witness comes from the walk along each contact
// segment (boolean.go, spanWitness).
func TestUnionOfTiltedPrismsCrossingMidWall(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	joint := r3.Vec{Z: 10}
	lean := r3.Vec{X: math.Sin(math.Pi / 6), Z: math.Cos(math.Pi / 6)}
	pa := newPlacedPrism(axisFrame(t, r3.Vec{}, r3.Vec{Z: 1}), 11, 1.5, 8, 0)
	pb := newPlacedPrism(axisFrame(t, joint.Sub(lean.Scale(1.5)), lean), 11.5, 1.2, 8, 0.2)

	u, err := decad.Union(t.Context(), pa.body(t, doc), pb.body(t, doc))
	require.NoError(t, err)
	require.True(t, u.IsSolid())
	require.Len(t, u.Lumps(), 1)

	// V(A ∪ B) = V(A) + V(B) − V(A ∩ B). Both prisms are convex, so A ∩ B is
	// the convex polytope A clipped by B's ten face planes.
	octagon := func(r float64) float64 { return 4 * r * r * math.Sin(math.Pi/4) }
	want := octagon(1.5)*11 + octagon(1.2)*11.5 - intersectionVolume(pa, pb)
	vol, err := u.Volume()
	require.NoError(t, err)
	got, bound := volumeMM(t, vol), boundMM3(t, vol)
	require.Positive(t, bound, `the placed operands carry rounding, so the volume is bounded, not exact`)
	require.Less(t, bound, 1e-9, `planar operands compose a bound at rounding scale`)
	// The reference is a float computation over a few hundred operations on
	// values near 10, so it carries its own error of a few 1e-14; 1e-13 covers
	// it and stays below the bound.
	require.InDelta(t, want, got, bound+1e-13, `the proven interval encloses the analytic union volume`)

	m, err := u.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, m.BoundaryVerified())
}

// lyingPrismFrame places an octagonal prism of circumradius r on its side:
// local +Z runs along (cos 0.3, sin 0.3, 0) from (4, 5, axisZ) and local +Y is
// world +Z. The rows that set a point's height are exact, so the ring vertex at
// 270 degrees, and the long edge through it, sit at exactly axisZ − r.
func lyingPrismFrame(t *testing.T, axisZ float64) r3.Transform {
	t.Helper()
	c, s := math.Cos(0.3), math.Sin(0.3)
	frame, err := r3.FromBasis(r3.Basis{EX: r3.Vec{X: -s, Y: c}, EY: r3.Vec{Z: 1}, EZ: r3.Vec{X: c, Y: s}}, r3.Vec{X: 4, Y: 5, Z: axisZ})
	require.NoError(t, err)
	return frame
}

// TestUnionRefusesPlacedPrismGrazingAlongAnEdge pins that the hidden-tangency
// gate, segment walk included, admits nothing a touch can produce. B is a
// placed octagonal prism lying on its side whose lowest long edge rests on the
// top of slab A (z = 1). The placement rounds B's other coordinates, so the gate
// runs with a bound near 1e-15 mm, finds contact segments along that edge, and
// walks A's top facet from them.
//
// In the touching case the edge lies exactly in A's top plane: the walked
// points are deeper than the bound from B's boundary but outside B, so the
// exact parity rejects every one. In the shallow case the edge sits one ulp
// (2.2e-16 mm) below the plane: B's edge vertices lie inside A, but no deeper
// than the bound, so the certified depth rejects every one. Both unions are
// refused as undecidable.
func TestUnionRefusesPlacedPrismGrazingAlongAnEdge(t *testing.T) {
	t.Parallel()
	const r = 0.25
	testcases := []struct {
		Name  string
		AxisZ float64
		EdgeZ float64
	}{
		{Name: "touching", AxisZ: 1 + r, EdgeZ: 1},
		{Name: "crossing shallower than the bound", AxisZ: 1 + r - 0x1p-52, EdgeZ: 1 - 0x1p-52},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			a := boxBody(t, doc, 0, 0, 20, 20, 1)
			pb := newPlacedPrism(lyingPrismFrame(t, tc.AxisZ), 10, r, 8, 0)
			require.Equal(t, tc.EdgeZ, pb.bottom[6].Z, `B's lowest long edge sits where the case says`)
			require.Equal(t, tc.EdgeZ, pb.top[6].Z)

			_, err := decad.Union(t.Context(), a, pb.body(t, doc))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, `without provably interpenetrating`)
		})
	}
}
