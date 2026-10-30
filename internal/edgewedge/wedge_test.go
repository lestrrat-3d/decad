package edgewedge_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/edgewedge"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// corner builds the straight edge from a to b and the vertices it needs.
func corner(a, b r3.Vec, bound float64) *topo.Edge {
	return &topo.Edge{
		Curve: surfacegeom.Line3{},
		Start: &topo.Vertex{Position: a, Bound: units.Millimeters(bound)},
		End:   &topo.Vertex{Position: b, Bound: units.Millimeters(bound)},
	}
}

// planeFace builds a planar face through the origin spanned by u and v, whose
// outward normal is u × v, bounded by one loop through pts in order. Each
// side of the loop that joins shared's two ends is shared itself, walked in
// the loop's direction; every other side is a fresh line. The face joins
// shared's faces.
func planeFace(t *testing.T, u, v r3.Vec, outer bool, shared *topo.Edge, pts ...r3.Vec) {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, u, v)
	require.NoError(t, err)
	f := &topo.Face{Surface: surfacegeom.Plane{Frame: frame}}
	loop := &topo.Loop{Outer: outer}
	for i, a := range pts {
		b := pts[(i+1)%len(pts)]
		switch {
		case a == shared.Start.Position && b == shared.End.Position:
			loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: shared, Forward: true})
		case b == shared.Start.Position && a == shared.End.Position:
			loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: shared, Forward: false})
		default:
			loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: corner(a, b, 0), Forward: true})
		}
	}
	f.Loops = []*topo.Loop{loop}
	shared.Faces = append(shared.Faces, f)
}

var (
	ex = r3.Vec{X: 1}
	ey = r3.Vec{Y: 1}
	ez = r3.Vec{Z: 1}
)

// boxCorner is a box's top edge along y at x = 0, z = 0: the top face z = 0
// spans x 0..1 and the side face x = 0 spans z −1..0.
func boxCorner(t *testing.T) *topo.Edge {
	e := corner(r3.Vec{}, ey, 0)
	planeFace(t, ex, ey, true, e, r3.Vec{}, ex, r3.Vec{X: 1, Y: 1}, ey)
	planeFace(t, ez, ey, true, e, r3.Vec{}, ey, r3.Vec{Y: 1, Z: -1}, r3.Vec{Z: -1})
	return e
}

// insideCorner is a floor z = 0 spanning x −1..0 meeting a wall x = 0
// spanning z 0..1 whose material lies at x > 0.
func insideCorner(t *testing.T) *topo.Edge {
	e := corner(r3.Vec{}, ey, 0)
	planeFace(t, ex, ey, true, e, r3.Vec{}, ey, r3.Vec{X: -1, Y: 1}, r3.Vec{X: -1})
	planeFace(t, ez, ey, true, e, r3.Vec{}, r3.Vec{Z: 1}, r3.Vec{Y: 1, Z: 1}, ey)
	return e
}

func TestOfPlanarCorners(t *testing.T) {
	t.Parallel()

	t.Run("box edge is salient", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, edgewedge.Salient, edgewedge.Of(boxCorner(t)))
	})
	t.Run("inside corner is reflex", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, edgewedge.Reflex, edgewedge.Of(insideCorner(t)))
	})
	t.Run("stored loop direction does not matter", func(t *testing.T) {
		t.Parallel()
		for _, build := range []func(*testing.T) *topo.Edge{boxCorner, insideCorner} {
			e := build(t)
			want := edgewedge.Of(e)
			for _, f := range e.Faces {
				l := f.Loops[0]
				slices.Reverse(l.CoEdges)
				for i := range l.CoEdges {
					l.CoEdges[i].Forward = !l.CoEdges[i].Forward
				}
			}
			require.Equal(t, want, edgewedge.Of(e))
		}
	})
	t.Run("a hole walked like an outer loop reads the other side", func(t *testing.T) {
		t.Parallel()
		// The top face of the box edge as a hole loop: its material lies
		// outside the square, at x < 0, so the corner turns reflex.
		e := corner(r3.Vec{}, ey, 0)
		planeFace(t, ex, ey, false, e, r3.Vec{}, ex, r3.Vec{X: 1, Y: 1}, ey)
		planeFace(t, ez, ey, true, e, r3.Vec{}, r3.Vec{Z: 1}, r3.Vec{Y: 1, Z: 1}, ey)
		require.Equal(t, edgewedge.Reflex, edgewedge.Of(e))
	})
}

func TestOfUndecided(t *testing.T) {
	t.Parallel()

	t.Run("coplanar faces turn by zero", func(t *testing.T) {
		t.Parallel()
		e := corner(r3.Vec{}, ey, 0)
		planeFace(t, ex, ey, true, e, r3.Vec{}, ex, r3.Vec{X: 1, Y: 1}, ey)
		planeFace(t, ex, ey, true, e, r3.Vec{}, ey, r3.Vec{X: -1, Y: 1}, r3.Vec{X: -1})
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("a turn inside the tangent's displacement", func(t *testing.T) {
		t.Parallel()
		// The faces turn by 1e-6; the ends move 1e-3 on a unit edge, which
		// can swing the tangent by far more.
		e := corner(r3.Vec{}, ey, 1e-3)
		tilt := r3.Vec{X: -math.Cos(1e-6), Z: -math.Sin(1e-6)}
		planeFace(t, ex, ey, true, e, r3.Vec{}, ex, r3.Vec{X: 1, Y: 1}, ey)
		planeFace(t, ey, tilt, true, e, r3.Vec{}, ey, ey.Add(tilt), tilt)
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("a turn inside a small cylinder's normal shift", func(t *testing.T) {
		t.Parallel()
		// A 1000 mm line keeps the tangent's own displacement near 4e-6, but
		// its 1e-3 mm ends can turn a radius-0.01 cylinder's normal by more
		// than the 0.01 turn between the faces.
		e := corner(r3.Vec{}, r3.Vec{Y: 1000}, 1e-3)
		planeFace(t, ex, ey, true, e, r3.Vec{}, ex, r3.Vec{X: 1, Y: 1000}, r3.Vec{Y: 1000})
		axis := r3.Vec{X: -0.01 * math.Sin(0.01), Z: -0.01 * math.Cos(0.01)}
		e.Faces = append(e.Faces, &topo.Face{Surface: surfacegeom.Cylinder{Origin: axis, Axis: ey, Radius: units.Millimeters(0.01)}})
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
		// The same edge with exact ends decides.
		e.Start.Bound, e.End.Bound = units.Millimeters(0), units.Millimeters(0)
		require.NotEqual(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("faceted face", func(t *testing.T) {
		t.Parallel()
		e := boxCorner(t)
		e.Faces[1].Surface = surfacegeom.Faceted{Bound: units.Millimeters(0.01)}
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("faceted curve", func(t *testing.T) {
		t.Parallel()
		e := boxCorner(t)
		e.Curve = surfacegeom.FacetedCurve{Bound: units.Millimeters(0.01)}
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("two readings disagree", func(t *testing.T) {
		t.Parallel()
		// The side face's one loop is flagged a hole, so it reads the
		// material on the wrong side and contradicts the top face.
		e := boxCorner(t)
		e.Faces[1].Loops[0].Outer = false
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("a cylinder across the line", func(t *testing.T) {
		t.Parallel()
		// A cylinder whose axis crosses the line meets the plane at a
		// changing angle; both ends would read the same sign, but the class
		// is not one whose ends speak for the edge.
		e := corner(r3.Vec{}, ey, 0)
		planeFace(t, ex, ey, true, e, r3.Vec{}, ex, r3.Vec{X: 1, Y: 1}, ey)
		e.Faces = append(e.Faces, &topo.Face{Surface: surfacegeom.Cylinder{
			Origin: r3.Vec{X: -5, Y: 0.5}, Axis: ez, Radius: units.Millimeters(5),
		}})
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("an arc without a curve bound in the loop", func(t *testing.T) {
		t.Parallel()
		// The top face's far side is a half circle with no proven curve
		// displacement, so its loop has no area to orient it by.
		e := boxCorner(t)
		far := e.Faces[0].Loops[0].CoEdges[1].Edge
		far.Curve = surfacegeom.Arc3{Center: r3.Vec{X: 1, Y: 0.5}, Axis: ez, Radius: units.Millimeters(0.5)}
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
		far.CurveBounded = true
		require.Equal(t, edgewedge.Salient, edgewedge.Of(e))
	})
	t.Run("a loop with no area", func(t *testing.T) {
		t.Parallel()
		// The top face's loop runs out along the edge and straight back, so
		// it encloses nothing and states no walk sense.
		e := boxCorner(t)
		e.Faces[0].Loops[0].CoEdges = []topo.CoEdge{{Edge: e, Forward: true}, {Edge: corner(ey, r3.Vec{}, 0), Forward: true}}
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("an edge one face holds twice", func(t *testing.T) {
		t.Parallel()
		e := boxCorner(t)
		top := e.Faces[0].Loops[0]
		top.CoEdges = append(top.CoEdges, topo.CoEdge{Edge: e, Forward: false})
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("one face", func(t *testing.T) {
		t.Parallel()
		e := boxCorner(t)
		e.Faces = e.Faces[:1]
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
}

// A circle on the plane z = 0 meets a coaxial cylinder: a boss's root when
// the plane's material lies outside the circle, its top rim when inside.
func TestOfCircle(t *testing.T) {
	t.Parallel()
	circle := func(t *testing.T, hole, bounded bool) *topo.Edge {
		v := &topo.Vertex{Position: r3.Vec{X: 2}}
		e := &topo.Edge{
			Curve: surfacegeom.Circle3{Axis: ez, Radius: units.Millimeters(2)},
			Start: v, End: v, CurveBounded: bounded,
		}
		frame, err := r3.NewFrame(r3.Vec{}, ex, ey)
		require.NoError(t, err)
		plane := &topo.Face{Surface: surfacegeom.Plane{Frame: frame},
			Loops: []*topo.Loop{{Outer: !hole, CoEdges: []topo.CoEdge{{Edge: e, Forward: true}}}}}
		// The wall's material lies inside the cylinder.
		wall := &topo.Face{Surface: surfacegeom.Cylinder{Axis: ez, Radius: units.Millimeters(2)}}
		e.Faces = []*topo.Face{wall, plane}
		return e
	}
	t.Run("root", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, edgewedge.Reflex, edgewedge.Of(circle(t, true, true)))
	})
	t.Run("rim", func(t *testing.T) {
		t.Parallel()
		// A disk z = 0 under the cylinder with outward normal +z is a top
		// rim's cap.
		require.Equal(t, edgewedge.Salient, edgewedge.Of(circle(t, false, true)))
	})
	t.Run("no curve bound", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(circle(t, false, false)))
	})
}

// roundCircle builds a whole-circle edge through p about center and axis,
// with vertex bound delta, between a disk on the plane through origin
// spanned by u and v and a face carrying round.
func roundCircle(t *testing.T, origin, u, v r3.Vec, round surfacegeom.Surface, p, center, axis r3.Vec, delta float64) *topo.Edge {
	t.Helper()
	vertex := &topo.Vertex{Position: p, Bound: units.Millimeters(delta)}
	e := &topo.Edge{
		Curve: surfacegeom.Circle3{Center: center, Axis: axis, Radius: units.Millimeters(p.Sub(center).Len())},
		Start: vertex, End: vertex, CurveBounded: true,
	}
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	disk := &topo.Face{Surface: surfacegeom.Plane{Frame: frame},
		Loops: []*topo.Loop{{Outer: true, CoEdges: []topo.CoEdge{{Edge: e, Forward: true}}}}}
	e.Faces = []*topo.Face{{Surface: round}, disk}
	return e
}

// Each circle below leaves the class: the plane, the round face's axis or
// its centre is off the circle's axis by far more than ParallelDirs admits,
// though the turn at the vertex is large enough to decide.
func TestOfCircleMisaligned(t *testing.T) {
	t.Parallel()
	cyl := func(origin, axis r3.Vec) surfacegeom.Surface {
		return surfacegeom.Cylinder{Origin: origin, Axis: axis, Radius: units.Millimeters(2)}
	}
	p := r3.Vec{X: 2}
	tests := []struct {
		name  string
		u, v  r3.Vec
		round surfacegeom.Surface
	}{
		{"plane tilted from the axis", ex, r3.Vec{Y: math.Cos(0.1), Z: math.Sin(0.1)}, cyl(r3.Vec{}, ez)},
		{"cylinder axis tilted", ex, ey, cyl(r3.Vec{}, r3.Vec{Y: math.Sin(0.05), Z: math.Cos(0.05)})},
		{"cylinder off the axis", ex, ey, cyl(r3.Vec{X: 0.3}, ez)},
		{"sphere off the axis", ex, ey, surfacegeom.Sphere{Center: r3.Vec{X: 0.3, Z: -1}, Radius: units.Millimeters(math.Hypot(1.7, 1))}},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			e := roundCircle(t, r3.Vec{}, tc.u, tc.v, tc.round, p, r3.Vec{}, ez, 0)
			require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
		})
	}
}

// A vertex displacement turns a curved face's normal by more than the turn
// at the edge, though the tangent's own displacement does not reach it. The
// same circle with an exact vertex decides.
func TestOfCurvedNormalShift(t *testing.T) {
	t.Parallel()
	z0 := math.Sqrt(0.99)
	coneHalf := math.Acos(0.03)
	tests := []struct {
		name   string
		origin r3.Vec
		round  surfacegeom.Surface
		p      r3.Vec
		delta  float64
	}{
		// The unit sphere meets z = z0 in a radius-0.1 circle at a 0.1 turn.
		{"sphere", r3.Vec{Z: z0}, surfacegeom.Sphere{Radius: units.Millimeters(1)}, r3.Vec{X: 0.1, Z: z0}, 0.0045},
		// A cone of half angle acos(0.03) meets z = 0 at a 0.03 turn.
		{"cone", r3.Vec{}, surfacegeom.Cone{Axis: ez, Radius: units.Millimeters(1), HalfAngle: units.Radians(coneHalf)},
			r3.Vec{X: 1}, 0.0099},
		// A major-10, minor-1 torus meets z = z0 at a 0.1 turn, where its
		// normal swings with the tube centre.
		{"torus", r3.Vec{Z: z0}, surfacegeom.Torus{Axis: ez, Major: units.Millimeters(10), Minor: units.Millimeters(1)},
			r3.Vec{X: 10.1, Z: z0}, 0.02},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			center := r3.Vec{Z: tc.origin.Z}
			e := roundCircle(t, tc.origin, ex, ey, tc.round, tc.p, center, ez, tc.delta)
			require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
			exact := roundCircle(t, tc.origin, ex, ey, tc.round, tc.p, center, ez, 0)
			require.NotEqual(t, edgewedge.Undecided, edgewedge.Of(exact))
		})
	}
}

// boundTopLoop gives the box edge's top face loop, all but the shared edge,
// vertex bound d.
func boundTopLoop(e *topo.Edge, d float64) {
	for _, ce := range e.Faces[0].Loops[0].CoEdges {
		if ce.Edge == e {
			continue
		}
		ce.Edge.Start.Bound, ce.Edge.End.Bound = units.Millimeters(d), units.Millimeters(d)
	}
}

func TestOfLoopAreaBound(t *testing.T) {
	t.Parallel()

	t.Run("vertex displacement band", func(t *testing.T) {
		t.Parallel()
		// The unit top loop, perimeter 4, with corners displaced by 0.11:
		// 2·4·0.11 + 4π·0.11² ≈ 1.03 covers its area of 1, though either
		// term alone does not.
		e := boxCorner(t)
		boundTopLoop(e, 0.11)
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
		boundTopLoop(e, 0.05)
		require.Equal(t, edgewedge.Salient, edgewedge.Of(e))
	})
	t.Run("plane normal bound", func(t *testing.T) {
		t.Parallel()
		// The band of 0.1 displacement, about 0.93, leaves the area of 1
		// decided; a 0.15 normal bound on the top face tilts the area's
		// reading by up to 0.15 of it, which closes the gap.
		e := boxCorner(t)
		boundTopLoop(e, 0.1)
		require.Equal(t, edgewedge.Salient, edgewedge.Of(e))
		e.Faces[0].NormalBound = 0.15
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("summation of many terms", func(t *testing.T) {
		t.Parallel()
		// A sliver 3.7e-12 wide beside the unit edge, walked back through
		// 500 points: each term's own rounding allowance leaves its area
		// decided, but the running sum of 502 terms may drift by more.
		const n = 500
		const h = 3.7e-12
		pts := []r3.Vec{{}, ey}
		for k := range n {
			pts = append(pts, r3.Vec{X: h, Y: 1 - float64(k)/float64(n-1)})
		}
		e := corner(r3.Vec{}, ey, 0)
		planeFace(t, ex, ey, true, e, pts...)
		planeFace(t, ez, ey, true, e, r3.Vec{}, ey, r3.Vec{Y: 1, Z: -1}, r3.Vec{Z: -1})
		require.Len(t, e.Faces[0].Loops[0].CoEdges, n+2)
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
	t.Run("arc sweep within its ends' displacement", func(t *testing.T) {
		t.Parallel()
		// A radius-100 arc sweeps about 0.001 between ends displaced by
		// 0.06, which could put the true ends either way round: the held
		// sweep cannot say whether the arc is short or nearly whole.
		e := corner(r3.Vec{}, ey, 0)
		planeFace(t, ex, ey, true, e, r3.Vec{}, ex, r3.Vec{X: 1, Y: 0.45}, r3.Vec{X: 1, Y: 0.55}, r3.Vec{X: 1, Y: 1}, ey)
		planeFace(t, ez, ey, true, e, r3.Vec{}, ey, r3.Vec{Y: 1, Z: -1}, r3.Vec{Z: -1})
		arc := e.Faces[0].Loops[0].CoEdges[2].Edge
		arc.Curve = surfacegeom.Arc3{Center: r3.Vec{X: -99, Y: 0.5}, Axis: ez, Radius: units.Millimeters(math.Hypot(100, 0.05))}
		arc.CurveBounded = true
		require.Equal(t, edgewedge.Salient, edgewedge.Of(e))
		arc.Start.Bound, arc.End.Bound = units.Millimeters(0.06), units.Millimeters(0.06)
		require.Equal(t, edgewedge.Undecided, edgewedge.Of(e))
	})
}

func TestView(t *testing.T) {
	t.Parallel()
	reflex := edgewedge.View{Edge: insideCorner(t)}
	decided, isReflex := reflex.SelectorWedge()
	require.True(t, decided)
	require.True(t, isReflex)
	salient := edgewedge.View{Edge: boxCorner(t)}
	decided, isReflex = salient.SelectorWedge()
	require.True(t, decided)
	require.False(t, isReflex)
	e := boxCorner(t)
	e.Faces = e.Faces[:1]
	decided, _ = edgewedge.View{Edge: e}.SelectorWedge()
	require.False(t, decided)
}
