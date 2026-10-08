package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// internalClassBTool extrudes one profile drawn on plane by depth to each
// side of it: draw adds the profile's entities to the sketch.
func internalClassBTool(t *testing.T, doc *Document, w *sketch.World, plane *sketch.Plane, depth float64, draw func(*sketch.Sketch)) *Body {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := doc.Extrude(s, profiles[0], Symmetric{D: units.Millimeters(depth)})
	require.NoError(t, err)
	return body
}

// internalDrillAlongY is a radius-r cylinder along y through (x, ·, z),
// sketched on an XZ plane moved to y = 10 and extruded 11 to each side, so
// it spans y from −1 to 21.
func internalDrillAlongY(t *testing.T, doc *Document, x, z, r float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	return internalClassBTool(t, doc, w, plane, 11, func(s *sketch.Sketch) {
		c := s.CreatePoint(x, z)
		s.Fix(c)
		s.CreateCircle(c, r)
	})
}

// internalCrossDrilled is §9's S1: a 40×20×20 box cut by a Ø6 hole along y
// through (20, ·, 10).
func internalCrossDrilled(t *testing.T) (*Document, *Body) {
	t.Helper()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	drill := internalDrillAlongY(t, doc, 20, 10, 3)
	result, err := Cut(t.Context(), box, drill)
	require.NoError(t, err)
	return doc, result
}

func requireMeshPathResult(t *testing.T, body *Body) {
	t.Helper()
	_, faceted := body.payload.(facetedPayload)
	require.True(t, faceted, `the pair took the mesh path, got %T`, body.payload)
}

func TestClassBCutCrossDrilledBox(t *testing.T) {
	t.Parallel()
	doc, result := internalCrossDrilled(t)
	bp, ok := result.payload.(brepPayload)
	require.True(t, ok, `a cross-drilled box builds a brep body, got %T`, result.payload)
	require.Len(t, doc.Bodies(), 1, `both operands are consumed`)
	requireClosedTopology(t, result)
	require.Len(t, result.Faces(), 7)
	cylinders, holed := 0, 0
	for _, f := range result.Faces() {
		if _, ok := f.Surface().(Cylinder); ok {
			cylinders++
			require.True(t, f.reversed, `the hole's wall faces into the hole`)
		}
		if _, planar := f.Surface().(Plane); planar && len(f.Loops()) == 2 {
			holed++
			_, isCircle := f.Loops()[1].Edges()[0].Curve().(Circle3)
			require.True(t, isCircle, `a y wall's inner loop is the hole's rim`)
		}
	}
	require.Equal(t, 1, cylinders)
	require.Equal(t, 2, holed, `the walls at y = 0 and y = 20 each carry one hole`)
	// 16000 − 9π·20, with a bound below 1e-9 mm³ (§9).
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(-180, 1))
	requireCoversInterval(t, result.volume, lo, hi)
	require.Equal(t, Approximate, result.volume.Exactness)
	require.Less(t, result.volume.Bound.Base(), 1e-9)
	// Caps 2·800, x walls 2·400, y walls 2·(800 − 9π), the hole 6π·20.
	lo, hi = piEnclosed(big.NewRat(4000, 1), big.NewRat(102, 1))
	requireCoversInterval(t, result.area, lo, hi)
	requireCentroidCovers(t, result.centroid, [3]*big.Rat{big.NewRat(20, 1), big.NewRat(10, 1), big.NewRat(10, 1)})
	require.Equal(t, r3.NewVec(0, 0, 0), result.bounds.Min)
	require.Equal(t, r3.NewVec(40, 20, 20), result.bounds.Max)
	// The hole's wall runs exactly between the two walls it pierces.
	for _, f := range bp.faces {
		if f.planar() {
			continue
		}
		if _, ok := f.wall.(CircleSeg); ok {
			require.Equal(t, [2]float64{-20, 0}, [2]float64{f.z0, f.z1}, `the drill's frame normal is −y`)
		}
	}
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := report.ForBody(result)
	require.NoError(t, err)
	require.Equal(t, Sound, br.Status)
	mesh, err := tessellateContext(t.Context(), result, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	held := internalMeshVolumeRat(mesh)
	bound := new(big.Rat).SetFloat64(mesh.volSymDiff)
	lo, hi = piEnclosed(big.NewRat(16000, 1), big.NewRat(-180, 1))
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, lo)).Cmp(bound), 0)
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(held, hi)).Cmp(bound), 0)
}

func TestClassBCutCrossSlot(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	slot := internalClassBTool(t, doc, w, plane, 11, func(s *sketch.Sketch) {
		r := s.CreateRectangle(15, 5, 25, 15)
		s.Fix(r.A)
	})
	result, err := Cut(t.Context(), box, slot)
	require.NoError(t, err)
	_, ok := result.payload.(brepPayload)
	require.True(t, ok, `a cross slot builds a brep body, got %T`, result.payload)
	requireClosedTopology(t, result)
	// B1: 16000 − 10·10·20, Exact, and every face planar.
	require.Len(t, result.Faces(), 10)
	for _, f := range result.Faces() {
		_, planar := f.Surface().(Plane)
		require.True(t, planar)
	}
	require.Len(t, result.Edges(), 24)
	require.Equal(t, 14000.0, result.volume.Value.Base())
	require.Equal(t, Exact, result.volume.Exactness)
	// 2·800 + 2·400 + 2·(800 − 100) + the slot's 4·10·20.
	require.Equal(t, 4600.0, result.area.Value.Base())
	require.Equal(t, Exact, result.area.Exactness)
	require.Equal(t, r3.NewVec(20, 10, 10), result.centroid.Value)
	require.Equal(t, Exact, result.centroid.Exactness)
}

// TestClassBRootedCutKeepsThePrismsConvexity drills a blind Ø6 hole along
// −y into the wall y = 20 of an L prism, (0,0) (40,0) (40,20) (20,20)
// (20,40) (0,40), z 0..20, through (30, ·, 10) down to y = 12. The pierced
// wall becomes a planar face recording the prism's axis, so the vertical line
// it shares with the swept wall x = 20 at the reflex corner (20, 20) is a
// junction, read from the turn as the plain prism reads it: concave. Every
// straight edge carries the plain prism's answer, and the hole's two circles
// read concave. Shown to fail with the side-versus-planar arm of
// brepgeom.Convex reading the planar face's loop role, and with the slab face
// recording no sweep (the reflex line then read convex either way).
func TestClassBRootedCutKeepsThePrismsConvexity(t *testing.T) {
	t.Parallel()
	doc := New()
	l := internalPolyPrismBody(t, doc, [][2]float64{{0, 0}, {40, 0}, {40, 20}, {20, 20}, {20, 40}, {0, 40}}, 20)
	want := internalConvexityByEnds(t, l)
	require.Len(t, want, 18)
	require.False(t, want[[2]r3.Vec{r3.NewVec(20, 20, 0), r3.NewVec(20, 20, 20)}])
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -20)
	require.NoError(t, err)
	drill := internalClassBTool(t, doc, w, plane, 8, func(s *sketch.Sketch) {
		c := s.CreatePoint(30, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
	})
	result, err := Cut(t.Context(), l, drill)
	require.NoError(t, err)
	bp, ok := result.payload.(brepPayload)
	require.True(t, ok, `got %T`, result.payload)
	requireClosedTopology(t, result)
	// 24000 − 9π·8: the drill reaches y = 12 inside the arm x 20..40.
	lo, hi := piEnclosed(big.NewRat(24000, 1), big.NewRat(-72, 1))
	requireCoversInterval(t, result.volume, lo, hi)
	pierced := 0
	for _, f := range bp.faces {
		if f.planar() && f.sweep != (r3.Vec{}) {
			pierced++
			require.Len(t, f.region.Holes, 1, "the pierced wall carries the hole")
		}
	}
	require.Equal(t, 1, pierced)

	require.Equal(t, want, internalConvexityByEnds(t, result))
	circles := 0
	for _, e := range result.Edges() {
		if _, ok := e.Curve().(Circle3); ok {
			circles++
			require.False(t, e.IsConvex(), "a hole's rim reads concave")
		}
	}
	require.Equal(t, 2, circles)
}

func TestClassBCutExactOffsetPlane(t *testing.T) {
	t.Parallel()
	// A drill sketched on a plane whose origin sits off the box's: the shift
	// is admitted when every shifted coordinate is a float, and misses to the
	// mesh path when one would round.
	cases := []struct {
		name    string
		originX float64
		brep    bool
	}{
		{name: "exact shift", originX: 0.5, brep: true},
		{name: "rounding shift", originX: 0.1, brep: false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			doc := New()
			box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
			w := sketch.NewWorld()
			frame, err := r3.NewFrame(r3.NewVec(tc.originX, 10, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
			require.NoError(t, err)
			plane, err := w.CreatePlaneFromFrame(frame)
			require.NoError(t, err)
			drill := internalClassBTool(t, doc, w, plane, 11, func(s *sketch.Sketch) {
				c := s.CreatePoint(20-tc.originX, 10)
				s.Fix(c)
				s.CreateCircle(c, 3)
			})
			x := box.payload.(prismPayload)
			_, admitted, err := admitClassBPair(t.Context(), box, drill)
			require.NoError(t, err)
			require.Equal(t, tc.brep, admitted)
			if !tc.brep {
				exact := new(big.Rat).Add(new(big.Rat).SetFloat64(20-tc.originX), new(big.Rat).SetFloat64(tc.originX))
				require.NotZero(t, exact.Cmp(big.NewRat(20, 1)), `the fixture's shifted centre is not a float`)
			}
			result, err := Cut(t.Context(), box, drill)
			require.NoError(t, err)
			if !tc.brep {
				requireMeshPathResult(t, result)
				return
			}
			bp, ok := result.payload.(brepPayload)
			require.True(t, ok)
			require.Equal(t, x.frame.Origin(), bp.faces[len(bp.faces)-1].frame.Origin(), `the tool's walls share the box's origin`)
			lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(-180, 1))
			requireCoversInterval(t, result.volume, lo, hi)
		})
	}
}

func TestClassBCutGateMissesTakeMeshPath(t *testing.T) {
	t.Parallel()
	box := func(doc *Document) *Body { return internalBoxBody(t, doc, 0, 0, 40, 20, 20) }
	toolOn := func(t *testing.T, doc *Document, frame r3.Frame, r float64) *Body {
		w := sketch.NewWorld()
		plane, err := w.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		return internalClassBTool(t, doc, w, plane, 15, func(s *sketch.Sketch) {
			c := s.CreatePoint(0, 0)
			s.Fix(c)
			s.CreateCircle(c, r)
		})
	}
	frameOf := func(t *testing.T, o, u, v r3.Vec) r3.Frame {
		f, err := r3.NewFrame(o, u, v)
		require.NoError(t, err)
		return f
	}
	cases := []struct {
		name   string
		tool   func(t *testing.T, doc *Document) *Body
		target func(doc *Document) *Body
	}{
		{name: "tilted plane (S10)", tool: func(t *testing.T, doc *Document) *Body {
			s, c := math.Sin(math.Pi/6), math.Cos(math.Pi/6)
			return toolOn(t, doc, frameOf(t, r3.NewVec(20, 10, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, s, c)), 3)
		}},
		{name: "a hair from perpendicular", tool: func(t *testing.T, doc *Document) *Body {
			tilt := 1e-12
			body := toolOn(t, doc, frameOf(t, r3.NewVec(20, 10, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, tilt, 1)), 3)
			n := body.payload.(prismPayload).frame.N()
			require.NotZero(t, n.Dot(r3.NewVec(0, 0, 1)), `the fixture's normals are not exactly perpendicular`)
			return body
		}},
		{name: "not a signed permutation", tool: func(t *testing.T, doc *Document) *Body {
			return toolOn(t, doc, frameOf(t, r3.NewVec(20, 10, 10), r3.NewVec(0.6, 0, 0.8), r3.NewVec(-0.8, 0, 0.6)), 3)
		}},
		{name: "cylinder across cylinder (S7)", tool: func(t *testing.T, doc *Document) *Body {
			return internalDrillAlongY(t, doc, 0, 10, 5)
		}, target: func(doc *Document) *Body {
			return internalCircleBody(t, doc, 0, 5, 0, Distance{D: units.Millimeters(20), Dir: Along})
		}},
		{name: "slanted tool wall (B6)", tool: func(t *testing.T, doc *Document) *Body {
			w := sketch.NewWorld()
			plane, err := w.CreateOffsetPlane(w.XZ(), -10)
			require.NoError(t, err)
			return internalClassBTool(t, doc, w, plane, 11, func(s *sketch.Sketch) {
				pts := []*sketch.Point{s.CreatePoint(15, 5), s.CreatePoint(25, 5), s.CreatePoint(20, 15)}
				for i, p := range pts {
					s.Fix(p)
					s.CreateLine(p, pts[(i+1)%len(pts)])
				}
			})
		}},
		{name: "tool face in the box's cap plane (B8)", tool: func(t *testing.T, doc *Document) *Body {
			w := sketch.NewWorld()
			plane, err := w.CreateOffsetPlane(w.XZ(), -10)
			require.NoError(t, err)
			return internalClassBTool(t, doc, w, plane, 11, func(s *sketch.Sketch) {
				r := s.CreateRectangle(15, 5, 25, 20)
				s.Fix(r.A)
			})
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			doc := New()
			target := box
			if tc.target != nil {
				target = tc.target
			}
			a := target(doc)
			b := tc.tool(t, doc)
			_, ok, err := tryClassB(t.Context(), meshbool.OpCut, a, b)
			require.NoError(t, err)
			require.False(t, ok)
			result, err := Cut(t.Context(), a, b)
			if err != nil {
				var be *BooleanError
				require.ErrorAs(t, err, &be, `a miss reaches the mesh path's own result`)
				return
			}
			requireMeshPathResult(t, result)
		})
	}
}
