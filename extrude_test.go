package decad_test

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// plateSketch builds a solved 100×60 rectangle on the XY plane.
func plateSketch(tb testing.TB) (*sketch.Sketch, *sketch.Profile) {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	return s, s.Profiles()[0]
}

// requireManifold asserts every edge of the body bounds exactly two faces.
func requireManifold(t *testing.T, b *decad.Body) {
	t.Helper()
	for _, e := range b.Edges() {
		require.Len(t, e.Faces(), 2, `every edge of a closed solid bounds exactly two faces`)
	}
}

func TestExtrudePlate(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	require.True(t, body.IsSolid())
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Value.Equal(units.CubicMillimeters(60000), 1e-9), `100×60×10 = 60000 mm³, got %s`, vol.Value)

	area, err := body.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(2*6000+320*10), 1e-9), `caps + sides, got %s`, area.Value)

	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 50.0, c.Value.X, 1e-9)
	require.InDelta(t, 30.0, c.Value.Y, 1e-9)
	require.InDelta(t, 5.0, c.Value.Z, 1e-9)

	bounds, err := body.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, bounds.Exactness)
	require.InDelta(t, 0.0, bounds.Min.X, 1e-9)
	require.InDelta(t, 0.0, bounds.Min.Y, 1e-9)
	require.InDelta(t, 0.0, bounds.Min.Z, 1e-9)
	require.InDelta(t, 100.0, bounds.Max.X, 1e-9)
	require.InDelta(t, 60.0, bounds.Max.Y, 1e-9)
	require.InDelta(t, 10.0, bounds.Max.Z, 1e-9)

	// Topology: four planar sides + two caps, all edges manifold, and the
	// side faces' provenance roles are stable.
	faces := body.Faces()
	require.Len(t, faces, 6)
	requireManifold(t, body)
	planes := 0
	for _, f := range faces {
		if _, ok := f.Surface().(decad.Plane); ok {
			planes++
		}
	}
	require.Equal(t, 6, planes, `a rectangular plate is all planar faces`)
	for _, e := range body.Edges() {
		require.True(t, e.IsConvex(), `every edge of a convex plate is convex`)
	}

	// The document holds the body, and the recipe records the step exactly.
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 1)
	require.Equal(t, decad.OpExtrude, recipe.Steps[0].Op)
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, `the recorded recipe round-trips`)
}

func TestExtrudePlateWithHole(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)

	vol, err := body.Volume()
	require.NoError(t, err)
	wantVol := (6000 - math.Pi*100) * 8
	got, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, got, 1e-9)

	area, err := body.Area()
	require.NoError(t, err)
	wantArea := 2*(6000-math.Pi*100) + 320*8 + 2*math.Pi*10*8
	gotA, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantArea, gotA, 1e-9, `the hole's inner cylinder counts toward the boundary`)

	// Topology: 4 outer sides + 1 hole cylinder + 2 caps.
	faces := body.Faces()
	require.Len(t, faces, 7)
	requireManifold(t, body)
	cylinders := 0
	for _, f := range faces {
		cyl, ok := f.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		cylinders++
		require.True(t, cyl.Radius.Equal(units.Millimeters(10), 1e-9))
		concave := 0
		for _, e := range f.Edges() {
			if !e.IsConvex() {
				concave++
			}
		}
		require.Equal(t, len(f.Edges()), concave, `a hole's edges are concave`)
	}
	require.Equal(t, 1, cylinders)
}

func TestExtrudeExtents(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)

	// Symmetric full-length 8: the sweep is [−4, 4].
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Symmetric{D: units.Millimeters(8), FullLength: true})
	require.NoError(t, err)
	bounds, err := body.Bounds()
	require.NoError(t, err)
	require.InDelta(t, -4.0, bounds.Min.Z, 1e-9)
	require.InDelta(t, 4.0, bounds.Max.Z, 1e-9)
	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 0.0, c.Value.Z, 1e-9)

	// Against: the sweep is [−10, 0].
	doc2 := decad.New()
	p2 := s.Profiles()[0]
	body2, err := doc2.Extrude(s, p2, decad.Distance{D: units.Millimeters(10), Dir: decad.Against})
	require.NoError(t, err)
	b2, err := body2.Bounds()
	require.NoError(t, err)
	require.InDelta(t, -10.0, b2.Min.Z, 1e-9)
	require.InDelta(t, 0.0, b2.Max.Z, 1e-9)
	vol2, err := body2.Volume()
	require.NoError(t, err)
	require.True(t, vol2.Value.Equal(units.CubicMillimeters(60000), 1e-9), `an Against sweep never reads negative`)

	// TwoSided distance sides: One along 7, Two against 3 → [−3, 7].
	doc3 := decad.New()
	p3 := s.Profiles()[0]
	body3, err := doc3.Extrude(s, p3, decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(7)},
		Two: decad.DistanceSide{D: units.Millimeters(3)},
	})
	require.NoError(t, err)
	b3, err := body3.Bounds()
	require.NoError(t, err)
	require.InDelta(t, -3.0, b3.Min.Z, 1e-9)
	require.InDelta(t, 7.0, b3.Max.Z, 1e-9)
}

func TestExtrudeQuarterDiskPrism(t *testing.T) {
	t.Parallel()
	// An arc-bounded profile: the side face is a true cylinder patch and
	// every measurement stays exact.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(20, 0)
	py := s.CreatePoint(0, 20)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	vol, err := body.Volume()
	require.NoError(t, err)
	got, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*400/4*5, got, 1e-9)

	bounds, err := body.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 0.0, bounds.Min.X, 1e-9)
	require.InDelta(t, 20.0, bounds.Max.X, 1e-9)
	require.InDelta(t, 20.0, bounds.Max.Y, 1e-9)
	requireManifold(t, body)

	cylinders := 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cylinders++
		}
	}
	require.Equal(t, 1, cylinders, `the arc sweeps one cylinder patch`)
}

func TestPlaced(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(90))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(200, 0, 0))
	require.NoError(t, err)
	motion, err := rot.Then(shift)
	require.NoError(t, err)

	placed, err := body.Placed(motion)
	require.NoError(t, err)

	// The motion is rigid: volume and area are untouched, the centroid maps
	// exactly, and the bounds are the rotated prism's own.
	vol, err := placed.Volume()
	require.NoError(t, err)
	require.True(t, vol.Value.Equal(units.CubicMillimeters(60000), 1e-9))
	c, err := placed.Centroid()
	require.NoError(t, err)
	want := motion.Apply(r3.NewVec(50, 30, 5))
	require.InDelta(t, want.X, c.Value.X, 1e-9)
	require.InDelta(t, want.Y, c.Value.Y, 1e-9)
	require.InDelta(t, want.Z, c.Value.Z, 1e-9)
	bounds, err := placed.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 140.0, bounds.Min.X, 1e-9, `a 90° turn maps the 60-wide side across x: [140, 200]`)
	require.InDelta(t, 200.0, bounds.Max.X, 1e-9)
	require.InDelta(t, 0.0, bounds.Min.Y, 1e-9)
	require.InDelta(t, 100.0, bounds.Max.Y, 1e-9)
	requireManifold(t, placed)

	// The receiver is retired but readable; retired bodies take no ops.
	require.Equal(t, []*decad.Body{placed}, doc.Bodies())
	origVol, err := body.Volume()
	require.NoError(t, err, `a retired body remains readable`)
	require.True(t, origVol.Value.Equal(units.CubicMillimeters(60000), 1e-9))
	_, err = body.Placed(motion)
	require.ErrorIs(t, err, decad.ErrRetiredBody)

	// The recipe holds both steps; the placed step depends on the extrude.
	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 2)
	require.Equal(t, decad.OpPlaced, recipe.Steps[1].Op)
	require.Equal(t, []decad.StepRef{0}, recipe.Steps[1].Inputs)
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got)
}

func TestExtrudeRejections(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()

	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithTaper(units.Degrees(3)))
	require.ErrorIs(t, err, decad.ErrUnsupported, `a nonzero taper is staged, loudly`)
	_, err = doc.Extrude(s, p, decad.ThroughAll{Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrDegenerate, `a through-all sweep with no live body in its path has no stop`)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(0), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(-1), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Degrees(10), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, err = doc.Extrude(s, p, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithTaper(units.Millimeters(3)))
	require.ErrorIs(t, err, decad.ErrUnitKind, `a taper must be an angle`)

	// Every rejection left the recipe and the document untouched.
	require.Empty(t, doc.Bodies())
	require.Empty(t, doc.Recipe().Steps)

	// A stale profile is sketch's answer, consumed at the gate.
	s.AddConstraint(sketch.NewDistance(s.Points()[0], s.Points()[1], 55))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrStaleProfile)
}

func TestFaceNormals(t *testing.T) {
	t.Parallel()
	// Outward normals: −z on the start cap, +z on the end cap, −y on the
	// bottom side, and INTO a hole's void on its cylinder wall.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)

	requireNormal := func(f *decad.Face, at, want r3.Vec) {
		t.Helper()
		n, err := f.NormalAt(at)
		require.NoError(t, err)
		require.Equal(t, decad.Exact, n.Exactness)
		require.InDelta(t, want.X, n.Value.X, 1e-12)
		require.InDelta(t, want.Y, n.Value.Y, 1e-12)
		require.InDelta(t, want.Z, n.Value.Z, 1e-12)
	}

	caps, cylinders := 0, 0
	for _, f := range body.Faces() {
		for _, role := range f.Origins() {
			switch role.Role {
			case roleCapStart:
				caps++
				requireNormal(f, r3.NewVec(10, 10, 0), r3.NewVec(0, 0, -1))
			case roleCapEnd:
				caps++
				requireNormal(f, r3.NewVec(10, 10, 8), r3.NewVec(0, 0, 1))
			case "side(0,0)", "side(0,1)", "side(0,2)", "side(0,3)":
				// One of the rectangle sides; check the bottom edge's face
				// by its plane normal rather than guessing indices.
				pl, ok := f.Surface().(decad.Plane)
				require.True(t, ok)
				n := pl.Frame.N()
				if math.Abs(n.Y+1) < 1e-9 {
					requireNormal(f, r3.NewVec(50, 0, 4), r3.NewVec(0, -1, 0))
				}
			case "side(1,0)":
				cylinders++
				// The hole wall: outward-from-material points INTO the hole,
				// toward the axis. At (60, 30, 4) the axis is at x=70, so
				// the outward normal is +x.
				requireNormal(f, r3.NewVec(60, 30, 4), r3.NewVec(1, 0, 0))
			}
		}
	}
	require.Equal(t, 2, caps)
	require.Equal(t, 1, cylinders)
}

func TestWholeCircleEdgeSeamVertices(t *testing.T) {
	t.Parallel()
	// A full circle's edge closes on itself: start == end, non-nil.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 7)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(3), Dir: decad.Along})
	require.NoError(t, err)
	for _, e := range body.Edges() {
		if _, ok := e.Curve().(decad.Circle3); !ok {
			continue
		}
		require.NotNil(t, e.Start(), `a full circle edge closes on a seam vertex`)
		require.Same(t, e.Start(), e.End())
	}
	requireManifold(t, body)
}

func TestExtrudeRejectsUnknownDirection(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Direction(99)})
	require.ErrorIs(t, err, decad.ErrDegenerate, `an unknown direction is malformed, never silently Along`)
	require.Empty(t, doc.Recipe().Steps)
}

func TestRecipeIsAValue(t *testing.T) {
	t.Parallel()
	// Mutating a returned recipe never changes the document's own record.
	s, p := plateSketch(t)
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	stolen := doc.Recipe()
	stolen.Steps[0].Profile.Outer.Segments[0] = decad.LineSeg{TStart: 0.5, TEnd: 0.5}
	stolen.Steps[0].Inputs = append(stolen.Steps[0].Inputs, 42)

	fresh := doc.Recipe()
	require.NotEqual(t, stolen.Steps[0].Profile.Outer.Segments[0], fresh.Steps[0].Profile.Outer.Segments[0],
		`the document's record is isolated from caller mutation`)
	require.Empty(t, fresh.Steps[0].Inputs)
}

func TestExtrudeCoalescesCollinearSides(t *testing.T) {
	t.Parallel()
	// A rectangle authored with a midpoint on its bottom edge: two collinear
	// boundary segments coalesce into ONE side face carrying both roles
	// (evaluator §3 canonicalization), so v1 counts match the analytic answer.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	s.Fix(a)
	m := s.CreatePoint(50, 0)
	b := s.CreatePoint(100, 0)
	c := s.CreatePoint(100, 60)
	d := s.CreatePoint(0, 60)
	s.CreateLine(a, m)
	s.CreateLine(m, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.Len(t, body.Faces(), 6, `the split edge merges: four sides + two caps`)
	requireManifold(t, body)
	merged := 0
	for _, f := range body.Faces() {
		if len(f.Origins()) == 2 {
			merged++
		}
	}
	require.Equal(t, 1, merged, `the merged face carries both contributing roles`)
}

func TestWholeCircleCylinderHasTwoLoops(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 7)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(3), Dir: decad.Along})
	require.NoError(t, err)
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); !ok {
			continue
		}
		require.Len(t, f.Loops(), 2, `a closed cylindrical band has one boundary loop per cap`)
	}
}

func TestArcEdgeAxisFollowsWalkSense(t *testing.T) {
	t.Parallel()
	// A hole circle is walked clockwise, so its cap edges are CCW about −z;
	// the outer quarter-disk arc is walked CCW, so its edges are about +z.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	circles := 0
	for _, e := range body.Edges() {
		c, ok := e.Curve().(decad.Circle3)
		if !ok {
			continue
		}
		circles++
		require.InDelta(t, -1.0, c.Axis.Z, 1e-12, `a clockwise hole walk is CCW about −z`)
	}
	require.Equal(t, 2, circles)
}

func TestReflectedPlacementKeepsOutwardNormals(t *testing.T) {
	t.Parallel()
	// A mirrored body's plane normals must still point out of the material,
	// and its centroid must map exactly.
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	require.True(t, refl.IsReflection())

	placed, err := body.Placed(refl)
	require.NoError(t, err)
	c, err := placed.Centroid()
	require.NoError(t, err)
	wantC := refl.Apply(r3.NewVec(50, 30, 5))
	require.InDelta(t, wantC.X, c.Value.X, 1e-9)
	require.InDelta(t, wantC.Y, c.Value.Y, 1e-9)
	require.InDelta(t, wantC.Z, c.Value.Z, 1e-9)

	// Outwardness, checked face by face: the plane's normal at a point on
	// the face points away from the body's centroid.
	for _, f := range placed.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok {
			continue
		}
		at := pl.Frame.Origin()
		n, err := f.NormalAt(at)
		require.NoError(t, err)
		away := at.Sub(c.Value)
		require.Positive(t, n.Value.Dot(away), `an outward normal points away from the centroid`)
	}
	requireManifold(t, placed)

	vol, err := placed.Volume()
	require.NoError(t, err)
	require.True(t, vol.Value.Equal(units.CubicMillimeters(60000), 1e-9), `a reflection preserves volume`)
}

func TestExtrudeRejectsNonFiniteTaper(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along}, decad.WithTaper(units.Degrees(math.Inf(1))))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along}, decad.WithTaper(units.Degrees(math.NaN())))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	require.Empty(t, doc.Recipe().Steps)
}

func TestExtrudeRejectsNilOption(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along}, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Recipe().Steps)
}

func TestRecordedExtentNeverAliasesCallerPointers(t *testing.T) {
	t.Parallel()
	// A nested pointer side is normalized to a value at the feature call, so
	// mutating the caller's struct after the fact cannot rewrite the recipe.
	s, p := plateSketch(t)
	doc := decad.New()
	side := &decad.DistanceSide{D: units.Millimeters(7)}
	_, err := doc.Extrude(s, p, decad.TwoSided{One: side, Two: decad.DistanceSide{D: units.Millimeters(3)}})
	require.NoError(t, err)

	side.D = units.Millimeters(999)
	got := doc.Recipe().Steps[0].Extent.(decad.TwoSided)
	one, ok := got.One.(decad.DistanceSide)
	require.True(t, ok, `the recorded side is a value, not the caller's pointer`)
	require.True(t, one.D.Equal(units.Millimeters(7), 1e-12), `the recorded magnitude is the one given at the call`)
}

func TestNilExtentPointersAreBranchable(t *testing.T) {
	t.Parallel()
	// A typed nil extent or side pointer follows the same branchable
	// contract as an untyped nil: errors.Is(err, ErrDegenerate).
	s, p := plateSketch(t)
	doc := decad.New()
	_, err := doc.Extrude(s, p, (*decad.Distance)(nil))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.Extrude(s, p, decad.TwoSided{One: (*decad.DistanceSide)(nil), Two: decad.DistanceSide{D: units.Millimeters(1)}})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Recipe().Steps)
}

// roundedPlateBody extrudes a 100×60 plate by 5 mm whose top edge carries a
// semicircular round of radius 10 centred at (50, 60): a BITE out of the
// boundary (the arc dips to (50, 50), walked clockwise about its centre) or a
// BUMP on it (the arc rises to (50, 70), walked counter-clockwise). Both arcs
// sit on the OUTER loop, which walks counter-clockwise either way — so the
// loop's role cannot tell the two apart and only the walk can.
func roundedPlateBody(t *testing.T, bite bool) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {60, 60}, {40, 60}, {0, 60}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	centre := s.CreatePoint(50, 60)
	s.Fix(centre)
	s.CreateLine(pts[0], pts[1])
	s.CreateLine(pts[1], pts[2])
	s.CreateLine(pts[2], pts[3])
	if bite {
		s.CreateArc(centre, pts[4], pts[3]) // CCW from (40,60) to (60,60): dips to (50,50)
	} else {
		s.CreateArc(centre, pts[3], pts[4]) // CCW from (60,60) to (40,60): rises to (50,70)
	}
	s.CreateLine(pts[4], pts[5])
	s.CreateLine(pts[5], pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestExtrudeConcaveRoundWall(t *testing.T) {
	t.Parallel()
	// A 100×60 plate whose top edge carries a semicircular BITE of radius 10
	// centred at (50, 60): a concave round on the OUTER loop. The outer loop
	// walks counter-clockwise, but that arc is walked CLOCKWISE about its own
	// centre, so the material lies OUTSIDE its cylinder — exactly as a hole's
	// does. The wall's outward normal is the radial direction negated, and a
	// rule that read the loop's role instead would point it into the metal.
	body := roundedPlateBody(t, true)

	vol, err := body.Volume()
	require.NoError(t, err)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, (6000-math.Pi*100/2)*5, gotVol, 1e-9, `the bite removes a half disc from every section`)

	// The notch wall's outward normal, checked against an independent
	// material test: the region is the rectangle MINUS the disc, so a step
	// along the outward normal must leave the solid and a step against it
	// must stay inside.
	material := func(p r3.Vec) bool {
		inRect := p.X >= 0 && p.X <= 100 && p.Y >= 0 && p.Y <= 60 && p.Z >= 0 && p.Z <= 5
		outBite := math.Hypot(p.X-50, p.Y-60) >= 10
		return inRect && outBite
	}
	cylinders := 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); !ok {
			continue
		}
		cylinders++
		p := r3.NewVec(50, 50, 2.5) // the deepest point of the bite, mid-height
		n, err := f.NormalAt(p)
		require.NoError(t, err)
		require.InDelta(t, 0.0, n.Value.X, 1e-12)
		require.InDelta(t, 1.0, n.Value.Y, 1e-12, `the outward normal points at the arc centre — the material is outside the cylinder`)
		require.InDelta(t, 0.0, n.Value.Z, 1e-12)
		require.True(t, material(p.Sub(n.Value.Scale(0.01))), `against the outward normal is metal`)
		require.False(t, material(p.Add(n.Value.Scale(0.01))), `along the outward normal leaves the solid`)
	}
	require.Equal(t, 1, cylinders)

	// The bite's rim edges follow the SAME walk as its wall: the material is
	// outside the cylinder, so the boundary turns into the metal at the rim —
	// concave, like a hole's. A fillet or chamfer query asking for convex
	// rounds must not pick this bite up.
	requireCircularRims(t, body, false)

	// The mesh is wound outward by construction, so every facet's winding
	// must agree with its source face's outward normal.
	mesh, err := body.Tessellate(units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Positive(t, meshVolume(mesh))
	verts := mesh.Vertices()
	src := mesh.SourceFaces()
	for i, tri := range mesh.Triangles() {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		facet := b.Sub(a).Cross(c.Sub(a))
		n, err := src[i].NormalAt(a.Add(b).Add(c).Scale(1.0 / 3))
		require.NoError(t, err)
		require.Positive(t, facet.Dot(n.Value), `facet %d is wound against its face's outward normal`, i)
	}
}

// requireCircularRims asserts that the body's two circular rim edges — the
// bottom and top of its one circular wall — report the given convexity, read
// both straight off Edge.IsConvex and through the Convex/Concave selector
// predicates a fillet or chamfer query would use.
func requireCircularRims(t *testing.T, body *decad.Body, convex bool) {
	t.Helper()
	rims, err := decad.Edges(decad.Circular()).SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, rims, 2, `the wall's bottom and top rim`)
	for _, e := range rims {
		require.Equal(t, convex, e.IsConvex())
	}
	matching, missing := decad.Concave(), decad.Convex()
	if convex {
		matching, missing = decad.Convex(), decad.Concave()
	}
	picked, err := decad.Edges(decad.Circular(), matching).SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, picked, 2, `the selector predicate agrees with IsConvex`)
	_, err = decad.Edges(decad.Circular(), missing).SelectEdges(body)
	require.ErrorIs(t, err, decad.ErrNoMatch, `the opposite predicate picks nothing`)
}

// TestCircularRimConvexity pins the three cases together: a circular wall's
// rim edges are concave exactly when the wall is WALKED CLOCKWISE — which the
// loop's role gets right for a hole and wrong for a round on the outer loop.
func TestCircularRimConvexity(t *testing.T) {
	t.Parallel()
	t.Run("Hole", func(t *testing.T) {
		// Walked clockwise, on a hole loop: concave (the pinned case).
		requireCircularRims(t, holePlateBody(t), false)
	})
	t.Run("ConvexOuterRound", func(t *testing.T) {
		// Walked counter-clockwise, on the outer loop: convex.
		body := roundedPlateBody(t, false)
		vol, err := body.Volume()
		require.NoError(t, err)
		got, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.InDelta(t, (6000+math.Pi*100/2)*5, got, 1e-9, `the bump adds a half disc to every section`)
		requireCircularRims(t, body, true)
	})
	t.Run("ConcaveOuterRound", func(t *testing.T) {
		// Walked clockwise, on the outer loop — a hole's wall in every way
		// but its loop's role: concave.
		requireCircularRims(t, roundedPlateBody(t, true), false)
	})
}

// TestExtrudeRescaledDistanceCarriesConversionRounding pins the third way a
// sweep level is COMPUTED rather than stated: a distance carried in a non-base
// unit reaches the evaluator rescaled into millimetres, and where that rescale
// rounds, the level and every reading built on it carry the rounding.
//
// The bound is measured per value, not charged to the unit: 1 in is exactly the
// inch factor, so it rescales without rounding and stays Exact, while 0.1 in
// and 3 in do not and do not.
func TestExtrudeRescaledDistanceCarriesConversionRounding(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	for _, tc := range []struct {
		name  string
		d     units.Value
		exact bool
	}{
		{name: "millimetres need no rescale", d: units.Millimeters(25.4), exact: true},
		{name: "one inch rescales exactly", d: units.Inches(1), exact: true},
		{name: "a tenth of an inch rounds", d: units.Inches(0.1)},
		{name: "three inches round", d: units.Inches(3)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			body, err := doc.Extrude(s, p, decad.Distance{D: tc.d, Dir: decad.Along})
			require.NoError(t, err)
			box, err := body.Bounds()
			require.NoError(t, err)
			if tc.exact {
				require.Equal(t, decad.Exact, box.Exactness)
				require.Zero(t, box.Bound.Mag())
				return
			}
			require.Equal(t, decad.Approximate, box.Exactness, `a rescaled level is not the level it denotes`)
			require.Positive(t, box.Bound.Mag())
			// The bound covers the distance from the held level to the one the
			// caller's quantity denotes.
			denoted, err := tc.d.In(units.Millimeter)
			require.NoError(t, err)
			bound, err := box.Bound.In(units.Millimeter)
			require.NoError(t, err)
			require.GreaterOrEqual(t, bound, math.Abs(box.Max.Z-denoted))
		})
	}
}

// BenchmarkExtrudePlate measures the public end-to-end cost of extruding
// plateSketch's four-line rectangle, the call shape ratLerp's endpoint fast
// path (moments.go) targets: every whole LineSeg's TStart/TEnd is exactly 0
// and 1 (seam.go carries sketch's range through unchanged), so
// exactLineMoments and lineWalkBounds each take the fast path twice per
// segment.
func BenchmarkExtrudePlate(b *testing.B) {
	s, p := plateSketch(b)
	d := decad.Distance{D: units.Millimeters(10), Dir: decad.Along}
	for b.Loop() {
		doc := decad.New()
		if _, err := doc.Extrude(s, p, d); err != nil {
			b.Fatal(err)
		}
	}
}

// This file proves end to end the pair of fixes this branch and its parent
// carry: a fit-spline chain's interior joints no longer fold a
// rounding-noise cross product (docs/spline-design.md §6.5), and a prism
// evaluation now resolves each boundary segment's walk once rather than
// eight times. Together they let a real involute gear-tooth flank, drawn as
// a CreateFitSpline profile, extrude and verify through the public API —
// this used to refuse first R19 (the convexity certificate), then R7 (the
// walk-cost budget), and now does neither.

// involuteFlankSketch builds one involute gear-tooth flank as a fit spline:
// module 1, 17 teeth, 20 degree pressure angle — the gear dialog's own
// defaults. The flank runs from the base circle to the tooth tip, sampled at
// 15 points along the involute parametrization, mirrored across +X per the
// gear spec's step 4.2, and closed back to the origin by two straight lines
// so it bounds a single region.
func involuteFlankSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	const (
		module        = 1.0
		toothNumber   = 17.0
		pressureAngle = 20 * math.Pi / 180
		steps         = 15
	)
	pitchR := module * toothNumber / 2
	baseR := pitchR * math.Cos(pressureAngle)
	tipR := (module*toothNumber + 2*module) / 2

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)

	flank := make([]*sketch.Point, 0, steps)
	for i := range steps {
		r := baseR + (tipR-baseR)*float64(i)/float64(steps-1)
		alpha := math.Acos(baseR / r)
		tt := math.Tan(alpha)
		x := baseR * (math.Cos(tt) + tt*math.Sin(tt))
		y := baseR * (math.Sin(tt) - tt*math.Cos(tt))
		p := s.CreatePoint(x, -y) // mirrored across +X, per the gear spec's step 4.2
		s.Fix(p)
		flank = append(flank, p)
	}
	_, err = s.CreateFitSpline(flank...)
	require.NoError(t, err)
	s.CreateLine(flank[len(flank)-1], o)
	s.CreateLine(o, flank[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestExtrudeInvoluteFlankBuildsAndVerifies is the end-to-end proof: the
// involute flank fixture extrudes without error (the whole point — on the
// unfixed evaluator this refused, first R19, then R7), publishes the volume,
// centroid and bounds this branch measured through this exact public path,
// and reports Sound under Verify.
//
// The comparison tolerance is 1e-9, NOT the published Bound. Every input
// coordinate above is built from expressions of the shape a*b + c
// (math.Cos(tt) + tt*math.Sin(tt)), which Go permits the compiler to fuse
// into a single rounding step on some architectures (arm64) and not others
// (amd64). That makes the sampled points themselves — and so every
// measurement derived from them — differ by roughly an ulp between CI hosts.
// The published Bound (on the order of 1e-15) is far tighter than that
// cross-host input difference and would make this test flaky on macOS CI;
// 1e-9 still pins the geometry to ten significant figures while tolerating
// the platform-dependent FMA contraction. Do not tighten this without
// re-reading this comment.
const involuteCompareTolerance = 1e-9

func TestExtrudeInvoluteFlankBuildsAndVerifies(t *testing.T) {
	t.Parallel()
	s, p := involuteFlankSketch(t)

	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err, "a fit-spline involute flank must build: the R19 and R7 refusals this branch fixes")
	require.NotNil(t, body)

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.InDelta(t, 28.386587733941017, vol.Value.Mag(), involuteCompareTolerance)
	// The published Bound moves with the host too, and for the same reason:
	// it is derived from the actual float coordinates, so FMA-contracted
	// inputs give a slightly different bound (measured 1.14e-15 on amd64 and
	// 2.11e-15 on arm64). What is worth asserting is that the bound stays
	// vanishingly small against a ~28 mm^3 volume — a relative error under
	// 1e-13 — not which of those two figures this host produces.
	require.Less(t, vol.Bound.Mag(), 1e-12,
		"the volume's proven error must stay negligible against the volume itself")

	area, err := body.Area()
	require.NoError(t, err)
	require.InDelta(t, 197.10903776598994, area.Value.Mag(), involuteCompareTolerance)

	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5.939041425788393, c.Value.X, involuteCompareTolerance)
	require.InDelta(t, -0.22965573060319747, c.Value.Y, involuteCompareTolerance)
	require.InDelta(t, 5.0, c.Value.Z, involuteCompareTolerance)
	// The centroid publishes a bound of its own, and it is asserted on the
	// same terms as the volume's: a free-form section's centroid is not an
	// exactly representable reading, so the bound is positive — a zero here
	// would claim an exactness this evaluator never proved — and it stays
	// vanishingly small against the body's own ~9.5 mm extent. The figure
	// moves with the host for the same FMA reason the value does (measured
	// 1.70e-15 on amd64), so the assertion pins the tier, not the digits.
	require.Equal(t, decad.Approximate, c.Exactness)
	require.Positive(t, c.Bound.Mag(),
		"a free-form centroid's proven error is never zero: this reading is computed, not exact")
	require.Less(t, c.Bound.Mag(), 1e-12,
		"the centroid's proven error must stay negligible against the body's own extent")

	box, err := body.Bounds()
	require.NoError(t, err)
	// Exactness is read off the box's own Bound, which is derived from the
	// same host-dependent coordinates, so assert the bound is negligible
	// rather than pinning which Exactness tier this host lands in.
	require.Less(t, box.Bound.Mag(), 1e-12,
		"the box's proven error must stay negligible against the body's own extent")
	require.InDelta(t, 0.0, box.Min.X, involuteCompareTolerance)
	require.InDelta(t, -0.6817636915234364, box.Min.Y, involuteCompareTolerance)
	require.InDelta(t, 0.0, box.Min.Z, involuteCompareTolerance)
	require.InDelta(t, 9.475505172228038, box.Max.X, involuteCompareTolerance)
	require.InDelta(t, 0.0, box.Max.Y, involuteCompareTolerance)
	require.InDelta(t, 10.0, box.Max.Z, involuteCompareTolerance)

	// The flank wall is free-form: exactly one built face carries the fit
	// spline's surface, tagged KindNURBS.
	wall := freeformWallFace(t, body)
	require.Equal(t, decad.KindNURBS, wall.Surface().Kind())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}
