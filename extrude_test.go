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
func plateSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
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
	s, p := plateSketch(t)
	doc := decad.New()

	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithTaper(units.Degrees(3)))
	require.ErrorIs(t, err, decad.ErrUnsupported, `a nonzero taper is staged, loudly`)
	_, err = doc.Extrude(s, p, decad.ThroughAll{Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrUnsupported, `through-all lands with the body-relative stops`)
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
			case "capStart":
				caps++
				requireNormal(f, r3.NewVec(10, 10, 0), r3.NewVec(0, 0, -1))
			case "capEnd":
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
	s, p := plateSketch(t)
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Direction(99)})
	require.ErrorIs(t, err, decad.ErrDegenerate, `an unknown direction is malformed, never silently Along`)
	require.Empty(t, doc.Recipe().Steps)
}

func TestRecipeIsAValue(t *testing.T) {
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
	s, p := plateSketch(t)
	doc := decad.New()
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along}, decad.WithTaper(units.Degrees(math.Inf(1))))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along}, decad.WithTaper(units.Degrees(math.NaN())))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	require.Empty(t, doc.Recipe().Steps)
}

func TestRecordedExtentNeverAliasesCallerPointers(t *testing.T) {
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
