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

// uAxis is the sketch plane's own u axis: the revolve axis every hand
// computation below is checked against.
var uAxis = decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}

// annularSketch builds a solved rectangle u∈[0,10], v∈[5,15] on the XY
// plane: revolved about the u axis it is an annular cylinder (Pappus by
// hand: A = 100, ρ̄ = 10, so a full turn's volume is 2π·10·100 = 2000π).
func annularSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// solidSketch builds a solved rectangle u∈[0,10], v∈[0,8] on the XY plane:
// one edge lies ON the revolve axis, so a full turn is a solid cylinder.
func solidSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// semicircleSketch builds a solved half-disk: the diameter on the u axis,
// the arc bulging to +v — a sphere generator.
func semicircleSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	end := s.CreatePoint(10, 0)
	c := s.CreatePoint(5, 0)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// surfaceKinds tallies a body's faces by surface kind.
func surfaceKinds(b *decad.Body) map[decad.SurfaceKind]int {
	out := map[decad.SurfaceKind]int{}
	for _, f := range b.Faces() {
		out[f.Surface().Kind()]++
	}
	return out
}

// faceByRole finds the face carrying the given provenance role.
func faceByRole(t *testing.T, b *decad.Body, role string) *decad.Face {
	t.Helper()
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if o.Role == role {
				return f
			}
		}
	}
	t.Fatalf(`no face with role %q`, role)
	return nil
}

func requireVolume(t *testing.T, b *decad.Body, want float64) {
	t.Helper()
	vol, err := b.Volume()
	require.NoError(t, err)
	got, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, want, got, 1e-9*math.Max(1, want))
}

func requireBounds(t *testing.T, b *decad.Body, minX, minY, minZ, maxX, maxY, maxZ float64) {
	t.Helper()
	bounds, err := b.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, bounds.Exactness)
	require.InDelta(t, minX, bounds.Min.X, 1e-9)
	require.InDelta(t, minY, bounds.Min.Y, 1e-9)
	require.InDelta(t, minZ, bounds.Min.Z, 1e-9)
	require.InDelta(t, maxX, bounds.Max.X, 1e-9)
	require.InDelta(t, maxY, bounds.Max.Y, 1e-9)
	require.InDelta(t, maxZ, bounds.Max.Z, 1e-9)
}

func TestRevolveFullAnnularCylinder(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	require.True(t, body.IsSolid())
	requireVolume(t, body, 2000*math.Pi)

	area, err := body.Area()
	require.NoError(t, err)
	// Outer wall 2π·15·10, inner wall 2π·5·10, two annuli 2·π·(15²−5²).
	require.True(t, area.Value.Equal(units.SquareMillimeters(800*math.Pi), 1e-9), `got %s`, area.Value)

	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5.0, c.Value.X, 1e-9)
	require.InDelta(t, 0.0, c.Value.Y, 1e-9)
	require.InDelta(t, 0.0, c.Value.Z, 1e-9)

	requireBounds(t, body, 0, -15, -15, 10, 15, 15)

	// Topology: two cylinder walls + two planar annuli, no caps and no seam
	// edges — every junction sweeps to a whole latitude circle with a seam
	// vertex.
	require.Len(t, body.Faces(), 4)
	kinds := surfaceKinds(body)
	require.Equal(t, 2, kinds[decad.KindCylinder])
	require.Equal(t, 2, kinds[decad.KindPlane])
	requireManifold(t, body)
	edges := body.Edges()
	require.Len(t, edges, 4)
	for _, e := range edges {
		_, ok := e.Curve().(decad.Circle3)
		require.True(t, ok, `a full revolution's junction edges are whole circles`)
		require.Same(t, e.Start(), e.End(), `a whole circle closes on its seam vertex`)
		require.True(t, e.IsConvex(), `every junction of a convex section is convex`)
	}

	// The planar annulus faces carry the inner circle as a hole loop.
	for _, f := range body.Faces() {
		if f.Surface().Kind() != decad.KindPlane {
			continue
		}
		loops := f.Loops()
		require.Len(t, loops, 2)
		require.True(t, loops[0].IsOuter())
		require.False(t, loops[1].IsOuter())
	}

	// The recorded step: the axis and the extent, exactly as given.
	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 1)
	step := recipe.Steps[0]
	require.Equal(t, decad.OpRevolve, step.Op)
	require.Equal(t, uAxis, step.Axis)
	require.Equal(t, decad.FullRevolution{}, step.Angular)
	require.Nil(t, step.Extent)
	require.Empty(t, step.Inputs)

	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, `the recorded recipe round-trips`)
}

func TestRevolveSolidCylinderHasNoInnerFace(t *testing.T) {
	s, p := solidSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	requireVolume(t, body, 640*math.Pi) // π·8²·10

	area, err := body.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(288*math.Pi), 1e-9), `wall 160π + two disks 128π, got %s`, area.Value)

	// The on-axis edge sweeps nothing: one cylinder wall + two disks, and
	// each disk has a single boundary loop.
	require.Len(t, body.Faces(), 3)
	kinds := surfaceKinds(body)
	require.Equal(t, 1, kinds[decad.KindCylinder])
	require.Equal(t, 2, kinds[decad.KindPlane])
	requireManifold(t, body)
	require.Len(t, body.Edges(), 2)
	for _, f := range body.Faces() {
		if f.Surface().Kind() == decad.KindPlane {
			require.Len(t, f.Loops(), 1, `a disk reaching the axis has no inner loop`)
		}
	}
	requireBounds(t, body, 0, -8, -8, 10, 8, 8)
}

func TestRevolvePartialSweeps(t *testing.T) {
	// Pappus by hand for the quarter turn: V = Δφ·∫ρ dA = (π/2)·1000.
	s, p := annularSketch(t)
	doc := decad.New()
	quarter, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	requireVolume(t, quarter, 500*math.Pi)
	require.Len(t, quarter.Faces(), 6, `four side faces and two caps`)
	requireManifold(t, quarter)
	requireBounds(t, quarter, 0, 0, 0, 10, 15, 15)

	area, err := quarter.Area()
	require.NoError(t, err)
	// A quarter of the full 800π side area, plus two 100 mm² caps.
	require.True(t, area.Value.Equal(units.SquareMillimeters(200*math.Pi+200), 1e-9), `got %s`, area.Value)

	// The solid centroid is closed form in the sweep angle: the axial part
	// from ∫zρ dA, the in-plane part from ∫ρ² dA (evaluator-design §6).
	c, err := quarter.Centroid()
	require.NoError(t, err)
	inPlane := (32500.0 / 3) / ((math.Pi / 2) * 1000)
	require.InDelta(t, 5.0, c.Value.X, 1e-9)
	require.InDelta(t, inPlane, c.Value.Y, 1e-9)
	require.InDelta(t, inPlane, c.Value.Z, 1e-9)

	// Caps face outward: the start cap against the sweep, the end cap
	// along it.
	capStart := faceByRole(t, quarter, roleCapStart)
	n, err := capStart.NormalAt(r3.NewVec(5, 10, 0))
	require.NoError(t, err)
	require.InDelta(t, 0.0, n.Value.X, 1e-9)
	require.InDelta(t, 0.0, n.Value.Y, 1e-9)
	require.InDelta(t, -1.0, n.Value.Z, 1e-9)
	capEnd := faceByRole(t, quarter, "capEnd")
	n, err = capEnd.NormalAt(r3.NewVec(5, 0, 10))
	require.NoError(t, err)
	require.InDelta(t, 0.0, n.Value.X, 1e-9)
	require.InDelta(t, -1.0, n.Value.Y, 1e-9)
	require.InDelta(t, 0.0, n.Value.Z, 1e-9)

	// Volume scales linearly with the sweep angle, and a half turn's
	// centroid sits on the symmetry plane.
	s2, p2 := annularSketch(t)
	half, err := decad.New().Revolve(s2, p2, uAxis, decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along})
	require.NoError(t, err)
	requireVolume(t, half, 1000*math.Pi)
	c, err = half.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5.0, c.Value.X, 1e-9)
	require.InDelta(t, 0.0, c.Value.Y, 1e-9)
	require.InDelta(t, 2*(32500.0/3)/(math.Pi*1000), c.Value.Z, 1e-9)
	requireManifold(t, half)
}

func TestRevolveWedgeSharesAxisEdgeBetweenCaps(t *testing.T) {
	// A solid section swept a quarter turn: the on-axis line emits no side
	// face, and its single edge is shared by the two caps.
	s, p := solidSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	requireVolume(t, body, 160*math.Pi)
	require.Len(t, body.Faces(), 5, `wall, two pie sectors, two caps`)
	requireManifold(t, body)

	capStart := faceByRole(t, body, roleCapStart)
	capEnd := faceByRole(t, body, "capEnd")
	shared := 0
	for _, e := range body.Edges() {
		faces := e.Faces()
		if len(faces) != 2 {
			continue
		}
		if (faces[0] == capStart && faces[1] == capEnd) || (faces[0] == capEnd && faces[1] == capStart) {
			_, ok := e.Curve().(decad.Line3)
			require.True(t, ok, `the caps meet along the axis line`)
			shared++
		}
	}
	require.Equal(t, 1, shared, `exactly one shared axis edge between the caps`)
}

func TestRevolveSphere(t *testing.T) {
	s, p := semicircleSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	requireVolume(t, body, 4.0/3*math.Pi*125) // (4/3)πr³, r = 5

	area, err := body.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(100*math.Pi), 1e-9), `4πr², got %s`, area.Value)

	// One spherical face and nothing else: the poles bound no edge, and
	// the on-axis diameter sweeps nothing.
	require.Len(t, body.Faces(), 1)
	sphere, ok := body.Faces()[0].Surface().(decad.Sphere)
	require.True(t, ok)
	require.InDelta(t, 5.0, sphere.Center.X, 1e-9)
	require.InDelta(t, 0.0, sphere.Center.Y, 1e-9)
	require.True(t, sphere.Radius.Equal(units.Millimeters(5), 1e-9))
	require.Empty(t, body.Edges())
	require.Empty(t, body.Faces()[0].Loops())

	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 5.0, c.Value.X, 1e-9)
	require.InDelta(t, 0.0, c.Value.Y, 1e-9)
	require.InDelta(t, 0.0, c.Value.Z, 1e-9)
	requireBounds(t, body, 0, -5, -5, 10, 5, 5)

	n, err := body.Faces()[0].NormalAt(r3.NewVec(5, 5, 0))
	require.NoError(t, err)
	require.InDelta(t, 0.0, n.Value.X, 1e-9)
	require.InDelta(t, 1.0, n.Value.Y, 1e-9)
	require.InDelta(t, 0.0, n.Value.Z, 1e-9)
	_, err = body.Faces()[0].NormalAt(r3.NewVec(5, 0, 0))
	require.ErrorIs(t, err, decad.ErrDegenerate, `the sphere center has no normal`)

	// A loop-less closed face is valid by construction. Its rounded centroid
	// has no support-point diameter for the tolerance gate, so Verify refuses
	// to call the bounded reading Sound.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.False(t, report.Trustworthy())
}

func TestRevolveTorus(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 10)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	requireVolume(t, body, 2*math.Pi*10*math.Pi*9) // Pappus: 2π·R·(πr²)
	area, err := body.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(2*math.Pi*10*2*math.Pi*3), 1e-9), `got %s`, area.Value)

	// One toroidal face bounding the whole solid: no edges, no loops.
	require.Len(t, body.Faces(), 1)
	torus, ok := body.Faces()[0].Surface().(decad.Torus)
	require.True(t, ok)
	require.True(t, torus.Major.Equal(units.Millimeters(10), 1e-9))
	require.True(t, torus.Minor.Equal(units.Millimeters(3), 1e-9))
	require.Empty(t, body.Edges())

	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 0.0, c.Value.X, 1e-9)
	require.InDelta(t, 0.0, c.Value.Y, 1e-9)
	require.InDelta(t, 0.0, c.Value.Z, 1e-9)
	requireBounds(t, body, -3, -13, -13, 3, 13, 13)

	// Outward normals on the outer and inner equators.
	f := body.Faces()[0]
	n, err := f.NormalAt(r3.NewVec(0, 13, 0))
	require.NoError(t, err)
	require.InDelta(t, 1.0, n.Value.Y, 1e-9)
	n, err = f.NormalAt(r3.NewVec(0, 7, 0))
	require.NoError(t, err)
	require.InDelta(t, -1.0, n.Value.Y, 1e-9)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)

	// A quarter of the same torus: the patch keeps both cap circles as
	// separate loops with seam vertices, mirroring the whole-circle prism
	// discipline, and each disk cap carries the circle as its boundary.
	s2, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c2 := s2.CreatePoint(0, 10)
	s2.Fix(c2)
	s2.CreateCircle(c2, 3)
	_, err = s2.Solve(t.Context())
	require.NoError(t, err)
	part, err := decad.New().Revolve(s2, s2.Profiles()[0], uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	requireVolume(t, part, math.Pi/2*10*math.Pi*9)
	require.Len(t, part.Faces(), 3)
	requireManifold(t, part)
	for _, f := range part.Faces() {
		if f.Surface().Kind() != decad.KindTorus {
			require.Len(t, f.Loops(), 1)
			continue
		}
		require.Len(t, f.Loops(), 2, `a partial torus patch is bounded by its two cap circles`)
		for _, l := range f.Loops() {
			edges := l.Edges()
			require.Len(t, edges, 1)
			require.Same(t, edges[0].Start(), edges[0].End(), `a cap circle closes on its seam vertex`)
		}
	}
}

func TestRevolveCone(t *testing.T) {
	// A right triangle: the base on the axis, the hypotenuse sweeping a
	// cone with its apex on the axis at (10, 0).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	apex := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 5)
	s.CreateLine(o, apex)
	s.CreateLine(apex, top)
	s.CreateLine(top, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	requireVolume(t, body, math.Pi*25*10/3) // (1/3)πr²h

	area, err := body.Area()
	require.NoError(t, err)
	// Lateral π·r·slant + base disk π·r².
	require.True(t, area.Value.Equal(units.SquareMillimeters(math.Pi*5*math.Sqrt(125)+math.Pi*25), 1e-9), `got %s`, area.Value)

	require.Len(t, body.Faces(), 2)
	kinds := surfaceKinds(body)
	require.Equal(t, 1, kinds[decad.KindCone])
	require.Equal(t, 1, kinds[decad.KindPlane])
	requireManifold(t, body)
	require.Len(t, body.Edges(), 1)

	var cone decad.Cone
	for _, f := range body.Faces() {
		if c, ok := f.Surface().(decad.Cone); ok {
			cone = c
		}
	}
	require.InDelta(t, 10.0, cone.Origin.X, 1e-9, `the apex is the cone origin`)
	require.True(t, cone.Radius.Equal(units.Millimeters(0), 1e-9))
	require.InDelta(t, -1.0, cone.Axis.X, 1e-9, `the radius grows toward the base`)
	require.True(t, cone.HalfAngle.Equal(units.Radians(math.Atan2(5, 10)), 1e-9))

	// The wall normal at the slant midpoint: (1, 2, 0)/√5 by hand.
	var wall *decad.Face
	for _, f := range body.Faces() {
		if f.Surface().Kind() == decad.KindCone {
			wall = f
		}
	}
	n, err := wall.NormalAt(r3.NewVec(5, 2.5, 0))
	require.NoError(t, err)
	require.InDelta(t, 1/math.Sqrt(5), n.Value.X, 1e-9)
	require.InDelta(t, 2/math.Sqrt(5), n.Value.Y, 1e-9)
	require.InDelta(t, 0.0, n.Value.Z, 1e-9)
	_, err = wall.NormalAt(r3.NewVec(10, 0, 0))
	require.ErrorIs(t, err, decad.ErrDegenerate, `the apex has no normal`)

	requireBounds(t, body, 0, -5, -5, 10, 5, 5)
}

func TestRevolveNegativeSideRegion(t *testing.T) {
	// The region lies on the NEGATIVE side of the axis: the evaluator
	// flips its internal frame, and the sweep senses must follow — Along a
	// quarter turn still rotates right-handed about the axis as given, so
	// the solid spans the −y/−z quadrant.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, -15, 10, -5)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	full, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	requireVolume(t, full, 2000*math.Pi)
	requireBounds(t, full, 0, -15, -15, 10, 15, 15)
	requireManifold(t, full)

	s2, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect2 := s2.CreateRectangle(0, -15, 10, -5)
	s2.Fix(rect2.A)
	_, err = s2.Solve(t.Context())
	require.NoError(t, err)
	quarter, err := decad.New().Revolve(s2, s2.Profiles()[0], uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	requireVolume(t, quarter, 500*math.Pi)
	requireBounds(t, quarter, 0, -15, -15, 10, 0, 0)
	requireManifold(t, quarter)
}

func TestRevolveExtents(t *testing.T) {
	revolve := func(t *testing.T, a decad.AngularExtent) (*decad.Body, error) {
		t.Helper()
		s, p := annularSketch(t)
		return decad.New().Revolve(s, p, uAxis, a)
	}

	t.Run("Against", func(t *testing.T) {
		body, err := revolve(t, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Against})
		require.NoError(t, err)
		requireVolume(t, body, 500*math.Pi)
		requireBounds(t, body, 0, 0, -15, 10, 15, 0)
	})
	t.Run("Symmetric", func(t *testing.T) {
		body, err := revolve(t, decad.SymmetricAngle{A: units.Degrees(90)})
		require.NoError(t, err)
		requireVolume(t, body, 1000*math.Pi)
		requireBounds(t, body, 0, 0, -15, 10, 15, 15)
	})
	t.Run("SymmetricFullLength", func(t *testing.T) {
		body, err := revolve(t, decad.SymmetricAngle{A: units.Degrees(90), FullLength: true})
		require.NoError(t, err)
		requireVolume(t, body, 500*math.Pi)
	})
	t.Run("TwoSided", func(t *testing.T) {
		body, err := revolve(t, decad.TwoSidedAngle{One: decad.AngleSide{A: units.Degrees(30)}, Two: decad.AngleSide{A: units.Degrees(60)}})
		require.NoError(t, err)
		requireVolume(t, body, 500*math.Pi)
		// The sweep spans φ ∈ [−60°, +30°]: the y minimum is the INNER
		// wall at the −60° cap (cos never reaches zero on the interval),
		// the z extremes the outer wall at each cap.
		requireBounds(t, body, 0, 5*math.Cos(math.Pi/3), -15*math.Sin(math.Pi/3), 10, 15, 15*math.Sin(math.Pi/6))
	})
	t.Run("FullTurnAsAngle", func(t *testing.T) {
		// 360° stated as an angle IS a full revolution: the caps would
		// coincide, so none are built.
		body, err := revolve(t, decad.AngleExtent{A: units.Degrees(360), Dir: decad.Along})
		require.NoError(t, err)
		requireVolume(t, body, 2000*math.Pi)
		require.Len(t, body.Faces(), 4, `a full turn has no caps`)
		requireManifold(t, body)
	})
	t.Run("ZeroSweep", func(t *testing.T) {
		_, err := revolve(t, decad.AngleExtent{A: units.Radians(0), Dir: decad.Along})
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = revolve(t, decad.SymmetricAngle{A: units.Radians(0)})
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = revolve(t, decad.TwoSidedAngle{One: decad.AngleSide{A: units.Radians(0)}, Two: decad.AngleSide{A: units.Radians(0)}})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("PastFullTurn", func(t *testing.T) {
		_, err := revolve(t, decad.AngleExtent{A: units.Radians(7), Dir: decad.Along})
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = revolve(t, decad.SymmetricAngle{A: units.Degrees(200)})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("NegativeMagnitude", func(t *testing.T) {
		_, err := revolve(t, decad.AngleExtent{A: units.Radians(-1), Dir: decad.Along})
		require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	})
	t.Run("WrongKind", func(t *testing.T) {
		_, err := revolve(t, decad.AngleExtent{A: units.Millimeters(90), Dir: decad.Along})
		require.ErrorIs(t, err, decad.ErrUnitKind)
	})
	t.Run("NilExtent", func(t *testing.T) {
		_, err := revolve(t, nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("NilSide", func(t *testing.T) {
		_, err := revolve(t, decad.TwoSidedAngle{One: decad.AngleSide{A: units.Degrees(30)}})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("UnknownDirection", func(t *testing.T) {
		_, err := revolve(t, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Direction(7)})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
}

func TestRevolveAxisContactRejections(t *testing.T) {
	t.Run("AxisThroughInterior", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, -5, 10, 15)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		_, err = decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate, `a region straddling the axis is rejected`)
	})
	t.Run("HornTorusTangency", func(t *testing.T) {
		// A circle kissing the axis would sweep a self-touching horn torus.
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		c := s.CreatePoint(0, 10)
		s.Fix(c)
		s.CreateCircle(c, 10)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		_, err = decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("InteriorArcTangency", func(t *testing.T) {
		// An open arc grazing the axis at an interior point — neither an
		// endpoint contact nor an on-axis line.
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		left := s.CreatePoint(0, 5)
		s.Fix(left)
		right := s.CreatePoint(10, 5)
		c := s.CreatePoint(5, 5)
		s.CreateLine(right, left)
		s.CreateArc(c, left, right) // CCW from (0,5) through (5,0) to (10,5)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		_, err = decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("EndpointContactIsAllowed", func(t *testing.T) {
		// The same arc split at its tangency point: both halves end ON the
		// axis, which is the allowed pole/apex contact.
		s, p := semicircleSketch(t)
		_, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
	})
}

func TestRevolveAxisValidation(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()

	t.Run("NilAxis", func(t *testing.T) {
		_, err := doc.Revolve(s, p, nil, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("NilAxisPointer", func(t *testing.T) {
		_, err := doc.Revolve(s, p, (*decad.SketchLine)(nil), decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("ZeroLengthSketchLine", func(t *testing.T) {
		_, err := doc.Revolve(s, p, decad.SketchLine{Start: decad.Point2{U: 1, V: 2}, End: decad.Point2{U: 1, V: 2}}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("NonFiniteSketchLine", func(t *testing.T) {
		_, err := doc.Revolve(s, p, decad.SketchLine{Start: decad.Point2{U: math.NaN()}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrNotFinite)
	})
	t.Run("ConstructionAxisWorks", func(t *testing.T) {
		s2, p2 := annularSketch(t)
		axis := decad.ConstructionAxis{Origin: r3.NewVec(0, 0, 0), Dir: r3.NewVec(1, 0, 0)}
		body, err := decad.New().Revolve(s2, p2, axis, decad.FullRevolution{})
		require.NoError(t, err)
		requireVolume(t, body, 2000*math.Pi)
	})
	t.Run("ConstructionAxisZeroDir", func(t *testing.T) {
		_, err := doc.Revolve(s, p, decad.ConstructionAxis{Origin: r3.NewVec(0, 0, 0)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("ConstructionAxisOffPlane", func(t *testing.T) {
		_, err := doc.Revolve(s, p, decad.ConstructionAxis{Origin: r3.NewVec(0, 0, 5), Dir: r3.NewVec(1, 0, 0)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate, `an origin off the plane is rejected`)
		_, err = doc.Revolve(s, p, decad.ConstructionAxis{Origin: r3.NewVec(0, 0, 0), Dir: r3.NewVec(0, 0, 1)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate, `a direction out of the plane is rejected`)
	})
	t.Run("NonFiniteConstructionAxis", func(t *testing.T) {
		_, err := doc.Revolve(s, p, decad.ConstructionAxis{Origin: r3.NewVec(math.Inf(1), 0, 0), Dir: r3.NewVec(1, 0, 0)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrNotFinite)
	})
	t.Run("NilOption", func(t *testing.T) {
		_, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("NilDocument", func(t *testing.T) {
		var nilDoc *decad.Document
		_, err := nilDoc.Revolve(s, p, uAxis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("StaleProfile", func(t *testing.T) {
		w := sketch.NewWorld()
		s2, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s2.CreateRectangle(0, 5, 10, 15)
		s2.Fix(rect.A)
		_, err = s2.Solve(t.Context())
		require.NoError(t, err)
		p2 := s2.Profiles()[0]
		s2.CreatePoint(100, 100) // the sketch moves on; the profile is stale
		_, err = decad.New().Revolve(s2, p2, uAxis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrStaleProfile)
	})
}

func TestRevolveAxisDerivedOverflow(t *testing.T) {
	t.Run("SketchLineDelta", func(t *testing.T) {
		s, p := annularSketch(t)
		doc := decad.New()
		axis := decad.SketchLine{
			Start: decad.Point2{U: -math.MaxFloat64},
			End:   decad.Point2{U: math.MaxFloat64},
		}

		_, err := doc.Revolve(s, p, axis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrNotFinite)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("SketchLineFiniteOverflowingMagnitude", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 20, 10, 30)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		p := s.Profiles()[0]
		doc := decad.New()
		axis := decad.SketchLine{
			Start: decad.Point2{},
			End:   decad.Point2{U: math.MaxFloat64, V: math.MaxFloat64},
		}

		body, err := doc.Revolve(s, p, axis, decad.FullRevolution{})
		require.NoError(t, err)
		require.True(t, body.IsSolid())
		requireVolume(t, body, 2000*math.Pi*math.Sqrt2)
		require.Len(t, doc.Bodies(), 1)
		require.Len(t, doc.Recipe().Steps, 1)
	})

	t.Run("ConstructionAxisFrameConversion", func(t *testing.T) {
		w := sketch.NewWorld()
		frame, err := r3.NewFrame(
			r3.NewVec(-math.MaxFloat64, 0, 0),
			r3.NewVec(1, 0, 0),
			r3.NewVec(0, 1, 0),
		)
		require.NoError(t, err)
		plane, err := w.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 5, 10, 15)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		p := s.Profiles()[0]
		doc := decad.New()
		axis := decad.ConstructionAxis{
			Origin: r3.NewVec(math.MaxFloat64, 0, 0),
			Dir:    r3.NewVec(1, 0, 0),
		}

		_, err = doc.Revolve(s, p, axis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrNotFinite)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})

	t.Run("ConstructionAxisLocalLength", func(t *testing.T) {
		s, p := annularSketch(t)
		doc := decad.New()
		axis := decad.ConstructionAxis{
			Origin: r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0),
			Dir:    r3.NewVec(1, 0, 0),
		}

		_, err := doc.Revolve(s, p, axis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrNotFinite)
		require.Empty(t, doc.Bodies())
		require.Empty(t, doc.Recipe().Steps)
	})
}

func TestRevolveEdgeAxisGates(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	es, ep := plateSketch(t)
	host, err := doc.Extrude(es, ep, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	t.Run("CountMissIsErrCardinality", func(t *testing.T) {
		// The implicit exactly-one of an EdgeAxis (core §12): more than one
		// match fails it, whatever the selector's own assertion says.
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: decad.Edges()}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrCardinality)
	})
	t.Run("ZeroMatchesIsErrCardinality", func(t *testing.T) {
		// ErrCardinality takes precedence at zero matches, even for a
		// selector asserting no cardinality of its own (core §12).
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: decad.Edges(decad.Circular())}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrCardinality)
		require.NotErrorIs(t, err, decad.ErrNoMatch)
	})
	t.Run("CircularEdgeIsErrDegenerate", func(t *testing.T) {
		// A non-linear edge named as a revolve axis spins about no line.
		holed := holePlateBody(t)
		ref := decad.FeatureRef{Step: holed.Origin().Step, Role: roleCapStart}
		axis := decad.EdgeAxis{Body: holed, Edge: decad.Edges(decad.Circular(), decad.CreatedBy(ref)).Exactly(1)}
		_, err := holed.Document().Revolve(s, p, axis, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("RejectionLeavesDocumentUntouched", func(t *testing.T) {
		steps := len(doc.Recipe().Steps)
		bodies := len(doc.Bodies())
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: decad.Edges()}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrCardinality)
		require.Len(t, doc.Recipe().Steps, steps)
		require.Len(t, doc.Bodies(), bodies)
	})
	t.Run("StepRefBody", func(t *testing.T) {
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: decad.StepRef(0), Edge: decad.Edges().Exactly(1)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrUnresolvedBody)
	})
	t.Run("NilBody", func(t *testing.T) {
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Edge: decad.Edges().Exactly(1)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("TypedNilEdge", func(t *testing.T) {
		var q *decad.EdgeQuery
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: q}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrUnsupported, `malformed input must not read as staged resolution`)
	})
	t.Run("NilEdge", func(t *testing.T) {
		_, err := doc.Revolve(s, p, decad.EdgeAxis{Body: host}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("ForeignBody", func(t *testing.T) {
		other := decad.New()
		_, err := other.Revolve(s, p, decad.EdgeAxis{Body: host, Edge: decad.Edges().Exactly(1)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrForeignBody)
	})
	t.Run("RetiredBody", func(t *testing.T) {
		es2, ep2 := plateSketch(t)
		doc2 := decad.New()
		old, err := doc2.Extrude(es2, ep2, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		move, err := r3.Translation(r3.NewVec(1, 0, 0))
		require.NoError(t, err)
		_, err = old.Placed(move)
		require.NoError(t, err)
		_, err = doc2.Revolve(s, p, decad.EdgeAxis{Body: old, Edge: decad.Edges().Exactly(1)}, decad.FullRevolution{})
		require.ErrorIs(t, err, decad.ErrRetiredBody)
	})
}

// trianglePrismHost extrudes a right triangle (0,0)-(100,0)-(0,60) by 10 mm
// into doc: its bottom cap holds exactly one edge parallel to x — the world
// x axis segment from (0,0,0) to (100,0,0).
func trianglePrismHost(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	s.Fix(a)
	b := s.CreatePoint(100, 0)
	c := s.CreatePoint(0, 60)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	host, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return host
}

func TestRevolveAboutEdgeAxis(t *testing.T) {
	// End to end: select the host prism's bottom x-axis edge by provenance +
	// direction, revolve the annular rectangle about it, and check the
	// closed-form volume — the axis is the world x axis, so a full turn is
	// the same annular cylinder the SketchLine axis builds (Pappus:
	// 2π·10·100 = 2000π).
	s, p := annularSketch(t)
	doc := decad.New()
	host := trianglePrismHost(t, doc)
	capStart := decad.FeatureRef{Step: host.Origin().Step, Role: roleCapStart}
	axis := decad.EdgeAxis{
		Body: host,
		Edge: decad.Edges(decad.CreatedBy(capStart), decad.ParallelTo(r3.NewVec(1, 0, 0))).Exactly(1),
	}

	body, err := doc.Revolve(s, p, axis, decad.FullRevolution{})
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireVolume(t, body, 2000*math.Pi)
	requireBounds(t, body, 0, -15, -15, 10, 15, 15)

	// The host is a dependency, not an operand: it stays live.
	require.Contains(t, doc.Bodies(), host)

	// The recorded step depends on the host's producing step, and the axis
	// records the query with the body as that StepRef (core §6.2).
	steps := doc.Recipe().Steps
	require.Len(t, steps, 2)
	step := steps[1]
	require.Equal(t, decad.OpRevolve, step.Op)
	require.Equal(t, []decad.StepRef{host.Origin().Step}, step.Inputs)
	ea, ok := step.Axis.(decad.EdgeAxis)
	require.True(t, ok, `the recorded axis stays an EdgeAxis, never the resolved line`)
	require.Equal(t, host.Origin().Step, ea.Body)

	// The recorded step round-trips through the wire codec.
	buf, err := json.Marshal(step)
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, step, got)
}

func TestRevolveEdgeAxisDirectionSense(t *testing.T) {
	// The axis runs from the resolved edge's start vertex toward its end
	// vertex, and Along is right-handed about it: a 90° Along sweep of the
	// +y-side region about the +x axis carries it toward +z, never −z.
	s, p := annularSketch(t)
	doc := decad.New()
	host := trianglePrismHost(t, doc)
	capStart := decad.FeatureRef{Step: host.Origin().Step, Role: roleCapStart}
	axis := decad.EdgeAxis{
		Body: host,
		Edge: decad.Edges(decad.CreatedBy(capStart), decad.ParallelTo(r3.NewVec(1, 0, 0))).Exactly(1),
	}

	body, err := doc.Revolve(s, p, axis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	requireVolume(t, body, 500*math.Pi)
	bounds, err := body.Bounds()
	require.NoError(t, err)
	require.GreaterOrEqual(t, bounds.Min.Z, -1e-9, `an Along sweep about start→end never reaches −z`)
	require.InDelta(t, 15, bounds.Max.Z, 1e-9)
}

func TestRevolveEdgeAxisMustBeCoplanar(t *testing.T) {
	// The resolved edge is a real axis candidate, so the coplanarity gate
	// still applies: a top-cap edge lies at z = 10, off the profile plane.
	s, p := annularSketch(t)
	doc := decad.New()
	host := trianglePrismHost(t, doc)
	capEnd := decad.FeatureRef{Step: host.Origin().Step, Role: "capEnd"}
	axis := decad.EdgeAxis{
		Body: host,
		Edge: decad.Edges(decad.CreatedBy(capEnd), decad.ParallelTo(r3.NewVec(1, 0, 0))).Exactly(1),
	}
	_, err := doc.Revolve(s, p, axis, decad.FullRevolution{})
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestRevolvePlacedRigidMotion(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	c0, err := body.Centroid()
	require.NoError(t, err)

	rot, err := r3.RotationAround(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(90))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	xf, err := rot.Then(move)
	require.NoError(t, err)

	placed, err := body.Placed(xf)
	require.NoError(t, err)
	requireVolume(t, placed, 500*math.Pi)
	require.Len(t, placed.Faces(), 6)
	requireManifold(t, placed)

	c1, err := placed.Centroid()
	require.NoError(t, err)
	want := xf.Apply(c0.Value)
	require.InDelta(t, want.X, c1.Value.X, 1e-9)
	require.InDelta(t, want.Y, c1.Value.Y, 1e-9)
	require.InDelta(t, want.Z, c1.Value.Z, 1e-9)

	area0, err := body.Area()
	require.NoError(t, err)
	area1, err := placed.Area()
	require.NoError(t, err)
	require.True(t, area1.Value.Equal(area0.Value, 1e-9), `a rigid motion preserves area`)

	require.Contains(t, doc.Bodies(), placed, `the placed body is live`)
	require.NotContains(t, doc.Bodies(), body, `the receiver is retired`)
}

func TestRevolveReflectedPlacementKeepsOutwardNormals(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	require.True(t, refl.IsReflection())

	placed, err := body.Placed(refl)
	require.NoError(t, err)
	requireVolume(t, placed, 500*math.Pi)
	requireManifold(t, placed)

	c, err := placed.Centroid()
	require.NoError(t, err)
	inPlane := (32500.0 / 3) / ((math.Pi / 2) * 1000)
	require.InDelta(t, 5.0, c.Value.X, 1e-9)
	require.InDelta(t, -inPlane, c.Value.Y, 1e-9, `the mirror flips y`)
	require.InDelta(t, inPlane, c.Value.Z, 1e-9)

	// Outwardness on every face of the reflected wedge: probes mapped
	// through the mirror, normals checked against the mapped centroid —
	// the annular quarter is star-shaped from its centroid, so every
	// outward normal points away from it.
	cos45 := math.Cos(math.Pi / 4)
	probes := map[decad.SurfaceKind][]r3.Vec{
		decad.KindCylinder: {
			refl.Apply(r3.NewVec(5, 15*cos45, 15*cos45)),
			refl.Apply(r3.NewVec(5, 5*cos45, 5*cos45)),
		},
	}
	checked := 0
	for _, f := range placed.Faces() {
		switch surf := f.Surface().(type) {
		case decad.Plane:
			n, err := f.NormalAt(surf.Frame.Origin())
			require.NoError(t, err)
			away := surf.Frame.Origin().Sub(c.Value)
			require.Positive(t, n.Value.Dot(away), `plane face normal points away from the centroid`)
			checked++
		case decad.Cylinder:
			r, err := surf.Radius.In(units.Millimeter)
			require.NoError(t, err)
			idx := 0
			if math.Abs(r-5) < 1e-6 {
				idx = 1
			}
			at := probes[decad.KindCylinder][idx]
			n, err := f.NormalAt(at)
			require.NoError(t, err)
			away := at.Sub(c.Value)
			require.Positive(t, n.Value.Dot(away), `cylinder wall normal points away from the centroid`)
			checked++
		}
	}
	require.Equal(t, 6, checked, `every face of the wedge was probed`)
}

func TestRevolveReflectedSphereAndConeNormals(t *testing.T) {
	// The curved-surface outwardness under a reflection, on the two kinds
	// the wedge test does not cover.
	s, p := semicircleSketch(t)
	doc := decad.New()
	sphere, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	placed, err := sphere.Placed(refl)
	require.NoError(t, err)
	requireVolume(t, placed, 4.0/3*math.Pi*125)
	c, err := placed.Centroid()
	require.NoError(t, err)
	require.InDelta(t, -5.0, c.Value.X, 1e-9)
	at := refl.Apply(r3.NewVec(5, 5, 0))
	n, err := placed.Faces()[0].NormalAt(at)
	require.NoError(t, err)
	require.Positive(t, n.Value.Dot(at.Sub(c.Value)), `the reflected sphere is still outward`)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
}

func TestRevolveRecipeAxisCodec(t *testing.T) {
	t.Run("AxisKeyedToRevolve", func(t *testing.T) {
		step := decad.Step{Op: decad.OpExtrude, Axis: uAxis}
		_, err := json.Marshal(step)
		require.Error(t, err, `an axis on a non-revolve step neither encodes nor decodes`)
		var s decad.Step
		err = json.Unmarshal([]byte(`{"op":"extrude","axis":{"kind":"sketch_line","start":{"u":0,"v":0},"end":{"u":1,"v":0}}}`), &s)
		require.Error(t, err)
	})
	t.Run("SketchLineRoundTrip", func(t *testing.T) {
		step := decad.Step{Op: decad.OpRevolve, Angular: decad.FullRevolution{}, Axis: uAxis}
		buf, err := json.Marshal(step)
		require.NoError(t, err)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got))
		require.Equal(t, step, got)
	})
	t.Run("ConstructionAxisRoundTrip", func(t *testing.T) {
		step := decad.Step{
			Op:      decad.OpRevolve,
			Angular: decad.SymmetricAngle{A: units.Degrees(45)},
			Axis:    decad.ConstructionAxis{Origin: r3.NewVec(1, 2, 3), Dir: r3.NewVec(0, 1, 0)},
		}
		buf, err := json.Marshal(step)
		require.NoError(t, err)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got))
		require.Equal(t, step, got)
	})
	t.Run("EdgeAxisRoundTrip", func(t *testing.T) {
		step := decad.Step{
			Op:      decad.OpRevolve,
			Angular: decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along},
			Axis:    decad.EdgeAxis{Body: decad.StepRef(2), Edge: decad.Edges(decad.Circular()).Exactly(1)},
		}
		buf, err := json.Marshal(step)
		require.NoError(t, err)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got))
		require.Equal(t, step, got)
	})
	t.Run("LiveBodyDoesNotEncode", func(t *testing.T) {
		es, ep := plateSketch(t)
		doc := decad.New()
		host, err := doc.Extrude(es, ep, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		step := decad.Step{
			Op:      decad.OpRevolve,
			Angular: decad.FullRevolution{},
			Axis:    decad.EdgeAxis{Body: host, Edge: decad.Edges().Exactly(1)},
		}
		_, err = json.Marshal(step)
		require.Error(t, err, `a live body is a handle, not a record`)
	})
	t.Run("UnknownAxisKind", func(t *testing.T) {
		var s decad.Step
		err := json.Unmarshal([]byte(`{"op":"revolve","axis":{"kind":"quaternion"}}`), &s)
		require.Error(t, err)
	})
}

// holedSketch builds the annular rectangle with a circular hole at (5, 10),
// radius 2: revolved it is a section with a toroidal void (full turn) or a
// toroidal groove through the caps (partial sweep). Q = ∫ρ dA = 1000 − 40π.
func holedSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(5, 10), 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			return s, p
		}
	}
	t.Fatal(`no holed profile`)
	return nil, nil
}

func TestRevolveFullTurnHoleIsVoidShell(t *testing.T) {
	s, p := holedSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	requireVolume(t, body, 2*math.Pi*(1000-40*math.Pi))
	require.Len(t, body.Faces(), 5, `four outer walls and the void torus`)
	requireManifold(t, body)

	// The enclosed toroidal void is its own shell, and a void one; its one
	// face is a whole torus with no boundary at all, its material OUTSIDE
	// the tube.
	shells := body.Shells()
	require.Len(t, shells, 2)
	require.False(t, shells[0].IsVoid())
	require.True(t, shells[1].IsVoid())
	require.Len(t, shells[1].Faces(), 1)
	torus, ok := shells[1].Faces()[0].Surface().(decad.Torus)
	require.True(t, ok)
	require.True(t, torus.Major.Equal(units.Millimeters(10), 1e-9))
	require.True(t, torus.Minor.Equal(units.Millimeters(2), 1e-9))
	require.Empty(t, shells[1].Faces()[0].Loops())

	// The void wall's outward normal leaves the material, so it points
	// INTO the tube: at the tube's nearest-to-axis point, toward the tube
	// center.
	n, err := shells[1].Faces()[0].NormalAt(r3.NewVec(5, 8, 0))
	require.NoError(t, err)
	require.InDelta(t, 0.0, n.Value.X, 1e-9)
	require.InDelta(t, 1.0, n.Value.Y, 1e-9)
	require.InDelta(t, 0.0, n.Value.Z, 1e-9)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Equal(t, 1, report.Bodies[0].Voids)
}

func TestRevolvePartialSweepWithHole(t *testing.T) {
	s, p := holedSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	requireVolume(t, body, math.Pi/2*(1000-40*math.Pi))
	require.Len(t, body.Faces(), 7, `four outer walls, the hole's torus wall, two caps`)
	requireManifold(t, body)
	require.Len(t, body.Shells(), 1, `the caps connect the hole wall to the outer boundary`)

	// Each cap carries the hole as a second, non-outer loop.
	for _, role := range []string{roleCapStart, "capEnd"} {
		capFace := faceByRole(t, body, role)
		loops := capFace.Loops()
		require.Len(t, loops, 2)
		require.True(t, loops[0].IsOuter())
		require.False(t, loops[1].IsOuter())
	}

	// The groove wall's outward normal at mid-sweep points toward the tube
	// center — the material is outside the tube.
	cos45 := math.Cos(math.Pi / 4)
	at := r3.NewVec(5, 8*cos45, 8*cos45)
	var wall *decad.Face
	for _, f := range body.Faces() {
		if f.Surface().Kind() == decad.KindTorus {
			wall = f
		}
	}
	require.NotNil(t, wall)
	require.Len(t, wall.Loops(), 2, `the hole's cap circles bound the torus patch`)
	n, err := wall.NormalAt(at)
	require.NoError(t, err)
	require.InDelta(t, 0.0, n.Value.X, 1e-9)
	require.InDelta(t, cos45, n.Value.Y, 1e-9)
	require.InDelta(t, cos45, n.Value.Z, 1e-9)
}

func TestRevolveVerifySound(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	_, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(120), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.False(t, report.Trustworthy())
	require.Len(t, report.Bodies, 1)
	require.True(t, report.Bodies[0].Watertight)
	require.True(t, report.Bodies[0].Manifold)
	require.Equal(t, decad.Approximate, report.Bodies[0].Exactness)
}

func TestRevolveRejectsSpindleTorusArc(t *testing.T) {
	// A boundary arc lying above the axis whose circle center sits BELOW it
	// sweeps a spindle-branch torus: a valid solid the shipped Torus
	// (non-negative Major) cannot represent, so the intent is refused as
	// staged — never a face with a negative major radius.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(15, -3)
	s.Fix(c)
	start := s.CreatePoint(20, 5)
	end := s.CreatePoint(10, 5)
	s.CreateArc(c, start, end) // CCW over the top: every arc point stays above v=0
	s.CreateLine(end, start)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.NotEmpty(t, s.Profiles())

	doc := decad.New()
	_, err = doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Recipe().Steps, `a refused revolve leaves the document untouched`)
}

func TestRevolveConcaveGrooveCapEdges(t *testing.T) {
	// A meridian rectangle u∈[0,20], v∈[5,15] with a semicircular GROOVE of
	// radius 3 centred at (10, 15) bitten out of its outer edge (the arc dips
	// to (10, 12)). The outer loop walks counter-clockwise, but that arc is
	// walked CLOCKWISE about its own centre, so the swept torus wall keeps the
	// material OUTSIDE its tube — a hole's wall in every way but its loop's
	// role. Its cap edges must be concave, exactly as a hole's would be.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{0, 5}, {20, 5}, {20, 15}, {13, 15}, {7, 15}, {0, 15}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	centre := s.CreatePoint(10, 15)
	s.Fix(centre)
	s.CreateLine(pts[0], pts[1])
	s.CreateLine(pts[1], pts[2])
	s.CreateLine(pts[2], pts[3])
	s.CreateArc(centre, pts[4], pts[3]) // CCW from (7,15) to (13,15): dips to (10,12)
	s.CreateLine(pts[4], pts[5])
	s.CreateLine(pts[5], pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.AngleExtent{A: units.Radians(math.Pi / 2), Dir: decad.Along})
	require.NoError(t, err)

	// Pappus by hand: the section is the rectangle minus the half disc, and
	// the removed disc's centroid sits 4r/3π below the groove's centre.
	half := math.Pi * 9 / 2
	moment := 2000 - half*(15-4/math.Pi) // ∫ρ dA over the section
	vol, err := body.Volume()
	require.NoError(t, err)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi/2*moment, gotVol, 1e-9)

	// The groove's wall is the body's one torus; its CAP edges are the arcs
	// lying in a cap plane, whose normal is perpendicular to the revolve axis
	// — the latitude arcs run about that axis (±X) instead.
	tori, caps := 0, 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Torus); !ok {
			continue
		}
		tori++
		for _, e := range f.Edges() {
			arc, ok := e.Curve().(decad.Arc3)
			require.True(t, ok, `a groove wall of a partial sweep is bounded by arcs`)
			if math.Abs(arc.Axis.Dot(r3.NewVec(1, 0, 0))) > 0.5 {
				continue // a latitude arc about the revolve axis, not a cap edge
			}
			caps++
			require.False(t, e.IsConvex(), `a clockwise-walked groove's cap edges are concave`)
		}
	}
	require.Equal(t, 1, tori)
	require.Equal(t, 2, caps, `one cap edge in each cap plane`)
}
