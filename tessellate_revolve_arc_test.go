package decad_test

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The docs/tessellation-design.md §14 fixtures a CIRCULAR revolve generator
// owns (increment T3, docs/tessellation-reach-design.md §6 R4): a full sphere,
// a spherical band, a ring torus, a concave torus wall, poles at one end and at
// both, and a hole in a narrow outer-loop bulge below and above the summed
// sagitta tubes.
//
// Every dense sample of a true surface below is a FALSIFIER and nothing else: a
// sample farther from the mesh than the published Bound disproves the bound,
// and samples that pass prove nothing.

// revolveMeshBound reads the mesh's published bound in millimetres.
func revolveMeshBound(t *testing.T, mesh *decad.Mesh) float64 {
	t.Helper()
	b, err := mesh.Bound().In(units.Millimeter)
	require.NoError(t, err)
	return b
}

// analyticVolume reads a body's own analytic volume in cubic millimetres.
func analyticVolume(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	vol, err := b.Volume()
	require.NoError(t, err)
	v, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	return v
}

func TestRevolveTessellateFullSphere(t *testing.T) {
	// semicircleSketch is the half disk of radius 5 about (5, 0): a full turn
	// sweeps the sphere of radius 5 centred at (5, 0, 0).
	s, p := semicircleSketch(t)
	body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)

	analytic := analyticVolume(t, body)
	require.InDelta(t, 4.0/3*math.Pi*125, analytic, 1e-6)
	got := meshVolume(mesh)
	require.Less(t, got, analytic, `a chorded sphere is inscribed in the one it stands for`)
	require.Greater(t, got, analytic*0.97)

	// Both ends of the circular generator sit on the axis, so the mesh carries
	// exactly two interned pole vertices, each the hub of a whole fan.
	axis := axisVertices(mesh)
	require.Len(t, axis, 2)
	for _, v := range axis {
		require.Equal(t, incidentTriangles(mesh, axis[0]), incidentTriangles(mesh, v))
		require.Greater(t, incidentTriangles(mesh, v), 2)
	}

	bound := revolveMeshBound(t, mesh)
	require.Positive(t, bound)
	require.LessOrEqual(t, bound, 0.05)
	for i := range 25 {
		theta := float64(i) / 24 * math.Pi
		for j := range 37 {
			phi := float64(j) / 36 * 2 * math.Pi
			rho := 5 * math.Sin(theta)
			at := r3.Vec{X: 5 + 5*math.Cos(theta), Y: rho * math.Cos(phi), Z: rho * math.Sin(phi)}
			require.LessOrEqual(t, distanceToMesh(mesh, at), bound, `sphere sample %v is farther from the mesh than Bound`, at)
		}
	}
}

func TestRevolveTessellateSphericalBand(t *testing.T) {
	// The zone of the radius-5 sphere about the origin between u = -3 and
	// u = 3, closed by two planar disks: a circular generator whose two ends
	// are both OFF the axis, so every cell is a quad and no pole exists.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	lo := s.CreatePoint(-3, 4)
	hi := s.CreatePoint(3, 4)
	loAxis := s.CreatePoint(-3, 0)
	hiAxis := s.CreatePoint(3, 0)
	s.CreateArc(o, hi, lo)
	s.CreateLine(lo, loAxis)
	s.CreateLine(loAxis, hiAxis)
	s.CreateLine(hiAxis, hi)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)

	// The spherical zone's own volume, π∫(25 − u²)du over [-3, 3] = 132π.
	analytic := analyticVolume(t, body)
	require.InDelta(t, 132*math.Pi, analytic, 1e-6)
	require.Less(t, meshVolume(mesh), analytic)
	require.Greater(t, meshVolume(mesh), analytic*0.97)

	// Two axis vertices, the two disks' centres, and both are line junctions
	// rather than arc poles.
	require.Len(t, axisVertices(mesh), 2)

	bound := revolveMeshBound(t, mesh)
	for i := range 21 {
		u := -3 + 6*float64(i)/20
		rho := math.Sqrt(25 - u*u)
		for j := range 25 {
			phi := float64(j) / 24 * 2 * math.Pi
			at := r3.Vec{X: u, Y: rho * math.Cos(phi), Z: rho * math.Sin(phi)}
			require.LessOrEqual(t, distanceToMesh(mesh, at), bound, `band sample %v is farther from the mesh than Bound`, at)
		}
	}
}

func TestRevolveTessellateRingTorus(t *testing.T) {
	// A whole closed generator: the circle of radius 3 about (0, 10), which
	// resolves as ONE closed circular walk and therefore has no junction at
	// all — every meridian sample is a chord station.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 10)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.2))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)
	require.Empty(t, axisVertices(mesh), `a ring torus touches the axis nowhere`)

	// Pappus: V = 2π²Rr² = 2π²·10·9.
	analytic := analyticVolume(t, body)
	require.InDelta(t, 2*math.Pi*math.Pi*10*9, analytic, 1e-6)
	require.Less(t, meshVolume(mesh), analytic)
	require.Greater(t, meshVolume(mesh), analytic*0.94)

	bound := revolveMeshBound(t, mesh)
	require.Positive(t, bound)
	require.LessOrEqual(t, bound, 0.2)
	for i := range 25 {
		tt := float64(i) / 24 * 2 * math.Pi
		rho := 10 + 3*math.Sin(tt)
		for j := range 33 {
			phi := float64(j) / 32 * 2 * math.Pi
			at := r3.Vec{X: 3 * math.Cos(tt), Y: rho * math.Cos(phi), Z: rho * math.Sin(phi)}
			require.LessOrEqual(t, distanceToMesh(mesh, at), bound, `torus sample %v is farther from the mesh than Bound`, at)
		}
	}
}

func TestRevolveTessellateConcaveTorusWall(t *testing.T) {
	// An arc bulging TOWARD the axis: the swept wall is concave, the case
	// docs/tessellation-design.md §11 names when it says a torus's inner and
	// outer walls move in opposite material senses.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(5, 11.125)
	s.Fix(c)
	left := s.CreatePoint(0, 10)
	right := s.CreatePoint(10, 10)
	topRight := s.CreatePoint(10, 20)
	topLeft := s.CreatePoint(0, 20)
	s.CreateArc(c, left, right)
	s.CreateLine(right, topRight)
	s.CreateLine(topRight, topLeft)
	s.CreateLine(topLeft, left)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)
	require.Empty(t, axisVertices(mesh))
	require.Positive(t, meshVolume(mesh))

	// The concave wall dips to ρ = 6 at u = 5, so the body holds strictly more
	// than the rectangle above it alone would.
	analytic := analyticVolume(t, body)
	require.Greater(t, analytic, 2*math.Pi*15*100)
	require.InDelta(t, analytic, meshVolume(mesh), analytic*0.05)

	// The falsifier over the true concave wall itself.
	bound := revolveMeshBound(t, mesh)
	for i := range 21 {
		u := 10 * float64(i) / 20
		rho := 11.125 - math.Sqrt(5.125*5.125-(u-5)*(u-5))
		for j := range 25 {
			phi := float64(j) / 24 * 2 * math.Pi
			at := r3.Vec{X: u, Y: rho * math.Cos(phi), Z: rho * math.Sin(phi)}
			require.LessOrEqual(t, distanceToMesh(mesh, at), bound, `concave wall sample %v is farther from the mesh than Bound`, at)
		}
	}
}

func TestRevolveTessellateHemispherePoleAtOneEnd(t *testing.T) {
	// A quarter disk: the axis line from (0,0) to (5,0), the arc up to (0,5),
	// and the planar disk back down. The arc's own end at (5,0) is a pole a
	// CIRCULAR generator makes, which is what separates this from R3's cone.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	apex := s.CreatePoint(5, 0)
	top := s.CreatePoint(0, 5)
	s.CreateLine(o, apex)
	s.CreateArc(o, apex, top)
	s.CreateLine(top, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)

	analytic := analyticVolume(t, body)
	require.InDelta(t, 2.0/3*math.Pi*125, analytic, 1e-6)
	require.Less(t, meshVolume(mesh), analytic)
	require.Greater(t, meshVolume(mesh), analytic*0.97)

	// Two axis vertices: the arc's pole at u = 5 and the flat disk's centre at
	// u = 0. Both are single interned vertices fanned to by a whole ring.
	axis := axisVertices(mesh)
	require.Len(t, axis, 2)
	for _, v := range axis {
		require.Greater(t, incidentTriangles(mesh, v), 2)
	}
}

func TestRevolveTessellateNarrowBulgeHoleClearance(t *testing.T) {
	// docs/tessellation-design.md §14: a hole inside an outer loop, with the
	// clearance set below and above the summed sagitta tubes. Above them the
	// mesh is built; below them no bounded refinement can prove the section
	// simple, and the call refuses before any mesh or proof is returned.
	//
	// The hole's own station 0 sits at its rightmost point, so the outer loop's
	// RIGHT edge is the feature the chorded polyline approaches at a station
	// rather than between two: the measured distance is the true one, and the
	// gate is the hole's whole sagitta tube.
	sketchOf := func(t *testing.T, radius float64) (*sketch.Sketch, *sketch.Profile) {
		t.Helper()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 1, 30, 40)
		s.Fix(rect.A)
		s.CreateCircle(s.CreatePoint(15, 20), radius)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, cand := range s.Profiles() {
			if len(cand.Holes) == 1 {
				return s, cand
			}
		}
		require.Fail(t, "the sketch produced no holed profile")
		return nil, nil
	}
	t.Run("clearance above the summed tubes meshes", func(t *testing.T) {
		s, prof := sketchOf(t, 12)
		body, err := decad.New().Revolve(s, prof, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		mesh, err := body.Tessellate(units.Millimeters(0.5))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.Positive(t, meshVolume(mesh))
	})

	t.Run("clearance below the summed tubes refuses", func(t *testing.T) {
		s, prof := sketchOf(t, 14.999)
		body, err := decad.New().Revolve(s, prof, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		mesh, err := body.Tessellate(units.Millimeters(0.5))
		require.Nil(t, mesh, `no mesh is returned when the section proof cannot be met`)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "clearance gate")
	})

	t.Run("a partial sweep runs the same section proof", func(t *testing.T) {
		s, prof := sketchOf(t, 14.999)
		body, err := decad.New().Revolve(s, prof, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)
		mesh, err := body.Tessellate(units.Millimeters(0.5))
		require.Nil(t, mesh)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
}

func TestRevolveTessellateTangentDiskIsRefusedBeforeTessellation(t *testing.T) {
	// docs/tessellation-design.md §14: a disk tangent to the axis encoded as
	// TWO semicircular arcs sharing the tangent endpoint. Splitting the circle
	// does not turn that shared point into an admissible pole, and Revolve
	// itself refuses the two-sector horn before any mesh is asked for.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 5)
	s.Fix(c)
	bottom := s.CreatePoint(0, 0)
	top := s.CreatePoint(0, 10)
	s.CreateArc(c, bottom, top)
	s.CreateArc(c, top, bottom)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.NotEmpty(t, s.Profiles())
	body, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	_, err = body.Tessellate(units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "two off-axis segments")
}

func TestRevolveTessellateCircularGeneratorExportIsByteIdentical(t *testing.T) {
	s, p := semicircleSketch(t)
	body, err := decad.New().Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(150), Dir: decad.Along})
	require.NoError(t, err)

	var stl1, stl2, obj1, obj2 bytes.Buffer
	require.NoError(t, body.STL(&stl1, decad.WithChordTolerance(units.Millimeters(0.1))))
	require.NoError(t, body.STL(&stl2, decad.WithChordTolerance(units.Millimeters(0.1))))
	require.Equal(t, stl1.String(), stl2.String())
	require.NotEmpty(t, stl1.String())
	require.NoError(t, body.OBJ(&obj1, decad.WithChordTolerance(units.Millimeters(0.1))))
	require.NoError(t, body.OBJ(&obj2, decad.WithChordTolerance(units.Millimeters(0.1))))
	require.Equal(t, obj1.String(), obj2.String())
	require.NotEmpty(t, obj1.String())
}

func TestRevolveTessellateCircularMeridianChordsWithTolerance(t *testing.T) {
	// A coarser tolerance may never ask for more facets, and the published
	// bound must stay inside every tolerance that produced a mesh: the
	// meridian and the angular sequence are both chorded against it.
	s, p := semicircleSketch(t)
	body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	prev := 0
	for _, tol := range []float64{0.02, 0.05, 0.1, 0.25} {
		mesh, err := body.Tessellate(units.Millimeters(tol))
		require.NoError(t, err)
		n := len(mesh.Triangles())
		if prev != 0 {
			require.LessOrEqual(t, n, prev, `a coarser tolerance may never ask for more facets`)
		}
		prev = n
		require.LessOrEqual(t, revolveMeshBound(t, mesh), tol)
	}
}

func TestRevolveTessellatePartialGrooveMeshes(t *testing.T) {
	// The capability the edge-pair separating plane buys
	// (revolveVertexIsolated): a PARTIAL sweep whose meridian carries a chorded
	// arc. Its cap fan triangles meet the wall triangle of the next meridian
	// chord at exactly one vertex, and no plane built from one triangle's own
	// normal decides that pair — every rotation of it reads the wall's two
	// corners with opposite signs, and refining the angular count shrinks the
	// offending component without ever flipping it. Before that family existed
	// this body refused at every tolerance.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 4)
	s.Fix(a)
	b := s.CreatePoint(20, 4)
	c := s.CreatePoint(20, 14)
	right := s.CreatePoint(13, 14)
	left := s.CreatePoint(7, 14)
	d := s.CreatePoint(0, 14)
	centre := s.CreatePoint(10, 14)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, right)
	s.CreateArc(centre, left, right) // dips inward to ρ = 11: a concave torus wall
	s.CreateLine(left, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	body, err := decad.New().Revolve(s, s.Profiles()[0], uAxis,
		decad.AngleExtent{A: units.Degrees(270), Dir: decad.Along})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.2))
	require.NoError(t, err, `a partial sweep carrying a chorded arc must mesh`)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)
	require.NotEmpty(t, mesh.Triangles())

	analytic := analyticVolume(t, body)
	got := meshVolume(mesh)
	require.Positive(t, got)
	require.Less(t, got, analytic)
	require.Greater(t, got, analytic*0.9)

	// The two partial caps are the faces the fan triangles belong to, and both
	// are present: the pair the edge-pair family decides is a cap facet against
	// a wall facet, so a mesh missing either side would not exercise it.
	roles := map[string]int{}
	for _, f := range mesh.SourceFaces() {
		roles[f.Origins()[0].Role]++
	}
	require.Positive(t, roles[roleCapStart])
	require.Positive(t, roles[roleCapEnd])
}
