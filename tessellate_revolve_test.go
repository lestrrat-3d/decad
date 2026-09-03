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

// distanceToMesh is the minimum distance from p to the mesh's facets — the
// falsifier every "sample the true surface densely" assertion below reads. A
// sample farther from the mesh than the published Bound DISPROVES the bound;
// samples that pass prove nothing and never stand in for the proof
// (docs/tessellation-design.md §14).
func distanceToMesh(mesh *decad.Mesh, p r3.Vec) float64 {
	verts := mesh.Vertices()
	best := math.Inf(1)
	for _, tri := range mesh.Triangles() {
		best = math.Min(best, pointTriangleDistance(p, verts[tri[0]], verts[tri[1]], verts[tri[2]]))
	}
	return best
}

func pointTriangleDistance(p, a, b, c r3.Vec) float64 {
	n := b.Sub(a).Cross(c.Sub(a))
	best := math.Inf(1)
	if l2 := n.Dot(n); l2 > 0 {
		// The foot of the perpendicular, kept only when it lands inside.
		t := p.Sub(a).Dot(n) / l2
		foot := p.Sub(n.Scale(t))
		inside := true
		for _, e := range [3][2]r3.Vec{{a, b}, {b, c}, {c, a}} {
			if e[1].Sub(e[0]).Cross(foot.Sub(e[0])).Dot(n) < 0 {
				inside = false
				break
			}
		}
		if inside {
			best = math.Abs(t) * math.Sqrt(l2)
		}
	}
	for _, e := range [3][2]r3.Vec{{a, b}, {b, c}, {c, a}} {
		best = math.Min(best, pointSegmentDistance(p, e[0], e[1]))
	}
	return best
}

func pointSegmentDistance(p, s0, s1 r3.Vec) float64 {
	d := s1.Sub(s0)
	l2 := d.Dot(d)
	t := 0.0
	if l2 > 0 {
		t = math.Min(1, math.Max(0, p.Sub(s0).Dot(d)/l2))
	}
	return p.Sub(s0.Add(d.Scale(t))).Len()
}

// axisVertices lists the mesh vertices sitting exactly on the world X axis, the
// axis every fixture in this file revolves about.
func axisVertices(mesh *decad.Mesh) []int {
	var out []int
	for i, v := range mesh.Vertices() {
		if v.Y == 0 && v.Z == 0 {
			out = append(out, i)
		}
	}
	return out
}

// incidentTriangles counts the facets using a given vertex index.
func incidentTriangles(mesh *decad.Mesh, v int) int {
	n := 0
	for _, tri := range mesh.Triangles() {
		if tri[0] == v || tri[1] == v || tri[2] == v {
			n++
		}
	}
	return n
}

// requireSourceFacesLive asserts every facet names a face of the live body.
func requireSourceFacesLive(t *testing.T, b *decad.Body, mesh *decad.Mesh) {
	t.Helper()
	live := map[*decad.Face]struct{}{}
	for _, f := range b.Faces() {
		live[f] = struct{}{}
	}
	src := mesh.SourceFaces()
	require.Len(t, src, len(mesh.Triangles()))
	for _, f := range src {
		require.NotNil(t, f)
		require.Contains(t, live, f)
	}
}

func TestRevolveTessellateFullCylinder(t *testing.T) {
	s, p := solidSketch(t)
	body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	tol := units.Millimeters(0.05)
	mesh, err := body.Tessellate(tol)
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)

	// The chorded cylinder is inscribed, so its volume sits strictly below the
	// analytic one and within the chording the tolerance bought.
	vol, err := body.Volume()
	require.NoError(t, err)
	analytic, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	got := meshVolume(mesh)
	require.Positive(t, got)
	require.Less(t, got, analytic)
	require.Greater(t, got, analytic*0.98)

	bound, err := mesh.Bound().In(units.Millimeter)
	require.NoError(t, err)
	require.Positive(t, bound)
	tolMM, err := tol.In(units.Millimeter)
	require.NoError(t, err)
	require.LessOrEqual(t, bound, tolMM)

	// The falsifier: the true wall is the cylinder of radius 8 about the X
	// axis between x = 0 and x = 10, and its two disks. No sample of it may sit
	// farther from the mesh than the published bound.
	for i := range 24 {
		x := float64(i) / 23 * 10
		for j := range 37 {
			phi := float64(j) / 36 * 2 * math.Pi
			wall := r3.Vec{X: x, Y: 8 * math.Cos(phi), Z: 8 * math.Sin(phi)}
			require.LessOrEqual(t, distanceToMesh(mesh, wall), bound, `wall sample %v is farther from the mesh than Bound`, wall)
			for _, level := range []float64{0, 10} {
				rho := float64(i) / 23 * 8
				disk := r3.Vec{X: level, Y: rho * math.Cos(phi), Z: rho * math.Sin(phi)}
				require.LessOrEqual(t, distanceToMesh(mesh, disk), bound, `cap sample %v is farther from the mesh than Bound`, disk)
			}
		}
	}

	// The one interned vertex per on-axis meridian sample: the two disk
	// centres, each the hub of a full fan.
	axis := axisVertices(mesh)
	require.Len(t, axis, 2)
	for _, v := range axis {
		require.Equal(t, incidentTriangles(mesh, axis[0]), incidentTriangles(mesh, v))
		require.Greater(t, incidentTriangles(mesh, v), 2)
	}
}

func TestRevolveTessellateConeApexIsOneInternedVertex(t *testing.T) {
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
	body, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)

	// Both the cone's apex and the base disk's centre are on the axis, each a
	// SINGLE vertex fanned to by its whole ring — never a collapsed quad.
	axis := axisVertices(mesh)
	require.Len(t, axis, 2)
	nPhi := incidentTriangles(mesh, axis[0])
	require.Equal(t, nPhi, incidentTriangles(mesh, axis[1]))
	require.Len(t, mesh.Triangles(), 2*nPhi, `a two-fan body carries exactly its two fans`)

	vol, err := body.Volume()
	require.NoError(t, err)
	analytic, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, meshVolume(mesh), analytic)
	require.Greater(t, meshVolume(mesh), analytic*0.97)

	// The falsifier over the true cone: ρ falls linearly from 5 at x = 0 to 0
	// at x = 10.
	bound, err := mesh.Bound().In(units.Millimeter)
	require.NoError(t, err)
	for i := range 21 {
		x := float64(i) / 20 * 10
		rho := 5 * (1 - x/10)
		for j := range 37 {
			phi := float64(j) / 36 * 2 * math.Pi
			at := r3.Vec{X: x, Y: rho * math.Cos(phi), Z: rho * math.Sin(phi)}
			require.LessOrEqual(t, distanceToMesh(mesh, at), bound, `cone sample %v is farther from the mesh than Bound`, at)
		}
	}
}

func TestRevolveTessellatePartialAxisLineSharesOneCapEdge(t *testing.T) {
	s, p := solidSketch(t)
	body, err := decad.New().Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, body, mesh)

	// The on-axis generator sweeps no wall, and its single geometric edge is
	// shared by the two caps: exactly two axis vertices, and the facets holding
	// both of them are one start-cap and one end-cap facet.
	axis := axisVertices(mesh)
	require.Len(t, axis, 2)
	src := mesh.SourceFaces()
	var roles []string
	for i, tri := range mesh.Triangles() {
		holds := 0
		for _, v := range tri {
			if v == axis[0] || v == axis[1] {
				holds++
			}
		}
		if holds == 2 {
			roles = append(roles, src[i].Origins()[0].Role)
		}
	}
	require.ElementsMatch(t, []string{"capStart", "capEnd"}, roles)

	vol, err := body.Volume()
	require.NoError(t, err)
	analytic, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, meshVolume(mesh), analytic)
	require.Greater(t, meshVolume(mesh), analytic*0.98)
}

func TestRevolveTessellateAnnulusAndProfileHole(t *testing.T) {
	t.Run("planar annulus walls", func(t *testing.T) {
		s, p := annularSketch(t)
		body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		mesh, err := body.Tessellate(units.Millimeters(0.05))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		requireSourceFacesLive(t, body, mesh)
		require.Empty(t, axisVertices(mesh), `an annular tube touches the axis nowhere`)
		vol, err := body.Volume()
		require.NoError(t, err)
		analytic, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		// The outer wall chords inward and the bore wall chords outward, so the
		// held volume stays under the analytic one but by less than the outer
		// wall alone would lose.
		require.Less(t, meshVolume(mesh), analytic)
		require.Greater(t, meshVolume(mesh), analytic*0.99)
	})

	t.Run("profile hole sweeps a separate void shell", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		outer := s.CreateRectangle(0, 4, 30, 24)
		s.Fix(outer.A)
		s.CreateRectangle(10, 10, 20, 18)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		var prof *sketch.Profile
		for _, cand := range s.Profiles() {
			if len(cand.Holes) == 1 {
				prof = cand
			}
		}
		require.NotNil(t, prof)
		body, err := decad.New().Revolve(s, prof, uAxis, decad.FullRevolution{})
		require.NoError(t, err)

		mesh, err := body.Tessellate(units.Millimeters(0.2))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		requireSourceFacesLive(t, body, mesh)
		// Two shells: the outer tube and the toroidal void the hole sweeps.
		// The signed sum is the material volume, so it stays positive and
		// under the analytic figure.
		vol, err := body.Volume()
		require.NoError(t, err)
		analytic, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.Positive(t, meshVolume(mesh))
		require.Less(t, meshVolume(mesh), analytic)
		require.Greater(t, meshVolume(mesh), analytic*0.98)
	})
}

func TestRevolveTessellateReflectedPlacementKeepsOutwardWinding(t *testing.T) {
	s, p := solidSketch(t)
	doc := decad.New()
	body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(120), Dir: decad.Along})
	require.NoError(t, err)
	mirror, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 0, Y: 1, Z: 0}, r3.Vec{X: 0, Y: 0, Z: 1})
	require.NoError(t, err)
	xf, err := r3.Reflection(mirror)
	require.NoError(t, err)
	placed, err := body.Placed(xf)
	require.NoError(t, err)

	mesh, err := placed.Tessellate(units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	requireSourceFacesLive(t, placed, mesh)
	require.Positive(t, meshVolume(mesh), `a reflected placement must still wind outward`)
}

func TestRevolveTessellateChargesPlacementRoundingSeparately(t *testing.T) {
	// docs/tessellation-design.md §8's deltaR: an identity placement performs
	// no coordinate operation and adds nothing, while a real motion rounds at
	// its own magnitude and every source bound carries the difference.
	s, p := solidSketch(t)
	body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	tol := units.Millimeters(0.05)

	unplaced, err := body.Tessellate(tol)
	require.NoError(t, err)
	base, err := unplaced.Bound().In(units.Millimeter)
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Degrees(37))
	require.NoError(t, err)
	rotated, err := body.PlacedCopy(rot)
	require.NoError(t, err)
	rotMesh, err := rotated.Tessellate(tol)
	require.NoError(t, err)
	rotBound, err := rotMesh.Bound().In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, rotBound, base, `a rotation rounds every stored coordinate, and the bound must carry it`)

	far, err := r3.Translation(r3.Vec{X: 1e7, Y: 3e6, Z: -2e6})
	require.NoError(t, err)
	moved, err := body.PlacedCopy(far)
	require.NoError(t, err)
	farMesh, err := moved.Tessellate(tol)
	require.NoError(t, err)
	farBound, err := farMesh.Bound().In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, farBound, rotBound, `placement rounding grows with the magnitude it rounds at`)

	// The chording is unchanged by either motion — same radius, same sweep — so
	// every one of the three meshes carries the same facet count.
	require.Len(t, rotMesh.Triangles(), len(unplaced.Triangles()))
	require.Len(t, farMesh.Triangles(), len(unplaced.Triangles()))
}

func TestRevolveTessellateRefusals(t *testing.T) {
	t.Run("circular generator stays staged", func(t *testing.T) {
		s, p := semicircleSketch(t)
		body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		_, err = body.Tessellate(units.Millimeters(0.05))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("torus generator stays staged", func(t *testing.T) {
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
		_, err = body.Tessellate(units.Millimeters(0.05))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("a tolerance past the facet ceiling refuses", func(t *testing.T) {
		s, p := solidSketch(t)
		body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		_, err = body.Tessellate(units.Millimeters(1e-9))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("two off-axis generators meeting on the axis sweep no manifold pole", func(t *testing.T) {
		// The apex is on the axis but no on-axis segment continues through it,
		// so the two cones meet at a single point whose boundary link is two
		// circles rather than one — docs/tessellation-design.md §9's
		// axis-incidence refusal, raised before a sample is emitted.
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		apex := s.CreatePoint(0, 0)
		s.Fix(apex)
		a := s.CreatePoint(10, 5)
		b := s.CreatePoint(10, 10)
		s.CreateLine(apex, a)
		s.CreateLine(a, b)
		s.CreateLine(b, apex)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		body, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		_, err = body.Tessellate(units.Millimeters(0.05))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("no revolve mesh may be a boolean operand", func(t *testing.T) {
		s, p := solidSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		w := sketch.NewWorld()
		s2, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s2.CreateRectangle(2, 2, 6, 6)
		s2.Fix(rect.A)
		_, err = s2.Solve(t.Context())
		require.NoError(t, err)
		box, err := doc.Extrude(s2, s2.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
		require.NoError(t, err)
		_, err = decad.Union(body, box)
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

func TestRevolveTessellateExportIsByteIdentical(t *testing.T) {
	s, p := solidSketch(t)
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

func TestRevolveTessellateChordCountFollowsTolerance(t *testing.T) {
	s, p := solidSketch(t)
	body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	prev := 0
	for _, tol := range []float64{0.02, 0.04, 0.08, 0.16} {
		mesh, err := body.Tessellate(units.Millimeters(tol))
		require.NoError(t, err)
		n := len(mesh.Triangles())
		if prev != 0 {
			require.LessOrEqual(t, n, prev, `a coarser tolerance may never ask for more facets`)
		}
		prev = n
		bound, err := mesh.Bound().In(units.Millimeter)
		require.NoError(t, err)
		require.LessOrEqual(t, bound, tol)
	}
}
