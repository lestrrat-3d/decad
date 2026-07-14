package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireWatertight asserts the mesh is a closed, consistently oriented
// 2-manifold: every directed edge appears exactly once, and its reverse
// appears exactly once — each undirected edge is shared by exactly two
// facets, wound in opposite directions.
func requireWatertight(t *testing.T, mesh *decad.Mesh) {
	t.Helper()
	verts := mesh.Vertices()
	directed := map[[2]int]int{}
	for _, tri := range mesh.Triangles() {
		require.True(t, tri[0] != tri[1] && tri[1] != tri[2] && tri[2] != tri[0], `a facet uses three distinct vertices`)
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			require.GreaterOrEqual(t, a, 0)
			require.Less(t, a, len(verts))
			directed[[2]int{a, b}]++
		}
	}
	for e, n := range directed {
		require.Equal(t, 1, n, `directed edge %v used %d times`, e, n)
		require.Equal(t, 1, directed[[2]int{e[1], e[0]}], `directed edge %v has no opposing use`, e)
	}
}

// meshVolume integrates the signed volume enclosed by the mesh — positive
// exactly when the facets are wound outward.
func meshVolume(mesh *decad.Mesh) float64 {
	verts := mesh.Vertices()
	total := 0.0
	for _, tri := range mesh.Triangles() {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		total += a.Dot(b.Cross(c)) / 6
	}
	return total
}

// holedPlateBody extrudes the 100×60 plate with a 10 mm-radius hole at
// (70, 30) to an 8 mm prism.
func holedPlateBody(t *testing.T) *decad.Body {
	t.Helper()
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
	return body
}

func TestTessellatePlate(t *testing.T) {
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)

	// The classic box: 8 welded corner vertices, 12 triangles, and an all-
	// planar boundary triangulates exactly — the proven bound is zero.
	require.Len(t, mesh.Vertices(), 8)
	require.Len(t, mesh.Triangles(), 12)
	require.True(t, mesh.Bound().Equal(units.Millimeters(0), 1e-12), `a box chords nothing, got %s`, mesh.Bound())
	requireWatertight(t, mesh)
	require.InDelta(t, 60000.0, meshVolume(mesh), 1e-9, `outward-consistent winding integrates the exact volume`)

	// Every facet remembers its source face, and all six faces are covered.
	src := mesh.SourceFaces()
	require.Len(t, src, 12)
	covered := map[*decad.Face]struct{}{}
	live := map[*decad.Face]struct{}{}
	for _, f := range body.Faces() {
		live[f] = struct{}{}
	}
	for _, f := range src {
		_, ok := live[f]
		require.True(t, ok, `a facet's source is one of the body's own faces`)
		covered[f] = struct{}{}
	}
	require.Len(t, covered, 6)
}

func TestTessellatePlateWithHole(t *testing.T) {
	body := holedPlateBody(t)
	tol := 0.5
	mesh, err := body.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)

	// r = 10, tol = 0.5 → 10 chords around the hole: 4 outer + 10 hole
	// boundary samples, top and bottom. Walls: 8 outer + 20 hole facets;
	// caps: the 16-vertex bridged polygon yields 14 facets each.
	require.Len(t, mesh.Vertices(), 28)
	require.Len(t, mesh.Triangles(), 56)

	// The proven bound is the sagitta actually taken: positive, within tol.
	n := 10.0
	wantSag := 10 * (1 - math.Cos(math.Pi/n))
	require.InDelta(t, wantSag, mesh.Bound().Mag(), 1e-12)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	// The hole polygon is inscribed in the true circle, so the mesh volume
	// is exactly the prism over (rectangle − inscribed n-gon).
	wantVol := 8 * (6000 - n/2*100*math.Sin(2*math.Pi/n))
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-9)

	// Cap and cylinder wall share the same chording of the hole edge —
	// watertightness already proves it — and every chord honors the bound:
	// hole-wall vertices sit exactly on the circle, chord midpoints no
	// deeper than the sagitta.
	var cyl *decad.Face
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cyl = f
		}
	}
	require.NotNil(t, cyl)
	verts := mesh.Vertices()
	src := mesh.SourceFaces()
	cylFacets := 0
	for i, tri := range mesh.Triangles() {
		if src[i] != cyl {
			continue
		}
		cylFacets++
		for k := range 3 {
			a, b := verts[tri[k]], verts[tri[(k+1)%3]]
			require.InDelta(t, 10.0, math.Hypot(a.X-70, a.Y-30), 1e-9, `hole samples lie exactly on the circle`)
			if a.Z != b.Z {
				continue
			}
			mid := a.Add(b).Scale(0.5)
			d := math.Hypot(mid.X-70, mid.Y-30)
			require.GreaterOrEqual(t, d, 10-wantSag-1e-9, `a chord midpoint deviates by at most the proven sagitta`)
		}
	}
	require.Equal(t, 20, cylFacets)
}

func TestTessellateNonConvexOutline(t *testing.T) {
	// An L-shaped plate: the cap triangulation must respect the reflex
	// corner — a convex fan would spill outside the region.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := [][2]float64{{0, 0}, {40, 0}, {40, 20}, {20, 20}, {20, 40}, {0, 40}}
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	s.Fix(sp[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, mesh.Vertices(), 12)
	require.Len(t, mesh.Triangles(), 20, `12 wall + 2×4 cap facets`)
	requireWatertight(t, mesh)
	require.InDelta(t, 1200*5, meshVolume(mesh), 1e-9)
}

func TestTessellateQuarterDisk(t *testing.T) {
	// An arc-bounded profile: the wall and both caps must chord the arc at
	// the same samples.
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

	tol := 0.1
	mesh, err := body.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)
	require.Positive(t, mesh.Bound().Mag())

	// The boundary polyline is the two leg junctions plus the arc samples;
	// the enclosed volume is exactly the prism over the inscribed fan.
	n := float64(len(mesh.Vertices())/2 - 2)
	wantVol := 5 * (n / 2 * 400 * math.Sin(math.Pi/2/n))
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-9)
	require.InDelta(t, 5*math.Pi*100, meshVolume(mesh), 5*math.Pi*100*0.02, `the chorded volume converges on the true quarter cylinder`)
}

func TestTessellatePlacedReflected(t *testing.T) {
	// A reflected placement flips handedness; the mesh must come out wound
	// outward all the same.
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	placed, err := body.Placed(refl)
	require.NoError(t, err)

	mesh, err := placed.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, mesh.Triangles(), 12)
	requireWatertight(t, mesh)
	require.InDelta(t, 60000.0, meshVolume(mesh), 1e-9, `outward winding survives a reflected placement`)
}

func TestTessellateToleranceValidation(t *testing.T) {
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	testcases := []struct {
		Name string
		Tol  units.Value
		Err  error
	}{
		{Name: "angle tolerance", Tol: units.Degrees(1), Err: decad.ErrUnitKind},
		{Name: "NaN tolerance", Tol: units.Millimeters(math.NaN()), Err: decad.ErrNotFinite},
		{Name: "infinite tolerance", Tol: units.Millimeters(math.Inf(1)), Err: decad.ErrNotFinite},
		{Name: "negative tolerance", Tol: units.Millimeters(-0.1), Err: decad.ErrNegativeMagnitude},
		{Name: "zero tolerance", Tol: units.Millimeters(0), Err: decad.ErrDegenerate},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			_, err := body.Tessellate(tc.Tol)
			require.ErrorIs(t, err, tc.Err)
		})
	}
}
