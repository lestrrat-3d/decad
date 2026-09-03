package decad_test

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

func TestTessellateFacetedToleranceBoundary(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 10, 10, 2), 0, 0, -6)
	body, err := decad.Cut(plate, tool)
	require.NoError(t, err)

	held, err := body.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	bound := held.Bound()
	require.Positive(t, bound.Mag())

	requested := units.Millimeters(math.Nextafter(bound.Mag(), 0))
	mesh, err := body.Tessellate(requested)
	require.Nil(t, mesh)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, fmt.Sprintf("requested tolerance %s", requested))
	require.ErrorContains(t, err, fmt.Sprintf("minimum mesh bound %s", bound))
	require.ErrorContains(t, err, fmt.Sprintf("retry with a tolerance of at least %s", bound))
	require.ErrorContains(t, err, "to restate the held mesh")

	atBound, err := body.Tessellate(bound)
	require.NoError(t, err)
	require.Equal(t, held.Vertices(), atBound.Vertices())
	require.Equal(t, held.Triangles(), atBound.Triangles())

	above := units.Millimeters(math.Nextafter(bound.Mag(), math.Inf(1)))
	aboveBound, err := body.Tessellate(above)
	require.NoError(t, err)
	require.Equal(t, held.Vertices(), aboveBound.Vertices())
	require.Equal(t, held.Triangles(), aboveBound.Triangles())
}

func TestTessellatePayloadClasses(t *testing.T) {
	t.Parallel()
	t.Run("prism", func(t *testing.T) {
		mesh, err := holedPlateBody(t).Tessellate(units.Millimeters(1))
		require.NoError(t, err)
		require.NotEmpty(t, mesh.Triangles())
	})

	t.Run("cup", func(t *testing.T) {
		_, box := shellBox(t)
		cup, err := box.Shell(topCap(box), units.Millimeters(5))
		require.NoError(t, err)
		mesh, err := cup.Tessellate(units.Millimeters(1))
		require.NoError(t, err)
		require.NotEmpty(t, mesh.Triangles())
	})

	t.Run("faceted", func(t *testing.T) {
		doc := decad.New()
		plate := boxBody(t, doc, 0, 0, 20, 20, 8)
		tool := translated(t, diskBody(t, doc, 10, 10, 2), 0, 0, -6)
		body, err := decad.Cut(plate, tool)
		require.NoError(t, err)
		mesh, err := body.Tessellate(units.Millimeters(1))
		require.NoError(t, err)
		require.NotEmpty(t, mesh.Triangles())
	})

	t.Run("revolve", func(t *testing.T) {
		s, p := solidSketch(t)
		body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		mesh, err := body.Tessellate(units.Millimeters(1))
		require.NoError(t, err)
		require.NotEmpty(t, mesh.Triangles())
	})

	t.Run("revolve with a circular generator stays staged", func(t *testing.T) {
		doc := decad.New()
		body := ballBody(t, doc, 5)
		mesh, err := body.Tessellate(units.Millimeters(1))
		require.Nil(t, mesh)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "circular generator")
	})

	t.Run("cap-loop chamfer stays staged", func(t *testing.T) {
		_, box := capBlendBox(t)
		chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
		require.NoError(t, err)
		mesh, err := chamfered.Tessellate(units.Millimeters(1))
		require.Nil(t, mesh)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "capBlendPayload")
		require.ErrorContains(t, err, "supported payload classes are prism, revolve, cup, loft, and faceted")
	})
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

	// The published bound is the PROVEN sagitta of the chording actually
	// taken: positive, within tol, and at or above the true chord-to-arc
	// deviation. Both figures below are derived here rather than read back
	// from the implementation — the proven bound is
	// docs/tessellation-design.md Sec 3's r*theta^2/(8n^2), the sin(x)<=x
	// reduction of the true sagitta r*(1-cos(theta/2n)) beside it.
	n := 10.0
	sweep := 2 * math.Pi
	wantSag := 10 * sweep * sweep / (8 * n * n)
	trueSag := 10 * (1 - math.Cos(math.Pi/n))
	require.InDelta(t, wantSag, mesh.Bound().Mag(), 1e-12)
	require.GreaterOrEqual(t, mesh.Bound().Mag(), trueSag,
		`the published bound must enclose the deviation the chords actually take`)
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
			require.GreaterOrEqual(t, d, 10-trueSag-1e-9, `a chord midpoint deviates by exactly the true sagitta, which the published bound covers`)
		}
	}
	require.Equal(t, 20, cylFacets)
}

func TestTessellateContextMatchesCompatibilityWrapper(t *testing.T) {
	body := holedPlateBody(t)
	tol := units.Millimeters(0.5)

	want, err := body.Tessellate(tol)
	require.NoError(t, err)
	got, err := body.TessellateContext(t.Context(), tol)
	require.NoError(t, err)

	require.Equal(t, want.Vertices(), got.Vertices())
	require.Equal(t, want.Triangles(), got.Triangles())
	require.Equal(t, want.SourceFaces(), got.SourceFaces())
	require.Equal(t, want.Bound(), got.Bound())
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

func TestTessellateChordingBoundNeverExceedsTolerance(t *testing.T) {
	// A threshold tolerance that used to land one chord short: the chording
	// bound of an exact payload must never exceed what the caller asked for.
	body := holedPlateBody(t)
	for _, tol := range []float64{0.489434836999924627, 0.5, 0.1, 1e-3, 3.7e-2} {
		mesh, err := body.Tessellate(units.Millimeters(tol))
		require.NoError(t, err)
		require.LessOrEqual(t, mesh.Bound().Mag(), tol, `tol %v`, tol)
	}
}

func TestTessellateBoundIncludesComputedLevelDisplacement(t *testing.T) {
	const (
		plateHeight = 1e12
		shortBy     = 1e-3
		tol         = 1e-9
		rounding    = 2.34375e-05
	)
	s, plateProfile, pinProfile := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProfile, decad.Distance{D: units.Millimeters(plateHeight), Dir: decad.Along})
	require.NoError(t, err)
	pin, err := doc.Extrude(s, pinProfile, decad.ToFace{
		Body:   plate,
		Face:   capEndFace(plate),
		Offset: units.Millimeters(-shortBy),
	})
	require.NoError(t, err)

	mesh, err := pin.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	bound, err := mesh.Bound().In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, bound, tol)
	require.GreaterOrEqual(t, bound, rounding)
}

func TestTessellateRejectsTangentHole(t *testing.T) {
	testcases := []struct {
		Name string
		CX   float64
		CY   float64
	}{
		// Tangent toward max-u: the bridge ray itself hits the contact.
		{Name: "tangent at max-u", CX: 90, CY: 30},
		// Tangent to the top edge, away from the bridge direction: only the
		// loop-clearance gate can see it.
		{Name: "tangent away from the bridge", CX: 50, CY: 50},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			testTangentHole(t, tc.CX, tc.CY)
		})
	}
}

func testTangentHole(t *testing.T, cx, cy float64) {
	// A hole tangent to the outline pinches the cap region: the loops fail
	// the chord-clearance proof, so the tessellation refuses rather than
	// emit a pinched or cracked mesh.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(cx, cy), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	if prof == nil {
		t.Skip(`the arrangement did not produce a holed region for the tangent circle`)
	}
	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	_, err = body.Tessellate(units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestTessellateRejectsImpossiblyFineTolerance(t *testing.T) {
	// acos(1 − tol/r) rounds to zero for tiny tolerances; the stable inverse
	// must refuse the unbuildable ask rather than walk up forever.
	body := holedPlateBody(t)
	_, err := body.Tessellate(units.Millimeters(1e-20))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	// A tolerance that passes the precheck but lands past the cap after the
	// ceil/walk-up must be refused too, never returned with n over the cap.
	_, err = body.Tessellate(units.Millimeters(2.8055335832277702e-12))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestTessellateReservesSectionDisplacementFromTolerance pins
// docs/tessellation-design.md §5's reservation on an assembled prism. Each
// case is the far-placed union PR 111's audit measured: a 10×10 box unioned
// with a 6×6 box drawn at -shift and placed by +shift, whose merged section
// sits a proven displacement from the section the pair denotes
// (docs/prism-boolean-design.md §7). That displacement is part of the mesh's
// deviation, so it is reserved from the requested tolerance before chording,
// and a tolerance it exhausts refuses rather than chording against a budget it
// has not got. Both operands are swept by a stated millimetre distance, so
// these bodies carry no axial displacement and the reserved terms are their
// whole deviation — that is why every bound below stays within its tolerance.
// TestTessellateUnreservedAxialDisplacementCanExceedTolerance covers the prism
// that does carry one.
func TestTessellateReservesSectionDisplacementFromTolerance(t *testing.T) {
	for _, tc := range []struct {
		name  string
		shift float64
		delta float64
	}{
		{name: "5e13 — the displacement fits under 1 mm", shift: 5e13, delta: 0.8660254037844389},
		{name: "1e14 — the 1.73 mm reproduction", shift: 1e14, delta: 1.7320508075688783},
		{name: "3e14 — the 3.46 mm reproduction", shift: 3e14, delta: 3.4641016151377566},
		{name: "1e15 — the 13.86 mm reproduction", shift: 1e15, delta: 13.856406460551026},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			a := boxBody(t, doc, 0, 0, 10, 10, 10)
			b := placedFar(t, boxBody(t, doc, 2-tc.shift, 2, 8-tc.shift, 8, 10), tc.shift)
			got, err := decad.Union(a, b)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

			// Every reading of the merged section carries the same displacement,
			// so the body's own Box bound names the figure the mesh must pay.
			box, err := got.Bounds()
			require.NoError(t, err)
			require.InDelta(t, tc.delta, box.Bound.Base(), 1e-9)

			// 1 mm is the tolerance the audit measured its 13.86 mm bound at.
			const tol = 1.0
			mesh, err := got.Tessellate(units.Millimeters(tol))
			if tc.delta >= tol {
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.Nil(t, mesh)
				require.Contains(t, err.Error(), "section displacement")
				require.Contains(t, err.Error(), units.Millimeters(tc.delta).String())
			} else {
				require.NoError(t, err)
				requireWatertight(t, mesh)
				require.LessOrEqual(t, mesh.Bound().Mag(), tol)
				require.GreaterOrEqual(t, mesh.Bound().Mag(), tc.delta,
					"the published bound must still carry the displacement")
			}

			// A tolerance above the displacement admits the mesh, and the bound
			// it publishes stays within that tolerance.
			const wide = 20.0
			mesh, err = got.Tessellate(units.Millimeters(wide))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			require.LessOrEqual(t, mesh.Bound().Mag(), wide)
			require.GreaterOrEqual(t, mesh.Bound().Mag(), tc.delta)
		})
	}
}

// TestTessellateUndisplacedPrismSpendsTheWholeTolerance is the other half of
// the reservation: a prism a caller drew carries no section displacement, so
// nothing is withheld from its chord budget and it chords exactly as it always
// has — the same count, the same bound, all of it within the tolerance asked
// for.
func TestTessellateUndisplacedPrismSpendsTheWholeTolerance(t *testing.T) {
	body := holedPlateBody(t)
	tol := 0.5
	mesh, err := body.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)

	// r = 10 at tol = 0.5 still buys 10 chords around the hole: a reservation
	// taken from an undisplaced prism would buy fewer. The comparison is
	// against docs/tessellation-design.md Sec 3's published sagitta
	// r*theta^2/(8n^2), derived here, and is checked to enclose the true
	// chord-to-arc deviation — see TestTessellatePlateWithHole.
	require.Len(t, mesh.Vertices(), 28)
	require.InDelta(t, 10*(2*math.Pi)*(2*math.Pi)/(8*10*10), mesh.Bound().Mag(), 1e-12)
	require.GreaterOrEqual(t, mesh.Bound().Mag(), 10*(1-math.Cos(math.Pi/10)),
		`the published bound must enclose the deviation the chords actually take`)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	// A straight-only prism chords exactly, bound and all.
	doc := decad.New()
	plain := boxBody(t, doc, 0, 0, 10, 10, 10)
	flat, err := plain.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, flat)
	require.Zero(t, flat.Bound().Mag())
}

// inchPrismBody extrudes an axis-aligned rectangle by a distance stated in
// inches. The millimetre level that distance denotes is one the conversion
// COMPUTED, so the swept end carries that conversion's rounding as its own
// axial displacement (docs/evaluator-design.md §5).
func inchPrismBody(t *testing.T, doc *decad.Document, x0, y0, x1, y1, inches float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Inches(inches), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestTessellateUnreservedAxialDisplacementCanExceedTolerance pins the other
// side of docs/tessellation-design.md §1's Tolerance row. The reservation of §5
// covers chording and the section displacement and nothing else, so a prism
// whose sweep level a non-base-unit conversion computed still publishes a Bound
// above the tolerance the caller asked for. Every section here is straight, so
// the chording takes no sagitta at all and the whole published bound is
// displacement — which is why widening the tolerance does not change it.
// TestTessellateBoundIncludesComputedLevelDisplacement reads the same excess on
// a level a ToFace stop computed; the second case below is the one no other
// test reaches, carrying both displacements at once, one reserved and one not.
func TestTessellateUnreservedAxialDisplacementCanExceedTolerance(t *testing.T) {
	t.Run("axial displacement with no section displacement beside it", func(t *testing.T) {
		body := inchPrismBody(t, decad.New(), 0, 0, 10, 10, 0.1)

		// 0.1 in is 2.54 mm, which binary cannot hold exactly.
		const tol = 1e-18
		mesh, err := body.Tessellate(units.Millimeters(tol))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.InEpsilon(t, 3.6637359812630174e-17, mesh.Bound().Mag(), 1e-12)
		require.Greater(t, mesh.Bound().Mag(), tol,
			"an unreserved axial displacement lifts Bound above the requested tolerance")
	})

	t.Run("axial displacement beside a section displacement the reservation pays for", func(t *testing.T) {
		const shift = 1e3
		doc := decad.New()
		a := inchPrismBody(t, doc, 0, 0, 10, 10, 1e6)
		b := placedFar(t, inchPrismBody(t, doc, 2-shift, 2, 8-shift, 8, 1e6), shift)
		got, err := decad.Union(a, b)
		require.NoError(t, err)
		require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

		// The tolerance clears the merged section's displacement, so the mesh
		// builds instead of refusing, and it is the axial term riding on top
		// unreserved that carries the published bound past what was asked for.
		const tol = 1e-10
		mesh, err := got.Tessellate(units.Millimeters(tol))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.InEpsilon(t, 1.4336877997816838e-09, mesh.Bound().Mag(), 1e-12)
		require.Greater(t, mesh.Bound().Mag(), tol)

		// The same bound at a tolerance 10000× wider: it is displacement, not
		// chording, and no budget buys it down.
		wide, err := got.Tessellate(units.Millimeters(1e-6))
		require.NoError(t, err)
		require.Equal(t, mesh.Bound().Mag(), wide.Bound().Mag())
	})
}
