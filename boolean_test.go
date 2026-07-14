package decad_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The prism these tests build their operands from is boxBody
// (clearance_test.go): the same x0..x1 × y0..y1 rectangle extruded to a
// height-h prism on a fresh sketch of the given document, shared across the
// package's tests rather than restated here.

// diskBody extrudes a radius-r circle at (cx, cy) to a 20 mm prism — tall
// enough to pierce every plate in these tests.
func diskBody(t *testing.T, doc *decad.Document, cx, cy, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(cx, cy)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// wedgePrism extrudes the triangle through the three plane-local points to a
// prism symmetric about its sketch plane: the sweep of the first point is a
// knife edge, and the other two share the corner's opposite side.
func wedgePrism(t *testing.T, doc *decad.Document, w *sketch.World, plane *sketch.Plane, pts [3][2]float64, half float64) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	var corners [3]*sketch.Point
	for i, p := range pts {
		corners[i] = s.CreatePoint(p[0], p[1])
	}
	s.Fix(corners[0])
	for i := range corners {
		s.CreateLine(corners[i], corners[(i+1)%len(corners)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(half)})
	require.NoError(t, err)
	return body
}

// translated moves a body by (x, y, z), consuming it.
func translated(t *testing.T, b *decad.Body, x, y, z float64) *decad.Body {
	t.Helper()
	tr, err := r3.Translation(r3.Vec{X: x, Y: y, Z: z})
	require.NoError(t, err)
	moved, err := b.Placed(tr)
	require.NoError(t, err)
	return moved
}

// volumeMM reads a volume measurement in mm³.
func volumeMM(t *testing.T, m decad.Measurement) float64 {
	t.Helper()
	v, err := m.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	return v
}

// boundMM3 reads a volume bound in mm³.
func boundMM3(t *testing.T, m decad.Measurement) float64 {
	t.Helper()
	v, err := m.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	return v
}

// requireBodyWatertight tessellates the body at a coarse tolerance and runs
// the edge-pairing audit.
func requireBodyWatertight(t *testing.T, b *decad.Body) {
	t.Helper()
	mesh, err := b.Tessellate(units.Millimeters(1000))
	require.NoError(t, err)
	requireWatertight(t, mesh)
}

func TestUnionOverlappingCubes(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)

	got, err := decad.Union(a, b)
	require.NoError(t, err)

	// 1000 + 1000 − 5³ overlap: the pair is all-planar and every contact
	// point rounds exactly, so the boolean stays honest-Exact.
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Equal(t, 1875.0, volumeMM(t, vol))
	require.Zero(t, boundMM3(t, vol))

	require.Len(t, got.Faces(), 12)
	require.Len(t, got.Lumps(), 1)
	requireBodyWatertight(t, got)

	// The operands are consumed: retired from the document, the result
	// registered (core §6/§8).
	require.Equal(t, []*decad.Body{got}, doc.Bodies())
	_, err = a.Volume()
	require.NoError(t, err, `a retired body stays readable`)

	// The step records the op with both operands as StepRefs.
	steps := doc.Recipe().Steps
	require.Len(t, steps, 4) // two extrudes, then placed, then union
	require.Equal(t, decad.OpUnion, steps[3].Op)
	require.Equal(t, []decad.StepRef{0, 2}, steps[3].Inputs)
}

func TestUnionDisjointCubes(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 20, 0, 30, 10, 10)

	got, err := decad.Union(a, b)
	require.NoError(t, err)

	// Disjoint: the union is exactly the sum, two lumps, nothing chorded,
	// nothing rounded — Exact with a zero bound.
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Equal(t, 2000.0, volumeMM(t, vol))
	require.Zero(t, boundMM3(t, vol))
	require.Len(t, got.Lumps(), 2)
	require.Len(t, got.Shells(), 2)
	requireBodyWatertight(t, got)
}

func TestIntersectOverlappingCubes(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)

	got, err := decad.Intersect(a, b)
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Equal(t, 125.0, volumeMM(t, vol))
	requireBodyWatertight(t, got)
}

func TestIntersectDisjointIsEmpty(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 20, 0, 30, 10, 10)

	_, err := decad.Intersect(a, b)
	require.ErrorIs(t, err, decad.ErrBooleanFailed)

	// A failed boolean leaves the document untouched: both operands live,
	// no step recorded.
	require.Len(t, doc.Bodies(), 2)
	require.Len(t, doc.Recipe().Steps, 2)
}

func TestCutDrillsHole(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	// An off-center hole so the tool circle lands wholly inside one cap
	// facet — the closed-loop subdivision path.
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)

	got, err := decad.Cut(plate, tool)
	require.NoError(t, err)

	// 20×20×8 minus a full-height r=2 hole.
	analytic := 3200 - math.Pi*4*8
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	bound := boundMM3(t, vol)
	require.Positive(t, bound)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-analytic), bound,
		`the analytic volume lies within the proven bound`)
	// The §2 gate shape at the default tolerance: the bound is well inside
	// one part in a thousand of the value.
	require.LessOrEqual(t, bound, 1e-3*volumeMM(t, vol))

	// 4 plate sides + 2 drilled caps + the hole wall.
	require.Len(t, got.Faces(), 7)
	twoLoops := 0
	for _, f := range got.Faces() {
		require.Equal(t, decad.KindFaceted, f.Surface().Kind())
		if len(f.Loops()) == 2 {
			twoLoops++
		}
	}
	require.Equal(t, 3, twoLoops, `both caps carry a hole loop, and the hole wall its two rim loops`)
	requireBodyWatertight(t, got)

	// Provenance survives the boolean: the hole wall remembers the tool's
	// producing step (the placement that positioned it — Placed re-evaluates
	// under its own ref), so FaceCreatedBy can still find it.
	toolStep := decad.StepRef(2)
	found := false
	for _, f := range got.Faces() {
		for _, o := range f.Origins() {
			if o.Step == toolStep {
				found = true
			}
		}
	}
	require.True(t, found, `some face traces back to the tool's extrude`)

	// Cut's Inputs order is [target, tool] (core §6.2).
	steps := doc.Recipe().Steps
	last := steps[len(steps)-1]
	require.Equal(t, decad.OpCut, last.Op)
	require.Equal(t, []decad.StepRef{0, 2}, last.Inputs)
}

func TestCutEmbeddedToolMakesVoid(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, boxBody(t, doc, 8, 8, 12, 12, 4), 0, 0, 2)

	got, err := decad.Cut(plate, tool)
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Equal(t, 3200.0-64.0, volumeMM(t, vol))
	require.Len(t, got.Lumps(), 1)
	require.Len(t, got.Shells(), 2)
	voids := 0
	for _, s := range got.Shells() {
		if s.IsVoid() {
			voids++
		}
	}
	require.Equal(t, 1, voids)
	requireBodyWatertight(t, got)
}

func TestCutRemovingEverythingIsEmpty(t *testing.T) {
	doc := decad.New()
	// The target sits strictly inside the tool — no shared boundary plane.
	target := translated(t, boxBody(t, doc, 5, 5, 8, 8, 3), 0, 0, 2)
	tool := boxBody(t, doc, 0, 0, 20, 20, 10)

	_, err := decad.Cut(target, tool)
	require.ErrorIs(t, err, decad.ErrBooleanFailed)
	require.Len(t, doc.Bodies(), 2)
}

func TestCutDisjointKeepsTarget(t *testing.T) {
	doc := decad.New()
	target := boxBody(t, doc, 0, 0, 10, 10, 10)
	tool := boxBody(t, doc, 20, 0, 30, 10, 10)

	got, err := decad.Cut(target, tool)
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 1000.0, volumeMM(t, vol))
	require.Len(t, doc.Bodies(), 1)
}

func TestBooleanRejections(t *testing.T) {
	t.Run("NilOperand", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		_, err := decad.Union(nil, a)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = decad.Union(a, nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("SameBody", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		_, err := decad.Union(a, a)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("ForeignBody", func(t *testing.T) {
		docA := decad.New()
		docB := decad.New()
		a := boxBody(t, docA, 0, 0, 10, 10, 10)
		b := boxBody(t, docB, 5, 5, 15, 15, 10)
		_, err := decad.Union(a, b)
		require.ErrorIs(t, err, decad.ErrForeignBody)
	})
	t.Run("RetiredBody", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
		got, err := decad.Union(a, b)
		require.NoError(t, err)
		c := boxBody(t, doc, 0, 0, 4, 4, 4)
		_, err = decad.Union(a, c)
		require.ErrorIs(t, err, decad.ErrRetiredBody)
		_ = got
	})
	t.Run("CoplanarContact", func(t *testing.T) {
		// Two cubes stacked face on face: a tangent contact the exact
		// predicates refuse to classify — ErrDegenerate, never a wrong mesh.
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 0, 0, 10)
		_, err := decad.Union(a, b)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Len(t, doc.Bodies(), 2)
	})
}

func TestBooleanBoundComposition(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)
	cutBody, err := decad.Cut(plate, tool)
	require.NoError(t, err)
	cutVol, err := cutBody.Volume()
	require.NoError(t, err)

	// A boolean of a boolean: the composed bound never shrinks below what
	// the first result already carried.
	extra := translated(t, boxBody(t, doc, 0, 0, 5, 5, 5), 30, 0, 0)
	both, err := decad.Union(cutBody, extra)
	require.NoError(t, err)
	bothVol, err := both.Volume()
	require.NoError(t, err)
	require.GreaterOrEqual(t, boundMM3(t, bothVol), boundMM3(t, cutVol))
	require.Equal(t, decad.Approximate, bothVol.Exactness)
	requireBodyWatertight(t, both)
}

func TestFacetedPlaced(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)
	drilled, err := decad.Cut(plate, tool)
	require.NoError(t, err)
	before, err := drilled.Volume()
	require.NoError(t, err)

	moved := translated(t, drilled, 100, 0, 0)
	after, err := moved.Volume()
	require.NoError(t, err)
	// A rigid motion preserves the volume; the bound grows by the honest
	// transform-rounding allowance, never shrinks.
	require.InDelta(t, volumeMM(t, before), volumeMM(t, after), 1e-6)
	require.GreaterOrEqual(t, boundMM3(t, after), boundMM3(t, before))
	requireBodyWatertight(t, moved)
	cb, err := drilled.Centroid()
	require.NoError(t, err)
	ca, err := moved.Centroid()
	require.NoError(t, err)
	require.InDelta(t, cb.Value.X+100, ca.Value.X, 1e-6)
	require.Equal(t, []*decad.Body{moved}, doc.Bodies())
}

func TestBooleanRecipeRoundTrip(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 3), 0, 0, -6)
	_, err := decad.Cut(plate, tool)
	require.NoError(t, err)

	recipe := doc.Recipe()
	data, err := recipe.Steps[len(recipe.Steps)-1].MarshalJSON()
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, got.UnmarshalJSON(data))
	require.Equal(t, decad.OpCut, got.Op)
	require.Equal(t, []decad.StepRef{0, 2}, got.Inputs)
}

func TestBooleanVerifyReadsSuspect(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)
	got, err := decad.Cut(plate, tool)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	br := report.Bodies[0]
	require.Same(t, got, br.Body)
	// The held boundary's structural facts are decided; the approximate
	// quantities are staged reading Suspect — an asked question this
	// evaluator cannot yet judge is never a silent pass.
	require.True(t, br.Solid)
	require.True(t, br.Watertight)
	require.True(t, br.Manifold)
	require.NotNil(t, br.Volume)
	require.Equal(t, decad.Approximate, br.Exactness)
	require.Equal(t, decad.Suspect, br.Status)
	require.Equal(t, decad.Suspect, report.Status)
	require.False(t, report.Trustworthy())
}

func TestFacetedTessellateAndExport(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	// A centered hole: the tool circle crosses the cap facets' shared
	// diagonal, exercising the open-chain subdivision path.
	tool := translated(t, diskBody(t, doc, 10, 10, 2), 0, 0, -6)
	got, err := decad.Cut(plate, tool)
	require.NoError(t, err)

	mesh, err := got.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Positive(t, mesh.Bound().Mag())

	// Every facet remembers one of the result's own faces.
	live := map[*decad.Face]struct{}{}
	for _, f := range got.Faces() {
		live[f] = struct{}{}
	}
	for _, f := range mesh.SourceFaces() {
		_, ok := live[f]
		require.True(t, ok)
	}

	// The held polygons cannot be refined: finer than their own bound is
	// staged, never a mesh whose bound overstates its trust.
	_, err = got.Tessellate(units.Millimeters(1e-12))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestCurvedRimLengthRefuses(t *testing.T) {
	// A hole drilled by a cylindrical tool leaves boolean rims on a curved
	// source: their chord-chain length provably understates the true rim,
	// no chord bound covers the excess, and Length refuses. Straight rims
	// between planar sources still answer with their proven bound.
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)
	got, err := decad.Cut(plate, tool)
	require.NoError(t, err)

	var curvedRefused, straightAnswered int
	for _, e := range got.Edges() {
		if _, ok := e.Curve().(decad.FacetedCurve); !ok {
			continue
		}
		m, err := e.Length()
		if err != nil {
			require.ErrorIs(t, err, decad.ErrUnsupported)
			curvedRefused++
			continue
		}
		require.GreaterOrEqual(t, m.Bound.Base(), 0.0)
		straightAnswered++
	}
	require.Positive(t, curvedRefused, `the drilled rims lie on a curved source`)
	require.Positive(t, straightAnswered, `the plate's own outline rims are straight`)
}

func TestUnionRejectsVertexTangentContact(t *testing.T) {
	// Two cubes sharing exactly one corner vertex pinch at a point: an
	// isolated point contact no crossing chain owns, which the boolean
	// refuses rather than stitching a non-manifold result.
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 10, 10, 10)
	_, err := decad.Union(a, b)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestPlanarUnionAreaBoundIsTiny(t *testing.T) {
	// The all-planar union keeps an Exact volume; the area always carries
	// the ulp-scale float-summation bound — Approximate, but tiny against
	// any real tolerance.
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 20, 20, 8)
	b := translated(t, boxBody(t, doc, 0, 0, 20, 20, 8), 15, 15, -4)
	got, err := decad.Union(a, b)
	require.NoError(t, err)
	area, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Less(t, area.Bound.Base(), 1e-9*area.Value.Base(), `ulp-scale, far under any gate`)
}

func TestPlanarUnionCentroidBoundCoversRounding(t *testing.T) {
	// A union whose exact centroid coordinates are non-binary rationals:
	// all three coordinates round at once, and the reported 3D bound must
	// cover the true distance between the exact and reported centroids.
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 7, 5, 3)
	b := translated(t, boxBody(t, doc, 0, 0, 7, 5, 3), 5, 3, -1)
	got, err := decad.Union(a, b)
	require.NoError(t, err)
	cen, err := got.Centroid()
	require.NoError(t, err)

	// Recompute the exact centroid from the two boxes minus their overlap
	// (inclusion-exclusion over axis-aligned boxes, exact in closed form).
	type box struct{ x0, y0, z0, x1, y1, z1 float64 }
	boxes := []struct {
		b box
		w float64
	}{
		{box{0, 0, 0, 7, 5, 3}, 1},
		{box{5, 3, -1, 12, 8, 2}, 1},
		{box{5, 3, 0, 7, 5, 2}, -1},
	}
	var vol, mx, my, mz float64
	for _, e := range boxes {
		v := (e.b.x1 - e.b.x0) * (e.b.y1 - e.b.y0) * (e.b.z1 - e.b.z0) * e.w
		vol += v
		mx += v * (e.b.x0 + e.b.x1) / 2
		my += v * (e.b.y0 + e.b.y1) / 2
		mz += v * (e.b.z0 + e.b.z1) / 2
	}
	want := r3.NewVec(mx/vol, my/vol, mz/vol)
	dist := cen.Value.Sub(want).Len()
	require.LessOrEqual(t, dist, cen.Bound.Base(), `the 3D bound covers the true rounding distance`)
}

func TestUnionRejectsEdgeEdgePointTouch(t *testing.T) {
	// Two knife-edged prisms crossing at right angles: one wedge opens toward
	// +x and sweeps along z, the other opens toward −x and sweeps along y, so
	// the solids meet at the origin and nowhere else. The touch sits in the
	// interior of both apex edges — no vertex of either mesh lands on it — so
	// the facet pairs that see it properly cross each other's planes and their
	// crossing intervals meet at exactly one point. That zero-length overlap
	// is the contact, and the boolean refuses it like any isolated point touch.
	doc := decad.New()
	w := sketch.NewWorld()
	a := wedgePrism(t, doc, w, w.XY(), [3][2]float64{{0, 0}, {12, -4}, {12, 6}}, 5)
	b := wedgePrism(t, doc, w, w.XZ(), [3][2]float64{{0, 0}, {-12, -6}, {-12, 5}}, 5)

	// The apex edges really do cross at the origin: each prism owns it as an
	// interior point of an edge, never as a vertex.
	for _, body := range []*decad.Body{a, b} {
		for _, v := range body.Vertices() {
			require.NotZero(t, v.Position().Value.Len(), `the touch point is no mesh vertex`)
		}
	}

	_, err := decad.Union(a, b)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

// polyPrism extrudes a closed polygon on the given plane, symmetric about it.
func polyPrism(t *testing.T, doc *decad.Document, w *sketch.World, plane *sketch.Plane, pts [][2]float64, half float64) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	corners := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		corners[i] = s.CreatePoint(p[0], p[1])
	}
	s.Fix(corners[0])
	for i := range corners {
		s.CreateLine(corners[i], corners[(i+1)%len(corners)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(half)})
	require.NoError(t, err)
	return body
}

func TestUnionRefusesTangentCylinder(t *testing.T) {
	// A Ø20 cylinder whose TRUE surface is tangent to the plate's x = 20 face
	// along a line. The chord polygon lies strictly inside the true cylinder,
	// so the tangency vanishes from the tessellation and the mesh boolean would
	// happily report a clean two-lump body for two solids that genuinely touch —
	// with the verdict decided by where the chord samples happened to fall. The
	// pre-tessellation proximity gate refuses the question instead.
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	cyl := diskBody(t, doc, 30, 10, 10)

	_, err := decad.Union(plate, cyl)
	require.ErrorIs(t, err, decad.ErrUnsupported)

	// A refused boolean leaves the document untouched.
	require.Len(t, doc.Bodies(), 2)

	// Moving the cylinder a millimetre clear of the plate's face is decidable
	// again — the refusal is about proximity, not about the shapes.
	doc2 := decad.New()
	plate2 := boxBody(t, doc2, 0, 0, 20, 20, 8)
	apart := diskBody(t, doc2, 31, 10, 10)
	got, err := decad.Union(plate2, apart)
	require.NoError(t, err)
	require.Len(t, got.Lumps(), 2)
}

func TestUnionCrossingApexEdgeBuilds(t *testing.T) {
	// A diamond-section prism driven through a plate, its two apex edges landing
	// EXACTLY in the plate's top plane z = 2. Whether an in-plane edge grazes the
	// other operand or crosses it is not a property of any facet pair — it is a
	// property of the edge's two adjacent facets, which straddle z = 2 here. So
	// this is an ordinary crossing and it must build, at the exact volume.
	doc := decad.New()
	w := sketch.NewWorld()
	plate := boxBody(t, doc, 0, 0, 10, 10, 2)
	prism := polyPrism(t, doc, w, w.XZ(), [][2]float64{{5, 2}, {7, 3}, {9, 2}, {7, 1}}, 5)

	got, err := decad.Union(plate, prism)
	require.NoError(t, err)

	// 10×10×2 plate = 200; the diamond (diagonals 4 and 2) swept ±5 = 40; the
	// overlap is the diamond's lower half (area 2) over the plate's own y range
	// 0..5 = 10. 200 + 40 − 10.
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, 230.0, volumeMM(t, vol))
	require.Len(t, got.Lumps(), 1)
	requireBodyWatertight(t, got)

	// The control: lift the prism 0.25 mm so nothing is coincident. Only the
	// exact coincidence was ever in question.
	doc2 := decad.New()
	w2 := sketch.NewWorld()
	plate2 := boxBody(t, doc2, 0, 0, 10, 10, 2)
	lifted := polyPrism(t, doc2, w2, w2.XZ(), [][2]float64{{5, 2.25}, {7, 3.25}, {9, 2.25}, {7, 1.25}}, 5)
	got2, err := decad.Union(plate2, lifted)
	require.NoError(t, err)
	vol2, err := got2.Volume()
	require.NoError(t, err)
	require.Equal(t, 234.375, volumeMM(t, vol2))
}

func TestUnionRejectsKnifeEdgeGraze(t *testing.T) {
	// A wedge unioned into a wider wedge sharing the same knife-edge line. The
	// inner wedge's two apex facets lie strictly on ONE side of the outer wedge's
	// face: the boundary touches the plane and comes back. That is a true graze,
	// no side classification is proven for it, and it stays refused.
	doc := decad.New()
	w := sketch.NewWorld()
	a := wedgePrism(t, doc, w, w.XY(), [3][2]float64{{0, 0}, {12, -4}, {12, 6}}, 5)
	b := wedgePrism(t, doc, w, w.XY(), [3][2]float64{{0, 0}, {8, -1}, {8, 2}}, 3)

	_, err := decad.Union(a, b)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Len(t, doc.Bodies(), 2)
}

func TestBooleanRimVertexBoundCoversTrimAmplification(t *testing.T) {
	// Two Ø10 prisms 9.9 mm apart: their walls cross at a shallow angle, so a rim
	// vertex — which sits at the crossing of two chord PLANES, not on either
	// surface — is displaced from the TRUE rim by (δA + δB)/sin θ, not by δ. The
	// reported bound must cover the real displacement.
	doc := decad.New()
	a := diskBody(t, doc, 0, 0, 5)
	b := translated(t, diskBody(t, doc, 9.9, 0, 5), 0, 0, 5)
	got, err := decad.Union(a, b)
	require.NoError(t, err)

	// The true rim: the vertical lines through the two circles' intersections.
	rx := 9.9 / 2
	ry := math.Sqrt(25 - rx*rx)
	rims := 0
	for _, v := range got.Vertices() {
		pos := v.Position()
		p := pos.Value
		// A rim vertex lies strictly inside BOTH circles — it is on neither
		// operand's own chording, which samples the circles exactly.
		if math.Hypot(p.X, p.Y) > 4.9999 || math.Hypot(p.X-9.9, p.Y) > 4.9999 {
			continue
		}
		rims++
		d := math.Hypot(p.X-rx, math.Abs(p.Y)-ry)
		require.LessOrEqual(t, d, pos.Bound.Base(),
			`a rim vertex must lie within its own reported bound of the true rim`)
	}
	require.Positive(t, rims, `the two walls do cross`)
}

func TestBooleanAreaBoundsCoverShallowCrossing(t *testing.T) {
	// The intersection of two parallel-axis circular prisms is a prism over the
	// 2-D lens, so its true area and volume are closed form. At a 9.99 mm centre
	// distance the two Ø10 walls cross at a very shallow angle: the rim moves far
	// more than δ, and every bound that multiplies a perimeter must say so.
	doc := decad.New()
	const d, r, h = 9.99, 5.0, 15.0
	a := diskBody(t, doc, 0, 0, r)
	b := translated(t, diskBody(t, doc, d, 0, r), 0, 0, 5)
	got, err := decad.Intersect(a, b)
	require.NoError(t, err)

	th := math.Acos(d / 2 / r)
	lens := 2 * r * r * (th - math.Sin(th)*math.Cos(th))
	trueArea := 2*lens + 4*r*th*h
	trueVol := lens * h

	vol, err := got.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-trueVol), boundMM3(t, vol))

	area, err := got.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-trueArea), area.Bound.Base(),
		`the body's area bound covers the true area`)

	// Each wall face is a cylinder patch of area r·2θ·h. Its own bound must
	// cover it too — the rim bounding it moved by the amplified displacement.
	walls := 0
	trueWall := r * 2 * th * h
	for _, f := range got.Faces() {
		m, err := f.Area()
		require.NoError(t, err)
		if m.Value.Base() < 1 {
			continue // a cap: the sliver-thin lens
		}
		walls++
		require.LessOrEqual(t, math.Abs(m.Value.Base()-trueWall), m.Bound.Base(),
			`a wall face's area bound covers its true area`)
	}
	require.Equal(t, 2, walls)
}

func TestFacetedPlacedFarFromOriginBoundHolds(t *testing.T) {
	// An all-planar union built at x = 1e7, then rotated and translated back to
	// the origin. The rounding a rigid motion commits happens at the magnitude of
	// its INPUT, not of its result — a body charged at the result's magnitude is
	// charged nothing. A rigid motion preserves volume and area exactly, so the
	// pre-move Exact readings ARE the truth.
	doc := decad.New()
	a := boxBody(t, doc, 1e7, 0, 1e7+10, 10, 10)
	b := translated(t, boxBody(t, doc, 1e7, 0, 1e7+10, 10, 10), 5, 5, 5)
	got, err := decad.Union(a, b)
	require.NoError(t, err)
	volBefore, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volBefore.Exactness)
	areaBefore, err := got.Area()
	require.NoError(t, err)

	cen, err := got.Centroid()
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(1))
	require.NoError(t, err)
	back, err := r3.Translation(rot.Apply(cen.Value).Scale(-1))
	require.NoError(t, err)
	xform, err := rot.Then(back)
	require.NoError(t, err)
	moved, err := got.Placed(xform)
	require.NoError(t, err)

	volAfter, err := moved.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volAfter)-volumeMM(t, volBefore)), boundMM3(t, volAfter),
		`the moved volume stays within its own bound of the exact pre-move volume`)
	areaAfter, err := moved.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(areaAfter.Value.Base()-areaBefore.Value.Base()), areaAfter.Bound.Base(),
		`the moved area stays within its own bound of the pre-move area`)
}

func TestFacetedBoxBoundIsA3DRadius(t *testing.T) {
	// Box.Min and Box.Max are POSITIONS, and Bound is the error bound on them:
	// a 3-D radius, not a per-axis extent. The held extreme sits one chord
	// displacement inside the truth on EVERY axis at once, so the per-coordinate
	// reading is exactly tight and leaves no headroom for the corner distance.
	doc := decad.New()
	disk := diskBody(t, doc, 0, 0, 5)
	box := translated(t, boxBody(t, doc, 0, 0, 8, 8, 20), 0, 0, 3)
	got, err := decad.Union(disk, box)
	require.NoError(t, err)

	bounds, err := got.Bounds()
	require.NoError(t, err)
	trueMin := r3.NewVec(-5, -5, 0)
	require.LessOrEqual(t, bounds.Min.Sub(trueMin).Len(), bounds.Bound.Base(),
		`the box's proven bound covers the 3D distance from the held corner to the true one`)
}

func TestFacetedEdgeLengthNeverClaimsExact(t *testing.T) {
	// A boolean rim's held length is a float SUM of square roots. It is not
	// exactly representable, so it may never report Exact with a zero bound —
	// and its bound must cover the difference from the true chord length. The
	// two models matter for different halves of that: the cubes' contacts round
	// EXACTLY (nothing displaces the rim, so the old bound was a literal zero),
	// while the wedge's rims are diagonal (so the float sum really is off).
	for _, tc := range []string{"exactly-rounding cubes", "diagonal rims"} {
		t.Run(tc, func(t *testing.T) { facetedEdgeLengths(t, tc) })
	}
}

func facetedEdgeLengths(t *testing.T, model string) {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	var b *decad.Body
	if model == "diagonal rims" {
		b = wedgePrism(t, doc, w, w.XZ(), [3][2]float64{{3, 5}, {9, 13}, {13, 4}}, 12)
	} else {
		b = translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	}
	got, err := decad.Union(a, b)
	require.NoError(t, err)

	answered := 0
	for _, e := range got.Edges() {
		if _, ok := e.Curve().(decad.FacetedCurve); !ok {
			continue
		}
		m, err := e.Length()
		if err != nil {
			continue // a rim on a curved source refuses outright
		}
		answered++
		require.Equal(t, decad.Approximate, m.Exactness, `a float sum of sqrts is not exactly representable`)
		require.Positive(t, m.Bound.Base())

		// The rim is straight (both sources are planar), so its true length is
		// the distance between its endpoints — at 200 bits, well past float64.
		p := e.Start().Position().Value
		q := e.End().Position().Value
		sum := new(big.Float).SetPrec(200)
		for _, c := range []float64{q.X - p.X, q.Y - p.Y, q.Z - p.Z} {
			sq := new(big.Float).SetPrec(200).SetFloat64(c)
			sum.Add(sum, sq.Mul(sq, sq))
		}
		diff := new(big.Float).SetPrec(200).SetFloat64(m.Value.Base())
		diff.Sub(diff, sum.Sqrt(sum))
		err2, _ := diff.Abs(diff).Float64()
		require.LessOrEqual(t, err2, m.Bound.Base(), `the length bound covers the true length`)
	}
	require.Positive(t, answered)
}

func TestSecondGenerationPlanarRimsAnswer(t *testing.T) {
	// A boolean of a boolean. Every face of both operands is genuinely planar,
	// so every rim of the result is a straight line whose chord length is
	// honest — and every one of them must ANSWER. A Faceted face IS its
	// polygons: its flatness is a fact about the held mesh, decided exactly on
	// it, not a guess read off the surface tag.
	doc := decad.New()
	w := sketch.NewWorld()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := wedgePrism(t, doc, w, w.XZ(), [3][2]float64{{3, 5}, {9, 13}, {13, 4}}, 12)
	first, err := decad.Union(a, b)
	require.NoError(t, err)

	c := translated(t, boxBody(t, doc, 0, 0, 4, 4, 4), 6, 2, 8)
	second, err := decad.Union(first, c)
	require.NoError(t, err)

	refused := 0
	for _, e := range second.Edges() {
		if _, err := e.Length(); err != nil {
			refused++
		}
	}
	require.Positive(t, len(second.Edges()))
	require.Zero(t, refused, `an all-planar second-generation boolean refuses no rim`)
}
