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

// boxBody extrudes an x0..x1 × y0..y1 rectangle to a height-h prism on a
// fresh sketch of the given document.
func boxBody(t *testing.T, doc *decad.Document, x0, y0, x1, y1, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

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
