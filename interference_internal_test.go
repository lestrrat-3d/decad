package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func internalDiskRegion(t *testing.T, cx, cy, radius float64) region2 {
	t.Helper()
	e, ok := arcElem(cx, cy, radius, 0, 2*math.Pi, true)
	require.True(t, ok)
	return newRegion2([]surveyElem{e})
}

func TestTrimmedCircleCrossingRequiresRevolvedFaceAdmission(t *testing.T) {
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	wide := [2]r3.Vec{r3.NewVec(-20, -20, -20), r3.NewVec(20, 20, 20)}
	k := testKernel()
	k.ctx = t.Context()

	t.Run("partial cylinder sweep outside the plane trim", func(t *testing.T) {
		plane := &cFace{kind: ckPlane, u: x, v: y, n: z, region: internalDiskRegion(t, -10, 0, 2), box: wide}
		cyl := &cFace{
			kind: ckCylinder, axis: z, refU: x, refV: y, radius: 10,
			zWin: newLinWindow(-2, 2), sweep: newAngWindow(0, math.Pi/4), box: wide,
		}
		sink := &cellSink{}
		k.planeCrossesRevolved(plane, cyl, sink)
		require.False(t, sink.overlap,
			`the full carrier meets the plane trim only outside the partial revolve sweep`)
		cyl.sweep = angWindow{full: true}
		sink = &cellSink{}
		k.planeCrossesRevolved(plane, cyl, sink)
		require.True(t, sink.overlap)
	})

	t.Run("sphere meridian outside the plane trim", func(t *testing.T) {
		plane := &cFace{kind: ckPlane, u: x, v: y, n: z, region: internalDiskRegion(t, 10, 0, 2), box: wide}
		sphere := &cFace{
			kind: ckSphere, axis: z, refU: x, refV: y, radius: 10,
			merid: newAngWindow(0, math.Pi/4), sweep: angWindow{full: true}, box: wide,
		}
		sink := &cellSink{}
		k.planeSphere(plane, sphere, sink)
		require.False(t, sink.overlap,
			`the equator is outside the shipped sphere patch's meridian trim`)
		sphere.merid = angWindow{full: true}
		sink = &cellSink{}
		k.planeSphere(plane, sphere, sink)
		require.True(t, sink.overlap)
	})
}

type internalCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	limit           int
	calls           int
}

func (c *internalCancelContext) Err() error {
	c.calls++
	if c.calls >= c.limit {
		return context.Canceled
	}
	return nil
}

func TestPointInBodyCancellationReachesTorusRootPath(t *testing.T) {
	z := r3.NewVec(0, 0, 1)
	torus := torFace(r3.Vec{}, z, 5, 1)
	g := &bodyGeom{faces: []*cFace{torus}}
	ctx := &internalCancelContext{Context: t.Context(), limit: 4}

	_, _, err := g.pointInBody(ctx, r3.NewVec(0.31, 0.73, 0.19), 1e-9)
	require.ErrorIs(t, err, context.Canceled)
	require.GreaterOrEqual(t, ctx.calls, 4, `cancellation must pass pointInBody and rayCrossings into the torus root path`)
}

func TestCertifiedRootRefinementCancellation(t *testing.T) {
	p := ratPoly{big.NewRat(-2, 1), new(big.Rat), big.NewRat(1, 1)}
	ivs, err := rpIsolateRootsContext(t.Context(), p)
	require.NoError(t, err)
	require.NotEmpty(t, ivs)
	ctx := &internalCancelContext{Context: t.Context(), limit: 1}

	_, err = rpRefineRootContext(ctx, sturmChain(p), ivs[0], func(float64, float64) bool { return false })
	require.ErrorIs(t, err, context.Canceled)
}

func TestFacetAdjacencyCancellationIsBounded(t *testing.T) {
	tris := make([][3]int, 600)
	for i := range tris {
		tris[i] = [3]int{3 * i, 3*i + 1, 3*i + 2}
	}
	ctx := &internalCancelContext{Context: t.Context(), limit: 2}

	_, err := facetAdjacencyContext(ctx, tris)
	require.ErrorIs(t, err, context.Canceled)
}

func internalBoxBody(t *testing.T, doc *Document, x0, y0, x1, y1, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

func TestMultiLumpFacetedBodyBypassesAnalyticContainment(t *testing.T) {
	doc := New()
	outer := internalBoxBody(t, doc, 0, 0, 20, 20, 10)
	a := internalBoxBody(t, doc, 2, 2, 4, 4, 2)
	b := internalBoxBody(t, doc, 10, 10, 12, 12, 2)
	multi, err := Union(a, b)
	require.NoError(t, err)
	require.Len(t, multi.Lumps(), 2)

	_, ok := newBodyGeom(multi)
	require.False(t, ok, `a faceted multi-lump body must not enter analytic containment with a partial model`)
	res, err := clearancePair(t.Context(), outer, multi, false)
	require.NoError(t, err)
	require.Equal(t, pairUndecided, res.verdict)
	require.Nil(t, res.contained)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, Suspect, report.Status, `unsupported multi-lump intersection stays explicit rather than reusing a partial containment witness`)
	require.Empty(t, report.Interferences)
}
