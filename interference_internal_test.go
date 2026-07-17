package decad

import (
	"context"
	"math"
	"math/big"
	"runtime"
	"strings"
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

type internalNestedScanCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	enteredNested   bool
}

func (c *internalNestedScanCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		frame, more := frames.Next()
		if strings.HasSuffix(frame.Function, ".coplanarBoundaryClearanceBudget") {
			c.enteredNested = true
			return context.Canceled
		}
		if !more {
			return nil
		}
	}
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

func TestFacetCutCancellationIsBounded(t *testing.T) {
	tri := [3]xpt{
		xptOf(r3.NewVec(0, 0, 0)),
		xptOf(r3.NewVec(10, 0, 0)),
		xptOf(r3.NewVec(0, 10, 0)),
	}
	normal := xcross(xsub(tri[1], tri[0]), xsub(tri[2], tri[0]))
	seg := xseg{
		a: xptOf(r3.NewVec(1, 1, 0)),
		b: xptOf(r3.NewVec(2, 1, 0)),
	}
	segs := make([]xseg, 300)
	for i := range segs {
		segs[i] = seg
	}
	ctx := &internalCancelContext{Context: t.Context(), limit: 2}

	_, err := cutTriangle(ctx, tri, normal, segs)
	require.ErrorIs(t, err, context.Canceled)
}

func internalPolygonRegion(t *testing.T, cx, cy, radius float64, sides int) region2 {
	t.Helper()
	elems := make([]surveyElem, sides)
	for i := range sides {
		a := 2 * math.Pi * float64(i) / float64(sides)
		b := 2 * math.Pi * float64(i+1) / float64(sides)
		e, ok := lineElem(
			cx+radius*math.Cos(a), cy+radius*math.Sin(a),
			cx+radius*math.Cos(b), cy+radius*math.Sin(b),
		)
		require.True(t, ok)
		elems[i] = e
	}
	return newRegion2(elems)
}

func TestCoplanarRelationCancellationIsBounded(t *testing.T) {
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	f := &cFace{
		kind: ckPlane, u: x, v: y, n: z,
		region: internalPolygonRegion(t, -30, 0, 10, 24),
	}
	g := &cFace{
		kind: ckPlane, u: x, v: y.Scale(-1), n: z.Scale(-1),
		region: internalPolygonRegion(t, 30, 0, 10, 24),
	}
	k := &pairKernel{
		a: &bodyGeom{faces: []*cFace{f}},
		b: &bodyGeom{faces: []*cFace{g}},
	}

	t.Run("whole certificate", func(t *testing.T) {
		ctx := &internalNestedScanCancelContext{Context: t.Context()}
		_, err := k.coplanarContactCertified(ctx)
		require.ErrorIs(t, err, context.Canceled)
		require.True(t, ctx.enteredNested, `the public certificate path must reach the nested boundary scan before cancellation`)
	})

	t.Run("nested boundary scan", func(t *testing.T) {
		ctx := &internalCancelContext{Context: t.Context(), limit: 1}
		_, err := coplanarBoundaryClearanceBudget(newClearanceBudget(ctx), f.region, g.region)
		require.ErrorIs(t, err, context.Canceled)
	})
}

func TestInterferencePairDiameterUsesAllFacetedPayloadVertices(t *testing.T) {
	a := &Body{payload: facetedPayload{verts: []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(100, 0, 0),
	}}}
	b := &Body{payload: facetedPayload{verts: []r3.Vec{r3.NewVec(0, 0, 0)}}}

	diameter, err := interferencePairDiameter(t.Context(), a, b)
	require.NoError(t, err)
	require.Equal(t, 100.0, diameter)
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
