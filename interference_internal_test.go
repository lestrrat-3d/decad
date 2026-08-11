package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"reflect"
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

// internalFrameCancelContext cancels only once the named function is on the
// call stack. A test using it proves the poll it observed is INSIDE that
// function, rather than at some earlier phase boundary that would report
// cancellation without ever entering the loop under test.
type internalFrameCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	target          string
	entered         bool
}

func (c *internalFrameCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		frame, more := frames.Next()
		if strings.HasSuffix(frame.Function, "."+c.target) {
			c.entered = true
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
	chain := newSturmChainInt(sturmChain(p))
	ivs, err := rpIsolateRootsContext(t.Context(), p, chain)
	require.NoError(t, err)
	require.NotEmpty(t, ivs)
	ctx := &internalCancelContext{Context: t.Context(), limit: 1}

	_, err = rpRefineRootContext(ctx, chain, ivs[0], func(float64, float64) bool { return false })
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
		ctx := &internalFrameCancelContext{Context: t.Context(), target: "coplanarBoundaryClearanceBudget"}
		_, err := k.coplanarContactCertified(ctx)
		require.ErrorIs(t, err, context.Canceled)
		require.True(t, ctx.entered, `the public certificate path must reach the nested boundary scan before cancellation`)
	})

	t.Run("nested boundary scan", func(t *testing.T) {
		ctx := &internalCancelContext{Context: t.Context(), limit: 1}
		_, err := coplanarBoundaryClearanceBudget(newWorkBudget(ctx), f.region, g.region)
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

func TestInterferenceExpectedCausesKeepDistinctDiagnostics(t *testing.T) {
	a := &Body{origin: FeatureRef{Step: 12}}
	b := &Body{origin: FeatureRef{Step: 34}}

	for _, tc := range []struct {
		name        string
		expected    *booleanExpectedError
		wantOutcome interferenceOutcome
		wantCode    DiagnosticCode
		wantMessage []string
	}{
		{
			name:        "first payload",
			expected:    &booleanExpectedError{kind: booleanExpectedStaging, operand: 0},
			wantOutcome: interferenceUnsupportedPayloadFirst,
			wantCode:    DiagUnsupportedPairPayload,
			wantMessage: []string{"first operand", "step 12", "tessellatable body type"},
		},
		{
			name:        "second payload",
			expected:    &booleanExpectedError{kind: booleanExpectedStaging, operand: 1},
			wantOutcome: interferenceUnsupportedPayloadSecond,
			wantCode:    DiagUnsupportedPairPayload,
			wantMessage: []string{"second operand", "step 34", "tessellatable body type"},
		},
		{
			name:        "contact policy",
			expected:    &booleanExpectedError{kind: booleanExpectedContact, operand: -1},
			wantOutcome: interferenceUnsupportedContact,
			wantCode:    DiagUnsupportedPairContact,
			wantMessage: []string{"contact", "clear separation"},
		},
		{
			name:        "in-pipeline reach",
			expected:    &booleanExpectedError{kind: booleanExpectedUnsupported, operand: -1},
			wantOutcome: interferenceUnsupportedPipeline,
			wantCode:    DiagUnsupportedPairPipeline,
			wantMessage: []string{"both operands tessellate", "simplify the boolean geometry"},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			outcome := interferenceOutcomeForExpected(tc.expected)
			require.Equal(t, tc.wantOutcome, outcome)

			diag := undecidedPairDiag(a, b, pairOverlapping, outcome)
			require.Equal(t, tc.wantCode, diag.Code)
			require.Equal(t, Suspect, diag.Status)
			require.Equal(t, ReadingNone, diag.Reading)
			require.Equal(t, &DiagnosticPair{A: a, B: b}, diag.Pair)
			for _, want := range tc.wantMessage {
				require.Contains(t, diag.Message, want)
			}

			legacy, ok := legacyUnsupportedPairDiag(a, b, diag.Code)
			require.True(t, ok, `a staged cause preserves the broad compatibility signal`)
			require.Equal(t, DiagUnsupportedPair, legacy.Code)
			require.Equal(t, Suspect, legacy.Status)
			require.Equal(t, ReadingNone, legacy.Reading)
			require.Equal(t, &DiagnosticPair{A: a, B: b}, legacy.Pair)
		})
	}

	undecided := undecidedPairDiag(a, b, pairOverlapping, interferenceUndecided)
	require.Equal(t, DiagUndecidedInterference, undecided.Code)
	_, legacy := legacyUnsupportedPairDiag(a, b, undecided.Code)
	require.False(t, legacy, `an unmeasured overlap has no staged-pair compatibility signal`)

	partition := undecidedPairDiag(a, b, pairUndecided, interferenceUndecided)
	require.Equal(t, DiagUndecidedPair, partition.Code)
}

func TestSharesFacePlane(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	b := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	require.True(t, sharesFacePlane(a, b),
		`two prisms built on the same sketch plane share every face plane`)

	c := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	tr, err := r3.Translation(r3.NewVec(100, 100, 100))
	require.NoError(t, err)
	off, err := c.Placed(tr)
	require.NoError(t, err)
	require.False(t, sharesFacePlane(a, off),
		`translating a prism off every one of its planes leaves no shared face plane`)
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

// The cancellation tests below each drive one loop that the interference
// design's §7.2 polling rule covers. The work in each is sized past
// workPollInterval on purpose: any run of workPollInterval consecutive budget
// steps contains a poll, so a loop that steps that many times must observe a
// cancellation delivered before it started. internalFrameCancelContext then
// proves the poll happened INSIDE the loop under test rather than at an earlier
// phase boundary.

func TestHoleOrderingCancellationIsBounded(t *testing.T) {
	pts := []Point2{{U: 0, V: 0}, {U: 100, V: 0}, {U: 100, V: 100}, {U: 0, V: 100}}
	loops := [][]int{{0, 1, 2, 3}}
	for h := range 8 {
		var hole []int
		cx := 10 + 10*float64(h)
		for k := range 40 {
			th := -2 * math.Pi * float64(k) / 40 // holes run clockwise
			pts = append(pts, Point2{U: cx + 2*math.Cos(th), V: 50 + 2*math.Sin(th)})
			hole = append(hole, len(pts)-1)
		}
		loops = append(loops, hole)
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "maxU"}

	_, err := triangulate2DContext(ctx, pts, loops)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered,
		`hole ordering must poll inside the key scan, not only before the sort`)
}

func TestHoleOrderingKeepsRightToLeftBridging(t *testing.T) {
	// The same eight-hole region, uncancelled: the ordering refactor must still
	// reduce it to triangles whose total area is the outer square less every
	// hole. The holes are supplied left-to-right, so a triangulation this exact
	// only comes out if they were re-ordered right-to-left before bridging.
	pts := []Point2{{U: 0, V: 0}, {U: 100, V: 0}, {U: 100, V: 100}, {U: 0, V: 100}}
	loops := [][]int{{0, 1, 2, 3}}
	holeArea := 0.0
	for h := range 8 {
		var hole []int
		cx := 10 + 10*float64(h)
		for k := range 40 {
			th := -2 * math.Pi * float64(k) / 40
			pts = append(pts, Point2{U: cx + 2*math.Cos(th), V: 50 + 2*math.Sin(th)})
			hole = append(hole, len(pts)-1)
		}
		loops = append(loops, hole)
		holeArea += 0.5 * 40 * math.Sin(2*math.Pi/40) * 4 // regular 40-gon, r = 2
	}

	tris, err := triangulate2DContext(t.Context(), pts, loops)
	require.NoError(t, err)
	require.NotEmpty(t, tris)
	total := 0.0
	for _, tri := range tris {
		total += 0.5 * cross2(pts[tri[0]], pts[tri[1]], pts[tri[2]])
	}
	require.InDelta(t, 100*100-holeArea, total, 1e-9,
		`the bridged triangulation must cover the outer square less every hole`)
}

func TestConformCandidateScanCancellationIsBounded(t *testing.T) {
	verts := []xpt{
		xptOf(r3.NewVec(0, 0, 0)),
		xptOf(r3.NewVec(1000, 0, 0)),
		xptOf(r3.NewVec(0, 1, 0)),
	}
	// An edge spanning the mesh diagonal sweeps the cells of the whole grid and
	// every vertex standing in them, which is well past the polling interval on
	// either count.
	for i := range 2 * workPollInterval {
		verts = append(verts, xptOf(r3.NewVec(float64(i)+0.5, 7, 0)))
	}
	scan, err := newConformScan(newWorkBudget(t.Context()), verts)
	require.NoError(t, err)
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "edgeInteriorHits"}

	_, err = scan.edgeInteriorHits(newWorkBudget(ctx), 0, 1, [3]int{0, 1, 2})
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered,
		`the grid-cell candidate scan must poll, not run to completion between facet polls`)
}

func TestConformCandidateScanFindsEdgeInteriorVertices(t *testing.T) {
	verts := []xpt{
		xptOf(r3.NewVec(0, 0, 0)),
		xptOf(r3.NewVec(10, 0, 0)),
		xptOf(r3.NewVec(0, 1, 0)),
		xptOf(r3.NewVec(4, 0, 0)),  // exactly interior to edge (0, 1)
		xptOf(r3.NewVec(4, 5, 0)),  // off the edge
		xptOf(r3.NewVec(10, 0, 0)), // the edge's own endpoint, by position
	}
	scan, err := newConformScan(newWorkBudget(t.Context()), verts)
	require.NoError(t, err)

	hits, err := scan.edgeInteriorHits(newWorkBudget(t.Context()), 0, 1, [3]int{0, 1, 2})
	require.NoError(t, err)
	require.Equal(t, []int{3}, hits,
		`only the vertex exactly in the edge's interior conforms the subdivision`)
}

func TestSortAlongEdgeCancellationIsBounded(t *testing.T) {
	verts := []xpt{xptOf(r3.NewVec(0, 0, 0)), xptOf(r3.NewVec(1000, 0, 0))}
	var hits []int
	for i := range 300 {
		verts = append(verts, xptOf(r3.NewVec(float64(300-i), 0, 0)))
		hits = append(hits, i+2)
	}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "sortAlongEdge"}

	err := sortAlongEdge(newWorkBudget(ctx), verts, 0, 1, hits)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered,
		`the along-edge ordering must poll rather than run its whole quadratic pass`)
}

func TestSortAlongEdgeOrdersByExactParameter(t *testing.T) {
	verts := []xpt{xptOf(r3.NewVec(0, 0, 0)), xptOf(r3.NewVec(10, 0, 0))}
	for _, x := range []float64{7, 1, 4} {
		verts = append(verts, xptOf(r3.NewVec(x, 0, 0)))
	}
	hits := []int{2, 3, 4} // parameters 0.7, 0.1, 0.4

	require.NoError(t, sortAlongEdge(newWorkBudget(t.Context()), verts, 0, 1, hits))
	require.Equal(t, []int{3, 4, 2}, hits,
		`inserted vertices must come back ordered along the edge, nearest end first`)
}

func TestAnalyticBodiesEqualCancellationIsBounded(t *testing.T) {
	segs := make([]CurveSegment, 300)
	for i := range segs {
		segs[i] = LineSeg{Start: Point2{U: float64(i)}, End: Point2{U: float64(i + 1)}, TEnd: 1}
	}
	profile := ProfileRecord{Outer: LoopRecord{Segments: segs}}
	a := &Body{payload: prismPayload{profile: profile, z1: 1}}
	b := &Body{payload: prismPayload{profile: profile, z1: 1}}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "loopRecordsEqual"}

	_, err := analyticBodiesEqual(newWorkBudget(ctx), a, b)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered,
		`the set-identity walk must poll inside the per-segment comparison`)
}

func TestAnalyticBodiesEqualMatchesPlainPrismSetIdentity(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	same := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	taller := internalBoxBody(t, doc, 0, 0, 10, 10, 6)
	wider := internalBoxBody(t, doc, 0, 0, 12, 10, 5)

	for _, tc := range []struct {
		name string
		x, y *Body
	}{
		{name: "identical", x: a, y: same},
		{name: "different sweep", x: a, y: taller},
		{name: "different section", x: a, y: wider},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := analyticBodiesEqual(newWorkBudget(t.Context()), tc.x, tc.y)
			require.NoError(t, err)
			require.Equal(t, reflect.DeepEqual(tc.x.payload, tc.y.payload), got,
				`the budgeted walk must agree with set identity for plain prism records`)
		})
	}
}

func TestNewBodyGeomCancellationIsBounded(t *testing.T) {
	doc := New()
	// A real prism, its recorded section swapped for a 300-sided polygon:
	// resolving the carrier profile must poll before the carrier faces are built.
	body := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	pp, ok := body.payload.(prismPayload)
	require.True(t, ok)
	const sides = 300
	corner := func(i int) Point2 {
		th := 2 * math.Pi * float64(i%sides) / sides
		return Point2{U: 100 * math.Cos(th), V: 100 * math.Sin(th)}
	}
	segs := make([]CurveSegment, sides)
	for i := range segs {
		segs[i] = LineSeg{Start: corner(i), End: corner(i + 1), TEnd: 1}
	}
	pp.profile = ProfileRecord{Outer: LoopRecord{Segments: segs}}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "recordLoops"}

	_, _, err := newBodyGeomBudget(newWorkBudget(ctx), &Body{
		lumps:   body.lumps,
		payload: pp,
	})
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered,
		`the kernel model must poll while resolving carrier profile loops`)
}

func TestAddRevolveFacesCancellationReachesRevolveLoops(t *testing.T) {
	calls := 0
	budget := &workBudget{
		stepFn: func() error {
			calls++
			return context.Canceled
		},
		errFn: func() error { return nil },
	}
	_, err := (&bodyGeom{}).addRevolveFaces(budget, revolvePayload{
		profile: ProfileRecord{Outer: LoopRecord{}},
		ax:      axisFrame{dU: 1},
	})
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 1, calls, `revolve carrier faces must pass the budget to meridian resolution`)
}

func TestAddRevolveFacesPreservesMeridianErrorMapping(t *testing.T) {
	for _, tc := range []struct {
		name string
		seg  CurveSegment
		want error
	}{
		{
			name: "unsupported curve",
			seg:  EllipseSeg{},
		},
		{
			name: "malformed circle",
			seg: CircleSeg{
				Radius: units.Millimeters(1),
				TStart: 0,
				TEnd:   1,
			},
			want: ErrDegenerate,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ok, err := (&bodyGeom{}).addRevolveFaces(newWorkBudget(t.Context()), revolvePayload{
				profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{tc.seg}}},
				ax:      axisFrame{dU: 1},
			})
			require.False(t, ok)
			if tc.want == nil {
				require.NoError(t, err)
				return
			}
			require.ErrorIs(t, err, tc.want)
		})
	}
}

// TestChordingRefusalsSplitFromOperandDegeneracy pins the §7.1 line the cap
// triangulator sits on. Both refusals below are ErrDegenerate to a public
// Tessellate caller; they differ in whether a finer chording could ever change
// the answer, which is what decides an undecided pair from a returned error.
func TestChordingRefusalsSplitFromOperandDegeneracy(t *testing.T) {
	t.Run("stalled ear clip is expected", func(t *testing.T) {
		// A self-crossing chorded boundary: no corner is ever clippable.
		pts := []Point2{{U: 0, V: 0}, {U: 10, V: 10}, {U: 10, V: 0}, {U: 0, V: 10}}

		_, err := earClip(t.Context(), pts, []int{0, 1, 2, 3})
		require.ErrorIs(t, err, ErrDegenerate,
			`public Tessellate must still see ErrDegenerate through Unwrap`)
		var coarse *tessellationExpectedError
		require.True(t, errors.As(err, &coarse),
			`a chording too coarse to prove the region is an expected undecided outcome, never an evaluator failure`)
	})

	t.Run("hole outside its outline is the operand's own", func(t *testing.T) {
		pts := []Point2{
			{U: 0, V: 0}, {U: 10, V: 0}, {U: 10, V: 10}, {U: 0, V: 10},
			{U: 50, V: 50}, {U: 52, V: 50}, {U: 51, V: 52},
		}

		_, err := bridgeHole(t.Context(), pts, []int{0, 1, 2, 3}, []int{4, 5, 6})
		require.ErrorIs(t, err, ErrDegenerate)
		var coarse *tessellationExpectedError
		require.False(t, errors.As(err, &coarse),
			`a hole outside its outline is geometry no tolerance changes, so Verify must return it rather than read Suspect`)
	})
}
