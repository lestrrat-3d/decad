package decad_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func extendLeftRibbon(t *testing.T, doc *decad.Document) (*decad.Body, decad.LineSeg) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	s.Fix(a)
	line := s.CreateLine(a, s.CreatePoint(100, 0))
	c := s.CreatePoint(40, -10)
	s.Fix(c)
	s.CreateLine(c, s.CreatePoint(40, 10))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, chain := range s.Chains() {
		if len(chain.Edges) != 1 || chain.Edges[0].Entity != line || chain.Edges[0].TStart != 0 {
			continue
		}
		record, _, err := decad.RecordChain(s, chain)
		require.NoError(t, err)
		require.Len(t, record.Segments, 1)
		seg, ok := record.Segments[0].(decad.LineSeg)
		require.True(t, ok)
		require.Equal(t, 0.0, seg.TStart)
		require.InDelta(t, 0.4, seg.TEnd, 1e-12)
		body, err := doc.ExtrudeChain(s, chain, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		return body, seg
	}
	t.Fatal("sketch published no left fragment of the source line")
	return nil, decad.LineSeg{}
}

// extendReversedRibbon is extendLeftRibbon's mirror in ONE respect that
// decides which parameterisation the extension is resolved in: the line is
// authored (100, 0.1) -> (0, 0.1), so sketch's chain walk of the x >= 40
// fragment runs AGAINST that direction, publishes the fragment Reversed, and
// recordEdge (seam.go) stores it with its range order swapped — TStart 0.6 >
// TEnd 0. The ribbon covers x in [40, 100], and its free end at x = 40 is the
// recorded TStart. v = 0.1 is deliberate: an untouched bound at a coordinate
// whose float bits are not trivially zero is what makes a byte-identity
// assertion carry content.
func extendReversedRibbon(t *testing.T, doc *decad.Document) (*decad.Body, decad.LineSeg) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(100, 0.1)
	s.Fix(a)
	line := s.CreateLine(a, s.CreatePoint(0, 0.1))
	c := s.CreatePoint(40, -10)
	s.Fix(c)
	s.CreateLine(c, s.CreatePoint(40, 10))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, chain := range s.Chains() {
		if len(chain.Edges) != 1 || chain.Edges[0].Entity != line || !chain.Edges[0].Reversed {
			continue
		}
		record, _, err := decad.RecordChain(s, chain)
		require.NoError(t, err)
		require.Len(t, record.Segments, 1)
		seg, ok := record.Segments[0].(decad.LineSeg)
		require.True(t, ok)
		if seg.TEnd != 0 {
			continue // the other reversed fragment, anchored at the line's far end
		}
		require.Greater(t, seg.TStart, seg.TEnd, "the recorded range runs against the line's authored direction")
		require.InDelta(t, 0.6, seg.TStart, 1e-12)
		require.Equal(t, decad.Point2{U: 100, V: 0.1}, seg.Start)
		require.Equal(t, decad.Point2{U: 0, V: 0.1}, seg.End)
		body, err := doc.ExtrudeChain(s, chain, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		return body, seg
	}
	t.Fatal("sketch published no reversed fragment of the source line")
	return nil, decad.LineSeg{}
}

// TestSurfaceExtendReversedFragmentToNearestCut is T178's reading over a
// receiver whose recorded range runs against its line's authored direction.
// The two arms are the two ways the parameterisations can be confused: a tool
// lying entirely INSIDE the ribbon names no crossing past the receiver's own
// end and must be RS4, and a tool genuinely past that end must lengthen the
// ribbon to reach it.
func TestSurfaceExtendReversedFragmentToNearestCut(t *testing.T) {
	t.Parallel()
	query := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 0.1, 0))).Exactly(1)

	insideDoc := decad.New()
	insideRibbon, _ := extendReversedRibbon(t, insideDoc)
	beforeBounds, err := insideRibbon.Bounds()
	require.NoError(t, err)
	beforeArea, err := insideRibbon.Area()
	require.NoError(t, err)
	require.InDelta(t, 40, beforeBounds.Min.X, 1e-12)
	require.InDelta(t, 100, beforeBounds.Max.X, 1e-12)
	require.InDelta(t, 600, beforeArea.Value.Base(), 1e-9)
	t.Logf("before: bounds x [%g, %g], area %g mm²", beforeBounds.Min.X, beforeBounds.Max.X, beforeArea.Value.Base())

	// x in [70, 90] lies inside the ribbon's own x in [40, 100]; read in the
	// scene's parameter order those crossings sit PAST the named end, so an
	// unmapped reading publishes a boundary at x = 30 where the tool is not.
	insideTool := trimSpanningTool(t, insideDoc, 70, -10, 90, 10)
	before := insideDoc.Bodies()
	_, err = insideRibbon.Extend(t.Context(), query, insideTool)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "carrier's own natural domain")
	require.Equal(t, before, insideDoc.Bodies())

	doc := decad.New()
	ribbon, source := extendReversedRibbon(t, doc)
	// x in [10, 30] is genuinely past the named x = 40 end; the nearest of its
	// two crossings is x = 30.
	tool := trimSpanningTool(t, doc, 10, -10, 30, 10)
	result, err := ribbon.Extend(t.Context(), query, tool)
	require.NoError(t, err)

	area, err := result.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	// With the cut displacement omitted this fixture retains 1.421085471520203e-13 mm²
	// of arithmetic bound; the charged result measures 6.710918370891792e-11 mm².
	require.Greater(t, area.Bound.Base(), 1e-11,
		"extended area bound must include the cut parameter displacement")
	require.LessOrEqual(t, math.Abs(area.Value.Base()-700), area.Bound.Base(),
		"the area interval must enclose the 700 mm² ribbon")
	bounds, err := result.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 30, bounds.Min.X, bounds.Bound.Base())
	require.InDelta(t, 100, bounds.Max.X, bounds.Bound.Base())
	// Without trimBoundsWalks's per-component cut charge this bounds interval
	// measures 7.105427357601008e-15 mm; with it, 1.8474111129762615e-13 mm.
	require.Greater(t, bounds.Bound.Base(), 5e-14,
		"extended bounds interval must include the cut endpoint displacement")

	// The x = 100 end is the one the extension never named. Its vertex is
	// selected by elimination against the end that DID move, so a nudge of the
	// untouched bound reaches the byte comparison instead of being filtered out
	// by the selector before it.
	sweeps, moved := extendFreeSweepEdges(t, result, 30)
	untouched := sweeps[1-moved]
	require.Equal(t, math.Float64bits(source.Start.U), math.Float64bits(untouched.Start().Position().Value.X))
	require.Equal(t, math.Float64bits(source.Start.V), math.Float64bits(untouched.Start().Position().Value.Y))
	t.Logf("after: bounds x [%g, %g] ± %g mm, area %g ± %g mm²; untouched end x = %v",
		bounds.Min.X, bounds.Max.X, bounds.Bound.Base(), area.Value.Base(), area.Bound.Base(),
		untouched.Start().Position().Value.X)
}

// extendArcCutT is the parameter of (60, 80) on the half-circle of radius 100
// about the origin running counter-clockwise from (100, 0) to (-100, 0): the
// point is exact (a 3-4-5 triangle) and its fraction of the pi sweep is
// atan2(80, 60)/pi.
var extendArcCutT = math.Atan2(80, 60) / math.Pi

// extendArcRibbon builds the ArcSeg receiver: that same half-circle, cut at
// (60, 80) by a vertical line at x = 60. sketch walks the published fragment
// against the arc's own counter-clockwise sense, so the record runs
// TStart > TEnd and the ribbon covers (100, 0) to (60, 80).
// extendFreeSweepEdges returns a ribbon's two free sweep edges, and the index
// of the one whose start vertex stands at x. A cut end of a circular carrier
// has no exact coordinate to name in an EndpointAt clause — it is reached
// through cos/sin — so a fixture that must name one reads the body's own
// published vertex instead of pinning a literal.
func extendFreeSweepEdges(t *testing.T, b *decad.Body, x float64) ([]*decad.Edge, int) {
	t.Helper()
	edges, err := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(2).SelectEdges(b)
	require.NoError(t, err)
	at := -1
	for i, e := range edges {
		if math.Abs(e.Start().Position().Value.X-x) > 1e-9 {
			continue
		}
		require.Equal(t, -1, at, "two free sweep edges stand at x = %v", x)
		at = i
	}
	require.NotEqual(t, -1, at, "no free sweep edge stands at x = %v", x)
	return edges, at
}

func extendArcRibbon(t *testing.T, doc *decad.Document) (*decad.Body, decad.ArcSeg) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	start := s.CreatePoint(100, 0)
	s.Fix(start)
	end := s.CreatePoint(-100, 0)
	s.Fix(end)
	arc := s.CreateArc(centre, start, end)
	c := s.CreatePoint(60, -10)
	s.Fix(c)
	d := s.CreatePoint(60, 110)
	s.Fix(d)
	s.CreateLine(c, d)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, chain := range s.Chains() {
		if len(chain.Edges) != 1 || chain.Edges[0].Entity != arc {
			continue
		}
		record, _, err := decad.RecordChain(s, chain)
		require.NoError(t, err)
		require.Len(t, record.Segments, 1)
		seg, ok := record.Segments[0].(decad.ArcSeg)
		require.True(t, ok)
		if seg.TEnd != 0 {
			continue // the (60, 80) to (-100, 0) fragment
		}
		require.Greater(t, seg.TStart, seg.TEnd, "the recorded range runs against the arc's authored sense")
		require.InDelta(t, extendArcCutT, seg.TStart, 1e-12)
		require.Equal(t, decad.Point2{U: 100, V: 0}, seg.Start)
		require.Equal(t, decad.Point2{U: -100, V: 0}, seg.End)
		body, err := doc.ExtrudeChain(s, chain, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		return body, seg
	}
	t.Fatal("sketch published no arc fragment anchored at the arc's own start")
	return nil, decad.ArcSeg{}
}

// TestSurfaceExtendArcReceiverToNearestCut covers fullExtendSegment's and
// extendSetBound's ArcSeg arms through the public API. The arc is recorded in
// the reversed sense, so it also confirms the claim its own derivation rests
// on: an arc's scene entity is CreateArc(centre, Start, End) either way, so no
// map runs and the extension lands where the tool actually is.
func TestSurfaceExtendArcReceiverToNearestCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ribbon, source := extendArcRibbon(t, doc)
	beforeBounds, err := ribbon.Bounds()
	require.NoError(t, err)
	beforeArea, err := ribbon.Area()
	require.NoError(t, err)
	require.InDelta(t, 60, beforeBounds.Min.X, 1e-9)
	require.InDelta(t, 100, beforeBounds.Max.X, 1e-9)
	require.InDelta(t, 100*extendArcCutT*math.Pi*10, beforeArea.Value.Base(), 1e-6)
	t.Logf("before: bounds x [%g, %g], area %g mm²", beforeBounds.Min.X, beforeBounds.Max.X, beforeArea.Value.Base())

	// The tool's two vertical edges meet the arc at (-60, 80) and (-80, 60) —
	// both 3-4-5 points, so both are exact. The nearest past the receiver's own
	// end is (-60, 80), at t = atan2(80, -60)/pi.
	tool := trimSpanningTool(t, doc, -80, -10, -60, 110)
	sweeps, cut := extendFreeSweepEdges(t, ribbon, 60)
	query := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(sweeps[cut].Start().Position().Value)).Exactly(1)
	result, err := ribbon.Extend(t.Context(), query, tool)
	require.NoError(t, err)

	wantT := math.Atan2(80, -60) / math.Pi
	wantArea := 100 * wantT * math.Pi * 10
	area, err := result.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	// With the cut displacement omitted this fixture retains 4.42545691493224e-13 mm²
	// of arithmetic bound, which the 4.547473508864641e-13 mm² residual against the
	// analytic area already escapes; the charged result measures 4.212090884126917e-10 mm².
	require.Greater(t, area.Bound.Base(), 1e-11,
		"extended area bound must include the cut parameter displacement")
	require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base(),
		"the area interval must enclose the extended quarter-plus arc ribbon")
	bounds, err := result.Bounds()
	require.NoError(t, err)
	decadtest.Encloses(t, "extended arc bounds", bounds, r3.NewVec(-60, 80, 10))
	require.InDelta(t, -60, bounds.Min.X, bounds.Bound.Base())
	require.InDelta(t, 100, bounds.Max.Y, bounds.Bound.Base())
	// Without trimBoundsWalks's per-component cut charge this bounds interval
	// measures 2.842170943040403e-14 mm; with it, 1.1439738045737621e-12 mm.
	require.Greater(t, bounds.Bound.Base(), 1e-13,
		"extended arc bounds interval must include the cut endpoint displacement")

	// The (100, 0) end is the arc's own natural bound and was never named. It is
	// found by elimination against the end that DID move, so a nudge of the
	// untouched bound reaches the byte comparison rather than being filtered out
	// by a selector keyed on it.
	after, moved := extendFreeSweepEdges(t, result, -60)
	untouched := after[1-moved]
	require.InDelta(t, 80, after[moved].Start().Position().Value.Y, 1e-9, "the moved end stands where the tool is")
	require.Equal(t, math.Float64bits(source.Start.U), math.Float64bits(untouched.Start().Position().Value.X))
	require.Equal(t, math.Float64bits(source.Start.V), math.Float64bits(untouched.Start().Position().Value.Y))
	t.Logf("after: bounds x [%g, %g] ± %g mm, area %g ± %g mm² (want %g)",
		bounds.Min.X, bounds.Max.X, bounds.Bound.Base(), area.Value.Base(), area.Bound.Base(), wantArea)
}

func TestSurfaceExtendLeftFragmentToNearestCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ribbon, source := extendLeftRibbon(t, doc)
	tool := trimSpanningTool(t, doc, 70, -10, 90, 10)
	query := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
	selected, err := query.SelectEdges(ribbon)
	require.NoError(t, err)
	require.Len(t, selected, 1)
	result, err := ribbon.Extend(t.Context(), query, tool)
	require.NoError(t, err)
	area, err := result.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	// With the cut displacement omitted, this fixture retains only
	// 1.421085471520203e-13 mm² of arithmetic bound. The charged result
	// measures 6.710918370891792e-11 mm².
	require.Greater(t, area.Bound.Base(), 1e-11,
		"extended area bound must include the cut parameter displacement")
	require.LessOrEqual(t, math.Abs(area.Value.Base()-700), area.Bound.Base(),
		"the area interval must enclose the 700 mm² ribbon")
	bounds, err := result.Bounds()
	require.NoError(t, err)
	decadtest.Encloses(t, "extended bounds", bounds, r3.NewVec(70, 0, 10))
	require.InDelta(t, 70, bounds.Max.X, bounds.Bound.Base())
	// Without trimBoundsWalks's per-component cut charge, this bounds
	// interval measures 1.4210854715202016e-14 mm; with it, 1.9895196601282815e-13 mm.
	require.Greater(t, bounds.Bound.Base(), 5e-14,
		"extended bounds interval must include the cut endpoint displacement")
	// T178's byte-identity obligation is about the bound the extension never
	// named, so the edge carrying it is found by elimination against the end
	// that DID move — never by an EndpointAt clause keyed on the untouched
	// coordinate, which would filter a nudged bound out before the comparison
	// and leave the comparison proving nothing.
	sweeps, moved := extendFreeSweepEdges(t, result, 70)
	untouched := sweeps[1-moved]
	require.Equal(t, math.Float64bits(source.Start.U), math.Float64bits(untouched.Start().Position().Value.X))
	require.Equal(t, math.Float64bits(source.Start.V), math.Float64bits(untouched.Start().Position().Value.Y))
	t.Logf("area %g ± %g mm²; bounds max x %g ± %g mm", area.Value.Base(), area.Bound.Base(),
		bounds.Max.X, bounds.Bound.Base())
}

func TestSurfaceExtendRefusesBeyondCarrierDomain(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ribbon, _ := extendLeftRibbon(t, doc)
	tool := trimSpanningTool(t, doc, 110, -10, 130, 10)
	query := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
	before := doc.Bodies()
	_, err := ribbon.Extend(t.Context(), query, tool)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "carrier's own natural domain")
	require.Equal(t, before, doc.Bodies())

	insideDoc := decad.New()
	insideRibbon, _ := extendLeftRibbon(t, insideDoc)
	insideTool := trimSpanningTool(t, insideDoc, 70, -10, 90, 10)
	inside, err := insideRibbon.Extend(t.Context(), query, insideTool)
	require.NoError(t, err)
	area, err := inside.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-700), area.Bound.Base(),
		"the inside-domain tool must lengthen the same ribbon to 700 mm²")
}

func TestSurfaceExtendRequiresNamedEnd(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ribbon, _ := extendLeftRibbon(t, doc)
	tool := trimSpanningTool(t, doc, 70, -10, 90, 10)
	_, err := ribbon.Extend(t.Context(), decad.Edges(decad.EndpointAt(r3.NewVec(41, 0, 0))), tool)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	var selection *decad.SelectionError
	require.True(t, errors.As(err, &selection))
	require.Equal(t, 0, selection.Actual)
	require.Equal(t, "edges(endpoint_at(41,0,0))", selection.Query)
	_, err = decad.Edges(decad.EndpointAt(r3.NewVec(math.NaN(), 0, 0))).SelectEdges(ribbon)
	require.ErrorIs(t, err, decad.ErrNotFinite)

	_, err = ribbon.Extend(t.Context(), decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(0, 0, 0))).Exactly(1), tool)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "already reaches its carrier's own domain")

	closedDoc := decad.New()
	closed := trimRectSheet(t, closedDoc)
	closedTool := trimSpanningTool(t, closedDoc, 70, -10, 90, 70)
	_, err = closed.Extend(t.Context(), decad.Edges(decad.Free()).Exactly(8), closedTool)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "open section")
}
