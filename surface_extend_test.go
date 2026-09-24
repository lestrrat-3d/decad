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
	untouched, err := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(0, 0, 0))).Exactly(1).SelectEdges(result)
	require.NoError(t, err)
	require.Equal(t, math.Float64bits(source.Start.U), math.Float64bits(untouched[0].Start().Position().Value.X))
	require.Equal(t, math.Float64bits(source.Start.V), math.Float64bits(untouched[0].Start().Position().Value.Y))
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
