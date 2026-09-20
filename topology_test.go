package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func coedgeBox(t *testing.T) *decad.Body {
	t.Helper()
	s, p := plateSketch(t)
	body, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestLoopCoEdgesMatchEdgesCompatibilityView(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)

	for _, face := range body.Faces() {
		for _, loop := range face.Loops() {
			directed := loop.CoEdges()
			undirected := loop.Edges()
			require.Len(t, directed, len(undirected))
			require.NotEmpty(t, directed)
			for i, use := range directed {
				require.Same(t, undirected[i], use.Edge())
			}
		}
	}
}

func TestLoopCoEdgesFollowDirectedBoundary(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)
	uses := make(map[*decad.Edge][]decad.CoEdge)
	sawReverse := false

	for _, face := range body.Faces() {
		for _, loop := range face.Loops() {
			directed := loop.CoEdges()
			require.NotEmpty(t, directed)
			for i, use := range directed {
				require.Same(t, use.End(), directed[(i+1)%len(directed)].Start())
				if use.IsForward() {
					require.Same(t, use.Edge().Start(), use.Start())
					require.Same(t, use.Edge().End(), use.End())
				} else {
					sawReverse = true
					require.Same(t, use.Edge().End(), use.Start())
					require.Same(t, use.Edge().Start(), use.End())
				}
				uses[use.Edge()] = append(uses[use.Edge()], use)
			}
		}
	}

	require.True(t, sawReverse, `the box exposes edge uses that oppose global edge orientation`)
	require.Len(t, uses, len(body.Edges()))
	for _, edge := range body.Edges() {
		require.Len(t, uses[edge], 2, `each manifold edge has two loop uses`)
	}
}

func TestLoopCoEdgesReturnImmutableView(t *testing.T) {
	t.Parallel()
	loop := coedgeBox(t).Faces()[0].Loops()[0]
	directed := loop.CoEdges()
	undirected := loop.Edges()
	require.NotEmpty(t, directed)

	wantEdge := directed[0].Edge()
	wantStart := directed[0].Start()
	wantEnd := directed[0].End()
	wantForward := directed[0].IsForward()
	directed[0] = decad.CoEdge{}
	undirected[0] = nil

	got := loop.CoEdges()[0]
	require.Same(t, wantEdge, got.Edge())
	require.Same(t, wantStart, got.Start())
	require.Same(t, wantEnd, got.End())
	require.Equal(t, wantForward, got.IsForward())
	require.Same(t, wantEdge, loop.Edges()[0])
}

// TestNURBSSurfaceReportsKindNURBS pins docs/spline-design.md §7: a
// NURBSSurface is a tagged, opaque Surface variant reporting the existing
// KindNURBS discriminant, carrying no exported geometry of its own.
func TestNURBSSurfaceReportsKindNURBS(t *testing.T) {
	t.Parallel()
	var s decad.Surface = decad.NURBSSurface{}
	require.Equal(t, decad.KindNURBS, s.Kind())
}

// TestNURBSCurveSealsIntoCurve pins docs/spline-design.md §7: a NURBSCurve is
// NURBSSurface's one-dimensional analog. Curve is sealed by its marker method
// alone and declares no Kind, so assigning a NURBSCurve to a Curve variable
// is the whole of what the variant publishes.
func TestNURBSCurveSealsIntoCurve(t *testing.T) {
	t.Parallel()
	var c decad.Curve = decad.NURBSCurve{}
	require.NotNil(t, c)
}

// requireSolidTopologyBaseline asserts the sheet-body-vocabulary readings
// that hold on every solid this evaluator builds today (docs/surface-design.md
// §2.1, §2.2): the body reports BodySolid without any builder ever writing
// `kind: BodySolid` — the seven &Body{...} literals rely on BodySolid being
// the iota zero — every edge bounds exactly two faces and none is free, and
// the body's single shell is neither open nor void.
func requireSolidTopologyBaseline(t *testing.T, body *decad.Body, wantFaces, wantEdges int) {
	t.Helper()
	require.Equal(t, decad.BodySolid, body.Kind())
	require.True(t, body.IsSolid())
	require.Len(t, body.Faces(), wantFaces)
	edges := body.Edges()
	require.Len(t, edges, wantEdges)
	for _, e := range edges {
		require.Len(t, e.Faces(), 2, `a solid's edge bounds exactly two faces`)
		require.False(t, e.IsFree(), `a solid has no free edge`)
	}
	require.Len(t, body.Lumps(), 1)
	shells := body.Shells()
	require.Len(t, shells, 1)
	require.False(t, shells[0].IsOpen(), `a solid shell has no free edge to be open over`)
	require.False(t, shells[0].IsVoid(), `an outer shell bounds no internal cavity`)
}

// TestSolidBodiesReportSolidKindAndNoFreeBoundary pins docs/surface-design.md
// §2.1's Table K first row and §2.2's Edge.IsFree/Shell.IsOpen readings
// against three solids the evaluator already builds: a box extrude, a holed
// extrude and a full-revolution annulus. None of today's builders produces a
// free edge or an open shell, so every one of these reads false — the
// negative space Table K and §2.2 both require before a later PR's
// WithSurfaceResult can ever make it read true.
func TestSolidBodiesReportSolidKindAndNoFreeBoundary(t *testing.T) {
	t.Parallel()

	t.Run("CoedgeBox", func(t *testing.T) {
		t.Parallel()
		body := coedgeBox(t)
		requireSolidTopologyBaseline(t, body, 6, 12)

		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, vol.Exactness)
		gotVol, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.Equal(t, 60000.0, gotVol)

		box, err := body.Bounds()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, box.Exactness)
		require.Equal(t, r3.NewVec(0, 0, 0), box.Min)
		require.Equal(t, r3.NewVec(100, 60, 10), box.Max)
	})

	t.Run("HoledExtrude", func(t *testing.T) {
		t.Parallel()
		body := holePlateBody(t)
		requireSolidTopologyBaseline(t, body, 7, 14)

		// A 100x60x8 plate less a diameter-20 through hole: the circle's
		// area is irrational, so the volume is Approximate, never Exact.
		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, vol.Exactness)
		gotVol, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.InDelta(t, (100*60-math.Pi*100)*8, gotVol, 1e-6)

		box, err := body.Bounds()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, box.Exactness, `the plate's box is the extrude's own recorded extents`)
		require.Equal(t, r3.NewVec(0, 0, 0), box.Min)
		require.Equal(t, r3.NewVec(100, 60, 8), box.Max)
	})

	t.Run("FullRevolve", func(t *testing.T) {
		t.Parallel()
		body := annularRevolveBody(t)
		requireSolidTopologyBaseline(t, body, 4, 4)

		// A full revolution of the v-in-[5,15], u-in-[0,10] rectangle about
		// the u axis: a tube of outer radius 15, inner radius 5, height 10.
		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, vol.Exactness)
		gotVol, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.InDelta(t, math.Pi*(15*15-5*5)*10, gotVol, 1e-6)

		box, err := body.Bounds()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, box.Exactness)
		require.Equal(t, r3.NewVec(0, -15, -15), box.Min)
		require.Equal(t, r3.NewVec(10, 15, 15), box.Max)
	})
}
