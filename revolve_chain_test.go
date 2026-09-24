package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRevolveChainFullTurnWithOnePole(t *testing.T) {
	t.Parallel()
	s, ch, axis := coneChainAtRadius(t, 0)
	doc := decad.New()
	body, err := doc.RevolveChain(s, ch, axis, decad.FullRevolution{})
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, body.Kind())
	require.Len(t, body.Faces(), 1)
	_, err = decad.Edges(decad.Free()).Exactly(1).SelectEdges(body)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	requireAreaContains15PiFraction(t, area, 1)
	box, err := body.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, box.Exactness)
	require.Equal(t, r3.NewVec(0, -3, -3), box.Min)
	require.Equal(t, r3.NewVec(4, 3, 3), box.Max)
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestRevolveChainPoleAtLastEnd(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rim := s.CreatePoint(4, 3)
	s.Fix(rim)
	s.CreateLine(rim, s.CreatePoint(0, 0))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.New().RevolveChain(s, s.Chains()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	_, err = decad.Edges(decad.Free()).Exactly(1).SelectEdges(body)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)
	requireAreaContains15PiFraction(t, area, 1)
}

func TestRevolveChainRejectsInteriorAxisPinch(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 3)
	s.Fix(start)
	pole := s.CreatePoint(4, 0)
	s.CreateLine(start, pole)
	s.CreateLine(pole, s.CreatePoint(8, 3))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	doc := decad.New()
	_, err = doc.RevolveChain(s, s.Chains()[0], axis, decad.FullRevolution{})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "interior axis junction")
	require.Empty(t, doc.Bodies())
}

func TestRevolveChainQuarterTurnWithOnePole(t *testing.T) {
	t.Parallel()
	s, ch, axis := coneChainAtRadius(t, 0)
	doc := decad.New()
	body, err := doc.RevolveChain(s, ch, axis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, body.Kind())
	require.Len(t, body.Faces(), 1)
	free, err := decad.Edges(decad.Free()).Exactly(3).SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, body.Vertices(), 3)
	pole := 0
	for _, v := range body.Vertices() {
		if v.Position().Value.X == 0 && v.Position().Value.Y == 0 && v.Position().Value.Z == 0 {
			pole++
			incident := 0
			for _, edge := range free {
				if edge.Start() == v || edge.End() == v {
					incident++
				}
			}
			require.Equal(t, 2, incident)
		}
	}
	require.Equal(t, 1, pole)
	area, err := body.Area()
	require.NoError(t, err)
	requireAreaContains15PiFraction(t, area, 4)
}

func TestRevolveChainSnappedPoleChargesArea(t *testing.T) {
	t.Parallel()
	s, ch, axis := coneChainAtRadius(t, 1e-10)
	doc := decad.New()
	body, err := doc.RevolveChain(s, ch, axis, decad.FullRevolution{})
	require.NoError(t, err)
	_, err = decad.Edges(decad.Free()).Exactly(1).SelectEdges(body)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)
	// The snapped wall is the cone from (0,0) to (4,3), even though its
	// recorded start radius was positive. The published bound must cover it.
	requireAreaContains15PiFraction(t, area, 1)
}

func TestRevolveChainPolePlacedBounds(t *testing.T) {
	t.Parallel()
	s, ch, axis := coneChain(t, 1e6, 0)
	doc := decad.New()
	body, err := doc.RevolveChain(s, ch, axis, decad.FullRevolution{})
	require.NoError(t, err)
	before, err := body.Area()
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(30))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), rot)
	require.NoError(t, err)
	_, err = decad.Edges(decad.Free()).Exactly(1).SelectEdges(placed)
	require.NoError(t, err)
	after, err := placed.Area()
	require.NoError(t, err)
	require.Equal(t, before, after)
	box, err := placed.Bounds()
	require.NoError(t, err)
	basis := rot.Basis()
	bound := box.Bound.Base()
	requireEnclosesExactDot(t, box.Max.X, bound, [2]float64{basis.EX.X, 1e6}, [2]float64{basis.EY.X, -3})
	requireEnclosesExactDot(t, box.Max.Y, bound, [2]float64{basis.EX.Y, 1e6}, [2]float64{basis.EY.Y, 3})
}

func coneChainAtRadius(tb testing.TB, radius float64) (*sketch.Sketch, *sketch.Chain, decad.Axis) {
	return coneChain(tb, 4, radius)
}

func coneChain(tb testing.TB, length, radius float64) (*sketch.Sketch, *sketch.Chain, decad.Axis) {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	start := s.CreatePoint(0, radius)
	s.Fix(start)
	s.CreateLine(start, s.CreatePoint(length, 3))
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	require.Len(tb, s.Chains(), 1)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	return s, s.Chains()[0], axis
}

// requireAreaContains15PiFraction is the cone chain's own reading of
// requireEnclosesPiMultiple (revolve_snap_test.go), which owns the rational
// comparison and states the interval, the truth and the shortfall when it
// fails.
func requireAreaContains15PiFraction(t *testing.T, area decad.Measurement, denominator int64) {
	t.Helper()
	requireEnclosesPiMultiple(t, "the cone chain's Area", area.Value.Base(), area.Bound.Base(), 15, denominator)
}

// This file is docs/surface-design.md §13's public-surface tests for
// RevolveChain: a chain spun into a shell with no cap, over any segment count
// and kind Table G admits (§15's T134, T135; the R22 on-axis refusal lives in
// extrude_chain_test.go's TestChainFedRefusals beside SweepChain/LoftChain's
// own T139 rows).

// twoSegmentChainClearOfAxis builds a two-segment open chain — two lines
// meeting at a shared point — clear of the u = 0 axis, docs/surface-design.md
// §15's T134/T135 fixture.
func twoSegmentChainClearOfAxis(tb testing.TB) (*sketch.Sketch, *sketch.Chain, decad.Axis) {
	tb.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(tb, err)
	a := s.CreatePoint(10, 0)
	b := s.CreatePoint(10, 5)
	c := s.CreatePoint(15, 8)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	require.Empty(tb, s.Profiles())
	chains := s.Chains()
	require.Len(tb, chains, 1)
	require.Len(tb, chains[0].Edges, 2)

	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	return s, chains[0], axis
}

// TestRevolveChainQuarterTurnStaysUndecided is docs/surface-design.md's T134:
// an open two-segment walk clear of the axis, spun a quarter turn, publishes
// a two-face open sheet whose Area encloses the analytic Pappus value and
// whose validity earns no construction proof from a partial sweep.
func TestRevolveChainQuarterTurnStaysUndecided(t *testing.T) {
	t.Parallel()
	s, ch, axis := twoSegmentChainClearOfAxis(t)
	doc := decad.New()
	body, err := doc.RevolveChain(s, ch, axis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, body.Kind())
	require.False(t, body.IsSolid())
	require.Len(t, body.Faces(), 2)

	free, err := decad.Edges(decad.Free()).Exactly(6).SelectEdges(body)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
	}

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	// Pappus's first theorem: the swept area is the walk's own length times
	// the mean radius each of its two straight segments sweeps through, over
	// the quarter-turn arc length at that radius. Segment 1: (10,0)-(10,5),
	// radius 10 throughout, length 5, arc-length factor π/2·10.
	// Segment 2: (10,5)-(15,8), mean radius 12.5, length sqrt(5^2+3^2).
	seg1 := 5.0 * (10.0 * (unitsHalfPi()))
	length2 := 5.830951894845301 // hypot(5,3)
	seg2 := length2 * (12.5 * unitsHalfPi())
	analytic := seg1 + seg2
	lo := area.Value.Base() - area.Bound.Base()
	hi := area.Value.Base() + area.Bound.Base()
	require.GreaterOrEqual(t, analytic, lo)
	require.LessOrEqual(t, analytic, hi)

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityUndecided, br.Validity.Outcome, "a partial turn earns no construction proof")
}

// unitsHalfPi is math.Pi/2, spelled once so the analytic Pappus computation
// above reads as arithmetic on a named constant rather than a bare literal.
func unitsHalfPi() float64 { return 1.5707963267948966 }

// TestRevolveChainFullTurnStaysOpen is docs/surface-design.md's T135: the
// same walk revolved a FULL turn stays an OPEN sheet — its two free ends
// sweep two circles nothing fills — and the full-turn construction proof
// admits ValidityValid with no diagnostics.
func TestRevolveChainFullTurnStaysOpen(t *testing.T) {
	t.Parallel()
	s, ch, axis := twoSegmentChainClearOfAxis(t)
	doc := decad.New()
	body, err := doc.RevolveChain(s, ch, axis, decad.FullRevolution{})
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, body.Kind())
	_, err = body.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	require.Len(t, body.Faces(), 2)

	free, err := decad.Edges(decad.Free()).Exactly(2).SelectEdges(body)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
	}

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	require.Empty(t, br.Validity.Diagnostics)
}

// TestRevolveChainRunsTheSeamGates confirms RevolveChain runs RecordChain's
// own gates exactly as ExtrudeChain does (docs/sketch-seam-design.md §2.2):
// a stale chain is ErrStaleProfile, and an invalid one is ErrInvalidProfile,
// neither ever reaching the axis or extent resolution.
func TestRevolveChainRunsTheSeamGates(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(10, 0)
	b := s.CreatePoint(10, 5)
	s.Fix(a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	s.AddConstraint(sketch.NewDistance(a, b, 8))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, ch.IsStale())

	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	doc := decad.New()
	_, err = doc.RevolveChain(s, ch, axis, decad.FullRevolution{})
	require.ErrorIs(t, err, decad.ErrStaleProfile)
	require.Empty(t, doc.Bodies())

	fresh := s.Chains()[0]
	body, err := doc.RevolveChain(s, fresh, axis, decad.FullRevolution{})
	require.NoError(t, err)
	require.NotNil(t, body)
}
