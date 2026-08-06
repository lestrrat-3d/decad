package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file asserts docs/spline-design.md §6.2's directional-extreme row: the
// bracket must ENCLOSE a dense-sample reference (never understate a Box), it
// must recognize the exact cases a free-form section shares with an analytic
// one (a zero-width enclosure at an endpoint, a collapsed span), it must
// respect the §5.2 work ceiling, and R11's build-time gate must refuse
// exactly where the bracket carries a nonzero bound while leaving every
// analytic reading untouched.

// ratSpan builds a bezierSpan directly from plane-local coordinates, for
// tests that exercise the bracket machinery itself rather than the record
// conversion spline_bezier.go already owns and tests.
func ratSpan(uv [][2]float64) bezierSpan {
	span := make(bezierSpan, len(uv))
	for i, p := range uv {
		span[i] = ratPoint{u: mustRatOf(p[0]), v: mustRatOf(p[1])}
	}
	return span
}

// denseSpanExtreme samples g(u, v) = gu*u + gv*v densely over one span,
// evaluated by an independent de Casteljau (evalSpans, spline_bezier_internal_test.go)
// rather than through any of the bracket's own machinery — the falsifier a
// bracket that understated its enclosure could not survive.
func denseSpanExtreme(t *testing.T, spans []bezierSpan, gu, gv float64, samples int) (lo, hi float64) {
	t.Helper()
	lo, hi = math.Inf(1), math.Inf(-1)
	for i := 0; i <= samples; i++ {
		at := float64(i) / float64(samples)
		u, v := evalSpans(t, spans, at)
		g := gu*u + gv*v
		lo = math.Min(lo, g)
		hi = math.Max(hi, g)
	}
	return lo, hi
}

// splineProfile wraps a single cubic SplineSeg as an (open) one-segment
// profile — boundaryExtremesBoundedContext reads segment geometry alone and
// needs no closed loop.
func splineProfile(control []Point2) ProfileRecord {
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		SplineSeg{Control: control, TStart: 0, TEnd: 1},
	}}}
}

func identityFrame(t *testing.T) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return frame
}

// 1. A cubic hump whose directional maximum sits at an interior parameter:
// the enclosure contains a dense-sample maximum, and the reported extreme is
// strictly above what a candidate set built from the two endpoints alone
// would report — an endpoint-only reading understates the Box, the unsound
// direction §6.2 names.
func TestBoundaryExtremesBoundedInteriorMaximumBeatsEndpointOnly(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 3}, {U: 3, V: 1}, {U: 4, V: 0}}
	profile := splineProfile(control)

	lo, hi, bound, err := boundaryExtremesBoundedContext(t.Context(), profile, 0, 1, newFreeformWork())
	require.NoError(t, err)
	require.Positive(t, bound, "a free-form directional extreme is never claimed exact")
	require.Less(t, lo, hi)

	spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, newFreeformWork())
	require.NoError(t, err)
	_, denseHi := denseSpanExtreme(t, spans, 0, 1, 20_000)
	require.LessOrEqual(t, denseHi, hi+bound, "the enclosure's upper end must not fall below a dense sample")
	require.GreaterOrEqual(t, denseHi, hi-bound, "the dense sample must land inside the reported enclosure")

	endpointOnly := math.Max(control[0].V, control[len(control)-1].V)
	require.Greater(t, hi, endpointOnly,
		"the interior candidate must move the reported maximum strictly above the endpoint-only reading")
}

// 2. The same span read along the perpendicular direction, whose extreme is
// held by a span endpoint: the bound is exactly zero and the reported value
// is the recorded control coordinate. Every control-U difference here is
// positive, so U is monotone and P' has no root in [0, 1] at all — the
// candidate set is the two exact endpoints and nothing else.
func TestBoundaryExtremesBoundedEndpointExtremeIsExact(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 3}, {U: 3, V: 1}, {U: 4, V: 0}}
	profile := splineProfile(control)

	lo, hi, bound, err := boundaryExtremesBoundedContext(t.Context(), profile, 1, 0, newFreeformWork())
	require.NoError(t, err)
	require.Zero(t, bound, "an extreme held by a recorded control coordinate carries no bound")
	require.Equal(t, control[0].U, lo, "the minimum is the first control point's own U")
	require.Equal(t, control[len(control)-1].U, hi, "the maximum is the last control point's own U")
}

// 3. A symmetric quadratic hump with its stationary point exactly at
// t = 1/2: bernsteinRestrict must handle a rational parameter sitting on a
// bisection boundary without dividing by a zero span width, and the span's
// own enclosure must still contain the exact interior maximum.
func TestSpanExtremeEnclosureHandlesRootAtOneHalf(t *testing.T) {
	// P(t) = Bernstein([0, 2, 0]) = 4t(1-t): P'(t) = 4 - 8t has its only root
	// at the exact rational t = 1/2, where P(1/2) = 1.
	span := ratSpan([][2]float64{{0, 0}, {2, 0}, {0, 0}})
	gu, gv := mustRatOf(1), mustRatOf(0)

	half := mustRatOf(0.5)
	restricted := bernsteinRestrict(spanDirectionalValues(span, gu, gv), half, half)
	lo, hi := bernsteinHull(restricted)
	one := mustRatOf(1)
	require.Zero(t, lo.Cmp(one), "restricting to the zero-width interval [1/2, 1/2] evaluates P there exactly")
	require.Zero(t, hi.Cmp(one))

	minIv, maxIv, err := spanExtremeEnclosureContext(t.Context(), span, gu, gv, newFreeformWork())
	require.NoError(t, err)
	maxLo, _ := maxIv.lo.Float64()
	maxHi, _ := maxIv.hi.Float64()
	require.LessOrEqual(t, maxLo, 1.0, "the enclosure must contain the true maximum 1")
	require.GreaterOrEqual(t, maxHi, 1.0)
	minLo, _ := minIv.lo.Float64()
	minHi, _ := minIv.hi.Float64()
	require.LessOrEqual(t, minLo, 0.0, "the minimum is held at both endpoints, value 0")
	require.GreaterOrEqual(t, minHi, 0.0)
}

// 4. A collapsed span (every control point coincident): the stationarity
// polynomial is identically zero, rpIsolateRootsContext returns no interval
// (§6.2: a zero root count is never on its own the proof of anything, but the
// endpoint candidates carry the constant either way), and both endpoints
// report the same constant with a zero-width enclosure.
func TestSpanExtremeEnclosureCollapsedSpanIsExact(t *testing.T) {
	span := ratSpan([][2]float64{{5, -1}, {5, -1}, {5, -1}, {5, -1}})
	gu, gv := mustRatOf(1), mustRatOf(1)

	minIv, maxIv, err := spanExtremeEnclosureContext(t.Context(), span, gu, gv, newFreeformWork())
	require.NoError(t, err)
	require.Zero(t, minIv.lo.Cmp(minIv.hi), "a collapsed span's minimum has zero width")
	require.Zero(t, maxIv.lo.Cmp(maxIv.hi), "a collapsed span's maximum has zero width")
	four := mustRatOf(4)
	require.Zero(t, minIv.lo.Cmp(four), "the constant value is 5*1 + (-1)*1 = 4")
	require.Zero(t, maxIv.lo.Cmp(four))
}

// 5. A NURBSSeg with a repeated interior knot: the interior knot 0.5 repeats
// to multiplicity degree (2) with three coincident control points at the
// join, so the converted chain holds one real quadratic span and one
// collapsed ("empty") span carrying no interior candidate. The region-level
// fold must still enclose a dense-sample reference over the WHOLE walk.
func TestBoundaryExtremesBoundedRepeatedInteriorKnot(t *testing.T) {
	seg := NURBSSeg{
		Degree: 2,
		Control: []Point2{
			{U: 0, V: 0}, {U: 1, V: 1}, {U: 2, V: 0}, {U: 2, V: 0}, {U: 2, V: 0},
		},
		Knots:   []float64{0, 0, 0, 0.5, 0.5, 1, 1, 1},
		Weights: []float64{1, 1, 1, 1, 1},
		TStart:  0, TEnd: 1,
	}
	require.NoError(t, validateNURBSSegment(seg))
	spans, _, err := freeformBezierSpans(seg, newFreeformWork())
	require.NoError(t, err)
	require.Len(t, spans, 2, "the repeated interior knot splits the chain into two spans")
	// The second span's three control points coincide (compared by rational
	// VALUE, never by *big.Rat pointer identity): it is empty of any real
	// curve, carrying no interior candidate of its own.
	collapsed := spans[1]
	require.Zero(t, collapsed[0].u.Cmp(collapsed[1].u))
	require.Zero(t, collapsed[0].v.Cmp(collapsed[1].v))
	require.Zero(t, collapsed[1].u.Cmp(collapsed[2].u))
	require.Zero(t, collapsed[1].v.Cmp(collapsed[2].v))

	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{seg}}}
	gu, gv := 1.0, 2.0
	lo, hi, bound, err := boundaryExtremesBoundedContext(t.Context(), profile, gu, gv, newFreeformWork())
	require.NoError(t, err)

	denseLo, denseHi := denseSpanExtreme(t, spans, gu, gv, 20_000)
	require.LessOrEqual(t, denseLo, hi+bound)
	require.GreaterOrEqual(t, denseLo, lo-bound, "the dense minimum must land inside the reported enclosure")
	require.LessOrEqual(t, denseHi, hi+bound, "the dense maximum must land inside the reported enclosure")
	require.GreaterOrEqual(t, denseHi, lo-bound)
}

// 6. R7: a freeformWork pre-drained to just under the ceiling refuses
// ErrUnsupported from spanExtremeEnclosureContext itself, before a single
// Bernstein coefficient is built — the charge is the function's first
// statement.
func TestSpanExtremeEnclosureRefusesOverBudget(t *testing.T) {
	span := ratSpan([][2]float64{{0, 0}, {2, 0}, {0, 0}})
	cost := freeformExtremeCost(len(span))
	require.Positive(t, cost, "the fixture span must actually cost something to charge against")
	work := &freeformWork{spent: freeformWorkLimit - cost/2}

	_, _, err := spanExtremeEnclosureContext(t.Context(), span, mustRatOf(1), mustRatOf(0), work)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
}

// 7. A non-finite gu/gv refuses ErrNotFinite ahead of any rational lift — the
// profile below carries no free-form segment at all, so the only possible
// source of the refusal is the direction components themselves, checked
// before the segment scan even starts.
func TestBoundaryExtremesBoundedNonFiniteDirectionRefuses(t *testing.T) {
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}, TStart: 0, TEnd: 1},
	}}}
	for _, tc := range []struct {
		name   string
		gu, gv float64
	}{
		{"NaN gu", math.NaN(), 0},
		{"Inf gv", 0, math.Inf(1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, _, _, err := boundaryExtremesBoundedContext(t.Context(), profile, tc.gu, tc.gv, newFreeformWork())
			require.Error(t, err)
			require.ErrorIs(t, err, ErrNotFinite)
		})
	}
}

// 8. R11: prismPayload.extentAlongWork over a free-form section refuses
// ErrUnsupported (a through-all stop reads this coordinate as exact and has
// no bound to widen), while prismBoundsContext over the identical payload
// answers with Approximate and a positive Bound instead of refusing — a Box
// has one to widen into.
func TestPrismExtentAlongWorkRefusesFreeformBoxAnswersApproximate(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}, {U: 6, V: 1}, {U: 7, V: -2}}
	pp := prismPayload{
		profile: splineProfile(control),
		frame:   identityFrame(t),
		z0:      0, z1: 5,
		xform: r3.Identity(),
	}

	// The Y axis: this fixture's control U values are strictly increasing
	// (monotone, so extentAlongWork along X would answer exactly), but its V
	// values rise and fall — a genuine interior extreme.
	_, _, err := pp.extentAlongWork(t.Context(), r3.NewVec(0, 1, 0), newFreeformWork())
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)

	box, err := prismBoundsContext(t.Context(), pp, newFreeformWork())
	require.NoError(t, err)
	require.Equal(t, Approximate, box.Exactness)
	require.Positive(t, box.Bound.Base())
}

// 9. Regression: boundaryExtremesContext still refuses a free-form section —
// the exact protection revolve.go's resolveAxisSide and capblend.go's
// extentBoundedAlong both rely on running BEFORE their own free-form gates —
// and every existing analytic prism's Box still reports Exact with a zero
// bound.
func TestBoundaryExtremesContextRegression(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}, {U: 6, V: 1}, {U: 7, V: -2}}
	// V (not U) has an interior extreme on this fixture (see the comment on
	// TestPrismExtentAlongWorkRefusesFreeformBoxAnswersApproximate).
	_, _, err := boundaryExtremesContext(t.Context(), splineProfile(control), 0, 1, newFreeformWork())
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)

	analytic := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 0}, End: Point2{U: 10, V: 10}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 10}, End: Point2{U: 0, V: 10}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 10}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	pp := prismPayload{profile: analytic, frame: identityFrame(t), z0: 0, z1: 5, xform: r3.Identity()}
	box, err := prismBoundsContext(t.Context(), pp, newFreeformWork())
	require.NoError(t, err)
	require.Equal(t, Exact, box.Exactness)
	require.Zero(t, box.Bound.Base())
}

// 10. Cancellation: a cancelled context returns ctx.Err() from the scan.
func TestBoundaryExtremesBoundedContextCancellation(t *testing.T) {
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}, TStart: 0, TEnd: 1},
	}}}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	lo, hi, bound, err := boundaryExtremesBoundedContext(ctx, profile, 1, 0, newFreeformWork())
	require.ErrorIs(t, err, context.Canceled)
	require.Zero(t, lo)
	require.Zero(t, hi)
	require.Zero(t, bound)
}
