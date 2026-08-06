package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// displacedUnionBody is the far-placed analytic union every section-
// displacement case here reads: a 10×10 box over the origin unioned with a 6×6
// box drawn at -shift and placed by +shift, so operand B's re-expression into
// A's frame is a genuine recomputation at that magnitude and the merged section
// carries the displacement it rounds to (docs/prism-boolean-design.md §7). B
// stays strictly inside A, so the merged outline IS A's own 10×10 square.
func displacedUnionBody(t *testing.T, shift float64) *Body {
	t.Helper()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalBoxBody(t, doc, 2-shift, 2, 8-shift, 8, 10)
	m, err := r3.Translation(r3.NewVec(shift, 0, 0))
	require.NoError(t, err)
	moved, err := b.Placed(m)
	require.NoError(t, err)
	got, err := Union(a, moved)
	require.NoError(t, err)
	return got
}

// TestTessellateChargesSectionDisplacementToEveryProof is
// docs/tessellation-design.md §5's section-displacement term read at the two
// proofs the mesh publishes. The merged section is straight-only, so the
// chording itself takes no sagitta at all and every millimetre of bound and
// every square millimetre of slack the mesh reports is the displacement's own.
func TestTessellateChargesSectionDisplacementToEveryProof(t *testing.T) {
	got := displacedUnionBody(t, 1e14)
	pp, ok := got.payload.(prismPayload)
	require.True(t, ok, `the analytic reduction must own this pair`)
	require.Positive(t, pp.sectionDelta)

	mesh, err := tessellateContext(t.Context(), got, units.Millimeters(20))
	require.NoError(t, err)

	// The bound is the displacement, up-rounded once.
	require.GreaterOrEqual(t, mesh.bound, pp.sectionDelta)
	require.Equal(t, upRound(pp.sectionDelta), mesh.bound)

	// The slack is the same displacement read as an area, composed exactly as
	// evalPrism composes it: the section's own tube once per cap — 2·δ·p over
	// the 40 mm outline plus a δ-disk at each of the four corners — and the
	// outline's length displacement over the 10 mm sweep.
	const perimeter, height, walks = 40.0, 10.0, 4.0
	d := pp.sectionDelta
	capMove := 2*d*perimeter + walks*math.Pi*d*d
	wallMove := walks * 12 * math.Pi * d * height
	require.InEpsilon(t, 2*capMove+wallMove, mesh.areaSlack, 1e-12)

	// An undisplaced straight prism charges neither term.
	plain := internalBoxBody(t, New(), 0, 0, 10, 10, 10)
	flat, err := tessellateContext(t.Context(), plain, units.Millimeters(1))
	require.NoError(t, err)
	require.Zero(t, flat.bound)
	require.Zero(t, flat.areaSlack)
}

func TestRequireLoopClearanceOffersValidRetry(t *testing.T) {
	pts := []Point2{
		{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 1}, {U: 0, V: 1},
		{U: 1.25, V: 0}, {U: 2.25, V: 0}, {U: 2.25, V: 1}, {U: 1.25, V: 1},
	}
	loops := [][]int{{0, 1, 2, 3}, {4, 5, 6, 7}}
	sags := []float64{0.2, 0.2}
	floor := 1e-9*math.Hypot(2.25, 1) + 4*(math.Nextafter(2.25, math.Inf(1))-2.25)

	err := requireLoopClearance(t.Context(), pts, loops, sags)
	require.ErrorIs(t, err, ErrDegenerate)
	require.Contains(t, err.Error(), `cap boundary loops 0 and 1`)
	require.Contains(t, err.Error(), `measured distance `+units.Millimeters(0.25).String())
	require.Contains(t, err.Error(), `required clearance gate `+units.Millimeters(0.4+floor).String())
	require.Contains(t, err.Error(), `retry with a finer chord tolerance`)

	require.NoError(t, requireLoopClearance(t.Context(), pts, loops, []float64{0.1, 0.1}))
}

func TestRequireLoopClearanceOmitsInvalidRetry(t *testing.T) {
	pts := []Point2{
		{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 1}, {U: 0, V: 1},
		{U: 1, V: 0}, {U: 2, V: 0}, {U: 2, V: 1}, {U: 1, V: 1},
	}
	loops := [][]int{{0, 1, 2, 3}, {4, 5, 6, 7}}
	sags := []float64{0.2, 0.2}
	floor := 1e-9*math.Hypot(2, 1) + 4*(math.Nextafter(2, math.Inf(1))-2)

	err := requireLoopClearance(t.Context(), pts, loops, sags)
	require.ErrorIs(t, err, ErrDegenerate)
	require.Contains(t, err.Error(), `cap boundary loops 0 and 1`)
	require.Contains(t, err.Error(), `measured distance 0 mm`)
	require.Contains(t, err.Error(), `required clearance gate `+units.Millimeters(0.4+floor).String())
	require.NotContains(t, err.Error(), `retry`)

	err = requireLoopClearance(t.Context(), pts, loops, []float64{0, 0})
	require.ErrorIs(t, err, ErrDegenerate)
}

func TestTessellateContextReachesCapTriangulationCancellation(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	center := s.CreatePoint(70, 30)
	s.Fix(center)
	s.CreateCircle(center, 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	doc := New()
	body, err := doc.Extrude(s, prof, Distance{D: units.Millimeters(8), Dir: Along})
	require.NoError(t, err)
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "maxU"}

	_, err = body.TessellateContext(ctx, units.Millimeters(0.0005))
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `the public context must reach cap hole ordering`)
}
