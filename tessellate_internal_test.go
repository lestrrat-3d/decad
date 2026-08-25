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

// bigSinTaylor returns sin(x) to prec bits via a Taylor series carried
// entirely in big.Float, run for a fixed 200 terms — no libm call anywhere,
// so it carries none of Go's math.Sin's own missing accuracy contract. It
// assumes |x| stays comfortably under pi, which every caller here meets:
// chordSagitta's own argument sweep/(4n) never reaches pi/2 for n>=1 and a
// sweep under one full turn.
func bigSinTaylor(x float64, prec uint) *big.Float {
	bx := new(big.Float).SetPrec(prec).SetFloat64(x)
	x2 := new(big.Float).SetPrec(prec).Mul(bx, bx)
	term := new(big.Float).SetPrec(prec).Set(bx)
	sum := new(big.Float).SetPrec(prec).Set(bx)
	for k := 1; k <= 200; k++ {
		denom := new(big.Float).SetPrec(prec).SetInt64(int64(2*k) * int64(2*k+1))
		term = new(big.Float).SetPrec(prec).Quo(new(big.Float).SetPrec(prec).Mul(term, x2), denom)
		term.Neg(term)
		sum.Add(sum, term)
	}
	return sum
}

// bigSagittaReference computes the TRUE closed-form sagitta,
// 2*radius*sin(sweep/n/4)^2 — a chord's own exact deviation from its arc,
// NOT chordSagitta's own deliberately looser proven-upper-bound formula —
// entirely in big.Float at prec bits, so the comparisons this file's
// chordSagitta tests draw never lean on any accuracy claim about float64
// arithmetic or Go's Sin.
func bigSagittaReference(radius, sweep float64, n int, prec uint) *big.Float {
	s := bigSinTaylor(sweep/float64(n)/4, prec)
	s2 := new(big.Float).SetPrec(prec).Mul(s, s)
	r := new(big.Float).SetPrec(prec).SetFloat64(radius)
	return new(big.Float).SetPrec(prec).Mul(big.NewFloat(2).SetPrec(prec), new(big.Float).SetPrec(prec).Mul(r, s2))
}

// TestChordSagittaNeverUnderstatesTheHighPrecisionReference is what makes
// chordSagitta's own doc-comment claim — PROVEN, so it may be conservative
// but must never be understated — true rather than merely stated: it checks
// the published sagitta against a from-scratch 300-bit-precision reference
// (bigSagittaReference, which never calls Go's math.Sin either) over a
// spread of radii, sweeps and chord counts, plus one deliberately
// off-the-grid row (radius=15.42, sweep=4.1657, n=57) whose radius, sweep
// and count share no factor with the round table below it.
func TestChordSagittaNeverUnderstatesTheHighPrecisionReference(t *testing.T) {
	const prec = 300
	type row struct {
		radius, sweep float64
		n             int
	}
	rows := []row{
		{15.42, 4.1657, 57},
	}
	for _, radius := range []float64{0.001, 1, 7, 15.42, 100, 5000} {
		for _, sweepDeg := range []float64{1, 5, 30, 90, 180, 270, 359} {
			for _, n := range []int{1, 2, 3, 7, 57, 128, 1024} {
				rows = append(rows, row{radius, sweepDeg * math.Pi / 180, n})
			}
		}
	}

	for _, rw := range rows {
		got := chordSagitta(rw.radius, rw.sweep, rw.n)
		want := bigSagittaReference(rw.radius, rw.sweep, rw.n, prec)
		gotBig := new(big.Float).SetPrec(prec).SetFloat64(got)
		diff := new(big.Float).SetPrec(prec).Sub(gotBig, want)
		require.GreaterOrEqualf(t, diff.Sign(), 0,
			"chordSagitta(radius=%g, sweep=%g, n=%d) = %.20g must be at or above the high-precision reference %s",
			rw.radius, rw.sweep, rw.n, got, want.Text('g', 40))
	}
}

// TestChordSagittaCoarsestClosedWalkStaysProven pins the OTHER end of
// chordSagitta's own conservatism, at the coarsest chording a CLOSED walk
// can ever reach: chordCount never admits fewer than 3 chords for a closed
// walk, and a full 2*pi sweep split 3 ways is the widest single-chord angle
// this package ever asks the bound to cover. The sin(x)<=x reduction is
// loosest exactly where x is largest, so this is where the (x/sin x)^2
// slack is at its worst — still enclosing the true sagitta, and still doing
// so by a bounded, checked margin rather than an unbounded one.
//
// This row is the one place the true sagitta is known EXACTLY with no trig
// call and no accuracy contract at all: the chord's quarter angle is
// 2*pi/(4*3) = pi/6, sin(pi/6) is exactly 1/2, so the true sagitta is
// 2*radius*(1/2)^2 = radius/2 as a rational number. A bound below that is
// disproven outright — which is what rules out evaluating the tight closed
// form in float64 and calling the result PROVEN.
//
// The measured ratio is pinned against pi^2/9, the same exact figure
// docs/tessellation-design.md Sec 3, [Body.Tessellate] and chordSagitta
// itself state for this worst case.
func TestChordSagittaCoarsestClosedWalkStaysProven(t *testing.T) {
	const prec = 300
	const radius, sweep, n = 7.0, 2 * math.Pi, 3

	got := chordSagitta(radius, sweep, n)
	require.GreaterOrEqualf(t, got, radius/2,
		"chordSagitta(radius=%g, sweep=%g, n=%d) = %.20g must be at or above the EXACT true sagitta radius/2 = %g",
		radius, sweep, n, got, radius/2)

	want := bigSagittaReference(radius, sweep, n, prec)
	gotBig := new(big.Float).SetPrec(prec).SetFloat64(got)
	diff := new(big.Float).SetPrec(prec).Sub(gotBig, want)
	require.GreaterOrEqualf(t, diff.Sign(), 0,
		"chordSagitta(radius=%g, sweep=%g, n=%d) = %.20g must enclose the high-precision reference %s even at the coarsest closed-walk chording",
		radius, sweep, n, got, want.Text('g', 40))

	ratio, _ := new(big.Float).SetPrec(prec).Quo(gotBig, want).Float64()
	require.InDeltaf(t, math.Pi*math.Pi/9, ratio, 1e-9,
		"the sin(x)<=x slack at the coarsest closed-walk chording (n=3, full circle) must sit at the stated pi^2/9, got ratio=%.12g", ratio)
}

// TestChordCountRefusesTheToleranceWindowAtTheMeshCap pins the caller-facing
// half of that same conservatism, at its exact boundary. chordCount decides
// on the PROVEN bound, so the finest tolerance a full circle can be chorded
// to is that bound's value at maxChordsPerWalk — not the smaller true
// sagitta there. Every tolerance in between, a window whose relative width
// is the (x/sin x)^2 - 1 factor [Body.Tessellate] states, leaves no
// admissible count and refuses with errTooManyChords, which is an
// ErrUnsupported. The refusal is the whole point: the alternative is a mesh
// whose published bound is not one this package can prove.
func TestChordCountRefusesTheToleranceWindowAtTheMeshCap(t *testing.T) {
	const prec = 300
	const sweep = 2 * math.Pi

	rows := []struct {
		radius float64
		// inside is a tolerance strictly within the window: at or above the
		// true sagitta at the cap, and below the proven bound there.
		inside float64
	}{
		{radius: 10, inside: 1.8383570706191657e-07},
		{radius: 0.5, inside: 9.1917853530958284169e-09},
	}
	for _, row := range rows {
		w := segmentWalk{radius: row.radius, th0: 0, th1: sweep, closed: true}
		atCap := chordSagitta(row.radius, sweep, maxChordsPerWalk)

		// The window has real width: the true sagitta at the cap sits
		// strictly below the proven bound the walk-up must satisfy.
		trueAtCap, _ := bigSagittaReference(row.radius, sweep, maxChordsPerWalk, prec).Float64()
		require.Lessf(t, trueAtCap, atCap,
			"radius=%g: the proven bound at the cap must exceed the true sagitta there", row.radius)
		require.Greaterf(t, row.inside, trueAtCap,
			"radius=%g: the sampled tolerance %.20g must sit at or above the true sagitta %.20g", row.radius, row.inside, trueAtCap)
		require.Lessf(t, row.inside, atCap,
			"radius=%g: the sampled tolerance %.20g must sit below the proven bound %.20g", row.radius, row.inside, atCap)

		// The proven bound itself is chordable, at exactly the cap count.
		n, s, err := chordCount(w, atCap)
		require.NoErrorf(t, err, "radius=%g: the proven sagitta at the cap must itself be admissible", row.radius)
		require.Equalf(t, maxChordsPerWalk, n, "radius=%g: that tolerance must spend the whole cap", row.radius)
		require.Equalf(t, atCap, s, "radius=%g: the returned sagitta is the proven bound itself", row.radius)

		for _, tol := range []float64{math.Nextafter(atCap, 0), row.inside, trueAtCap} {
			_, _, err := chordCount(w, tol)
			require.ErrorIsf(t, err, errTooManyChords,
				"radius=%g: tol=%.20g lies in the window and must refuse", row.radius, tol)
			require.ErrorIsf(t, err, ErrUnsupported,
				"radius=%g: tol=%.20g must refuse with a typed ErrUnsupported", row.radius, tol)
		}
	}
}

// TestChordSagittaRefusesRatherThanUnderstatesOnBrokenClaims pins
// chordSagitta's own three failure arms (its doc comment): a negative sweep
// and a non-positive n each answer +Inf rather than a silently-understated
// 0, since productUpper's own a<=0 guard would otherwise read a negative
// sweep as a zero sagitta when the true sagitta is positive. A negative
// radius is different — the true sagitta r·(1−cos) is itself non-positive
// there, so 0 remains a genuine (if unattained) upper bound and needs no
// refusal.
func TestChordSagittaRefusesRatherThanUnderstatesOnBrokenClaims(t *testing.T) {
	require.True(t, math.IsInf(chordSagitta(5, -1, 8), 1), "negative sweep must refuse, not understate")
	require.True(t, math.IsInf(chordSagitta(5, 1, -3), 1), "non-positive n must refuse, not understate")
	require.True(t, math.IsInf(chordSagitta(5, 1, 0), 1), "n=0 must refuse, not understate")
	require.Zero(t, chordSagitta(-5, 1, 8), "a negative radius has a genuine 0 upper bound and needs no refusal")
}
