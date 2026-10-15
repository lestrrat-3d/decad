package decad

import (
	"context"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

// A heel tool can clear the finite blank outer line while its apex lies just
// inside the blank's whole heel cone. The tangent cap must then be declined.
func TestSpiralFirstJoinDeclinesTangentHeelOutsideFiniteBlankOnly(t *testing.T) {
	doc := New()
	blank, tooth, _ := spiralTangentBlankAndTooth(t, doc)
	section, ok := exactSpiralBlank(blank)
	require.True(t, ok)
	trim := tooth.n.Payload.(facetedPayload).LoftFacesTrim
	require.NotNil(t, trim)
	require.True(t, spiralTangentRecordCurrent(trim))
	m, b := section.section.meridianEquation()
	toeX, ok := spiralRootToolX(m, b, trim.Toe)
	require.True(t, ok)
	heelX, ok := spiralRootToolX(m, b, trim.Heel)
	require.True(t, ok)
	_, ok = spiralToolOuterGap(trim.Heel, section.ded, section.heelOuter, -1)
	require.True(t, ok, "the finite heel gap admits this producer-built tool")
	require.True(t, spiralTangentCapTouchesHeelCircle(trim, m, b, toeX, heelX))
	require.False(t, spiralHeelCapEightOutside(trim.Maps[8],
		lfAdd(lfF(section.ded.U), lfF(section.ded.V))))

	proof, accepted, err := certifySpiralFirstJoin(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.False(t, accepted, "the tangent heel cap cannot certify the whole blank cone")
	require.Nil(t, proof)
}

// The toe tool can reach cap 0 while the tangent heel-circle test still
// passes. The whole original toe cap must stay before the tool/root circle.
func TestSpiralFirstJoinDeclinesTangentToeCapPastToeCircle(t *testing.T) {
	blank, tooth, _ := spiralTangentBlankAndToothWithTools(t, New(), -3.02, false)
	section, ok := exactSpiralBlank(blank)
	require.True(t, ok)
	trim := tooth.n.Payload.(facetedPayload).LoftFacesTrim
	require.NotNil(t, trim)
	require.True(t, spiralTangentRecordCurrent(trim))
	m, b := section.section.meridianEquation()
	toeX, ok := spiralRootToolX(m, b, trim.Toe)
	require.True(t, ok)
	heelX, ok := spiralRootToolX(m, b, trim.Heel)
	require.True(t, ok)
	box, err := lfSourceBox(trim.Source)
	require.NoError(t, err)
	capX := loftface.ScalarRange(trim.Maps[0], loftFaceRatVec{lfOne(), lfZero(), lfZero()},
		loftface.BoxCorners(box))
	require.GreaterOrEqual(t, capX.Hi.Cmp(toeX), 0, "cap 0 reaches the toe tool/root circle")
	require.True(t, loftface.CapPlaneTouchesRootCircle(trim.Maps[8], loftface.BoxCorners(box),
		m, b, heelX, trim.Maps[7]), "the heel-circle leg still passes")
	require.False(t, spiralOriginalCapsAvoidToolRootCircles(trim, m, b, toeX, heelX))

	proof, accepted, err := certifySpiralFirstJoin(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.False(t, accepted, "the tangent branch must check the whole toe cap")
	require.Nil(t, proof)
}

func TestSpiralRootTubePropagatesCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, ok, err := spiralRootTube(ctx, nil, 0, big.NewRat(0, 1), big.NewRat(1, 1))
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
}

func TestSpiralCapPlaneMustMissTheWholeRootCircle(t *testing.T) {
	box := [4]*big.Rat{big.NewRat(-1, 1), big.NewRat(1, 1),
		big.NewRat(-1, 1), big.NewRat(1, 1)}
	corners := loftface.BoxCorners(box)
	capMap := loftFaceMap{
		Constant: loftFaceRatVec{lfZero(), lfZero(), lfZero()},
		Q0:       loftFaceRatVec{lfZero(), big.NewRat(1, 1), lfZero()},
		QU:       loftFaceRatVec{lfOne(), lfZero(), lfZero()},
		QV:       loftFaceRatVec{lfZero(), lfZero(), lfOne()},
		G0:       lfOne(), GU: lfZero(), GV: lfZero(),
	}
	m, b, x := lfZero(), lfOne(), lfZero()
	require.False(t, loftface.CapPlaneAvoidsRootCircle(capMap, corners, m, b, x),
		"a tangent cap can meet the circle")
	capMap.Q0[1] = lfAdd(capMap.Q0[1], big.NewRat(1, 1<<50))
	require.True(t, loftface.CapPlaneAvoidsRootCircle(capMap, corners, m, b, x),
		"an exact sub-femtometre gap still separates the full cap")
	capMap.Q0[1] = lfZero()
	require.False(t, loftface.CapPlaneAvoidsRootCircle(capMap, corners, m, b, x),
		"a cap crossing the circle must refuse")
	capMap.QU = loftFaceRatVec{lfZero(), lfZero(), lfZero()}
	require.False(t, loftface.CapPlaneAvoidsRootCircle(capMap, corners, m, b, x),
		"a collapsed cap has no supporting plane")
}

func TestSpiralToolCapMustStayOnCertifiedSideOfBlankMeridian(t *testing.T) {
	toe := planePoint{U: 2, V: 8}
	outer := planePoint{U: 3, V: 7}
	cone := &pointConeLimit{apex: big.NewRat(9999999999999999, 1000000000000000),
		slope: big.NewRat(1, 1)}
	gap, ok := spiralToolOuterGap(cone, toe, outer, 1)
	require.True(t, ok)
	require.Equal(t, big.NewRat(1, 1000000000000000), gap)

	// A tiny side reversal must refuse the source-specific join.
	cone.apex = big.NewRat(10000000000000001, 1000000000000000)
	_, ok = spiralToolOuterGap(cone, toe, outer, 1)
	require.False(t, ok)

	// A different slope may keep the side, but its full finite-meridian
	// separation must be returned for the later cap-strip allowance gate.
	cone.apex, cone.slope = big.NewRat(10, 1), big.NewRat(2, 1)
	gap, ok = spiralToolOuterGap(cone, toe, outer, 1)
	require.True(t, ok)
	require.Equal(t, big.NewRat(8, 1), gap)
}

func TestSpiralFiniteConeSideRequiresWholeGeneratorAndFiniteTool(t *testing.T) {
	cone := &pointConeLimit{apex: big.NewRat(10, 1), far: big.NewRat(-1, 1),
		slope: big.NewRat(1, 1)}
	first, last := planePoint{U: 2, V: 9}, planePoint{U: 3, V: 8}
	require.True(t, spiralFiniteConeSide(cone, first, last, 1))
	last.V = 6
	require.False(t, spiralFiniteConeSide(cone, first, last, 1),
		"opposite endpoint signs cannot certify a complete face")
	last.V = 8
	cone.far = big.NewRat(0, 1)
	require.False(t, spiralFiniteConeSide(cone, first, last, 1),
		"the admitted tool must have a finite far plane before the source face")
	cone.far = big.NewRat(-1, 1)
	last.U = 10
	require.False(t, spiralFiniteConeSide(cone, first, last, 1),
		"the admitted generator must end before the cone apex")
}

// The *Ref functions are the big.Rat formulations the integer primitives
// replaced. The differential tests compare against them value for value.

// Products of Bernstein polynomials retain exact rational coefficients.
// The largest degree used here is 23, whose binomial weights fit int64.
// Products of Bernstein polynomials retain exact rational coefficients.
// The largest degree used here is 23, whose binomial weights fit int64.
func spiralMulRef(a, b []*big.Rat) []*big.Rat {
	n, m := len(a)-1, len(b)-1
	out := make([]*big.Rat, n+m+1)
	for k := range out {
		out[k] = lfZero()
		for i := max(0, k-m); i <= min(n, k); i++ {
			j := k - i
			weight := big.NewRat(spiralBinom(n, i)*spiralBinom(m, j), spiralBinom(n+m, k))
			out[k] = lfAdd(out[k], lfMul(weight, lfMul(a[i], b[j])))
		}
	}
	return out
}

func spiralBinom(n, k int) int64 {
	if k < 0 || k > n {
		return 0
	}
	if k > n-k {
		k = n - k
	}
	out := int64(1)
	for i := 1; i <= k; i++ {
		out = out * int64(n-k+i) / int64(i)
	}
	return out
}

func spiralBernValueRef(coeff []*big.Rat, parameter *big.Rat) *big.Rat {
	work := append([]*big.Rat(nil), coeff...)
	for width := len(work); width > 1; width-- {
		for i := 0; i+1 < width; i++ {
			work[i] = lfAdd(lfMul(lfSub(lfOne(), parameter), work[i]),
				lfMul(parameter, work[i+1]))
		}
	}
	return work[0]
}

func spiralRootPolynomialRef(first, second loftFaceMap, span freeform.BezierSpan,
	m, b *big.Rat) [3][]*big.Rat {
	g0, h0 := loftface.CapBernsteinRows(first, span)
	g1, h1 := loftface.CapBernsteinRows(second, span)
	d := spiralMulRef(g0, g1)
	var a, z [3][]*big.Rat
	for axis := range a {
		a[axis] = spiralMulRef(h0[axis], g1)
		z[axis] = spiralMulRef(h1[axis], g0)
	}
	al := polynomial.BernsteinAdd(polynomial.BernsteinScale(a[0], m), polynomial.BernsteinScale(d, b))
	zl := polynomial.BernsteinAdd(polynomial.BernsteinScale(z[0], m), polynomial.BernsteinScale(d, b))
	dy, dz, dl := polynomial.BernsteinSub(z[1], a[1]), polynomial.BernsteinSub(z[2], a[2]), polynomial.BernsteinSub(zl, al)
	q0 := polynomial.BernsteinSub(polynomial.BernsteinAdd(spiralMulRef(a[1], a[1]), spiralMulRef(a[2], a[2])), spiralMulRef(al, al))
	q1 := polynomial.BernsteinScale(polynomial.BernsteinSub(polynomial.BernsteinAdd(spiralMulRef(a[1], dy), spiralMulRef(a[2], dz)),
		spiralMulRef(al, dl)), big.NewRat(2, 1))
	q2 := polynomial.BernsteinSub(polynomial.BernsteinAdd(spiralMulRef(dy, dy), spiralMulRef(dz, dz)), spiralMulRef(dl, dl))
	return [3][]*big.Rat{q0, polynomial.BernsteinAdd(q0, polynomial.BernsteinScale(q1, big.NewRat(1, 2))),
		polynomial.BernsteinAdd(polynomial.BernsteinAdd(q0, q1), q2)}
}

func spiralRootXOrderRef(first, second loftFaceMap, span freeform.BezierSpan,
	q [3][]*big.Rat) [3][]*big.Rat {
	g0, h0 := loftface.CapBernsteinRows(first, span)
	g1, h1 := loftface.CapBernsteinRows(second, span)
	d := spiralMulRef(g0, g1)
	a, z := spiralMulRef(h0[0], g1), spiralMulRef(h1[0], g0)
	b := polynomial.BernsteinSub(z, a)
	bd := spiralMulRef(b, d)
	u0 := polynomial.BernsteinSub(spiralMulRef(polynomial.BernsteinDeriv(a), d), spiralMulRef(a, polynomial.BernsteinDeriv(d)))
	u1 := polynomial.BernsteinSub(spiralMulRef(polynomial.BernsteinDeriv(b), d), spiralMulRef(b, polynomial.BernsteinDeriv(d)))
	q0 := q[0]
	q1 := polynomial.BernsteinScale(polynomial.BernsteinSub(q[1], q[0]), big.NewRat(2, 1))
	q2 := polynomial.BernsteinAdd(polynomial.BernsteinSub(q[2], polynomial.BernsteinScale(q[1], big.NewRat(2, 1))), q[0])
	j0 := polynomial.BernsteinSub(spiralMulRef(bd, polynomial.BernsteinDeriv(q0)), spiralMulRef(u0, q1))
	j1 := polynomial.BernsteinSub(spiralMulRef(bd, polynomial.BernsteinDeriv(q1)),
		polynomial.BernsteinAdd(polynomial.BernsteinScale(spiralMulRef(u0, q2), big.NewRat(2, 1)), spiralMulRef(u1, q1)))
	j2 := polynomial.BernsteinSub(spiralMulRef(bd, polynomial.BernsteinDeriv(q2)),
		polynomial.BernsteinScale(spiralMulRef(u1, q2), big.NewRat(2, 1)))
	return [3][]*big.Rat{j0, polynomial.BernsteinAdd(j0, polynomial.BernsteinScale(j1, big.NewRat(1, 2))),
		polynomial.BernsteinAdd(polynomial.BernsteinAdd(j0, j1), j2)}
}

func requireSameRows(t *testing.T, want, got []*big.Rat) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		require.Equal(t, 0, want[i].Cmp(got[i]), "index %d", i)
		require.Equal(t, want[i].Num().String(), got[i].Num().String())
		require.Equal(t, want[i].Denom().String(), got[i].Denom().String())
	}
}

func TestSpiralRootRowsMatchReference(t *testing.T) {
	rng := rand.New(rand.NewPCG(13, 17))
	for range 12 {
		first, second := randomGateMap(rng), randomGateMap(rng)
		span := make(freeform.BezierSpan, 4)
		for i := range span {
			span[i] = freeform.RatPoint{U: lfF(rng.Float64()*4 - 2), V: lfF(rng.Float64()*4 - 2)}
		}
		m, b := lfF(rng.Float64()*2-1), lfF(rng.Float64()*2-1)
		want := spiralRootPolynomialRef(first, second, span, m, b)
		got := loftface.MeridianQRows(first, second, span, m, b)
		for i := range want {
			requireSameRows(t, want[i], got[i])
		}
		wantX := spiralRootXOrderRef(first, second, span, want)
		gotX := loftface.MeridianJRows(first, second, span, got)
		for i := range wantX {
			requireSameRows(t, wantX[i], gotX[i])
		}
	}
}

// spiralRootTube encloses the one true flank/root crossing branch over a
// caller-specified exact X strip. J>0 orders its two endpoint cuts in band
// time. Bernstein signs enclose every intermediate source parameter before
// the rational cap maps enclose the entire resulting ruled strip.
func spiralRootTube(ctx context.Context, proof *spiralFirstJoinSource, side int,
	xLo, xHi *big.Rat) ([3]loftFaceInterval, bool, error) {
	var empty [3]loftFaceInterval
	if ctx == nil {
		return empty, false, nil
	}
	if err := ctx.Err(); err != nil {
		return empty, false, err
	}
	if proof == nil || side < 0 || side >= len(proof.Flank) ||
		xLo == nil || xHi == nil || xLo.Cmp(xHi) >= 0 {
		return empty, false, nil
	}
	span := proof.Flank[side]
	low, ok := spiralIsolateJunction(ctx, proof.Tooth.Maps, span,
		proof.RootM, proof.RootB, proof.Crossings[side], xLo)
	if !ok {
		return empty, false, ctx.Err()
	}
	high, ok := spiralIsolateJunction(ctx, proof.Tooth.Maps, span,
		proof.RootM, proof.RootB, proof.Crossings[side], xHi)
	if !ok || low.Band != high.Band || low.THi.Cmp(high.TLo) >= 0 {
		return empty, false, ctx.Err()
	}
	band := low.Band
	q := loftface.MeridianQRows(proof.Tooth.Maps[band], proof.Tooth.Maps[band+1],
		span, proof.RootM, proof.RootB)
	sLo, sHi := lfZero(), lfOne()
	if !spiralJunctionRootBracket(q, sLo, sHi, low.TLo, high.THi) {
		return empty, false, ctx.Err()
	}
	for step := range 64 {
		if step%8 == 0 && ctx.Err() != nil {
			return empty, false, ctx.Err()
		}
		mid := lfMul(lfAdd(sLo, sHi), big.NewRat(1, 2))
		if polynomial.BernsteinRowsSignAt(q, mid, low.TLo, high.THi, -1) {
			sLo = mid
			continue
		}
		if polynomial.BernsteinRowsSignAt(q, mid, low.TLo, high.THi, 1) {
			sHi = mid
			continue
		}
		break
	}
	box, ok := loftface.JunctionBox(proof.Tooth.Maps[band], proof.Tooth.Maps[band+1],
		span, sLo, sHi, low.TLo, high.THi)
	if err := ctx.Err(); err != nil {
		return empty, false, err
	}
	if !ok || box[0].Lo.Cmp(xLo) > 0 || box[0].Hi.Cmp(xHi) < 0 {
		return empty, false, nil
	}
	box[0] = loftFaceInterval{Lo: xLo, Hi: xHi}
	return box, true, nil
}

// Draws a random loftFaceMap for the gate comparison.
func randomGateMap(rng *rand.Rand) loftFaceMap {
	f := func(scale float64) *big.Rat { return lfF((rng.Float64()*2 - 1) * scale) }
	vec := func(scale float64) loftFaceRatVec { return loftFaceRatVec{f(scale), f(scale), f(scale)} }
	return loftFaceMap{Constant: vec(10), Q0: vec(5), QU: vec(2), QV: vec(2),
		G0: lfF(3 + rng.Float64()), GU: f(0.3), GV: f(0.3), Stretch: 1}
}
