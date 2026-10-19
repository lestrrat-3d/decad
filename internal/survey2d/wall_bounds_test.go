package survey2d_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestWallKernelValidateUsesCandidateInterval(t *testing.T) {
	points := [][2]float64{{0, 0}, {40, 0}, {40, 10}, {0, 10}}
	elems := make([]survey2d.SurveyElem, 0, len(points))
	for i := range points {
		elem, ok := survey2d.LineElem(points[i][0], points[i][1], points[(i+1)%len(points)][0], points[(i+1)%len(points)][1])
		require.True(t, ok)
		elems = append(elems, elem)
	}
	kernel := survey2d.NewWallKernel(elems, points, math.Inf(1))
	check := func(candidate survey2d.DiskCand, wantSpanning, wantEmpty bool) {
		t.Helper()
		spanning, empty, ok, err := kernel.Validate(candidate, nil)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, wantEmpty, empty)
		require.Equal(t, wantSpanning, spanning)
	}
	check(survey2d.DiskCand{X: 20, Y: 5, R: 5 + 2e-6, RBound: 3e-6}, true, true)
	check(survey2d.DiskCand{X: 20, Y: 5, R: 5 - 2e-6, RBound: 3e-6}, true, true)
	check(survey2d.DiskCand{X: 20, Y: 5, R: 6, RBound: 1e-12}, false, false)
	check(survey2d.DiskCand{X: 20, Y: 5, R: 1, RBound: 1e-12}, false, true)
}

func TestWallKernelCandidateRadiusBounds(t *testing.T) {
	segment := sectionrecord.ArcSeg{
		Center: sectionrecord.Point2{U: 10, V: 9},
		Start:  sectionrecord.Point2{U: 9, V: 10}, End: sectionrecord.Point2{U: 11, V: 10}, TStart: 0, TEnd: 1,
	}
	walk, err := boundarywalk.WalkOf(segment, freeform.NewFreeformWork())
	require.NoError(t, err)
	elem, ok := survey2d.WalkElem(walk)
	require.True(t, ok)
	require.Equal(t, walk.RadiusBound, elem.RrBound)
	require.True(t, elem.MatInside)
	var first *survey2d.DiskCand
	require.NoError(t, survey2d.NewWallKernel([]survey2d.SurveyElem{elem}, nil, math.Inf(1)).Generate(nil, func(c survey2d.DiskCand) error {
		if first == nil {
			candidate := c
			first = &candidate
		}
		return nil
	}))
	require.NotNil(t, first)
	require.Equal(t, elem.Rr, first.R)
	require.Equal(t, elem.RrBound, first.RBound)
	sqrt2 := new(big.Float).SetPrec(200).Sqrt(new(big.Float).SetPrec(200).SetInt64(2))
	truth, _ := sqrt2.Rat(nil)
	requireRatInInterval(t, truth, first.R, first.RBound)

	sine, _ := survey2d.RadianTrigBounds(math.Pi / 6)
	line, ok := survey2d.LineElem(0, 10, 1, 10)
	require.True(t, ok)
	kernel, err := survey2d.NewWallKernelBudget(nil, []survey2d.SurveyElem{line}, nil, [][2]float64{{0, 10}},
		15*math.Pi/180, sine, false, math.Inf(1))
	require.NoError(t, err)
	var near *survey2d.DiskCand
	require.NoError(t, kernel.WedgeCands(nil, func(c survey2d.DiskCand) error {
		if math.Abs(c.R-10) < 1e-6 {
			candidate := c
			near = &candidate
		}
		return nil
	}))
	require.NotNil(t, near)
	require.Greater(t, near.RBound, 0.0)
	require.LessOrEqual(t, math.Abs(near.R-10), near.RBound)
}

func TestWallKernelExactCandidateChainsStayExact(t *testing.T) {
	holeA, ok := survey2d.ArcElem(48.5, 28.5, 1, 2*math.Pi, 0, true)
	require.True(t, ok)
	holeB, ok := survey2d.ArcElem(51.5, 32.5, 1, 2*math.Pi, 0, true)
	require.True(t, ok)
	kernel := survey2d.NewWallKernel([]survey2d.SurveyElem{holeA, holeB}, nil, math.Inf(1))
	collect := func(generate func(func(float64, float64, float64, float64))) map[float64]float64 {
		bounds := make(map[float64]float64)
		generate(func(_, _, radius, bound float64) {
			if current, ok := bounds[radius]; !ok || bound > current {
				bounds[radius] = bound
			}
		})
		return bounds
	}
	arcArc := collect(func(add func(float64, float64, float64, float64)) { kernel.ArcArcCands(holeA, holeB, add) })
	bound, found := arcArc[1.5]
	require.True(t, found)
	require.Zero(t, bound)
	vertexArc := collect(func(add func(float64, float64, float64, float64)) {
		kernel.VertexElemCands([2]float64{40, 30}, holeAt(t, 37, 26), add)
	})
	bound, found = vertexArc[2]
	require.True(t, found)
	require.Zero(t, bound)
}

func holeAt(t *testing.T, x, y float64) survey2d.SurveyElem {
	t.Helper()
	elem, ok := survey2d.ArcElem(x, y, 1, 2*math.Pi, 0, true)
	require.True(t, ok)
	return elem
}

func TestWallKernelConcentricSpanPublishesRadiusDifferenceBound(t *testing.T) {
	outer, ok := survey2d.ArcElem(0, 0, 1e16, 0, 2*math.Pi, true)
	require.True(t, ok)
	inner, ok := survey2d.ArcElem(0, 0, 1, 2*math.Pi, 0, true)
	require.True(t, ok)
	out := survey2d.NewWallKernel([]survey2d.SurveyElem{outer, inner}, nil, math.Inf(1)).Run()
	require.True(t, out.Ok)
	require.True(t, out.HasSpan)
	truth := new(big.Rat).SetInt64(9999999999999999)
	require.NotZero(t, new(big.Rat).SetFloat64(out.Span).Cmp(truth))
	requireRatInInterval(t, truth, out.Span, out.SpanBound)
}

func requireRatInInterval(t *testing.T, truth *big.Rat, value, bound float64) {
	t.Helper()
	v, b := new(big.Rat).SetFloat64(value), new(big.Rat).SetFloat64(bound)
	require.NotNil(t, v)
	require.NotNil(t, b)
	require.LessOrEqual(t, new(big.Rat).Sub(v, b).Cmp(truth), 0)
	require.GreaterOrEqual(t, new(big.Rat).Add(v, b).Cmp(truth), 0)
}

func TestWallKernelFitGateUsesCandidateInterval(t *testing.T) {
	outer, ok := survey2d.ArcElem(0, 0, 20, 0, 2*math.Pi, true)
	require.True(t, ok)
	outer.RrBound = 2e-3
	inner, ok := survey2d.ArcElem(0, 0, 10, 2*math.Pi, 0, true)
	require.True(t, ok)
	fitMax := 10 - 1e-3
	kernel, err := survey2d.NewWallKernelBudget(nil, []survey2d.SurveyElem{outer, inner}, nil, nil,
		15*math.Pi/180, proofbound.ExactScalar(0), false, fitMax)
	require.NoError(t, err)
	out, err := kernel.RunBudget(nil)
	require.NoError(t, err)
	require.True(t, out.Ok)
	require.True(t, out.HasSpan)
	require.Greater(t, out.Span, fitMax)
	require.Less(t, out.Span-out.SpanBound, fitMax)
}

func TestWallKernelInradiusAggregatesRivalIntervals(t *testing.T) {
	points := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]survey2d.SurveyElem, 0, len(points)+2)
	for i := range points {
		elem, ok := survey2d.LineElem(points[i][0], points[i][1], points[(i+1)%len(points)][0], points[(i+1)%len(points)][1])
		require.True(t, ok)
		elems = append(elems, elem)
	}
	for i, x := range []float64{30, 70} {
		radius := 20.0
		if i == 0 {
			radius += 1e-4
		}
		hole, ok := survey2d.ArcElem(x, 30, radius, 2*math.Pi, 0, true)
		require.True(t, ok)
		if i == 0 {
			hole.RrBound = 1e-3
		}
		elems = append(elems, hole)
	}
	kernel := survey2d.NewWallKernel(elems, points, math.Inf(1))
	out := kernel.Run()
	require.True(t, out.Ok)
	aggregate := survey2d.MaxAggregate()
	heldMax, heldBound := math.Inf(-1), 0.0
	require.NoError(t, kernel.Generate(nil, func(candidate survey2d.DiskCand) error {
		_, empty, ok, err := kernel.Validate(candidate, nil)
		require.NoError(t, err)
		require.True(t, ok)
		if !empty {
			return nil
		}
		aggregate.Take(candidate.R, candidate.RBound)
		if candidate.R > heldMax {
			heldMax, heldBound = candidate.R, candidate.RBound
		}
		return nil
	}))
	wantValue, wantBound, resolved := aggregate.Resolve()
	require.True(t, resolved)
	require.Equal(t, wantValue, out.Inradius)
	require.Equal(t, wantBound, out.InradiusBound)
	winnerHi := new(big.Rat).Add(proofarith.FloatRat(heldMax), proofarith.FloatRat(heldBound))
	require.Greater(t, aggregate.Hi.Cmp(winnerHi), 0)
	require.NotEqual(t, heldBound, out.InradiusBound)
	require.Greater(t, out.InradiusBound, 0.0)
}

func TestSolve3LinearBoundCoversCramerArithmetic(t *testing.T) {
	points := [][2]float64{{0, 0}, {20.3, 1.7}, {16.1, 9.4}, {3.7, 7.9}}
	elems := make([]survey2d.SurveyElem, 0, len(points))
	for i := range points {
		elem, ok := survey2d.LineElem(points[i][0], points[i][1], points[(i+1)%len(points)][0], points[(i+1)%len(points)][1])
		require.True(t, ok)
		elems = append(elems, elem)
	}
	equations, err := survey2d.NewWallKernel(elems, points, math.Inf(1)).TripleEquations(nil)
	require.NoError(t, err)
	var linear []survey2d.CircEq
	for _, equation := range equations {
		if !equation.Quad {
			linear = append(linear, equation)
		}
	}
	require.Len(t, linear, len(elems))
	seen, exceedsDivisionOnly := 0, 0
	for i := range linear {
		for j := i + 1; j < len(linear); j++ {
			for k := j + 1; k < len(linear); k++ {
				triple := []survey2d.CircEq{linear[i], linear[j], linear[k]}
				survey2d.Solve3Linear(triple, func(_, _, radius, bound float64) {
					seen++
					truth := exactCramerRadius(triple)
					require.NotNil(t, truth)
					require.LessOrEqual(t, proofbound.RatAbsDiff(truth, radius), bound)
					if proofbound.RatAbsDiff(truth, radius) > divisionOnlyRadiusBound(triple) {
						exceedsDivisionOnly++
					}
				})
			}
		}
	}
	require.Positive(t, seen)
	require.Positive(t, exceedsDivisionOnly)
}

func TestWallKernelStraddledTripleKeepsBoundedCandidates(t *testing.T) {
	var elems []survey2d.SurveyElem
	for _, circle := range [][3]float64{
		{8255.0884454771622, 4195.7682375914947, 149.98644153683318},
		{2475.9000902096891, -11681.738286473166, 131.34705082306061},
		{-959.14779827151824, -21119.048840666175, 104.09453452133486},
	} {
		elem, ok := survey2d.ArcElem(circle[0], circle[1], circle[2], 0, 2*math.Pi, true)
		require.True(t, ok)
		elems = append(elems, elem)
	}
	kernel, err := survey2d.NewWallKernelBudget(nil, elems, nil, nil, 15*math.Pi/180,
		proofbound.ExactScalar(0), false, math.Inf(1))
	require.NoError(t, err)
	out, err := kernel.RunBudget(nil)
	require.NoError(t, err)
	require.True(t, out.Ok)
	require.True(t, out.HasSpan)
	require.Equal(t, 2*104.09453452133486, out.Span)
	require.Zero(t, out.SpanBound)
	require.Equal(t, 149.98644153683318, out.Inradius)
	require.Zero(t, out.InradiusBound)
	require.NoError(t, kernel.Generate(nil, func(candidate survey2d.DiskCand) error {
		_, empty, ok, err := kernel.Validate(candidate, nil)
		require.NoError(t, err)
		require.True(t, ok)
		if empty {
			require.False(t, math.IsInf(candidate.RBound, 1))
		}
		return nil
	}))
}

func TestSolveTripleStraddledDeterminantRetainsPairCandidate(t *testing.T) {
	lineA := survey2d.CircEq{A: proofbound.ExactScalar(1), B: proofbound.ExactScalar(0), E: proofbound.ExactScalar(-1), F: proofbound.ExactScalar(0)}
	lineB := survey2d.CircEq{A: proofbound.ExactScalar(-1), B: proofbound.MeasuredScalar(1e-13, 1e-9),
		E: proofbound.ExactScalar(-1), F: proofbound.ExactScalar(10)}
	hole := survey2d.CircEq{Quad: true, G: proofbound.ExactScalar(-10), H: proofbound.ExactScalar(-10),
		Kk: proofbound.ExactScalar(-4), M: proofbound.ExactScalar(46)}
	require.Equal(t, proofbound.SurvStraddle,
		proofbound.AdmitMagnitudeAbove(proofbound.BoundedSub(proofbound.BoundedMul(lineA.A, lineB.B),
			proofbound.BoundedMul(lineB.A, lineA.B)), 1e-12))
	var radii []float64
	survey2d.SolveTriple([3]survey2d.CircEq{lineA, lineB, hole}, 1, func(_, _, radius, bound float64) {
		require.False(t, math.IsInf(bound, 1))
		radii = append(radii, radius)
	})
	require.NotEmpty(t, radii)
	for _, radius := range radii {
		require.InDelta(t, 5.0, radius, 1e-9)
	}
}

func exactCramerRadius(equations []survey2d.CircEq) *big.Rat {
	rat := func(value proofbound.BoundedScalar) *big.Rat { return proofarith.FloatRat(value.Value) }
	minor := func(a, b, c, d *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a, d), new(big.Rat).Mul(c, b))
	}
	a, b, e, f := [3]*big.Rat{}, [3]*big.Rat{}, [3]*big.Rat{}, [3]*big.Rat{}
	for i, equation := range equations {
		a[i], b[i], e[i], f[i] = rat(equation.A), rat(equation.B), rat(equation.E), rat(equation.F)
	}
	be, ae := minor(b[1], e[1], b[2], e[2]), minor(a[1], e[1], a[2], e[2])
	ab, af := minor(a[1], b[1], a[2], b[2]), minor(a[1], f[1], a[2], f[2])
	bf := minor(b[1], f[1], b[2], f[2])
	det := new(big.Rat).Add(new(big.Rat).Sub(new(big.Rat).Mul(a[0], be), new(big.Rat).Mul(b[0], ae)), new(big.Rat).Mul(e[0], ab))
	if det.Sign() == 0 {
		return nil
	}
	dr := new(big.Rat).Add(new(big.Rat).Sub(new(big.Rat).Mul(new(big.Rat).Neg(a[0]), bf), new(big.Rat).Neg(new(big.Rat).Mul(b[0], af))), new(big.Rat).Neg(new(big.Rat).Mul(f[0], ab)))
	return new(big.Rat).Quo(dr, det)
}

func divisionOnlyRadiusBound(equations []survey2d.CircEq) float64 {
	minor := func(a, b, c, d float64) float64 { return a*d - c*b }
	a, b, e, f := [3]float64{}, [3]float64{}, [3]float64{}, [3]float64{}
	for i, equation := range equations {
		a[i], b[i], e[i], f[i] = equation.A.Value, equation.B.Value, equation.E.Value, equation.F.Value
	}
	be, ae := minor(b[1], e[1], b[2], e[2]), minor(a[1], e[1], a[2], e[2])
	ab, af := minor(a[1], b[1], a[2], b[2]), minor(a[1], f[1], a[2], f[2])
	bf := minor(b[1], f[1], b[2], f[2])
	det := a[0]*be - b[0]*ae + e[0]*ab
	dr := -a[0]*bf + b[0]*af - f[0]*ab
	return proofbound.BoundedQuotient(dr, 0, det, 0).Bound
}
