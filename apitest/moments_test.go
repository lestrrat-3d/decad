package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func recordOne(t *testing.T, s *sketch.Sketch, pick func(*sketch.Profile) bool) momentinput.Profile {
	t.Helper()
	_, err := s.Solve(t.Context())
	require.NoError(t, err)
	for _, profile := range s.Profiles() {
		if !pick(profile) {
			continue
		}
		record, _, err := momentinput.RecordProfile(s, profile)
		require.NoError(t, err)
		return record
	}
	t.Fatal(`no profile matched`)
	return momentinput.Profile{}
}

func momentLine(u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
	return sectionrecord.LineSeg{
		Start: decad.Point2{U: u0, V: v0},
		End:   decad.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1,
	}
}

func momentSquare(u0, v0, u1, v1 float64, clockwise bool) sectionrecord.LoopRecord {
	points := [][2]float64{{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}}
	if clockwise {
		points = [][2]float64{{u0, v0}, {u0, v1}, {u1, v1}, {u1, v0}}
	}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		momentLine(points[0][0], points[0][1], points[1][0], points[1][1]),
		momentLine(points[1][0], points[1][1], points[2][0], points[2][1]),
		momentLine(points[2][0], points[2][1], points[3][0], points[3][1]),
		momentLine(points[3][0], points[3][1], points[0][0], points[0][1]),
	}}
}

func momentWholeCircle(center decad.Point2, radius float64, counterclockwise bool) sectionrecord.LoopRecord {
	segment := sectionrecord.CircleSeg{
		Center: center,
		Radius: units.Millimeters(radius),
		CCW:    counterclockwise,
	}
	if counterclockwise {
		segment.TEnd = 1
	} else {
		segment.TStart = 1
	}
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{segment}}
}

func requireProfileMomentError(t *testing.T, record momentinput.Profile, target error) {
	t.Helper()
	_, err := record.Area()
	require.ErrorIs(t, err, target)
	_, err = record.Centroid()
	require.ErrorIs(t, err, target)
	_, err = record.SecondMoments()
	require.ErrorIs(t, err, target)
}

func requireBoundContainsBig(t *testing.T, got, bound float64, want *big.Float) {
	t.Helper()
	const precision = 256
	diff := new(big.Float).SetPrec(precision).Sub(
		new(big.Float).SetPrec(precision).SetFloat64(got),
		want,
	)
	diff.Abs(diff)
	heldBound := new(big.Float).SetPrec(precision).SetFloat64(bound)
	require.GreaterOrEqual(t, heldBound.Cmp(diff), 0, "error %s exceeds bound %.17g", diff.Text('g', 18), bound)
}

func precisePi(t *testing.T) *big.Float {
	t.Helper()
	const digits = "3.141592653589793238462643383279502884197169399375105820974944592307816406286"
	pi, ok := new(big.Float).SetPrec(256).SetString(digits)
	require.True(t, ok)
	return pi
}

func preciseAtan(t *testing.T, x *big.Rat) *big.Float {
	t.Helper()
	const precision = uint(256)
	xSquared := new(big.Rat).Mul(x, x)
	term := new(big.Rat).Set(x)
	sum := new(big.Rat)
	for n := range 256 {
		addend := new(big.Rat).Quo(term, big.NewRat(int64(2*n+1), 1))
		if n%2 == 0 {
			sum.Add(sum, addend)
		} else {
			sum.Sub(sum, addend)
		}
		term.Mul(term, xSquared)
	}
	return new(big.Float).SetPrec(precision).SetRat(sum)
}

func preciseGreenArcArea(t *testing.T, center, start, end decad.Point2) *big.Float {
	t.Helper()
	const precision = uint(256)
	floatRat := func(value float64) *big.Rat {
		return new(big.Rat).SetFloat64(value)
	}
	bigFloatRat := func(value *big.Rat) *big.Float {
		return new(big.Float).SetPrec(precision).SetRat(value)
	}
	bigFloat := func(value float64) *big.Float {
		return new(big.Float).SetPrec(precision).SetFloat64(value)
	}

	dx0 := new(big.Rat).Sub(floatRat(start.U), floatRat(center.U))
	dy0 := new(big.Rat).Sub(floatRat(start.V), floatRat(center.V))
	dx1 := new(big.Rat).Sub(floatRat(end.U), floatRat(center.U))
	dy1 := new(big.Rat).Sub(floatRat(end.V), floatRat(center.V))
	r0Squared := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
	r1Squared := new(big.Rat).Add(new(big.Rat).Mul(dx1, dx1), new(big.Rat).Mul(dy1, dy1))
	r0 := new(big.Float).SetPrec(precision).Sqrt(bigFloatRat(r0Squared))
	r1 := new(big.Float).SetPrec(precision).Sqrt(bigFloatRat(r1Squared))

	angleArg := new(big.Rat).Quo(
		new(big.Rat).Sub(dy1, dx1),
		new(big.Rat).Add(dy1, dx1),
	)
	theta := new(big.Float).SetPrec(precision).Quo(precisePi(t), bigFloat(4))
	theta.Add(theta, preciseAtan(t, angleArg))

	endSin := new(big.Float).SetPrec(precision).Quo(bigFloatRat(dy1), r1)
	endSin.Mul(endSin, r0)
	endCos := new(big.Float).SetPrec(precision).Quo(bigFloatRat(dx1), r1)
	endCos.Mul(endCos, r0)

	arc := new(big.Float).SetPrec(precision).Mul(new(big.Float).SetPrec(precision).Mul(r0, r0), theta)
	centerU, centerV := bigFloat(center.U), bigFloat(center.V)
	arc.Add(arc, new(big.Float).SetPrec(precision).Mul(
		centerU,
		new(big.Float).SetPrec(precision).Sub(endSin, bigFloatRat(dy0)),
	))
	arc.Sub(arc, new(big.Float).SetPrec(precision).Mul(
		centerV,
		new(big.Float).SetPrec(precision).Sub(endCos, bigFloatRat(dx0)),
	))
	arc.Mul(arc, bigFloat(0.5))

	startU, startV := bigFloat(start.U), bigFloat(start.V)
	endU, endV := bigFloat(end.U), bigFloat(end.V)
	line := new(big.Float).SetPrec(precision).Mul(endU, centerV)
	line.Sub(line, new(big.Float).SetPrec(precision).Mul(endV, centerU))
	line.Add(line, new(big.Float).SetPrec(precision).Mul(centerU, startV))
	line.Sub(line, new(big.Float).SetPrec(precision).Mul(centerV, startU))
	line.Mul(line, bigFloat(0.5))
	return arc.Add(arc, line)
}

// preciseArcWedge holds the 256-bit trig reading of the wedge record
// preciseGreenArcArea integrates — an arc from Start (at angle 0 about Center)
// on Start's radius to End's angle, closed by End→Center→Start — built the
// same way that helper builds it: the radius is the square root of Start's
// exact squared distance, the end angle is π/4 + atan((dy1−dx1)/(dy1+dx1)),
// and End's sine/cosine are its exact deltas over End's own radius.
type preciseArcWedge struct {
	cU, cV, r, theta *big.Float
	sin0, cos0       *big.Float
	sin1, cos1       *big.Float
	startU, startV   *big.Float
	endU, endV       *big.Float
}

// preciseLineMoment is addLine's own first- and second-moment contribution of
// one line segment, in 256-bit arithmetic.
type preciseLineMoment struct {
	mu, mv, muu, muv, mvv *big.Float
}

const preciseWedgePrecision = uint(256)

func preciseFloat() *big.Float { return new(big.Float).SetPrec(preciseWedgePrecision) }

func preciseFloat64(value float64) *big.Float { return preciseFloat().SetFloat64(value) }

func preciseRat(value *big.Rat) *big.Float { return preciseFloat().SetRat(value) }

func preciseAdd(values ...*big.Float) *big.Float {
	out := preciseFloat()
	for _, value := range values {
		out.Add(out, value)
	}
	return out
}

func preciseMul(values ...*big.Float) *big.Float {
	out := preciseFloat64(1)
	for _, value := range values {
		out.Mul(out, value)
	}
	return out
}

func preciseSub(a, b *big.Float) *big.Float { return preciseFloat().Sub(a, b) }

func preciseQuo(a, b *big.Float) *big.Float { return preciseFloat().Quo(a, b) }

func newPreciseArcWedge(t *testing.T, center, start, end decad.Point2) preciseArcWedge {
	t.Helper()
	floatRat := func(value float64) *big.Rat { return new(big.Rat).SetFloat64(value) }
	dx0 := new(big.Rat).Sub(floatRat(start.U), floatRat(center.U))
	dy0 := new(big.Rat).Sub(floatRat(start.V), floatRat(center.V))
	dx1 := new(big.Rat).Sub(floatRat(end.U), floatRat(center.U))
	dy1 := new(big.Rat).Sub(floatRat(end.V), floatRat(center.V))
	require.Zero(t, dy0.Sign(), `the wedge's Start must sit at angle 0 about its Center`)
	require.Positive(t, dx0.Sign(), `the wedge's Start must sit at angle 0 about its Center`)
	r0Squared := new(big.Rat).Add(new(big.Rat).Mul(dx0, dx0), new(big.Rat).Mul(dy0, dy0))
	r1Squared := new(big.Rat).Add(new(big.Rat).Mul(dx1, dx1), new(big.Rat).Mul(dy1, dy1))
	r0 := preciseFloat().Sqrt(preciseRat(r0Squared))
	r1 := preciseFloat().Sqrt(preciseRat(r1Squared))

	angleArg := new(big.Rat).Quo(new(big.Rat).Sub(dy1, dx1), new(big.Rat).Add(dy1, dx1))
	theta := preciseAdd(preciseQuo(precisePi(t), preciseFloat64(4)), preciseAtan(t, angleArg))

	return preciseArcWedge{
		cU:     preciseFloat64(center.U),
		cV:     preciseFloat64(center.V),
		r:      r0,
		theta:  theta,
		sin0:   preciseQuo(preciseRat(dy0), r0),
		cos0:   preciseQuo(preciseRat(dx0), r0),
		sin1:   preciseQuo(preciseRat(dy1), r1),
		cos1:   preciseQuo(preciseRat(dx1), r1),
		startU: preciseFloat64(start.U),
		startV: preciseFloat64(start.V),
		endU:   preciseFloat64(end.U),
		endV:   preciseFloat64(end.V),
	}
}

// preciseLineMoments evaluates addLine's own first- and second-moment
// closed forms for the segment (u0, v0)→(u1, v1) in 256-bit arithmetic.
func preciseLineMoments(u0, v0, u1, v1 *big.Float) preciseLineMoment {
	six, twelve := preciseFloat64(6), preciseFloat64(12)
	du, dv := preciseSub(u1, u0), preciseSub(v1, v0)
	mu := preciseQuo(preciseMul(dv, preciseAdd(preciseMul(u0, u0), preciseMul(u0, u1), preciseMul(u1, u1))), six)
	mv := preciseQuo(preciseMul(du, preciseAdd(preciseMul(v0, v0), preciseMul(v0, v1), preciseMul(v1, v1))), six)
	mv.Neg(mv)
	muu := preciseQuo(preciseMul(dv, preciseAdd(
		preciseMul(u0, u0, u0), preciseMul(u0, u0, u1), preciseMul(u0, u1, u1), preciseMul(u1, u1, u1),
	)), twelve)
	mvv := preciseQuo(preciseMul(du, preciseAdd(
		preciseMul(v0, v0, v0), preciseMul(v0, v0, v1), preciseMul(v0, v1, v1), preciseMul(v1, v1, v1),
	)), twelve)
	mvv.Neg(mvv)
	intU2V := preciseAdd(
		preciseMul(v0, preciseAdd(preciseMul(u0, u0), preciseMul(u0, du), preciseQuo(preciseMul(du, du), preciseFloat64(3)))),
		preciseMul(dv, preciseAdd(
			preciseQuo(preciseMul(u0, u0), preciseFloat64(2)),
			preciseQuo(preciseMul(preciseFloat64(2), u0, du), preciseFloat64(3)),
			preciseQuo(preciseMul(du, du), preciseFloat64(4)),
		)),
	)
	muv := preciseMul(preciseFloat64(0.5), dv, intU2V)
	return preciseLineMoment{mu: mu, mv: mv, muu: muu, muv: muv, mvv: mvv}
}

// arcTrigIntegrals are addCircular's own trig integrals over [0, θ1] in
// 256-bit arithmetic, every higher multiple taken through the double-angle
// identities.
type arcTrigIntegrals struct {
	intCos, intCos2, intCos3, intCos4 *big.Float
	intSin, intSin2, intSin3, intSin4 *big.Float
	intSC, intSC2, intSC3             *big.Float
}

func (w preciseArcWedge) trigIntegrals() arcTrigIntegrals {
	two, three, four := preciseFloat64(2), preciseFloat64(3), preciseFloat64(4)
	sin2 := func(s, c *big.Float) *big.Float { return preciseMul(two, s, c) }
	cos2 := func(s, c *big.Float) *big.Float { return preciseSub(preciseMul(c, c), preciseMul(s, s)) }
	sin4 := func(s, c *big.Float) *big.Float { return preciseMul(two, sin2(s, c), cos2(s, c)) }
	cube := func(x *big.Float) *big.Float { return preciseMul(x, x, x) }
	quartic := func(x *big.Float) *big.Float { return preciseMul(x, x, x, x) }
	dth := w.theta
	sin2Diff := preciseSub(sin2(w.sin1, w.cos1), sin2(w.sin0, w.cos0))
	sin4Diff := preciseSub(sin4(w.sin1, w.cos1), sin4(w.sin0, w.cos0))
	halfDth := preciseQuo(dth, two)
	threeEighthsDth := preciseQuo(preciseMul(three, dth), preciseFloat64(8))
	return arcTrigIntegrals{
		intCos:  preciseSub(w.sin1, w.sin0),
		intCos2: preciseAdd(halfDth, preciseQuo(sin2Diff, four)),
		intCos3: preciseSub(
			preciseSub(w.sin1, preciseQuo(cube(w.sin1), three)),
			preciseSub(w.sin0, preciseQuo(cube(w.sin0), three)),
		),
		intCos4: preciseAdd(threeEighthsDth, preciseQuo(sin2Diff, four), preciseQuo(sin4Diff, preciseFloat64(32))),
		intSin:  preciseSub(w.cos0, w.cos1),
		intSin2: preciseSub(halfDth, preciseQuo(sin2Diff, four)),
		intSin3: preciseSub(
			preciseSub(w.cos0, preciseQuo(cube(w.cos0), three)),
			preciseSub(w.cos1, preciseQuo(cube(w.cos1), three)),
		),
		intSin4: preciseAdd(
			preciseSub(threeEighthsDth, preciseQuo(sin2Diff, four)),
			preciseQuo(sin4Diff, preciseFloat64(32)),
		),
		intSC:  preciseQuo(preciseSub(preciseMul(w.sin1, w.sin1), preciseMul(w.sin0, w.sin0)), two),
		intSC2: preciseQuo(preciseSub(cube(w.cos0), cube(w.cos1)), three),
		intSC3: preciseQuo(preciseSub(quartic(w.cos0), quartic(w.cos1)), four),
	}
}

// preciseGreenArcFirstMoments is preciseGreenArcArea's first-moment sibling:
// addCircular's own mu/mv trig closed forms over the denoted arc plus addLine's
// over End→Center and Center→Start, all in 256-bit arithmetic. It evaluates the
// trig closed form independently of the rational substitution the bracket
// under test makes.
func preciseGreenArcFirstMoments(t *testing.T, center, start, end decad.Point2) (*big.Float, *big.Float) {
	t.Helper()
	w := newPreciseArcWedge(t, center, start, end)
	in := w.trigIntegrals()
	two, half := preciseFloat64(2), preciseFloat64(0.5)
	mu := preciseMul(half, w.r, preciseAdd(
		preciseMul(w.cU, w.cU, in.intCos),
		preciseMul(two, w.cU, w.r, in.intCos2),
		preciseMul(w.r, w.r, in.intCos3),
	))
	mv := preciseMul(half, w.r, preciseAdd(
		preciseMul(w.cV, w.cV, in.intSin),
		preciseMul(two, w.cV, w.r, in.intSin2),
		preciseMul(w.r, w.r, in.intSin3),
	))
	for _, line := range [][4]*big.Float{
		{w.endU, w.endV, w.cU, w.cV},
		{w.cU, w.cV, w.startU, w.startV},
	} {
		lm := preciseLineMoments(line[0], line[1], line[2], line[3])
		mu = preciseAdd(mu, lm.mu)
		mv = preciseAdd(mv, lm.mv)
	}
	return mu, mv
}

// preciseGreenArcSecondMoments is the second-moment sibling of
// preciseGreenArcFirstMoments: addCircular's own muu/muv/mvv trig closed forms
// plus addLine's, in 256-bit arithmetic.
func preciseGreenArcSecondMoments(t *testing.T, center, start, end decad.Point2) (*big.Float, *big.Float, *big.Float) {
	t.Helper()
	w := newPreciseArcWedge(t, center, start, end)
	in := w.trigIntegrals()
	two, three := preciseFloat64(2), preciseFloat64(3)
	cU, cV, r := w.cU, w.cV, w.r
	muu := preciseMul(preciseQuo(r, three), preciseAdd(
		preciseMul(cU, cU, cU, in.intCos),
		preciseMul(three, cU, cU, r, in.intCos2),
		preciseMul(three, cU, r, r, in.intCos3),
		preciseMul(r, r, r, in.intCos4),
	))
	mvv := preciseMul(preciseQuo(r, three), preciseAdd(
		preciseMul(cV, cV, cV, in.intSin),
		preciseMul(three, cV, cV, r, in.intSin2),
		preciseMul(three, cV, r, r, in.intSin3),
		preciseMul(r, r, r, in.intSin4),
	))
	muv := preciseMul(preciseFloat64(0.5), r, preciseAdd(
		preciseMul(cV, preciseAdd(
			preciseMul(cU, cU, in.intCos),
			preciseMul(two, cU, r, in.intCos2),
			preciseMul(r, r, in.intCos3),
		)),
		preciseMul(r, preciseAdd(
			preciseMul(cU, cU, in.intSC),
			preciseMul(two, cU, r, in.intSC2),
			preciseMul(r, r, in.intSC3),
		)),
	))
	for _, line := range [][4]*big.Float{
		{w.endU, w.endV, w.cU, w.cV},
		{w.cU, w.cV, w.startU, w.startV},
	} {
		lm := preciseLineMoments(line[0], line[1], line[2], line[3])
		muu = preciseAdd(muu, lm.muu)
		muv = preciseAdd(muv, lm.muv)
		mvv = preciseAdd(mvv, lm.mvv)
	}
	return muu, muv, mvv
}

// driftedArcWedge is the wedge record the arc-endpoint-drift tests share: an
// arc about (5, −3) from angle 0 on the unit circle to End, one ulp of the
// radius off that circle, closed by End→Center→Start.
func driftedArcWedge() (momentinput.Profile, decad.Point2, decad.Point2, decad.Point2) {
	center := decad.Point2{U: 5, V: -3}
	start := decad.Point2{U: 6, V: -3}
	driftedRadius := math.Nextafter(1, math.Inf(1))
	end := decad.Point2{
		U: 5 + 0.6*driftedRadius,
		V: -3 + 0.8*driftedRadius,
	}
	record := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: end, End: center, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: center, End: start, TStart: 0, TEnd: 1},
	}}}
	return record, center, start, end
}

func requireArcRadiiDiffer(t *testing.T, center, start, end decad.Point2) {
	t.Helper()
	floatRat := func(value float64) *big.Rat { return new(big.Rat).SetFloat64(value) }
	squared := func(p decad.Point2) *big.Rat {
		du := new(big.Rat).Sub(floatRat(p.U), floatRat(center.U))
		dv := new(big.Rat).Sub(floatRat(p.V), floatRat(center.V))
		return new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	}
	require.NotEqual(t, 0, squared(end).Cmp(squared(start)), `the fixture's exact squared radii must differ`)
}

func TestRegionAreaAndCentroidRectangle(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(10, 20, 110, 80)
	s.Fix(rect.A)
	record := recordOne(t, s, func(profile *sketch.Profile) bool { return len(profile.Outer) == 4 })

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Zero(t, area.Bound.Mag())
	require.True(t, area.Value.Equal(units.SquareMillimeters(6000), 1e-9))

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, centroid.Exactness)
	require.InDelta(t, 60, centroid.Value.X, 1e-9)
	require.InDelta(t, 50, centroid.Value.Y, 1e-9)
	require.Zero(t, centroid.Value.Z)
}

func TestRegionAreaAndCentroidWithHole(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	record := recordOne(t, s, func(profile *sketch.Profile) bool { return len(profile.Holes) == 1 })

	holeArea := math.Pi * 100
	wantArea := 6000 - holeArea
	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantArea, got, 1e-9)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.InDelta(t, (6000*50-holeArea*70)/wantArea, centroid.Value.X, 1e-9)
	require.InDelta(t, 30, centroid.Value.Y, 1e-9)
}

func TestRegionAreaWholeCircle(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(5, -3)
	s.Fix(center)
	s.CreateCircle(center, 7)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*49, got, 1e-9)
	wantArea := new(big.Float).SetPrec(256).Mul(
		precisePi(t),
		new(big.Float).SetPrec(256).SetFloat64(49),
	)
	requireBoundContainsBig(t, got, area.Bound.Base(), wantArea)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, centroid.Exactness)
	require.Positive(t, centroid.Bound.Base())
	require.InDelta(t, 5.0, centroid.Value.X, 1e-9)
	require.InDelta(t, -3.0, centroid.Value.Y, 1e-9)
	require.LessOrEqual(t, math.Hypot(centroid.Value.X-5, centroid.Value.Y+3), centroid.Bound.Base())
}

func TestRegionAreaMatchesSketchOnCertifiedFragments(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(95, 30), 15)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	checked := 0
	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		record, _, err := momentinput.RecordProfile(s, profile)
		require.NoError(t, err)
		area, err := record.Area()
		require.NoError(t, err)
		got, err := area.Value.In(units.SquareMillimeter)
		require.NoError(t, err)
		require.InDelta(t, profile.Area, got, 1e-9)
		checked++
	}
	require.GreaterOrEqual(t, checked, 3)
}

func TestRegionMomentsRejectUnsupportedAndEmptyRecords(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateEllipse(center, 20, 10, 0)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })
	requireProfileMomentError(t, record, decad.ErrUnsupported)

	requireProfileMomentError(t, momentinput.Profile{}, decad.ErrDegenerate)
}

func TestCircleSegMomentRadiusErrorsUseDecadSentinels(t *testing.T) {
	t.Parallel()
	calls := []struct {
		name string
		call func(momentinput.Profile) error
	}{
		{
			name: "area",
			call: func(rec momentinput.Profile) error {
				_, err := rec.Area()
				return err
			},
		},
		{
			name: "centroid",
			call: func(rec momentinput.Profile) error {
				_, err := rec.Centroid()
				return err
			},
		},
		{
			name: "second moments",
			call: func(rec momentinput.Profile) error {
				_, err := rec.SecondMoments()
				return err
			},
		},
	}
	radii := []struct {
		name       string
		radius     units.Value
		want       error
		dependency error
	}{
		{
			name:       "wrong kind",
			radius:     units.Degrees(1),
			want:       decad.ErrUnitKind,
			dependency: units.ErrIncompatible,
		},
		{
			name:       "NaN",
			radius:     units.Millimeters(math.NaN()),
			want:       decad.ErrNotFinite,
			dependency: units.ErrNotFinite,
		},
		{
			name:       "infinite",
			radius:     units.Millimeters(math.Inf(1)),
			want:       decad.ErrNotFinite,
			dependency: units.ErrNotFinite,
		},
		{
			name:       "conversion overflow",
			radius:     units.Meters(math.MaxFloat64),
			want:       decad.ErrNotFinite,
			dependency: units.ErrNotFinite,
		},
	}
	forms := []struct {
		name string
		seg  func(units.Value) sectionrecord.CurveSegment
	}{
		{
			name: "value",
			seg: func(radius units.Value) sectionrecord.CurveSegment {
				return sectionrecord.CircleSeg{Radius: radius, CCW: true, TEnd: 1}
			},
		},
		{
			name: "pointer",
			seg: func(radius units.Value) sectionrecord.CurveSegment {
				return &sectionrecord.CircleSeg{Radius: radius, CCW: true, TEnd: 1}
			},
		},
	}

	for _, radius := range radii {
		for _, form := range forms {
			rec := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				form.seg(radius.radius),
			}}}
			for _, call := range calls {
				t.Run(radius.name+"/"+form.name+"/"+call.name, func(t *testing.T) {
					err := call.call(rec)
					require.ErrorIs(t, err, radius.want)
					require.NotErrorIs(t, err, radius.dependency,
						`mass properties expose decad's sentinel, not the units dependency error`)
				})
			}
		}
	}
}

func TestRegionMomentsPointerVariants(t *testing.T) {
	t.Parallel()
	record := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		&sectionrecord.LineSeg{Start: decad.Point2{}, End: decad.Point2{U: 4}, TStart: 0, TEnd: 1},
		&sectionrecord.LineSeg{Start: decad.Point2{U: 4}, End: decad.Point2{U: 4, V: 4}, TStart: 0, TEnd: 1},
		&sectionrecord.LineSeg{Start: decad.Point2{U: 4, V: 4}, End: decad.Point2{V: 4}, TStart: 0, TEnd: 1},
		&sectionrecord.LineSeg{Start: decad.Point2{V: 4}, End: decad.Point2{}, TStart: 0, TEnd: 1},
	}}}
	area, err := record.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(16), 1e-12))

	bad := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{(*sectionrecord.LineSeg)(nil)}}}
	requireProfileMomentError(t, bad, decad.ErrDegenerate)
	bad.Outer.Segments[0] = nil
	requireProfileMomentError(t, bad, decad.ErrDegenerate)
}

func TestRegionMomentsRejectMalformedFields(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name   string
		record momentinput.Profile
		target error
	}{
		{
			name: "OpenLoop",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				momentLine(0, 0, 1, 0), momentLine(1, 0, 1, 1),
			}}},
			target: decad.ErrDegenerate,
		},
		{
			name: "NonFiniteCoordinate",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				momentLine(math.NaN(), 0, 1, 0),
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "NonFiniteRange",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.LineSeg{Start: decad.Point2{}, End: decad.Point2{U: 1}, TEnd: math.Inf(1)},
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "WrongRadiusUnit",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{Radius: units.Degrees(1), CCW: true, TEnd: 1},
			}}},
			target: decad.ErrUnitKind,
		},
		{
			name: "NonFiniteRadius",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{Radius: units.Millimeters(math.Inf(1)), CCW: true, TEnd: 1},
			}}},
			target: decad.ErrNotFinite,
		},
		{
			name: "NegativeRadius",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{Radius: units.Millimeters(-1), CCW: true, TEnd: 1},
			}}},
			target: decad.ErrNegativeMagnitude,
		},
		{
			name: "InconsistentArcPins",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.ArcSeg{
					Center: decad.Point2{}, Start: decad.Point2{U: 1}, End: decad.Point2{V: 2}, TEnd: 1,
				},
				momentLine(0, 1, 1, 0),
			}}},
			target: decad.ErrDegenerate,
		},
		{
			name: "NearlyFullCircle",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{
					Radius: units.Millimeters(1), CCW: true, TEnd: math.Nextafter(1, 0),
				},
			}}},
			target: decad.ErrDegenerate,
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			requireProfileMomentError(t, test.record, test.target)
		})
	}
}

func TestRegionMomentsRejectMalformedTopology(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name   string
		record momentinput.Profile
	}{
		{
			name: "CrossingOuter",
			record: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				momentLine(0, 0, 4, 4),
				momentLine(4, 4, 0, 4),
				momentLine(0, 4, 4, 0),
				momentLine(4, 0, 0, 0),
			}}},
		},
		{name: "WrongOuterWinding", record: momentinput.Profile{Outer: momentSquare(0, 0, 10, 10, true)}},
		{
			name: "HoleOutsideOuter",
			record: momentinput.Profile{
				Outer: momentSquare(0, 0, 10, 10, false),
				Holes: []sectionrecord.LoopRecord{
					momentSquare(12, 2, 13, 3, true),
				},
			},
		},
		{
			name: "OverlappingHoles",
			record: momentinput.Profile{
				Outer: momentSquare(0, 0, 10, 10, false),
				Holes: []sectionrecord.LoopRecord{
					momentSquare(2, 2, 6, 6, true),
					momentSquare(4, 4, 8, 8, true),
				},
			},
		},
		{
			name: "OuterHoleInternalTangency",
			record: momentinput.Profile{
				Outer: momentWholeCircle(decad.Point2{}, 10, true),
				Holes: []sectionrecord.LoopRecord{
					momentWholeCircle(decad.Point2{U: 5}, 5, false),
				},
			},
		},
		{
			name: "HoleHoleExternalTangency",
			record: momentinput.Profile{
				Outer: momentWholeCircle(decad.Point2{}, 10, true),
				Holes: []sectionrecord.LoopRecord{
					momentWholeCircle(decad.Point2{U: -2}, 2, false),
					momentWholeCircle(decad.Point2{U: 2}, 2, false),
				},
			},
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			requireProfileMomentError(t, test.record, decad.ErrDegenerate)
		})
	}
}

func TestRegionMomentsAcceptThinAnnulus(t *testing.T) {
	t.Parallel()
	const outerRadius = 10.0
	const gap = 1e-10
	holeRadius := outerRadius - gap
	record := momentinput.Profile{
		Outer: momentWholeCircle(decad.Point2{}, outerRadius, true),
		Holes: []sectionrecord.LoopRecord{
			momentWholeCircle(decad.Point2{}, holeRadius, false),
		},
	}

	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*(outerRadius*outerRadius-holeRadius*holeRadius), got, 1e-12)
	_, err = record.Centroid()
	require.NoError(t, err)
	_, err = record.SecondMoments()
	require.NoError(t, err)
}

func TestRegionMomentsAcceptSeparatedWholeCircleHoles(t *testing.T) {
	t.Parallel()
	record := momentinput.Profile{
		Outer: momentWholeCircle(decad.Point2{}, 10, true),
		Holes: []sectionrecord.LoopRecord{
			momentWholeCircle(decad.Point2{U: -2.0000000001}, 2, false),
			momentWholeCircle(decad.Point2{U: 2.0000000001}, 2, false),
		},
	}

	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 92*math.Pi, got, 1e-12)
	_, err = record.Centroid()
	require.NoError(t, err)
	_, err = record.SecondMoments()
	require.NoError(t, err)
}

func TestRegionMomentsRequestedOrderControlsOverflow(t *testing.T) {
	t.Parallel()
	record := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{
			Center: decad.Point2{V: 1e200},
			Radius: units.Millimeters(1),
			CCW:    true,
			TEnd:   1,
		},
	}}}

	area, err := record.Area()
	require.NoError(t, err)
	require.True(t, area.Value.Equal(units.SquareMillimeters(math.Pi), 1e-12))
	_, err = record.Centroid()
	require.NoError(t, err)
	_, err = record.SecondMoments()
	require.ErrorIs(t, err, decad.ErrNotFinite)
}

// The measured defect behind publishExact's per-field publication. The exact
// rational accumulator holds all six moments whatever order the caller asked
// for, so a region whose SECOND moment has no float64 image still has an exact
// area rational in hand. Publishing the six together abandoned every one of
// them on the first overflow, and Area fell back to the sum of its per-segment
// float roundings: one ulp off the correctly rounded area, carrying a bound past
// the half ulp spline design §3 promises unconditionally. Each field publishes on
// its own now, so Area carries its single rounding while SecondMoments still
// refuses honestly.
//
// The polygon is LineSeg-only on purpose: the accumulator is shared with the
// Tier A free-form path, and the defect was never specific to it.
func TestOverflowingSecondMomentKeepsExactArea(t *testing.T) {
	t.Parallel()
	// 1e78 mm squares to a finite area and raises the second moment past
	// float64's range. No plausible model reaches this scale; the guarantee is
	// unconditional, so it is asserted where it is reachable at all.
	const scale = 1e78
	polygon := []decad.Point2{{}, {U: 0.1 * scale}, {U: 0.3 * scale, V: 0.2 * scale}, {U: 0.05 * scale, V: 0.1 * scale}, {V: 0.4 * scale}}
	segments := make([]sectionrecord.CurveSegment, len(polygon))
	for i, corner := range polygon {
		next := polygon[(i+1)%len(polygon)]
		segments[i] = momentLine(corner.U, corner.V, next.U, next.V)
	}
	record := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}

	// The falsifier: the polygon's own shoelace over exact rationals.
	exact := new(big.Rat)
	for i, corner := range polygon {
		next := polygon[(i+1)%len(polygon)]
		au, av := new(big.Rat).SetFloat64(corner.U), new(big.Rat).SetFloat64(corner.V)
		bu, bv := new(big.Rat).SetFloat64(next.U), new(big.Rat).SetFloat64(next.V)
		exact.Add(exact, new(big.Rat).Sub(new(big.Rat).Mul(au, bv), new(big.Rat).Mul(bu, av)))
	}
	exact.Quo(exact, big.NewRat(2, 1))

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness, "this area is not representable in float64")
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	requireSingleRounding(t, exact, value, bound)

	_, err = record.SecondMoments()
	require.ErrorIs(t, err, decad.ErrNotFinite,
		"the second moment has no float64 image and is still refused, not published")
}

func TestSecondMomentsRectangle(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 30)
	s.Fix(rect.A)
	record := recordOne(t, s, func(profile *sketch.Profile) bool { return len(profile.Outer) == 4 })
	measured, err := decad.MeasureProfile(s, s.Profiles()[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.Equal(t, units.SquareMillimeters(1200), area.Value)
	centroid, err := measured.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 20, centroid.Value.X, 1e-12)
	require.InDelta(t, 15, centroid.Value.Y, 1e-12)

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	measuredMoments, err := measured.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, moments, measuredMoments)
	require.Equal(t, decad.Exact, moments.UU.Exactness)
	require.Equal(t, decad.Exact, moments.UV.Exactness)
	require.Equal(t, decad.Exact, moments.VV.Exactness)
	require.Zero(t, moments.UU.Bound.Base())
	require.Zero(t, moments.UV.Bound.Base())
	require.Zero(t, moments.VV.Bound.Base())
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := moments.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := moments.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 40.0*40*40*30/3, uu, 1e-6)
	require.InDelta(t, 40.0*40*30*30/4, uv, 1e-6)
	require.InDelta(t, 40.0*30*30*30/3, vv, 1e-6)
}

func TestSecondMomentsOffsetCircle(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(5, -3)
	s.Fix(center)
	s.CreateCircle(center, 7)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, moments.UU.Exactness)
	require.Equal(t, decad.Approximate, moments.UV.Exactness)
	require.Equal(t, decad.Approximate, moments.VV.Exactness)
	require.Positive(t, moments.UU.Bound.Base())
	require.Positive(t, moments.UV.Bound.Base())
	require.Positive(t, moments.VV.Bound.Base())
	area := math.Pi * 49
	quarter := math.Pi * 7 * 7 * 7 * 7 / 4
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := moments.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := moments.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, quarter+25*area, uu, 1e-6)
	require.InDelta(t, -15*area, uv, 1e-6)
	require.InDelta(t, quarter+9*area, vv, 1e-6)
}

// TestSecondMomentsSemicircleBoundTightens is design §15's T111:
// semicircleSketch's own profile record published a UV bound of 38385.42 mm⁴
// against a 416.67 mm⁴ value before this fix — 92 times the value itself,
// never mind hundreds — and UU/VV were wider still (188201.04 against
// 1227.18, 98751.38 against 245.44). moments_circular.go's
// circularSecondMomentInterval now brackets all three exactly, so every
// bound shrinks to a tiny fraction of its own value.
//
// Shown-to-fail: forcing circularSecondMomentInterval to answer ok == false
// reproduces the old three magnitudes and turns every assertion below red —
// verified by hand, since the toggle lives in unexported production code no
// external test can reach.
func TestSecondMomentsSemicircleBoundTightens(t *testing.T) {
	t.Parallel()
	s, _ := semicircleSketch(t)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	checkTight := func(name string, m decad.Measurement) {
		t.Helper()
		value, err := m.Value.In(units.QuarticMillimeter)
		require.NoError(t, err)
		bound, err := m.Bound.In(units.QuarticMillimeter)
		require.NoError(t, err)
		require.Positive(t, bound, "%s: the bound should still be positive (the true value carries pi)", name)
		require.LessOrEqual(t, bound, 1e-6*math.Abs(value),
			"%s: the bound %g mm^4 is not tight against the value %g mm^4", name, bound, value)
	}
	checkTight("UU", moments.UU)
	checkTight("UV", moments.UV)
	checkTight("VV", moments.VV)
}

func TestLineRationalRoundingIsBounded(t *testing.T) {
	t.Parallel()
	rec := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: decad.Point2{U: 1, V: 0}, End: decad.Point2{U: 0, V: 1}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: decad.Point2{U: 0, V: 1}, End: decad.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}

	area, err := rec.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Zero(t, area.Bound.Base())

	centroid, err := rec.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, centroid.Exactness)
	require.Positive(t, centroid.Bound.Base())
	want := 1.0 / 3
	require.LessOrEqual(
		t,
		math.Hypot(centroid.Value.X-want, centroid.Value.Y-want),
		centroid.Bound.Base(),
	)

	moments, err := rec.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, moments.UU.Exactness)
	got, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(got-1.0/12), moments.UU.Bound.Base())
}

// The underflow reading is not free-form-specific: the line path integrates to
// exact rationals too, so a square of four LineSegs whose exact area is strictly
// positive and below the smallest float64 owes the same bounded zero — value 0
// with the rounding that produced it as the bound — rather than a refusal for
// enclosing no positive area.
func TestUnderflowingLineRegionAreaPublishesBoundedZero(t *testing.T) {
	t.Parallel()
	const side = 1e-163
	record := momentinput.Profile{Outer: momentSquare(0, 0, side, side, false)}

	area, err := record.Area()
	require.NoError(t, err, "the exact rational area is side², which is strictly positive")
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Zero(t, value, "no float64 holds 1e-326")
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base(), "the bound is the rounding that produced the zero")
}

func TestArcSegExactQuarterDisk(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	px := s.CreatePoint(20, 0)
	py := s.CreatePoint(0, 20)
	s.CreateLine(origin, px)
	s.CreateLine(py, origin)
	s.CreateArc(origin, px, py)
	record := recordOne(t, s, func(*sketch.Profile) bool { return true })

	const radius = 20.0
	area, err := record.Area()
	require.NoError(t, err)
	gotArea, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*radius*radius/4, gotArea, 1e-9)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())
	require.InDelta(t, 4*radius/(3*math.Pi), centroid.Value.X, 1e-9)
	require.InDelta(t, 4*radius/(3*math.Pi), centroid.Value.Y, 1e-9)
	wantArea := new(big.Float).SetPrec(256).Mul(
		precisePi(t),
		new(big.Float).SetPrec(256).SetFloat64(radius*radius/4),
	)
	requireBoundContainsBig(t, gotArea, area.Bound.Base(), wantArea)

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, centroid.Exactness)
	require.Positive(t, centroid.Bound.Base())
	wantCentroid := 4 * radius / (3 * math.Pi)
	require.LessOrEqual(t, math.Hypot(centroid.Value.X-wantCentroid, centroid.Value.Y-wantCentroid), centroid.Bound.Base())
	require.Equal(t, decad.Approximate, moments.UU.Exactness)
	// UV collapses to a rational with no swept-angle term at all (center on
	// the origin cancels muv's own dth coefficient,
	// moments_circular.go's circularSecondMomentInterval doc comment), so it
	// is Exact with a zero bound — unlike UU/VV, whose true value carries pi.
	require.Equal(t, decad.Exact, moments.UV.Exactness)
	require.Zero(t, moments.UV.Bound.Base())
	require.Equal(t, decad.Approximate, moments.VV.Exactness)
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	uv, err := moments.UV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	vv, err := moments.VV.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, math.Pi*math.Pow(radius, 4)/16, uu, 1e-6)
	require.InDelta(t, math.Pow(radius, 4)/8, uv, 1e-6)
	require.InDelta(t, math.Pi*math.Pow(radius, 4)/16, vv, 1e-6)
	require.LessOrEqual(t, math.Abs(uu-math.Pi*math.Pow(radius, 4)/16), moments.UU.Bound.Base())
	require.LessOrEqual(t, math.Abs(uv-math.Pow(radius, 4)/8), moments.UV.Bound.Base())
	require.LessOrEqual(t, math.Abs(vv-math.Pi*math.Pow(radius, 4)/16), moments.VV.Bound.Base())
}

func TestRegionMomentsAcceptsGeneratedArcEndpointDrift(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(1.23456789, -2.34567891)
	s.Fix(center)
	start := s.CreatePoint(17.89101112, 4.32109876)
	s.Fix(start)
	end := s.CreatePoint(3.33333333, 18.7654321)
	s.CreateLine(center, start)
	s.CreateArc(center, start, end)
	s.CreateLine(end, center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		record, _, err := momentinput.RecordProfile(s, profile)
		require.NoError(t, err)
		_, err = record.Centroid()
		require.NoError(t, err)
		return
	}
	t.Fatal("no valid profile generated")
}

func TestRegionAreaBoundContainsArcEndpointDriftGreenIntegral(t *testing.T) {
	t.Parallel()
	center := decad.Point2{U: 5, V: -3}
	start := decad.Point2{U: 6, V: -3}
	driftedRadius := math.Nextafter(1, math.Inf(1))
	end := decad.Point2{
		U: 5 + 0.6*driftedRadius,
		V: -3 + 0.8*driftedRadius,
	}
	record := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: end, End: center, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: center, End: start, TStart: 0, TEnd: 1},
	}}}

	area, err := record.Area()
	require.NoError(t, err)
	got, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	want := preciseGreenArcArea(t, center, start, end)
	requireBoundContainsBig(t, got, area.Bound.Base(), want)
}

// Shown-to-fail: restoring the `if endR2.Cmp(r2) != 0 { return …, false }`
// bail-out in circularFirstMomentInterval's ArcSeg arm leaves the arc's first
// moments to the magnitude envelope, and the 1e-9 ceiling leg goes red.
func TestRegionCentroidBoundContainsArcEndpointDriftGreenIntegral(t *testing.T) {
	t.Parallel()
	record, center, start, end := driftedArcWedge()
	requireArcRadiiDiffer(t, center, start, end)

	centroid, err := record.Centroid()
	require.NoError(t, err)
	area := preciseGreenArcArea(t, center, start, end)
	mu, mv := preciseGreenArcFirstMoments(t, center, start, end)
	bound := centroid.Bound.Base()
	requireBoundContainsBig(t, centroid.Value.X, bound, preciseQuo(mu, area))
	requireBoundContainsBig(t, centroid.Value.Y, bound, preciseQuo(mv, area))
	require.LessOrEqual(t, bound, 1e-9, `a drifted End is charged into the bracket, not left to the envelope`)
}

// Shown-to-fail: restoring the `if endR2.Cmp(r2) != 0 { return …, false }`
// bail-out in circularSecondMomentInterval's ArcSeg arm leaves the arc's
// second moments to the magnitude envelope, and the 1e-9 ceiling legs go red.
func TestRegionSecondMomentsBoundContainArcEndpointDriftGreenIntegral(t *testing.T) {
	t.Parallel()
	record, center, start, end := driftedArcWedge()
	requireArcRadiiDiffer(t, center, start, end)

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	muu, muv, mvv := preciseGreenArcSecondMoments(t, center, start, end)
	for _, tc := range []struct {
		name string
		m    decad.Measurement
		want *big.Float
	}{
		{"UU", moments.UU, muu},
		{"UV", moments.UV, muv},
		{"VV", moments.VV, mvv},
	} {
		got, err := tc.m.Value.In(units.QuarticMillimeter)
		require.NoError(t, err, tc.name)
		bound := tc.m.Bound.Base()
		requireBoundContainsBig(t, got, bound, tc.want)
		require.LessOrEqual(t, bound, 1e-9, "%s: a drifted End is charged into the bracket, not left to the envelope", tc.name)
	}
}

// filletCornerRecord is a 96×20 rectangle about the origin whose (48, −10)
// corner is rounded by an arc of radius r: the centre is (48 − r, −10 + r) in
// float arithmetic and the two feet sit on the rectangle's own sides. 48 and
// 10 sit in different binades, so the two subtractions round r to different
// grids and the pinned radii differ by ulps of the COORDINATE — the same
// rounding a fillet's or an outward shell's corner arc carries. endU moves the
// arc's End (and the following line's Start) along u.
func filletCornerRecord(r, endU float64) (momentinput.Profile, sectionrecord.ArcSeg) {
	oU := 48 - r
	oV := -10 + r
	arc := sectionrecord.ArcSeg{
		Center: decad.Point2{U: oU, V: oV},
		Start:  decad.Point2{U: oU, V: -10},
		End:    decad.Point2{U: endU, V: oV},
		TStart: 0,
		TEnd:   1,
	}
	record := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		arc,
		momentLine(endU, oV, 48, 10),
		momentLine(48, 10, -48, 10),
		momentLine(-48, 10, -48, -10),
		momentLine(-48, -10, oU, -10),
	}}}
	return record, arc
}

// Shown-to-fail: the acceptance leg goes red with the radius-anchored
// tolerance (momentCoordinateJoins, 1024 ulps of the radius) restored in
// place of arcPinnedRadiiJoin; the refusal leg's reason assertion goes red
// with arcPinnedRadiiJoin made to return true unconditionally.
func TestRegionMomentsArcPinToleranceFollowsCoordinateScale(t *testing.T) {
	t.Parallel()
	const r = 0.001

	record, arc := filletCornerRecord(r, 48)
	requireArcRadiiDiffer(t, arc.Center, arc.Start, arc.End)
	startRadius := math.Hypot(arc.Start.U-arc.Center.U, arc.Start.V-arc.Center.V)
	endRadius := math.Hypot(arc.End.U-arc.Center.U, arc.End.V-arc.Center.V)
	radiusScale := math.Max(startRadius, endRadius)
	radiusULP := radiusScale - math.Nextafter(radiusScale, 0)
	require.Greater(t, math.Abs(startRadius-endRadius), 1024*radiusULP,
		`the radii must differ by more than 1024 ulps of the radius itself`)

	area, err := record.Area()
	require.NoError(t, err)
	gotArea, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 96*20-(1-math.Pi/4)*r*r, gotArea, 1e-9)
	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.LessOrEqual(t, centroid.Bound.Base(), 1e-9)
	_, err = record.SecondMoments()
	require.NoError(t, err)

	// The moved End still closes the loop, but the record no longer matches
	// what sketch rebuilds from it either; the reason asserted here pins the
	// refusal to the radius check, which reads it first.
	refused, _ := filletCornerRecord(r, 48+1e-6)
	requireProfileMomentError(t, refused, decad.ErrDegenerate)
	_, err = refused.Area()
	require.ErrorContains(t, err, `pinned start and end radii differ`)
}
