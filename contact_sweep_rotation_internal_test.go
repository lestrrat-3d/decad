package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// cornerSpan and pointDeviation evaluate their interval expressions in the
// common-denominator form (motion_bound.go's scaledIvMat). These tests hold
// that form to the big.Rat evaluation of the same expressions, kept below as
// cornerSpanRational and pointDeviationRational: every endpoint, hull and bound
// must be the identical rational, not merely an enclosing one.

// cornerSpanRational is cornerSpan's expression evaluated one big.Rat
// operation at a time.
func cornerSpanRational(p rotationalSweepPath, from, to *big.Rat) []ivVec {
	output := make([]ivVec, len(p.startPoints))
	if p.path.drift == nil {
		for index, corner := range p.startPoints {
			for axis := range 3 {
				start := corner[axis].Rat()
				lo := new(big.Rat).Mul(p.path.delta[axis].Rat(), from)
				hi := new(big.Rat).Mul(p.path.delta[axis].Rat(), to)
				if lo.Cmp(hi) > 0 {
					lo, hi = hi, lo
				}
				output[index][axis] = proofbound.Interval(new(big.Rat).Add(start, lo), new(big.Rat).Add(start, hi))
			}
		}
		return output
	}
	lowTime := new(big.Rat).Mul(p.path.duration, from)
	highTime := new(big.Rat).Mul(p.path.duration, to)
	lowAngle := new(big.Rat).Mul(p.omegaLow, lowTime)
	highAngle := new(big.Rat).Mul(p.omegaHigh, highTime)
	sin, cos := rotationalSinCosSpan(lowAngle, highAngle)
	rotationSpan := p.frame.rotation(sin, cos)
	midTime := new(big.Rat).Quo(new(big.Rat).Add(lowTime, highTime), big.NewRat(2, 1))
	angleAtMidLow := new(big.Rat).Mul(p.omegaLow, midTime)
	angleAtMidHigh := new(big.Rat).Mul(p.omegaHigh, midTime)
	midSin, midCos := rotationalSinCosSpan(angleAtMidLow, angleAtMidHigh)
	rotationMid := p.frame.rotation(midSin, midCos)
	halfDuration := new(big.Rat).Quo(new(big.Rat).Sub(highTime, lowTime), big.NewRat(2, 1))
	pivot := pointVec(p.frame.center)
	for index, corner := range p.startPoints {
		start := ratVec{corner[0].Rat(), corner[1].Rat(), corner[2].Rat()}
		relative := ivVecSub(pointVec(start), pivot)
		spanRelative := rotationSpan.apply(relative)
		point := ivVecAdd(rotationMid.apply(relative), pivot)
		for axis := range 3 {
			point[axis] = proofbound.IntervalAdd(point[axis],
				proofbound.PointInterval(new(big.Rat).Mul(p.velocity[axis], midTime)))
			following, preceding := (axis+1)%3, (axis+2)%3
			derivative := proofbound.IntervalAdd(proofbound.PointInterval(p.velocity[axis]), proofbound.IntervalSub(
				proofbound.IntervalScale(spanRelative[preceding], p.frame.axis[following]),
				proofbound.IntervalScale(spanRelative[following], p.frame.axis[preceding])))
			maximum := new(big.Rat).Abs(derivative.Lo)
			if other := new(big.Rat).Abs(derivative.Hi); other.Cmp(maximum) > 0 {
				maximum = other
			}
			travel := new(big.Rat).Mul(maximum, halfDuration)
			output[index][axis] = proofbound.IntervalOwned(new(big.Rat).Sub(point[axis].Lo, travel),
				new(big.Rat).Add(point[axis].Hi, travel))
		}
	}
	return output
}

// idealAtRational is idealAt's ideal pose evaluated one big.Rat operation at
// a time.
func idealAtRational(p rotationalSweepPath, f *big.Rat) idealPose {
	zero := proofbound.PointInterval(new(big.Rat))
	if p.path.drift == nil {
		shift := pointVec(p.fromT)
		for axis := range 3 {
			shift[axis] = proofbound.IntervalAdd(shift[axis],
				proofbound.PointInterval(new(big.Rat).Mul(p.path.delta[axis].Rat(), f)))
		}
		return idealPose{rot: p.fromRot, pivot: ivVec{zero, zero, zero}, shift: shift}
	}
	elapsed := new(big.Rat).Mul(p.path.duration, f)
	angleLow := new(big.Rat).Mul(p.omegaLow, elapsed)
	angleHigh := new(big.Rat).Mul(p.omegaHigh, elapsed)
	sin, cos := radianSinCos(angleLow)
	width := new(big.Rat).Sub(angleHigh, angleLow)
	sin = proofbound.IntervalOwned(new(big.Rat).Sub(sin.Lo, width), new(big.Rat).Add(sin.Hi, width))
	cos = proofbound.IntervalOwned(new(big.Rat).Sub(cos.Lo, width), new(big.Rat).Add(cos.Hi, width))
	rot := p.frame.rotation(sin, cos)
	center := pointVec(p.frame.center)
	shift := ivVecAdd(rot.apply(ivVecSub(pointVec(p.fromT), center)), center)
	for axis := range 3 {
		shift[axis] = proofbound.IntervalAdd(shift[axis],
			proofbound.PointInterval(new(big.Rat).Mul(p.velocity[axis], elapsed)))
	}
	return idealPose{rot: rot.mul(p.fromRot), pivot: ivVec{zero, zero, zero}, shift: shift}
}

// transferChargeRational is transferCharge evaluated one big.Rat operation at
// a time against idealAtRational's enclosure.
func transferChargeRational(p rotationalSweepPath, pose r3.Transform, f *big.Rat) (*big.Rat, bool) {
	if p.delta.Sign() == 0 {
		return new(big.Rat), true
	}
	ideal := idealAtRational(p, f)
	rounded, _, ok := exactTransform(pose)
	if !ok {
		return nil, false
	}
	entries := make([]proofbound.RatInterval, 0, 9)
	for i := range 3 {
		for k := range 3 {
			entries = append(entries, proofbound.IntervalSub(rounded[i][k], ideal.rot[i][k]))
		}
	}
	norm := proofbound.RatSqrtUp(magnitudeSquaredUpper(entries...))
	if !finiteMeasurementValues(norm) {
		return nil, false
	}
	return new(big.Rat).Mul(proofarith.FloatRat(norm), p.delta.Rat()), true
}

// requireScaledMatrix asserts that a common-denominator matrix holds exactly
// the rational intervals of want.
func requireScaledMatrix(t *testing.T, want ivMat, got scaledIvMat, msg string) {
	t.Helper()
	for i := range 3 {
		for j := range 3 {
			lo, hi := new(big.Rat).SetFrac(got.lo[i][j], got.den), new(big.Rat).SetFrac(got.hi[i][j], got.den)
			require.Zero(t, lo.Cmp(want[i][j].Lo), "%s entry (%d, %d)", msg, i, j)
			require.Zero(t, hi.Cmp(want[i][j].Hi), "%s entry (%d, %d)", msg, i, j)
		}
	}
}

// pointDeviationRational is pointDeviation's squared-distance bound evaluated
// one big.Rat operation at a time.
func pointDeviationRational(p rotationalSweepPath, pose r3.Transform, f *big.Rat) float64 {
	ideal := idealAtRational(p, f)
	maxSquared := new(big.Rat)
	for _, source := range p.sourcePoints {
		actual := exactContactTransform(pose, source)
		point := pointVec(ratVec{source[0].Rat(), source[1].Rat(), source[2].Rat()})
		idealPoint := ivVecAdd(ivVecAdd(ideal.rot.apply(ivVecSub(point, ideal.pivot)), ideal.pivot), ideal.shift)
		observed := pointVec(ratVec{actual[0].Rat(), actual[1].Rat(), actual[2].Rat()})
		difference := ivVecSub(observed, idealPoint)
		squared := magnitudeSquaredUpper(difference[:]...)
		if squared.Cmp(maxSquared) > 0 {
			maxSquared = squared
		}
	}
	return proofbound.RatSqrtUp(maxSquared)
}

// rotationFormPaths are prepared sweep paths of every kind cornerSpan and
// pointDeviation serve: a source box turning about a generic axis while it
// translates, from a turned and shifted pose; an exact planar body on the
// same drift; a source box spinning about Z far from the origin, whose
// rounded poses deviate from the ideal path; a translation; and a screw
// about Z.
func rotationFormPaths(t *testing.T) map[string]rotationalSweepPath {
	t.Helper()
	doc := New()
	box := internalBoxBody(t, doc, -6, -4, 6, 4, 10)
	block := bandChamferedBlock(t, doc)
	turn, err := r3.RotationAround(r3.Vec{X: 1, Y: 2}, r3.Vec{X: 1, Y: 1, Z: 0}, units.Degrees(30))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: -45, Y: 12.5, Z: 40})
	require.NoError(t, err)
	from, err := turn.Then(shift)
	require.NoError(t, err)
	drift := RigidDriftSegment{From: from, Center: r3.Vec{X: -44, Y: 13, Z: 41},
		LinearVelocity: QuantityVec{X: units.MillimetersPerSecond(5), Y: units.MillimetersPerSecond(-3),
			Z: units.MillimetersPerSecond(-9810.0 / 64)},
		AngularVelocity: QuantityVec{X: units.RadiansPerSecond(2), Y: units.RadiansPerSecond(-1),
			Z: units.RadiansPerSecond(.75)},
		Duration: units.Seconds(1.0 / 256)}
	slide, err := r3.Translation(r3.Vec{X: 3, Y: -2, Z: 1.5})
	require.NoError(t, err)
	spinZ, err := r3.RotationAround(r3.Vec{X: 2}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	screwTo, err := shift.Then(spinZ)
	require.NoError(t, err)
	shiftTo, err := shift.Then(slide)
	require.NoError(t, err)

	paths := map[string]rotationalSweepPath{}
	sourceBox := func(name string, body *Body, segment PairPath) {
		path, err := validatePairPath(segment)
		require.NoError(t, err, name)
		prepared, ok := prepareRotationalSweepPath(body, path)
		require.True(t, ok, name)
		paths[name] = prepared
	}
	sourceBox("box drift", box, drift)
	sourceBox("box far spin", box, bandSpin(t, r3.Vec{X: bandTestOffset}))
	sourceBox("box translation", box, PoseSegment{From: shift, To: shiftTo, Duration: units.Seconds(1.0 / 256)})
	sourceBox("box screw", box, PoseSegment{From: shift, To: screwTo, Duration: units.Seconds(1.0 / 256)})
	planar := bandRun(t, doc, block, box, drift, drift)
	paths["planar drift"] = planar.a
	require.NotNil(t, paths["box screw"].path.screw, "premise: the Z turn reads as a screw")
	require.NotNil(t, paths["box drift"].path.drift)
	require.Nil(t, paths["box translation"].path.drift)
	return paths
}

// rotationFormFractions are fraction spans like the ones the sweep reads: the
// whole step, dyadic refinement intervals, a span ending at zero width, and
// non-dyadic fractions as the band and rolling tracks pass.
func rotationFormFractions() [][2]*big.Rat {
	return [][2]*big.Rat{
		{big.NewRat(0, 1), big.NewRat(1, 1)},
		{big.NewRat(0, 1), big.NewRat(1, 2)},
		{big.NewRat(1, 2), big.NewRat(3, 4)},
		{big.NewRat(5, 16), big.NewRat(11, 32)},
		{big.NewRat(1023, 1024), big.NewRat(1, 1)},
		{big.NewRat(3, 8), big.NewRat(3, 8)},
		{big.NewRat(0, 1), big.NewRat(1, 3)},
		{big.NewRat(2, 7), big.NewRat(5, 9)},
	}
}

func TestCornerSpanMatchesRationalForm(t *testing.T) {
	t.Parallel()
	for name, path := range rotationFormPaths(t) {
		for _, span := range rotationFormFractions() {
			got := path.cornerSpan(span[0], span[1])
			want := cornerSpanRational(path, span[0], span[1])
			require.Equal(t, len(want), got.len(), name)
			for axis := range 3 {
				for index := range want {
					endpoints := got.span(index, axis)
					require.Zero(t, endpoints.Lo.Cmp(want[index][axis].Lo), "%s %v point %d axis %d", name, span, index, axis)
					require.Zero(t, endpoints.Hi.Cmp(want[index][axis].Hi), "%s %v point %d axis %d", name, span, index, axis)
				}
				low, high := want[0][axis].Lo, want[0][axis].Hi
				for _, point := range want[1:] {
					low, high = ratMin(low, point[axis].Lo), ratMax(high, point[axis].Hi)
				}
				gotLow, gotHigh := got.hull(axis)
				require.Zero(t, gotLow.Cmp(low), "%s %v axis %d", name, span, axis)
				require.Zero(t, gotHigh.Cmp(high), "%s %v axis %d", name, span, axis)
			}
		}
	}
}

func TestPointDeviationMatchesRationalForm(t *testing.T) {
	t.Parallel()
	for name, path := range rotationFormPaths(t) {
		for _, span := range rotationFormFractions() {
			f := span[1]
			pose, err := path.poseAt(f)
			require.NoError(t, err, name)
			// The pose read at f, and the start pose read at f, which deviates
			// from the ideal path by the whole step's motion.
			for _, at := range []r3.Transform{pose, path.path.from} {
				points, bound, ok, err := path.pointDeviation(at, f, noSweepPoll)
				require.NoError(t, err, name)
				require.True(t, ok, name)
				require.Len(t, points, len(path.sourcePoints), name)
				for i, source := range path.sourcePoints {
					require.Equal(t, exactContactTransform(at, source), points[i], name)
				}
				require.Equal(t, pointDeviationRational(path, at, f), bound, "%s at %v", name, f)
				// The transfer charge over a held displacement δ = 2⁻¹².
				displaced := path
				displaced.delta = proofarith.MustDyOf(1.0 / 4096)
				ideal, ok := displaced.idealAt(f)
				require.True(t, ok, name)
				charge, ok := displaced.transferCharge(at, ideal)
				wantCharge, wantOK := transferChargeRational(displaced, at, f)
				require.Equal(t, wantOK, ok, name)
				require.Zero(t, wantCharge.Cmp(charge), "%s at %v", name, f)
			}
			ideal, ok := path.idealAt(f)
			require.True(t, ok, name)
			want := idealAtRational(path, f)
			requireScaledMatrix(t, want.rot, ideal.rot, name)
			for axis := range 3 {
				require.Zero(t, ideal.shift[axis].Lo.Cmp(want.shift[axis].Lo), "%s axis %d", name, axis)
				require.Zero(t, ideal.shift[axis].Hi.Cmp(want.shift[axis].Hi), "%s axis %d", name, axis)
			}
		}
	}
}

func TestScaledRotationMatchesRotation(t *testing.T) {
	t.Parallel()
	// Axes with every sign pattern and zero components, at turns whose sine
	// interval lies above zero, below it, and across it.
	axes := []ratVec{
		{big.NewRat(2, 1), big.NewRat(-1, 1), big.NewRat(3, 4)},
		{big.NewRat(0, 1), big.NewRat(0, 1), big.NewRat(1, 1)},
		{big.NewRat(-5, 8), big.NewRat(0, 1), big.NewRat(-3, 16)},
		{big.NewRat(1, 3), big.NewRat(2, 5), big.NewRat(-7, 9)},
	}
	angles := []*big.Rat{big.NewRat(1, 1000), big.NewRat(-3, 7), big.NewRat(0, 1), big.NewRat(22, 7)}
	for _, axis := range axes {
		unit, ok := unitScaleInterval(axis)
		require.True(t, ok)
		frame := motionFrame{kind: motionRevolute, axis: axis, unit: unit, center: ratVec{new(big.Rat), new(big.Rat), new(big.Rat)}}
		for _, angle := range angles {
			sin, cos := radianSinCos(angle)
			width := big.NewRat(1, 1<<20)
			sin = proofbound.IntervalOwned(new(big.Rat).Sub(sin.Lo, width), new(big.Rat).Add(sin.Hi, width))
			cos = proofbound.IntervalOwned(new(big.Rat).Sub(cos.Lo, width), new(big.Rat).Add(cos.Hi, width))
			requireScaledMatrix(t, frame.rotation(sin, cos), frame.scaledRotation(sin, cos), angle.String())
		}
	}
}

// pointDeviationSquaredCommonDenom is pointDeviationSquared before it read
// the common denominator off the coordinates' dyadic exponents: q is the LCM
// of all 6V rational denominators, and each numerator is ScaledNum over q.
func pointDeviationSquaredCommonDenom(p rotationalSweepPath, pose r3.Transform,
	ideal sweepIdealPose) ([]proofarith.DyV3, *big.Rat) {
	actual := make([]proofarith.DyV3, len(p.sourcePoints))
	for i, source := range p.sourcePoints {
		actual[i] = exactContactTransform(pose, source)
	}
	coordinates := make([]*big.Rat, 0, 6*len(p.sourcePoints))
	for i, source := range p.sourcePoints {
		for axis := range 3 {
			coordinates = append(coordinates, source[axis].Rat(), actual[i][axis].Rat())
		}
	}
	q := proofarith.CommonDenom(coordinates...)
	rot := ideal.rot
	rotDen := new(big.Int).Mul(rot.den, q)
	var den, rotMultiplier, observedMultiplier, shiftLo, shiftHi, toWhole [3]*big.Int
	whole := big.NewInt(1)
	for axis := range 3 {
		shift := ideal.shift[axis]
		den[axis] = proofarith.LcmInt(proofarith.LcmInt(rotDen, shift.Lo.Denom()), shift.Hi.Denom())
		rotMultiplier[axis] = new(big.Int).Quo(den[axis], rotDen)
		observedMultiplier[axis] = new(big.Int).Quo(den[axis], q)
		shiftLo[axis], shiftHi[axis] = proofarith.ScaledNum(shift.Lo, den[axis]), proofarith.ScaledNum(shift.Hi, den[axis])
		whole = proofarith.LcmInt(whole, den[axis])
	}
	for axis := range 3 {
		toWhole[axis] = new(big.Int).Quo(whole, den[axis])
	}
	maxSquared := new(big.Int)
	for i := range p.sourcePoints {
		var point [3]*big.Int
		for axis := range 3 {
			point[axis] = proofarith.ScaledNum(coordinates[6*i+2*axis], q)
		}
		lo, hi := rot.applyScaled(point)
		squared := new(big.Int)
		for axis := range 3 {
			observed := proofarith.ScaledNum(coordinates[6*i+2*axis+1], q)
			observed.Mul(observed, observedMultiplier[axis])
			low := lo[axis].Mul(lo[axis], rotMultiplier[axis])
			low.Add(low, shiftLo[axis])
			high := hi[axis].Mul(hi[axis], rotMultiplier[axis])
			high.Add(high, shiftHi[axis])
			below := high.Sub(observed, high)
			above := low.Sub(observed, low)
			maximum := below.Abs(below)
			if above.Abs(above).Cmp(maximum) > 0 {
				maximum = above
			}
			maximum.Mul(maximum, toWhole[axis])
			squared.Add(squared, maximum.Mul(maximum, maximum))
		}
		if squared.Cmp(maxSquared) > 0 {
			maxSquared = squared
		}
	}
	return actual, new(big.Rat).SetFrac(maxSquared, new(big.Int).Mul(whole, whole))
}

// TestPointDeviationMatchesCommonDenomForm holds pointDeviationSquared, which
// reads the points' common denominator 2^shift off their dyadic exponents, to
// the CommonDenom form it replaces: the same staged points and the identical
// rational maxSquared/whole², so proofbound.RatSqrtUp rounds the same. Every path kind of
// rotationFormPaths is read at the fraction's own pose, its start pose, and
// that pose moved by random turns and shifts from 2⁻⁴⁰ to 2¹⁰, and a half-unit
// shift that stages half-integer source points to integers, so either the
// source or the staged points may set the shift.
//
// Legs shown to fail: the shift taken over the staged points alone, the
// half-integer source points need a negative shift and the scaling panics;
// dyScaledNum shifting by the shift alone, dropping the exponent, the first
// box pose differs.
func TestPointDeviationMatchesCommonDenomForm(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(103, 107))
	compared, sourceSet := 0, 0
	// Half-integer source points under a half-unit shift stage to integers,
	// so the source points alone set the shift.
	halfShift, err := r3.Translation(r3.Vec{X: .5, Y: -.5, Z: .5})
	require.NoError(t, err)
	paths := rotationFormPaths(t)
	for name, path := range rotationFormPaths(t) {
		halves := path
		halves.sourcePoints = make([]proofarith.DyV3, len(path.sourcePoints))
		for i := range halves.sourcePoints {
			halves.sourcePoints[i] = proofarith.DyVec(r3.Vec{X: float64(i) + .5, Y: -float64(i) - 1.5, Z: 2.5})
		}
		paths[name+" halves"] = halves
	}
	for name, path := range paths {
		for _, span := range rotationFormFractions() {
			f := span[1]
			pose, err := path.poseAt(f)
			require.NoError(t, err, name)
			ideal, ok := path.idealAt(f)
			require.True(t, ok, name)
			poses := []r3.Transform{pose, path.path.from, halfShift}
			for range 6 {
				moved, err := pose.Then(randomContactPose(t, rng))
				require.NoError(t, err, name)
				poses = append(poses, moved)
			}
			for k, at := range poses {
				wantPoints, wantSquared := pointDeviationSquaredCommonDenom(path, at, ideal)
				gotPoints, gotSquared, err := path.pointDeviationSquared(at, ideal, noSweepPoll)
				require.NoError(t, err, name)
				require.Len(t, gotPoints, len(wantPoints), name)
				sourceShift, stagedShift := 0, 0
				for i := range wantPoints {
					requireDyV3Equal(t, wantPoints[i], gotPoints[i], "%s %v pose %d point %d", name, f, k, i)
					for axis := range 3 {
						sourceShift = max(sourceShift, dyDenominatorExp(path.sourcePoints[i][axis]))
						stagedShift = max(stagedShift, dyDenominatorExp(gotPoints[i][axis]))
					}
				}
				require.Zero(t, wantSquared.Cmp(gotSquared), "%s %v pose %d", name, f, k)
				_, bound, ok, err := path.pointDeviationFrom(at, ideal, noSweepPoll)
				require.NoError(t, err, name)
				require.True(t, ok, name)
				require.Equal(t, math.Float64bits(proofbound.RatSqrtUp(wantSquared)), math.Float64bits(bound), "%s %v pose %d", name, f, k)
				compared++
				if sourceShift > stagedShift {
					sourceSet++
				}
			}
		}
	}
	require.Positive(t, compared)
	require.Positive(t, sourceSet, "premise: some source points set the common denominator")
}
