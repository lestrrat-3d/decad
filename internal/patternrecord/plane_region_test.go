package patternrecord

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// arcFit is a smooth open curve, a few lengths wide, sampled by n fit points.
func arcFit() []Point2 {
	const n = 15
	fit := make([]Point2, n)
	for i := range fit {
		t := float64(i) / float64(n-1)
		fit[i] = Point2{U: 8 + 2*t + 0.3*math.Sin(3*t), V: 1 + 1.5*t*t}
	}
	return fit
}

func xyFrame(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

func rotationAboutZ(t *testing.T, turn float64) r3.Transform {
	t.Helper()
	x, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), units.Radians(2*math.Pi*turn))
	require.NoError(t, err)
	return x
}

func TestMovePlaneRegionRefusesWhatLeavesThePlane(t *testing.T) {
	f := xyFrame(t)
	region := Region{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 8, V: 0}, End: Point2{U: 9, V: 1}, TStart: 0, TEnd: 1},
	}}}
	move := func(x r3.Transform) error {
		pm, ok := NewPlaneMap(f, x, f, r3.Identity())
		require.True(t, ok, "the target frame has a plane")
		_, _, err := MovePlaneRegion(proofbound.NewWorkBudget(t.Context()), region, pm)
		return err
	}
	t.Run("a tilted placement leaves the plane", func(t *testing.T) {
		x, err := r3.RotationAround(r3.Vec{}, r3.NewVec(1, 0, 0), units.Radians(0.1))
		require.NoError(t, err)
		require.ErrorIs(t, move(x), decaderr.ErrUnsupported)
	})
	t.Run("a lifted placement leaves the plane", func(t *testing.T) {
		x, err := r3.Translation(r3.NewVec(0, 0, 1))
		require.NoError(t, err)
		require.ErrorIs(t, move(x), decaderr.ErrUnsupported)
	})
	t.Run("the identity is exact", func(t *testing.T) {
		pm, ok := NewPlaneMap(f, r3.Identity(), f, r3.Identity())
		require.True(t, ok)
		require.Equal(t, 1.0, pm.ScaleLo)
		require.Equal(t, 1.0, pm.ScaleHi)
		require.Zero(t, pm.ScaleDeviation())
		require.Zero(t, pm.Defect)
		require.Zero(t, pm.NormalCharge(10, 10))
		require.NoError(t, move(r3.Identity()))
	})
}

// obliqueFrame is a frame no axis aligns with, orthonormal only to rounding.
func obliqueFrame(t *testing.T) r3.Frame {
	t.Helper()
	rot, err := r3.RotationAround(r3.Vec{}, r3.NewVec(1, 1, 1), units.Radians(math.Pi/6))
	require.NoError(t, err)
	f, err := r3.NewFrame(r3.NewVec(3, -2, 5), rot.ApplyDir(r3.NewVec(1, 0, 0)), rot.ApplyDir(r3.NewVec(0, 1, 0)))
	require.NoError(t, err)
	return f
}

// TestPlaneMapOnAnObliqueFrameChargesItsDefects turns a section by a 17th of a turn
// about the normal of an oblique frame: the map exists, its defects are a few float
// steps, and the charge of the moved record stays tiny but positive.
func TestPlaneMapOnAnObliqueFrameChargesItsDefects(t *testing.T) {
	f := obliqueFrame(t)
	x, err := r3.RotationAround(f.Origin(), f.N(), units.Radians(2*math.Pi/17))
	require.NoError(t, err)
	pm, ok := NewPlaneMap(f, x, f, r3.Identity())
	require.True(t, ok)
	require.Less(t, pm.Defect, 1e-14)
	require.Less(t, pm.ScaleDeviation(), 1e-14)
	require.Less(t, pm.NormalCharge(10, 10), 1e-13)

	fit := arcFit()
	region := Region{Outer: LoopRecord{Segments: []CurveSegment{
		FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
		ArcSeg{Center: Point2{}, Start: fit[14], End: Point2{U: 8, V: 1}, TStart: 0, TEnd: 1},
		CircleSeg{Center: Point2{U: 1, V: 1}, Radius: units.Millimeters(0.5), CCW: true, TStart: 0, TEnd: 1},
		LineSeg{Start: fit[14], End: fit[0], TStart: 0, TEnd: 1},
	}}}
	_, delta, err := MovePlaneRegion(proofbound.NewWorkBudget(t.Context()), region, pm)
	require.NoError(t, err)
	require.Positive(t, delta)
	require.Less(t, delta, 1e-12)
}

// TestPlaneMapOnAnObliqueFrameRefusesATilt tilts the same frame's placement out of
// its plane by a hundredth of a radian.
func TestPlaneMapOnAnObliqueFrameRefusesATilt(t *testing.T) {
	f := obliqueFrame(t)
	x, err := r3.RotationAround(f.Origin(), f.U(), units.Radians(0.01))
	require.NoError(t, err)
	pm, ok := NewPlaneMap(f, x, f, r3.Identity())
	require.True(t, ok)
	region := Region{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 1}, End: Point2{U: 0, V: 9}, TStart: 0, TEnd: 1},
	}}}
	_, _, err = MovePlaneRegion(proofbound.NewWorkBudget(t.Context()), region, pm)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestMovePlaneRegionChargesFitSplineRounding(t *testing.T) {
	f := xyFrame(t)
	pm, ok := NewPlaneMap(f, rotationAboutZ(t, 1.0/17), f, r3.Identity())
	require.True(t, ok)
	fit := arcFit()
	region := Region{Outer: LoopRecord{Segments: []CurveSegment{
		FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
		LineSeg{Start: fit[14], End: fit[0], TStart: 0, TEnd: 1},
	}}}
	moved, delta, err := MovePlaneRegion(proofbound.NewWorkBudget(t.Context()), region, pm)
	require.NoError(t, err)
	require.Positive(t, delta, "rotating by a non-dyadic turn must round")
	require.Less(t, delta, 1e-13)
	mf := moved.Outer.Segments[0].(FitSplineSeg).Fit
	require.Len(t, mf, len(fit))
	for i, p := range mf {
		ex, ey := pm.Exact(fit[i])
		exF, _ := ex.Float64()
		eyF, _ := ey.Float64()
		require.InDelta(t, exF, p.U, 1e-15)
		require.InDelta(t, eyF, p.V, 1e-15)
	}
	line := moved.Outer.Segments[1].(LineSeg)
	require.Equal(t, mf[14], line.Start, "a shared point rounds to one held point")
	require.Equal(t, mf[0], line.End)
}

func TestFitMoveChargeIsNotVacuous(t *testing.T) {
	f := xyFrame(t)
	pm, ok := NewPlaneMap(f, rotationAboutZ(t, 1.0/17), f, r3.Identity())
	require.True(t, ok)
	fit := arcFit()
	moved := make([]Point2, len(fit))
	for i, p := range fit {
		u, v := pm.Exact(p)
		uf, _ := u.Float64()
		vf, _ := v.Float64()
		moved[i] = Point2{U: uf, V: vf}
	}
	exact, err := fitMoveCharge(fit, moved, pm)
	require.NoError(t, err)
	require.Less(t, exact, 1e-13)

	// Displace one interior knot by a known amount: the refit spline must
	// then sit at least that far from the image, and the charge must say so.
	const shift = 1e-3
	moved[7].V += shift
	charged, err := fitMoveCharge(fit, moved, pm)
	require.NoError(t, err)
	require.GreaterOrEqual(t, charged, shift)
}

func TestMovePlaneRegionChargesScaleOfCircularSegments(t *testing.T) {
	f := xyFrame(t)
	pm, ok := NewPlaneMap(f, rotationAboutZ(t, 1.0/17), f, r3.Identity())
	require.True(t, ok)
	region := Region{Outer: LoopRecord{Segments: []CurveSegment{
		CircleSeg{Center: Point2{}, Radius: units.Millimeters(8), CCW: true, TStart: 0, TEnd: 1},
	}}}
	moved, delta, err := MovePlaneRegion(proofbound.NewWorkBudget(t.Context()), region, pm)
	require.NoError(t, err)
	require.Equal(t, Point2{}, moved.Outer.Segments[0].(CircleSeg).Center)
	// The centre is a fixed point of the rotation, so the whole charge is the
	// radius' scale deviation: 8 mm times |scale − 1|, a few float steps.
	require.InDelta(t, 8*pm.ScaleDeviation(), delta, 1e-30+8*pm.ScaleDeviation()*0.01)
	require.Less(t, delta, 1e-14)
}

// TestFitMoveChargeEnclosesTheSampledDistance samples the matched cubics: the charge
// is at least the largest sampled distance between the refit spline and the image of
// the source, and within a small factor of it, so it is a usable enclosure and not a
// loose one.
func TestFitMoveChargeEnclosesTheSampledDistance(t *testing.T) {
	f := xyFrame(t)
	for _, turn := range []float64{1.0 / 17, 1.0 / 42, 0.3} {
		pm, ok := NewPlaneMap(f, rotationAboutZ(t, turn), f, r3.Identity())
		require.True(t, ok)
		fit := arcFit()
		moved := make([]Point2, len(fit))
		for i, p := range fit {
			m, _, err := pm.Motion()(p)
			require.NoError(t, err)
			moved[i] = m
		}
		charge, err := fitMoveCharge(fit, moved, pm)
		require.NoError(t, err)

		hold := func(in []Point2) [][2]float64 {
			out := make([][2]float64, len(in))
			for i, p := range in {
				out[i] = [2]float64{p.U, p.V}
			}
			return out
		}
		src, err := geom.NewFitInterpolant(hold(fit))
		require.NoError(t, err)
		refit, err := geom.NewFitInterpolant(hold(moved))
		require.NoError(t, err)
		ss, err := src.ExactSpans()
		require.NoError(t, err)
		qs, err := refit.ExactSpans()
		require.NoError(t, err)
		// Exact rational evaluation: float evaluation of coefficients near 10 carries
		// a rounding error as large as the distances measured.
		eval := func(c [4]*big.Rat, u *big.Rat) *big.Rat {
			acc := new(big.Rat).Set(c[3])
			for k := 2; k >= 0; k-- {
				acc.Mul(acc, u).Add(acc, c[k])
			}
			return acc
		}
		mul := proofbound.RatMul
		worst := 0.0
		for j := range ss {
			for k := range 201 {
				u := big.NewRat(int64(k), 200)
				sx, sy := eval(ss[j].X, u), eval(ss[j].Y, u)
				ix := proofbound.RatAdd(mul(pm.A[0][0], sx), mul(pm.A[0][1], sy), pm.T[0])
				iy := proofbound.RatAdd(mul(pm.A[1][0], sx), mul(pm.A[1][1], sy), pm.T[1])
				dx := new(big.Rat).Sub(eval(qs[j].X, u), ix)
				dy := new(big.Rat).Sub(eval(qs[j].Y, u), iy)
				d := proofbound.RatSqrtUp(proofbound.RatAdd(mul(dx, dx), mul(dy, dy)))
				worst = math.Max(worst, d)
			}
		}
		require.LessOrEqual(t, worst, charge, "turn %v", turn)
		require.LessOrEqual(t, charge, 100*worst+1e-14, "turn %v", turn)
	}
}
