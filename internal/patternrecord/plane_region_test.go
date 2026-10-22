package patternrecord

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// arcFit is a smooth open curve, a few lengths wide, sampled by n fit points.
func arcFit(n int) []Point2 {
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

func TestPlaneMapRefusesWhatIsNotAnExactSimilarity(t *testing.T) {
	f := xyFrame(t)
	t.Run("a tilted placement leaves the plane", func(t *testing.T) {
		x, err := r3.RotationAround(r3.Vec{}, r3.NewVec(1, 0, 0), units.Radians(0.1))
		require.NoError(t, err)
		_, ok := NewPlaneMap(f, x, f, r3.Identity())
		require.False(t, ok)
	})
	t.Run("a lifted placement leaves the plane", func(t *testing.T) {
		x, err := r3.Translation(r3.NewVec(0, 0, 1))
		require.NoError(t, err)
		_, ok := NewPlaneMap(f, x, f, r3.Identity())
		require.False(t, ok)
	})
	t.Run("the identity is exact", func(t *testing.T) {
		pm, ok := NewPlaneMap(f, r3.Identity(), f, r3.Identity())
		require.True(t, ok)
		require.Equal(t, 1.0, pm.ScaleLo)
		require.Equal(t, 1.0, pm.ScaleHi)
		require.Zero(t, pm.ScaleDeviation())
	})
}

func TestMovePlaneRegionChargesFitSplineRounding(t *testing.T) {
	f := xyFrame(t)
	pm, ok := NewPlaneMap(f, rotationAboutZ(t, 1.0/17), f, r3.Identity())
	require.True(t, ok)
	fit := arcFit(15)
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
	fit := arcFit(15)
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
