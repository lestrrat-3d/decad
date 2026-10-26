package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The turned-shaft fixtures of agent-docs/general-boolean-design.md §3
// "Revolved shaft as a prism": a full Revolve of a rectangle with one side on
// the axis is read as the circular prism it denotes, so a perpendicular box
// spanning its length cuts it through class B's crossing reach. Each exact
// fixture's tool is drawn on an offset datum plane, so it shares the shaft's
// placement.
//
// Shown-to-fail: without the Revolve arm in booleandispatch.classB each exact
// fixture takes the mesh path and its volume bound is about 2 mm³. Without
// the tool-wall gate (classbgeom.CutPrismView) the keyseat ending inside the
// shaft refuses with "a brep edge must bound exactly two distinct faces"
// instead of keeping its mesh result; without the gate's sign, the keyseat
// drawn against the shaft axis does.

// shaftTool extrudes the rectangle (u0, v0)–(u1, v1) on plane 20 mm along its
// normal.
func shaftTool(t *testing.T, sc *retentionScene, plane *sketch.Plane, u0, v0, u1, v1 float64) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return box
}

// flatPlane is the world XY plane moved to z = flatZ.
func flatPlane(t *testing.T, sc *retentionScene) *sketch.Plane {
	t.Helper()
	p, err := sc.w.CreateOffsetPlane(sc.w.XY(), flatZ)
	require.NoError(t, err)
	return p
}

// shaftSqrt is √x at the closed forms' working precision.
func shaftSqrt(x float64) *big.Float {
	return new(big.Float).SetPrec(junctionPrec).Sqrt(jf(x))
}

// flatSegment is the area of the r = 10 disc above the chord at height h = 7:
// r²·acos(h/r) − h·√(r² − h²), with acos(0.7) = π/2 − atan(7/√51).
func flatSegment() *big.Float {
	s51 := shaftSqrt(51)
	half := new(big.Float).SetPrec(junctionPrec).Quo(junctionPi(), jf(2))
	acos := half.Sub(half, junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(jf(7), s51)))
	area := acos.Mul(acos, jf(100))
	return area.Sub(area, new(big.Float).SetPrec(junctionPrec).Mul(jf(7), s51))
}

// keySeat is the area of the r = 10 disc with |y| ≤ 3 above z = 7:
// ∫(√(100 − y²) − 7) dy over [−3, 3] = 3·√91 + 100·asin(0.3) − 42, with
// asin(0.3) = atan(3/√91).
func keySeat() *big.Float {
	s91 := shaftSqrt(91)
	asin := junctionAtan(new(big.Float).SetPrec(junctionPrec).Quo(jf(3), s91))
	area := asin.Mul(asin, jf(100))
	area.Add(area, new(big.Float).SetPrec(junctionPrec).Mul(jf(3), s91))
	return area.Sub(area, jf(42))
}

// shaftLess is the shaft's volume, 40·π·100, less 40 times removed.
func shaftLess(removed *big.Float) *big.Float {
	disc := new(big.Float).SetPrec(junctionPrec).Mul(junctionPi(), jf(100))
	disc.Sub(disc, removed)
	return disc.Mul(disc, jf(shaftL))
}

// requireClassBShaft checks a cut shaft built through class B: no Faceted
// face, the given plane count beside one cylinder piece, a volume within its
// bound of truth, and a mesh whose boundary and occupied volume verify.
func requireClassBShaft(t *testing.T, got *decad.Body, planes int, truth *big.Float) decad.Measurement {
	t.Helper()
	cylinders, flat := 0, 0
	for _, f := range got.Faces() {
		switch f.Surface().(type) {
		case decad.Faceted:
			require.Fail(t, "a class-B result has no Faceted face")
		case decad.Cylinder:
			cylinders++
		case decad.Plane:
			flat++
		}
	}
	require.Equal(t, 1, cylinders)
	require.Equal(t, planes, flat)
	volume, err := got.Volume()
	require.NoError(t, err)
	value, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	bound, err := volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, bound, 1e-6, `class B, not the mesh path, measures the cut`)
	requireJunctionEncloses(t, `cut shaft volume`, value, bound, truth)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	return volume
}

func TestRevolvedShaftClassBCut(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name    string
		v0, v1  float64
		planes  int
		removed func() *big.Float
	}{
		{"D-flat", -20, 20, 3, flatSegment},
		{"keyway", -3, 3, 5, keySeat},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sc := newRetentionScene()
			got, err := decad.Cut(t.Context(), revolvedShaft(t, sc),
				shaftTool(t, sc, flatPlane(t, sc), -5, tc.v0, shaftL+5, tc.v1))
			require.NoError(t, err)
			volume := requireClassBShaft(t, got, tc.planes, shaftLess(tc.removed()))

			// The Extrude twin is the same prism record, so it measures the
			// same value and bound bit for bit.
			tsc := newRetentionScene()
			twin, err := decad.Cut(t.Context(), extrudedShaft(t, tsc),
				shaftTool(t, tsc, flatPlane(t, tsc), -5, tc.v0, shaftL+5, tc.v1))
			require.NoError(t, err)
			twinVolume, err := twin.Volume()
			require.NoError(t, err)
			require.Equal(t, twinVolume.Value, volume.Value)
			require.Equal(t, twinVolume.Bound, volume.Bound)
		})
	}

	t.Run("axis along the sketch V", func(t *testing.T) {
		t.Parallel()
		// The rectangle u in [0, 10], v in [0, 40] about the V axis is the
		// same shaft along world y; the flat is the plane x = 7, cut by a box
		// drawn on the YZ plane moved to x = 7.
		sc := newRetentionScene()
		s, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		r := s.CreateRectangle(0, 0, shaftR, shaftL)
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		vAxis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
		shaft, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], vAxis, decad.FullRevolution{})
		require.NoError(t, err)
		plane, err := sc.w.CreateOffsetPlane(sc.w.YZ(), flatZ)
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), shaft, shaftTool(t, sc, plane, -5, -20, shaftL+5, 20))
		require.NoError(t, err)
		requireClassBShaft(t, got, 3, shaftLess(flatSegment()))
	})

	t.Run("thread on the D-flat wall", func(t *testing.T) {
		t.Parallel()
		// The cut wall takes a thread, to the same volume as its Extrude
		// twin's. A coarse pitch keeps the thread to five turns.
		threaded := func(shaft func(*testing.T, *retentionScene) *decad.Body) (float64, decad.Measurement) {
			sc := newRetentionScene()
			cut, err := decad.Cut(t.Context(), shaft(t, sc),
				shaftTool(t, sc, flatPlane(t, sc), -5, -20, shaftL+5, 20))
			require.NoError(t, err)
			before, err := cut.Volume()
			require.NoError(t, err)
			out, err := decad.Thread(t.Context(), cut, decad.ThreadSpec{
				Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(8), Form: acmeForm()})
			require.NoError(t, err)
			after, err := out.Volume()
			require.NoError(t, err)
			b, err := before.Value.In(units.CubicMillimeter)
			require.NoError(t, err)
			return b, after
		}
		before, got := threaded(revolvedShaft)
		_, twin := threaded(extrudedShaft)
		require.Equal(t, twin.Value, got.Value)
		require.Equal(t, twin.Bound, got.Bound)
		after, err := got.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.Less(t, after, before, `the thread removes material`)
	})

	t.Run("blends of the D-flat refuse", func(t *testing.T) {
		t.Parallel()
		// The class-B flat carries a crossing displacement, so brep-modify's
		// SB1 refuses a blend of its edges; a Transform-placed tool keeps the
		// mesh path and its retained blend (faceted_retained_blend_revolve_test.go).
		sc := newRetentionScene()
		cut, err := decad.Cut(t.Context(), revolvedShaft(t, sc),
			shaftTool(t, sc, flatPlane(t, sc), -5, -20, shaftL+5, 20))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), cut, flatEdges(t, cut), units.Millimeters(2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, `brep-modify SB1`)
		_, err = decad.Chamfer(t.Context(), cut, flatEdges(t, cut), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, `brep-modify SB1`)
	})
}

// requireShaftMeshPath checks that a cut shaft built, through the mesh path:
// its volume bound is far above class B's.
func requireShaftMeshPath(t *testing.T, got *decad.Body, err error) {
	t.Helper()
	require.NoError(t, err)
	volume, err := got.Volume()
	require.NoError(t, err)
	bound, err := volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Greater(t, bound, 1e-3, `the pair misses class B and takes the mesh path`)
}

func TestRevolvedShaftClassBMisses(t *testing.T) {
	t.Parallel()

	t.Run("keyseat ending inside the shaft", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		got, err := decad.Cut(t.Context(), revolvedShaft(t, sc),
			shaftTool(t, sc, flatPlane(t, sc), 15, -3, 25, 3))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("keyseat drawn against the shaft axis", func(t *testing.T) {
		t.Parallel()
		// The tool plane's U is -x and its normal -z, so the gate reads the
		// tool's x levels through a negative sign: u in [-25, -15] is
		// x in [15, 25], and the box runs from z = 27 down to z = 7.
		sc := newRetentionScene()
		frame, err := r3.NewFrame(r3.Vec{Z: flatZ + 20}, r3.NewVec(-1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		plane, err := sc.w.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), revolvedShaft(t, sc), shaftTool(t, sc, plane, -25, -3, -15, 3))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("tool wall in the end plane", func(t *testing.T) {
		t.Parallel()
		// The tool's x = 0 wall lies in the shaft's end cap plane. B8, not the
		// tool-wall gate, misses this pair: the gate never sees it.
		sc := newRetentionScene()
		got, err := decad.Cut(t.Context(), revolvedShaft(t, sc),
			shaftTool(t, sc, flatPlane(t, sc), 0, -20, shaftL+5, 20))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("tilted tool", func(t *testing.T) {
		t.Parallel()
		// The tool's plane turns about y by atan(0.75), so its floor meets
		// the wall in an ellipse: B4 misses.
		sc := newRetentionScene()
		frame, err := r3.NewFrame(r3.Vec{Z: flatZ}, r3.NewVec(0.8, 0, 0.6), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		plane, err := sc.w.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), revolvedShaft(t, sc), shaftTool(t, sc, plane, -10, -20, 60, 20))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("Transform-placed tool", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		got, err := decad.Cut(t.Context(), revolvedShaft(t, sc), flatTool(t, sc))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("stepped shaft", func(t *testing.T) {
		t.Parallel()
		// Radius 10 over x in [0, 20] and 8 over [20, 40]: the meridian is
		// no rectangle, so the shaft has no prism view.
		s, profile := polygonSketch(t, [][2]float64{{0, 0}, {40, 0}, {40, 8}, {20, 8}, {20, 10}, {0, 10}})
		sc := newRetentionScene()
		shaft, err := decad.Revolve(t.Context(), sc.doc, s, profile, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), shaft, shaftTool(t, sc, flatPlane(t, sc), -5, -20, shaftL+5, 20))
		requireShaftMeshPath(t, got, err)
	})

	t.Run("tube", func(t *testing.T) {
		t.Parallel()
		s, profile := polygonSketch(t, [][2]float64{{0, 4}, {40, 4}, {40, 10}, {0, 10}})
		sc := newRetentionScene()
		tube, err := decad.Revolve(t.Context(), sc.doc, s, profile, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), tube, shaftTool(t, sc, flatPlane(t, sc), -5, -20, shaftL+5, 20))
		requireShaftMeshPath(t, got, err)
	})
}
