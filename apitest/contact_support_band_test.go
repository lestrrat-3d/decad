package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The ContactPair fixtures of docs/multibody-dynamics-design.md §13 PR 14a:
// §10.5's support set. An 8 mm source cube rests on a floor whose top face is
// z = 0, its edge x = z = 0 on the floor and its body turned about Y by
// sin θ = 2⁻²⁴, so its far bottom edge stands exactly 8·2⁻²⁴ = 2⁻²¹ mm up.
// cos θ is rounded to 1 − 2⁻⁴⁹, the float nearest √(1 − 2⁻⁴⁸); every bottom
// vertex height is x·sin θ, a float, so each published Separation below is
// exact.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the foot test of PlanarSupportSets (the foot strictly inside the face):
//     TestContactPairSupportSetPastRim publishes four points, and
//     TestContactPairSupportSetTray publishes the far bottom edge over the
//     floor;
//   - the band comparison (h² <= band²·n·n): every fixture below publishes
//     corners above the band, among them the cube's top corners;
//   - the exclusion of the contact set from the lifted set:
//     TestContactPairSupportSetTouch publishes the near edge twice.

// supportSine is sin θ of the tilted cube, and supportCosine its rounded cos θ.
const (
	supportSine   = 1.0 / (1 << 24)
	supportCosine = 1 - 1.0/(1<<49)
	// supportLift is the far bottom edge's height, 8·sin θ.
	supportLift = 1.0 / (1 << 21)
)

// tiltedCubePose turns the cube about Y by θ, keeping its edge x = z = 0 at
// the origin, then moves it by at.
func tiltedCubePose(t *testing.T, at r3.Vec) r3.Transform {
	t.Helper()
	return basisPose(t, r3.Vec{X: supportCosine, Z: supportSine}, r3.Vec{Y: 1},
		r3.Vec{X: -supportSine, Z: supportCosine}, at)
}

// supportBandRequest is contactRequest with a SupportBand of band millimetres.
func supportBandRequest(band float64) decad.ContactRequest {
	req := contactRequest()
	req.SupportBand = units.Millimeters(band)
	return req
}

// supportCubeScene is the floor and the 8 mm cube at the identity pose.
func supportCubeScene(t *testing.T, doc *decad.Document, floorX1 float64) (*decad.Body, *decad.Body) {
	t.Helper()
	floor := boxBodyAtZ(t, doc, -100, -100, floorX1, 100, -10, 10)
	cube := boxBody(t, doc, 0, -4, 8, 4, 8)
	return floor, cube
}

// requireSupportPoints checks a manifold of the cube on a plane with outward
// normal up (pointing from A to B when the floor is A): every point's cube
// witness has the exact height its Separation states, its floor witness is
// that point's foot, and the heights match want in order.
func requireSupportPoints(t *testing.T, manifold *decad.ContactManifold, cubeIsA bool, want []float64) {
	t.Helper()
	require.NotNil(t, manifold)
	require.Len(t, manifold.Points, len(want))
	var heights []float64
	for i, point := range manifold.Points {
		onCube, onFloor := point.OnB, point.OnA
		normal := r3.Vec{Z: 1}
		if cubeIsA {
			onCube, onFloor = point.OnA, point.OnB
			normal = r3.Vec{Z: -1}
		}
		require.Equal(t, normal, point.Normal.Value, "point %d", i)
		height := point.Separation.Value.Base()
		heights = append(heights, height)
		require.Zero(t, point.Separation.Bound.Base(), "point %d", i)
		require.InDelta(t, height, onCube.Value.Z, onCube.Bound.Base(), "point %d", i)
		require.InDelta(t, 0, onFloor.Value.Z, onFloor.Bound.Base(), "point %d", i)
		require.InDelta(t, onCube.Value.X, onFloor.Value.X, onCube.Bound.Base()+onFloor.Bound.Base(), "point %d", i)
		require.InDelta(t, onCube.Value.Y, onFloor.Value.Y, onCube.Bound.Base()+onFloor.Bound.Base(), "point %d", i)
		// A lifted point names the cube's vertex.
		feature := point.FeatureB
		if cubeIsA {
			feature = point.FeatureA
		}
		if height > 0 {
			require.NotNil(t, feature.Vertex, "point %d", i)
		}
	}
	require.ElementsMatch(t, want, heights)
	// The exact contact points, at want's first height when it is not
	// positive, come first, then the lifted ones.
	if want[0] <= 0 {
		for i, w := range want {
			if w != want[0] {
				break
			}
			require.Equal(t, w, heights[i], "point %d", i)
		}
	}
}

func TestContactPairSupportSetTouch(t *testing.T) {
	doc := decad.New()
	floor, cube := supportCubeScene(t, doc, 100)
	pose := tiltedCubePose(t, r3.Vec{})
	for _, cubeIsA := range []bool{false, true} {
		a, b, poseA, poseB := floor, cube, r3.Identity(), pose
		if cubeIsA {
			a, b, poseA, poseB = cube, floor, pose, r3.Identity()
		}
		// At a band of 2⁻²⁰ the touch publishes the near edge's two exact
		// points, then the far edge's two at their exact height.
		report, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, supportBandRequest(2*supportLift))
		require.NoError(t, err)
		require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
		require.Zero(t, report.Gap.Value.Base())
		requireSupportPoints(t, report.Manifold, cubeIsA, []float64{0, 0, supportLift, supportLift})
		for _, point := range report.Manifold.Points[2:] {
			onCube := point.OnB
			if cubeIsA {
				onCube = point.OnA
			}
			require.InDelta(t, 8*supportCosine, onCube.Value.X, onCube.Bound.Base())
		}

		// A band below the far edge's height, and a zero band, publish the
		// exact contact set alone.
		for _, band := range []float64{supportLift / 2, 0} {
			report, err = doc.ContactPair(t.Context(), a, b, poseA, poseB, supportBandRequest(band))
			require.NoError(t, err)
			require.Equal(t, decad.ContactTouching, report.Relation)
			requireSupportPoints(t, report.Manifold, cubeIsA, []float64{0, 0})
		}
		report, err = doc.ContactPair(t.Context(), a, b, poseA, poseB, contactRequest())
		require.NoError(t, err)
		requireSupportPoints(t, report.Manifold, cubeIsA, []float64{0, 0})
	}
}

func TestContactPairSupportSetGap(t *testing.T) {
	const lift = 1.0 / (1 << 30)
	doc := decad.New()
	floor, cube := supportCubeScene(t, doc, 100)
	pose := tiltedCubePose(t, r3.Vec{Z: lift})
	for _, cubeIsA := range []bool{false, true} {
		a, b, poseA, poseB := floor, cube, r3.Identity(), pose
		if cubeIsA {
			a, b, poseA, poseB = cube, floor, pose, r3.Identity()
		}
		// Lifted by 2⁻³⁰ mm the cube is apart, within the 2⁻²⁰ mm band: its
		// four bottom vertices are the lifted set, at 2⁻³⁰ and 2⁻³⁰ + 2⁻²¹.
		report, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, supportBandRequest(2*supportLift))
		require.NoError(t, err)
		require.Equal(t, decad.ContactBand, report.Relation, "reason=%v", report.Reason)
		require.Zero(t, report.Gap.Value.Base())
		require.GreaterOrEqual(t, report.Gap.Bound.Base(), lift)
		requireSupportPoints(t, report.Manifold, cubeIsA, []float64{lift, lift, lift + supportLift, lift + supportLift})

		// A band below the gap leaves the pair Separated.
		report, err = doc.ContactPair(t.Context(), a, b, poseA, poseB, supportBandRequest(lift/2))
		require.NoError(t, err)
		require.Equal(t, decad.ContactSeparated, report.Relation)
		require.Nil(t, report.Manifold)
	}
}

func TestContactPairSupportSetOverlap(t *testing.T) {
	const sink = 1.0 / (1 << 30)
	doc := decad.New()
	floor, cube := supportCubeScene(t, doc, 100)
	pose := tiltedCubePose(t, r3.Vec{Z: -sink})
	for _, cubeIsA := range []bool{false, true} {
		a, b, poseA, poseB := floor, cube, r3.Identity(), pose
		if cubeIsA {
			a, b, poseA, poseB = cube, floor, pose, r3.Identity()
		}
		// Sunk by 2⁻³⁰ mm the near edge pokes through the floor; the far edge
		// stands 2⁻²¹ − 2⁻³⁰ above it, inside the band.
		report, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, supportBandRequest(2*supportLift))
		require.NoError(t, err)
		require.Equal(t, decad.ContactOverlapping, report.Relation, "reason=%v", report.Reason)
		requireSupportPoints(t, report.Manifold, cubeIsA, []float64{-sink, -sink, supportLift - sink, supportLift - sink})
	}
}

func TestContactPairSupportSetPastRim(t *testing.T) {
	// The floor ends at x = 6, so the far edge's feet lie past its rim: only
	// the near edge is published.
	doc := decad.New()
	floor, cube := supportCubeScene(t, doc, 6)
	report, err := doc.ContactPair(t.Context(), floor, cube, r3.Identity(), tiltedCubePose(t, r3.Vec{}),
		supportBandRequest(2*supportLift))
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, report.Relation)
	requireSupportPoints(t, report.Manifold, false, []float64{0, 0})
}

func TestContactPairSupportSetTray(t *testing.T) {
	// An L-shaped tray: a floor whose top face is z = 0 and a wall whose inner
	// face is x = 8·cos θ, the tilted cube's far bottom edge, which therefore
	// touches the wall. The section lies in XY and is turned onto XZ, its
	// extrusion running along −Y. The cube's far top edge stands 8·sin θ =
	// 2⁻²¹ mm off the wall: the wall publishes it as its lifted set with the
	// wall's own normal, while the floor's lifted set is empty, since the far
	// bottom edge's foot on the floor lies on the floor face's rim.
	doc := decad.New()
	wall := 8 * supportCosine
	tray := prismBody(t, doc, [][2]float64{{-20, -10}, {wall + 10, -10}, {wall + 10, 20}, {wall, 20}, {wall, 0},
		{-20, 0}}, 40)
	cube := boxBody(t, doc, 0, -4, 8, 4, 8)
	trayPose := basisPose(t, r3.Vec{X: 1}, r3.Vec{Z: 1}, r3.Vec{Y: -1}, r3.Vec{Y: 20})
	report, err := doc.ContactPair(t.Context(), tray, cube, trayPose, tiltedCubePose(t, r3.Vec{}),
		supportBandRequest(2*supportLift))
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
	require.NotNil(t, report.Manifold)
	floorPoints, wallPoints := 0, 0
	var wallLifted []float64
	for _, point := range report.Manifold.Points {
		switch point.Normal.Value {
		case r3.Vec{Z: 1}:
			floorPoints++
			require.Zero(t, point.Separation.Value.Base())
			require.InDelta(t, 0, point.OnB.Value.X, point.OnB.Bound.Base())
		case r3.Vec{X: -1}:
			wallPoints++
			require.InDelta(t, wall, point.OnA.Value.X, point.OnA.Bound.Base())
			if point.Separation.Value.Base() != 0 {
				wallLifted = append(wallLifted, point.Separation.Value.Base())
			}
		default:
			require.Failf(t, "unexpected normal", "%v", point.Normal.Value)
		}
	}
	require.Equal(t, 2, floorPoints)
	require.Equal(t, 4, wallPoints)
	require.Equal(t, []float64{supportLift, supportLift}, wallLifted)
}

func TestContactPairSupportBandValidation(t *testing.T) {
	doc := decad.New()
	floor, cube := supportCubeScene(t, doc, 100)
	pose := tiltedCubePose(t, r3.Vec{})
	for _, tc := range []struct {
		band units.Value
		err  error
	}{
		{units.Millimeters(-1), decad.ErrDegenerate},
		{units.Millimeters(math.Inf(1)), decad.ErrNotFinite},
		{units.Radians(1), decad.ErrUnitKind},
	} {
		req := contactRequest()
		req.SupportBand = tc.band
		_, err := doc.ContactPair(t.Context(), floor, cube, r3.Identity(), pose, req)
		require.ErrorIs(t, err, tc.err)
		sweep := tumbleRequest()
		sweep.SupportBand = tc.band
		_, err = doc.SweepPair(t.Context(), floor, cube, sweepDrift(r3.Vec{}, 1), sweepDrift(r3.Vec{}, 1), sweep)
		require.ErrorIs(t, err, tc.err)
	}
}
