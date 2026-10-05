package decad_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures exercise the clearance kernel's ruling certificates and the
// manifold docs/contact-geometry-design.md §4.5 publishes from them. Every
// coordinate is dyadic, so each ruling end is an exact float and every
// published bound is computed, never pinned.
//
// Each gate of the certificate was deleted in turn and the named fixture
// went red:
//   - the zero carrier displacement of both bodies: "placed" reads Touching;
//   - the exact axis-to-plane distance: "near tangent" reads Touching;
//   - the whole ruling inside the plane trim: "overhang" reads Touching;
//   - the plane body's extent along the normal: "channel" reads Touching;
//   - the cylinder body's extent along the normal: "void wall" reads Touching;
//   - the tangent azimuth inside the cylinder trim: "arc end" reads Touching;
//   - the exact radius-sum axis offset: "near tangent pair" reads Touching;
//   - either cylinder's extent along the normal: "void wall pair" reads
//     Touching in one body order each;
//   - the positive-length axial overlap: "end to end" reads Touching.
//
// Charging a full revolve's end-angle term into its carrier displacement
// turns every touching fixture here red.
//
// No fixture can show the radial tilt rulingSideNormal charges for a rounded
// cylinder witness failing. On every admitted ruling the separating normal
// and the cylinder axis are signed coordinate axes, and the third coordinate
// of an end is the carrier's own float. A witness therefore rounds only along
// the normal or the axis, and neither turns the radial direction
// Face.NormalAt reads. The leg is redundant there and stays as the general
// bound.

// revolvedCylinder revolves the rectangle u∈[u0,u1], v∈[axisV, axisV+r]
// about the line v = axisV: a full source cylinder along world X.
func revolvedCylinder(t *testing.T, doc *decad.Document, u0, u1, axisV, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, axisV, u1, axisV+r)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: axisV}, End: decad.Point2{U: 1, V: axisV}}
	body, err := doc.Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func surfaceFace[T decad.Surface](t *testing.T, b *decad.Body, pick func(*decad.Face) bool) *decad.Face {
	t.Helper()
	var found *decad.Face
	for _, face := range b.Faces() {
		if _, ok := face.Surface().(T); ok && pick(face) {
			require.Nil(t, found, "more than one face matches")
			found = face
		}
	}
	require.NotNil(t, found)
	return found
}

// requireRulingManifold checks the two published ends, the face identities,
// and that each entry's normal is the tighter of the two faces' own
// NormalAt balls at that end, oriented from A toward B.
func requireRulingManifold(t *testing.T, report *decad.ContactReport, faceA, faceB *decad.Face,
	ends [2]r3.Vec, normal r3.Vec) {
	t.Helper()
	require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
	require.Equal(t, decad.ContactNoReason, report.Reason)
	require.NotNil(t, report.Gap)
	require.Equal(t, decad.Exact, report.Gap.Exactness)
	require.Zero(t, report.Gap.Value.Base())
	require.NotNil(t, report.Manifold)
	require.Len(t, report.Manifold.Points, 2)
	for i, point := range report.Manifold.Points {
		require.Equal(t, ends[i], point.OnA.Value)
		require.Equal(t, ends[i], point.OnB.Value)
		require.LessOrEqual(t, point.OnA.Bound.Base(), report.Request.PointResolution.Base())
		require.Same(t, faceA, point.FaceA)
		require.Same(t, faceB, point.FaceB)
		require.Same(t, faceA, point.FeatureA.Face)
		require.Same(t, faceB, point.FeatureB.Face)
		readingA, err := faceA.NormalAt(ends[i])
		require.NoError(t, err)
		readingB, err := faceB.NormalAt(ends[i])
		require.NoError(t, err)
		require.Equal(t, normal, readingA.Value)
		require.Equal(t, normal.Scale(-1), readingB.Value)
		ball := math.Min(readingA.Bound.Base(), readingB.Bound.Base())
		require.Equal(t, normal, point.Normal.Value)
		require.Equal(t, ball, point.Normal.Bound.Base())
		require.LessOrEqual(t, point.NormalAngle.Base(), report.Request.NormalResolution.Base())
		require.Equal(t, decad.Exact, point.Separation.Exactness)
		require.Zero(t, point.Separation.Value.Base())
	}
}

func TestContactPairCylinderRulingOnFloor(t *testing.T) {
	doc := decad.New()
	// Radius 8 about world X, x∈[0,10]; the floor's top face is z = -8.
	cylinder := revolvedCylinder(t, doc, 0, 10, 0, 8)
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -18, 10)
	wall := surfaceFace[decad.Cylinder](t, cylinder, func(*decad.Face) bool { return true })
	top := surfaceFace[decad.Plane](t, floor, func(face *decad.Face) bool {
		reading, err := face.NormalAt(r3.Vec{})
		require.NoError(t, err)
		return reading.Value == r3.Vec{Z: 1}
	})
	ends := [2]r3.Vec{{Z: -8}, {X: 10, Z: -8}}
	before := doc.Bodies()

	report, err := doc.ContactPair(t.Context(), floor, cylinder, r3.Identity(), r3.Identity(), contactRequest())
	require.NoError(t, err)
	requireRulingManifold(t, report, top, wall, ends, r3.Vec{Z: 1})

	reversed, err := doc.ContactPair(t.Context(), cylinder, floor, r3.Identity(), r3.Identity(), contactRequest())
	require.NoError(t, err)
	requireRulingManifold(t, reversed, wall, top, ends, r3.Vec{Z: -1})

	// The certificate is the clearance kernel's own, so Verify reads the
	// same touch as an Exact zero gap.
	verify, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, verify.Status)
	requireExactGap(t, verify, 0)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = doc.ContactPair(canceled, floor, cylinder, r3.Identity(), r3.Identity(), contactRequest())
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, before, doc.Bodies())
}

func TestContactPairCylinderPairRuling(t *testing.T) {
	doc := decad.New()
	// Radius 8 about y = 0 and y = 16; the axial windows overlap on x∈[4,10].
	lower := revolvedCylinder(t, doc, 0, 10, 0, 8)
	upper := revolvedCylinder(t, doc, 4, 14, 16, 8)
	wallLower := surfaceFace[decad.Cylinder](t, lower, func(*decad.Face) bool { return true })
	wallUpper := surfaceFace[decad.Cylinder](t, upper, func(*decad.Face) bool { return true })
	ends := [2]r3.Vec{{X: 4, Y: 8}, {X: 10, Y: 8}}

	report, err := doc.ContactPair(t.Context(), lower, upper, r3.Identity(), r3.Identity(), contactRequest())
	require.NoError(t, err)
	requireRulingManifold(t, report, wallLower, wallUpper, ends, r3.Vec{Y: 1})

	reversed, err := doc.ContactPair(t.Context(), upper, lower, r3.Identity(), r3.Identity(), contactRequest())
	require.NoError(t, err)
	requireRulingManifold(t, reversed, wallUpper, wallLower, ends, r3.Vec{Y: -1})
}

func TestContactPairRulingResolutionGates(t *testing.T) {
	doc := decad.New()
	cylinder := revolvedCylinder(t, doc, 0, 10, 0, 8)
	floor := boxBodyAtZ(t, doc, -20, -20, 20, 20, -18, 10)
	// Exact ends and an exact axis-aligned normal publish at the smallest
	// positive resolutions a request can state.
	req := decad.ContactRequest{
		PointResolution:  units.Millimeters(math.SmallestNonzeroFloat64),
		NormalResolution: units.Radians(math.SmallestNonzeroFloat64),
	}
	report, err := doc.ContactPair(t.Context(), floor, cylinder, r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, report.Relation)
	require.NotNil(t, report.Manifold)
	for _, point := range report.Manifold.Points {
		require.Zero(t, point.OnA.Bound.Base())
		require.Zero(t, point.Normal.Bound.Base())
		require.Zero(t, point.NormalAngle.Base())
	}
}

func TestContactPairRulingRefusals(t *testing.T) {
	tiny := math.Ldexp(1, -30)
	for _, tc := range []struct {
		name string
		// build returns the pair; touching names a relation that stays
		// proved while the manifold is withheld for reason.
		build    func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body)
		touching bool
		reason   decad.ContactReason
	}{
		{name: "near tangent", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return boxBodyAtZ(t, doc, -20, -20, 20, 20, -18-tiny, 10), revolvedCylinder(t, doc, 0, 10, 0, 8)
		}},
		{name: "shallow crossing", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return boxBodyAtZ(t, doc, -20, -20, 20, 20, -18+tiny, 10), revolvedCylinder(t, doc, 0, 10, 0, 8)
		}},
		{name: "overhang", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return boxBodyAtZ(t, doc, 2, -20, 20, 20, -18, 10), revolvedCylinder(t, doc, 0, 10, 0, 8)
		}},
		{name: "channel", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return channelBody(t, doc), verticalCylinder(t, doc, 0, 8, 8)
		}},
		{name: "void wall", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			tubeSketch, tubeProfile := annularSketch(t)
			tube, err := doc.Revolve(tubeSketch, tubeProfile, uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			return boxBodyAtZ(t, doc, -20, -20, 20, 20, -15, 10), tube
		}},
		{name: "arc end", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			halfSketch, halfProfile := semicircleSketch(t)
			half, err := doc.Extrude(halfSketch, halfProfile,
				decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)
			return boxBodyAtZ(t, doc, 10, -20, 30, 20, -5, 20), half
		}},
		{name: "placed", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			cylinder := revolvedCylinder(t, doc, 0, 10, 0, 8)
			placed, err := cylinder.PlacedCopy(t.Context(), contactPose(t, r3.Vec{Z: .5}))
			require.NoError(t, err)
			return boxBodyAtZ(t, doc, -20, -20, 20, 20, -17.5, 10), placed
		}},
		{name: "near tangent pair", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return revolvedCylinder(t, doc, 0, 10, 0, 8), revolvedCylinder(t, doc, 4, 14, 16+tiny, 8)
		}},
		{name: "void wall pair", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			tubeSketch, tubeProfile := annularSketch(t)
			tube, err := doc.Revolve(tubeSketch, tubeProfile, uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			return tube, revolvedCylinder(t, doc, 0, 10, 9, 4)
		}},
		{name: "end to end", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return revolvedCylinder(t, doc, 0, 10, 0, 8), revolvedCylinder(t, doc, 10, 20, 16, 8)
		}},
		{name: "tube", touching: true, reason: decad.ContactPayloadUnsupported,
			build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
				tubeSketch, tubeProfile := annularSketch(t)
				tube, err := doc.Revolve(tubeSketch, tubeProfile, uAxis, decad.FullRevolution{})
				require.NoError(t, err)
				return boxBodyAtZ(t, doc, -20, -20, 20, 20, -25, 10), tube
			}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			a, b := tc.build(t, doc)
			for _, pair := range [][2]*decad.Body{{a, b}, {b, a}} {
				report, err := doc.ContactPair(t.Context(), pair[0], pair[1], r3.Identity(), r3.Identity(),
					contactRequest())
				require.NoError(t, err)
				require.Nil(t, report.Manifold)
				if tc.touching {
					require.Equal(t, decad.ContactTouching, report.Relation)
					require.Equal(t, tc.reason, report.Reason)
					continue
				}
				require.NotEqual(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
			}
		})
	}
}

// channelBody is a U-shaped prism whose notch floor y = 0 faces +y between
// walls at x = ±6, standing on z∈[-10,10].
func channelBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	corners := [][2]float64{{-20, -20}, {20, -20}, {20, 20}, {6, 20}, {6, 0}, {-6, 0}, {-6, 20}, {-20, 20}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// verticalCylinder extrudes a circle of radius r centered at (cx, cy) over
// z∈[-5,5].
func verticalCylinder(t *testing.T, doc *decad.Document, cx, cy, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -5)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(cx, cy)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// The placed-pose fixtures of docs/contact-geometry-design.md §4.5: a radius-10
// source cylinder along world X lies on a floor whose top face is z = −10, at
// query poses other than the identity. Every coordinate is dyadic. A
// signed-axis pose proves an exact touch or gap; any other pose stages the
// section through a float basis that is orthonormal only to rounding, and the
// relation is a gap or a ContactBand.
//
// Each band, ball and separation is checked against the true occupied set of
// the float pose, computed here in 512-bit arithmetic from the pose's own
// entries: an end disk's least height along Z is c_z − r·ρ with
// ρ = |(B_zy, B_zz)|, at the point c − (r/ρ)·(B_zy·EY + B_zz·EZ).
//
// Legs shown to fail (each deleted or zeroed in turn, fixture red, then
// restored):
//   - the gram term of sectionDrift: "long roll" publishes a band that misses
//     the true least height;
//   - the α² term of sectionDrift: "tilted" publishes a separation bound the
//     rising end's true least height exceeds;
//   - the end heights max|H±| in the band: "tilted" publishes a band below its
//     rising end;
//   - the gram term of rimDrift: "long roll" publishes rim balls that miss the
//     true lowest points;
//   - the α term of rimDrift: "tilted" publishes rim balls that miss them;
//   - the foot check: "off the face" reads Touching;
//   - the threshold's gram term: "long roll" reads a gap or nothing instead of
//     its band, and its tilt term |α|·L: "lifted tilt" reads Separated.
//
// The gram ≤ 1/16 gate cannot fail for a valid pose: Transform.IsValid holds
// every column within 1e-9 of unit length and every pair within 1e-9 of
// orthogonal, so gram stays below 1e-8.

// rolledPose turns n times by angle about world X through the origin, the
// float composition a long roll accumulates: its basis drifts from
// orthonormal by about n ulps.
func rolledPose(t *testing.T, n int, angle float64) r3.Transform {
	t.Helper()
	step, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(angle))
	require.NoError(t, err)
	pose := r3.Identity()
	for range n {
		pose, err = pose.Then(step)
		require.NoError(t, err)
	}
	return pose
}

// tiltedPose tips the cylinder about world Y by phi and lifts it so its +X
// end center sits at the height lift, to rounding.
func tiltedPose(t *testing.T, phi, lift float64) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Radians(phi))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{Z: lift - turn.Apply(r3.Vec{X: 15}).Z})
	require.NoError(t, err)
	pose, err := turn.Then(shift)
	require.NoError(t, err)
	return pose
}

const bigPrecision = 512

func bigOf(v float64) *big.Float { return new(big.Float).SetPrec(bigPrecision).SetFloat64(v) }

// trueRim is the exact least height above z = −10 of the end disk whose
// identity center is (x, axisY, 0), radius 10, under pose, and its lowest
// point.
func trueRim(pose r3.Transform, x, axisY float64) (*big.Float, [3]*big.Float) {
	b := pose.Basis()
	at := pose.Translation()
	cols := [3][3]float64{{b.EX.X, b.EX.Y, b.EX.Z}, {b.EY.X, b.EY.Y, b.EY.Z}, {b.EZ.X, b.EZ.Y, b.EZ.Z}}
	trans := [3]float64{at.X, at.Y, at.Z}
	var center, point [3]*big.Float
	for k := range 3 {
		center[k] = bigOf(trans[k])
		center[k].Add(center[k], new(big.Float).Mul(bigOf(x), bigOf(cols[0][k])))
		center[k].Add(center[k], new(big.Float).Mul(bigOf(axisY), bigOf(cols[1][k])))
	}
	rho := new(big.Float).Add(new(big.Float).Mul(bigOf(cols[1][2]), bigOf(cols[1][2])),
		new(big.Float).Mul(bigOf(cols[2][2]), bigOf(cols[2][2])))
	rho.Sqrt(rho)
	scale := new(big.Float).Quo(bigOf(10), rho)
	for k := range 3 {
		offset := new(big.Float).Add(new(big.Float).Mul(bigOf(cols[1][2]), bigOf(cols[1][k])),
			new(big.Float).Mul(bigOf(cols[2][2]), bigOf(cols[2][k])))
		point[k] = new(big.Float).Sub(center[k], offset.Mul(offset, scale))
	}
	height := new(big.Float).Add(center[2], bigOf(10))
	height.Sub(height, new(big.Float).Mul(bigOf(10), rho))
	return height, point
}

// requireWithin checks |value| <= bound exactly.
func requireWithin(t *testing.T, value *big.Float, bound float64, msg string) {
	t.Helper()
	require.LessOrEqual(t, new(big.Float).Abs(value).Cmp(bigOf(bound)), 0, "%s: %v beyond %v", msg, value, bound)
}

// requireBallHolds checks that point lies in the ball exactly.
func requireBallHolds(t *testing.T, ball decad.VecMeasurement, point [3]*big.Float, msg string) {
	t.Helper()
	squared := new(big.Float).SetPrec(bigPrecision)
	for k, v := range [3]float64{ball.Value.X, ball.Value.Y, ball.Value.Z} {
		d := new(big.Float).Sub(point[k], bigOf(v))
		squared.Add(squared, d.Mul(d, d))
	}
	limit := bigOf(ball.Bound.Base())
	require.LessOrEqual(t, squared.Cmp(limit.Mul(limit, limit)), 0, "%s: point outside ball %+v", msg, ball)
}

// requirePlacedBand checks a band report against the true occupied set at
// the cylinder's pose: the gap band holds the least height, and each
// published end holds its true lowest point, its foot and its height.
func requirePlacedBand(t *testing.T, report *decad.ContactReport, pose r3.Transform, axisY float64,
	floorFirst bool) {
	t.Helper()
	require.Equal(t, decad.ContactBand, report.Relation, "reason=%v", report.Reason)
	require.NotNil(t, report.Gap)
	require.Zero(t, report.Gap.Value.Base())
	require.NotNil(t, report.Manifold, "reason=%v", report.Reason)
	require.Len(t, report.Manifold.Points, 2)
	normal := r3.Vec{Z: 1}
	if !floorFirst {
		normal = normal.Scale(-1)
	}
	for i, x := range []float64{-15, 15} {
		height, rim := trueRim(pose, x, axisY)
		requireWithin(t, height, report.Gap.Bound.Base(), "gap band")
		point := report.Manifold.Points[i]
		onFloor, onCylinder := point.OnA, point.OnB
		if !floorFirst {
			onFloor, onCylinder = point.OnB, point.OnA
		}
		requireBallHolds(t, onCylinder, rim, "rim")
		requireBallHolds(t, onFloor, [3]*big.Float{rim[0], rim[1], bigOf(-10)}, "foot")
		requireWithin(t, height, point.Separation.Bound.Base(), "separation")
		require.Zero(t, point.Separation.Value.Base())
		require.Equal(t, normal, point.Normal.Value)
		require.Zero(t, point.NormalAngle.Base())
	}
}

func TestContactPairPlacedRuling(t *testing.T) {
	floorScene := func(t *testing.T, axisY, x0 float64) (*decad.Document, *decad.Body, *decad.Body) {
		doc := decad.New()
		cylinder := revolvedCylinder(t, doc, -15, 15, axisY, 10)
		floor := boxBodyAtZ(t, doc, x0, -200, 100, 200, -20, 10)
		return doc, floor, cylinder
	}
	pair := func(t *testing.T, doc *decad.Document, floor, cylinder *decad.Body, pose r3.Transform,
		req decad.ContactRequest, floorFirst bool) *decad.ContactReport {
		t.Helper()
		a, b, poseA, poseB := floor, cylinder, r3.Identity(), pose
		if !floorFirst {
			a, b, poseA, poseB = cylinder, floor, pose, r3.Identity()
		}
		report, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, req)
		require.NoError(t, err)
		return report
	}
	wide := contactRequest()
	wide.PointResolution = units.Millimeters(.05)

	for _, floorFirst := range []bool{true, false} {
		t.Run(map[bool]string{true: "floor first", false: "cylinder first"}[floorFirst], func(t *testing.T) {
			t.Run("translated", func(t *testing.T) {
				// A signed-axis pose keeps the section a true disk: the rims
				// touch the floor exactly at (−14, 5.375, −10) and (16, 5.375, −10).
				doc, floor, cylinder := floorScene(t, rollingAxisY, -100)
				wall := surfaceFace[decad.Cylinder](t, cylinder, func(*decad.Face) bool { return true })
				top := surfaceFace[decad.Plane](t, floor, func(face *decad.Face) bool {
					reading, err := face.NormalAt(r3.Vec{Z: -10})
					return err == nil && reading.Value == r3.Vec{Z: 1}
				})
				report := pair(t, doc, floor, cylinder, contactPose(t, r3.Vec{X: 1, Y: 2}), contactRequest(), floorFirst)
				require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
				require.Equal(t, decad.Exact, report.Gap.Exactness)
				require.Zero(t, report.Gap.Value.Base())
				require.NotNil(t, report.Manifold)
				require.Len(t, report.Manifold.Points, 2)
				for i, end := range []r3.Vec{{X: -14, Y: 5.375, Z: -10}, {X: 16, Y: 5.375, Z: -10}} {
					point := report.Manifold.Points[i]
					require.Equal(t, end, point.OnA.Value)
					require.Equal(t, end, point.OnB.Value)
					require.Zero(t, point.OnA.Bound.Base())
					require.Zero(t, point.OnB.Bound.Base())
					require.Equal(t, decad.Exact, point.Separation.Exactness)
					onFloor, onCylinder := point.FaceA, point.FaceB
					normal := r3.Vec{Z: 1}
					if !floorFirst {
						onFloor, onCylinder, normal = point.FaceB, point.FaceA, normal.Scale(-1)
					}
					require.Same(t, top, onFloor)
					require.Same(t, wall, onCylinder)
					require.Equal(t, normal, point.Normal.Value)
				}
				lifted := pair(t, doc, floor, cylinder, contactPose(t, r3.Vec{X: 1, Z: .5}), contactRequest(), floorFirst)
				require.Equal(t, decad.ContactSeparated, lifted.Relation)
				require.Equal(t, .5, lifted.Gap.Value.Base())
				require.Zero(t, lifted.Gap.Bound.Base())
				sunk := pair(t, doc, floor, cylinder, contactPose(t, r3.Vec{X: 1, Z: -.5}), contactRequest(), floorFirst)
				require.Equal(t, decad.ContactUndecided, sunk.Relation)
				require.Nil(t, sunk.Manifold)
			})
			t.Run("quarter turn", func(t *testing.T) {
				// A quarter turn about X carries the axis y = 3.375 to z = 3.375;
				// the translation puts it back on the floor exactly.
				doc, floor, cylinder := floorScene(t, rollingAxisY, -100)
				pose, err := r3.FromBasis(r3.Basis{EX: r3.Vec{X: 1}, EY: r3.Vec{Z: 1}, EZ: r3.Vec{Y: -1}},
					r3.Vec{Y: rollingAxisY, Z: -rollingAxisY})
				require.NoError(t, err)
				report := pair(t, doc, floor, cylinder, pose, contactRequest(), floorFirst)
				require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
				require.NotNil(t, report.Manifold)
				require.Equal(t, r3.Vec{X: -15, Y: rollingAxisY, Z: -10}, report.Manifold.Points[0].OnA.Value)
			})
			t.Run("long roll", func(t *testing.T) {
				// 65536 turns of 0.7 rad leave a basis some 1e-12 off
				// orthonormal: the section is no longer a true disk, its least
				// height is off zero by about r times that, and its lowest
				// point drifts off c − r·n̂ by as much.
				doc, floor, cylinder := floorScene(t, 0, -100)
				pose := rolledPose(t, 1<<16, .7)
				report := pair(t, doc, floor, cylinder, pose, contactRequest(), floorFirst)
				requirePlacedBand(t, report, pose, 0, floorFirst)
				// The band is negligible against every request in this suite.
				require.Less(t, report.Gap.Bound.Base(), 1e-9)
				height, _ := trueRim(pose, 15, 0)
				require.NotZero(t, height.Sign(), "the fixture must leave the true least height off zero")
				deeper, err := pose.Then(contactPose(t, r3.Vec{Z: -1e-6}))
				require.NoError(t, err)
				sunk := pair(t, doc, floor, cylinder, deeper, contactRequest(), floorFirst)
				require.Equal(t, decad.ContactUndecided, sunk.Relation)
			})
			t.Run("tilted", func(t *testing.T) {
				// Tipped by 2^-10 rad about Y with its +X end center at the
				// floor: the −X end center rises 30·sin φ, about 0.029 mm, and
				// the tipped disks reach r·(1 − cos φ) below their centers less r.
				doc, floor, cylinder := floorScene(t, 0, -100)
				pose := tiltedPose(t, math.Ldexp(1, -10), 0)
				requirePlacedBand(t, pair(t, doc, floor, cylinder, pose, wide, floorFirst), pose, 0, floorFirst)
			})
			t.Run("lifted tilt", func(t *testing.T) {
				// Lifted 0.02 mm, the pair still touches within the tilt's
				// reach |α|·L, about 0.029 mm; lifted 0.04 mm it is apart.
				doc, floor, cylinder := floorScene(t, 0, -100)
				near := tiltedPose(t, math.Ldexp(1, -10), .02)
				requirePlacedBand(t, pair(t, doc, floor, cylinder, near, wide, floorFirst), near, 0, floorFirst)
				far := tiltedPose(t, math.Ldexp(1, -10), .04)
				report := pair(t, doc, floor, cylinder, far, wide, floorFirst)
				require.Equal(t, decad.ContactSeparated, report.Relation)
				height, _ := trueRim(far, 15, 0)
				gap := new(big.Float).Sub(height, bigOf(report.Gap.Value.Base()))
				requireWithin(t, gap, report.Gap.Bound.Base(), "gap")
			})
			t.Run("off the face", func(t *testing.T) {
				// The floor ends at x = −10, inside the ruling's x∈[−14, 16].
				doc, floor, cylinder := floorScene(t, rollingAxisY, -10)
				report := pair(t, doc, floor, cylinder, contactPose(t, r3.Vec{X: 1}), contactRequest(), floorFirst)
				require.Equal(t, decad.ContactUndecided, report.Relation)
				require.Nil(t, report.Manifold)
			})
			t.Run("standing", func(t *testing.T) {
				// An axis along the normal has no ruling on the floor.
				doc, floor, cylinder := floorScene(t, 0, -100)
				pose := tiltedPose(t, math.Pi/2, 0)
				report := pair(t, doc, floor, cylinder, pose, contactRequest(), floorFirst)
				require.Equal(t, decad.ContactUndecided, report.Relation)
			})
		})
	}
}
