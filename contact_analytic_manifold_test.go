package decad_test

import (
	"context"
	"math"
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
