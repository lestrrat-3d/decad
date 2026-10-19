package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// coneFromFaceSpiralSection is the six-point axial section of the 31 tooth,
// module 1, 35 degree spiral bevel blank that the spiral first-join fixture
// revolves (apitest/spiral_first_join_fixture_test.go). Its heel side lies on
// x + r = C with C = 31 + 2^-48, a float, so nearest rounding of C is exact.
func coneFromFaceSpiralSection() []planePoint {
	const module, teeth = 1.0, 31
	r := module * teeth / 2
	gamma := math.Atan2(math.Sin(math.Pi/2)*r, r+math.Cos(math.Pi/2)*r)
	coneDist := r / math.Sin(gamma)
	width := math.Min(math.Hypot(2*r, 2*r)/6, 0.95*coneDist*math.Pow(math.Sin(gamma), 2))
	dedendum := 1.25 * module
	baseHeight := math.Min(math.Max(module*teeth/8, 1.05*dedendum*math.Sin(gamma)),
		0.95*(r-dedendum*math.Cos(gamma))*math.Tan(gamma))
	heelStation := coneDist*math.Cos(gamma) + baseHeight
	heelRadius := r - baseHeight/math.Tan(gamma)
	ded := planePoint{U: coneDist*math.Cos(gamma) + dedendum*math.Sin(gamma), V: r - dedendum*math.Cos(gamma)}
	toeRadius := r - width/math.Sin(gamma)
	apexToDed := math.Hypot(coneDist, dedendum)
	toeFactor := 1 - width*apexToDed/coneDist/apexToDed
	toeRoot := planePoint{U: toeFactor * ded.U, V: toeFactor * ded.V}
	toeStation := toeRoot.U + (toeRoot.V-toeRadius)/math.Cos(gamma)*math.Sin(gamma)
	return []planePoint{{U: toeStation}, {U: heelStation}, {U: heelStation, V: heelRadius},
		ded, toeRoot, {U: toeStation, V: toeRadius}}
}

// coneFromFaceSelector names the section side from a to b by its creating
// origin, which is the documented body-relative way to select one face.
func coneFromFaceSelector(t *testing.T, blank *Body, a, b planePoint) FaceSelector {
	t.Helper()
	spans, _, _, ok := revolveshell.RecordedSection(bodyNode(blank))
	require.True(t, ok)
	for _, span := range spans {
		if span.Face != nil && (span.Start == a && span.End == b || span.Start == b && span.End == a) {
			require.NotEmpty(t, span.Origins)
			return Faces(FaceCreatedBy(featureRefOf(span.Origins[0]))).Exactly(1)
		}
	}
	require.FailNow(t, "section side not found")
	return nil
}

func TestConeFromFaceIsStrictlyOffBlankFaces(t *testing.T) {
	section := coneFromFaceSpiralSection()
	toeRoot, toeOuter, ded, heelOuter := section[4], section[5], section[3], section[2]
	far := units.Millimeters(-5)
	tests := []struct {
		name  string
		a, b  planePoint
		first planePoint
		last  planePoint
		sign  int
	}{
		{name: "heel", a: ded, b: heelOuter, first: ded, last: heelOuter, sign: -1},
		{name: "toe", a: toeRoot, b: toeOuter, first: toeRoot, last: toeOuter, sign: 1},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			blank := spiralTestRevolveBlank(t, section)
			doc := documentHandle(blank.n.Doc)
			before := len(doc.Bodies())
			cone, err := ConeFromFace(t.Context(), blank, coneFromFaceSelector(t, blank, tc.a, tc.b), far)
			require.NoError(t, err)
			require.Len(t, doc.Bodies(), before+1)
			require.NoError(t, doc.requireLive(blank), "the source stays live")
			limit, ok := exactPointCone(cone)
			require.True(t, ok, "the result must be an exact point cone")
			gap, ok := spiralsector.ToolOuterGap(limit, tc.first, tc.last, tc.sign)
			require.True(t, ok, "both face ends must lie strictly on the certified side")
			require.Positive(t, gap.Sign())
			require.True(t, spiralsector.FiniteConeSide(limit, tc.first, tc.last, tc.sign),
				"the whole finite generator must lie on the certified side")
			require.Zero(t, limit.Far.Cmp(lfF(-5)), "far plane is the requested one")
		})
	}
}

func TestConeFromFaceRefuses(t *testing.T) {
	section := coneFromFaceSpiralSection()
	cylinderSection := []planePoint{{U: 1}, {U: 3}, {U: 3, V: 2}, {U: 2, V: 2}, {U: 1, V: 1}}
	faceOf := func(t *testing.T, body *Body, want func(Surface) bool) FaceSelector {
		t.Helper()
		for _, face := range body.Faces() {
			if want(face.Surface()) {
				return Faces(FaceCreatedBy(featureRefOf(face.originRefs()[0]))).Exactly(1)
			}
		}
		require.FailNow(t, "no face of the wanted kind")
		return nil
	}
	isPlane := func(s Surface) bool { _, ok := s.(Plane); return ok }
	isCylinder := func(s Surface) bool { _, ok := s.(Cylinder); return ok }
	isCone := func(s Surface) bool { _, ok := s.(Cone); return ok }
	far := units.Millimeters(-5)

	t.Run("plane side", func(t *testing.T) {
		blank := spiralTestRevolveBlank(t, section)
		_, err := ConeFromFace(t.Context(), blank, faceOf(t, blank, isPlane), far)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "no offset cone")
	})
	t.Run("cylinder side", func(t *testing.T) {
		blank := spiralTestRevolveBlank(t, cylinderSection)
		_, err := ConeFromFace(t.Context(), blank, faceOf(t, blank, isCylinder), far)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "no offset cone")
	})
	t.Run("placed blank", func(t *testing.T) {
		blank := spiralTestRevolveBlank(t, section)
		move, err := r3.Translation(r3.Vec{X: 1})
		require.NoError(t, err)
		placed, err := Transform(t.Context(), blank, move)
		require.NoError(t, err)
		_, err = ConeFromFace(t.Context(), placed, faceOf(t, placed, isCone), far)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("retired body", func(t *testing.T) {
		blank := spiralTestRevolveBlank(t, section)
		sel := faceOf(t, blank, isCone)
		require.NoError(t, Remove(blank))
		_, err := ConeFromFace(t.Context(), blank, sel, far)
		require.ErrorIs(t, err, ErrRetiredBody)
	})
	t.Run("far", func(t *testing.T) {
		blank := spiralTestRevolveBlank(t, section)
		sel := faceOf(t, blank, isCone)
		for name, value := range map[string]units.Value{
			"above the face": units.Millimeters(100),
			"at the face":    units.Millimeters(section[3].U),
			"nonnegative":    units.Millimeters(0),
			// Below the face but above the origin: exactPointCone needs far < 0.
			"positive below the face": units.Millimeters(1),
		} {
			_, err := ConeFromFace(t.Context(), blank, sel, value)
			require.ErrorIs(t, err, ErrUnsupported, name)
		}
		for name, value := range map[string]units.Value{
			"not a length": units.Seconds(1),
			"nan":          units.Millimeters(math.NaN()),
			"infinite":     units.Millimeters(math.Inf(-1)),
		} {
			_, err := ConeFromFace(t.Context(), blank, sel, value)
			require.ErrorIs(t, err, ErrDegenerate, name)
		}
		require.Len(t, documentHandle(blank.n.Doc).Bodies(), 1, "refusals leave the document unchanged")
	})
	t.Run("selector cardinality", func(t *testing.T) {
		blank := spiralTestRevolveBlank(t, section)
		_, err := ConeFromFace(t.Context(), blank, Faces(), far)
		require.ErrorIs(t, err, ErrCardinality)
		_, err = ConeFromFace(t.Context(), blank, nil, far)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	t.Run("nil arguments", func(t *testing.T) {
		blank := spiralTestRevolveBlank(t, section)
		sel := faceOf(t, blank, isCone)
		_, err := ConeFromFace(nil, blank, sel, far) //nolint:staticcheck // a nil context is the case under test.
		require.ErrorIs(t, err, ErrDegenerate)
		_, err = ConeFromFace(t.Context(), nil, sel, far)
		require.ErrorIs(t, err, ErrDegenerate)
	})
}

func TestConeFromFaceCancelled(t *testing.T) {
	blank := spiralTestRevolveBlank(t, coneFromFaceSpiralSection())
	sel := coneFromFaceSelector(t, blank, coneFromFaceSpiralSection()[3], coneFromFaceSpiralSection()[2])
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	before := documentHandle(blank.n.Doc).Bodies()
	_, err := ConeFromFace(ctx, blank, sel, units.Millimeters(-5))
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, before, documentHandle(blank.n.Doc).Bodies())
}

// coneFromFaceNonConvexSection is a counter-clockwise, non-convex section
// whose selected side (1, 3)-(2, 1) is a cone with the outside half-plane
// x + r/2 < 5/2, a side the notch of the section cuts back into the blank.
func coneFromFaceNonConvexSection() []planePoint {
	return []planePoint{{U: 2}, {U: 3}, {U: 3, V: 5}, {U: 1, V: 5}, {U: 1, V: 3}, {U: 2, V: 1}}
}

// coneFromFaceShallowSection has a cone side whose strictly offset generator
// has slope 1e-9 against its length, the boundary where Revolve names a wall
// a cylinder. Sketch rejects every triangle that shallow, so the cone is
// refused while it is built.
func coneFromFaceShallowSection() []planePoint {
	const x1, xe = 1.0, 2.0000000009999998e+09
	return []planePoint{{U: xe}, {U: xe + 1e9}, {U: xe + 1e9, V: 1e5}, {U: x1, V: 1e5}, {U: x1, V: 1002},
		{U: xe, V: 1000}}
}

// coneFromFaceNextFeatureOrigins revolves a small triangle into d and returns
// the creating origins of its faces, which carry the producer numbering.
func coneFromFaceNextFeatureOrigins(t *testing.T, d *Document) [][]provenance.Ref {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	corners := []planePoint{{U: 10}, {U: 12}, {U: 12, V: 2}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c.U, c.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	next, err := Revolve(t.Context(), d, s, s.Profiles()[0],
		SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	var origins [][]provenance.Ref
	for _, face := range next.Faces() {
		origins = append(origins, face.originRefs())
	}
	return origins
}

// coneFromFaceRefusalKeepsDocument runs a refused ConeFromFace and checks that
// the document, its provenance numbering and the next produced feature match a
// control document that never saw the call.
func coneFromFaceRefusalKeepsDocument(t *testing.T, section []planePoint, a, b planePoint, far units.Value,
	want error, message string) {
	t.Helper()
	control := spiralTestRevolveBlank(t, section)
	wantOrigins := coneFromFaceNextFeatureOrigins(t, documentHandle(control.n.Doc))

	blank := spiralTestRevolveBlank(t, section)
	doc := documentHandle(blank.n.Doc)
	before := doc.Bodies()
	producer := doc.nextProducerID()
	_, err := ConeFromFace(t.Context(), blank, coneFromFaceSelector(t, blank, a, b), far)
	require.ErrorIs(t, err, want)
	require.ErrorContains(t, err, message)
	require.Equal(t, before, doc.Bodies())
	require.Equal(t, producer, doc.nextProducerID())
	require.Equal(t, wantOrigins, coneFromFaceNextFeatureOrigins(t, doc))
}

func TestConeFromFaceShallowGeneratorKeepsDocument(t *testing.T) {
	section := coneFromFaceShallowSection()
	coneFromFaceRefusalKeepsDocument(t, section, section[4], section[5], units.Millimeters(-1e9),
		ErrUnsupported, "not one closed profile")
}

// TestCommitExactPointConeRefusalKeepsDocument builds a body that is a Revolve
// but not a point cone and checks that the refusal precedes registration.
func TestCommitExactPointConeRefusalKeepsDocument(t *testing.T) {
	blank := spiralTestRevolveBlank(t, coneFromFaceSpiralSection())
	doc := documentHandle(blank.n.Doc)
	before := doc.Bodies()
	producer := doc.nextProducerID()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	corners := []planePoint{{U: 10}, {U: 12}, {U: 12, V: 2}, {U: 10, V: 2}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c.U, c.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	built, err := revolveProfileBuild(t.Context(), doc, s, s.Profiles()[0],
		SketchLine{Start: planePoint{}, End: planePoint{U: 1}}, FullRevolution{}, nil)
	require.NoError(t, err)
	require.Equal(t, before, doc.Bodies(), "building registers nothing")
	require.Equal(t, producer, doc.nextProducerID())

	err = commitExactPointCone(t.Context(), doc, built)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	require.Equal(t, producer, doc.nextProducerID())
}

func TestConeFromFaceNonConvexSectionIsAccepted(t *testing.T) {
	section := coneFromFaceNonConvexSection()
	blank := spiralTestRevolveBlank(t, section)
	cone, err := ConeFromFace(t.Context(), blank, coneFromFaceSelector(t, blank, section[4], section[5]),
		units.Millimeters(-5))
	require.NoError(t, err)
	limit, ok := exactPointCone(cone)
	require.True(t, ok)
	// The side lies on x + r/2 = 5/2 and the empty half-plane is x + r/2 < 5/2,
	// so the apex is the float just below 5/2 and the far radius the float
	// just below (5/2+5)/(1/2) = 15.
	wantApex := math.Nextafter(2.5, math.Inf(-1))
	wantRadius := math.Nextafter(15, math.Inf(-1))
	require.Zero(t, limit.Apex.Cmp(new(big.Rat).SetFloat64(wantApex)))
	radius := new(big.Rat).Quo(new(big.Rat).Sub(limit.Apex, limit.Far), limit.Slope)
	require.Zero(t, radius.Cmp(new(big.Rat).SetFloat64(wantRadius)))
}

func TestConeFromFaceMoreRefusals(t *testing.T) {
	far := units.Millimeters(-5)
	t.Run("zero-body", func(t *testing.T) {
		_, err := ConeFromFace(t.Context(), bornBody(&Body{}), Faces(), far)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	extrudedBox := func(t *testing.T) *Body {
		t.Helper()
		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		require.NoError(t, err)
		a, b := s.CreatePoint(0, 0), s.CreatePoint(4, 0)
		c, e := s.CreatePoint(4, 3), s.CreatePoint(0, 3)
		for _, p := range []*sketch.Point{a, b, c, e} {
			s.Fix(p)
		}
		s.CreateLine(a, b)
		s.CreateLine(b, c)
		s.CreateLine(c, e)
		s.CreateLine(e, a)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		box, err := Extrude(t.Context(), New(), s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
		require.NoError(t, err)
		return box
	}
	t.Run("non-revolve body", func(t *testing.T) {
		box := extrudedBox(t)
		sel := Faces(FaceCreatedBy(featureRefOf(box.Faces()[0].originRefs()[0]))).Exactly(1)
		_, err := ConeFromFace(t.Context(), box, sel, far)
		require.ErrorIs(t, err, ErrUnsupported)
		require.Len(t, documentHandle(box.n.Doc).Bodies(), 1)
	})
	t.Run("selectors", func(t *testing.T) {
		blank := spiralTestRevolveBlank(t, coneFromFaceSpiralSection())
		before := blank.n.Doc.NextProducer
		foreign := extrudedBox(t).Faces()[0].originRefs()[0]
		// A selector that matches nothing and one that fails its assertion
		// are both cardinality failures of the implicit exactly-one.
		_, err := ConeFromFace(t.Context(), blank, Faces(FaceCreatedBy(featureRefOf(foreign))), far)
		require.ErrorIs(t, err, ErrCardinality)
		_, err = ConeFromFace(t.Context(), blank, Faces().Exactly(2), far)
		require.ErrorIs(t, err, ErrCardinality)
		require.Len(t, documentHandle(blank.n.Doc).Bodies(), 1)
		require.Equal(t, before, blank.n.Doc.NextProducer)
	})
}

// TestConeFromFaceCancelledDuringConstruction cancels at every context poll
// after the entry check, up to the first call that completes, and checks that
// each canceled call leaves the document and its numbering unchanged.
func TestConeFromFaceCancelledDuringConstruction(t *testing.T) {
	section := coneFromFaceSpiralSection()
	canceledLate := false
	for cancelAt := 2; ; cancelAt++ {
		require.Less(t, cancelAt, 200, "the call never completed")
		blank := spiralTestRevolveBlank(t, section)
		sel := coneFromFaceSelector(t, blank, section[3], section[2])
		doc := documentHandle(blank.n.Doc)
		before, producer := doc.Bodies(), doc.nextProducerID()
		ctx := newCancelAfterContext(t.Context(), cancelAt)
		_, err := ConeFromFace(ctx, blank, sel, units.Millimeters(-5))
		if err == nil {
			require.Greater(t, cancelAt, 3, "the build polls the context after the entry check")
			require.True(t, canceledLate, "a cancellation landed inside construction")
			return
		}
		require.ErrorIs(t, err, context.Canceled, "cancelAt=%d", cancelAt)
		require.Equal(t, before, doc.Bodies())
		require.Equal(t, producer, doc.nextProducerID())
		canceledLate = canceledLate || cancelAt > 3
	}
}
