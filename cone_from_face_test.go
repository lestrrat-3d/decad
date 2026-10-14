package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
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
	spans, _, _, ok := recordedRevolveSection(blank)
	require.True(t, ok)
	for _, span := range spans {
		if span.face != nil && (span.start == a && span.end == b || span.start == b && span.end == a) {
			require.NotEmpty(t, span.origins)
			return Faces(FaceCreatedBy(span.origins[0])).Exactly(1)
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
			doc := blank.doc
			before := len(doc.Bodies())
			cone, err := ConeFromFace(t.Context(), blank, coneFromFaceSelector(t, blank, tc.a, tc.b), far)
			require.NoError(t, err)
			require.Len(t, doc.Bodies(), before+1)
			require.NoError(t, doc.requireLive(blank), "the source stays live")
			limit, ok := exactPointCone(cone)
			require.True(t, ok, "the result must be an exact point cone")
			gap, ok := spiralToolOuterGap(limit, tc.first, tc.last, tc.sign)
			require.True(t, ok, "both face ends must lie strictly on the certified side")
			require.Positive(t, gap.Sign())
			require.True(t, spiralFiniteConeSide(limit, tc.first, tc.last, tc.sign),
				"the whole finite generator must lie on the certified side")
			require.Zero(t, limit.far.Cmp(lfF(-5)), "far plane is the requested one")
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
				return Faces(FaceCreatedBy(face.Origins()[0])).Exactly(1)
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
		require.NoError(t, blank.doc.Remove(blank))
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
		require.Len(t, blank.doc.Bodies(), 1, "refusals leave the document unchanged")
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
	before := blank.doc.Bodies()
	_, err := ConeFromFace(ctx, blank, sel, units.Millimeters(-5))
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, before, blank.doc.Bodies())
}
