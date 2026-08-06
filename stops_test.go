package decad_test

import (
	"encoding/json"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// roleCapEnd is the provenance role of an extrude/revolve end cap.
const roleCapEnd = "capEnd"

// plateAndPin builds one solved sketch holding two disjoint profiles: the
// 100×60 plate at the origin and a 20×20 pin footprint beside it.
func plateAndPin(t *testing.T) (*sketch.Sketch, *sketch.Profile, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateRectangle(120, 0, 140, 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var plate, pin *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Area > 1000 {
			plate = p
		} else {
			pin = p
		}
	}
	require.NotNil(t, plate)
	require.NotNil(t, pin)
	return s, plate, pin
}

// capEndFace selects a body's end cap by provenance.
func capEndFace(b *decad.Body) *decad.FaceQuery {
	return decad.Faces(decad.FaceCreatedBy(decad.FeatureRef{Step: b.Origin().Step, Role: roleCapEnd}))
}

// capStartFace selects a body's start cap by provenance.
func capStartFace(b *decad.Body) *decad.FaceQuery {
	return decad.Faces(decad.FaceCreatedBy(decad.FeatureRef{Step: b.Origin().Step, Role: roleCapStart}))
}

func TestExtrudeThroughAll(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	pin, err := doc.Extrude(s, pinProf, decad.ThroughAll{Dir: decad.Along})
	require.NoError(t, err)
	requireVolume(t, pin, 4000)
	requireBounds(t, pin, 120, 0, 0, 140, 20, 10)
	requireManifold(t, pin)

	// The stop body is a dependency, not an operand: it stays live, and its
	// StepRef is recorded in the step's Inputs (core §6.2).
	require.Contains(t, doc.Bodies(), plate)
	steps := doc.Recipe().Steps
	require.Len(t, steps, 2)
	require.Equal(t, []decad.StepRef{plate.Origin().Step}, steps[1].Inputs)
	require.Equal(t, decad.ThroughAll{Dir: decad.Along}, steps[1].Extent)

	// The recorded step round-trips through the wire codec.
	buf, err := json.Marshal(steps[1])
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, steps[1], got)

	// The two bodies are box-disjoint, so Verify proves the model Sound —
	// stop-built bodies are ordinary analytic prisms.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Trustworthy())
	for _, br := range report.Bodies {
		require.Equal(t, decad.Sound, br.Status)
	}
}

func TestExtrudeThroughAllRefusesDisplacedPrismExtentOnYZPlane(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	const shift = 1e9
	b := placedFar(t, boxBody(t, doc, 2-shift, 2, 8-shift, 8, 10), shift)
	union, err := decad.Union(a, b)
	require.NoError(t, err)
	box, err := union.Bounds()
	require.NoError(t, err)
	require.Positive(t, box.Bound.Base())

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.YZ())
	require.NoError(t, err)
	profile := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(profile.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	before := snapshotDocument(t, doc)

	for _, tc := range []struct {
		name   string
		extent decad.Extent
	}{
		{name: "through all", extent: decad.ThroughAll{Dir: decad.Along}},
		{name: "through all side", extent: decad.TwoSided{
			One: decad.ThroughAllSide{},
			Two: decad.DistanceSide{D: units.Millimeters(1)},
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := doc.Extrude(s, s.Profiles()[0], tc.extent)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, "proven section displacement")
			requireDocumentUnchanged(t, doc, before)
		})
	}
}

func TestExtrudeThroughAllStacked(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	lower, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	second, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(0, 0, 25))
	require.NoError(t, err)
	upper, err := second.Placed(move)
	require.NoError(t, err)

	// The sweep runs through the far side of EVERY live body it meets:
	// lower spans [0, 10], upper [25, 35], so the stop is 35 — and both stop
	// bodies are recorded, in stop order along the sweep.
	pin, err := doc.Extrude(s, pinProf, decad.ThroughAll{Dir: decad.Along})
	require.NoError(t, err)
	requireVolume(t, pin, 400*35)
	requireBounds(t, pin, 120, 0, 0, 140, 20, 35)
	steps := doc.Recipe().Steps
	require.Equal(t, []decad.StepRef{lower.Origin().Step, upper.Origin().Step}, steps[len(steps)-1].Inputs)

	// Nothing lies below the sketch plane, so an Against through-all has no
	// stop at all.
	_, err = doc.Extrude(s, pinProf, decad.ThroughAll{Dir: decad.Against})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.NotErrorIs(t, err, decad.ErrUnsupported)
}

func TestExtrudeThroughAllSides(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	above, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	_, err = doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(6), Dir: decad.Against})
	require.NoError(t, err)

	// One through-all side, one distance side: [−3, 10]. The plate below
	// has no material above the sketch plane, so only the plate above is a
	// stop body of the along side.
	pin, err := doc.Extrude(s, pinProf, decad.TwoSided{
		One: decad.ThroughAllSide{},
		Two: decad.DistanceSide{D: units.Millimeters(3)},
	})
	require.NoError(t, err)
	requireVolume(t, pin, 400*13)
	requireBounds(t, pin, 120, 0, -3, 140, 20, 10)
	steps := doc.Recipe().Steps
	require.Equal(t, []decad.StepRef{above.Origin().Step}, steps[len(steps)-1].Inputs)

	// Both sides through-all: [−6, 10], one stop body per side, along side
	// first. A fresh document, so the earlier pin is not itself a stop body.
	doc2 := decad.New()
	above2, err := doc2.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	below2, err := doc2.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(6), Dir: decad.Against})
	require.NoError(t, err)
	pin2, err := doc2.Extrude(s, pinProf, decad.TwoSided{
		One: decad.ThroughAllSide{},
		Two: decad.ThroughAllSide{},
	})
	require.NoError(t, err)
	requireBounds(t, pin2, 120, 0, -6, 140, 20, 10)
	steps = doc2.Recipe().Steps
	last := steps[len(steps)-1]
	require.Equal(t, []decad.StepRef{above2.Origin().Step, below2.Origin().Step}, last.Inputs)

	// The recorded two-sided extent round-trips.
	buf, err := json.Marshal(last)
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, last, got)
}

func TestExtrudeToFace(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// Stop at the plate's far cap: the sense comes from the target face, so
	// the extent carries no Direction — the sweep is exactly [0, 10].
	q := capEndFace(plate)
	pin, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: q})
	require.NoError(t, err)
	requireVolume(t, pin, 4000)
	requireBounds(t, pin, 120, 0, 0, 140, 20, 10)
	requireManifold(t, pin)
	require.Contains(t, doc.Bodies(), plate, `a stop body is depended on, never retired`)

	// The step records the extent with the body as its producing StepRef and
	// a deep-copied selector, and depends on the body (core §6.2).
	steps := doc.Recipe().Steps
	require.Len(t, steps, 2)
	require.Equal(t, []decad.StepRef{plate.Origin().Step}, steps[1].Inputs)
	tf, ok := steps[1].Extent.(decad.ToFace)
	require.True(t, ok, `the recorded extent stays a ToFace, never the resolved interval`)
	require.Equal(t, plate.Origin().Step, tf.Body)
	require.NotSame(t, q, tf.Face.(*decad.FaceQuery), `the recorded query never aliases the caller's`)

	// The recorded step round-trips through the wire codec.
	buf, err := json.Marshal(steps[1])
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, steps[1], got)

	// A positive offset overshoots the face, a negative one stops short of
	// it (core §8.1).
	over, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capEndFace(plate), Offset: units.Millimeters(2)})
	require.NoError(t, err)
	requireBounds(t, over, 120, 0, 0, 140, 20, 12)
	short, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capEndFace(plate), Offset: units.Millimeters(-3)})
	require.NoError(t, err)
	requireBounds(t, short, 120, 0, 0, 140, 20, 7)
}

func TestExtrudeToFaceAgainst(t *testing.T) {
	// The target face below the sketch plane pulls the sweep Against: the
	// sense comes from the face, not from a Direction.
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	below, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(6), Dir: decad.Against})
	require.NoError(t, err)

	pin, err := doc.Extrude(s, pinProf, decad.ToFace{Body: below, Face: capStartFace(below)})
	require.NoError(t, err)
	requireVolume(t, pin, 400*6)
	requireBounds(t, pin, 120, 0, -6, 140, 20, 0)
}

func TestExtrudeToFaceSides(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	above, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	below, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(6), Dir: decad.Against})
	require.NoError(t, err)

	// Each side stops at its own face: [−6, 10], named-extent refs recorded
	// in extent order (core §6.2).
	pin, err := doc.Extrude(s, pinProf, decad.TwoSided{
		One: decad.ToFace{Body: above, Face: capEndFace(above)},
		Two: decad.ToFace{Body: below, Face: capStartFace(below)},
	})
	require.NoError(t, err)
	requireVolume(t, pin, 400*16)
	requireBounds(t, pin, 120, 0, -6, 140, 20, 10)
	steps := doc.Recipe().Steps
	last := steps[len(steps)-1]
	require.Equal(t, []decad.StepRef{above.Origin().Step, below.Origin().Step}, last.Inputs)

	// The recorded sides carry StepRefs and round-trip.
	ts, ok := last.Extent.(decad.TwoSided)
	require.True(t, ok)
	require.Equal(t, above.Origin().Step, ts.One.(decad.ToFace).Body)
	require.Equal(t, below.Origin().Step, ts.Two.(decad.ToFace).Body)
	buf, err := json.Marshal(last)
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, last, got)

	// A side's face must lie on the side's own sense: the against side
	// cannot stop at a face above the sketch plane.
	_, err = doc.Extrude(s, pinProf, decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(1)},
		Two: decad.ToFace{Body: above, Face: capEndFace(above)},
	})
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestExtrudeToFaceGates(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	liveBodies := len(doc.Bodies())
	liveSteps := len(doc.Recipe().Steps)

	t.Run("NilBody", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Face: capEndFace(plate)})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("StepRefBody", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: decad.StepRef(0), Face: capEndFace(plate)})
		require.ErrorIs(t, err, decad.ErrUnresolvedBody)
	})
	t.Run("ForeignBody", func(t *testing.T) {
		other := decad.New()
		_, err := other.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capEndFace(plate)})
		require.ErrorIs(t, err, decad.ErrForeignBody)
	})
	t.Run("RetiredBody", func(t *testing.T) {
		s2, plateProf2, pinProf2 := plateAndPin(t)
		doc2 := decad.New()
		old, err := doc2.Extrude(s2, plateProf2, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		move, err := r3.Translation(r3.NewVec(0, 0, 1))
		require.NoError(t, err)
		_, err = old.Placed(move)
		require.NoError(t, err)
		_, err = doc2.Extrude(s2, pinProf2, decad.ToFace{Body: old, Face: capEndFace(old)})
		require.ErrorIs(t, err, decad.ErrRetiredBody)
	})
	t.Run("NilFace", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("TypedNilFace", func(t *testing.T) {
		var q *decad.FaceQuery
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: q})
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrUnsupported, `malformed input must not read as staged resolution`)
	})
	t.Run("ZeroMatchesIsErrCardinality", func(t *testing.T) {
		// The implicit exactly-one takes precedence over ErrNoMatch even for
		// a selector asserting no cardinality of its own (core §12).
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: decad.Faces(decad.Cylindrical())})
		require.ErrorIs(t, err, decad.ErrCardinality)
		require.NotErrorIs(t, err, decad.ErrNoMatch)
	})
	t.Run("ManyMatchesIsErrCardinality", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: decad.Faces(decad.Planar())})
		require.ErrorIs(t, err, decad.ErrCardinality)
	})
	t.Run("OwnAssertionMiss", func(t *testing.T) {
		// The query's own cardinality assertion fails first: a plate has six
		// planar faces, not three.
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: decad.Faces(decad.Planar()).Exactly(3)})
		require.ErrorIs(t, err, decad.ErrCardinality)
	})
	t.Run("NonPlanarStopFace", func(t *testing.T) {
		holed := holePlateBody(t)
		hq := decad.Faces(decad.Cylindrical())
		_, err := holed.Document().Extrude(s, pinProf, decad.ToFace{Body: holed, Face: hq})
		require.ErrorIs(t, err, decad.ErrUnsupported, `a curved stop face is staged, loudly`)
	})
	t.Run("NotPerpendicularStopFace", func(t *testing.T) {
		// A side wall's plane is parallel to the sweep, not perpendicular:
		// its normal never aligns with the sweep direction. The triangle
		// prism has exactly one x-normal wall.
		doc2 := decad.New()
		prism := trianglePrismHost(t, doc2)
		q := decad.Faces(decad.Planar(), decad.NormalTo(r3.NewVec(1, 0, 0)))
		_, err := doc2.Extrude(s, pinProf, decad.ToFace{Body: prism, Face: q})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("CoplanarStopFace", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capStartFace(plate)})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("OffsetWrongKind", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capEndFace(plate), Offset: units.Degrees(5)})
		require.ErrorIs(t, err, decad.ErrUnitKind)
	})
	t.Run("OffsetNotFinite", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capEndFace(plate), Offset: units.Millimeters(math.Inf(1))})
		require.ErrorIs(t, err, decad.ErrNotFinite)
	})
	t.Run("OffsetPullsStopBehindPlane", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capEndFace(plate), Offset: units.Millimeters(-15)})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("NilExtentPointer", func(t *testing.T) {
		_, err := doc.Extrude(s, pinProf, (*decad.ToFace)(nil))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	// Every rejection left the recipe and the document untouched.
	require.Len(t, doc.Bodies(), liveBodies)
	require.Len(t, doc.Recipe().Steps, liveSteps)
}

func TestRevolveToFaceAngular(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	host, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	// Stop at the host's end cap — the radial plane at 90°. The sense comes
	// from the target face: the nearer way around is Along, so the sweep is
	// exactly [0, π/2].
	body, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: host, Face: capEndFace(host)})
	require.NoError(t, err)
	requireVolume(t, body, 500*math.Pi)
	requireBounds(t, body, 0, 0, 0, 10, 15, 15)
	require.Contains(t, doc.Bodies(), host, `a stop body is depended on, never retired`)

	// The step depends on the host, and the recorded extent carries the
	// StepRef (core §6.2); it round-trips through the wire codec.
	steps := doc.Recipe().Steps
	require.Len(t, steps, 2)
	require.Equal(t, []decad.StepRef{host.Origin().Step}, steps[1].Inputs)
	tfa, ok := steps[1].Angular.(decad.ToFaceAngular)
	require.True(t, ok, `the recorded extent stays a ToFaceAngular, never the resolved interval`)
	require.Equal(t, host.Origin().Step, tfa.Body)
	buf, err := json.Marshal(steps[1])
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, steps[1], got)

	// The start cap lies in the profile's own half-plane: a zero sweep.
	_, err = doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: host, Face: capStartFace(host)})
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestRevolveToFaceAngularNearerWay(t *testing.T) {
	// The target face supplies the sense: a cap at 270° Along is nearer the
	// other way, so a standalone ToFaceAngular sweeps Against to −90°.
	s, p := annularSketch(t)
	doc := decad.New()
	host, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(270), Dir: decad.Along})
	require.NoError(t, err)

	body, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: host, Face: capEndFace(host)})
	require.NoError(t, err)
	requireVolume(t, body, 500*math.Pi)
	requireBounds(t, body, 0, 0, -15, 10, 15, 0)
}

func TestRevolveToFaceAngularSides(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	host, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	// As the along side of a two-sided extent: [−π/4, π/2].
	body, err := doc.Revolve(s, p, uAxis, decad.TwoSidedAngle{
		One: decad.ToFaceAngular{Body: host, Face: capEndFace(host)},
		Two: decad.AngleSide{A: units.Degrees(45)},
	})
	require.NoError(t, err)
	requireVolume(t, body, 750*math.Pi)
	steps := doc.Recipe().Steps
	require.Equal(t, []decad.StepRef{host.Origin().Step}, steps[len(steps)-1].Inputs)

	// As the against side the side supplies the sense, so the same face is
	// reached the long way around: [−3π/2, π/4].
	long, err := doc.Revolve(s, p, uAxis, decad.TwoSidedAngle{
		One: decad.AngleSide{A: units.Degrees(45)},
		Two: decad.ToFaceAngular{Body: host, Face: capEndFace(host)},
	})
	require.NoError(t, err)
	requireVolume(t, long, 1750*math.Pi)

	// The recorded sides carry StepRefs and round-trip.
	steps = doc.Recipe().Steps
	last := steps[len(steps)-1]
	tsa, ok := last.Angular.(decad.TwoSidedAngle)
	require.True(t, ok)
	require.Equal(t, host.Origin().Step, tsa.Two.(decad.ToFaceAngular).Body)
	buf, err := json.Marshal(last)
	require.NoError(t, err)
	var got decad.Step
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, last, got)
}

func TestRevolveToFaceAngularHalfDiskCap(t *testing.T) {
	// A partial sphere's cap is a half-disk: its straight edge lies ON the
	// axis, so the face's off-axis material is witnessed only by its arc —
	// still an exact, closed-form stop.
	ss, sp := semicircleSketch(t)
	doc := decad.New()
	host, err := doc.Revolve(ss, sp, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)

	s, p := annularSketch(t)
	body, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: host, Face: capEndFace(host)})
	require.NoError(t, err)
	requireVolume(t, body, 500*math.Pi)
}

func TestRevolveToFaceAngularGates(t *testing.T) {
	s, p := annularSketch(t)
	doc := decad.New()
	host, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	es, ep := plateSketch(t)
	plate, err := doc.Extrude(es, ep, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	prism := trianglePrismHost(t, doc)
	liveBodies := len(doc.Bodies())
	liveSteps := len(doc.Recipe().Steps)

	t.Run("NonPlanarStopFace", func(t *testing.T) {
		cs, cp := solidSketch(t)
		doc2 := decad.New()
		cyl, err := doc2.Revolve(cs, cp, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		_, err = doc2.Revolve(cs, cp, uAxis, decad.ToFaceAngular{Body: cyl, Face: decad.Faces(decad.Cylindrical())})
		require.ErrorIs(t, err, decad.ErrUnsupported, `a curved stop face is staged, loudly`)
	})
	t.Run("PlaneNormalAlongAxis", func(t *testing.T) {
		// The prism's x=0 wall: its plane's normal is the revolve axis
		// itself, so the rotating half-plane never coincides with it.
		q := decad.Faces(decad.Planar(), decad.NormalTo(r3.NewVec(1, 0, 0)))
		_, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: prism, Face: q})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("PlaneMissesAxis", func(t *testing.T) {
		// The plate's far cap plane (z=10) is parallel to the axis but never
		// contains it.
		_, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: plate, Face: capEndFace(plate)})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("FaceSpansBothSides", func(t *testing.T) {
		// A cap straddling the axis puts material in both half-planes: it
		// names two stops at once.
		w := sketch.NewWorld()
		ws, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := ws.CreateRectangle(0, -30, 100, 30)
		ws.Fix(rect.A)
		_, err = ws.Solve(t.Context())
		require.NoError(t, err)
		straddle, err := doc.Extrude(ws, ws.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		_, err = doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: straddle, Face: capStartFace(straddle)})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("StepRefBody", func(t *testing.T) {
		_, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: decad.StepRef(0), Face: capEndFace(host)})
		require.ErrorIs(t, err, decad.ErrUnresolvedBody)
	})
	t.Run("NilBody", func(t *testing.T) {
		_, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Face: capEndFace(host)})
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("TypedNilFace", func(t *testing.T) {
		var q *decad.FaceQuery
		_, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: host, Face: q})
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.NotErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("ZeroMatchesIsErrCardinality", func(t *testing.T) {
		_, err := doc.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: plate, Face: decad.Faces(decad.Cylindrical())})
		require.ErrorIs(t, err, decad.ErrCardinality)
		require.NotErrorIs(t, err, decad.ErrNoMatch)
	})
	t.Run("ForeignBody", func(t *testing.T) {
		other := decad.New()
		_, err := other.Revolve(s, p, uAxis, decad.ToFaceAngular{Body: host, Face: capEndFace(host)})
		require.ErrorIs(t, err, decad.ErrForeignBody)
	})

	// Every rejection above left doc untouched (the subtests that build
	// their own hosts append their own steps to doc first, so the counts are
	// taken against the final successful state).
	require.Len(t, doc.Bodies(), liveBodies+1)
	require.Len(t, doc.Recipe().Steps, liveSteps+1)
}

func TestBodyStopExtentKeying(t *testing.T) {
	// The one-of contract of core §6.2, on both wire directions, for the new
	// variants: a linear ToFace is keyed to extrude, an angular ToFaceAngular
	// to revolve.
	linear := decad.ToFace{Body: decad.StepRef(0), Face: decad.Faces(decad.Planar()), Offset: units.Millimeters(0)}
	angular := decad.ToFaceAngular{Body: decad.StepRef(0), Face: decad.Faces(decad.Planar())}

	wrongRevolve := validCodecStep(decad.OpRevolve)
	wrongRevolve.Extent = linear
	_, err := json.Marshal(wrongRevolve)
	require.Error(t, err, `a linear to-face under revolve does not encode`)
	wrongExtrude := validCodecStep(decad.OpExtrude)
	wrongExtrude.Angular = angular
	_, err = json.Marshal(wrongExtrude)
	require.Error(t, err, `an angular to-face under extrude does not encode`)

	// Decode direction: swap the ops in otherwise-valid wire steps.
	extrude := validCodecStep(decad.OpExtrude)
	extrude.Extent = linear
	extrudeStep, err := json.Marshal(extrude)
	require.NoError(t, err)
	var tampered decad.Step
	swapped := strings.Replace(string(extrudeStep), `"op":"extrude"`, `"op":"revolve"`, 1)
	require.Error(t, json.Unmarshal([]byte(swapped), &tampered))

	revolve := validCodecStep(decad.OpRevolve)
	revolve.Angular = angular
	revolveStep, err := json.Marshal(revolve)
	require.NoError(t, err)
	swapped = strings.Replace(string(revolveStep), `"op":"revolve"`, `"op":"extrude"`, 1)
	require.Error(t, json.Unmarshal([]byte(swapped), &tampered))
}

func TestBodyStopCodecGates(t *testing.T) {
	t.Run("LiveBodyNeverEncodes", func(t *testing.T) {
		s, plateProf, _ := plateAndPin(t)
		doc := decad.New()
		plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		step := validCodecStep(decad.OpExtrude)
		step.Extent = decad.ToFace{Body: plate, Face: capEndFace(plate)}
		_, err = json.Marshal(step)
		require.Error(t, err, `a recorded step holds a StepRef, never a live body`)
	})
	t.Run("MissingFields", func(t *testing.T) {
		for _, raw := range []string{
			`{"kind":"to_face"}`,
			`{"kind":"to_face","body":0,"face":{"kind":"faces","preds":[]}}`,
			`{"kind":"to_face","body":0,"offset":"0 mm"}`,
		} {
			var step decad.Step
			require.Error(t, json.Unmarshal([]byte(`{"op":"extrude","extent":`+raw+`}`), &step), raw)
		}
		var step decad.Step
		require.Error(t, json.Unmarshal([]byte(`{"op":"revolve","angular":{"kind":"to_face_angular","body":0}}`), &step))
	})
	t.Run("PointerFormsNormalize", func(t *testing.T) {
		tf := &decad.ToFace{Body: decad.StepRef(0), Face: decad.Faces(decad.Planar()), Offset: units.Millimeters(1)}
		extrude := validCodecStep(decad.OpExtrude)
		extrude.Extent = tf
		buf, err := json.Marshal(extrude)
		require.NoError(t, err)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got))
		require.Equal(t, *tf, got.Extent, `the pointer form records the value it names`)

		tfa := &decad.ToFaceAngular{Body: decad.StepRef(0), Face: decad.Faces(decad.Planar())}
		revolve := validCodecStep(decad.OpRevolve)
		revolve.Angular = tfa
		buf, err = json.Marshal(revolve)
		require.NoError(t, err)
		require.NoError(t, json.Unmarshal(buf, &got))
		require.Equal(t, *tfa, got.Angular)
	})
	t.Run("NilPointersAreBranchable", func(t *testing.T) {
		extrude := validCodecStep(decad.OpExtrude)
		extrude.Extent = (*decad.ToFace)(nil)
		_, err := json.Marshal(extrude)
		require.Error(t, err)
		revolve := validCodecStep(decad.OpRevolve)
		revolve.Angular = (*decad.ToFaceAngular)(nil)
		_, err = json.Marshal(revolve)
		require.Error(t, err)
	})
	t.Run("ZeroOffsetNormalizes", func(t *testing.T) {
		// A zero-value Offset means no displacement; the wire always carries
		// an explicit length.
		step := validCodecStep(decad.OpExtrude)
		step.Extent = decad.ToFace{Body: decad.StepRef(0), Face: decad.Faces(decad.Planar())}
		buf, err := json.Marshal(step)
		require.NoError(t, err)
		var got decad.Step
		require.NoError(t, json.Unmarshal(buf, &got))
		require.Equal(t, units.Millimeters(0), got.Extent.(decad.ToFace).Offset)
	})
}

func TestBodyStopRecipeRoundTrip(t *testing.T) {
	// A whole recipe holding every stop shape — through-all, to-face with an
	// offset, two-sided mixes, and the angular stop — survives the wire.
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	_, err = doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(6), Dir: decad.Against})
	require.NoError(t, err)
	_, err = doc.Extrude(s, pinProf, decad.ThroughAll{Dir: decad.Along})
	require.NoError(t, err)
	_, err = doc.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capEndFace(plate), Offset: units.Millimeters(-2)})
	require.NoError(t, err)
	_, err = doc.Extrude(s, pinProf, decad.TwoSided{
		One: decad.ToFace{Body: plate, Face: capEndFace(plate)},
		Two: decad.ThroughAllSide{},
	})
	require.NoError(t, err)

	rs, rp := annularSketch(t)
	host, err := doc.Revolve(rs, rp, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	_, err = doc.Revolve(rs, rp, uAxis, decad.ToFaceAngular{Body: host, Face: capEndFace(host)})
	require.NoError(t, err)

	recipe := doc.Recipe()
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got)
}

// TestExtrudeThroughAllCupStop proves docs/modify-design.md Table D, D5: a
// ThroughAll stop reads a live cup's OUTER prism extent via cupPayload's own
// extentAlong (the cavity being interior), rather than refusing with
// ErrUnsupported because the cup lacks a directional extent.
func TestExtrudeThroughAllCupStop(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	box, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)

	// Hollow the plate into a cup (one cap removed): the box is retired, so the
	// cup is the only live body the pin's sweep can meet. The outer prism still
	// spans z ∈ [0, 20], so the through-all stop must resolve to 20.
	cup, err := box.Shell(topCap(box), units.Millimeters(5))
	require.NoError(t, err)
	require.NotContains(t, doc.Bodies(), box)

	pin, err := doc.Extrude(s, pinProf, decad.ThroughAll{Dir: decad.Along})
	require.NoError(t, err)
	require.NotErrorIs(t, err, decad.ErrUnsupported)

	// The sweep read the cup's outer extent (20): the 20×20 pin swept [0, 20]
	// is 8000 mm³, bounded z ∈ [0, 20]. The cavity did not lower the stop.
	requireVolume(t, pin, 400*20)
	requireBounds(t, pin, 120, 0, 0, 140, 20, 20)
	requireManifold(t, pin)

	// The cup is a recorded dependency of the stop step, not an operand: it
	// stays live and its StepRef is the step's Inputs (core §6.2).
	require.Contains(t, doc.Bodies(), cup)
	steps := doc.Recipe().Steps
	require.Equal(t, []decad.StepRef{cup.Origin().Step}, steps[len(steps)-1].Inputs)
}

// TestExtrudeThroughAllSideCupStop is the ThroughAllSide sibling: a two-sided
// sweep whose Along side runs through a live cup reads the same outer extent.
func TestExtrudeThroughAllSideCupStop(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	box, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	cup, err := box.Shell(topCap(box), units.Millimeters(5))
	require.NoError(t, err)

	pin, err := doc.Extrude(s, pinProf, decad.TwoSided{
		One: decad.ThroughAllSide{},
		Two: decad.DistanceSide{D: units.Millimeters(3)},
	})
	require.NoError(t, err)
	require.NotErrorIs(t, err, decad.ErrUnsupported)

	// The Along side (One) stops at the cup's outer extent (20), the Against
	// side (Two) at 3: the pin spans z ∈ [-3, 20], a 20×20 footprint →
	// 23·400 = 9200 mm³.
	requireVolume(t, pin, 400*23)
	requireBounds(t, pin, 120, 0, -3, 140, 20, 20)
	require.Contains(t, doc.Bodies(), cup)
}

// sweepEdges resolves an XY-plane prism's four lateral edges — the ones that
// span the sweep, and so the ones a computed level reaches.
func sweepEdges(t *testing.T, b *decad.Body) []*decad.Edge {
	t.Helper()
	edges, err := verticalEdges().SelectEdges(b)
	require.NoError(t, err)
	return edges
}

// TestExtrudeToFaceComputedLevelIsBounded pins the rule a computed sweep level
// obeys: the level a ToFace stop resolves is arithmetic over another body's
// face, so every reading built from it publishes that arithmetic's own proven
// displacement instead of claiming the level it denotes.
//
// The fixture is the one that makes the displacement visible at millimetre
// scale: a plate swept 1e12 mm, stopped 0.001 mm short of its far cap. The
// denoted level is 999999999999.999 and the float sum lands on
// 999999999999.9990234375, 2.34375e-05 mm away.
func TestExtrudeToFaceComputedLevelIsBounded(t *testing.T) {
	const (
		plateHeight = 1e12
		shortBy     = 0.001
		// The stop the float sum holds, and its distance from the level the
		// extent denotes (1e12 − 0.001).
		heldStop = 999999999999.9990234375
		rounding = 2.34375e-05
	)
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(plateHeight), Dir: decad.Along})
	require.NoError(t, err)

	pin, err := doc.Extrude(s, pinProf, decad.ToFace{
		Body:   plate,
		Face:   capEndFace(plate),
		Offset: units.Millimeters(-shortBy),
	})
	require.NoError(t, err)

	// The stop is the float sum, unchanged: this repair bounds the level, it
	// does not move it.
	top := 0
	for _, v := range pin.Vertices() {
		p := v.Position()
		if p.Value.Z == 0 {
			// The sketch plane is the end the caller did not compute: it stays
			// exact, which is what proves the bound is per-end and not a blanket
			// stamp on the body.
			require.Equal(t, decad.Exact, p.Exactness)
			require.Zero(t, p.Bound.Mag())
			continue
		}
		require.Equal(t, heldStop, p.Value.Z)
		require.Equal(t, decad.Approximate, p.Exactness, `a vertex at a computed level is not exact`)
		bound, err := p.Bound.In(units.Millimeter)
		require.NoError(t, err)
		require.InDelta(t, rounding, bound, 1e-12, `the bound covers the level's own rounding`)
		top++
	}
	require.Equal(t, 4, top, `the 20×20 pin has four vertices at the stop level`)

	verticals := sweepEdges(t, pin)
	require.Len(t, verticals, 4)
	for _, e := range verticals {
		length, err := e.Length()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, length.Exactness, `a vertical edge spanning a computed level is not exact`)
		bound, err := length.Bound.In(units.Millimeter)
		require.NoError(t, err)
		require.GreaterOrEqual(t, bound, rounding, `the height carries the level's rounding`)
	}

	// The measurements the same height feeds report it too.
	bounds, err := pin.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, bounds.Exactness)
	require.Positive(t, bounds.Bound.Mag())
	area, err := pin.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Mag())
}

// TestExtrudeStatedLevelStaysExact is the other half of the same rule: a level
// the caller stated denotes itself, so nothing about it is approximate. It
// covers the plain Distance sweep and the ToFace stop whose arithmetic happens
// to be exact alike — the bound is MEASURED per level, never charged to every
// stop.
func TestExtrudeStatedLevelStaysExact(t *testing.T) {
	s, plateProf, pinProf := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProf, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	toFace, err := doc.Extrude(s, pinProf, decad.ToFace{
		Body:   plate,
		Face:   capEndFace(plate),
		Offset: units.Millimeters(2),
	})
	require.NoError(t, err)

	for name, body := range map[string]*decad.Body{"distance": plate, "toFace": toFace} {
		t.Run(name, func(t *testing.T) {
			for _, v := range body.Vertices() {
				p := v.Position()
				require.Equal(t, decad.Exact, p.Exactness, `a vertex at a stated level is exact`)
				require.Zero(t, p.Bound.Mag())
			}
			verticals := sweepEdges(t, body)
			require.Len(t, verticals, 4)
			for _, e := range verticals {
				length, err := e.Length()
				require.NoError(t, err)
				require.Equal(t, decad.Exact, length.Exactness)
				require.Zero(t, length.Bound.Mag())
			}
			vol, err := body.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, vol.Exactness)
			bounds, err := body.Bounds()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, bounds.Exactness)
		})
	}
}
