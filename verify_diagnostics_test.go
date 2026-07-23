package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// worstDiagnosticStatus is verification §1.1's aggregate: the worst
// Diagnostic.Status in the slice, Sound when it is empty.
func worstDiagnosticStatus(diags []decad.Diagnostic) decad.Status {
	worst := decad.Sound
	for _, d := range diags {
		if d.Status > worst {
			worst = d.Status
		}
	}
	return worst
}

// requireDiagnosticInvariants asserts verification §1.1's contract holds for a
// whole report: the slice is empty EXACTLY when Sound, Status is the worst
// diagnostic status, and every entry keys exactly one Observed* form off its
// Reading with a body-xor-pair owner.
func requireDiagnosticInvariants(t *testing.T, report *decad.Report) {
	t.Helper()

	if report.Status == decad.Sound {
		require.Empty(t, report.Diagnostics, `a Sound report carries no diagnostics`)
	} else {
		require.NotEmpty(t, report.Diagnostics, `a non-Sound report itemizes its reasons`)
	}
	require.Equal(t, report.Status, worstDiagnosticStatus(report.Diagnostics),
		`Status is the worst diagnostic status`)

	for _, d := range report.Diagnostics {
		// A diagnostic concerns a body or a pair, never both and never neither.
		require.True(t, (d.Body != nil) != (d.Pair != nil),
			`%s names exactly one of Body / Pair`, d.Code)

		// Exactly one Observed* form is set, keyed by Reading (invariant #2).
		switch d.Reading {
		case decad.ReadingNone:
			require.Nil(t, d.Observed, `%s: ReadingNone carries no scalar`, d.Code)
			require.Nil(t, d.ObservedVec, `%s: ReadingNone carries no vector`, d.Code)
			require.Nil(t, d.ObservedBox, `%s: ReadingNone carries no box`, d.Code)
		case decad.ReadingCentroid:
			require.NotNil(t, d.ObservedVec, `%s: a centroid rides ObservedVec`, d.Code)
			require.Nil(t, d.Observed)
			require.Nil(t, d.ObservedBox)
		case decad.ReadingBounds:
			require.NotNil(t, d.ObservedBox, `%s: a bounds box rides ObservedBox`, d.Code)
			require.Nil(t, d.Observed)
			require.Nil(t, d.ObservedVec)
		default:
			require.NotNil(t, d.Observed, `%s: a scalar rides Observed`, d.Code)
			require.Nil(t, d.ObservedVec)
			require.Nil(t, d.ObservedBox)
		}

		require.NotEmpty(t, d.Message, `%s carries a human-readable message`, d.Code)
		require.NotEqual(t, d.Code.String(), d.Message, `the message is never the branch key`)
	}
}

func TestDiagnosticReadingKindTokens(t *testing.T) {
	// The pinned lower-snake tokens (verification §1.1) — the identity a caller
	// branches on, never the iota value.
	require.Equal(t, "none", decad.ReadingNone.String())
	require.Equal(t, "area", decad.ReadingArea.String())
	require.Equal(t, "bounds", decad.ReadingBounds.String())
	require.Equal(t, "volume", decad.ReadingVolume.String())
	require.Equal(t, "centroid", decad.ReadingCentroid.String())
	require.Equal(t, "wall", decad.ReadingWall.String())
	require.Equal(t, "min_radius", decad.ReadingMinRadius.String())
	require.Equal(t, "overlap_volume", decad.ReadingOverlapVolume.String())
	require.Equal(t, "gap", decad.ReadingGap.String())
	// An out-of-range value renders reading(<n>), never a panic.
	require.Equal(t, "reading(42)", decad.ReadingKind(42).String())
}

func TestDiagnosticCodeTokens(t *testing.T) {
	require.Equal(t, "measurement_beyond_tolerance", decad.DiagMeasurementBeyondTolerance.String())
	require.Equal(t, "undecided_validity", decad.DiagUndecidedValidity.String())
	require.Equal(t, "invalid_body", decad.DiagInvalidBody.String())
	require.Equal(t, "wall_too_thin", decad.DiagWallTooThin.String())
	require.Equal(t, "undercut", decad.DiagUndercut.String())
	require.Equal(t, "undecided_wall", decad.DiagUndecidedWall.String())
	require.Equal(t, "undecided_undercut", decad.DiagUndecidedUndercut.String())
	require.Equal(t, "undecided_min_radius", decad.DiagUndecidedMinRadius.String())
	require.Equal(t, "interference", decad.DiagInterference.String())
	require.Equal(t, "undecided_pair", decad.DiagUndecidedPair.String())
	require.Equal(t, "unsupported_pair", decad.DiagUnsupportedPair.String())
	require.Equal(t, "undecided_clearance", decad.DiagUndecidedClearance.String())
	require.Equal(t, "undecided_interference", decad.DiagUndecidedInterference.String())
	require.Equal(t, "unsupported_pair_payload", decad.DiagUnsupportedPairPayload.String())
	require.Equal(t, "unsupported_pair_contact", decad.DiagUnsupportedPairContact.String())
	require.Equal(t, "unsupported_pair_pipeline", decad.DiagUnsupportedPairPipeline.String())
	require.Equal(t, "unsupported_survey_payload", decad.DiagUnsupportedSurveyPayload.String())
	// An out-of-range value renders diagnostic(<n>), never a panic.
	require.Equal(t, "diagnostic(99)", decad.DiagnosticCode(99).String())
}

func TestVerifyDiagnosticsEmptyWhenSound(t *testing.T) {
	doc, _ := extrudePlate(t)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics, `a Sound report is empty exactly`)
	requireDiagnosticInvariants(t, report)
}

// findDiagnostic returns the first diagnostic with the given code.
func findDiagnostic(diags []decad.Diagnostic, code decad.DiagnosticCode) (decad.Diagnostic, bool) {
	for _, d := range diags {
		if d.Code == code {
			return d, true
		}
	}
	return decad.Diagnostic{}, false
}

func TestVerifyDiagnosticsWallTooThin(t *testing.T) {
	// The 10×10×0.5 mm plate against a 1 mm tool: a proven-thin wall.
	doc := rectPrism(t, 10, 10, 0.5)
	tool := units.Millimeters(1)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(tool))
	require.NoError(t, err)

	require.Equal(t, decad.Violating, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagWallTooThin)
	require.True(t, ok, `a thin wall emits DiagWallTooThin`)
	require.Equal(t, decad.Violating, d.Status)
	require.Equal(t, decad.ReadingWall, d.Reading)
	require.Same(t, report.Bodies[0].Body, d.Body)
	require.Nil(t, d.Pair)
	require.NotNil(t, d.Observed, `the wall reading rides Observed`)
	require.InDelta(t, 0.5, d.Observed.Value.Base(), 1e-9)
	require.NotNil(t, d.Required, `the tool rides Required`)
	require.True(t, d.Required.Equal(tool, 1e-9), `Required is the tool`)
}

func TestVerifyDiagnosticsUndercut(t *testing.T) {
	// A prism under a tilted pull: two proven undercut faces, one DiagUndercut.
	doc := rectPrism(t, 100, 60, 10)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
	require.NoError(t, err)

	require.Equal(t, decad.Violating, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUndercut)
	require.True(t, ok, `a proven undercut emits DiagUndercut`)
	require.Equal(t, decad.Violating, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading, `an undercut is a predicate, not a scalar`)
	require.Same(t, report.Bodies[0].Body, d.Body)
	require.NotEmpty(t, report.Bodies[0].Undercuts, `the faces are listed on the BodyReport`)
}

func TestVerifyDiagnosticsUnsupportedFacetedSurveys(t *testing.T) {
	doc, body := allPlanarBoolean(t, 1)
	report, err := doc.Verify(t.Context(),
		decad.WithMinWallThickness(units.Millimeters(1)),
		decad.WithPullDirection(r3.NewVec(0, 0, 1)),
		decad.WithMinRadius())
	require.NoError(t, err)

	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)
	require.Len(t, report.Diagnostics, 3, `each staged faceted survey names its own refusal`)
	for _, diagnostic := range report.Diagnostics {
		require.Equal(t, decad.DiagUnsupportedSurveyPayload, diagnostic.Code)
		require.Equal(t, decad.Suspect, diagnostic.Status)
		require.Equal(t, decad.ReadingNone, diagnostic.Reading)
		require.Same(t, body, diagnostic.Body)
		require.Contains(t, diagnostic.Message, "facetedPayload")
		require.Contains(t, diagnostic.Message, "use an analytic body")
	}
	require.Contains(t, report.Diagnostics[0].Message, "wall survey")
	require.Contains(t, report.Diagnostics[1].Message, "pull survey")
	require.Contains(t, report.Diagnostics[2].Message, "concave-radius survey")
}

func TestVerifyDiagnosticsInterference(t *testing.T) {
	// Two coincident analytic plates overlap by an exact set-identity volume.
	doc, _ := extrudePlate(t)
	s, p := plateSketch(t)
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Interfering, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagInterference)
	require.True(t, ok, `a proven overlap emits DiagInterference`)
	require.Equal(t, decad.Interfering, d.Status)
	require.Equal(t, decad.ReadingOverlapVolume, d.Reading)
	require.Nil(t, d.Body)
	require.NotNil(t, d.Pair, `an interference names its pair`)
	require.NotNil(t, d.Observed, `the overlap volume rides Observed`)
	require.InDelta(t, 60000.0, d.Observed.Value.Base(), 1e-6)
	require.Len(t, report.Interferences, 1, `the diagnostic mirrors the Interference row`)
}

func TestVerifyDiagnosticsUnsupportedPairStagedContact(t *testing.T) {
	// Two 10×10×10 boxes, the second translated to (0,5,5): they overlap with
	// positive volume while sharing coplanar side faces. The read-only intersect
	// stages the face-on-face contact (booleanExpectedContact). Per verification
	// §1.1 a staged boolean contact is a DiagUnsupportedPairContact, not a
	// payload, pipeline, or undecided-partition diagnostic; the Status stays
	// Suspect.
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	p := s.Profiles()[0]

	doc := decad.New()
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	box2, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	shift, err := r3.Translation(r3.NewVec(0, 5, 5))
	require.NoError(t, err)
	_, err = box2.Placed(shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
	require.True(t, ok, `a staged boolean contact emits its cause-specific code`)
	require.Equal(t, decad.Suspect, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading, `an unsupported pair names no reading`)
	require.Nil(t, d.Body)
	require.NotNil(t, d.Pair, `an unsupported pair names its pair`)
	require.Equal(t,
		`the pair reaches a contact or near-contact that the read-only intersection cannot classify; adjust the geometry to create clear separation or deeper overlap, or wait for contact support`,
		d.Message,
		`the contact diagnostic names contact and near-contact cases and gives accurate corrective guidance`)

	_, undecided := findDiagnostic(report.Diagnostics, decad.DiagUndecidedPair)
	require.False(t, undecided, `a staged contact is not an undecided partition`)
	legacy, broad := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPair)
	require.True(t, broad, `a staged contact preserves the broad compatibility code`)
	require.Equal(t, decad.Suspect, legacy.Status)
	require.Equal(t, decad.ReadingNone, legacy.Reading)
	require.NotNil(t, legacy.Pair)
	_, payload := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPayload)
	require.False(t, payload, `a contact refusal is not a payload capability limit`)
	_, pipeline := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPipeline)
	require.False(t, pipeline, `a contact refusal is not an in-pipeline reach`)
}

func TestVerifyDiagnosticsProximityRefusalIsContact(t *testing.T) {
	// The cylinder's analytic surface overlaps the plate's x = 20 face by only
	// 0.0005 mm. The pre-tessellation proximity gate refuses the undecidable
	// chord result, so Verify must retain the contact cause for its diagnostic.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 20, 20, 8)
	translated(t, diskBody(t, doc, 29.9995, 10, 10), 0, 0, -6)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
	require.True(t, ok, `a proximity refusal emits the contact diagnostic`)
	require.Equal(t, decad.Suspect, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading)
	require.Nil(t, d.Body)
	require.NotNil(t, d.Pair)
	require.Equal(t,
		`the pair reaches a contact or near-contact that the read-only intersection cannot classify; adjust the geometry to create clear separation or deeper overlap, or wait for contact support`,
		d.Message,
		`the proximity diagnostic does not describe a near-contact as actual contact`)

	_, pipeline := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPipeline)
	require.False(t, pipeline, `a proximity refusal is not an in-pipeline reach`)
}

func TestVerifyDiagnosticsBeyondTolerance(t *testing.T) {
	// A faceted union carries an Approximate reading; a tolerance just below the
	// proven boundary pushes its bound past the gate.
	doc, body := allPlanarBoolean(t, 0.3)
	required := requiredBodyTolerance(t, body)
	require.Positive(t, required)
	report, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(math.Nextafter(required, 0))))
	require.NoError(t, err)

	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagMeasurementBeyondTolerance)
	require.True(t, ok, `a coarse reading emits DiagMeasurementBeyondTolerance`)
	require.Equal(t, decad.Suspect, d.Status)
	require.NotNil(t, d.Body, `a body reading names its body`)
	require.Nil(t, d.Pair)
	require.NotNil(t, d.Observed)
	require.NotNil(t, d.Required, `Required is rel*Ref, the largest passing bound`)
	require.Equal(t, d.Observed.Value.Kind(), d.Required.Kind(),
		`Required shares the reading's Kind`)
	require.Positive(t, d.Required.Base())
}

func TestVerifyDiagnosticsUndecidedClearance(t *testing.T) {
	// Two box-disjoint cups: the box test proves the pair apart, but the
	// clearance kernel stages the cup payload, so the requested gap is
	// unmeasured — DiagUndecidedClearance, never an unsupported-pair code.
	doc, box1 := shellBox(t)
	cup1, err := box1.Shell(topCap(box1), units.Millimeters(5))
	require.NoError(t, err)
	far, err := r3.Translation(r3.NewVec(500, 0, 0))
	require.NoError(t, err)
	_, err = cup1.Placed(far)
	require.NoError(t, err)
	s2, p2 := plateSketch(t)
	box2, err := doc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	_, err = box2.Shell(topCap(box2), units.Millimeters(5))
	require.NoError(t, err)

	// No clearance asked: bounded cup mass results remain visible.
	quiet, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, quiet.Status)
	requireDiagnosticInvariants(t, quiet)

	// WithClearances invokes the staged kernel: the gap is unmeasured.
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUndecidedClearance)
	require.True(t, ok, `a box-disjoint pair with an unmeasured gap emits DiagUndecidedClearance`)
	require.Equal(t, decad.Suspect, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading)
	require.NotNil(t, d.Pair)
	require.Nil(t, d.Body)
	require.Empty(t, report.Clearances, `no Clearance row is fabricated`)
}
