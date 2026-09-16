package decadtest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// soundPlateDoc builds the "Sound plate" fixture: a fresh document holding
// one 100x60x10 mm block, and the plate itself.
func soundPlateDoc(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()

	doc := decad.New()
	plate := decadtest.Block(t, doc, 0, 0, 100, 60, units.Millimeters(10))
	return doc, plate
}

// interferingPairDoc builds the "Interfering pair" fixture: a fresh document
// holding two overlapping 10 mm tall blocks, (0,0)-(10,10) and
// (5,5)-(20,20), NOT unioned.
func interferingPairDoc(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()

	doc := decad.New()
	a := decadtest.Block(t, doc, 0, 0, 10, 10, units.Millimeters(10))
	b := decadtest.Block(t, doc, 5, 5, 20, 20, units.Millimeters(10))
	return doc, a, b
}

// clearancePairDoc builds the "Clearance pair" fixture: a fresh document
// holding two disjoint 10 mm tall blocks, (0,0)-(10,10) and (30,30)-(40,40).
func clearancePairDoc(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()

	doc := decad.New()
	a := decadtest.Block(t, doc, 0, 0, 10, 10, units.Millimeters(10))
	b := decadtest.Block(t, doc, 30, 30, 40, 40, units.Millimeters(10))
	return doc, a, b
}

// suspectUnionDoc builds the "Suspect report" fixture: a fresh document
// holding the decad.Union of two overlapping 10 mm tall blocks, and the
// union body. The caller supplies decad.WithTolerance to Verify to make the
// four region readings read beyond tolerance.
func suspectUnionDoc(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()

	doc := decad.New()
	a := decadtest.Block(t, doc, 0, 0, 10, 10, units.Millimeters(10))
	b := decadtest.Block(t, doc, 5, 5, 20, 20, units.Millimeters(10))
	u, err := decad.Union(a, b)
	require.NoError(t, err)
	return doc, u
}

func TestVerifyReturnsTheReport(t *testing.T) {
	t.Parallel()

	doc, plate := soundPlateDoc(t)
	report := decadtest.Verify(t, doc)
	require.NotNil(t, report)
	require.Equal(t, decad.Sound, report.Status)
	require.Len(t, report.Bodies, 1)
	require.Equal(t, plate, report.Bodies[0].Body)
}

func TestSoundAcceptsThePlate(t *testing.T) {
	t.Parallel()

	doc, _ := soundPlateDoc(t)
	report := decadtest.Sound(t, doc)
	require.True(t, report.Passed())
}

func TestStatusAcceptsInterfering(t *testing.T) {
	t.Parallel()

	doc, _, _ := interferingPairDoc(t)
	report := decadtest.Verify(t, doc)
	decadtest.Status(t, report, decad.Interfering)
}

func TestBodyReportResolvesTheBody(t *testing.T) {
	t.Parallel()

	doc, plate := soundPlateDoc(t)
	report := decadtest.Verify(t, doc)
	br := decadtest.BodyReport(t, report, plate)
	require.Equal(t, plate, br.Body)
}

func TestValidAcceptsThePlate(t *testing.T) {
	t.Parallel()

	doc, plate := soundPlateDoc(t)
	report := decadtest.Verify(t, doc)
	br := decadtest.Valid(t, report, plate)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
}

func TestValidIgnoresToleranceDiagnostics(t *testing.T) {
	t.Parallel()

	doc, u := suspectUnionDoc(t)
	report := decadtest.Verify(t, doc, decad.WithTolerance(units.Scalar(1e-30)))
	require.Equal(t, decad.Suspect, report.Status)
	require.NotEmpty(t, report.Diagnostics)

	br := decadtest.Valid(t, report, u)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
}

func TestDiagnosedReturnsTheMatchingDiagnostics(t *testing.T) {
	t.Parallel()

	doc, _ := suspectUnionDoc(t)
	report := decadtest.Verify(t, doc, decad.WithTolerance(units.Scalar(1e-30)))

	ds := decadtest.Diagnosed(t, report, decad.DiagMeasurementBeyondTolerance)
	require.Len(t, ds, 4)

	readings := map[decad.ReadingKind]struct{}{}
	for _, d := range ds {
		readings[d.Reading] = struct{}{}
	}
	require.Equal(t, map[decad.ReadingKind]struct{}{
		decad.ReadingArea:     {},
		decad.ReadingBounds:   {},
		decad.ReadingVolume:   {},
		decad.ReadingCentroid: {},
	}, readings)
}

func TestOnlyDiagnosticsAcceptsTheAllowedCode(t *testing.T) {
	t.Parallel()

	doc, _ := suspectUnionDoc(t)
	report := decadtest.Verify(t, doc, decad.WithTolerance(units.Scalar(1e-30)))
	decadtest.OnlyDiagnostics(t, report, decad.DiagMeasurementBeyondTolerance)
}

func TestOnlyDiagnosticsAcceptsACleanReport(t *testing.T) {
	t.Parallel()

	doc, _ := soundPlateDoc(t)
	report := decadtest.Verify(t, doc)
	require.Empty(t, report.Diagnostics)
	decadtest.OnlyDiagnostics(t, report)
}

func TestClearanceMeasuresTheGap(t *testing.T) {
	t.Parallel()

	doc, a, b := clearancePairDoc(t)
	report := decadtest.Verify(t, doc, decad.WithClearances())
	decadtest.Clearance(t, report, a, b, units.Millimeters(28.284271247461902), decadtest.WithinRel(units.Scalar(1e-12)))
}

func TestClearanceFindsThePairInEitherOrder(t *testing.T) {
	t.Parallel()

	doc, a, b := clearancePairDoc(t)
	report := decadtest.Verify(t, doc, decad.WithClearances())
	decadtest.Clearance(t, report, b, a, units.Millimeters(28.284271247461902), decadtest.WithinRel(units.Scalar(1e-12)))
}

func TestInterferenceMeasuresTheOverlap(t *testing.T) {
	t.Parallel()

	doc, a, b := interferingPairDoc(t)
	report := decadtest.Verify(t, doc)
	decadtest.Interference(t, report, a, b, units.CubicMillimeters(250))
}

func TestVerifyRejectsANilDocument(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Verify(tb, nil)
	})
	require.Contains(t, out, "doc must not be nil")
}

func TestSoundReportsASuspectReport(t *testing.T) {
	t.Parallel()

	doc, _ := suspectUnionDoc(t)
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Sound(tb, doc, decad.WithTolerance(units.Scalar(1e-30)))
	})
	require.Contains(t, out, "report is Suspect, want Sound")
}

func TestSoundPrintsEveryDiagnostic(t *testing.T) {
	t.Parallel()

	doc, _ := suspectUnionDoc(t)
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Sound(tb, doc, decad.WithTolerance(units.Scalar(1e-30)))
	})
	require.Contains(t, out, "measurement_beyond_tolerance")
	require.Contains(t, out, "4 diagnostic(s):")
}

func TestSoundNamesTheBodyByStepAndOp(t *testing.T) {
	t.Parallel()

	doc, _ := suspectUnionDoc(t)
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Sound(tb, doc, decad.WithTolerance(units.Scalar(1e-30)))
	})
	require.Contains(t, out, "body[0] (step 2 union)")
}

func TestStatusReportsAMismatch(t *testing.T) {
	t.Parallel()

	doc, _, _ := interferingPairDoc(t)
	report := decadtest.Verify(t, doc)
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Status(tb, report, decad.Sound)
	})
	require.Contains(t, out, "report is Interfering, want Sound")
}

func TestBodyReportRejectsAForeignBody(t *testing.T) {
	t.Parallel()

	doc, plate := soundPlateDoc(t)
	report := decadtest.Verify(t, doc)

	foreignDoc := decad.New()
	foreign := decadtest.Block(t, foreignDoc, 0, 0, 10, 10, units.Millimeters(10))
	_ = plate

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.BodyReport(tb, report, foreign)
	})
	require.Contains(t, out, "the report holds no record of this body")
}

func TestBodyReportRejectsANilReport(t *testing.T) {
	t.Parallel()

	_, plate := soundPlateDoc(t)
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.BodyReport(tb, nil, plate)
	})
	require.Contains(t, out, "report must not be nil")
}

// TestValidReportsAnUndecidedValidity uses a hand-built *decad.BodyReport:
// no public decad operation returns one whose Validity.Outcome is not
// decad.ValidityValid, because every one of decad's six builders sets the
// body solid and assigns a payload on every success path — the invalid and
// undecided branches of publishBodyResult are proven unreachable through a
// live document body (verify_publish_internal_test.go). This record is one
// decad itself would never emit; the literal is the only way to reach
// Valid's validity-mismatch branch, and the point here is Valid's own
// dispatch, never decad's geometry.
func TestValidReportsAnUndecidedValidity(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	plate := decadtest.Block(t, doc, 0, 0, 100, 60, units.Millimeters(10))

	fake := &decad.BodyReport{
		Body: plate,
		Validity: decad.ValidityResult{
			Outcome: decad.ValidityUndecided,
			Diagnostics: []decad.Diagnostic{{
				Code:    decad.DiagUndecidedValidity,
				Status:  decad.Suspect,
				Body:    plate,
				Reading: decad.ReadingNone,
				Message: "the held boundary is not decisive beyond its own proven bound",
			}},
		},
	}
	report := &decad.Report{Bodies: []*decad.BodyReport{fake}}

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Valid(tb, report, plate)
	})
	require.Contains(t, out, "want valid")
}

func TestDiagnosedReportsAnAbsentCode(t *testing.T) {
	t.Parallel()

	doc, _ := soundPlateDoc(t)
	report := decadtest.Verify(t, doc)
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Diagnosed(tb, report, decad.DiagUndercut)
	})
	require.Contains(t, out, "no diagnostic carries")
}

func TestOnlyDiagnosticsReportsADisallowedCode(t *testing.T) {
	t.Parallel()

	doc, _ := suspectUnionDoc(t)
	report := decadtest.Verify(t, doc, decad.WithTolerance(units.Scalar(1e-30)))
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.OnlyDiagnostics(tb, report)
	})
	require.Contains(t, out, "is not among the allowed diagnostic codes")
}

func TestClearanceReportsAMissingRow(t *testing.T) {
	t.Parallel()

	doc, a, b := clearancePairDoc(t)
	report := decadtest.Verify(t, doc) // no decad.WithClearances()
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Clearance(tb, report, a, b, units.Millimeters(1))
	})
	require.Contains(t, out, "no clearance row for the pair")
}

func TestClearanceReportsAGapMiss(t *testing.T) {
	t.Parallel()

	doc, a, b := clearancePairDoc(t)
	report := decadtest.Verify(t, doc, decad.WithClearances())
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Clearance(tb, report, a, b, units.Millimeters(1))
	})
	require.Contains(t, out, "does not enclose")
}

func TestInterferenceReportsAMissingRow(t *testing.T) {
	t.Parallel()

	doc, a, b := clearancePairDoc(t)
	report := decadtest.Verify(t, doc)
	out := captureFailure(t, func(tb testing.TB) {
		decadtest.Interference(tb, report, a, b, units.CubicMillimeters(1))
	})
	require.Contains(t, out, "no interference row for the pair")
}
