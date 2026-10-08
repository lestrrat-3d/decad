package reportvocab

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// WallRequest is the effective WithMinWallThickness spec (proposal §5):
// canonicalized to millimetres and radians.
type WallRequest struct {
	Minimum        units.Value
	DraftAllowance units.Value
}

// UndercutRequest is the effective WithPullDirection spec (proposal §5): the
// exact accepted pull vector, unnormalized.
type UndercutRequest struct {
	PullDirection r3.Vec
}

// VerifyRequest is one Verify call's effective settings (proposal §5),
// recorded on Report.Request even for an empty document. Wall is non-nil
// exactly when WithMinWallThickness was requested, Undercut exactly when
// WithPullDirection was; WallResult.Request and UndercutResult.Request point
// at these same records.
type VerifyRequest struct {
	RelativeTolerance units.Value
	Wall              *WallRequest
	Undercut          *UndercutRequest
	ConcaveRadius     bool
	Clearances        bool
}

// ToleranceResult is one reading's tolerance verdict (proposal §8): Limit is
// nil for the zero-bound short circuit and for an undecided reference,
// otherwise the computed relative-tolerance-times-reference threshold, in
// the reading's own unit.
type ToleranceResult struct {
	State ToleranceState
	Limit *units.Value
}

// ScalarReading is a scalar measurement plus its tolerance verdict
// (proposal §8). Embedding Measurement gives callers typed value access
// (.Value, .Exactness, .Bound) with Tolerance as a separate decision.
type ScalarReading struct {
	measurement.Measurement
	Tolerance ToleranceResult
}

// VectorReading is a vector measurement (a centroid) plus its tolerance
// verdict.
type VectorReading struct {
	measurement.VecMeasurement
	Tolerance ToleranceResult
}

// BoundsReading is a bounding box plus its tolerance verdict.
type BoundsReading struct {
	measurement.Box
	Tolerance ToleranceResult
}

// WallResult is one body's wall survey result (proposal §6). Request is
// non-nil exactly for a requested survey and shares the pointer
// VerifyRequest.Wall carries. Diagnostics is this result's own local
// findings: the survey's own explanation (or the DiagSurveyPrerequisite a
// non-valid body's validity substitutes for it), plus the reading's own
// precision finding when it fired.
type WallResult[BodyT comparable, CellT any] struct {
	Request     *WallRequest
	Outcome     ScalarOutcome
	Minimum     *ScalarReading
	Assessment  Assessment
	Diagnostics []Diagnostic[BodyT, CellT]
}

// UndercutResult is one body's undercut survey result (proposal §7): every
// face in Faces is a CONFIRMED opposing face against the requested pull; no
// uncertain face appears. Faces appear once each, in their body's Faces()
// order.
type UndercutResult[BodyT comparable, FaceT any, CellT any] struct {
	Request     *UndercutRequest
	Coverage    Coverage
	Faces       []FaceT
	Assessment  Assessment
	Diagnostics []Diagnostic[BodyT, CellT]
}

// ConcaveRadiusResult is one body's concave-radius survey result (proposal
// §6). It has no Assessment field: Verify accepts no radius requirement.
type ConcaveRadiusResult[BodyT comparable, CellT any] struct {
	Outcome     ScalarOutcome
	Minimum     *ScalarReading
	Diagnostics []Diagnostic[BodyT, CellT]
}

// ValidityResult is one body's validity verdict and the diagnostic that
// explains it (proposal §9): at most one entry, since a body carries exactly
// one underlying validity finding.
type ValidityResult[BodyT comparable, CellT any] struct {
	Outcome     ValidityOutcome
	Diagnostics []Diagnostic[BodyT, CellT]
}

// HeldTopology is one body's lump and void counts (proposal §9), using the
// current count definitions; more than one lump is descriptive, not a
// failed requirement.
type HeldTopology struct {
	Lumps int
	Voids int
}

// RegionReadings is one proven-solid body's region quantities (proposal §9):
// present on BodyReport.Region exactly when Validity.Outcome is
// ValidityValid.
type RegionReadings struct {
	Volume   ScalarReading
	Centroid VectorReading
}

// BodyReport is one live body's verdict and readings (verification §1,
// proposal §3). Area and Bounds are unconditional boundary readings on
// every returned body; Region carries the two region quantities and is
// non-nil exactly when Validity.Outcome is ValidityValid (proposal §9).
// Every successful Verify call fills Validity.Outcome, Wall.Outcome,
// Undercut.Coverage, and ConcaveRadius.Outcome nonzero — NotRequested for an
// omitted survey — though this guarantee does not extend to Assessment or
// Tolerance states (proposal §4). Diagnostics is the deterministic
// flattening of Validity's, the core readings', Wall's, Undercut's, and
// ConcaveRadius's own local diagnostics, in that order, each finding
// present exactly once even though it is also reachable through its own
// result's local Diagnostics slice (proposal §10); it is empty exactly when
// Status is Sound.
type BodyReport[BodyT comparable, FaceT any, CellT any] struct {
	Body          BodyT
	Status        Status
	Validity      ValidityResult[BodyT, CellT]
	Topology      HeldTopology
	Area          ScalarReading
	Bounds        BoundsReading
	Region        *RegionReadings
	Wall          WallResult[BodyT, CellT]
	Undercut      UndercutResult[BodyT, FaceT, CellT]
	ConcaveRadius ConcaveRadiusResult[BodyT, CellT]
	Diagnostics   []Diagnostic[BodyT, CellT]
}

// Report is what Verify returns: the 3D counterpart of
// sketch.VerificationReport (core §10, verification §1). Request records the
// validated effective settings this call used, including defaults, even for
// an empty document (proposal §5). Bodies follows Document.Bodies() order at
// the call. Diagnostics is the canonical inventory for counting and logging
// the whole call: body diagnostics in body order, then pair diagnostics in
// pair order, each finding present exactly once (proposal §10, §11); it is
// empty EXACTLY when Status == Sound, and Status is the worst
// Diagnostic.Status in it. The zero Report is Unverified and carries no
// verdict.
type Report[BodyT comparable, FaceT any, CellT any] struct {
	Request       VerifyRequest
	Bodies        []*BodyReport[BodyT, FaceT, CellT]
	Interferences []Interference[BodyT]
	Clearances    []Clearance[BodyT]
	Diagnostics   []Diagnostic[BodyT, CellT]
	Status        Status
}

// Passed reports whether the whole report is Sound for the effective request,
// including the default interference check (proposal §1, §4). It does not
// imply that every optional survey ran — Wall, Undercut, and ConcaveRadius
// can each read NotRequested on a Sound report. It returns false for a nil
// Report and for any Status other than Sound, including an unrecognized one:
// it is a convenience predicate over Status, not a validator of a
// caller-built or decoded Report, so an unrecognized nested result enum
// never overrides a caller-assigned Sound Status.
func (r *Report[BodyT, FaceT, CellT]) Passed() bool {
	return r != nil && r.Status == Sound
}

// ForBody looks up body's BodyReport by exact pointer identity (proposal
// §11). A nil receiver or a nil body returns ErrDegenerate. Any absent
// non-nil identity, including a foreign body, returns
// ErrBodyReportNotFound. Successful lookup returns the report's own
// existing pointer and performs no verification: it consults only this
// Report's recorded bodies, never current document membership or liveness,
// so it still resolves a body later retired from its document.
func (r *Report[BodyT, FaceT, CellT]) ForBody(body BodyT) (*BodyReport[BodyT, FaceT, CellT], error) {
	var zero BodyT
	if r == nil || body == zero {
		return nil, fmt.Errorf(`%w: a nil report or body names no lookup`, decaderr.ErrDegenerate)
	}
	for _, br := range r.Bodies {
		if br.Body == body {
			return br, nil
		}
	}
	return nil, decaderr.ErrBodyReportNotFound
}
