package decad

import (
	"context"
	"errors"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

// OperationDiagnosticError is implemented by errors with structured operation reasons.
type OperationDiagnosticError interface {
	error
	OperationDiagnostics() []OperationDiagnostic
}

// OperationDiagnosticCode is a stable reason code for an operation event.
type OperationDiagnosticCode int

const (
	OpDiagUnknown OperationDiagnosticCode = iota
	OpDiagInputRejected
	OpDiagSelectionFailed
	OpDiagProfileUnrecordable
	OpDiagResultEmpty
	OpDiagUnsupportedContact
	OpDiagOperandProofUnavailable
	OpDiagOperandBoundExceeded
	OpDiagTessellationUnresolved
	OpDiagWorkLimitReached
	OpDiagUnsupportedIntent
	OpDiagEvaluatorFailure
	OpDiagGapShiftApplied
	OpDiagEmbeddingReachApplied
	OpDiagStitchMergeApplied
	OpDiagChainAllowanceApplied
)

var operationDiagnosticCodeNames = [...]string{
	operationUnknownName,
	"input_rejected",
	"selection_failed",
	"profile_unrecordable",
	"result_empty",
	"unsupported_contact",
	"operand_proof_unavailable",
	"operand_bound_exceeded",
	"tessellation_unresolved",
	"work_limit_reached",
	"unsupported_intent",
	"evaluator_failure",
	"gap_shift_applied",
	"embedding_reach_applied",
	"stitch_merge_applied",
	"chain_allowance_applied",
}

// String returns the stable lower-snake token for c.
func (c OperationDiagnosticCode) String() string {
	if int(c) >= 0 && int(c) < len(operationDiagnosticCodeNames) {
		return operationDiagnosticCodeNames[c]
	}
	return fmt.Sprintf("OperationDiagnosticCode(%d)", int(c))
}

// OperationDiagnosticSeverity reports whether an event is a warning or failure.
type OperationDiagnosticSeverity int

const (
	OperationDiagnosticSeverityUnknown OperationDiagnosticSeverity = iota
	OperationDiagnosticWarning
	OperationDiagnosticFailure
)

func (s OperationDiagnosticSeverity) String() string {
	switch s {
	case OperationDiagnosticSeverityUnknown:
		return operationUnknownName
	case OperationDiagnosticWarning:
		return "warning"
	case OperationDiagnosticFailure:
		return "failure"
	default:
		return fmt.Sprintf("OperationDiagnosticSeverity(%d)", int(s))
	}
}

// OperationStage names the broad stage where an operation reason occurred.
type OperationStage int

const (
	OperationStageUnknown OperationStage = iota
	OperationStageInput
	OperationStageRecording
	OperationStageEvaluation
	OperationStageResult
)

func (s OperationStage) String() string {
	if int(s) >= 0 && int(s) < len(operationStageNames) {
		return operationStageNames[s]
	}
	return fmt.Sprintf("OperationStage(%d)", int(s))
}

var operationStageNames = [...]string{operationUnknownName, "input", "recording", "evaluation", "result"}

// OperationSubject identifies a public argument location.
type OperationSubject struct {
	Name    string
	Ordinal *int
	Body    *Body
}

// OperationChangeMechanism names a bounded adjustment made to accepted geometry.
type OperationChangeMechanism int

const (
	OperationChangeUnknown OperationChangeMechanism = iota
	OperationChangeGapShift
	OperationChangeEmbeddingReach
	OperationChangeStitchMerge
)

func (m OperationChangeMechanism) String() string {
	switch m {
	case OperationChangeUnknown:
		return operationUnknownName
	case OperationChangeGapShift:
		return "gap_shift"
	case OperationChangeStitchMerge:
		return "stitch_merge"
	case OperationChangeEmbeddingReach:
		return "embedding_reach"
	default:
		return fmt.Sprintf("OperationChangeMechanism(%d)", int(m))
	}
}

// OperationChange describes a certified geometric adjustment.
type OperationChange struct {
	Mechanism         OperationChangeMechanism
	DisplacementBound units.Value
	AllowedReach      units.Value
}

// OperationChainDetail preserves the three values of the local chain gate.
type OperationChainDetail struct {
	TouchedBound   units.Value
	PairChordLimit units.Value
	ChainAllowance *units.Value
}

// OperationWorkLimit describes an existing bounded-work refusal.
type OperationWorkLimit struct {
	Resource string
	Observed int
	Maximum  int
}

// OperationDiagnostic is one structured operation failure or warning.
type OperationDiagnostic struct {
	Code      OperationDiagnosticCode
	Severity  OperationDiagnosticSeverity
	Operation OperationKind
	Stage     OperationStage
	Subject   OperationSubject
	Chain     *OperationChainDetail
	Change    *OperationChange
	WorkLimit *OperationWorkLimit
	Message   string
}

// OperationError preserves an original error and adds structured diagnostics.
type OperationError struct {
	err         error
	diagnostics []OperationDiagnostic
}

func (e *OperationError) Error() string { return e.err.Error() }
func (e *OperationError) Unwrap() error { return e.err }
func (e *OperationError) OperationDiagnostics() []OperationDiagnostic {
	if e == nil {
		return nil
	}
	return cloneOperationDiagnostics(e.diagnostics)
}

func operationError(kind OperationKind, err error) error {
	if err == nil || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return err
	}
	if _, ok := errors.AsType[OperationDiagnosticError](err); ok {
		if nested, ok := errors.AsType[*OperationError](err); ok {
			diagnostics := nested.OperationDiagnostics()
			changed := false
			for i := range diagnostics {
				if diagnostics[i].Operation == OperationSelectEdges || diagnostics[i].Operation == OperationSelectFaces {
					continue
				}
				diagnostics[i].Operation = kind
				changed = true
			}
			if changed {
				return &OperationError{err: err, diagnostics: diagnostics}
			}
		}
		return err
	}
	if _, ok := errors.AsType[*tessellation.ExpectedError](err); ok {
		return &OperationError{err: err, diagnostics: []OperationDiagnostic{{
			Code: OpDiagTessellationUnresolved, Severity: OperationDiagnosticFailure,
			Operation: kind, Stage: OperationStageEvaluation, Message: err.Error(),
		}}}
	}
	if coarse, ok := errors.AsType[*meshbool.CoarseHeldContactError](err); ok {
		name, ordinal := booleanOperandLocation(kind, coarse.Operand)
		diagnostic := OperationDiagnostic{
			Code: OpDiagOperandBoundExceeded, Severity: OperationDiagnosticFailure,
			Operation: kind, Stage: OperationStageEvaluation,
			Subject: OperationSubject{Name: name, Ordinal: ordinal},
			Chain: &OperationChainDetail{TouchedBound: units.Millimeters(coarse.Bound),
				PairChordLimit: units.Millimeters(coarse.Tol)},
			Message: err.Error(),
		}
		if coarse.ChainAllowance > 0 {
			allowance := units.Millimeters(coarse.ChainAllowance)
			diagnostic.Chain.ChainAllowance = &allowance
		}
		return &OperationError{err: err, diagnostics: []OperationDiagnostic{diagnostic}}
	}
	if arrangementLimit, ok := errors.AsType[*prismcells.WorkLimitError](err); ok {
		return &OperationError{err: err, diagnostics: []OperationDiagnostic{{
			Code: OpDiagWorkLimitReached, Severity: OperationDiagnosticFailure,
			Operation: kind, Stage: OperationStageEvaluation,
			WorkLimit: &OperationWorkLimit{Resource: "arrangement_segments",
				Observed: arrangementLimit.Observed, Maximum: arrangementLimit.Maximum},
			Message: err.Error(),
		}}}
	}
	if proofLimit, ok := errors.AsType[*proofbound.WorkLimitError](err); ok {
		maxInt := uint64(^uint(0) >> 1)
		var workLimit *OperationWorkLimit
		if proofLimit.Observed <= maxInt && proofLimit.Maximum <= maxInt {
			workLimit = &OperationWorkLimit{Resource: proofLimit.Resource,
				Observed: int(proofLimit.Observed), Maximum: int(proofLimit.Maximum)}
		}
		return &OperationError{err: err, diagnostics: []OperationDiagnostic{{
			Code: OpDiagWorkLimitReached, Severity: OperationDiagnosticFailure,
			Operation: kind, Stage: OperationStageEvaluation,
			WorkLimit: workLimit, Message: err.Error(),
		}}}
	}
	if expected, ok := errors.AsType[*meshbool.BooleanExpectedError](err); ok {
		code, stage := OpDiagUnsupportedIntent, OperationStageEvaluation
		switch expected.Kind {
		case meshbool.BooleanExpectedEmpty:
			code, stage = OpDiagResultEmpty, OperationStageResult
		case meshbool.BooleanExpectedContact:
			code = OpDiagUnsupportedContact
		case meshbool.BooleanExpectedCoarseTessellation:
			code = OpDiagTessellationUnresolved
		case meshbool.BooleanExpectedVolumeProof:
			code = OpDiagOperandProofUnavailable
		}
		diagnostic := OperationDiagnostic{Code: code, Severity: OperationDiagnosticFailure,
			Operation: kind, Stage: stage, Message: err.Error()}
		if expected.Operand >= 0 {
			name, ordinal := booleanOperandLocation(kind, expected.Operand)
			diagnostic.Subject = OperationSubject{Name: name, Ordinal: ordinal}
		}
		return &OperationError{err: err, diagnostics: []OperationDiagnostic{diagnostic}}
	}
	code, stage, ok := operationErrorCode(err)
	if !ok {
		return err
	}
	return &OperationError{err: err, diagnostics: []OperationDiagnostic{{
		Code: code, Severity: OperationDiagnosticFailure, Operation: kind, Stage: stage, Message: err.Error(),
	}}}
}

func booleanOperandLocation(kind OperationKind, operand int) (string, *int) {
	ordinal := operand
	switch kind {
	case OperationCut:
		if operand == 0 {
			return operationRoleTarget, nil
		}
		return operationRoleTool, nil
	case OperationIntersect:
		if operand == 0 {
			return "a", nil
		}
		return "b", nil
	case OperationUnion:
		if operand == 0 {
			return "first", nil
		}
		ordinal--
		return "others", &ordinal
	default:
		return operationRoleOperand, &ordinal
	}
}

func operationErrorCode(err error) (OperationDiagnosticCode, OperationStage, bool) {
	switch {
	case errors.Is(err, ErrDegenerate):
		return OpDiagInputRejected, OperationStageInput, true
	case errors.Is(err, ErrUnrecordableProfile):
		return OpDiagProfileUnrecordable, OperationStageRecording, true
	case errors.Is(err, ErrNoMatch), errors.Is(err, ErrCardinality):
		return OpDiagSelectionFailed, OperationStageEvaluation, true
	case errors.Is(err, ErrUnsupported):
		return OpDiagUnsupportedIntent, OperationStageEvaluation, true
	case errors.Is(err, ErrBooleanFailed):
		return OpDiagEvaluatorFailure, OperationStageResult, true
	case errors.Is(err, ErrForeignBody), errors.Is(err, ErrForeignProfile), errors.Is(err, ErrStaleProfile),
		errors.Is(err, ErrRetiredBody), errors.Is(err, ErrNegativeMagnitude), errors.Is(err, ErrInvalidProfile),
		errors.Is(err, ErrUnitKind), errors.Is(err, ErrNotFinite), errors.Is(err, ErrNotSolid):
		return OpDiagInputRejected, OperationStageInput, true
	default:
		return OpDiagUnknown, OperationStageUnknown, false
	}
}

func (e *BooleanError) OperationDiagnostics() []OperationDiagnostic {
	if e == nil {
		return nil
	}
	code := OpDiagEvaluatorFailure
	switch e.Code {
	case BooleanEmpty:
		code = OpDiagResultEmpty
	case BooleanUnsupportedContact:
		code = OpDiagUnsupportedContact
	case BooleanEvaluatorFailure:
		code = OpDiagEvaluatorFailure
	}
	stage := OperationStageEvaluation
	if code == OpDiagResultEmpty {
		stage = OperationStageResult
	}
	diagnostic := OperationDiagnostic{
		Code:      code,
		Severity:  OperationDiagnosticFailure,
		Operation: operationKindForBoolean(e.op),
		Stage:     stage,
		Message:   e.msg,
	}
	var expected *meshbool.BooleanExpectedError
	if errors.As(e.err, &expected) && expected.Operand >= 0 {
		name, ordinal := booleanOperandLocation(diagnostic.Operation, expected.Operand)
		diagnostic.Subject = OperationSubject{Name: name, Ordinal: ordinal}
	}
	out := []OperationDiagnostic{diagnostic}
	if e.shift != nil {
		name, ordinal := booleanOperandLocation(diagnostic.Operation, 1)
		out = append(out, OperationDiagnostic{
			Code: OpDiagGapShiftApplied, Severity: OperationDiagnosticWarning,
			Operation: diagnostic.Operation, Stage: OperationStageResult,
			Subject: OperationSubject{Name: name, Ordinal: ordinal},
			Change: &OperationChange{Mechanism: OperationChangeGapShift,
				DisplacementBound: e.shift.DisplacementBound, AllowedReach: e.shift.AllowedReach},
			Message: "The accepted Boolean retry shifted an operand before producing an empty result.",
		})
	}
	return out
}

func (e *SelectionError) OperationDiagnostics() []OperationDiagnostic {
	if e == nil {
		return nil
	}
	kind := OperationSelectEdges
	if e.Kind == FaceSelectorKind {
		kind = OperationSelectFaces
	}
	return []OperationDiagnostic{{
		Code:      OpDiagSelectionFailed,
		Severity:  OperationDiagnosticFailure,
		Operation: kind,
		Stage:     OperationStageEvaluation,
		Subject:   OperationSubject{Name: "selector", Body: e.Body},
		Message:   e.Error(),
	}}
}

// OperationDiagnostics returns warnings from the call that created b.
func (b *Body) OperationDiagnostics() []OperationDiagnostic {
	if b == nil {
		return nil
	}
	if len(b.n.OperationWarnings) == 0 {
		return nil
	}
	out := make([]OperationDiagnostic, 0, len(b.n.OperationWarnings))
	for _, record := range b.n.OperationWarnings {
		if record.Code == opdiagnostic.CodeEmbeddingReachApplied {
			out = append(out, OperationDiagnostic{
				Code: OpDiagEmbeddingReachApplied, Severity: OperationDiagnosticWarning,
				Operation: OperationKind(record.PublicOperation), Stage: OperationStageResult,
				Change: &OperationChange{Mechanism: OperationChangeEmbeddingReach,
					DisplacementBound: units.Millimeters(record.DisplacementBound),
					AllowedReach:      units.Millimeters(record.AllowedReach)},
				Message: "The accepted held-mesh repair used a position within the configured gap tolerance.",
			})
			continue
		}
		if record.Code == opdiagnostic.CodeStitchMergeApplied {
			ordinal := record.Operand
			out = append(out, OperationDiagnostic{
				Code: OpDiagStitchMergeApplied, Severity: OperationDiagnosticWarning,
				Operation: OperationStitch, Stage: OperationStageResult,
				Subject: OperationSubject{Name: "bodies", Ordinal: &ordinal},
				Change: &OperationChange{Mechanism: OperationChangeStitchMerge,
					DisplacementBound: units.Millimeters(record.DisplacementBound),
					AllowedReach:      units.Millimeters(record.AllowedReach)},
				Message: "Stitch merged distinct input positions within the configured gap tolerance.",
			})
			continue
		}
		if record.Code == opdiagnostic.CodeChainAllowanceApplied {
			kind := operationKindForBoolean(meshbool.OperationKind(record.Operation))
			if record.PublicOperation != 0 {
				kind = OperationKind(record.PublicOperation)
			}
			var subject OperationSubject
			if record.Operand >= 0 {
				name, ordinal := booleanOperandLocation(kind, record.Operand)
				subject = OperationSubject{Name: name, Ordinal: ordinal}
			}
			allowance := units.Millimeters(*record.ChainAllowance)
			out = append(out, OperationDiagnostic{
				Code: OpDiagChainAllowanceApplied, Severity: OperationDiagnosticWarning,
				Operation: kind, Stage: OperationStageEvaluation,
				Subject: subject,
				Chain: &OperationChainDetail{
					TouchedBound:   units.Millimeters(record.TouchedBound),
					PairChordLimit: units.Millimeters(record.PairChordLimit), ChainAllowance: &allowance,
				},
				Message: "The configured chain allowance raised the local held-facet bound gate.",
			})
			continue
		}
		if record.Code != opdiagnostic.CodeGapShiftApplied {
			continue
		}
		kind := operationKindForBoolean(meshbool.OperationKind(record.Operation))
		if record.PublicOperation != 0 {
			kind = OperationKind(record.PublicOperation)
		}
		var subject OperationSubject
		if record.Operand >= 0 {
			name, ordinal := booleanOperandLocation(kind, record.Operand)
			subject = OperationSubject{Name: name, Ordinal: ordinal}
		}
		out = append(out, OperationDiagnostic{
			Code:      OpDiagGapShiftApplied,
			Severity:  OperationDiagnosticWarning,
			Operation: kind,
			Stage:     OperationStageResult,
			Subject:   subject,
			Change: &OperationChange{
				Mechanism:         OperationChangeGapShift,
				DisplacementBound: units.Millimeters(record.DisplacementBound),
				AllowedReach:      units.Millimeters(record.AllowedReach),
			},
			Message: "The accepted Boolean retry shifted an operand within the configured gap tolerance.",
		})
	}
	return cloneOperationDiagnostics(out)
}

func cloneOperationDiagnostics(in []OperationDiagnostic) []OperationDiagnostic {
	if len(in) == 0 {
		return nil
	}
	out := slices.Clone(in)
	for i := range out {
		if in[i].Subject.Ordinal != nil {
			ordinal := *in[i].Subject.Ordinal
			out[i].Subject.Ordinal = &ordinal
		}
		if in[i].Chain != nil {
			chain := *in[i].Chain
			if chain.ChainAllowance != nil {
				allowance := *chain.ChainAllowance
				chain.ChainAllowance = &allowance
			}
			out[i].Chain = &chain
		}
		if in[i].Change != nil {
			change := *in[i].Change
			out[i].Change = &change
		}
		if in[i].WorkLimit != nil {
			limit := *in[i].WorkLimit
			out[i].WorkLimit = &limit
		}
	}
	return out
}
