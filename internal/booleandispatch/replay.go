package booleandispatch

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// BodyBrakeClevisCut runs the brake route's one fixed-scale private Cut.
func BodyBrakeClevisCut(ctx context.Context, env topo.Env, a, b *topo.Body, hooks Hooks, budget *proofbound.ReplayBudget) (*topo.Body, error) {
	return bodyScaledReplay(ctx, meshbool.OpCut, env, a, b, hooks, .25, 0, 0, nil, budget, true, nil)
}

// BodyBrakeRefinedClevisCut requires a verified, single-use refined tool certificate.
func BodyBrakeRefinedClevisCut(ctx context.Context, env topo.Env, a, b *topo.Body, cert *booleaneval.BrakeCandidateCertificate,
	hooks Hooks, budget *proofbound.ReplayBudget) (*topo.Body, error) {
	if cert == nil {
		return nil, fmt.Errorf("%w: refined clevis tool certificate missing", decaderr.ErrUnsupported)
	}
	return bodyScaledReplay(ctx, meshbool.OpCut, env, a, b, hooks, .25, 0, 0, nil, budget, true, cert)
}

// BodyBrakeRetainedUnion runs the authenticated brake route's clamped Union.
func BodyBrakeRetainedUnion(ctx context.Context, env topo.Env, a, b *topo.Body, hooks Hooks, chordScale, candidate, original float64, retained *booleaneval.RetainedTarget, budget *proofbound.ReplayBudget) (*topo.Body, error) {
	if retained == nil {
		return nil, fmt.Errorf("%w: missing brake target certificate", decaderr.ErrUnsupported)
	}
	return bodyScaledReplay(ctx, meshbool.OpUnion, env, a, b, hooks, chordScale, candidate, original, retained, budget, true, nil)
}

func bodyScaledReplay(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *topo.Body, hooks Hooks, chordScale, candidate, original float64, retained *booleaneval.RetainedTarget, budget *proofbound.ReplayBudget, brake bool, cert *booleaneval.BrakeCandidateCertificate) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if budget == nil || a == nil || b == nil || hooks.AsBooleanError == nil {
		return nil, fmt.Errorf("%w: bounded Boolean needs operands, an error mapper and a budget", decaderr.ErrUnsupported)
	}
	// Placement and authenticated shift capture are not connected yet.
	if !replayReady && (!brake || !budget.IsBrakeReplay()) {
		return nil, fmt.Errorf("%w: bounded Boolean replay is not yet complete", decaderr.ErrUnsupported)
	}
	// In the admitted Union neither operand can enter the lower-support or
	// prism recognition routes. The admitted Cut cannot enter the cone-toe
	// or circular-prism recognition routes.
	switch op {
	case meshbool.OpUnion:
		if _, ok := a.Payload.(prismrecord.Payload); ok {
			admitted, err := replayCircleFacetedUnionRecognizersDecline(ctx, a, b, brake && retained != nil, budget)
			if err != nil {
				return nil, err
			}
			if !admitted {
				return nil, fmt.Errorf("%w: bounded Union support recognition is unavailable", decaderr.ErrUnsupported)
			}
		}
		if _, ok := b.Payload.(prismrecord.Payload); ok {
			return nil, fmt.Errorf("%w: bounded Union support recognition is unavailable", decaderr.ErrUnsupported)
		}
	case meshbool.OpCut:
		if p, ok := a.Payload.(facetedrecord.Payload); ok && p.LoftFacesPrimitive {
			return nil, fmt.Errorf("%w: bounded cone-toe recognition is unavailable", decaderr.ErrUnsupported)
		}
		if p, ok := a.Payload.(prismrecord.Payload); ok && len(p.Profile.Outer.Segments) != 4 {
			return nil, fmt.Errorf("%w: bounded Cut requires the recorded rectangular blank", decaderr.ErrUnsupported)
		}
	default:
		return nil, fmt.Errorf("%w: bounded Boolean operation is unavailable", decaderr.ErrUnsupported)
	}
	var ev booleaneval.Evaluation
	var err error
	if cert != nil {
		if err := booleaneval.AuthorizeBrakeCandidate(ctx, cert, a, b, budget); err != nil {
			return nil, err
		}
		ev, err = booleaneval.EvaluateBrakeClevisReplay(ctx, a, b, cert, budget)
	} else if retained != nil {
		ev, err = booleaneval.EvaluateReplayRetainedClamped(ctx, a, b, chordScale, candidate, original, retained, budget)
	} else if candidate != 0 {
		ev, err = booleaneval.EvaluateReplayClamped(ctx, op, a, b, chordScale, candidate, original, budget)
	} else {
		ev, err = booleaneval.EvaluateReplay(ctx, op, a, b, chordScale, budget)
	}
	if err != nil {
		return nil, hooks.AsBooleanError(op, err)
	}
	volume := measurement.Measurement{Value: units.CubicMillimeters(ev.Volume), Exactness: measurement.ExactnessOf(ev.VolumeBound), Bound: units.CubicMillimeters(ev.VolumeBound)}
	body, ok, err := facetedrecord.BuildWithProofReplay(ctx, budget.WorkBudget(ctx), env, ev.Payload, ev.Audit, volume, ev.VolumeRat)
	if err != nil {
		return nil, hooks.AsBooleanError(op, err)
	}
	if !ok {
		return nil, hooks.AsBooleanError(op, fmt.Errorf("%w: bounded result body refused", decaderr.ErrUnsupported))
	}
	if err := budget.ReserveRecords(ctx, 8); err != nil {
		return nil, err
	}
	diagnostics, err := ev.DiagnosticsReplay(ctx, op, budget)
	if err != nil {
		return nil, err
	}
	if len(diagnostics) > 0 {
		if err := budget.ReserveArray(ctx, uint64(len(body.OperationWarnings)+len(diagnostics)), uint64(unsafe.Sizeof(opdiagnostic.Record{}))); err != nil {
			return nil, err
		}
		warnings := make([]opdiagnostic.Record, 0, len(body.OperationWarnings)+len(diagnostics))
		warnings = append(warnings, body.OperationWarnings...)
		warnings = append(warnings, diagnostics...)
		body.OperationWarnings = warnings
	}
	if ev.ShiftCertificate != nil {
		if cert != nil && !ev.ShiftCertificate.SealPublicCutWarning(env.Ref, body.OperationWarnings, 0) {
			return nil, fmt.Errorf("%w: bounded clevis shift warning changed", decaderr.ErrUnsupported)
		}
		body.PrivateShift = ev.ShiftCertificate
	}
	return body, nil
}

// The ordinary Union recognizers require two prism payloads, except lower
// support, which requires a rectangular prism source. A one-segment circular
// prism and a faceted operand cannot enter any of those recognizers.
func replayCircleFacetedUnionRecognizersDecline(ctx context.Context, a, b *topo.Body, route bool,
	budget *proofbound.ReplayBudget) (bool, error) {
	if err := budget.SpendWork(ctx, 16); err != nil {
		return false, err
	}
	if !route || a == nil || b == nil {
		return false, nil
	}
	p, ok := a.Payload.(prismrecord.Payload)
	if !ok || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) != 1 {
		return false, nil
	}
	if _, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg); !ok {
		return false, nil
	}
	_, ok = b.Payload.(facetedrecord.Payload)
	return ok, nil
}

const replayReady = false

// BodyBrakeFinalCut uses only the brake route's authenticated tool certificate.
func BodyBrakeFinalCut(ctx context.Context, env topo.Env, a, b *topo.Body, tool *booleaneval.FixedShiftedTool,
	targetAllowance float64, hooks Hooks, budget *proofbound.ReplayBudget) (*topo.Body, error) {
	return bodyFixedCutReplay(ctx, env, a, b, tool, targetAllowance, hooks, budget, true)
}

func bodyFixedCutReplay(ctx context.Context, env topo.Env, a, b *topo.Body, tool *booleaneval.FixedShiftedTool,
	targetAllowance float64, hooks Hooks, budget *proofbound.ReplayBudget, brake bool) (*topo.Body, error) {
	if !replayReady && (!brake || !budget.IsBrakeReplay()) {
		return nil, fmt.Errorf("%w: bounded Boolean replay is not yet complete", decaderr.ErrUnsupported)
	}
	if hooks.AsBooleanError == nil || budget == nil {
		return nil, fmt.Errorf("%w: bounded Cut requires an error mapper and budget", decaderr.ErrUnsupported)
	}
	ev, err := booleaneval.EvaluateReplayFixedCut(ctx, a, b, tool, targetAllowance, budget)
	if err != nil {
		return nil, hooks.AsBooleanError(meshbool.OpCut, err)
	}
	volume := measurement.Measurement{Value: units.CubicMillimeters(ev.Volume), Exactness: measurement.ExactnessOf(ev.VolumeBound), Bound: units.CubicMillimeters(ev.VolumeBound)}
	body, ok, err := facetedrecord.BuildWithProofReplay(ctx, budget.WorkBudget(ctx), env, ev.Payload, ev.Audit, volume, ev.VolumeRat)
	if err != nil {
		return nil, hooks.AsBooleanError(meshbool.OpCut, err)
	}
	if !ok {
		return nil, fmt.Errorf("%w: bounded Cut body refused", decaderr.ErrUnsupported)
	}
	if err := budget.ReserveRecords(ctx, 8); err != nil {
		return nil, err
	}
	diagnostics, err := ev.DiagnosticsReplay(ctx, meshbool.OpCut, budget)
	if err != nil {
		return nil, err
	}
	if len(diagnostics) > 0 {
		if err := budget.ReserveArray(ctx, uint64(len(body.OperationWarnings)+len(diagnostics)), uint64(unsafe.Sizeof(opdiagnostic.Record{}))); err != nil {
			return nil, err
		}
		warnings := make([]opdiagnostic.Record, 0, len(body.OperationWarnings)+len(diagnostics))
		warnings = append(warnings, body.OperationWarnings...)
		warnings = append(warnings, diagnostics...)
		body.OperationWarnings = warnings
	}
	return body, nil
}
