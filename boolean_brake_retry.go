package decad

import (
	"context"
	"errors"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/chaintol"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type brakeReplayInputs struct {
	ctx                   context.Context //nolint:containedctx // The value lives only for one bounded replay call.
	a, b, blank, u, c0, c *topo.Body
	placed                *topo.Body
	sourceResult          *topo.Body
	cutSnapshot           *booleaneval.ShiftCertificateSnapshot
	fixed                 *booleaneval.FixedShiftedTool
	retained              *booleaneval.RetainedTarget
	bWarning              opdiagnostic.Record
	c0Growth              *booleaneval.ShiftCertificate
	c0Snapshot            *booleaneval.ShiftCertificateSnapshot
	c0Warning             opdiagnostic.Record
}

const brakeReplayEnabled = true

func reserveBrakeReplayInputs(ctx context.Context, budget *proofbound.ReplayBudget) error {
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return err
	}
	return budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(brakeReplayInputs{})))
}

type brakeReplayMeterKey struct{}
type brakeReplayTraceKey struct{}
type brakeReplayTrace struct {
	stage string
	err   error
}

func traceBrakeReplay(ctx context.Context, stage string, err error) {
	if trace, ok := ctx.Value(brakeReplayTraceKey{}).(*brakeReplayTrace); ok && trace != nil && trace.stage == "" {
		trace.stage, trace.err = stage, err
	}
}

func retryBrakeUnion(ctx context.Context, d *Document, op meshbool.OperationKind, a, b *Body,
	cfg booleanConfig, cause error) (*Body, producerID, bool, bool, error) {
	if !brakeReplayEnabled {
		return nil, 0, false, false, nil
	}
	budget := proofbound.NewBrakeReplayBudget()
	if meter, ok := ctx.Value(brakeReplayMeterKey{}).(**proofbound.ReplayBudget); ok && meter != nil {
		if *meter != nil {
			budget = *meter
		} else {
			*meter = budget
		}
	}
	return retryBrakeUnionBudgeted(ctx, d, op, a, b, cfg, cause, budget)
}

func retryBrakeUnionBudgeted(ctx context.Context, d *Document, op meshbool.OperationKind, a, b *Body,
	cfg booleanConfig, cause error, budget *proofbound.ReplayBudget) (*Body, producerID, bool, bool, error) {
	in, err := brakeReplaySources(ctx, d, op, a, b, cfg, cause, budget)
	if err != nil {
		traceBrakeReplay(ctx, "source authentication", err)
		return brakeRetryFailure(ctx, err)
	}
	if in == nil {
		traceBrakeReplay(ctx, "source eligibility", nil)
		return nil, 0, false, false, nil
	}
	if in.c0Growth != nil && len(in.u.OperationWarnings) == 0 {
		return retryBrakeRefinedUnion(d, in, booleanHooks(cfg), budget)
	}
	ref, hooks := d.nextProducerID(), booleanHooks(cfg)
	placed, growth, growthWarnings, err := brakeReplayClevis(d, in, ref, hooks, budget)
	if err != nil {
		traceBrakeReplay(ctx, "clevis rebuild", err)
		return brakeRetryFailure(ctx, err)
	}
	joined, movement, shiftWarnings, err := brakeReplayUnion(d, in, placed, growth, ref+1, hooks, budget)
	if err != nil {
		traceBrakeReplay(ctx, "private Union", err)
		return brakeRetryFailure(ctx, err)
	}
	final, err := brakeReplayFinal(d, in, joined, movement, growthWarnings, shiftWarnings, ref+2, hooks, budget)
	if err != nil {
		traceBrakeReplay(ctx, "final Cut", err)
		return brakeRetryFailure(ctx, err)
	}
	for _, source := range []*topo.Body{in.a, in.b, in.blank, in.u, in.c0, in.c} {
		if source == nil || source.Origin.Producer == ref || source.Origin.Producer == ref+1 || source.Origin.Producer == ref+2 {
			return brakeRetryFailure(ctx, errBrakePrivateProducerCollidesWithAPublicSource)
		}
	}
	if err := stripBrakePrivateSources(ctx, final, placed, joined, ref, budget); err != nil {
		traceBrakeReplay(ctx, "source ancestry", err)
		return brakeRetryFailure(ctx, err)
	}
	return bodyHandle(final), 3, true, true, nil
}

// brakeReplaySources authenticates the recorded Cut and exact clevis placement
// before any private Boolean is attempted. The caller owns the shared budget.
func brakeReplaySources(ctx context.Context, d *Document, op meshbool.OperationKind, target, c *Body,
	cfg booleanConfig, cause error, budget *proofbound.ReplayBudget) (*brakeReplayInputs, error) {
	var held *meshbool.CoarseHeldContactError
	if budget == nil || d == nil || target == nil || c == nil || op != meshbool.OpUnion || cfg.matchingFaces ||
		!errors.Is(cause, ErrUnsupported) || !errors.As(cause, &held) || held.Operand != 0 ||
		len(target.n.OperationWarnings) != 1 || len(c.n.OperationWarnings) != 0 {
		traceBrakeReplay(ctx, "typed refusal or warning shape", nil)
		return nil, nil //nolint:nilnil // An ineligible retry leaves the original refusal unchanged.
	}
	kind, src, ok, err := brakeBooleanProducer(ctx, d, &target.n, budget)
	if err != nil {
		return nil, err
	}
	if !ok || kind != meshbool.OpCut || len(src) != 2 {
		traceBrakeReplay(ctx, "original Cut producer", nil)
		return nil, nil //nolint:nilnil // An ineligible retry leaves the original refusal unchanged.
	}
	cert, ok := target.n.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok || cert == nil {
		traceBrakeReplay(ctx, "original Cut shift certificate", nil)
		return nil, nil //nolint:nilnil // An ineligible retry leaves the original refusal unchanged.
	}
	if err := budget.SpendWork(ctx, 8); err != nil {
		return nil, err
	}
	d.n.LiveMu.RLock()
	producer := d.producers[c]
	if producer == nil || producer.doc != &d.n || producer.kind != OperationTransform || len(producer.inputs) != 1 ||
		producer.inputs[0].Role != operationRoleSource || producer.inputs[0].Use != InputConsumed || producer.inputs[0].Body == nil ||
		len(producer.outputs) != 1 || producer.outputs[0] != c {
		d.n.LiveMu.RUnlock()
		traceBrakeReplay(ctx, "clevis Transform producer", nil)
		return nil, nil //nolint:nilnil // An ineligible retry leaves the original refusal unchanged.
	}
	c0 := &producer.inputs[0].Body.n
	d.n.LiveMu.RUnlock()
	if c0.Doc != &d.n || c.n.Doc != &d.n || c0.Payload == nil || c.n.Payload == nil || len(c0.OperationWarnings) > 1 ||
		!c0.Payload.Transform().IsValid() || !c.n.Payload.Transform().IsValid() {
		traceBrakeReplay(ctx, "clevis placement shape", nil)
		return nil, nil //nolint:nilnil // An ineligible retry leaves the original refusal unchanged.
	}
	payload, ok := c0.Payload.(facetedrecord.Payload)
	if !ok || payload.PointCone != nil {
		traceBrakeReplay(ctx, "clevis payload class", nil)
		return nil, nil //nolint:nilnil // An ineligible retry leaves the original refusal unchanged.
	}
	var originalGrowth *booleaneval.ShiftCertificate
	var originalWarning opdiagnostic.Record
	var clevis []*topo.Body
	if len(c0.OperationWarnings) == 0 {
		kind, clevis, ok, err = brakeExpandable(ctx, d, c0, budget)
	} else {
		kind, clevis, ok, err = brakeBooleanProducer(ctx, d, c0, budget)
		originalGrowth, _ = c0.PrivateShift.(*booleaneval.ShiftCertificate)
		originalWarning = c0.OperationWarnings[0]
		ok = ok && originalGrowth != nil
	}
	if err != nil {
		return nil, err
	}
	if !ok || kind != meshbool.OpCut || len(clevis) != 2 {
		traceBrakeReplay(ctx, "clevis Cut producer or growth certificate", nil)
		return nil, nil //nolint:nilnil // An ineligible retry leaves the original refusal unchanged.
	}
	privateCtx, err := d.evalContext(ctx)
	if err != nil {
		return nil, err
	}
	if chaintol.Of(privateCtx) != 0 {
		traceBrakeReplay(ctx, "caller chain allowance", nil)
		return nil, nil //nolint:nilnil // An ineligible retry leaves the original refusal unchanged.
	}
	var growthSnapshot *booleaneval.ShiftCertificateSnapshot
	if originalGrowth != nil {
		growthSnapshot, err = booleaneval.SnapshotShiftCertificate(privateCtx, budget, originalGrowth)
		if err != nil {
			return nil, err
		}
		if _, err := booleaneval.AuthenticateStoredGrowthSnapshot(privateCtx, growthSnapshot, c0, clevis[0], clevis[1], originalWarning, budget); err != nil {
			traceBrakeReplay(ctx, "original clevis growth authentication", err)
			return nil, err
		}
	}
	snapshot, err := booleaneval.SnapshotShiftCertificate(privateCtx, budget, cert)
	if err != nil {
		return nil, err
	}
	fixed, err := booleaneval.AuthenticateStoredCutSnapshot(privateCtx, snapshot, &target.n, src[0], src[1], target.n.OperationWarnings[0], budget)
	if err != nil {
		traceBrakeReplay(ctx, "original Cut shift authentication", err)
		return nil, err
	}
	retained, err := booleaneval.RetainedCutTargetSnapshot(privateCtx, snapshot, &target.n, src[0], src[1], budget)
	if err != nil {
		traceBrakeReplay(ctx, "original Cut target authentication", err)
		return nil, err
	}

	placed, err := payload.PlacedReplay(privateCtx, envOf(documentNode(d), c.n.Origin.Producer), c.n.Payload.Transform(), budget)
	if err != nil {
		traceBrakeReplay(ctx, "clevis placement replay", err)
		return nil, err
	}
	equal, err := brakeBodiesEqual(privateCtx, placed, &c.n, budget)
	if err != nil || !equal {
		traceBrakeReplay(ctx, "clevis placement comparison", err)
		return nil, err
	}
	if err := reserveBrakeReplayInputs(privateCtx, budget); err != nil {
		return nil, err
	}
	return &brakeReplayInputs{ctx: privateCtx, a: src[0], b: src[1], blank: clevis[0], u: clevis[1],
		c0: c0, c: &c.n, placed: &c.n, sourceResult: &target.n, cutSnapshot: snapshot, fixed: fixed, retained: retained, bWarning: target.n.OperationWarnings[0], c0Growth: originalGrowth, c0Snapshot: growthSnapshot, c0Warning: originalWarning}, nil
}

// brakeReplayClevis rebuilds the authenticated clevis and checks its growth.
func brakeReplayClevis(d *Document, in *brakeReplayInputs, ref producerID, hooks booleandispatch.Hooks,
	budget *proofbound.ReplayBudget) (*topo.Body, float64, []opdiagnostic.Record, error) {
	if d == nil || in == nil || in.blank == nil || in.u == nil || in.c0 == nil || in.c == nil || in.placed == nil || budget == nil ||
		len(in.c.OperationWarnings) != 0 || len(in.placed.OperationWarnings) != 0 {
		return nil, 0, nil, errBrakeRecordedClevisIsNotAuthenticated
	}
	var kind meshbool.OperationKind
	var operands []*topo.Body
	var ok bool
	var err error
	if in.c0Growth == nil {
		kind, operands, ok, err = brakeExpandable(in.ctx, d, in.c0, budget)
	} else {
		kind, operands, ok, err = brakeBooleanProducer(in.ctx, d, in.c0, budget)
		ok = ok && in.c0.PrivateShift == in.c0Growth && len(in.c0.OperationWarnings) == 1 && in.c0.OperationWarnings[0] == in.c0Warning
	}
	if err != nil {
		return nil, 0, nil, err
	}
	if !ok || kind != meshbool.OpCut || len(operands) != 2 || operands[0] != in.blank || operands[1] != in.u {
		return nil, 0, nil, errBrakeClevisSourceIdentityChanged
	}
	if in.c0Growth != nil {
		if _, err := booleaneval.AuthenticateStoredGrowthSnapshot(in.ctx, in.c0Snapshot, in.c0, in.blank, in.u, in.c0Warning, budget); err != nil {
			return nil, 0, nil, err
		}
	}

	if err := budget.ReserveRecords(in.ctx, uint64(len(in.u.OperationWarnings))); err != nil {
		return nil, 0, nil, err
	}
	if err := budget.ReserveArray(in.ctx, uint64(len(in.u.OperationWarnings)), uint64(unsafe.Sizeof(opdiagnostic.Record{}))); err != nil {
		return nil, 0, nil, err
	}
	beforeWarnings := append([]opdiagnostic.Record(nil), in.u.OperationWarnings...)
	for i := range beforeWarnings {
		if beforeWarnings[i].ChainAllowance != nil {
			if err := budget.ReserveRecords(in.ctx, 1); err != nil {
				return nil, 0, nil, err
			}
			if err := budget.ReserveArray(in.ctx, 1, uint64(unsafe.Sizeof(float64(0)))); err != nil {
				return nil, 0, nil, err
			}
			value := *beforeWarnings[i].ChainAllowance
			beforeWarnings[i].ChainAllowance = &value
		}
	}
	beforePayload, ok := in.u.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, 0, nil, errBrakeClevisToolPayloadIsNotFaceted
	}
	toolSnapshot, err := snapshotBrakeTool(in.ctx, beforePayload, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	beforeOrigin := in.u.Origin
	rebuilt, err := booleandispatch.BodyBrakeClevisCut(in.ctx, envOf(documentNode(d), ref), in.blank, in.u, hooks, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if in.u.Origin != beforeOrigin {
		return nil, 0, nil, errBrakeClevisToolProducerChanged
	}
	afterPayload, ok := in.u.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, 0, nil, errBrakeClevisToolPayloadChanged
	}
	same, err := toolSnapshot.unchanged(in.ctx, afterPayload, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if !same {
		return nil, 0, nil, errBrakeClevisToolPayloadChanged
	}
	same, err = brakeCompare(in.ctx, beforeWarnings, in.u.OperationWarnings, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if !same {
		return nil, 0, nil, errBrakeClevisToolWarningsChanged
	}
	cert, ok := rebuilt.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok {
		return nil, 0, nil, errBrakeClevisGrowthEvidenceMissing
	}
	growth, err := cert.AuthenticatePrivateShift(in.ctx, in.blank, in.u, meshbool.OpCut, rebuilt.OperationWarnings, true, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	_, _, expandable, err := brakeExpandable(in.ctx, d, rebuilt, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if expandable {
		return nil, 0, nil, errBrakePrivateClevisIsExpandable
	}
	payload, ok := rebuilt.Payload.(facetedrecord.Payload)
	if !ok || payload.PointCone != nil {
		return nil, 0, nil, errBrakeRebuiltClevisPayloadChanged
	}
	placed, err := payload.PlacedReplay(in.ctx, envOf(documentNode(d), ref), in.c.Payload.Transform(), budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if len(placed.OperationWarnings) != 0 {
		return nil, 0, nil, errBrakeClevisPlacementWarned
	}
	return placed, growth, rebuilt.OperationWarnings, nil
}

func brakeReplayUnion(d *Document, in *brakeReplayInputs, placed *topo.Body, growth float64,
	ref producerID, hooks booleandispatch.Hooks, budget *proofbound.ReplayBudget) (*topo.Body, float64, []opdiagnostic.Record, error) {
	readChord := func(a, b *topo.Body) (float64, error) {
		x, _, err := booloperand.PairChordReplay(in.ctx, a, b, budget)
		if err != nil {
			return 0, err
		}
		y, _, err := booloperand.PairChordReplay(in.ctx, a, b, budget)
		if err != nil {
			return 0, err
		}
		if !(x > 0) || proofbound.IsNonFinite(x) || math.Float64bits(x) != math.Float64bits(y) {
			return 0, errBrakeUnstableReplayChord
		}
		return x, nil
	}
	original, err := readChord(in.a, in.c)
	if err != nil {
		return nil, 0, nil, err
	}
	candidate, err := readChord(in.a, placed)
	if err != nil {
		return nil, 0, nil, err
	}
	floor, restating := booloperand.HeldFloor(placed)
	if !restating || !(floor <= candidate) {
		return nil, 0, nil, errBrakeRecordedClevisExceedsUnionChord
	}
	mesh, err := tessdispatch.FacetedReplay(in.ctx, placed, max(candidate, floor), budget)
	if err != nil {
		return nil, 0, nil, err
	}
	bBox, err := in.fixed.Box(in.ctx, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	cBox, err := booleaneval.ReplayBodyBoxes(in.ctx, placed, mesh, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if err := budget.SpendWork(in.ctx, 8192); err != nil {
		return nil, 0, nil, err
	}
	separated, err := booleaneval.ReplayBoxesDisjointBounded(in.ctx, bBox, cBox, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if !separated {
		return nil, 0, nil, errBrakeShiftedToolAndClevisAreNotStrictlySeparated
	}
	scale := 1.0
	if candidate > original {
		scale = math.Nextafter(original/candidate, 0)
	}
	if !(scale > 0) || scale > 1 || proofbound.IsNonFinite(scale) || candidate*scale > original {
		return nil, 0, nil, errBrakeInvalidReplayChordClamp
	}
	joined, err := booleandispatch.BodyBrakeRetainedUnion(in.ctx, envOf(documentNode(d), ref), in.a, placed,
		hooks, scale, candidate, original, in.retained, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	cert, ok := joined.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok {
		unshifted, err := brakeUnionUnshifted(in.ctx, joined, budget)
		if err != nil {
			return nil, 0, nil, err
		}
		if !unshifted {
			return nil, 0, nil, errBrakeClevisUnionShiftMissing
		}
		payload, ok := joined.Payload.(facetedrecord.Payload)
		if !ok {
			return nil, 0, nil, errBrakeUnshiftedUnionPayloadChanged
		}
		verified, err := tessdispatch.FacetedReplay(in.ctx, joined, max(1, payload.MeshBound), budget)
		if err != nil {
			return nil, 0, nil, err
		}
		if verified == nil || !verified.BoundaryOK || !verified.SymDiffOK {
			return nil, 0, nil, errBrakeUnshiftedUnionProofMissing
		}
		if proofbound.IsNonFinite(growth) || growth < 0 || growth > gaptol.Reach(in.ctx) {
			return nil, 0, nil, errBrakeCumulativeClevisMovementExceedsReach
		}
		return joined, growth, joined.OperationWarnings, nil
	}
	shift, err := cert.AuthenticatePrivateShift(in.ctx, in.a, placed, meshbool.OpUnion, joined.OperationWarnings, false, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	combined := proofbound.AbsSumUpper(growth, shift)
	if proofbound.IsNonFinite(combined) || combined > gaptol.Reach(in.ctx) {
		return nil, 0, nil, errBrakeCumulativeClevisMovementExceedsReach
	}
	shiftedBox, err := cert.ShiftedBoxes(in.ctx, placed, shift, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if err := budget.SpendWork(in.ctx, 8192); err != nil {
		return nil, 0, nil, err
	}
	separated, err = booleaneval.ReplayBoxesDisjointBounded(in.ctx, bBox, shiftedBox, budget)
	if err != nil {
		return nil, 0, nil, err
	}
	if !separated {
		return nil, 0, nil, errBrakeAdjustedToolAndClevisAreNotStrictlySeparated
	}
	return joined, combined, joined.OperationWarnings, nil
}

func brakeUnionUnshifted(ctx context.Context, body *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	if budget == nil {
		return false, errBrakeMissingUnionReplayBudget
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if body == nil || body.PrivateShift != nil || len(body.OperationWarnings) > 8 {
		return false, nil
	}
	_, ok := body.Payload.(facetedrecord.Payload)
	// This payload has no Union shrink record on this branch.
	if !ok {
		return false, nil
	}
	for _, w := range body.OperationWarnings {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		switch w.Code {
		case opdiagnostic.CodeChainAllowanceApplied:
			if w.Change != opdiagnostic.ChangeUnknown || w.DisplacementBound != 0 || w.AllowedReach != 0 {
				return false, nil
			}
		case opdiagnostic.CodeStitchMergeApplied:
			if w.Change != opdiagnostic.ChangeStitchMerge {
				return false, nil
			}
		case opdiagnostic.CodeEmbeddingReachApplied:
			if w.Change != opdiagnostic.ChangeEmbeddingReach {
				return false, nil
			}
		default:
			return false, nil
		}
	}
	return true, nil
}

func brakeReplayFinal(d *Document, in *brakeReplayInputs, joined *topo.Body, movement float64,
	growth, third []opdiagnostic.Record, ref producerID, hooks booleandispatch.Hooks,
	budget *proofbound.ReplayBudget) (*topo.Body, error) {
	base, _, err := booloperand.PairChordReplay(in.ctx, joined, in.b, budget)
	if err != nil {
		return nil, err
	}
	again, _, err := booloperand.PairChordReplay(in.ctx, joined, in.b, budget)
	if err != nil {
		return nil, err
	}
	if !(base > 0) || proofbound.IsNonFinite(base) || math.Float64bits(base) != math.Float64bits(again) {
		return nil, errBrakeUnstableFinalCutChord
	}
	allowances := [2]float64{proofbound.AbsSumUpper(base, movement), 0}
	var ok bool
	allowances[1], ok = in.fixed.Allowance(base)
	if !ok || proofbound.IsNonFinite(allowances[0]) {
		return nil, errBrakeInvalidFinalCutGates
	}
	final, err := booleandispatch.BodyBrakeFinalCut(in.ctx, envOf(documentNode(d), ref), joined, in.b,
		in.fixed, movement, hooks, budget)
	if err != nil {
		return nil, err
	}
	if err := booleaneval.RevalidateStoredWarning(in.ctx, in.cutSnapshot, in.sourceResult, in.a, in.b, in.bWarning, budget); err != nil {
		return nil, err
	}
	limit := 1 + len(growth) + len(third) + len(final.OperationWarnings)
	summaries, err := opdiagnostic.NewReplayUnionSummaries(in.ctx, budget, limit)
	if err != nil {
		return nil, err
	}
	first := in.bWarning
	first.Operand = 0
	if err := summaries.AddPair(in.ctx, []opdiagnostic.Record{first}, 1); err != nil {
		return nil, err
	}
	if err := summaries.AddPair(in.ctx, growth, 1); err != nil {
		return nil, err
	}
	if err := summaries.AddPair(in.ctx, third, 1); err != nil {
		return nil, err
	}
	var used [2]bool
	for _, w := range final.OperationWarnings {
		if err := brakeCheckFinalChain(in.ctx, w, base, allowances, &used, budget); err != nil {
			return nil, err
		}
		w.Operand = 1 - w.Operand
		if err := summaries.AddPair(in.ctx, []opdiagnostic.Record{w}, 1); err != nil {
			return nil, err
		}
	}
	records, err := summaries.Records(in.ctx)
	if err != nil {
		return nil, err
	}
	var shifts [2]bool
	for i := range records {
		w := &records[i]
		if w.Code == opdiagnostic.CodeChainAllowanceApplied {
			w.AutomaticAllowance = true
		}
		if w.Code == opdiagnostic.CodeGapShiftApplied && w.Operand >= 0 && w.Operand <= 1 {
			shifts[w.Operand] = true
			if w.Operand == 1 {
				if w.DisplacementBound > gaptol.Reach(in.ctx) {
					return nil, errBrakePublicMovementExceedsReach
				}
				w.AllowedReach = gaptol.Reach(in.ctx)
			}
		}
	}
	if !shifts[0] || !shifts[1] {
		return nil, errBrakePublicShiftSummariesMissing
	}
	floor, _ := booloperand.HeldFloor(final)
	mesh, err := tessdispatch.FacetedReplay(in.ctx, final, max(base, floor), budget)
	if err != nil {
		return nil, err
	}
	if !mesh.BoundaryOK || !mesh.SymDiffOK {
		return nil, errBrakeFinalVerificationFailed
	}
	final.OperationWarnings = records
	return final, nil
}

func brakeCheckFinalChain(ctx context.Context, w opdiagnostic.Record, base float64, allowances [2]float64,
	used *[2]bool, budget *proofbound.ReplayBudget) error {
	if budget == nil || used == nil {
		return errBrakeFinalCutDiagnosticStateMissing
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	if w.Code != opdiagnostic.CodeChainAllowanceApplied || w.Operation != int(meshbool.OpCut) ||
		w.Operand < 0 || w.Operand > 1 || w.ChainAllowance == nil || used[w.Operand] ||
		math.Float64bits(w.PairChordLimit) != math.Float64bits(base) ||
		math.Float64bits(*w.ChainAllowance) != math.Float64bits(allowances[w.Operand]) ||
		!(w.TouchedBound > base) || w.TouchedBound > allowances[w.Operand] {
		return errBrakeFinalCutDiagnosticChanged
	}
	used[w.Operand] = true
	return nil
}

// brakeRetryFailure preserves the caller's refusal unless cancellation or an invariant must surface.
func brakeRetryFailure(ctx context.Context, err error) (*Body, producerID, bool, bool, error) {
	if ctxErr := ctx.Err(); ctxErr != nil {
		return nil, 0, true, false, ctxErr
	}
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) || regroupSurfaces(err) {
		return nil, 0, true, false, err
	}
	return nil, 0, true, false, nil
}
