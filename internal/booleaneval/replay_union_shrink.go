package booleaneval

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// AuthenticateStoredUnionShrink replays a creation-issued one-component Union
// shrink and returns the adjusted tool as a bounded faceted evaluation. The
// result body and its warning must be the committed output of that Union.
func AuthenticateStoredUnionShrink(ctx context.Context, cert *ShiftCertificate, result, target, tool *topo.Body,
	warning opdiagnostic.Record, budget *proofbound.ReplayBudget, mesher OperandMesher) (Evaluation, error) {
	return authenticateStoredUnionShrink(ctx, cert, cert, result, target, tool, warning, budget, mesher)
}

// AuthenticateStoredUnionShrinkSnapshot uses one private copy of the committed
// witness and revalidates its source certificate before replay.
func AuthenticateStoredUnionShrinkSnapshot(ctx context.Context, snapshot *ShiftCertificateSnapshot,
	result, target, tool *topo.Body, warning opdiagnostic.Record, budget *proofbound.ReplayBudget,
	mesher OperandMesher) (Evaluation, error) {
	return authenticateStoredUnionRadialSnapshot(ctx, snapshot, result, target, tool, warning, budget, mesher, true, true)
}

// AuthenticatePrivateUnionAdjustmentSnapshot replays a freshly sealed private
// Union adjustment. Its warning remains attached to the enclosing public call.
func AuthenticatePrivateUnionAdjustmentSnapshot(ctx context.Context, snapshot *ShiftCertificateSnapshot,
	result, target, tool *topo.Body, warning opdiagnostic.Record, budget *proofbound.ReplayBudget,
	mesher OperandMesher) (Evaluation, error) {
	return authenticateStoredUnionRadialSnapshot(ctx, snapshot, result, target, tool, warning, budget, mesher, false, false)
}

func authenticateStoredUnionRadialSnapshot(ctx context.Context, snapshot *ShiftCertificateSnapshot,
	result, target, tool *topo.Body, warning opdiagnostic.Record, budget *proofbound.ReplayBudget,
	mesher OperandMesher, shrinkOnly, public bool) (Evaluation, error) {
	if snapshot == nil {
		return Evaluation{}, fmt.Errorf("%w: missing Union radial-shift snapshot", decaderr.ErrUnsupported)
	}
	cert, err := snapshot.Certificate(ctx, budget, result, target, tool)
	if err != nil {
		return Evaluation{}, err
	}
	return authenticateStoredUnionRadial(ctx, snapshot.source, cert, result, target, tool, warning, budget,
		mesher, shrinkOnly, public)
}

func authenticateStoredUnionShrink(ctx context.Context, sealed, cert *ShiftCertificate,
	result, target, tool *topo.Body, warning opdiagnostic.Record, budget *proofbound.ReplayBudget,
	mesher OperandMesher) (Evaluation, error) {
	return authenticateStoredUnionRadial(ctx, sealed, cert, result, target, tool, warning, budget, mesher, true, true)
}

func authenticateStoredUnionRadial(ctx context.Context, sealed, cert *ShiftCertificate,
	result, target, tool *topo.Body, warning opdiagnostic.Record, budget *proofbound.ReplayBudget,
	mesher OperandMesher, shrinkOnly, public bool) (Evaluation, error) {
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	if cert == nil || budget == nil || result == nil || target == nil || tool == nil ||
		!cert.sealed || (shrinkOnly && !cert.IsPureShrink()) ||
		(!shrinkOnly && !cert.IsSingleComponentAdjustment()) || sealed == nil || result.PrivateShift != sealed ||
		cert.resultID != result.Origin.Producer || cert.target != target || cert.tool != tool ||
		cert.targetID != target.Origin.Producer || cert.toolID != tool.Origin.Producer ||
		(public && !storedUnionShrinkWarningMatches(result, cert, warning)) ||
		(!public && !privateUnionAdjustmentWarningMatches(result, cert, warning)) {
		return Evaluation{}, fmt.Errorf("%w: stored Union shrink identity or warning changed", decaderr.ErrUnsupported)
	}
	if err := validateShiftCaptureSeal(ctx, cert, budget); err != nil {
		return Evaluation{}, err
	}
	if len(cert.groupFaces) == 0 || len(cert.groupFaces) != len(cert.groups) || len(cert.groups) > 65536 {
		return Evaluation{}, fmt.Errorf("%w: stored Union shrink source faces are unavailable", decaderr.ErrUnsupported)
	}
	if err := budget.ReserveRecords(ctx, uint64(len(cert.groupFaces))); err != nil {
		return Evaluation{}, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(cert.groupFaces)), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return Evaluation{}, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(cert.groupFaces)), uint64(unsafe.Sizeof(int(0))+unsafe.Sizeof(uintptr(0)))); err != nil {
		return Evaluation{}, err
	}
	faceID := make(map[*topo.Face]int, len(cert.groupFaces))
	for i, face := range cert.groupFaces {
		if face == nil || cert.groups[i].Source == nil ||
			cert.groups[i].Source.Producer != tool.Origin.Producer ||
			cert.groups[i].Source.Operand != 1 || cert.groups[i].Source.Ordinal != i {
			return Evaluation{}, fmt.Errorf("%w: stored Union shrink source owner changed", decaderr.ErrUnsupported)
		}
		if _, exists := faceID[face]; exists {
			return Evaluation{}, fmt.Errorf("%w: stored Union shrink source owner is duplicated", decaderr.ErrUnsupported)
		}
		faceID[face] = i
	}
	if err := authenticateUnionRadialGeometry(ctx, cert, target, tool, warning, budget, mesher); err != nil {
		return Evaluation{}, err
	}
	charged, err := cloneShiftMeshReplay(ctx, cert.charged, budget)
	if err != nil {
		return Evaluation{}, err
	}
	if !charged.BoundaryOK || !charged.SymDiffOK || len(charged.Source) != len(charged.Triangles) ||
		len(charged.VertexBound) != len(charged.Vertices) {
		return Evaluation{}, fmt.Errorf("%w: adjusted Union operand has incomplete mesh proofs", decaderr.ErrUnsupported)
	}
	if err := loftmesh.LoftCrossingAuditBounded(budget.WorkBudget(ctx), charged.Vertices, charged.Triangles,
		loftmesh.MaxLoftAuditTriangles, proofbound.MaxFacetPairTestsPerCall); err != nil {
		return Evaluation{}, err
	}
	groups, err := snapshotShiftGroups(ctx, cert.groups, budget)
	if err != nil {
		return Evaluation{}, err
	}
	if err := budget.ReserveRecords(ctx, uint64(len(charged.Triangles))); err != nil {
		return Evaluation{}, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(charged.Triangles)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return Evaluation{}, err
	}
	if err := budget.SpendWork(ctx, uint64(len(charged.Vertices))); err != nil {
		return Evaluation{}, err
	}
	payload := facetedrecord.Payload{
		Verts: charged.Vertices, Tris: charged.Triangles, Src: make([]int, len(charged.Source)),
		Groups: groups, VertexBound: charged.VertexBound,
		MeshBound: charged.Bound, VolSymDiff: charged.VolSymDiff, AreaSlack: charged.AreaSlack,
		DPair:          facetedrecord.SourceDiameterUpper(charged.Vertices, charged.Bound),
		ContactAudited: true, Xform: r3.Identity(),
	}
	for i, owner := range charged.Source {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return Evaluation{}, err
		}
		id, ok := faceID[owner]
		if !ok {
			return Evaluation{}, fmt.Errorf("%w: adjusted Union facet has no original source owner", decaderr.ErrUnsupported)
		}
		payload.Src[i] = id
	}
	audit, admitted, err := facetproof.AuditFacetedMeshReplay(ctx, budget.WorkBudget(ctx), payload.Verts, payload.Tris)
	if err != nil {
		return Evaluation{}, err
	}
	if !admitted {
		return Evaluation{}, fmt.Errorf("%w: adjusted Union operand audit refused", decaderr.ErrUnsupported)
	}
	volume, volumeBound, volumeRat, admitted, err := facetproof.VolumeFromAuditReplay(ctx,
		budget.WorkBudget(ctx), audit, payload.VolSymDiff)
	if err != nil {
		return Evaluation{}, err
	}
	if !admitted {
		return Evaluation{}, fmt.Errorf("%w: adjusted Union operand volume proof refused", decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return Evaluation{}, err
	}
	return Evaluation{
		Payload: payload, Volume: volume, VolumeBound: volumeBound, VolumeRat: volumeRat, Audit: audit,
		SimilarityCenter: cert.center, SimilarityScale: 1 + cert.factor,
		ShiftMove: cert.shift.Move, HasSimilarity: cert.IsSingleComponentAdjustment(),
	}, nil
}

func authenticateUnionRadialGeometry(ctx context.Context, cert *ShiftCertificate, target, tool *topo.Body,
	warning opdiagnostic.Record, budget *proofbound.ReplayBudget, mesher OperandMesher) error {
	if cert == nil || budget == nil || mesher == nil || !cert.IsSingleComponentAdjustment() ||
		warning.Operation != int(meshbool.OpUnion) || warning.Operand != 1 ||
		warning.Code != opdiagnostic.CodeGapShiftApplied || warning.Change != opdiagnostic.ChangeGapShift ||
		math.Float64bits(warning.DisplacementBound) != math.Float64bits(cert.displacement) ||
		math.Float64bits(warning.AllowedReach) != math.Float64bits(cert.reach) {
		return fmt.Errorf("%w: stored Union shrink warning changed", decaderr.ErrUnsupported)
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	tol, dPair, err := booloperand.PairChord(ctx, target, tool)
	if err != nil {
		return err
	}
	pair := tol * cert.scale
	if math.Float64bits(pair) != math.Float64bits(cert.chord) {
		return fmt.Errorf("%w: stored Union shrink pair chord changed", decaderr.ErrUnsupported)
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	chord := operandChord(ctx, tool, pair, dPair, cert.scale)
	selected, _, err := mesher.TessellateOperand(ctx, tool, chord)
	if err != nil {
		return err
	}
	charged, displacement, err := cert.replayExactShift(ctx, target, tool, selected, budget)
	if err != nil {
		return err
	}
	increment := charged.Bound - selected.Bound
	if increment < 0 || proofbound.IsNonFinite(increment) {
		return errShiftAuthShiftedMeshBoundDecreased
	}
	if max(displacement, increment) > cert.reach {
		return errShiftAuthPrivateShiftExceedsItsPermittedReach
	}
	return ctx.Err()
}

func privateUnionAdjustmentWarningMatches(result *topo.Body, cert *ShiftCertificate,
	warning opdiagnostic.Record) bool {
	return result != nil && cert != nil && len(result.OperationWarnings) == 1 &&
		sameShiftWarning(result.OperationWarnings[0], warning) && sameShiftWarning(warning, cert.publicWarning) &&
		warning.Operation == int(meshbool.OpUnion) && warning.PublicOperation == 0 &&
		warning.Code == opdiagnostic.CodeGapShiftApplied && warning.Operand == 1 &&
		warning.Change == opdiagnostic.ChangeGapShift &&
		math.Float64bits(warning.DisplacementBound) == math.Float64bits(cert.displacement) &&
		math.Float64bits(warning.AllowedReach) == math.Float64bits(cert.reach)
}

func storedUnionShrinkWarningMatches(result *topo.Body, cert *ShiftCertificate,
	warning opdiagnostic.Record) bool {
	return result != nil && cert != nil && len(result.OperationWarnings) == 1 &&
		sameShiftWarning(result.OperationWarnings[0], warning) && sameShiftWarning(warning, cert.publicWarning) &&
		warning.Operation == int(meshbool.OpUnion) && warning.Code == opdiagnostic.CodeGapShiftApplied &&
		warning.Operand == 1 && warning.Change == opdiagnostic.ChangeGapShift &&
		math.Float64bits(warning.DisplacementBound) == math.Float64bits(cert.displacement) &&
		math.Float64bits(warning.AllowedReach) == math.Float64bits(cert.reach)
}

// StoredUnionShrinkWarningMatches confirms that the caller still holds the
// exact captured warning on the committed result body.
func StoredUnionShrinkWarningMatches(result *topo.Body, cert *ShiftCertificate,
	warning opdiagnostic.Record) bool {
	return storedUnionShrinkWarningMatches(result, cert, warning)
}

// StoredPrivateUnionAdjustmentWarningMatches confirms the fresh private Union
// warning still matches the captured result and sealed source certificate.
func StoredPrivateUnionAdjustmentWarningMatches(result *topo.Body, snapshot *ShiftCertificateSnapshot,
	warning opdiagnostic.Record) bool {
	return snapshot != nil && privateUnionAdjustmentWarningMatches(result, snapshot.source, warning)
}
