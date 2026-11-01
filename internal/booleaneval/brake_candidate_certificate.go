package booleaneval

import (
	"context"
	"math"
	"slices"
	"sync/atomic"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

type circleCandidateSeal struct {
	slot, pin                  *topo.Body
	slotID, pinID              provenance.Producer
	budget                     *proofbound.ReplayBudget
	chord, volume, volumeBound float64
	payload                    candidatePayloadSnapshot
}

// BrakeCandidateCertificate binds one verified private Union to one Cut.
type BrakeCandidateCertificate struct {
	target, tool, slot, pin         *topo.Body
	targetID, toolID, slotID, pinID provenance.Producer
	budget                          *proofbound.ReplayBudget
	chord                           float64
	payload                         candidatePayloadSnapshot
	verified                        *topo.Mesh
	volume, volumeBound             float64
	state                           atomic.Uint32
}

// BuildBrakeCircleCandidate builds and verifies one uncommitted quarter-scale tool.
func BuildBrakeCircleCandidate(ctx context.Context, env topo.Env, target, slot, pin *topo.Body,
	b *proofbound.ReplayBudget) (*topo.Body, *BrakeCandidateCertificate, error) {
	ev, err := EvaluateBrakeCircleCandidate(ctx, slot, pin, b)
	if err != nil {
		return nil, nil, err
	}
	volume := measurement.Measurement{Value: units.CubicMillimeters(ev.Volume), Exactness: measurement.ExactnessOf(ev.VolumeBound), Bound: units.CubicMillimeters(ev.VolumeBound)}
	tool, ok, err := facetedrecord.BuildWithProofReplay(ctx, b.WorkBudget(ctx), env, ev.Payload, ev.Audit, volume, ev.VolumeRat)
	if err != nil {
		return nil, nil, err
	}
	if !ok {
		return nil, nil, errBrakeEvalCandidatePublicationRefused
	}
	warnings, err := ev.DiagnosticsReplay(ctx, meshbool.OpUnion, b)
	if err != nil {
		return nil, nil, err
	}
	if len(warnings) != 0 || len(tool.OperationWarnings) != 0 {
		return nil, nil, errBrakeEvalCandidateWarningRequiresAuthentication
	}
	cert, err := issueBrakeCandidateCertificate(ctx, ev, target, tool, slot, pin, ev.circleSeal.chord, b)
	if err != nil {
		return nil, nil, err
	}
	return tool, cert, nil
}

func issueBrakeCandidateCertificate(ctx context.Context, ev Evaluation, target, tool, slot, pin *topo.Body,
	chord float64, b *proofbound.ReplayBudget) (*BrakeCandidateCertificate, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	s := ev.circleSeal
	if s == nil || target == nil || tool == nil || slot == nil || pin == nil || b == nil || !b.IsBrakeReplay() ||
		s.budget != b || s.slot != slot || s.pin != pin || s.slotID != slot.Origin.Producer || s.pinID != pin.Origin.Producer ||
		math.Float64bits(s.chord) != math.Float64bits(chord) || tool.Doc == nil || tool.Doc != slot.Doc || tool.Doc != pin.Doc || target.Doc != tool.Doc ||
		len(tool.OperationWarnings) != 0 || ev.Shift != nil || ev.EmbeddingReach != nil || ev.Chain != nil || len(ev.Chains) != 0 {
		return nil, errBrakeEvalCandidateSourceEvidenceChanged
	}
	payload, ok := tool.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, errBrakeEvalCandidateToolIsNotFaceted
	}
	// Publication computes these two fields; all remaining fields still equal
	// the exact evaluator payload and retain the evaluator's backing arrays.
	original := payload
	original.FaceOf, original.Diameter = ev.Payload.FaceOf, ev.Payload.Diameter
	same, err := s.payload.unchanged(ctx, original, b)
	if err != nil {
		return nil, err
	}
	if !same || math.Float64bits(tool.Volume.Value.Base()) != math.Float64bits(s.volume) ||
		math.Float64bits(tool.Volume.Bound.Base()) != math.Float64bits(s.volumeBound) {
		return nil, errBrakeEvalCandidatePublicationChangedProofFields
	}
	verified, err := tessdispatch.FacetedReplay(ctx, tool, math.Max(1, payload.MeshBound), b)
	if err != nil {
		return nil, err
	}
	if verified == nil || !verified.BoundaryOK || !verified.SymDiffOK {
		return nil, errBrakeEvalCandidateVerificationMissing
	}
	snapshot, err := snapshotCandidatePayload(ctx, payload, b)
	if err != nil {
		return nil, err
	}
	copyMesh, err := cloneShiftMeshReplay(ctx, verified, b)
	if err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(BrakeCandidateCertificate{}))); err != nil {
		return nil, err
	}
	c := &BrakeCandidateCertificate{target: target, tool: tool, slot: slot, pin: pin,
		targetID: target.Origin.Producer, toolID: tool.Origin.Producer, slotID: slot.Origin.Producer, pinID: pin.Origin.Producer,
		budget: b, chord: chord, payload: snapshot, verified: copyMesh, volume: s.volume, volumeBound: s.volumeBound}
	if err := c.validate(ctx, target, tool, b); err != nil {
		return nil, err
	}
	return c, nil
}

func (c *BrakeCandidateCertificate) validate(ctx context.Context, target, tool *topo.Body, b *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if c == nil || b == nil || c.budget != b || target == nil || tool == nil || target != c.target || tool != c.tool ||
		c.slot == nil || c.pin == nil || target.Origin.Producer != c.targetID || tool.Origin.Producer != c.toolID ||
		c.slot.Origin.Producer != c.slotID || c.pin.Origin.Producer != c.pinID || len(tool.OperationWarnings) != 0 ||
		math.Float64bits(tool.Volume.Value.Base()) != math.Float64bits(c.volume) || math.Float64bits(tool.Volume.Bound.Base()) != math.Float64bits(c.volumeBound) {
		return errBrakeEvalCandidateCertificateIdentityChanged
	}
	p, ok := tool.Payload.(facetedrecord.Payload)
	if !ok {
		return errBrakeEvalCandidatePayloadChanged
	}
	same, err := c.payload.unchanged(ctx, p, b)
	if err != nil {
		return err
	}
	if !same {
		return errBrakeEvalCandidatePayloadProofChanged
	}
	tol, diameter, err := booloperand.PairChordReplay(ctx, c.slot, c.pin, b)
	if err != nil {
		return err
	}
	chord, err := operandChordReplay(ctx, c.pin, tol*.25, diameter, .25, b)
	if err != nil {
		return err
	}
	if math.Float64bits(chord) != math.Float64bits(c.chord) {
		return errBrakeEvalCandidateChordChanged
	}
	left, err := facesReplay(ctx, c.slot, b)
	if err != nil {
		return err
	}
	right, err := facesReplay(ctx, c.pin, b)
	if err != nil {
		return err
	}
	for i, g := range p.Groups {
		if err := b.SpendWork(ctx, uint64(len(g.Origins)+len(g.Upstream))+1); err != nil {
			return err
		}
		s := g.Source
		if s == nil || s.Blend || s.Operand < 0 || s.Operand > 1 {
			return errBrakeEvalCandidateSourceMissing
		}
		faces, owner := left, c.slot
		if s.Operand == 1 {
			faces, owner = right, c.pin
		}
		if s.Ordinal < 0 || s.Ordinal >= len(faces) || faces[s.Ordinal] == nil || s.Producer != owner.Origin.Producer ||
			s.Parent != faces[s.Ordinal].Source || !slices.Equal(g.Origins, faces[s.Ordinal].Origins) {
			return newBrakeCandidateGroupError(ctx, b, i)
		}
	}
	mesh, err := tessdispatch.FacetedReplay(ctx, tool, math.Max(1, p.MeshBound), b)
	if err != nil {
		return err
	}
	if err := b.SpendWork(ctx, uint64(len(mesh.Vertices)+len(mesh.Triangles)+len(mesh.VertexBound)+len(mesh.Source))); err != nil {
		return err
	}
	same, err = sameShiftMeshReplay(ctx, mesh, c.verified, b)
	if err != nil {
		return err
	}
	if !same {
		return errBrakeEvalCandidateVerifiedMeshChanged
	}
	return nil
}

// AuthorizeBrakeCandidate authorizes one dispatch after rechecking its evidence.
func AuthorizeBrakeCandidate(ctx context.Context, c *BrakeCandidateCertificate, target, tool *topo.Body, b *proofbound.ReplayBudget) error {
	if c == nil || c.state.Load() != 0 {
		return errBrakeEvalCandidateCertificateAlreadyUsed
	}
	if err := c.validate(ctx, target, tool, b); err != nil {
		return err
	}
	if !c.state.CompareAndSwap(0, 1) {
		return errBrakeEvalCandidateCertificateAlreadyUsed
	}
	return nil
}

// RevalidateBrakeCandidateForPlacement checks the consumed candidate without
// authorizing another Boolean attempt.
func RevalidateBrakeCandidateForPlacement(ctx context.Context, c *BrakeCandidateCertificate, target, tool *topo.Body, b *proofbound.ReplayBudget) error {
	if c == nil || c.state.Load() != 2 {
		return errBrakeEvalConsumedBrakeCandidateIsUnavailable
	}
	return c.validate(ctx, target, tool, b)
}
