package booleaneval

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type authenticatedGrowthKey struct{}

// AuthenticatedGrowth binds one original growth adjustment to the private replay.
type AuthenticatedGrowth struct {
	cert                            *ShiftCertificate
	snapshot                        *ShiftCertificateSnapshot
	result, target, tool, slot, pin *topo.Body
	warning                         opdiagnostic.Record
	budget                          *proofbound.ReplayBudget
	selected, decision, charged     *topo.Mesh
	growBits, reachBits             uint64
	slotID, pinID                   provenance.Producer
	used                            bool
}

// AuthenticateBrakeGrowth replays and seals the original Cut adjustment.
func AuthenticateBrakeGrowth(ctx context.Context, snapshot *ShiftCertificateSnapshot, result, target, tool, slot, pin *topo.Body, warning opdiagnostic.Record, b *proofbound.ReplayBudget) (*AuthenticatedGrowth, error) {
	cert, err := snapshot.Certificate(ctx, b, result, target, tool)
	if err != nil {
		return nil, err
	}
	if cert == nil || result == nil || target == nil || tool == nil || slot == nil || pin == nil || slot == pin || b == nil || !b.IsBrakeReplay() ||
		cert.selected == nil || cert.decision == nil || cert.charged == nil || cert.component.count != 1 || cert.shift.Move != (r3.Vec{}) || !(cert.shift.Grow > 0) ||
		proofbound.IsNonFinite(cert.shift.Grow) || !(cert.reach > 0) || proofbound.IsNonFinite(cert.reach) ||
		cert.shift.Grow > cert.reach || math.Float64bits(gaptol.Reach(ctx)) != math.Float64bits(cert.reach) ||
		len(result.OperationWarnings) != 1 || !sameShiftWarning(result.OperationWarnings[0], warning) {
		return nil, errBrakeEvalOriginalGrowthEvidenceUnavailable
	}
	validStep := false
	for _, step := range shiftSteps {
		if math.Float64bits(step*cert.reach) == math.Float64bits(cert.shift.Grow) {
			validStep = true
			break
		}
	}
	if !validStep {
		return nil, errBrakeEvalOriginalGrowthIsNotAFixedStep
	}
	if _, err := authenticateStoredGrowthSnapshotCopy(ctx, cert, result, target, tool, warning, b); err != nil {
		return nil, err
	}
	selected, err := cloneShiftMeshReplay(ctx, cert.selected, b)
	if err != nil {
		return nil, err
	}
	decision, err := cloneShiftMeshReplay(ctx, cert.decision, b)
	if err != nil {
		return nil, err
	}
	charged, err := cloneShiftMeshReplay(ctx, cert.charged, b)
	if err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(AuthenticatedGrowth{}))); err != nil {
		return nil, err
	}
	return &AuthenticatedGrowth{cert: cert, snapshot: snapshot, result: result, target: target, tool: tool, slot: slot, pin: pin, warning: warning, budget: b,
		selected: selected, decision: decision, charged: charged, growBits: math.Float64bits(cert.shift.Grow), reachBits: math.Float64bits(cert.reach), slotID: slot.Origin.Producer, pinID: pin.Origin.Producer}, nil
}

func (g *AuthenticatedGrowth) validate(ctx context.Context, target *topo.Body, b *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if g == nil || b == nil || g.budget != b || g.cert == nil || g.result == nil || g.target == nil || g.tool == nil || g.slot == nil || g.pin == nil ||
		g.slot.Origin.Producer != g.slotID || g.pin.Origin.Producer != g.pinID ||
		target != g.target || !g.cert.sealed || g.result.Origin.Producer != g.cert.resultID ||
		g.cert.target != g.target || g.cert.tool != g.tool || g.target.Origin.Producer != g.cert.targetID ||
		g.tool.Origin.Producer != g.cert.toolID || len(g.result.OperationWarnings) != 1 ||
		!sameShiftWarning(g.warning, g.result.OperationWarnings[0]) || !sameShiftWarning(g.warning, g.cert.publicWarning) ||
		g.cert.component.count != 1 || g.cert.shift.Move != (r3.Vec{}) || math.Float64bits(g.cert.shift.Grow) != g.growBits ||
		math.Float64bits(g.cert.reach) != g.reachBits || math.Float64bits(gaptol.Reach(ctx)) != g.reachBits {
		return errBrakeEvalOriginalGrowthBindingChanged
	}
	if err := g.snapshot.Revalidate(ctx, b, g.result, g.target, g.tool); err != nil {
		return err
	}
	validStep := false
	for _, step := range shiftSteps {
		if math.Float64bits(step*g.cert.reach) == g.growBits {
			validStep = true
			break
		}
	}
	if !validStep || !(g.cert.shift.Grow > 0) || proofbound.IsNonFinite(g.cert.shift.Grow) || g.cert.shift.Grow > g.cert.reach {
		return errBrakeEvalOriginalGrowthAmountChanged
	}
	for _, pair := range [][2]*topo.Mesh{{g.cert.selected, g.selected}, {g.cert.decision, g.decision}, {g.cert.charged, g.charged}} {
		if pair[0] == nil || pair[1] == nil {
			return errBrakeEvalOriginalGrowthMeshMissing
		}
		if err := b.SpendWork(ctx, uint64(len(pair[0].Vertices)+len(pair[0].Triangles)+len(pair[0].VertexBound)+len(pair[0].Source)+len(pair[0].FaceBound))); err != nil {
			return err
		}
		same, err := sameShiftMeshReplay(ctx, pair[0], pair[1], b)
		if err != nil {
			return err
		}
		if !same {
			return errBrakeEvalOriginalGrowthMeshChanged
		}
	}
	return nil
}

// WithBrakeAuthenticatedGrowth carries the opaque proof into the refined Cut.
func WithBrakeAuthenticatedGrowth(ctx context.Context, growth *AuthenticatedGrowth) context.Context {
	return context.WithValue(ctx, authenticatedGrowthKey{}, growth)
}
