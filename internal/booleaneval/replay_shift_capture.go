package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// captureReplayShift snapshots the accepted private adjustment on its shared meter.
func captureReplayShift(ctx context.Context, a, b *topo.Body, selected, decision, charged *topo.Mesh, pc pieces,
	groups []facetedrecord.Group, shifts *ReplayShifts, chord, scale, displacement float64,
	budget *proofbound.ReplayBudget) (*ShiftCertificate, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if budget == nil || shifts == nil || shifts.budget != budget || a == nil || b == nil || selected == nil || decision == nil || charged == nil || pc.count != 1 || len(shifts.entries) != 1 {
		return nil, decaderr.ErrUnsupported
	}
	shift, ok, err := shifts.get(ctx, 0)
	if err != nil {
		return nil, err
	}
	if !ok || !proofbound.FiniteVec(shift.Move) || proofbound.IsNonFinite(shift.Grow) {
		return nil, decaderr.ErrUnsupported
	}
	if len(selected.Vertices) > 65536 || len(selected.Triangles) > 65536 || len(decision.Vertices) > 65536 || len(decision.Triangles) > 65536 || len(charged.Vertices) > 65536 || len(charged.Triangles) > 65536 {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ShiftCertificate{}))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, 2*uint64(len(selected.Vertices))); err != nil {
		return nil, err
	}
	center, radius, factor := shiftGeometry(selected, pc, shift)
	out := &ShiftCertificate{target: a, tool: b, targetID: a.Origin.Producer, toolID: b.Origin.Producer, shift: shift, center: center, radius: radius, factor: factor, chord: chord, scale: scale, displacement: displacement, reach: gaptol.Reach(ctx)}
	out.selected, err = snapshotShiftMesh(ctx, selected, budget)
	if err != nil {
		return nil, err
	}
	out.decision, err = snapshotShiftMesh(ctx, decision, budget)
	if err != nil {
		return nil, err
	}
	out.charged, err = snapshotShiftMesh(ctx, charged, budget)
	if err != nil {
		return nil, err
	}
	out.component.ofVertex, err = snapshotShiftSlice(ctx, pc.ofVertex, budget)
	if err != nil {
		return nil, err
	}
	out.component.ofFacet, err = snapshotShiftSlice(ctx, pc.ofFacet, budget)
	if err != nil {
		return nil, err
	}
	out.component.count = pc.count
	out.groups, err = snapshotShiftGroups(ctx, groups, budget)
	if err != nil {
		return nil, err
	}
	out.captureSeal, err = captureShiftGroupSeal(ctx, out.groups, budget)
	if err != nil {
		return nil, err
	}
	return out, nil
}
