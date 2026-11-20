package booleaneval

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/boolshift"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestCommittedRimTranslationWarning(t *testing.T) {
	c, r, a, b := shiftSnapshotFixture()
	c.sealed = false
	a.Payload, b.Payload = prismrecord.Payload{}, prismrecord.Payload{}
	a.Bounds.Max = r3.Vec{X: 1, Y: 1, Z: 1}
	b.Bounds.Max = a.Bounds.Max
	meter := proofbound.NewStrictReplayBudget()
	var err error
	c.scale = 1
	c.chord, _, err = booloperand.PairChordReplay(t.Context(), a, b, meter)
	if err != nil {
		t.Fatal(err)
	}
	shifts, err := boolshift.NewReplayShifts(t.Context(), meter)
	if err != nil {
		t.Fatal(err)
	}
	if err = shifts.Set(t.Context(), c.shift); err != nil {
		t.Fatal(err)
	}
	c.decision, c.charged, c.displacement, err = boolshift.ShiftOperandMeshReplay(t.Context(), c.selected, c.component, shifts, nil, meter)
	if err != nil {
		t.Fatal(err)
	}
	c.reach = c.displacement
	w := opdiagnostic.Record{Operation: int(meshbool.OpUnion), PublicOperation: 11, Code: opdiagnostic.CodeGapShiftApplied, Operand: 1, Change: opdiagnostic.ChangeGapShift, DisplacementBound: c.displacement, AllowedReach: c.reach}
	if !c.SealPublicUnionAdjustmentWarning(r.Origin.Producer, []opdiagnostic.Record{w}, 11) {
		t.Fatal("valid translation warning declined")
	}
	r.OperationWarnings = []opdiagnostic.Record{w}
	got, err := c.CommittedRimTranslation(t.Context(), r, a, b, meshbool.OpUnion, proofbound.NewStrictReplayBudget())
	if err != nil || got != c.shift.Move {
		t.Fatalf("translation %v, error %v", got, err)
	}
	r.OperationWarnings[0].AllowedReach += 1
	if _, err := c.CommittedRimTranslation(t.Context(), r, a, b, meshbool.OpUnion, proofbound.NewStrictReplayBudget()); err == nil {
		t.Fatal("changed public warning admitted")
	}
	r.OperationWarnings[0] = w
	if _, err := c.CommittedRimTranslation(t.Context(), r, b, a, meshbool.OpUnion, proofbound.NewStrictReplayBudget()); err == nil {
		t.Fatal("swapped operands admitted")
	}
	if _, err := c.CommittedRimTranslation(t.Context(), r, a, b, meshbool.OpCut, proofbound.NewStrictReplayBudget()); err == nil {
		t.Fatal("different original operation admitted")
	}
}
