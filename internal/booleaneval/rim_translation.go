package booleaneval

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func (c *ShiftCertificate) CommittedRimTranslation(ctx context.Context, result, a, b *topo.Body, kind meshbool.OperationKind, budget *proofbound.ReplayBudget) (r3.Vec, error) {
	if ctx == nil || budget == nil || result == nil || a == nil || b == nil || c == nil || !c.sealed || !c.IsTranslation() || c.component.Count() != 1 || result.PrivateShift != c || result.Origin.Producer != c.resultID || c.target != a || c.tool != b || c.targetID != a.Origin.Producer || c.toolID != b.Origin.Producer || len(result.OperationWarnings) != 1 || !sameShiftWarning(result.OperationWarnings[0], c.publicWarning) {
		return r3.Vec{}, errShiftAuthPrivateShiftEvidenceMissing
	}
	if c.publicWarning.Operation != int(kind) || c.displacement > c.reach {
		return r3.Vec{}, errShiftAuthPrivateShiftWarningChanged
	}
	if err := validateShiftCaptureSeal(ctx, c, budget); err != nil {
		return r3.Vec{}, err
	}
	chord, _, err := booloperand.PairChordReplay(ctx, a, b, budget)
	if err != nil {
		return r3.Vec{}, err
	}
	if math.Float64bits(chord*c.scale) != math.Float64bits(c.chord) {
		return r3.Vec{}, errShiftAuthPrivateShiftPairChordChanged
	}
	original, err := cloneShiftMeshReplay(ctx, c.selected, budget)
	if err != nil {
		return r3.Vec{}, err
	}
	if _, _, err = c.replayTranslation(ctx, a, b, original, budget); err != nil {
		return r3.Vec{}, err
	}
	return c.shift.Move, nil
}
