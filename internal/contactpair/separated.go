package contactpair

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarpose"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	vocab "github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ProveSeparated returns true only when the exact planar relation proves a
// positive gap between the true boundaries. Unsupported snapshots and every
// relation except separation return false without an error.
func ProveSeparated(ctx context.Context, mesher verifiedmesh.Mesher, a, b *topo.Body,
	poseA, poseB r3.Transform, req vocab.ContactRequest) (bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return false, err
	}
	if req.SupportBand != (units.Value{}) {
		return false, nil
	}
	chord := req.HeldChord.Base()
	solidA, deltaA, okA, err := planarpose.SolidAtPose(ctx, budget, mesher, a, poseA, chord)
	if err != nil || !okA {
		return false, err
	}
	solidB, deltaB, okB, err := planarpose.SolidAtPose(ctx, budget, mesher, b, poseB, chord)
	if err != nil || !okB {
		return false, err
	}
	delta := proofarith.DyAdd(deltaA, deltaB)
	result, err := planar.ClassifyPlanarWithin(&solidA, &solidB, delta, budget.Step)
	if errors.Is(err, planar.ErrWorkLimit) {
		return false, budget.Err()
	}
	if err != nil {
		return false, err
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	return result.Relation == pair.Separated, nil
}
