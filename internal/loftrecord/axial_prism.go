package loftrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ExactAxialPrism is a sufficient source certificate. Equal recorded
// line sections paired at zero offset make every ruled wall a vertical plane.
// SharedAxisOf proves the cap origins differ only along that same axis using
// exact rational arithmetic; the rounded sweep level carries its error.
func ExactAxialPrism(ctx context.Context, pl Payload) (prismrecord.Payload, bool, error) {
	if !pl.AuthenticatedReconstruction || pl.SourceIdentity == nil || pl.SurfaceResult ||
		pl.ThroughBoreCut || pl.AnalyticBore || pl.ConstructionDelta != 0 ||
		len(pl.BlendSegs) != 0 || len(pl.ConstructionArcUpper) != 0 ||
		len(pl.ConstructionLengthExtra) != 0 || pl.SectionDelta != 0 || pl.Chorded ||
		len(pl.Profile0.Holes) != 0 || len(pl.Profile1.Holes) != 0 ||
		pl.Frame0.U() != pl.Frame1.U() || pl.Frame0.V() != pl.Frame1.V() {
		return prismrecord.Payload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, offset := range pl.Alignment {
		if err := budget.Step(); err != nil {
			return prismrecord.Payload{}, false, err
		}
		if offset != 0 {
			return prismrecord.Payload{}, false, nil
		}
	}
	if len(pl.Profile0.Outer.Segments) < 3 {
		return prismrecord.Payload{}, false, nil
	}
	for _, segment := range pl.Profile0.Outer.Segments {
		if err := budget.Step(); err != nil {
			return prismrecord.Payload{}, false, err
		}
		line, ok := segment.(momentinput.LineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) ||
			(line.TStart == 1 && line.TEnd == 0)) {
			return prismrecord.Payload{}, false, nil
		}
	}
	same, err := momentinput.ExactProfileEqual(budget, pl.Profile0, pl.Profile1)
	if err != nil || !same {
		return prismrecord.Payload{}, false, err
	}
	axis := prismplacement.SharedAxisOf(
		prismplacement.Operand{Frame: pl.Frame0, Xform: pl.Xform},
		prismplacement.Operand{Frame: pl.Frame1, Xform: pl.Xform})
	if !axis.OK || axis.Shift.Sign() <= 0 {
		return prismrecord.Payload{}, false, nil
	}
	level, _ := axis.Shift.Float64()
	if math.IsInf(level, 0) || math.IsNaN(level) || level <= 0 {
		return prismrecord.Payload{}, false, nil
	}
	delta := proofarith.RationalFloatError(axis.Shift, level)
	if proofbound.IsNonFinite(delta) {
		return prismrecord.Payload{}, false, nil
	}
	return prismrecord.Payload{
		Profile: pl.Profile0, Frame: pl.Frame0, Xform: pl.Xform,
		Z0: 0, Z1: level, Z1Delta: delta,
	}, true, nil
}
