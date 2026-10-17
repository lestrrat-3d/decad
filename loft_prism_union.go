package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tryAxialLoftPrismUnion admits a Loft only when its authenticated source
// denotes an exact straight prism. The prism arranger then certifies every
// planar crossing and seam through sketch's exact fragments. This first
// subcase has no segment-count, radial-placement, or source-identity gate.
// A varying or curved Loft remains on the existing paths until its own wall
// intersection and source-boundary proofs are implemented.
func tryAxialLoftPrismUnion(ctx context.Context, op meshbool.OperationKind, a, b *Body) (prismPayload, bool, error) {
	if op != meshbool.OpUnion {
		return prismPayload{}, false, nil
	}
	pa, aPrism := a.n.Payload.(prismPayload)
	pb, bPrism := b.n.Payload.(prismPayload)
	la, aLoft := a.n.Payload.(loftPayload)
	lb, bLoft := b.n.Payload.(loftPayload)
	if aPrism && bLoft {
		var ok bool
		var err error
		pb, ok, err = exactAxialLoftPrism(ctx, lb)
		if err != nil || !ok {
			return prismPayload{}, false, err
		}
	} else if aLoft && bPrism {
		var ok bool
		var err error
		pa, ok, err = exactAxialLoftPrism(ctx, la)
		if err != nil || !ok {
			return prismPayload{}, false, err
		}
	} else {
		return prismPayload{}, false, nil
	}
	left := bornBody(&Body{n: topo.Body{Payload: pa}})
	right := bornBody(&Body{n: topo.Body{Payload: pb}})
	// A shared edge with no interior overlap is a tangent contact. Require
	// the same arranger to resolve a positive-area intersection before
	// asking it to merge the two outlines.
	_, overlap, err := tryPrismBoolean(ctx, meshbool.OpIntersect, left, right)
	if err != nil || !overlap {
		return prismPayload{}, false, err
	}
	return tryPrismBoolean(ctx, op, left, right)
}

// exactAxialLoftPrism is a sufficient source certificate. Equal recorded
// line sections paired at zero offset make every ruled wall a vertical plane.
// SharedAxisOf proves the cap origins differ only along that same axis using
// exact rational arithmetic; the rounded sweep level carries its error.
func exactAxialLoftPrism(ctx context.Context, pl loftPayload) (prismPayload, bool, error) {
	if !pl.AuthenticatedReconstruction || pl.SourceIdentity == nil || pl.SurfaceResult ||
		pl.ThroughBoreCut || pl.AnalyticBore || pl.ConstructionDelta != 0 ||
		len(pl.BlendSegs) != 0 || len(pl.ConstructionArcUpper) != 0 ||
		len(pl.ConstructionLengthExtra) != 0 || pl.SectionDelta != 0 || pl.Chorded ||
		len(pl.Profile0.Holes) != 0 || len(pl.Profile1.Holes) != 0 ||
		pl.Frame0.U() != pl.Frame1.U() || pl.Frame0.V() != pl.Frame1.V() {
		return prismPayload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, offset := range pl.Alignment {
		if err := budget.Step(); err != nil {
			return prismPayload{}, false, err
		}
		if offset != 0 {
			return prismPayload{}, false, nil
		}
	}
	if len(pl.Profile0.Outer.Segments) < 3 {
		return prismPayload{}, false, nil
	}
	for _, segment := range pl.Profile0.Outer.Segments {
		if err := budget.Step(); err != nil {
			return prismPayload{}, false, err
		}
		line, ok := segment.(momentinput.LineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) ||
			(line.TStart == 1 && line.TEnd == 0)) {
			return prismPayload{}, false, nil
		}
	}
	same, err := momentinput.ExactProfileEqual(budget, pl.Profile0, pl.Profile1)
	if err != nil || !same {
		return prismPayload{}, false, err
	}
	axis := prismplacement.SharedAxisOf(
		prismplacement.Operand{Frame: pl.Frame0, Xform: pl.Xform},
		prismplacement.Operand{Frame: pl.Frame1, Xform: pl.Xform})
	if !axis.OK || axis.Shift.Sign() <= 0 {
		return prismPayload{}, false, nil
	}
	level, _ := axis.Shift.Float64()
	if math.IsInf(level, 0) || math.IsNaN(level) || level <= 0 {
		return prismPayload{}, false, nil
	}
	delta := proofarith.RationalFloatError(axis.Shift, level)
	if proofbound.IsNonFinite(delta) {
		return prismPayload{}, false, nil
	}
	return prismPayload{
		Profile: pl.Profile0, Frame: pl.Frame0, Xform: pl.Xform,
		Z0: 0, Z1: level, Z1Delta: delta,
	}, true, nil
}
