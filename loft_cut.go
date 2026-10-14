package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftfillet"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// tryLoftThroughBoreCut keeps a matching axial loft as a loft when a whole
// circular through-tool becomes one new hole in each section. The section
// certificate proves the new hole lies inside the existing outer loop.
// Every other pair keeps the general boolean path.
func tryLoftThroughBoreCut(ctx context.Context, d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	pl, ok := a.payload.(loftPayload)
	if !ok || pl.surfaceResult || len(pl.profile0.Holes) != 0 ||
		!sectionrecord.IdenticalRecord(pl.profile0, pl.profile1) ||
		pl.frame0.U() != pl.frame1.U() || pl.frame0.V() != pl.frame1.V() ||
		pl.xform != r3.Identity() {
		return nil, false, nil
	}
	for _, off := range pl.alignment {
		if off != 0 {
			return nil, false, nil
		}
	}
	tool, ok := b.payload.(prismPayload)
	if !ok || tool.SurfaceResult || tool.Xform != r3.Identity() ||
		tool.Frame.U() != pl.frame0.U() || tool.Frame.V() != pl.frame0.V() ||
		len(tool.Profile.Holes) != 0 || len(tool.Profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	circle, ok := tool.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !((circle.TStart == 0 && circle.TEnd == 1) ||
		(circle.TStart == 1 && circle.TEnd == 0)) {
		return nil, false, nil
	}
	centerWorld := tool.Point(circle.Center.U, circle.Center.V, 0)
	center := pl.frame0.ToLocal(centerWorld)
	if pl.frame0.ToWorld(center) != centerWorld {
		return nil, false, nil
	}
	from := pl.frame0.ToLocal(tool.Point(0, 0, tool.Z0))
	to := pl.frame0.ToLocal(tool.Point(0, 0, tool.Z1))
	end := pl.frame0.ToLocal(pl.frame1.Origin())
	if from.Z > 0 || to.Z < end.Z || end.Z <= 0 || end.X != 0 || end.Y != 0 {
		return nil, false, nil
	}
	profile := pl.profile0
	hole := circle
	hole.Center = sectionrecord.Point2{U: center.X, V: center.Y}
	hole.CCW, hole.TStart, hole.TEnd = false, 1, 0
	profile.Holes = []loopRecord{{Segments: []curveSegment{hole}}}
	budget := proofbound.NewWorkBudget(ctx)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	pl.raiseReconstructionLimit(work0, work1)
	limit := pl.rewriteWorkLimit(uint64(len(profile.Outer.Segments) + 1))
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	contained, err := loftfillet.CircleHoleInside(budget, pl.profile0.Outer, hole, pl.constructionDelta, work0)
	if err != nil {
		if ctx.Err() != nil {
			return nil, false, ctx.Err()
		}
		return nil, false, nil
	}
	if !contained {
		return nil, false, nil
	}
	ig, err := profile.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work0)
	if err != nil {
		return nil, false, err
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	next := pl
	next.profile0, next.profile1 = profile, profile
	next.recordArea = [2]float64{ig.Area, ig.Area}
	// The analytic wall has no allowance for a tilt between its section axes.
	// A failed exact proof keeps the Loft's tessellated bore topology.
	next.analyticBore = loftBoreHasExactAxis(pl.frame0, pl.frame1, hole.Center.U, hole.Center.V, end.Z)
	out, err := evalLoft(ctx, d, ref, next, budget, work0, work1)
	if err != nil {
		return nil, false, err
	}
	return out, true, nil
}
