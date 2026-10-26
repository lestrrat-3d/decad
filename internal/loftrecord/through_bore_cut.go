package loftrecord

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftfillet"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ThroughBoreCut keeps a matching axial loft as a loft when a whole
// circular through-tool becomes one new hole in each section. The section
// certificate proves the new hole lies inside the existing outer loop.
// Every other pair keeps the general boolean path.
func ThroughBoreCut(ctx context.Context, env topo.Env, a, b topo.Payload) (*topo.Body, bool, error) {
	pl, ok := a.(Payload)
	if !ok || pl.SurfaceResult || len(pl.Profile0.Holes) != 0 ||
		!sectionrecord.IdenticalRecord(pl.Profile0, pl.Profile1) ||
		pl.Frame0.U() != pl.Frame1.U() || pl.Frame0.V() != pl.Frame1.V() ||
		pl.Xform != r3.Identity() {
		return nil, false, nil
	}
	for _, off := range pl.Alignment {
		if off != 0 {
			return nil, false, nil
		}
	}
	tool, ok := b.(prismrecord.Payload)
	if !ok || tool.SurfaceResult || tool.Xform != r3.Identity() ||
		tool.SectionDelta != 0 || tool.Z0Delta != 0 || tool.Z1Delta != 0 ||
		tool.Frame.U() != pl.Frame0.U() || tool.Frame.V() != pl.Frame0.V() ||
		len(tool.Profile.Holes) != 0 || len(tool.Profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	circle, ok := tool.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !((circle.TStart == 0 && circle.TEnd == 1) ||
		(circle.TStart == 1 && circle.TEnd == 0)) {
		return nil, false, nil
	}
	centerWorld := tool.Point(circle.Center.U, circle.Center.V, 0)
	center := pl.Frame0.ToLocal(centerWorld)
	if !loftToolCenterExact(tool, pl.Frame0, circle.Center.U, circle.Center.V, center) {
		return nil, false, nil
	}
	end := pl.Frame0.ToLocal(pl.Frame1.Origin())
	if end.Z <= 0 || !loftToolSpansExactCaps(tool, pl.Frame0, pl.Frame1) {
		return nil, false, nil
	}
	profile := pl.Profile0
	hole := circle
	hole.Center = sectionrecord.Point2{U: center.X, V: center.Y}
	hole.CCW, hole.TStart, hole.TEnd = false, 1, 0
	profile.Holes = []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{hole}}}
	budget := proofbound.NewWorkBudget(ctx)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	pl.RaiseReconstructionLimit(work0, work1)
	limit := pl.RewriteWorkLimit(uint64(len(profile.Outer.Segments) + 1))
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	contained, err := loftfillet.CircleHoleInside(budget, pl.Profile0.Outer, hole, pl.ConstructionDelta, work0)
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
	next.Profile0, next.Profile1 = profile, profile
	next.ThroughBoreCut = true
	next.RecordArea = [2]float64{ig.Area, ig.Area}
	// The analytic wall has no normal-departure allowance. A rounded center
	// or a non-isometric frame map keeps the Loft's tessellated bore topology.
	next.AnalyticBore = BoreHasExactAxis(pl.Frame0, pl.Frame1, hole.Center.U, hole.Center.V, end.Z) &&
		BoreCylinderExactGeometry(pl.Frame0, pl.Xform, hole.Center.U, hole.Center.V)
	out, err := Build(ctx, env, next, budget, work0, work1)
	if err != nil {
		return nil, false, err
	}
	return out, true, nil
}

// The candidate local coordinates are rounded readings. Compare their exact
// lifts with the recorded tool center before reusing them in a hole record.
func loftToolCenterExact(tool prismrecord.Payload, frame r3.Frame, u, v float64, candidate r3.Vec) bool {
	toolCenter, err := massmoment.RevolveAnchor(tool.Frame, u, v)
	if err != nil {
		return false
	}
	loftCenter, err := massmoment.RevolveAnchor(frame, candidate.X, candidate.Y)
	if err != nil {
		return false
	}
	axis, err := massmoment.PlaneMap(frame, r3.Identity())
	if err != nil {
		return false
	}
	z := proofarith.FloatRat(candidate.Z)
	if z == nil {
		return false
	}
	for i := range toolCenter {
		lift := new(big.Rat).Add(loftCenter[i], new(big.Rat).Mul(axis[i][2], z))
		if toolCenter[i].Cmp(lift) != 0 {
			return false
		}
	}
	return true
}

// The tool's exact endpoints must lie on or beyond the cap planes. A dot
// with the lower frame's exact normal decides each side without a rounded
// local-coordinate comparison.
func loftToolSpansExactCaps(tool prismrecord.Payload, lower, upper r3.Frame) bool {
	toolOrigin, err := massmoment.RevolveAnchor(tool.Frame, 0, 0)
	if err != nil {
		return false
	}
	bottom, err := massmoment.RevolveAnchor(lower, 0, 0)
	if err != nil {
		return false
	}
	top, err := massmoment.RevolveAnchor(upper, 0, 0)
	if err != nil {
		return false
	}
	toolMap, err := massmoment.PlaneMap(tool.Frame, r3.Identity())
	if err != nil {
		return false
	}
	loftMap, err := massmoment.PlaneMap(lower, r3.Identity())
	if err != nil {
		return false
	}
	for end, endCap := range [2][3]*big.Rat{bottom, top} {
		z := proofarith.FloatRat(tool.Z0)
		if end == 1 {
			z = proofarith.FloatRat(tool.Z1)
		}
		if z == nil {
			return false
		}
		side := new(big.Rat)
		for i := range toolOrigin {
			point := new(big.Rat).Add(toolOrigin[i], new(big.Rat).Mul(toolMap[i][2], z))
			delta := new(big.Rat).Sub(point, endCap[i])
			side.Add(side, new(big.Rat).Mul(delta, loftMap[i][2]))
		}
		if (end == 0 && side.Sign() > 0) || (end == 1 && side.Sign() < 0) {
			return false
		}
	}
	return true
}
