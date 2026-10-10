package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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
	if !ok || tool.surfaceResult || tool.xform != r3.Identity() ||
		tool.frame.U() != pl.frame0.U() || tool.frame.V() != pl.frame0.V() ||
		len(tool.profile.Holes) != 0 || len(tool.profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	circle, ok := tool.profile.Outer.Segments[0].(circleSeg)
	if !ok || !((circle.TStart == 0 && circle.TEnd == 1) ||
		(circle.TStart == 1 && circle.TEnd == 0)) {
		return nil, false, nil
	}
	centerWorld := tool.point(circle.Center.U, circle.Center.V, 0)
	center := pl.frame0.ToLocal(centerWorld)
	if pl.frame0.ToWorld(center) != centerWorld {
		return nil, false, nil
	}
	from := pl.frame0.ToLocal(tool.point(0, 0, tool.z0))
	to := pl.frame0.ToLocal(tool.point(0, 0, tool.z1))
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
	limit := loftmesh.StationWorkLimit(0, uint64(len(profile.Outer.Segments)+1))
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	contained, err := certifyLoftCircleHole(budget, pl.profile0.Outer, hole, work0)
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
	out, err := evalLoft(ctx, d, ref, next, budget, work0, work1)
	if err != nil {
		return nil, false, err
	}
	return out, true, nil
}

// certifyLoftCircleHole proves containment without reconstructing every
// free-form carrier in a new sketch scene. Each segment's exact enclosure,
// and the connector between its walked endpoint and the next, must miss the
// circle's containing square. Each curve can then be deformed to its chord
// without crossing the circle. Exact winding of those chords decides whether
// the circle center lies inside the already authenticated outer loop.
func certifyLoftCircleHole(budget *proofbound.WorkBudget, outer loopRecord, hole circleSeg,
	work *freeform.FreeformWork) (bool, error) {
	radius, err := hole.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || len(outer.Segments) < 3 {
		return false, err
	}
	cu, cv := new(big.Rat).SetFloat64(hole.Center.U), new(big.Rat).SetFloat64(hole.Center.V)
	r := new(big.Rat).SetFloat64(radius)
	if cu == nil || cv == nil || r == nil {
		return false, nil
	}
	square := loftSectionBox{
		u: proofbound.Interval(new(big.Rat).Sub(cu, r), new(big.Rat).Add(cu, r)),
		v: proofbound.Interval(new(big.Rat).Sub(cv, r), new(big.Rat).Add(cv, r)),
	}
	type vertex struct{ u, v *big.Rat }
	starts, ends := make([]vertex, len(outer.Segments)), make([]vertex, len(outer.Segments))
	for i, seg := range outer.Segments {
		if err := budget.Step(); err != nil {
			return false, err
		}
		walk, err := boundarywalk.WalkOf(seg, work)
		if err != nil || walk.Closed {
			return false, err
		}
		box, err := loftSectionSegmentBox(seg, walk)
		if err != nil {
			return false, err
		}
		starts[i] = vertex{new(big.Rat).SetFloat64(walk.StartU), new(big.Rat).SetFloat64(walk.StartV)}
		ends[i] = vertex{new(big.Rat).SetFloat64(walk.EndU), new(big.Rat).SetFloat64(walk.EndV)}
		if starts[i].u == nil || starts[i].v == nil || ends[i].u == nil || ends[i].v == nil {
			return false, nil
		}
		box.u.Lo = proofbound.RatMin(proofbound.RatMin(box.u.Lo, starts[i].u), ends[i].u)
		box.u.Hi = proofbound.RatMax(proofbound.RatMax(box.u.Hi, starts[i].u), ends[i].u)
		box.v.Lo = proofbound.RatMin(proofbound.RatMin(box.v.Lo, starts[i].v), ends[i].v)
		box.v.Hi = proofbound.RatMax(proofbound.RatMax(box.v.Hi, starts[i].v), ends[i].v)
		if !loftBoxSeparated(box, square, new(big.Rat)) {
			return false, nil
		}
	}
	inside := false
	for i := range starts {
		if err := budget.Step(); err != nil {
			return false, err
		}
		next := (i + 1) % len(starts)
		gap := loftSectionBox{
			u: loftPointInterval(ends[i].u, starts[next].u),
			v: loftPointInterval(ends[i].v, starts[next].v),
		}
		if !loftBoxSeparated(gap, square, new(big.Rat)) {
			return false, nil
		}
		// The segment chord and the short junction connector are both inside
		// proven circle-free boxes, so this edge has the same winding as the
		// original curve and connector together.
		a, b := starts[i], starts[next]
		up := a.v.Cmp(cv) <= 0 && b.v.Cmp(cv) > 0
		down := b.v.Cmp(cv) <= 0 && a.v.Cmp(cv) > 0
		if !up && !down {
			continue
		}
		cross := new(big.Rat).Sub(
			new(big.Rat).Mul(new(big.Rat).Sub(b.u, a.u), new(big.Rat).Sub(cv, a.v)),
			new(big.Rat).Mul(new(big.Rat).Sub(b.v, a.v), new(big.Rat).Sub(cu, a.u)),
		)
		if (up && cross.Sign() > 0) || (down && cross.Sign() < 0) {
			inside = !inside
		}
	}
	return inside, nil
}
