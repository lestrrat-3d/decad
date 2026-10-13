package loftfillet

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// CircleHoleInside proves containment without reconstructing every
// free-form carrier in a new sketch scene. Each segment's exact enclosure,
// and the connector between its walked endpoint and the next, must miss the
// circle's containing square. Each curve can then be deformed to its chord
// without crossing the circle. Exact winding of those chords decides whether
// the circle center lies inside the already authenticated outer loop.
func CircleHoleInside(budget *proofbound.WorkBudget, outer sectionrecord.LoopRecord, hole sectionrecord.CircleSeg, constructionDelta float64,
	work *freeform.FreeformWork) (bool, error) {
	radius, err := hole.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || len(outer.Segments) < 3 ||
		constructionDelta < 0 || proofbound.IsNonFinite(constructionDelta) {
		return false, err
	}
	gap := new(big.Rat).SetFloat64(constructionDelta)
	cu, cv := new(big.Rat).SetFloat64(hole.Center.U), new(big.Rat).SetFloat64(hole.Center.V)
	r := new(big.Rat).SetFloat64(radius)
	if cu == nil || cv == nil || r == nil {
		return false, nil
	}
	square := sectionBox{
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
		box, err := sectionSegmentBox(seg, walk)
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
		if !boxSeparated(box, square, gap) {
			return false, nil
		}
	}
	inside := false
	for i := range starts {
		if err := budget.Step(); err != nil {
			return false, err
		}
		next := (i + 1) % len(starts)
		gap := sectionBox{
			u: pointInterval(ends[i].u, starts[next].u),
			v: pointInterval(ends[i].v, starts[next].v),
		}
		if !boxSeparated(gap, square, new(big.Rat).SetFloat64(constructionDelta)) {
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
