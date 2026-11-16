// Package displacedface decides which faces of a displaced body are displaced
// walls: side faces whose recorded surface lies only within a loop's bound of
// the wall the body denotes (agent-docs/carrier-record-design.md §8.7, rule
// DW, and agent-docs/brep-modify-design.md §2.2). Both the brep-modify
// faceted view and the mesh Boolean's source tags decline the retained tag of
// such a face.
package displacedface

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// FaceLoop reads a prism face's roles: cap when every role is a cap's, or the
// loop index li when every role is a side role side(li,j) of that one loop.
// Any other face reads as neither.
func FaceLoop(f *topo.Face) (li int, side, isCap bool) {
	if len(f.Origins) == 0 {
		return 0, false, false
	}
	caps, loop := 0, -1
	for _, o := range f.Origins {
		if o.Role == provenance.RoleCapStart || o.Role == provenance.RoleCapEnd {
			caps++
			continue
		}
		var l, j int
		if n, err := fmt.Sscanf(o.Role, "side(%d,%d)", &l, &j); err != nil || n != 2 ||
			o.Role != fmt.Sprintf("side(%d,%d)", l, j) || l < 0 || (loop >= 0 && l != loop) {
			return 0, false, false
		}
		loop = l
	}
	switch {
	case caps == len(f.Origins):
		return 0, false, true
	case caps == 0:
		return loop, true, false
	default:
		return 0, false, false
	}
}

// PrismSideFaces lists, by index in topobuild.Faces order, every face of the
// displaced prism b whose surface must not be retained: each face that is
// neither a cap nor a side face of a loop with exact carriers, a face whose
// roles name no single loop included. kept reports whether some side face
// keeps its tag. record, when non-nil, is b's valid owned carrier record.
func PrismSideFaces(budget *proofbound.WorkBudget, b *topo.Body, pp prismrecord.Payload,
	record *prismrecord.CarrierRecord) (map[int]struct{}, bool, error) {
	exact := map[int]bool{}
	untag := map[int]struct{}{}
	kept := false
	for fi, f := range topobuild.Faces(b) {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		li, side, isCap := FaceLoop(f)
		if isCap {
			continue
		}
		if !side {
			untag[fi] = struct{}{}
			continue
		}
		isExact, seen := exact[li]
		if !seen {
			var err error
			if isExact, err = pp.CarrierExact(budget, li); err != nil {
				return nil, false, err
			}
			// An exact loop of the body's own record (all entries zero) holds that
			// loop of the section unchanged (§3.4).
			isExact = isExact || (record != nil && record.ExactLoop(li))
			exact[li] = isExact
		}
		if !isExact {
			untag[fi] = struct{}{}
			continue
		}
		kept = true
	}
	return untag, kept, nil
}

// Walls is the set of displaced walls of body b, read once for the body: the
// faces a mesh Boolean's source tag must decline (rule DW). It is empty for a
// body that is not a DisplacedPayload. A displaced prism's walls are not read: a
// per-segment carrier test (declining a wall whose record entry Segment[l][j]
// is nonzero) still declines the line walls beside a fillet's feet, whose
// entries are a few ulps, and so breaks the retained edge blend of a cut
// filleted tool (apitest TestFacetedRetainedBlendConcaveArcEnd).
func Walls(budget *proofbound.WorkBudget, b *topo.Body) (map[*topo.Face]struct{}, error) {
	faces := topobuild.Faces(b)
	switch pl := b.Payload.(type) {
	case capblend.DisplacedPayload:
		out := map[*topo.Face]struct{}{}
		for _, f := range faces {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if displacedCapBlendWall(f, pl.LoopDelta) {
				out[f] = struct{}{}
			}
		}
		return out, nil
	default:
		return map[*topo.Face]struct{}{}, nil
	}
}

// displacedCapBlendWall reports a side wall of a loop with a nonzero
// LoopDelta, or any other face that is neither a cap, a band patch of a
// selected loop nor a side wall of one loop.
func displacedCapBlendWall(f *topo.Face, loopDelta []float64) bool {
	for _, o := range f.Origins {
		if len(o.Role) > len("chamferCap(") && o.Role[:len("chamferCap(")] == "chamferCap(" {
			return false
		}
	}
	li, side, isCap := FaceLoop(f)
	switch {
	case isCap:
		return false
	case side:
		return li < len(loopDelta) && loopDelta[li] > 0
	default:
		return true
	}
}
