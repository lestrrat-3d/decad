package classbgeom

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// buildClassBWalk assembles op's result over a walked sweep
// (agent-docs/general-boolean-design.md §3 "Pin across several faces"), in g.
// Inside the tube the result is 1 for Union, 0 for Cut and the target's own
// layer for Intersect; outside it, the target for Union and Cut and nothing
// for Intersect. W6 needs the result to change the target in the tube and to
// state one body: a Union needs a layer of the tube outside the target, a Cut
// one inside it, and an Intersect exactly one inside it. Every other pair
// misses silently.
//
// Each walked face's perpendicular-face scene must prove the tool's section
// lies strictly inside its region (W5), or the pair misses. A face with the
// tube's value on both sides takes the section as a hole; a flush face whose
// outer side differs from the tube's value stays whole, its record unchanged.
func buildClassBWalk(ctx context.Context, op meshbool.OperationKind, cp Pair, w Walk) (topo.Payload, bool, error) {
	n := len(w.Slabs)
	var inside, outside []int
	for k := 0; k <= n; k++ {
		if _, _, nonempty := w.Interval(k); !nonempty {
			continue
		}
		if w.Inside[k] {
			inside = append(inside, k)
			continue
		}
		outside = append(outside, k)
	}
	switch op {
	case meshbool.OpUnion:
		if len(outside) == 0 {
			return nil, false, nil
		}
	case meshbool.OpCut:
		if len(inside) == 0 {
			return nil, false, nil
		}
	case meshbool.OpIntersect:
		if len(inside) != 1 {
			return nil, false, nil
		}
	default:
		return nil, false, fmt.Errorf(`%w: class B has no %s`, decaderr.ErrUnsupported, op)
	}
	budget := proofbound.NewWorkBudget(ctx)
	holed := make([]SlabFace, n)
	for i, s := range w.Slabs {
		f := cp.x.Faces[s.Face]
		face, err := RestateSlabFace(budget, SlabFaceInput{
			Region: f.Region, Wall: f.Wall, Z0: f.Z0, Z1: f.Z1,
			Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta, Delta: f.Delta,
			Sweep: f.Sweep, Normal: f.Frame.N(),
		}, cp.embeds[s.Face], cp.axis, cp.sign, s.Slab)
		if err != nil {
			return nil, false, err
		}
		region, matched, err := classBPerpendicularRegion(ctx, cp, face.Region)
		if err != nil || !matched {
			return nil, false, err
		}
		face.Region = region
		holed[i] = face
	}
	if op == meshbool.OpIntersect {
		lo, hi, _ := w.Interval(inside[0])
		return prismrecord.Payload{Profile: cp.y.Profile, Frame: cp.g, Xform: cp.x.Xform, Z0: lo, Z1: hi}, true, nil
	}
	// Point of no return (prism-boolean §3.4).
	tube := op == meshbool.OpUnion
	drop := map[int]struct{}{}
	for i, s := range w.Slabs {
		below, above := tube, tube
		if i == 0 && w.LowFlush() {
			below = w.Inside[0]
		}
		if i == n-1 && w.HighFlush() {
			above = w.Inside[n]
		}
		if below == above {
			drop[s.Face] = struct{}{}
		}
	}
	out := breprecord.Payload{Xform: cp.x.Xform}
	for fi, f := range cp.x.Faces {
		if _, gone := drop[fi]; gone {
			continue
		}
		out.Faces = append(out.Faces, f)
	}
	for i, s := range w.Slabs {
		if _, gone := drop[s.Face]; !gone {
			continue
		}
		h := holed[i]
		region := h.Region
		out.Faces = append(out.Faces, breprecord.Face{Frame: cp.g, Region: &region, Outward: h.Outward, Sweep: h.Sweep,
			Z0: h.Level, Z1: h.Level, Delta: h.Delta})
	}
	section := cp.y.Profile
	wall := section.Outer
	walls := outside
	if op == meshbool.OpCut {
		hole, err := offset2d.ReverseLoopRecordContext(ctx, section.Outer)
		if err != nil {
			return nil, false, err
		}
		wall, walls = hole, inside
	}
	for _, k := range walls {
		lo, hi, _ := w.Interval(k)
		for _, seg := range wall.Segments {
			out.Faces = append(out.Faces, breprecord.Face{Frame: cp.g, Wall: seg, Z0: lo, Z1: hi})
		}
	}
	addCap := func(at float64, outward bool) {
		region := section
		out.Faces = append(out.Faces, breprecord.Face{Frame: cp.g, Region: &region, Outward: outward, Z0: at, Z1: at})
	}
	// A cap with the target's layer outside and the tube's value inside
	// differs when they disagree; its outward normal points at the empty side.
	if !w.LowFlush() && w.Inside[0] != tube {
		addCap(w.A, !tube)
	}
	if !w.HighFlush() && w.Inside[n] != tube {
		addCap(w.B, tube)
	}
	out.AssignRoles()
	return out, true, nil
}
