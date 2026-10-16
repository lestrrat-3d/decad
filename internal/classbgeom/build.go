package classbgeom

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// buildClassB assembles op's result (§4) over the reach, in g. Each slab face
// becomes a planar face in g carrying Y's section as a new hole, its region
// the perpendicular-face scene's answer (§5): prism-boolean's clean-nesting
// match of the face's region against Y's section. A scene that does not
// return that match leaves the pair outside the reach and misses silently
// (prism-boolean §3.4); every refusal after it is an error.
//
// Along g's normal, Y spans [a, b] and the slab faces sit at s (rooted) or
// s0 < s1 (through). X's material over Y's section is [s0, s1], or the side of
// s its face's outward normal points away from. Then:
//
//   - Cut keeps X's other faces, the slab faces, and Y's walls reversed over
//     the inside interval; rooted, the inside end is the hole's floor, Y's
//     section facing back up the hole.
//   - Union keeps X's other faces, the slab faces, and Y's walls with Y's caps
//     over the parts of [a, b] outside X.
//   - Intersect is Y's section swept over the inside interval: a prism.
func buildClassB(ctx context.Context, op meshbool.OperationKind, cp Pair, reach []Slab) (topo.Payload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	slabs := make([]SlabFace, len(reach))
	for i, s := range reach {
		f := cp.x.Faces[s.Face]
		face, err := RestateSlabFace(budget, SlabFaceInput{
			Region: f.Region, Wall: f.Wall, Z0: f.Z0, Z1: f.Z1,
			Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta, Delta: f.Delta,
			Sweep: f.Sweep, Normal: f.Frame.N(),
		}, cp.embeds[s.Face], cp.axis, cp.sign, s)
		if err != nil {
			return nil, false, err
		}
		region, matched, err := classBPerpendicularRegion(ctx, cp, face.Region)
		if err != nil || !matched {
			return nil, false, err
		}
		face.Region = region
		slabs[i] = face
	}
	if len(slabs) == 2 && slabs[1].Level < slabs[0].Level {
		slabs[0], slabs[1] = slabs[1], slabs[0]
	}
	// The interval of [a, b] inside X, each end with its displacement.
	type end struct{ level, delta float64 }
	a, b := end{cp.y.Z0, cp.y.Z0Delta}, end{cp.y.Z1, cp.y.Z1Delta}
	var inside [2]end
	var outside [][2]end
	switch {
	case len(slabs) == 2:
		inside = [2]end{{slabs[0].Level, slabs[0].LevelDelta}, {slabs[1].Level, slabs[1].LevelDelta}}
		outside = [][2]end{{a, inside[0]}, {inside[1], b}}
	case slabs[0].Outward:
		// Outward +N: the material lies below the face.
		inside = [2]end{a, {slabs[0].Level, slabs[0].LevelDelta}}
		outside = [][2]end{{inside[1], b}}
	default:
		inside = [2]end{{slabs[0].Level, slabs[0].LevelDelta}, b}
		outside = [][2]end{{a, inside[0]}}
	}
	if op == meshbool.OpIntersect {
		return prismrecord.Payload{Profile: cp.y.Profile, Frame: cp.g, Xform: cp.x.Xform,
			Z0: inside[0].level, Z1: inside[1].level, Z0Delta: inside[0].delta, Z1Delta: inside[1].delta}, true, nil
	}
	// Point of no return (prism-boolean §3.4).
	drop := map[int]struct{}{}
	for _, s := range reach {
		drop[s.Face] = struct{}{}
	}
	out := breprecord.Payload{Xform: cp.x.Xform}
	for fi, f := range cp.x.Faces {
		if _, gone := drop[fi]; gone {
			continue
		}
		out.Faces = append(out.Faces, f)
	}
	for _, s := range slabs {
		region := s.Region
		out.Faces = append(out.Faces, breprecord.Face{Frame: cp.g, Region: &region, Outward: s.Outward, Sweep: s.Sweep,
			Z0: s.Level, Z1: s.Level, Z0Delta: s.LevelDelta, Z1Delta: s.LevelDelta, Delta: s.Delta})
	}
	walls := func(loop sectionrecord.LoopRecord, lo, hi end) {
		for _, seg := range loop.Segments {
			out.Faces = append(out.Faces, breprecord.Face{Frame: cp.g, Wall: seg,
				Z0: lo.level, Z1: hi.level, Z0Delta: lo.delta, Z1Delta: hi.delta})
		}
	}
	section := cp.y.Profile
	addCap := func(at end, outward bool) {
		region := section
		out.Faces = append(out.Faces, breprecord.Face{Frame: cp.g, Region: &region, Outward: outward,
			Z0: at.level, Z1: at.level, Z0Delta: at.delta, Z1Delta: at.delta})
	}
	switch op {
	case meshbool.OpCut:
		hole, err := offset2d.ReverseLoopRecordContext(ctx, section.Outer)
		if err != nil {
			return nil, false, err
		}
		walls(hole, inside[0], inside[1])
		if len(slabs) == 1 {
			// The floor faces back up the hole: outward toward the open end.
			if slabs[0].Outward {
				addCap(inside[0], true)
			} else {
				addCap(inside[1], false)
			}
		}
	case meshbool.OpUnion:
		for _, part := range outside {
			walls(section.Outer, part[0], part[1])
		}
		if len(slabs) == 2 || !slabs[0].Outward {
			addCap(a, false)
		}
		if len(slabs) == 2 || slabs[0].Outward {
			addCap(b, true)
		}
	default:
		return nil, false, fmt.Errorf(`%w: class B has no %s`, decaderr.ErrUnsupported, op)
	}
	out.AssignRoles()
	return out, true, nil
}

// classBPerpendicularRegion is the perpendicular-face scene (§5) of one slab
// face: its region and Y's section, both in g, resolved by prism-boolean's
// clean-nesting Cut match and authenticated through RecordProfile. matched
// is false when the scene returned no such match.
func classBPerpendicularRegion(ctx context.Context, cp Pair, region momentinput.Profile) (momentinput.Profile, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	target := prismrecord.Payload{Profile: region, Frame: cp.g, Xform: cp.x.Xform, Z0: 0, Z1: 1}
	tool := prismrecord.Payload{Profile: cp.y.Profile, Frame: cp.g, Xform: cp.x.Xform, Z0: 0, Z1: 1}
	segments, within, err := prismcells.RegionsWithinWorkCap(budget, target.Profile, tool.Profile)
	if err != nil {
		return momentinput.Profile{}, false, err
	}
	if !within {
		return momentinput.Profile{}, false, fmt.Errorf(`%w: the class-B face scene charges at least %d arranger segments against this evaluator's cap of %d`,
			decaderr.ErrUnsupported, segments, prismcells.MaxArrangementSegments)
	}
	reexpress, err := prismcells.NewReexpression(target.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return momentinput.Profile{}, false, err
	}
	s, match, sceneDelta, resolved, err := prismcells.ResolveCut(ctx, budget, target.Profile, tool.Profile, reexpress)
	if err != nil || !resolved || sceneDelta.A != 0 || sceneDelta.B != 0 {
		return momentinput.Profile{}, false, err
	}
	profile, err := prismcells.RecordProfileContext(ctx, s, match)
	if err != nil {
		return momentinput.Profile{}, false, err
	}
	return profile, true, nil
}
