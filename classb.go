package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is class B of docs/general-boolean-design.md §3: an operand X read
// through its face view (§4.1) — a prism, a stacked prism or a brep body — and
// a prism Y whose axes are signed axes of X's reference frame. Every reading
// is taken in X's reference frame, the first face's: d is the reference axis
// Y sweeps along, e and w the other two.
//
// This increment builds the through-nesting reach (§3 B.3), decided by exact
// box comparisons of recorded coordinates. Y's tube — its section box swept
// over its interval — meets one or two faces of X and no other, each a face
// across d (a planar face whose normal lies on d, or a straight wall at
// constant d) whose region holds Y's section. Over the tube, X is then the
// slab between two such faces (through) or the half-space past one (rooted),
// and every boolean of X and Y is Y's section swept over an interval, joined
// to X's faces by those one or two faces taking Y's section as a hole. Every
// other pair misses silently and takes the mesh path.

// classBPair is an admitted pair stated in X's reference frame: X's face view
// with each face's map onto the reference, Y's axes as signed reference axes,
// and Y's record re-expressed exactly into the frame g (Y's axes at the
// reference origin).
type classBPair struct {
	x      brepPayload
	embeds []brepEmbed
	faces  []classbgeom.FaceRecord
	tool   classbgeom.ToolRecord
	ref    r3.Frame
	y      prismPayload // re-expressed: frame g, coordinates shifted exactly
	g      r3.Frame
	// axis/sign map Y's local axis i (U, V, N) onto reference axis axis[i]
	// with sign[i]; axis[2] is d.
	axis [3]int
	sign [3]float64
}

// d is the reference axis Y sweeps along.
func (cp classBPair) d() int { return cp.axis[2] }

// tryClassB attempts op over the pair of bodies. ok=false (err nil) is a
// silent miss: the caller takes the mesh path unchanged. It is
// classBOfPayloads over the operands' payloads, placed as the first operand
// is.
func tryClassB(ctx context.Context, op meshbool.OperationKind, a, b *Body) (featurePayload, bool, error) {
	// A body this evaluator did not build has no payload and no class.
	if a.payload == nil || b.payload == nil {
		return nil, false, nil
	}
	return classBOfPayloads(ctx, op, a.payload, b.payload, a.payload.transform())
}

// classBOfPayloads is class B's entry over two payloads and the one placement
// both are stated under (docs/modify-general-design.md §3.3 step 4). It
// builds no Body and touches no document, so a caller that needs a boolean's
// record alone (a private cut) reaches it without one. ok=false (err nil) is
// a silent miss. A non-nil err is a refusal past the point of no return
// (prism-boolean §3.4) or a cancellation. Cut takes a as the target and b as
// the tool; Union and Intersect, being symmetric, also try the operands the
// other way round, so a brep payload or a stacked prism enters in either
// position. A payload whose transform is not placement is a silent miss.
func classBOfPayloads(ctx context.Context, op meshbool.OperationKind, a, b featurePayload, placement r3.Transform) (featurePayload, bool, error) {
	pairs := [][2]featurePayload{{a, b}}
	if op != meshbool.OpCut {
		pairs = append(pairs, [2]featurePayload{b, a})
	}
	for _, pair := range pairs {
		cp, ok, err := admitClassBPair(ctx, pair[0], pair[1], placement)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		slabs, ok, err := classbgeom.ThroughReach(ctx, cp.faces, cp.tool)
		if err != nil {
			return nil, false, err
		}
		if ok {
			return buildClassB(ctx, op, cp, slabs)
		}
		// A Cut of a prism outside the through reach takes the crossing
		// reach (classb_crossing.go).
		x, isPrism := pair[0].(prismPayload)
		if op != meshbool.OpCut || !isPrism {
			continue
		}
		bp, ok, err := tryClassBCrossingCut(ctx, x, cp)
		if err != nil || ok {
			return bp, ok, err
		}
	}
	return nil, false, nil
}

// classBFaceView is X's face view, or ok=false where X has none this
// increment reads: a prism (not a surface result), a stacked prism whose
// slabs keep one outer loop and one region, or a brep body.
func classBFaceView(ctx context.Context, payload featurePayload) (brepPayload, bool, error) {
	switch p := payload.(type) {
	case prismPayload:
		if p.surfaceResult {
			return brepPayload{}, false, nil
		}
		bp, err := brepOfPrism(p)
		return bp, err == nil, err
	case stackedPrismPayload:
		if p.isGroup() {
			return brepPayload{}, false, nil
		}
		runs, err := p.outerRuns()
		if err != nil || len(runs) != 1 {
			return brepPayload{}, false, err
		}
		bp, err := brepOfStacked(ctx, p)
		if err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return brepPayload{}, false, ctxErr
			}
			// A stack its own audit refuses has no face view: a silent miss.
			return brepPayload{}, false, nil
		}
		return bp, true, nil
	case brepPayload:
		// A route L body's band patches are no face of its record, so the
		// record is no face view of the body (docs/modify-general-design.md
		// Table DG's DG4): the pair takes the mesh path.
		if len(p.loopBands) > 0 {
			return brepPayload{}, false, nil
		}
		return p, true, nil
	default:
		return brepPayload{}, false, nil
	}
}

// admitClassBPair runs §3's entry gate B1–B8 in order, plus the two conditions
// the brep record itself needs: one shared placement, and Y's axes carried
// bit for bit as signed reference axes at the reference origin (§4.1). Every
// miss is silent.
func admitClassBPair(ctx context.Context, xPayload, yPayload featurePayload, placement r3.Transform) (classBPair, bool, error) {
	// B1: X exposes a face view; Y is a prism.
	y, okY := yPayload.(prismPayload)
	if !okY || y.surfaceResult {
		return classBPair{}, false, nil
	}
	x, ok, err := classBFaceView(ctx, xPayload)
	if err != nil || !ok {
		return classBPair{}, false, err
	}
	if err := ctx.Err(); err != nil {
		return classBPair{}, false, err
	}
	// B5: no section displacement to amplify.
	if x.sectionDelta() != 0 || y.sectionDelta != 0 {
		return classBPair{}, false, nil
	}
	if x.xform != placement || y.xform != placement {
		return classBPair{}, false, nil
	}
	embeds, err := brepEmbeds(x.faces)
	if err != nil {
		return classBPair{}, false, nil //nolint:nilerr // frames with no exact map onto the reference are a silent miss, not a refusal
	}
	ref := x.faces[0].frame
	// B3 and B4: every axis dot is exactly 0 or ±1, and Y's axes are the
	// reference axes, signed, bit for bit.
	refAxes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	yAxes := [3]r3.Vec{y.frame.U(), y.frame.V(), y.frame.N()}
	for _, a := range refAxes {
		for _, b := range yAxes {
			if dot := a.Dot(b); dot != 0 && dot != 1 && dot != -1 {
				return classBPair{}, false, nil
			}
		}
	}
	cp := classBPair{x: x, embeds: embeds, ref: ref}
	cp.faces = make([]classbgeom.FaceRecord, len(x.faces))
	for i, f := range x.faces {
		cp.faces[i] = classbgeom.FaceRecord{
			Region: f.region, Wall: f.wall, Z0: f.z0, Z1: f.z1,
			Z0Delta: f.z0Delta, Z1Delta: f.z1Delta, Outward: f.outward,
			Axis: embeds[i].Axis, Sign: embeds[i].Sign,
		}
	}
	used := [3]bool{}
	for i, b := range yAxes {
		found := false
		for j, a := range refAxes {
			switch {
			case b == a:
				cp.axis[i], cp.sign[i], found = j, 1, true
			case b == a.Scale(-1):
				cp.axis[i], cp.sign[i], found = j, -1, true
			}
			if found {
				break
			}
		}
		if !found || used[cp.axis[i]] {
			return classBPair{}, false, nil
		}
		used[cp.axis[i]] = true
	}
	// B3 for a prism or a stacked X: the sweeps are perpendicular. A
	// co-directional pair is class A's. A brep body has no single sweep, and
	// takes Y along any reference axis.
	if _, isBrep := xPayload.(brepPayload); !isBrep && cp.axis[2] == 2 {
		return classBPair{}, false, nil
	}
	g, err := r3.NewFrame(ref.Origin(), yAxes[0], yAxes[1])
	if err != nil {
		return classBPair{}, false, fmt.Errorf(`%w: the tool's axes at the target's origin make no frame: %s`, ErrDegenerate, err)
	}
	if g.U() != yAxes[0] || g.V() != yAxes[1] || g.N() != yAxes[2] {
		return classBPair{}, false, nil
	}
	cp.g = g
	// B2: lines, circles and arcs only, each over its natural range, so every
	// recorded point is the point the record states.
	if !classbgeom.NaturalPair(cp.faces, y.profile) {
		return classBPair{}, false, nil
	}
	// G6: the tool is hole-free.
	if len(y.profile.Holes) != 0 {
		return classBPair{}, false, nil
	}
	shifted, ok := classBShiftedTool(ref, y, g)
	if !ok {
		return classBPair{}, false, nil
	}
	cp.y = shifted
	cp.tool = classbgeom.ToolRecord{
		Profile: shifted.profile, Z0: shifted.z0, Z1: shifted.z1,
		Z0Delta: shifted.z0Delta, Z1Delta: shifted.z1Delta, Axis: cp.axis, Sign: cp.sign,
	}
	// B6: every line runs along a reference axis. Under B4 that is parallel
	// or perpendicular to the other operand's normal.
	if !classbgeom.AxisAlignedPair(cp.faces, cp.tool.Profile) {
		return classBPair{}, false, nil
	}
	ok, err = classbgeom.CurvedApart(ctx, cp.faces, cp.tool)
	if err != nil || !ok {
		return classBPair{}, false, err
	}
	ok, err = classbgeom.NoCoplanarFaces(ctx, cp.faces, cp.tool)
	if err != nil || !ok {
		return classBPair{}, false, err
	}
	return cp, true, nil
}

// classBShiftedTool re-expresses Y's record into g, Y's axes at the reference
// origin: every coordinate moves by the exact rational dot of the origin
// difference with its axis. The shift is admitted only where every shifted
// coordinate and level is itself a float, so the re-expression rounds
// nothing: a datum plane and its CreateOffsetPlane at a float distance meet
// that, and a pair whose sums would round misses.
func classBShiftedTool(ref r3.Frame, y prismPayload, g r3.Frame) (prismPayload, bool) {
	loops := append([]LoopRecord{y.profile.Outer}, y.profile.Holes...)
	moved, z0, z1, ok := classbgeom.ShiftedRecord(ref, y.frame, g, loops, y.z0, y.z1)
	out := y
	out.frame = g
	out.profile = ProfileRecord{Outer: moved[0], Holes: moved[1:]}
	out.z0, out.z1 = z0, z1
	out.walks = nil
	return out, ok
}

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
func buildClassB(ctx context.Context, op meshbool.OperationKind, cp classBPair, reach []classbgeom.Slab) (featurePayload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	slabs := make([]classbgeom.SlabFace, len(reach))
	for i, s := range reach {
		f := cp.x.faces[s.Face]
		face, err := classbgeom.RestateSlabFace(budget, classbgeom.SlabFaceInput{
			Region: f.region, Wall: f.wall, Z0: f.z0, Z1: f.z1,
			Z0Delta: f.z0Delta, Z1Delta: f.z1Delta, Delta: f.delta,
			Sweep: f.sweep, Normal: f.frame.N(),
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
	a, b := end{cp.y.z0, cp.y.z0Delta}, end{cp.y.z1, cp.y.z1Delta}
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
		return prismPayload{profile: cp.y.profile, frame: cp.g, xform: cp.x.xform,
			z0: inside[0].level, z1: inside[1].level, z0Delta: inside[0].delta, z1Delta: inside[1].delta}, true, nil
	}
	// Point of no return (prism-boolean §3.4).
	drop := map[int]struct{}{}
	for _, s := range reach {
		drop[s.Face] = struct{}{}
	}
	out := brepPayload{xform: cp.x.xform}
	for fi, f := range cp.x.faces {
		if _, gone := drop[fi]; gone {
			continue
		}
		out.faces = append(out.faces, f)
	}
	for _, s := range slabs {
		region := s.Region
		out.faces = append(out.faces, brepFace{frame: cp.g, region: &region, outward: s.Outward, sweep: s.Sweep,
			z0: s.Level, z1: s.Level, z0Delta: s.LevelDelta, z1Delta: s.LevelDelta, delta: s.Delta})
	}
	walls := func(loop LoopRecord, lo, hi end) {
		for _, seg := range loop.Segments {
			out.faces = append(out.faces, brepFace{frame: cp.g, wall: seg,
				z0: lo.level, z1: hi.level, z0Delta: lo.delta, z1Delta: hi.delta})
		}
	}
	section := cp.y.profile
	addCap := func(at end, outward bool) {
		region := section
		out.faces = append(out.faces, brepFace{frame: cp.g, region: &region, outward: outward,
			z0: at.level, z1: at.level, z0Delta: at.delta, z1Delta: at.delta})
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
		return nil, false, fmt.Errorf(`%w: class B has no %s`, ErrUnsupported, op)
	}
	out.assignRoles()
	return out, true, nil
}

// classBPerpendicularRegion is the perpendicular-face scene (§5) of one slab
// face: its region and Y's section, both in g, resolved by prism-boolean's
// clean-nesting Cut match and authenticated through RecordProfile. matched
// is false when the scene returned no such match.
func classBPerpendicularRegion(ctx context.Context, cp classBPair, region ProfileRecord) (ProfileRecord, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	target := prismPayload{profile: region, frame: cp.g, xform: cp.x.xform, z0: 0, z1: 1}
	tool := prismPayload{profile: cp.y.profile, frame: cp.g, xform: cp.x.xform, z0: 0, z1: 1}
	segments, within, err := prismcells.RegionsWithinWorkCap(budget, target.profile, tool.profile)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	if !within {
		return ProfileRecord{}, false, fmt.Errorf(`%w: the class-B face scene charges at least %d arranger segments against this evaluator's cap of %d`,
			ErrUnsupported, segments, prismcells.MaxArrangementSegments)
	}
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(target), prismPlacementOf(tool))
	if err != nil {
		return ProfileRecord{}, false, err
	}
	s, match, sceneDelta, resolved, err := resolvePrismCut(ctx, budget, target, tool, reexpress)
	if err != nil || !resolved || sceneDelta.A != 0 || sceneDelta.B != 0 {
		return ProfileRecord{}, false, err
	}
	profile, err := prismRecordProfileContext(ctx, s, match)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	return profile, true, nil
}
