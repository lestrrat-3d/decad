package classbgeom

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is class B of agent-docs/general-boolean-design.md §3: an operand X read
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
// to X's faces by those one or two faces taking Y's section as a hole. A Y
// cap lying in the one face's plane is the flush reach (FlushReach), the one
// coplanar contact B8 lets through. Every other pair misses silently and takes
// the mesh path.

// Pair is an admitted pair stated in X's reference frame: X's face view
// with each face's map onto the reference, Y's axes as signed reference axes,
// and Y's record re-expressed exactly into the frame g (Y's axes at the
// reference origin).
type Pair struct {
	x      breprecord.Payload
	embeds []brepgeom.Embed
	faces  []FaceRecord
	tool   ToolRecord
	ref    r3.Frame
	y      prismrecord.Payload // re-expressed: frame g, coordinates shifted exactly
	g      r3.Frame
	// axis/sign map Y's local axis i (U, V, N) onto reference axis axis[i]
	// with sign[i]; axis[2] is d.
	axis [3]int
	sign [3]float64
	// flush names each tool cap (Z0, Z1) lying in a target plane, which only
	// the flush reach admits (FlushReach).
	flush [2]bool
}

// d is the reference axis Y sweeps along.
func (cp Pair) d() int { return cp.axis[2] }

// OfPayloads is class B's entry over two payloads and the one placement
// both are stated under (agent-docs/modify-general-design.md §3.3 step 4). It
// builds no Body and touches no document, so a caller that needs a boolean's
// record alone (a private cut) reaches it without one. ok=false (err nil) is
// a silent miss. A non-nil err is a refusal past the point of no return
// (prism-boolean §3.4) or a cancellation. Cut takes a as the target and b as
// the tool; Union and Intersect, being symmetric, also try the operands the
// other way round, so a brep payload or a stacked prism enters in either
// position. A payload whose transform is not placement is a silent miss.
//
// delta is a displacement the caller charges every face of both operands
// afterwards, beyond what their records state (route S's offsets,
// agent-docs/modify-general-design.md §3.3 step 4); a class-B boolean passes zero.
// A positive delta widens every box B7 and the through reach compare by it,
// and refuses the crossing reach, which charges the crossings it cuts no
// angle for a displacement its records do not carry.
func OfPayloads(ctx context.Context, op meshbool.OperationKind, a, b topo.Payload, placement r3.Transform, delta float64) (topo.Payload, bool, error) {
	pairs := [][2]topo.Payload{{a, b}}
	if op != meshbool.OpCut {
		pairs = append(pairs, [2]topo.Payload{b, a})
	}
	for _, pair := range pairs {
		cp, ok, err := admitPair(ctx, pair[0], pair[1], placement, delta, delta == 0)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		if cp.flush[0] || cp.flush[1] {
			// B8's flush lift: only the flush reach reads a shared plane.
			slabs, side, ok, err := FlushReach(ctx, cp.faces, cp.tool, cp.flush)
			if err != nil {
				return nil, false, err
			}
			if ok && flushOpAdmitted(op, side) {
				return buildClassB(ctx, op, cp, slabs)
			}
			continue
		}
		slabs, ok, err := ThroughReach(ctx, cp.faces, cp.tool, delta)
		if err != nil {
			return nil, false, err
		}
		if ok {
			return buildClassB(ctx, op, cp, slabs)
		}
		// A Cut of a prism outside the through reach takes the crossing
		// reach (crossing_cut.go).
		x, isPrism := pair[0].(prismrecord.Payload)
		if op != meshbool.OpCut || !isPrism {
			continue
		}
		if delta > 0 {
			return nil, false, fmt.Errorf(`%w: the tool leaves class B's through reach, so the cut would take its crossing reach, and the crossings that reach cuts carry no charge for the displacement %s its operands carry beyond their records`, decaderr.ErrUnsupported, units.Millimeters(delta))
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
func classBFaceView(ctx context.Context, payload topo.Payload) (breprecord.Payload, bool, error) {
	switch p := payload.(type) {
	case prismrecord.Payload:
		if p.SurfaceResult {
			return breprecord.Payload{}, false, nil
		}
		bp, err := breprecord.OfPrism(p)
		return bp, err == nil, err
	case stackedrecord.Payload:
		if p.IsGroup() {
			return breprecord.Payload{}, false, nil
		}
		if len(p.OuterRuns()) != 1 {
			return breprecord.Payload{}, false, nil
		}
		bp, err := breprecord.OfStacked(ctx, p)
		if err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return breprecord.Payload{}, false, ctxErr
			}
			// A stack its own audit refuses has no face view: a silent miss.
			return breprecord.Payload{}, false, nil
		}
		return bp, true, nil
	case breprecord.Payload:
		// A route L body's band patches are no face of its record, so the
		// record is no face view of the body (agent-docs/modify-general-design.md
		// Table DG's DG4): the pair takes the mesh path.
		if len(p.LoopBands) > 0 || p.BossShell != nil || p.PocketShell != nil {
			return breprecord.Payload{}, false, nil
		}
		return p, true, nil
	default:
		return breprecord.Payload{}, false, nil
	}
}

// flushOpAdmitted reports whether op over a flush pair builds the solid the
// rooted build states: a Union of a tool standing outside the face, and a Cut
// or Intersect of a tool inside the target. The other three miss: a Union of
// a tool inside is the target itself, and a Cut or Intersect of a tool
// outside meets the target in the shared disk alone.
func flushOpAdmitted(op meshbool.OperationKind, side FlushSide) bool {
	if op == meshbool.OpUnion {
		return side == FlushOutside
	}
	return side == FlushInside
}

// AdmitPair runs §3's entry gate B1–B8 in order, plus the two conditions
// the brep record itself needs: one shared placement, and Y's axes carried
// bit for bit as signed reference axes at the reference origin (§4.1). Every
// miss is silent. delta widens B7's boxes (OfPayloads).
func AdmitPair(ctx context.Context, xPayload, yPayload topo.Payload, placement r3.Transform, delta float64) (Pair, bool, error) {
	return admitPair(ctx, xPayload, yPayload, placement, delta, false)
}

// admitPair is AdmitPair with B8's flush lift: when allowFlush is true, a
// tool cap may lie in target planes (CoplanarFaces), and the pair records
// which caps do in flush.
func admitPair(ctx context.Context, xPayload, yPayload topo.Payload, placement r3.Transform, delta float64, allowFlush bool) (Pair, bool, error) {
	// B1: X exposes a face view; Y is a prism.
	y, okY := yPayload.(prismrecord.Payload)
	if !okY || y.SurfaceResult {
		return Pair{}, false, nil
	}
	x, ok, err := classBFaceView(ctx, xPayload)
	if err != nil || !ok {
		return Pair{}, false, err
	}
	if err := ctx.Err(); err != nil {
		return Pair{}, false, err
	}
	// B5: no section displacement to amplify.
	if x.SectionDelta() != 0 || y.SectionDelta != 0 {
		return Pair{}, false, nil
	}
	if x.Xform != placement || y.Xform != placement {
		return Pair{}, false, nil
	}
	embeds, err := breprecord.Embeds(x.Faces)
	if err != nil {
		return Pair{}, false, nil //nolint:nilerr // frames with no exact map onto the reference are a silent miss, not a refusal
	}
	ref := x.Faces[0].Frame
	// B3 and B4: every axis dot is exactly 0 or ±1, and Y's axes are the
	// reference axes, signed, bit for bit.
	refAxes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	yAxes := [3]r3.Vec{y.Frame.U(), y.Frame.V(), y.Frame.N()}
	for _, a := range refAxes {
		for _, b := range yAxes {
			if dot := a.Dot(b); dot != 0 && dot != 1 && dot != -1 {
				return Pair{}, false, nil
			}
		}
	}
	cp := Pair{x: x, embeds: embeds, ref: ref}
	cp.faces = make([]FaceRecord, len(x.Faces))
	for i, f := range x.Faces {
		cp.faces[i] = FaceRecord{
			Region: f.Region, Wall: f.Wall, Z0: f.Z0, Z1: f.Z1,
			Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta, Outward: f.Outward,
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
			return Pair{}, false, nil
		}
		used[cp.axis[i]] = true
	}
	// B3 for a prism or a stacked X: the sweeps are perpendicular. A
	// co-directional pair is class A's. A brep body has no single sweep, and
	// takes Y along any reference axis.
	if _, isBrep := xPayload.(breprecord.Payload); !isBrep && cp.axis[2] == 2 {
		return Pair{}, false, nil
	}
	g, err := r3.NewFrame(ref.Origin(), yAxes[0], yAxes[1])
	if err != nil {
		return Pair{}, false, fmt.Errorf(`%w: the tool's axes at the target's origin make no frame: %s`, decaderr.ErrDegenerate, err)
	}
	if g.U() != yAxes[0] || g.V() != yAxes[1] || g.N() != yAxes[2] {
		return Pair{}, false, nil
	}
	cp.g = g
	// B2: lines, circles and arcs only, each over its natural range, so every
	// recorded point is the point the record states.
	if !NaturalPair(cp.faces, y.Profile) {
		return Pair{}, false, nil
	}
	// G6: the tool is hole-free.
	if len(y.Profile.Holes) != 0 {
		return Pair{}, false, nil
	}
	shifted, ok := ShiftedTool(ref, y, g)
	if !ok {
		return Pair{}, false, nil
	}
	cp.y = shifted
	cp.tool = ToolRecord{
		Profile: shifted.Profile, Z0: shifted.Z0, Z1: shifted.Z1,
		Z0Delta: shifted.Z0Delta, Z1Delta: shifted.Z1Delta, Axis: cp.axis, Sign: cp.sign,
	}
	// B6: every line runs along a reference axis. Under B4 that is parallel
	// or perpendicular to the other operand's normal.
	if !AxisAlignedPair(cp.faces, cp.tool.Profile) {
		return Pair{}, false, nil
	}
	ok, err = CurvedApart(ctx, cp.faces, cp.tool, delta)
	if err != nil || !ok {
		return Pair{}, false, err
	}
	cp.flush, ok, err = CoplanarFaces(ctx, cp.faces, cp.tool, allowFlush)
	if err != nil || !ok {
		return Pair{}, false, err
	}
	return cp, true, nil
}
