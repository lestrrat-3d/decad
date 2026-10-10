package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// This file is route L of docs/modify-general-design.md ("modify-general §N"
// below) and route V of docs/vertex-blend-design.md: a Chamfer or Fillet of
// complete planar-face loops on a brep or stacked receiver (§4;
// docs/loop-fillet-design.md, "loop-fillet
// §N", for the fillet arm). It also decides, for a Fillet or Chamfer that
// route P does not take, which of route E (docs/brep-modify-design.md §5) and
// route L reads the selection: route E takes single straight edges along
// reference axes, no two sharing a vertex. Route V takes a Fillet of those
// edges with complete loops, and route L reads the remaining selections,
// including a selected chain on a swept wall restated as a planar face.
// Table LB admits the loops (admitLoops, classifyLoop); the record is
// rewritten (rewriteLoopFaces): each loop's face takes the loop's cap contour,
// each face beside the loop is trimmed to the band's side level, and the band
// is recorded on the result (brepLoopBand) and attached at body build
// (brep_loop_band.go, and brep_loop_fillet.go for a fillet band).

// brepLoopRead holds one route L call's readings of the receiver: its record,
// topology, each planar loop segment's use, and the record edge each selected
// edge matched (-1 none, -2 several).
type brepLoopRead struct {
	bp      brepPayload
	topo    *brepTopology
	call    brepModifyRequest
	budget  *proofbound.WorkBudget
	loopUse map[[3]int]int
	matched []int
}

// brepLoopSel is one touched loop of Table LB: the loop of index loop on the
// planar face of index face, what lies beside each of its segments (LB3),
// and the band's sense (LB6), −1 for walls descending into the body and +1
// for walls rising off it.
type brepLoopSel struct {
	face, loop int
	beside     []brepLoopBeside
	sigma      float64
	setback    capSetback
}

// brepLoopBeside is the face beside one loop segment (LB3): use is that
// face's use of the segment's edge. A (sw) face has one far level, the end of
// its sweep away from the loop; a (pl) face has two, the far ends of its two
// neighbouring lines along the loop face's normal, whose uses are nb. Far
// levels are reference coordinates along that normal.
type brepLoopBeside struct {
	use    int
	planar bool
	far    []float64
	nb     [2]int
}

// brepLoopRoute is the brep route of a Fillet or Chamfer that route P does not
// take (modify-general §4.4's stage 2c, loop-fillet §6). Independent straight
// edges take route E. A Fillet of independent straight edges and complete
// planar loops takes route V, which runs E then L. Shared straight edges on
// a swept wall restate that wall before route L's partial-loop construction.
// Other selections take L or its SL1 refusal.
func brepLoopRoute(ctx context.Context, d *Document, bp brepPayload, call brepModifyRequest) (*Body, error) {
	r, err := newBrepLoopRead(ctx, bp, call)
	if err != nil {
		return nil, err
	}
	if err := r.matchEdges(); err != nil {
		return nil, err
	}
	if r.singleStraightEdges() {
		return brepBlendEdges(ctx, d, bp, call)
	}
	if call.asym != nil {
		loops, err := r.admitLoops()
		if err != nil {
			return nil, fmt.Errorf(`%w: an asymmetric chamfer on a brep requires independent straight edges or complete planar-face loops (modify-reach SX16): %v`, ErrUnsupported, err)
		}
		return r.buildSelectedLoops(ctx, d, loops)
	}
	if call.loopKind == brepBandFillet {
		if body, recognized, err := r.cornerTriad(ctx, d); recognized {
			return body, err
		}
		loops, singles, mixed, err := r.splitVertexBlend()
		if err != nil {
			return nil, err
		}
		if mixed {
			stepCall := call
			stepCall.edges = singles
			step, err := brepBlendEdges(ctx, d, bp, stepCall)
			if err != nil {
				return nil, err
			}
			rewritten, ok := step.payload.(brepPayload)
			if !ok {
				return nil, fmt.Errorf(`%w: the straight-edge fillet produced no brep record`, ErrUnsupported)
			}
			next, err := newBrepLoopRead(ctx, rewritten, call)
			if err != nil {
				return nil, err
			}
			return next.buildSelectedLoops(ctx, d, loops)
		}
		if sel, selected, ok := r.partialSelectedLoop(); ok {
			return brepFilletPartialLoop(ctx, d, bp, call, sel, selected)
		}
		// A pair of edges can share a swept wall without sharing a planar
		// loop in the input record. Route E's preparation restates that wall;
		// the same partial-loop builder can then close their common corner.
		if selectedStraightEdgesShareVertex(call.edges) {
			er, blends, err := prepareBrepEdgeBlends(ctx, bp, call, true, nil)
			if err == nil {
				next, err := newBrepLoopRead(ctx, er.bp, call)
				if err != nil {
					return nil, err
				}
				if err := next.matchEdges(); err != nil {
					return nil, err
				}
				if sel, selected, ok := next.partialSelectedLoop(); ok {
					selectedAt := map[[3]float64]int{}
					for _, eb := range blends {
						for _, v := range eb.v {
							selectedAt[v]++
						}
					}
					for _, eb := range blends {
						for k, v := range eb.v {
							eb.terminal[k] = selectedAt[v] == 1
						}
					}
					return next.buildPartialFillet(ctx, d, sel, selected, blends)
				}
			}
		}
	}
	return r.buildLoops(ctx, d)
}

// cornerTriad rounds three edges at one vertex by rounding the edge across
// a planar cap first. The cap's two shortened lines and the new tangent arc
// then form one partial fillet chain with a sphere at that arc.
func (r *brepLoopRead) cornerTriad(ctx context.Context, d *Document) (*Body, bool, error) {
	if len(r.call.edges) != 3 {
		return nil, false, nil
	}
	var corner *Vertex
	for _, v := range [2]*Vertex{r.call.edges[0].start, r.call.edges[0].end} {
		if v != nil && (r.call.edges[1].start == v || r.call.edges[1].end == v) &&
			(r.call.edges[2].start == v || r.call.edges[2].end == v) {
			corner = v
			break
		}
	}
	if corner == nil {
		return nil, false, nil
	}
	for single := range 3 {
		var others [2]int
		j := 0
		for i := range 3 {
			if i != single {
				others[j] = i
				j++
			}
		}
		capRead := *r
		capRead.call.edges = []*Edge{r.call.edges[others[0]], r.call.edges[others[1]]}
		capRead.matched = []int{r.matched[others[0]], r.matched[others[1]]}
		capSel, selected, ok := capRead.partialSelectedLoop()
		if !ok {
			continue
		}
		singleCall := r.call
		singleCall.edges = []*Edge{r.call.edges[single]}
		singleRead := *r
		singleRead.call, singleRead.matched = singleCall, []int{r.matched[single]}
		if !singleRead.singleStraightEdges() {
			continue
		}
		// Only a pair of neighbouring walks at the shared vertex can close
		// against the arc made by the first blend.
		adjacent := false
		for i, on := range selected {
			if on && selected[(i+1)%len(selected)] {
				adjacent = true
			}
		}
		if !adjacent {
			continue
		}
		step, err := brepBlendEdges(ctx, d, r.bp, singleCall)
		if err != nil {
			return nil, true, err
		}
		bp, ok := step.payload.(brepPayload)
		if !ok {
			return nil, true, fmt.Errorf(`%w: the first corner blend produced no brep record`, ErrUnsupported)
		}
		var face *Face
		for _, candidate := range step.Faces() {
			for _, origin := range candidate.origins {
				if origin.Role == bp.faces[capSel.face].role {
					face = candidate
				}
			}
		}
		if face == nil || capSel.loop >= len(face.loops) {
			return nil, true, fmt.Errorf(`%w: the rounded corner lost its cap face`, ErrUnsupported)
		}
		loopEdges := face.loops[capSel.loop].Edges()
		var shortened [2]*Edge
		var near [2]*Vertex
		for j, originalIndex := range others {
			original := r.call.edges[originalIndex]
			far := original.start
			if far == corner {
				far = original.end
			}
			oldDir := original.end.position.Sub(original.start.position)
			for _, edge := range loopEdges {
				if _, ok := edge.curve.(Line3); !ok {
					continue
				}
				other := edge.start
				switch {
				case edge.start.position.Sub(far.position).Len() <= 1e-6:
					other = edge.end
				case edge.end.position.Sub(far.position).Len() <= 1e-6:
				default:
					continue
				}
				newDir := edge.end.position.Sub(edge.start.position)
				if oldDir.Cross(newDir).Len() > 1e-6*oldDir.Len()*newDir.Len() {
					continue
				}
				shortened[j], near[j] = edge, other
				break
			}
			if shortened[j] == nil {
				return nil, true, fmt.Errorf(`%w: a corner cap edge has no shortened successor`, ErrUnsupported)
			}
		}
		var arc *Edge
		for _, edge := range loopEdges {
			if _, ok := edge.curve.(Arc3); !ok {
				continue
			}
			if (edge.start == near[0] && edge.end == near[1]) ||
				(edge.start == near[1] && edge.end == near[0]) {
				arc = edge
				break
			}
		}
		if arc == nil {
			return nil, true, fmt.Errorf(`%w: the rounded corner has no joining cap arc`, ErrUnsupported)
		}
		nextCall := r.call
		nextCall.edges = []*Edge{shortened[0], shortened[1], arc}
		next, err := newBrepLoopRead(ctx, bp, nextCall)
		if err != nil {
			return nil, true, err
		}
		if err := next.matchEdges(); err != nil {
			return nil, true, err
		}
		newCap, newSelected, ok := next.partialSelectedLoop()
		if !ok || newCap.face != capSel.face || newCap.loop != capSel.loop {
			return nil, true, next.refuse("SL1", `the rounded corner has no selected cap chain`)
		}
		body, err := brepFilletPartialLoop(ctx, d, bp, nextCall, newCap, newSelected)
		return body, true, err
	}
	return nil, false, nil
}

// splitVertexBlend finds complete selected planar loops and the remaining
// independent straight edges. Face and loop indices survive route E's rewrite.
func (r *brepLoopRead) splitVertexBlend() ([]brepLoopSel, []*Edge, bool, error) {
	// A box-like selection can complete loops on several pairs of opposite
	// faces. Assign one reference axis's independent edges to route E first;
	// the remaining selected edges then name their cap loops unambiguously.
	for axis := range 3 {
		var singles, loopEdges []*Edge
		var singleMatches, loopMatches []int
		for i, ei := range r.matched {
			if ei < 0 {
				return nil, nil, false, nil
			}
			owner := r.topo.uses[r.topo.edges[ei][0]]
			along := !owner.Key.Circular && owner.DirFrom[axis] != owner.DirTo[axis]
			for other := range 3 {
				if other != axis && owner.DirFrom[other] != owner.DirTo[other] {
					along = false
				}
			}
			if along {
				singles = append(singles, r.call.edges[i])
				singleMatches = append(singleMatches, ei)
			} else {
				loopEdges = append(loopEdges, r.call.edges[i])
				loopMatches = append(loopMatches, ei)
			}
		}
		if len(singles) == 0 || len(loopEdges) == 0 {
			continue
		}
		singleRead := *r
		singleRead.call.edges, singleRead.matched = singles, singleMatches
		if !singleRead.singleStraightEdges() {
			continue
		}
		loopRead := *r
		loopRead.call.edges, loopRead.matched = loopEdges, loopMatches
		loops, err := loopRead.admitLoops()
		if err == nil {
			return loops, singles, true, nil
		}
	}

	type faceLoop struct{ face, loop int }
	selected := map[faceLoop]map[int]struct{}{}
	for _, ei := range r.matched {
		if ei < 0 {
			return nil, nil, false, nil // admitLoops gives the existing SL1 reason
		}
		for _, ui := range r.topo.edges[ei] {
			u := r.topo.uses[ui]
			if u.Part != brepLoopSeg {
				continue
			}
			fl := faceLoop{u.Face, u.Loop}
			if selected[fl] == nil {
				selected[fl] = map[int]struct{}{}
			}
			selected[fl][u.Seg] = struct{}{}
		}
	}
	var loopEdges, singles []*Edge
	var loopMatches, singleMatches []int
	for i, ei := range r.matched {
		covering := 0
		for _, ui := range r.topo.edges[ei] {
			u := r.topo.uses[ui]
			if u.Part == brepLoopSeg && len(selected[faceLoop{u.Face, u.Loop}]) == len(r.topo.planar[u.Face][u.Loop]) {
				covering++
			}
		}
		if covering > 1 {
			return nil, nil, false, r.refuse("SL1", fmt.Sprintf(`%s lies on two selected complete loops`, selectedEdgeContext(i, r.call.edges[i])))
		}
		if covering == 1 {
			loopEdges = append(loopEdges, r.call.edges[i])
			loopMatches = append(loopMatches, ei)
			continue
		}
		singles = append(singles, r.call.edges[i])
		singleMatches = append(singleMatches, ei)
	}
	if len(loopEdges) == 0 || len(singles) == 0 {
		return nil, nil, false, nil
	}
	singleRead := *r
	singleRead.call.edges, singleRead.matched = singles, singleMatches
	if !singleRead.singleStraightEdges() {
		return nil, nil, false, r.refuse("SL1", `the edges outside the complete loops are not independent straight edges along reference axes`)
	}
	loopRead := *r
	loopRead.call.edges, loopRead.matched = loopEdges, loopMatches
	loops, err := loopRead.admitLoops()
	if err != nil {
		return nil, nil, false, err
	}
	return loops, singles, true, nil
}

// buildLoops is route L over the matched selection: Table LB admits the
// loops, the record is rewritten with one band of the call's kind per loop,
// and the rewritten record is built.
func (r *brepLoopRead) buildLoops(ctx context.Context, d *Document) (*Body, error) {
	call := r.call
	if call.loop == nil {
		return nil, fmt.Errorf(`%w: route L reads a %s request with no setback`, ErrUnsupported, call.op)
	}
	loops, err := r.admitLoops()
	if err != nil {
		return nil, err
	}
	return r.buildSelectedLoops(ctx, d, loops)
}

func (r *brepLoopRead) buildSelectedLoops(ctx context.Context, d *Document, loops []brepLoopSel) (*Body, error) {
	call := r.call
	out, err := r.rewriteLoopFaces(ctx, loops)
	if err != nil {
		return nil, err
	}
	body, err := evalBrepContext(ctx, d, d.nextProducerID(), out)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the %s's rewritten brep record (modify-general §4.2 step 6); selector %s matched [%s]`,
			err, call.loopKind, call.sel, selectedEdgesContext(call.edges))
	}
	if body.volume.Value.Base() <= 0 {
		return nil, fmt.Errorf(`%w: the %s's rewritten brep record encloses no volume (modify-general §4.2 step 6); selector %s matched [%s]`,
			ErrDegenerate, call.loopKind, call.sel, selectedEdgesContext(call.edges))
	}
	return body, nil
}

// newBrepLoopRead reads the receiver record bp for one route L call: its
// topology and each planar loop segment's use. Matching the selected edges is
// matchEdges's.
func newBrepLoopRead(ctx context.Context, bp brepPayload, call brepModifyRequest) (*brepLoopRead, error) {
	topo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	r := &brepLoopRead{bp: bp, topo: topo, call: call, budget: proofbound.NewWorkBudget(ctx), loopUse: map[[3]int]int{}}
	for ui, u := range topo.uses {
		if u.Part == brepLoopSeg {
			r.loopUse[[3]int{u.Face, u.Loop, u.Seg}] = ui
		}
	}
	return r, nil
}

// prismFaceViewBlend reads a prism through its brep face view for a Fillet
// of cap edges (RF3) or a Chamfer of one straight cap edge. Route E, L or V
// builds the selection or returns its specific refusal.
func prismFaceViewBlend(ctx context.Context, d *Document, pp prismPayload, call brepModifyRequest) (*Body, error) {
	bp, sourceRoles, err := brepOfPrismWithRoles(pp)
	if err != nil {
		return nil, err
	}
	if err := requireExactBrepSection(bp, call.op); err != nil {
		return nil, err
	}
	if call.asym != nil {
		call.asym.recordRoles = sourceRoles
	}
	return brepLoopRoute(ctx, d, bp, call)
}

// refuse is one route L refusal naming its row and the selection.
func (r *brepLoopRead) refuse(row, reason string) error {
	return fmt.Errorf(`%w: this evaluator %s complete loops of planar brep faces through route L only where Table LB admits them; %s (modify-general %s); selector %s matched [%s]`,
		ErrUnsupported, r.call.op, reason, row, r.call.sel, selectedEdgesContext(r.call.edges))
}

// partner is the other use of use ui's edge.
func (r *brepLoopRead) partner(ui int) int {
	pair := r.topo.edges[r.topo.edgeOf[ui]]
	if pair[0] == ui {
		return pair[1]
	}
	return pair[0]
}

// matchEdges matches each selected edge to the one record edge whose ends,
// lifted through the reference frame and placement, lie within 1e-6 of its
// vertices: a line to a line, an arc to an arc about the same centre, and a
// whole circle to a circle of the same centre, axis and radius
// (modify-general §4.1). The match identifies the edge and admits no
// geometry. A band boundary edge of an earlier route L call pairs with no
// record face and matches nothing.
func (r *brepLoopRead) matchEdges() error {
	refView := r.bp.refView()
	const tol = 1e-6
	r.matched = make([]int, len(r.call.edges))
	for i, e := range r.call.edges {
		r.matched[i] = -1
		for ei, pair := range r.topo.edges {
			if err := survey2d.WallBudgetStep(r.budget); err != nil {
				return err
			}
			if !brepEdgeMatches(refView, r.topo.uses[pair[0]], e, tol) {
				continue
			}
			if r.matched[i] != -1 {
				r.matched[i] = -2
				break
			}
			r.matched[i] = ei
		}
	}
	return nil
}

// brepEdgeMatches reports whether body edge e is the record edge owner names,
// within tol after lifting through refView.
func brepEdgeMatches(refView prismPayload, owner brepUse, e *Edge, tol float64) bool {
	if e.start == nil || e.end == nil {
		return false
	}
	lift := func(c [3]float64) r3.Vec { return refView.point(c[0], c[1], c[2]) }
	key := owner.Key
	switch c := e.curve.(type) {
	case Line3:
		if key.Circular {
			return false
		}
	case Arc3:
		if !key.Circular || key.Closed || lift(key.C).Sub(c.Center).Len() > tol {
			return false
		}
	case Circle3:
		if !key.Closed || lift(key.C).Sub(c.Center).Len() > tol || math.Abs(c.Radius.Base()-key.Radius) > tol {
			return false
		}
		var unit [3]float64
		unit[key.Axis] = 1
		axis := refView.dir(unit[0], unit[1], unit[2])
		return axis.Cross(c.Axis).Len() <= tol
	default:
		return false
	}
	return matchEndpoints(e.start.position, e.end.position, lift(owner.DirFrom), lift(owner.DirTo), tol)
}

// singleStraightEdges reports whether the selection is route E's class: every
// edge matched one straight record edge along a reference axis (Table EB's
// EB1), and no two share a vertex (EB7).
func (r *brepLoopRead) singleStraightEdges() bool {
	seen := map[[3]float64]struct{}{}
	for _, ei := range r.matched {
		if ei < 0 {
			return false
		}
		owner := r.topo.uses[r.topo.edges[ei][0]]
		if owner.Key.Circular {
			return false
		}
		differ := 0
		for i := range 3 {
			if owner.DirFrom[i] != owner.DirTo[i] {
				differ++
			}
		}
		if differ != 1 {
			return false
		}
		for _, v := range [2][3]float64{owner.DirFrom, owner.DirTo} {
			if _, ok := seen[v]; ok {
				return false
			}
			seen[v] = struct{}{}
		}
	}
	return true
}

// admitLoops is LB1 and LB2 (SL1): the selected edges are exactly the
// segments of one or more complete loops of planar faces, and no two of those
// loops share an edge. A loop is complete when every one of its segments is
// selected; every selected edge must lie on a complete loop, and on one only.
// The loops are returned in the order the selection first reaches them. A
// loop on a face with a section displacement is SB1, which the brep route
// already refused for the whole record.
func (r *brepLoopRead) admitLoops() ([]brepLoopSel, error) {
	type faceLoop struct{ face, loop int }
	selected := map[faceLoop]map[int]struct{}{}
	loopsOf := make([][]faceLoop, len(r.call.edges))
	for i, ei := range r.matched {
		switch ei {
		case -1:
			return nil, r.refuse("SL1", fmt.Sprintf(`%s matches no edge of the receiver's face record`, selectedEdgeContext(i, r.call.edges[i])))
		case -2:
			return nil, r.refuse("SL1", fmt.Sprintf(`%s matches more than one edge of the receiver's face record`, selectedEdgeContext(i, r.call.edges[i])))
		}
		for _, ui := range r.topo.edges[ei] {
			u := r.topo.uses[ui]
			if u.Part != brepLoopSeg {
				continue
			}
			fl := faceLoop{u.Face, u.Loop}
			loopsOf[i] = append(loopsOf[i], fl)
			if selected[fl] == nil {
				selected[fl] = map[int]struct{}{}
			}
			selected[fl][u.Seg] = struct{}{}
		}
	}
	complete := func(fl faceLoop) bool { return len(selected[fl]) == len(r.topo.planar[fl.face][fl.loop]) }
	chosen := map[faceLoop]struct{}{}
	var out []brepLoopSel
	for i := range r.call.edges {
		var covering []faceLoop
		for _, fl := range loopsOf[i] {
			if complete(fl) {
				covering = append(covering, fl)
			}
		}
		switch {
		case len(loopsOf[i]) == 0:
			return nil, r.refuse("SL1", fmt.Sprintf(`%s lies on no loop of a planar face, so it belongs to no complete loop; a %s of loops mixed with single edges needs a corner patch in no reference frame`, selectedEdgeContext(i, r.call.edges[i]), r.call.loopKind))
		case len(covering) == 0:
			return nil, r.refuse("SL1", fmt.Sprintf(`the selection covers only part of a loop of brep face %s; every geometric edge of a complete loop must be selected`, r.bp.faces[loopsOf[i][0].face].role))
		case len(covering) > 1:
			return nil, r.refuse("SL1", fmt.Sprintf(`%s lies on two selected complete loops, of brep faces %s and %s, which share it`,
				selectedEdgeContext(i, r.call.edges[i]), r.bp.faces[covering[0].face].role, r.bp.faces[covering[1].face].role))
		}
		fl := covering[0]
		if _, ok := chosen[fl]; ok {
			continue
		}
		chosen[fl] = struct{}{}
		out = append(out, brepLoopSel{face: fl.face, loop: fl.loop})
	}
	return out, nil
}

// naturalFaceLoops is LB4 over one face: every segment runs over its natural
// range, and no loop of a planar face holds two consecutive segments on one
// carrier (route E's EB5). It returns the reason a face fails, or "".
func (r *brepLoopRead) naturalFaceLoops(fi int) (string, error) {
	f := r.bp.faces[fi]
	if !f.planar() {
		if !naturalRange(f.wall) {
			return fmt.Sprintf(`brep face %s's wall runs over a narrowed range`, f.role), nil
		}
		return "", nil
	}
	loops, err := profileCornerLoopsBudget(r.budget, *f.region)
	if err != nil {
		return "", err
	}
	for li, loop := range append([]loopRecord{f.region.Outer}, f.region.Holes...) {
		for _, seg := range loop.Segments {
			if !naturalRange(seg) {
				return fmt.Sprintf(`brep face %s holds a segment over a narrowed range`, f.role), nil
			}
		}
		walks := loops[li].walks
		if len(walks) != len(loop.Segments) {
			return fmt.Sprintf(`brep face %s holds two consecutive segments on one line`, f.role), nil
		}
		n := len(walks)
		for i, w := range walks {
			next := walks[(i+1)%n]
			if n > 1 && w.IsCircular() && next.IsCircular() && !w.Closed && !next.Closed &&
				w.CU == next.CU && w.CV == next.CV && w.Radius == next.Radius && (w.Th1 > w.Th0) == (next.Th1 > next.Th0) {
				return fmt.Sprintf(`brep face %s holds two consecutive arcs on one circle`, f.role), nil
			}
		}
	}
	return "", nil
}

// classifyLoop is LB3, LB4 and LB6 (SL2) on one touched loop of face F,
// whose normal runs along reference axis n. Each segment's edge pairs with a
// face that is (sw), swept along n with the segment as a rim over its natural
// range and no side-line split, or (pl), planar across another axis, holding
// the segment as a loop segment whose two neighbours are lines along n. Every
// such face, and F, passes LB4. The faces' far ends must all lie on one side
// of F (LB6): against F's outward normal the walls descend into the body
// (sigma −1), along it they rise off F (sigma +1).
func (r *brepLoopRead) classifyLoop(sel *brepLoopSel) error {
	f := r.bp.faces[sel.face]
	eF := r.topo.embeds[sel.face]
	n := eF.Axis[2]
	level := eF.Canon(0, 0, f.z0)[n]
	outward := -eF.Sign[2]
	if f.outward {
		outward = eF.Sign[2]
	}
	refuse := func(reason string) error { return r.refuse("SL2", reason) }
	why, err := r.naturalFaceLoops(sel.face)
	if err != nil {
		return err
	}
	if why != "" {
		return refuse(why)
	}
	segs := len(r.topo.planar[sel.face][sel.loop])
	sel.beside = make([]brepLoopBeside, segs)
	for s := range segs {
		if err := survey2d.WallBudgetStep(r.budget); err != nil {
			return err
		}
		pu := r.topo.uses[r.partner(r.loopUse[[3]int{sel.face, sel.loop, s}])]
		a := r.bp.faces[pu.Face]
		eA := r.topo.embeds[pu.Face]
		b := brepLoopBeside{use: r.partner(r.loopUse[[3]int{sel.face, sel.loop, s}])}
		switch {
		case !a.planar() && eA.Axis[2] == n && brepIsRim(pu.Part):
			if len(a.side0) > 0 || len(a.side1) > 0 {
				return refuse(fmt.Sprintf(`brep face %s beside the loop has a split side line`, a.role))
			}
			far := a.z1
			if pu.Part == brepRim1 {
				far = a.z0
			}
			b.far = []float64{eA.Sign[2]*far + 0}
		case a.planar() && eA.Axis[2] != n && pu.Part == brepLoopSeg:
			b.planar = true
			nA := len(r.topo.planar[pu.Face][pu.Loop])
			for k, s2 := range [2]int{(pu.Seg + nA - 1) % nA, (pu.Seg + 1) % nA} {
				ui := r.loopUse[[3]int{pu.Face, pu.Loop, s2}]
				nu := r.topo.uses[ui]
				far, ok := brepLineAlong(nu, n, level)
				if !ok {
					return refuse(fmt.Sprintf(`brep face %s beside the loop continues past the loop's vertex on a segment that is not a straight line along the face's normal`, a.role))
				}
				b.far = append(b.far, far)
				b.nb[k] = ui
			}
		default:
			return refuse(fmt.Sprintf(`brep face %s beside the loop neither sweeps along brep face %s's normal nor holds the loop's edge as a plane across another axis`, a.role, f.role))
		}
		why, err := r.naturalFaceLoops(pu.Face)
		if err != nil {
			return err
		}
		if why != "" {
			return refuse(why)
		}
		for _, far := range b.far {
			sense := -1.0
			if (far > level) == (outward > 0) {
				sense = 1
			}
			if sel.sigma == 0 {
				sel.sigma = sense
			}
			if sel.sigma != sense {
				return refuse(fmt.Sprintf(`the faces beside brep face %s's loop leave it on both sides`, f.role))
			}
		}
		sel.beside[s] = b
	}
	return nil
}

// brepLineAlong reads one neighbouring segment of a (pl) face: a straight
// line whose ends differ only in reference coordinate n, one end at level.
// It returns the other end's coordinate along n.
func brepLineAlong(u brepUse, n int, level float64) (float64, bool) {
	if !u.Walk.IsLine() {
		return 0, false
	}
	for i := range 3 {
		if i != n && u.From[i] != u.To[i] {
			return 0, false
		}
	}
	switch {
	case u.From[n] == level && u.To[n] != level:
		return u.To[n], true
	case u.To[n] == level && u.From[n] != level:
		return u.From[n], true
	default:
		return 0, false
	}
}

func (r *brepLoopRead) refuseAsymmetricLoop(row, reason string) error {
	return fmt.Errorf(`%w: %s (modify-reach %s); selector %s matched [%s]`, ErrUnsupported,
		reason, row, r.call.sel, selectedEdgesContext(r.call.edges))
}

// assignLoopSetbacks maps the public reference face of each selected edge to
// its loop face or its neighbouring record face. Each loop and each material
// side of one face must use one cap/side pair for the shared cap-blend view.
func (r *brepLoopRead) assignLoopSetbacks(sels []brepLoopSel) error {
	for i := range sels {
		sels[i].setback = *r.call.loop
	}
	if r.call.asym == nil {
		return nil
	}
	selected := make(map[int]*Edge, len(r.matched))
	for i, ei := range r.matched {
		selected[ei] = r.call.edges[i]
	}
	seen := map[[2]int]capSetback{}
	for i := range sels {
		sel := &sels[i]
		assignment := -1
		for segment, beside := range sel.beside {
			ui := r.loopUse[[3]int{sel.face, sel.loop, segment}]
			edge := selected[r.topo.edgeOf[ui]]
			ref := r.call.asym.refs[edge]
			capFace := r.call.asym.matchesRecordFace(ref, sel.face)
			sideFace := r.call.asym.matchesRecordFace(ref, r.topo.uses[beside.use].Face)
			if capFace == sideFace {
				return r.refuseAsymmetricLoop("SX16", fmt.Sprintf(`the reference for brep face %s's loop segment %d has no single adjacent record-face identity`,
					r.bp.faces[sel.face].role, segment))
			}
			choice := 0
			if sideFace {
				choice = 1
			}
			if assignment >= 0 && assignment != choice {
				return r.refuseAsymmetricLoop("SX4", fmt.Sprintf(`brep face %s's loop mixes cap and side reference assignments`,
					r.bp.faces[sel.face].role))
			}
			assignment = choice
		}
		if assignment < 0 {
			return r.refuseAsymmetricLoop("SX16", fmt.Sprintf(`brep face %s's loop has no edges to assign a reference face`,
				r.bp.faces[sel.face].role))
		}
		if assignment == 0 {
			sel.setback = capSetback{dc: r.call.asym.d, dcDelta: r.call.asym.dDelta,
				ds: r.call.asym.other, dsDelta: r.call.asym.otherDelta}
		} else {
			sel.setback = capSetback{dc: r.call.asym.other, dcDelta: r.call.asym.otherDelta,
				ds: r.call.asym.d, dsDelta: r.call.asym.dDelta}
		}
		f := r.bp.faces[sel.face]
		band := brepLoopBand{sigma: sel.sigma}
		side := 0
		if band.matSign(f) > 0 {
			side = 1
		}
		key := [2]int{sel.face, side}
		if old, ok := seen[key]; ok && old != sel.setback {
			return r.refuseAsymmetricLoop("SX4", fmt.Sprintf(`brep face %s's loops on one material side use different cap and side setbacks`,
				f.role))
		}
		seen[key] = sel.setback
	}
	return nil
}

// requireBandReach is LB5 (reach SX7): the band reaches ds along every face
// beside its loop and must stop short of that face's far end. The reach is
// summed per face end: a (sw) face trimmed from both ends, or a (pl) face's
// neighbouring line claimed from both, must keep a part of its own.
func (r *brepLoopRead) requireBandReach(sels []brepLoopSel) error {
	type piece struct{ face, use int }
	reach := map[piece]float64{}
	for _, sel := range sels {
		f := r.bp.faces[sel.face]
		eF := r.topo.embeds[sel.face]
		level := eF.Canon(0, 0, f.z0)[eF.Axis[2]]
		for _, b := range sel.beside {
			a := r.bp.faces[r.topo.uses[b.use].Face]
			for k, far := range b.far {
				key := piece{face: r.topo.uses[b.use].Face, use: -1}
				height := math.Abs(far - level)
				if b.planar {
					key.use = b.nb[k]
				}
				reach[key] += sel.setback.ds
				if reach[key] >= height {
					return fmt.Errorf(`%w: the %s band on brep face %s's loop reaches or passes the far end of brep face %s beside it (%g mm of %g mm); a merging kernel is not available (modify-reach SX7); selector %s matched [%s]`,
						ErrUnsupported, r.call.loopKind, f.role, a.role, reach[key], height, r.call.sel, selectedEdgesContext(r.call.edges))
				}
			}
		}
	}
	return nil
}

// loopFaceView is the cap blend view of one face F and its touched loops:
// F's region and level, each loop chamfered on the cap its band's material
// sense names, both caps at the call's setbacks. The shipped cap-loop gates
// read F's loops through it exactly as they read a prism's cap loops.
func (r *brepLoopRead) loopFaceView(fi int, sels []brepLoopSel) capBlendPayload {
	f := r.bp.faces[fi]
	cbp := capBlendPayload{
		profile: *f.region, frame: f.frame, xform: r.bp.xform,
		z0: f.z0, z1: f.z0, z0Delta: f.z0Delta, z1Delta: f.z0Delta,
		start: *r.call.loop, end: *r.call.loop,
		startLoops: map[int]bool{}, endLoops: map[int]bool{},
		fillet: r.call.loopKind == brepBandFillet,
	}
	for _, sel := range sels {
		if sel.face != fi {
			continue
		}
		band := brepLoopBand{sigma: sel.sigma, setback: sel.setback}
		if band.matSign(f) > 0 {
			cbp.startLoops[sel.loop] = true
			cbp.start = sel.setback
		} else {
			cbp.endLoops[sel.loop] = true
			cbp.end = sel.setback
		}
	}
	return cbp
}

// rewriteLoopFaces is modify-general §4.2 steps 1 to 5 over every touched
// loop, after Table LB's topology gates (stage 3: LB3, LB4, LB6, then LB5). Per
// face F holding touched loops, stage 4 builds each cap contour (SX6) and
// decides SX13's axial and radial halves, and a fillet's corners take Table
// LF's classes (loop-fillet SF1); stage 5 runs SX14, then F's
// rewritten region faces modify §5's audit (S8, S7 — the contour against F's
// other loops, read as SX7/SX12 — and S9) at the setback and at the top of its
// conversion span. Each (sw) face's level at F moves to the side level, and
// each (pl) face's segment on the loop moves there along F's normal with its
// two neighbouring lines shortened by d, the connector-free trim route E
// applies, and the (pl) face then faces the same audit (S8, S6, S7, S9). F's
// section displacement takes every touched loop's contour displacement
// (modify-general §7). The result keeps the receiver's bands and records one
// more per touched loop.
func (r *brepLoopRead) rewriteLoopFaces(ctx context.Context, sels []brepLoopSel) (brepPayload, error) {
	for i := range sels {
		if err := r.classifyLoop(&sels[i]); err != nil {
			return brepPayload{}, err
		}
	}
	if err := r.assignLoopSetbacks(sels); err != nil {
		return brepPayload{}, err
	}
	if err := r.requireBandReach(sels); err != nil {
		return brepPayload{}, err
	}
	var faceOrder []int
	for _, sel := range sels {
		if !slices.Contains(faceOrder, sel.face) {
			faceOrder = append(faceOrder, sel.face)
		}
	}

	// Stage 4: the contours exist (SX6), and the setback survives float64 at
	// each side level (SX13's axial half) and each circular wall's radius
	// (its radial half).
	mixed := make(map[int]profileRecord, len(faceOrder))
	views := make(map[int]capBlendPayload, len(faceOrder))
	for _, fi := range faceOrder {
		cbp := r.loopFaceView(fi, sels)
		views[fi] = cbp
		profile, err := mixedOffsetProfile(r.budget, cbp)
		if err != nil {
			return brepPayload{}, r.wrapFace(fi, wrapCapBlendDropError(err))
		}
		mixed[fi] = profile
		if err := requireCapBlendLevelsSeparate(cbp); err != nil {
			return brepPayload{}, r.wrapFace(fi, err)
		}
	}
	work := freeform.NewFreeformWork()
	for _, sel := range sels {
		setback := sel.setback
		cl, err := oneLoopCornerLoop(r.budget, r.bp.faces[sel.face].regionLoop(sel.loop), work)
		if err != nil {
			return brepPayload{}, err
		}
		if len(cl.walks) == 1 && cl.walks[0].Closed {
			_, err = capband.BandRadius(cl.walks[0], setback.dc, shellTol)
		} else {
			_, err = views[sel.face].offsetJoins(r.budget, sel.loop, cl, setback.dc)
		}
		if err != nil {
			return brepPayload{}, r.wrapFace(sel.face, err)
		}
		// SF1 (loop-fillet §6): every corner of a fillet's loop takes a class of
		// Table LF.
		if r.call.loopKind == brepBandFillet {
			f := r.bp.faces[sel.face]
			if _, err := filletLoopOf(r.budget, f.regionLoop(sel.loop), setback.dc, f.role, work); err != nil {
				return brepPayload{}, r.wrapFace(sel.face, err)
			}
		}
	}

	// Stage 5: SX14 at every miter corner beside a circular wall, then each
	// face's rewritten region.
	for _, fi := range faceOrder {
		if err := requireCapBlendCornerLoci(r.budget, views[fi]); err != nil {
			return brepPayload{}, r.wrapFace(fi, err)
		}
		region := *r.bp.faces[fi].region
		if err := auditOffsetSectionBudget(r.budget, region, mixed[fi]); err != nil {
			return brepPayload{}, r.wrapFace(fi, wrapCapBlendAuditError(err))
		}
		if err := auditCapBlendSetbackSpan(r.budget, region, views[fi]); err != nil {
			return brepPayload{}, r.wrapFace(fi, err)
		}
	}

	faces := slices.Clone(r.bp.faces)
	for _, fi := range faceOrder {
		region := mixed[fi]
		faces[fi].region = &region
	}
	trims := map[int][]map[int]*cornerBlend{}
	bands := slices.Clone(r.bp.loopBands)
	for _, sel := range sels {
		setback := sel.setback
		f := r.bp.faces[sel.face]
		eF := r.topo.embeds[sel.face]
		n := eF.Axis[2]
		band := brepLoopBand{face: sel.face, loop: sel.loop, orig: cloneLoopRecord(f.regionLoop(sel.loop)), setback: setback, sigma: sel.sigma, kind: r.call.loopKind}
		delta, err := views[sel.face].loopContourDelta(ctx, sel.loop, band.orig, setback.dc, setback.dcDelta)
		if err != nil {
			return brepPayload{}, r.wrapFace(sel.face, err)
		}
		faces[sel.face].delta = math.Max(faces[sel.face].delta, delta)
		sideZ, sideDelta := band.sideLevel(f)
		sideRef := eF.Canon(0, 0, sideZ)[n]
		for _, b := range sel.beside {
			pu := r.topo.uses[b.use]
			eA := r.topo.embeds[pu.Face]
			a := &faces[pu.Face]
			if !b.planar {
				local := eA.Sign[2]*sideRef + 0
				if pu.Part == brepRim0 {
					a.z0, a.z0Delta = local, proofbound.AbsSumUpper(math.Max(a.z0Delta, f.z0Delta), sideDelta)
				} else {
					a.z1, a.z1Delta = local, proofbound.AbsSumUpper(math.Max(a.z1Delta, f.z0Delta), sideDelta)
				}
				continue
			}
			if err := r.trimPlanar(trims, pu, n, sideRef, setback.ds); err != nil {
				return brepPayload{}, err
			}
			a.delta = math.Max(a.delta, sideDelta)
		}
		bands = append(bands, band)
	}

	// The (pl) faces' audit, after every face F's.
	var planarOrder []int
	for fi := range trims {
		planarOrder = append(planarOrder, fi)
	}
	slices.Sort(planarOrder)
	for _, fi := range planarOrder {
		f := r.bp.faces[fi]
		loops, err := profileCornerLoopsBudget(r.budget, *f.region)
		if err != nil {
			return brepPayload{}, err
		}
		profile, _, err := rewriteProfileBudget(r.budget, *f.region, loops, trims[fi])
		if err != nil {
			return brepPayload{}, err
		}
		if err := auditRewriteBudget(r.budget, *f.region, profile, loops, trims[fi]); err != nil {
			return brepPayload{}, r.wrapFace(fi, renderAuditCoordinates(err))
		}
		faces[fi].region = &profile
	}
	out := brepPayload{faces: faces, xform: r.bp.xform, loopBands: bands}
	out.assignRoles()
	return out, nil
}

// trimPlanar marks one (pl) face's trim: the segment use pu walks moves to
// the side level along reference axis n, as a connector-free blend at each
// of its two corners whose foot is the moved end and whose cutback on the
// neighbouring line is d. A corner two bands claim is SL2.
func (r *brepLoopRead) trimPlanar(trims map[int][]map[int]*cornerBlend, pu brepUse, n int, sideRef, d float64) error {
	f := r.bp.faces[pu.Face]
	loops, err := profileCornerLoopsBudget(r.budget, *f.region)
	if err != nil {
		return err
	}
	if trims[pu.Face] == nil {
		trims[pu.Face] = make([]map[int]*cornerBlend, len(loops))
		for i := range trims[pu.Face] {
			trims[pu.Face][i] = map[int]*cornerBlend{}
		}
	}
	e := r.topo.embeds[pu.Face]
	walks := loops[pu.Loop].walks
	count := len(walks)
	seg := walks[pu.Seg]
	for c, at := range [2][2]float64{{seg.StartU, seg.StartV}, {seg.EndU, seg.EndV}} {
		moved := e.Canon(at[0], at[1], f.z0)
		moved[n] = sideRef
		local := e.Local(moved)
		p := Point2{U: local[0], V: local[1]}
		trim := &cornerBlend{FA: p, FB: p}
		ci := pu.Seg
		if c == 0 {
			trim.CutbackA = d
		} else {
			trim.CutbackB = d
			ci = (pu.Seg + 1) % count
		}
		if trims[pu.Face][pu.Loop][ci] != nil {
			return r.refuse("SL2", fmt.Sprintf(`a corner of brep face %s beside the loop is claimed by two bands`, f.role))
		}
		trims[pu.Face][pu.Loop][ci] = trim
	}
	return nil
}

// wrapFace names the face a refusal of one face's rewrite came from and the
// selection, passing a context error through unwrapped.
func (r *brepLoopRead) wrapFace(fi int, err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return fmt.Errorf(`%w; in brep face %s; selector %s matched [%s]`,
		err, r.bp.faces[fi].role, r.call.sel, selectedEdgesContext(r.call.edges))
}
