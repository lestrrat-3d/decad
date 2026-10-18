package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/breploop"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
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
// (internal/breprecord/band.go, and fillet.go for a fillet band).

// brepLoopRead joins the private route L record proof to the public call.
type brepLoopRead struct {
	state *breploop.State
	call  brepModifyRequest
}

// brepLoopSelection reads selected edges in order from one matched record.
// Its ordinals are private to the view; the reader's matches and budget are
// shared, and constructing a subset never matches an edge again.
type brepLoopSelection struct {
	read     *brepLoopRead
	ordinals []int
}

func (r *brepLoopRead) fullSelection() brepLoopSelection {
	ordinals := make([]int, len(r.call.edges))
	for i := range ordinals {
		ordinals[i] = i
	}
	return brepLoopSelection{read: r, ordinals: ordinals}
}

func (r *brepLoopRead) selection(ordinals []int) brepLoopSelection {
	return brepLoopSelection{read: r, ordinals: slices.Clone(ordinals)}
}

func (s brepLoopSelection) len() int { return len(s.ordinals) }

func (s brepLoopSelection) edge(i int) *Edge { return s.read.call.edges[s.ordinals[i]] }

func (s brepLoopSelection) match(i int) int { return s.read.state.Matched[s.ordinals[i]] }

func (s brepLoopSelection) edges() []*Edge {
	edges := make([]*Edge, s.len())
	for i := range edges {
		edges[i] = s.edge(i)
	}
	return edges
}

// brepLoopSel is one touched loop of Table LB: the loop of index loop on the
// planar face of index face, what lies beside each of its segments (LB3),
// and the band's sense (LB6), −1 for walls descending into the body and +1
// for walls rising off it.
type brepLoopSel = breploop.Sel

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
	if r.fullSelection().singleStraightEdges() {
		body, err := brepBlendEdges(ctx, d, bp, call)
		var mismatch *brepMaterialWedgeMismatchError
		if len(call.edges) == 1 && call.loopKind == breprecord.BandFillet && errors.As(err, &mismatch) {
			if sel, selected, ok := r.fullSelection().partialSelectedLoop(); ok {
				return brepFilletPartialLoop(ctx, d, bp, call, sel, selected)
			}
		}
		if len(call.edges) == 1 && call.loopKind == breprecord.BandChamfer && call.asym == nil &&
			errors.As(err, &mismatch) {
			if sel, selected, ok := r.fullSelection().partialSelectedLoop(); ok {
				return brepChamferPartialLoop(ctx, d, bp, call, sel, selected)
			}
		}
		return body, err
	}
	if call.asym != nil {
		loops, err := r.fullSelection().admitLoops()
		if err != nil {
			return nil, fmt.Errorf(`%w: an asymmetric chamfer on a brep requires independent straight edges or complete planar-face loops (modify-reach SX16): %v`, ErrUnsupported, err)
		}
		return r.buildSelectedLoops(ctx, d, loops)
	}
	if call.loopKind == breprecord.BandFillet {
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
			rewritten, ok := step.n.Payload.(brepPayload)
			if !ok {
				return nil, fmt.Errorf(`%w: the straight-edge fillet produced no brep record`, ErrUnsupported)
			}
			next, err := newBrepLoopRead(ctx, rewritten, call)
			if err != nil {
				return nil, err
			}
			return next.buildSelectedLoops(ctx, d, loops)
		}
		if sel, selected, ok := r.fullSelection().partialSelectedLoop(); ok {
			return brepFilletPartialLoop(ctx, d, bp, call, sel, selected)
		}
		// A pair of edges can share a swept wall without sharing a planar
		// loop in the input record. Route E's preparation restates that wall;
		// the same partial-loop builder can then close their common corner.
		if selectedStraightEdgesShareVertex(call.edges) {
			er, blends, err := prepareBrepEdgeBlends(ctx, bp, call, true, nil)
			if err == nil {
				next, err := newBrepLoopRead(ctx, er.BP, call)
				if err != nil {
					return nil, err
				}
				if err := next.matchEdges(); err != nil {
					return nil, err
				}
				if sel, selected, ok := next.fullSelection().partialSelectedLoop(); ok {
					selectedAt := map[[3]float64]int{}
					for _, eb := range blends {
						for _, v := range eb.V {
							selectedAt[v]++
						}
					}
					for _, eb := range blends {
						for k, v := range eb.V {
							eb.Terminal[k] = selectedAt[v] == 1
						}
					}
					return next.buildPartialFillet(ctx, d, sel, selected, blends)
				}
			}
		}
	}
	if call.loopKind == breprecord.BandChamfer && len(call.edges) == 2 {
		if sel, selected, ok := r.fullSelection().partialSelectedLoop(); ok {
			return brepChamferPartialLoop(ctx, d, bp, call, sel, selected)
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
	for _, v := range [2]*Vertex{vertexHandle(r.call.edges[0].n.Start), vertexHandle(r.call.edges[0].n.End)} {
		if v != nil && (vertexHandle(r.call.edges[1].n.Start) == v || vertexHandle(r.call.edges[1].n.End) == v) &&
			(vertexHandle(r.call.edges[2].n.Start) == v || vertexHandle(r.call.edges[2].n.End) == v) {
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
		capSelection := r.selection(others[:])
		capSel, selected, ok := capSelection.partialSelectedLoop()
		if !ok {
			continue
		}
		singleSelection := r.selection([]int{single})
		singleCall := r.call
		singleCall.edges = singleSelection.edges()
		if !singleSelection.singleStraightEdges() {
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
		step, err := brepBlendEdges(ctx, d, r.state.Payload, singleCall)
		if err != nil {
			return nil, true, err
		}
		bp, ok := step.n.Payload.(brepPayload)
		if !ok {
			return nil, true, fmt.Errorf(`%w: the first corner blend produced no brep record`, ErrUnsupported)
		}
		var face *Face
		for _, candidate := range step.Faces() {
			for _, origin := range candidate.n.Origins {
				if origin.Role == bp.Faces[capSel.Face].Role {
					face = candidate
				}
			}
		}
		if face == nil || capSel.Loop >= len(face.n.Loops) {
			return nil, true, fmt.Errorf(`%w: the rounded corner lost its cap face`, ErrUnsupported)
		}
		loopEdges := loopEdges(face.n.Loops[capSel.Loop])
		var shortened [2]*Edge
		var near [2]*Vertex
		for j, originalIndex := range others {
			original := r.call.edges[originalIndex]
			far := vertexHandle(original.n.Start)
			if far == corner {
				far = vertexHandle(original.n.End)
			}
			oldDir := original.n.End.Position.Sub(original.n.Start.Position)
			for _, edge := range loopEdges {
				if _, ok := edge.n.Curve.(surfacegeom.Line3); !ok {
					continue
				}
				other := vertexHandle(edge.n.Start)
				switch {
				case edge.n.Start.Position.Sub(far.n.Position).Len() <= 1e-6:
					other = vertexHandle(edge.n.End)
				case edge.n.End.Position.Sub(far.n.Position).Len() <= 1e-6:
				default:
					continue
				}
				newDir := edge.n.End.Position.Sub(edge.n.Start.Position)
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
			if _, ok := edge.n.Curve.(surfacegeom.Arc3); !ok {
				continue
			}
			if (vertexHandle(edge.n.Start) == near[0] && vertexHandle(edge.n.End) == near[1]) ||
				(vertexHandle(edge.n.Start) == near[1] && vertexHandle(edge.n.End) == near[0]) {
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
		newCap, newSelected, ok := next.fullSelection().partialSelectedLoop()
		if !ok || newCap.Face != capSel.Face || newCap.Loop != capSel.Loop {
			return nil, true, next.state.Refuse("SL1", `the rounded corner has no selected cap chain`)
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
		var singleOrdinals, loopOrdinals []int
		for i, ei := range r.state.Matched {
			if ei < 0 {
				return nil, nil, false, nil
			}
			owner := r.state.Topology.Uses[r.state.Topology.Edges[ei][0]]
			along := !owner.Key.Circular && owner.DirFrom[axis] != owner.DirTo[axis]
			for other := range 3 {
				if other != axis && owner.DirFrom[other] != owner.DirTo[other] {
					along = false
				}
			}
			if along {
				singleOrdinals = append(singleOrdinals, i)
			} else {
				loopOrdinals = append(loopOrdinals, i)
			}
		}
		if len(singleOrdinals) == 0 || len(loopOrdinals) == 0 {
			continue
		}
		singleSelection := r.selection(singleOrdinals)
		if !singleSelection.singleStraightEdges() {
			continue
		}
		loopSelection := r.selection(loopOrdinals)
		loops, err := loopSelection.admitLoops()
		if err == nil {
			return loops, singleSelection.edges(), true, nil
		}
	}

	type faceLoop struct{ face, loop int }
	selected := map[faceLoop]map[int]struct{}{}
	for _, ei := range r.state.Matched {
		if ei < 0 {
			return nil, nil, false, nil // admitLoops gives the existing SL1 reason
		}
		for _, ui := range r.state.Topology.Edges[ei] {
			u := r.state.Topology.Uses[ui]
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
	var loopOrdinals, singleOrdinals []int
	for i, ei := range r.state.Matched {
		covering := 0
		for _, ui := range r.state.Topology.Edges[ei] {
			u := r.state.Topology.Uses[ui]
			if u.Part == brepLoopSeg && len(selected[faceLoop{u.Face, u.Loop}]) == len(r.state.Topology.Planar[u.Face][u.Loop]) {
				covering++
			}
		}
		if covering > 1 {
			return nil, nil, false, r.state.Refuse("SL1", fmt.Sprintf(`%s lies on two selected complete loops`, selectedEdgeContext(i, r.call.edges[i])))
		}
		if covering == 1 {
			loopOrdinals = append(loopOrdinals, i)
			continue
		}
		singleOrdinals = append(singleOrdinals, i)
	}
	if len(loopOrdinals) == 0 || len(singleOrdinals) == 0 {
		return nil, nil, false, nil
	}
	singleSelection := r.selection(singleOrdinals)
	if !singleSelection.singleStraightEdges() {
		return nil, nil, false, r.state.Refuse("SL1", `the edges outside the complete loops are not independent straight edges along reference axes`)
	}
	loopSelection := r.selection(loopOrdinals)
	loops, err := loopSelection.admitLoops()
	if err != nil {
		return nil, nil, false, err
	}
	return loops, singleSelection.edges(), true, nil
}

// buildLoops is route L over the matched selection: Table LB admits the
// loops, the record is rewritten with one band of the call's kind per loop,
// and the rewritten record is built.
func (r *brepLoopRead) buildLoops(ctx context.Context, d *Document) (*Body, error) {
	call := r.call
	if call.loop == nil {
		return nil, fmt.Errorf(`%w: route L reads a %s request with no setback`, ErrUnsupported, call.op)
	}
	loops, err := r.fullSelection().admitLoops()
	if err != nil {
		return nil, err
	}
	return r.buildSelectedLoops(ctx, d, loops)
}

func (r *brepLoopRead) buildSelectedLoops(ctx context.Context, d *Document, loops []brepLoopSel) (*Body, error) {
	call := r.call
	for i := range loops {
		if err := r.state.ClassifyLoop(&loops[i]); err != nil {
			return nil, err
		}
	}
	if err := r.assignLoopSetbacks(loops); err != nil {
		return nil, err
	}
	if err := r.state.RequireBandReach(loops); err != nil {
		return nil, err
	}
	out, err := r.state.RewriteLoopFaces(ctx, loops)
	if err != nil {
		return nil, err
	}
	body, err := bodyResult(breprecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), out))
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the %s's rewritten brep record (modify-general §4.2 step 6); selector %s matched [%s]`,
			err, call.loopKind, call.sel, selectedEdgesContext(call.edges))
	}
	if body.n.Volume.Value.Base() <= 0 {
		return nil, fmt.Errorf(`%w: the %s's rewritten brep record encloses no volume (modify-general §4.2 step 6); selector %s matched [%s]`,
			ErrDegenerate, call.loopKind, call.sel, selectedEdgesContext(call.edges))
	}
	return body, nil
}

// newBrepLoopRead reads the receiver record bp for one route L call: its
// topology and each planar loop segment's use. Matching the selected edges is
// matchEdges's.
func newBrepLoopRead(ctx context.Context, bp brepPayload, call brepModifyRequest) (*brepLoopRead, error) {
	state, err := breploop.New(ctx, bp, call.op, call.loopKind, call.loop)
	if err != nil {
		return nil, err
	}
	state.Selector = fmt.Sprintf("%s", call.sel)
	state.SelectedText = selectedEdgesContext(call.edges)
	return &brepLoopRead{state: state, call: call}, nil
}

// prismFaceViewBlend reads a prism through its brep face view for a Fillet
// of cap edges (RF3) or a Chamfer of one straight cap edge. Route E, L or V
// builds the selection or returns its specific refusal.
func prismFaceViewBlend(ctx context.Context, d *Document, pp prismPayload, call brepModifyRequest) (*Body, error) {
	bp, sourceRoles, err := breprecord.OfPrismWithRoles(pp)
	if err != nil {
		return nil, err
	}
	if err := breprecord.RequireExactSection(bp, call.op); err != nil {
		return nil, err
	}
	if call.asym != nil {
		call.asym.recordRoles = sourceRoles
	}
	return brepLoopRoute(ctx, d, bp, call)
}

func (s brepLoopSelection) refuse(row, reason string) error {
	return s.read.state.RefuseWithSelection(row, reason, selectedEdgesContext(s.edges()))
}

// matchEdges matches each selected edge to the one record edge whose ends,
// lifted through the reference frame and placement, lie within 1e-6 of its
// vertices: a line to a line, an arc to an arc about the same centre, and a
// whole circle to a circle of the same centre, axis and radius
// (modify-general §4.1). The match identifies the edge and admits no
// geometry. A band boundary edge of an earlier route L call pairs with no
// record face and matches nothing.
func (r *brepLoopRead) matchEdges() error {
	refView := r.state.Payload.RefView()
	const tol = 1e-6
	r.state.Matched = make([]int, len(r.call.edges))
	for i, e := range r.call.edges {
		r.state.Matched[i] = -1
		for ei, pair := range r.state.Topology.Edges {
			if err := survey2d.WallBudgetStep(r.state.Budget); err != nil {
				return err
			}
			if !brepEdgeMatches(refView, r.state.Topology.Uses[pair[0]], e, tol) {
				continue
			}
			if r.state.Matched[i] != -1 {
				r.state.Matched[i] = -2
				break
			}
			r.state.Matched[i] = ei
		}
	}
	return nil
}

// brepEdgeMatches reports whether body edge e is the record edge owner names,
// within tol after lifting through refView.
func brepEdgeMatches(refView prismPayload, owner brepUse, e *Edge, tol float64) bool {
	if e.n.Start == nil || e.n.End == nil {
		return false
	}
	lift := func(c [3]float64) r3.Vec { return refView.Point(c[0], c[1], c[2]) }
	key := owner.Key
	switch c := e.n.Curve.(type) {
	case surfacegeom.Line3:
		if key.Circular {
			return false
		}
	case surfacegeom.Arc3:
		if !key.Circular || key.Closed || lift(key.C).Sub(c.Center).Len() > tol {
			return false
		}
	case surfacegeom.Circle3:
		if !key.Closed || lift(key.C).Sub(c.Center).Len() > tol || math.Abs(c.Radius.Base()-key.Radius) > tol {
			return false
		}
		var unit [3]float64
		unit[key.Axis] = 1
		axis := refView.Dir(unit[0], unit[1], unit[2])
		return axis.Cross(c.Axis).Len() <= tol
	default:
		return false
	}
	return matchEndpoints(e.n.Start.Position, e.n.End.Position, lift(owner.DirFrom), lift(owner.DirTo), tol)
}

// singleStraightEdges reports whether the selection is route E's class: every
// edge matched one straight record edge along a reference axis (Table EB's
// EB1), and no two share a vertex (EB7).
func (s brepLoopSelection) singleStraightEdges() bool {
	seen := map[[3]float64]struct{}{}
	for i := range s.len() {
		ei := s.match(i)
		if ei < 0 {
			return false
		}
		owner := s.read.state.Topology.Uses[s.read.state.Topology.Edges[ei][0]]
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
func (s brepLoopSelection) admitLoops() ([]brepLoopSel, error) {
	r := s.read
	type faceLoop struct{ face, loop int }
	selected := map[faceLoop]map[int]struct{}{}
	loopsOf := make([][]faceLoop, s.len())
	for i := range s.len() {
		ei := s.match(i)
		switch ei {
		case -1:
			return nil, s.refuse("SL1", fmt.Sprintf(`%s matches no edge of the receiver's face record`, selectedEdgeContext(i, s.edge(i))))
		case -2:
			return nil, s.refuse("SL1", fmt.Sprintf(`%s matches more than one edge of the receiver's face record`, selectedEdgeContext(i, s.edge(i))))
		}
		for _, ui := range r.state.Topology.Edges[ei] {
			u := r.state.Topology.Uses[ui]
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
	complete := func(fl faceLoop) bool { return len(selected[fl]) == len(r.state.Topology.Planar[fl.face][fl.loop]) }
	chosen := map[faceLoop]struct{}{}
	var out []brepLoopSel
	for i := range s.len() {
		var covering []faceLoop
		for _, fl := range loopsOf[i] {
			if complete(fl) {
				covering = append(covering, fl)
			}
		}
		switch {
		case len(loopsOf[i]) == 0:
			return nil, s.refuse("SL1", fmt.Sprintf(`%s lies on no loop of a planar face, so it belongs to no complete loop; a %s of loops mixed with single edges needs a corner patch in no reference frame`, selectedEdgeContext(i, s.edge(i)), r.call.loopKind))
		case len(covering) == 0:
			return nil, s.refuse("SL1", fmt.Sprintf(`the selection covers only part of a loop of brep face %s; every geometric edge of a complete loop must be selected`, r.state.Payload.Faces[loopsOf[i][0].face].Role))
		case len(covering) > 1:
			return nil, s.refuse("SL1", fmt.Sprintf(`%s lies on two selected complete loops, of brep faces %s and %s, which share it`,
				selectedEdgeContext(i, s.edge(i)), r.state.Payload.Faces[covering[0].face].Role, r.state.Payload.Faces[covering[1].face].Role))
		}
		fl := covering[0]
		if _, ok := chosen[fl]; ok {
			continue
		}
		chosen[fl] = struct{}{}
		out = append(out, brepLoopSel{Face: fl.face, Loop: fl.loop})
	}
	return out, nil
}

// assignLoopSetbacks maps the public reference face of each selected edge to
// its loop face or its neighbouring record face. Each loop and each material
// side of one face must use one cap/side pair for the shared cap-blend view.
func (r *brepLoopRead) assignLoopSetbacks(sels []brepLoopSel) error {
	for i := range sels {
		sels[i].Setback = *r.call.loop
	}
	if r.call.asym == nil {
		return nil
	}
	selected := make(map[int]*Edge, len(r.state.Matched))
	for i, ei := range r.state.Matched {
		selected[ei] = r.call.edges[i]
	}
	seen := map[[2]int]capSetback{}
	for i := range sels {
		sel := &sels[i]
		assignment := -1
		for segment, beside := range sel.Beside {
			ui := r.state.LoopUse[[3]int{sel.Face, sel.Loop, segment}]
			edge := selected[r.state.Topology.EdgeOf[ui]]
			ref := r.call.asym.refs[edge]
			capFace := r.call.asym.matchesRecordFace(ref, sel.Face)
			sideFace := r.call.asym.matchesRecordFace(ref, r.state.Topology.Uses[beside.Use].Face)
			if capFace == sideFace {
				return r.state.RefuseAsymmetricLoop("SX16", fmt.Sprintf(`the reference for brep face %s's loop segment %d has no single adjacent record-face identity`,
					r.state.Payload.Faces[sel.Face].Role, segment))
			}
			choice := 0
			if sideFace {
				choice = 1
			}
			if assignment >= 0 && assignment != choice {
				return r.state.RefuseAsymmetricLoop("SX4", fmt.Sprintf(`brep face %s's loop mixes cap and side reference assignments`,
					r.state.Payload.Faces[sel.Face].Role))
			}
			assignment = choice
		}
		if assignment < 0 {
			return r.state.RefuseAsymmetricLoop("SX16", fmt.Sprintf(`brep face %s's loop has no edges to assign a reference face`,
				r.state.Payload.Faces[sel.Face].Role))
		}
		if assignment == 0 {
			sel.Setback = capSetback{Dc: r.call.asym.d, DcDelta: r.call.asym.dDelta,
				Ds: r.call.asym.other, DsDelta: r.call.asym.otherDelta}
		} else {
			sel.Setback = capSetback{Dc: r.call.asym.other, DcDelta: r.call.asym.otherDelta,
				Ds: r.call.asym.d, DsDelta: r.call.asym.dDelta}
		}
		f := r.state.Payload.Faces[sel.Face]
		band := brepLoopBand{Sigma: sel.Sigma}
		side := 0
		if band.MatSign(f) > 0 {
			side = 1
		}
		key := [2]int{sel.Face, side}
		if old, ok := seen[key]; ok && old != sel.Setback {
			return r.state.RefuseAsymmetricLoop("SX4", fmt.Sprintf(`brep face %s's loops on one material side use different cap and side setbacks`,
				f.Role))
		}
		seen[key] = sel.Setback
	}
	return nil
}
