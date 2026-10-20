package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/breploop"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This file is route L of agent-docs/modify-general-design.md ("modify-general §N"
// below) and route V of agent-docs/vertex-blend-design.md: a Chamfer or Fillet of
// complete planar-face loops on a brep or stacked receiver (§4;
// agent-docs/loop-fillet-design.md, "loop-fillet
// §N", for the fillet arm). It also decides, for a Fillet or Chamfer that
// route P does not take, which of route E (agent-docs/brep-modify-design.md §5) and
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
	call  Request
	input breploop.SelectionInput
}

// brepLoopSelection reads selected edges in order from one matched record.
// Its ordinals are private to the view; the reader's matches and budget are
// shared, and constructing a subset never matches an edge again.
type brepLoopSelection struct {
	read *brepLoopRead
	view breploop.Selection
}

func (r *brepLoopRead) fullSelection() brepLoopSelection {
	return brepLoopSelection{read: r, view: breploop.Bind(r.state, r.input)}
}

func (r *brepLoopRead) selection(ordinals []int) brepLoopSelection {
	return brepLoopSelection{read: r, view: breploop.Bind(r.state, r.input).View(ordinals)}
}

func (s brepLoopSelection) len() int { return s.view.Len() }

func (s brepLoopSelection) edge(i int) *topo.Edge { return s.read.call.Edges[s.view.Ordinals[i]] }

func (s brepLoopSelection) edges() []*topo.Edge {
	edges := make([]*topo.Edge, s.len())
	for i := range edges {
		edges[i] = s.edge(i)
	}
	return edges
}

// breploop.Sel is one touched loop of Table LB: the loop of index loop on the
// planar face of index face, what lies beside each of its segments (LB3),
// and the band's sense (LB6), −1 for walls descending into the body and +1
// for walls rising off it.

// brepLoopRoute is the brep route of a Fillet or Chamfer that route P does not
// take (modify-general §4.4's stage 2c, loop-fillet §6). Independent straight
// edges take route E. A Fillet of independent straight edges and complete
// planar loops takes route V, which runs E then L. Shared straight edges on
// a swept wall restate that wall before route L's partial-loop construction.
// Other selections take L or its SL1 refusal.
func brepLoopRoute(ctx context.Context, env topo.Env, bp breprecord.Payload, call Request) (*topo.Body, error) {
	r, err := newBrepLoopRead(ctx, bp, call)
	if err != nil {
		return nil, err
	}
	if err := r.matchEdges(); err != nil {
		return nil, err
	}
	if r.fullSelection().singleStraightEdges() {
		body, err := BlendEdges(ctx, env, bp, call)
		var mismatch *breprecord.MaterialWedgeMismatchError
		if len(call.Edges) == 1 && call.LoopKind == breprecord.BandFillet && errors.As(err, &mismatch) {
			if sel, selected, ok := r.fullSelection().view.PartialSelectedLoop(); ok {
				return brepFilletPartialLoop(ctx, env, bp, call, sel, selected)
			}
		}
		if len(call.Edges) == 1 && call.LoopKind == breprecord.BandChamfer && call.Asym == nil &&
			errors.As(err, &mismatch) {
			if sel, selected, ok := r.fullSelection().view.PartialSelectedLoop(); ok {
				return brepChamferPartialLoop(ctx, env, bp, call, sel, selected)
			}
		}
		return body, err
	}
	if call.Asym != nil {
		loops, err := r.fullSelection().admitLoops()
		if err != nil {
			return nil, fmt.Errorf(`%w: an asymmetric chamfer on a brep requires independent straight edges or complete planar-face loops (modify-reach SX16): %v`, decaderr.ErrUnsupported, err)
		}
		return r.buildSelectedLoops(ctx, env, loops)
	}
	if call.LoopKind == breprecord.BandFillet {
		if body, recognized, err := r.cornerTriad(ctx, env); recognized {
			return body, err
		}
		loops, singles, mixed, err := r.splitVertexBlend()
		if err != nil {
			return nil, err
		}
		if mixed {
			stepCall := call
			stepCall.Edges = singles
			step, err := BlendEdges(ctx, env, bp, stepCall)
			if err != nil {
				return nil, err
			}
			rewritten, ok := step.Payload.(breprecord.Payload)
			if !ok {
				return nil, fmt.Errorf(`%w: the straight-edge fillet produced no brep record`, decaderr.ErrUnsupported)
			}
			next, err := newBrepLoopRead(ctx, rewritten, call)
			if err != nil {
				return nil, err
			}
			return next.buildSelectedLoops(ctx, env, loops)
		}
		if sel, selected, ok := r.fullSelection().view.PartialSelectedLoop(); ok {
			return brepFilletPartialLoop(ctx, env, bp, call, sel, selected)
		}
		// A pair of edges can share a swept wall without sharing a planar
		// loop in the input record. Route E's preparation restates that wall;
		// the same partial-loop builder can then close their common corner.
		if selectedStraightEdgesShareVertex(call.Edges) {
			er, blends, err := prepareBrepEdgeBlends(ctx, bp, call, true, nil)
			if err == nil {
				next, err := newBrepLoopRead(ctx, er.BP, call)
				if err != nil {
					return nil, err
				}
				if err := next.matchEdges(); err != nil {
					return nil, err
				}
				if sel, selected, ok := next.fullSelection().view.PartialSelectedLoop(); ok {
					breploop.MarkChainTerminals(blends)
					return next.buildPartialFillet(ctx, env, sel, selected, blends)
				}
			}
		}
	}
	if call.LoopKind == breprecord.BandChamfer && len(call.Edges) == 2 {
		if sel, selected, ok := r.fullSelection().view.PartialSelectedLoop(); ok {
			return brepChamferPartialLoop(ctx, env, bp, call, sel, selected)
		}
	}
	return r.buildLoops(ctx, env)
}

// cornerTriad rounds three edges at one vertex by rounding the edge across
// a planar cap first. The cap's two shortened lines and the new tangent arc
// then form one partial fillet chain with a sphere at that arc.
func (r *brepLoopRead) cornerTriad(ctx context.Context, env topo.Env) (*topo.Body, bool, error) {
	if len(r.call.Edges) != 3 {
		return nil, false, nil
	}
	var corner *topo.Vertex
	for _, v := range [2]*topo.Vertex{r.call.Edges[0].Start, r.call.Edges[0].End} {
		if v != nil && (r.call.Edges[1].Start == v || r.call.Edges[1].End == v) &&
			(r.call.Edges[2].Start == v || r.call.Edges[2].End == v) {
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
		capSel, selected, ok := capSelection.view.PartialSelectedLoop()
		if !ok {
			continue
		}
		singleSelection := r.selection([]int{single})
		singleCall := r.call
		singleCall.Edges = singleSelection.edges()
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
		step, err := BlendEdges(ctx, env, r.state.Payload, singleCall)
		if err != nil {
			return nil, true, err
		}
		bp, ok := step.Payload.(breprecord.Payload)
		if !ok {
			return nil, true, fmt.Errorf(`%w: the first corner blend produced no brep record`, decaderr.ErrUnsupported)
		}
		var face *topo.Face
		for _, candidate := range topobuild.Faces(step) {
			for _, origin := range candidate.Origins {
				if origin.Role == bp.Faces[capSel.Face].Role {
					face = candidate
				}
			}
		}
		if face == nil || capSel.Loop >= len(face.Loops) {
			return nil, true, fmt.Errorf(`%w: the rounded corner lost its cap face`, decaderr.ErrUnsupported)
		}
		loopEdges := topobuild.LoopEdges(face.Loops[capSel.Loop])
		var shortened [2]*topo.Edge
		var near [2]*topo.Vertex
		for j, originalIndex := range others {
			original := r.call.Edges[originalIndex]
			far := original.Start
			if far == corner {
				far = original.End
			}
			oldDir := original.End.Position.Sub(original.Start.Position)
			for _, edge := range loopEdges {
				if _, ok := edge.Curve.(surfacegeom.Line3); !ok {
					continue
				}
				other := edge.Start
				switch {
				case edge.Start.Position.Sub(far.Position).Len() <= 1e-6:
					other = edge.End
				case edge.End.Position.Sub(far.Position).Len() <= 1e-6:
				default:
					continue
				}
				newDir := edge.End.Position.Sub(edge.Start.Position)
				if oldDir.Cross(newDir).Len() > 1e-6*oldDir.Len()*newDir.Len() {
					continue
				}
				shortened[j], near[j] = edge, other
				break
			}
			if shortened[j] == nil {
				return nil, true, fmt.Errorf(`%w: a corner cap edge has no shortened successor`, decaderr.ErrUnsupported)
			}
		}
		var arc *topo.Edge
		for _, edge := range loopEdges {
			if _, ok := edge.Curve.(surfacegeom.Arc3); !ok {
				continue
			}
			if (edge.Start == near[0] && edge.End == near[1]) ||
				(edge.Start == near[1] && edge.End == near[0]) {
				arc = edge
				break
			}
		}
		if arc == nil {
			return nil, true, fmt.Errorf(`%w: the rounded corner has no joining cap arc`, decaderr.ErrUnsupported)
		}
		nextCall := r.call
		nextCall.Edges = []*topo.Edge{shortened[0], shortened[1], arc}
		next, err := newBrepLoopRead(ctx, bp, nextCall)
		if err != nil {
			return nil, true, err
		}
		if err := next.matchEdges(); err != nil {
			return nil, true, err
		}
		newCap, newSelected, ok := next.fullSelection().view.PartialSelectedLoop()
		if !ok || newCap.Face != capSel.Face || newCap.Loop != capSel.Loop {
			return nil, true, next.state.Refuse("SL1", `the rounded corner has no selected cap chain`)
		}
		body, err := brepFilletPartialLoop(ctx, env, bp, nextCall, newCap, newSelected)
		return body, true, err
	}
	return nil, false, nil
}

// splitVertexBlend finds complete selected planar loops and the remaining
// independent straight edges. Face and loop indices survive route E's rewrite.
func (r *brepLoopRead) splitVertexBlend() ([]breploop.Sel, []*topo.Edge, bool, error) {
	result, err := r.fullSelection().view.SplitVertexBlend()
	if err != nil {
		return nil, nil, false, err
	}
	straight := make([]*topo.Edge, len(result.StraightOrdinals))
	for i, ordinal := range result.StraightOrdinals {
		straight[i] = r.call.Edges[ordinal]
	}
	return result.Loops, straight, result.Mixed, nil
}

// buildLoops is route L over the matched selection: Table LB admits the
// loops, the record is rewritten with one band of the call's kind per loop,
// and the rewritten record is built.
func (r *brepLoopRead) buildLoops(ctx context.Context, env topo.Env) (*topo.Body, error) {
	call := r.call
	if call.Loop == nil {
		return nil, fmt.Errorf(`%w: route L reads a %s request with no setback`, decaderr.ErrUnsupported, call.Op)
	}
	loops, err := r.fullSelection().admitLoops()
	if err != nil {
		return nil, err
	}
	return r.buildSelectedLoops(ctx, env, loops)
}

func (r *brepLoopRead) buildSelectedLoops(ctx context.Context, env topo.Env, loops []breploop.Sel) (*topo.Body, error) {
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
	body, err := breprecord.Build(ctx, env, out)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the %s's rewritten brep record (modify-general §4.2 step 6); selector %s matched [%s]`,
			err, call.LoopKind, call.Sel, modifyselect.EdgesContext(call.Edges))
	}
	if body.Volume.Value.Base() <= 0 {
		return nil, fmt.Errorf(`%w: the %s's rewritten brep record encloses no volume (modify-general §4.2 step 6); selector %s matched [%s]`,
			decaderr.ErrDegenerate, call.LoopKind, call.Sel, modifyselect.EdgesContext(call.Edges))
	}
	return body, nil
}

// newBrepLoopRead reads the receiver record bp for one route L call: its
// topology and each planar loop segment's use. Matching the selected edges is
// matchEdges's.
func newBrepLoopRead(ctx context.Context, bp breprecord.Payload, call Request) (*brepLoopRead, error) {
	state, err := breploop.New(ctx, bp, call.Op, call.LoopKind, call.Loop)
	if err != nil {
		return nil, err
	}
	state.Selector = call.Sel
	state.SelectedText = modifyselect.EdgesContext(call.Edges)
	input := breploop.SelectionInput{
		Edges: slices.Clone(call.Edges),
		Describe: func(local, original int) string {
			return modifyselect.EdgeContext(local, call.Edges[original])
		},
	}
	if call.Asym != nil {
		asym := &breploop.AsymmetricInput{
			Receiver: call.Asym.Body, RecordRoles: call.Asym.RecordRoles,
			D: call.Asym.D, DDelta: call.Asym.DDelta,
			Other: call.Asym.Other, OtherDelta: call.Asym.OtherDelta,
			References: make([]*topo.Face, len(call.Edges)),
		}
		for i, edge := range call.Edges {
			ref := call.Asym.Refs[edge]
			if ref != nil {
				asym.References[i] = ref
			}
		}
		input.Asym = asym
	}
	return &brepLoopRead{state: state, call: call, input: input}, nil
}

// PrismFaceViewBlend reads a prism through its brep face view for a Fillet
// of cap edges (RF3) or a Chamfer of one straight cap edge. Route E, L or V
// builds the selection or returns its specific refusal.
func PrismFaceViewBlend(ctx context.Context, env topo.Env, pp prismrecord.Payload, call Request) (*topo.Body, error) {
	bp, sourceRoles, err := breprecord.OfPrismWithRoles(pp)
	if err != nil {
		return nil, err
	}
	if err := breprecord.RequireExactSection(bp, call.Op); err != nil {
		return nil, err
	}
	if call.Asym != nil {
		call.Asym.RecordRoles = sourceRoles
	}
	return brepLoopRoute(ctx, env, bp, call)
}

// matchEdges matches each selected edge to the one record edge whose ends,
// lifted through the reference frame and placement, lie within 1e-6 of its
// vertices: a line to a line, an arc to an arc about the same centre, and a
// whole circle to a circle of the same centre, axis and radius
// (modify-general §4.1). The match identifies the edge and admits no
// geometry. A band boundary edge of an earlier route L call pairs with no
// record face and matches nothing.
func (r *brepLoopRead) matchEdges() error {
	return r.fullSelection().view.MatchEdges()
}

// singleStraightEdges reports whether the selection is route E's class: every
// edge matched one straight record edge along a reference axis (Table EB's
// EB1), and no two share a vertex (EB7).
func (s brepLoopSelection) singleStraightEdges() bool {
	return s.view.SingleStraightEdges()
}

// admitLoops is LB1 and LB2 (SL1): the selected edges are exactly the
// segments of one or more complete loops of planar faces, and no two of those
// loops share an edge. A loop is complete when every one of its segments is
// selected; every selected edge must lie on a complete loop, and on one only.
// The loops are returned in the order the selection first reaches them. A
// loop on a face with a section displacement is SB1, which the brep route
// already refused for the whole record.
func (s brepLoopSelection) admitLoops() ([]breploop.Sel, error) {
	return s.view.AdmitLoops()
}

// assignLoopSetbacks maps the public reference face of each selected edge to
// its loop face or its neighbouring record face. Each loop and each material
// side of one face must use one cap/side pair for the shared cap-blend view.
func (r *brepLoopRead) assignLoopSetbacks(sels []breploop.Sel) error {
	return r.fullSelection().view.AssignLoopSetbacks(sels)
}
