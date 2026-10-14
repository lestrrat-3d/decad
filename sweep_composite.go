package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegroup"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// compositeSpanPart is one closed single-span reduction before its internal
// cap is removed. startCap and endCap follow path direction, including the
// cap-role reversal required by an arc whose Revolve axis was reoriented.
type compositeSpanPart struct {
	body     *Body
	startCap *Face
	endCap   *Face
}

func compositeSpanPartOf(body *Body) (compositeSpanPart, error) {
	part := compositeSpanPart{body: body}
	for _, face := range body.Faces() {
		for _, origin := range face.n.Origins {
			switch origin.Role {
			case provenance.RoleCapStart:
				if part.startCap != nil {
					return compositeSpanPart{}, fmt.Errorf(`%w: a sweep span carries more than one start cap`, ErrUnsupported)
				}
				part.startCap = face
			case provenance.RoleCapEnd:
				if part.endCap != nil {
					return compositeSpanPart{}, fmt.Errorf(`%w: a sweep span carries more than one end cap`, ErrUnsupported)
				}
				part.endCap = face
			}
		}
	}
	if part.startCap == nil || part.endCap == nil {
		return compositeSpanPart{}, fmt.Errorf(`%w: a sweep span does not carry both path-oriented caps`, ErrUnsupported)
	}
	return part, nil
}

// assembleCompositeSweepBody removes every internal span cap and reuses one
// rim edge and its endpoint vertices at each path join. The two cap loops are
// paired by their recorded loop and coedge order; no coordinate search or
// proximity weld participates in the pairing.
//
// surfaceResult additionally omits the first span's start cap and the last
// span's end cap from the published face set (docs/surface-design.md §4):
// every span still builds and sews solid (compositeSweepSpanPayload never
// carries the flag, docs/surface-design.md's own span-pairing requirement —
// both cap roles must exist for compositeSpanPartOf and sewCompositeSweepJoin
// above to run), so this is the ONLY place the flag is consumed. The two
// outer caps are simply never appended to faces and their body back-pointer
// is never reassigned to body: the live-face filter a few lines down keys on
// that back-pointer, so leaving it pointing at the temporary per-span body is
// what drops each outer rim edge to its one surviving wall face with no
// rim-specific code at all.
//
// Ordering matters and getting it wrong is silent: lumps must be set once
// with the provisional single shell so body.Edges()/body.Faces() traversal
// works, then the live-face filter runs, and only THEN may sheetLumps run,
// because it reads the adjacency the filter produces. Running it earlier
// would have every shell wrongly report open — shellIsOpen would see every
// discarded per-span face still on each edge's face list.
func assembleCompositeSweepBody(
	ctx context.Context,
	d *Document,
	ref producerID,
	parts []compositeSpanPart,
	surfaceResult bool,
) (*Body, error) {
	return assembleCompositeSweepBodyWithAudit(ctx, d, ref, parts, surfaceResult, auditCompositeSweep)
}

// assembleCompositeSweepBodyWithAudit shares the topological sewing with
// span classes that prove separation by a different geometric certificate.
func assembleCompositeSweepBodyWithAudit(
	ctx context.Context,
	d *Document,
	ref producerID,
	parts []compositeSpanPart,
	surfaceResult bool,
	auditSpans func(context.Context, []sweepAuditSpan) error,
) (*Body, error) {
	return assembleCompositeSweepBodyWithJoin(ctx, d, ref, parts, surfaceResult, auditSpans, sewCompositeSweepJoin)
}

func assembleClosedCompositeSweepBody(ctx context.Context, d *Document, ref producerID,
	parts []compositeSpanPart) (*Body, error) {
	return assembleCompositeSweepBodyWithJoinMode(ctx, d, ref, parts, false,
		auditClosedCompositeSweep, sewCompositeSweepJoin, true)
}

func assembleCompositeSweepBodyWithJoin(
	ctx context.Context,
	d *Document,
	ref producerID,
	parts []compositeSpanPart,
	surfaceResult bool,
	auditSpans func(context.Context, []sweepAuditSpan) error,
	joinSpans func(context.Context, *Face, *Face, *Body, map[*Edge]struct{}) error,
) (*Body, error) {
	return assembleCompositeSweepBodyWithJoinMode(ctx, d, ref, parts, surfaceResult, auditSpans, joinSpans, false)
}

func assembleCompositeSweepBodyWithJoinMode(
	ctx context.Context,
	d *Document,
	ref producerID,
	parts []compositeSpanPart,
	surfaceResult bool,
	auditSpans func(context.Context, []sweepAuditSpan) error,
	joinSpans func(context.Context, *Face, *Face, *Body, map[*Edge]struct{}) error,
	closed bool,
) (*Body, error) {
	if len(parts) < 2 {
		return nil, fmt.Errorf(`%w: a composite sweep requires at least two spans`, ErrDegenerate)
	}
	if err := validateCompositeSweepRims(ctx, parts); err != nil {
		return nil, err
	}
	if closed {
		if err := validateCompositeSweepRims(ctx, []compositeSpanPart{parts[len(parts)-1], parts[0]}); err != nil {
			return nil, err
		}
	}
	audit := make([]sweepAuditSpan, len(parts))
	for i, part := range parts {
		audit[i] = sweepAuditSpan(part)
	}
	if err := auditSpans(ctx, audit); err != nil {
		return nil, err
	}

	sewnEdges := make(map[*Edge]struct{})
	for join := 0; join+1 < len(parts); join++ {
		if err := joinSpans(
			ctx,
			parts[join].endCap,
			parts[join+1].startCap,
			parts[join].body,
			sewnEdges,
		); err != nil {
			return nil, wrapStageError(fmt.Sprintf(`sweep path join %d`, join+1), err)
		}
	}
	if closed {
		last := parts[len(parts)-1]
		if err := joinSpans(ctx, last.endCap, parts[0].startCap, last.body, sewnEdges); err != nil {
			return nil, wrapStageError(`sweep path closure`, err)
		}
	}

	kind := BodySolid
	if surfaceResult {
		kind = BodySheet
	}
	body := bornBody(&Body{
		doc: d,
		n:   topo.Body{Solid: !surfaceResult, Origin: provenance.Ref{Producer: ref, Role: provenance.RoleBody}, Kind: topo.BodyKind(kind)},
	})
	var faces []*Face
	if !surfaceResult && !closed {
		faces = append(faces, parts[0].startCap)
	}
	for k, part := range parts {
		for _, face := range part.body.Faces() {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			if face == part.startCap || face == part.endCap {
				continue
			}
			rewriteCompositeSideRoles(face, k)
			face.n.Body = bodyNode(body)
			faces = append(faces, face)
		}
	}
	if !surfaceResult && !closed {
		parts[0].startCap.n.Body = bodyNode(body)
		parts[len(parts)-1].endCap.n.Body = bodyNode(body)
		faces = append(faces, parts[len(parts)-1].endCap)
	}
	body.lumps = []*Lump{{shells: []*FaceShell{{faces: faces}}}}

	for _, edge := range body.Edges() {
		edge.n.Faces = liveCompositeFaces(edge.n.Faces, body)
	}
	if surfaceResult {
		// sheetLumps splits by connectivity (docs/surface-design.md §2.2): a
		// holed profile's outer and hole wall tubes touch nowhere once the two
		// outer caps that used to bridge them are gone, so each becomes its own
		// Lump. It must run after the live-face filter above, per this
		// function's own doc comment.
		body.lumps = sheetLumps(faces)
	}
	if err := auditCompositeBoundary(ctx, body, sewnEdges); err != nil {
		return nil, err
	}
	if closed {
		for _, edge := range body.Edges() {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			if len(edge.n.Faces) != 2 {
				return nil, fmt.Errorf(`%w: a closed sweep rim lacks two incident faces`, ErrUnsupported)
			}
		}
	}
	return body, nil
}

func validateCompositeSweepRims(ctx context.Context, parts []compositeSpanPart) error {
	budget := proofbound.NewWorkBudget(ctx)
	for join := 0; join+1 < len(parts); join++ {
		if err := budget.Err(); err != nil {
			return err
		}
		from, to := parts[join].endCap, parts[join+1].startCap
		if len(from.n.Loops) != len(to.n.Loops) {
			return fmt.Errorf(`%w: adjacent sweep sections have different loop counts`, ErrUnsupported)
		}
		for li := range from.n.Loops {
			if err := budget.Step(); err != nil {
				return err
			}
			if len(from.n.Loops[li].CoEdges) != len(to.n.Loops[li].CoEdges) {
				return fmt.Errorf(`%w: adjacent sweep section loop %d has different edge counts`, ErrUnsupported, li)
			}
			for ci := range from.n.Loops[li].CoEdges {
				if err := budget.Step(); err != nil {
					return err
				}
				fromUse := from.n.Loops[li].CoEdges[ci]
				toUse := to.n.Loops[li].CoEdges[ci]
				fromEdge := fromUse.Edge
				toEdge := toUse.Edge
				if fromEdge == nil || toEdge == nil || fromEdge.Start == nil || fromEdge.End == nil || toEdge.Start == nil || toEdge.End == nil {
					return fmt.Errorf(`%w: an adjacent sweep section carries incomplete rim topology`, ErrUnsupported)
				}
				if fromEdge == toEdge {
					return fmt.Errorf(`%w: a span reuses one rim edge at both path ends`, ErrUnsupported)
				}
			}
		}
	}
	return budget.Err()
}

func sewCompositeSweepJoin(
	ctx context.Context,
	from, to *Face,
	previous *Body,
	sewn map[*Edge]struct{},
) error {
	for li := range from.n.Loops {
		for ci := range from.n.Loops[li].CoEdges {
			if err := ctx.Err(); err != nil {
				return err
			}
			fromUse := from.n.Loops[li].CoEdges[ci]
			toUse := to.n.Loops[li].CoEdges[ci]
			// The next span's start rim is evaluated directly in the certified
			// transported frame. Keep that edge and replace the preceding span's
			// independently rotated end rim, so the published join reads the
			// exact shared section rather than a second libm image of it.
			shared, discarded := toUse.Edge, fromUse.Edge
			fromStart, fromEnd := vertexHandle(coedgeStart(fromUse)), vertexHandle(coedgeEnd(fromUse))
			toStart, toEnd := vertexHandle(coedgeStart(toUse)), vertexHandle(coedgeEnd(toUse))
			rewriteCompositeVertex(previous, fromStart, toStart)
			rewriteCompositeVertex(previous, fromEnd, toEnd)
			rewriteCompositeEdge(previous, edgeHandle(discarded), edgeHandle(shared))
			shared.Faces = append(shared.Faces, discarded.Faces...)
			sewn[edgeHandle(shared)] = struct{}{}
		}
	}
	return nil
}

func rewriteCompositeVertex(body *Body, old, replacement *Vertex) {
	if old == replacement {
		return
	}
	for _, edge := range body.Edges() {
		if vertexHandle(edge.n.Start) == old {
			edge.n.Start = vertexNode(replacement)
		}
		if vertexHandle(edge.n.End) == old {
			edge.n.End = vertexNode(replacement)
		}
	}
}

func rewriteCompositeEdge(body *Body, old, replacement *Edge) {
	for _, face := range body.Faces() {
		for _, loop := range face.n.Loops {
			for i := range loop.CoEdges {
				if loop.CoEdges[i].Edge == edgeNode(old) {
					loop.CoEdges[i].Edge = edgeNode(replacement)
				}
			}
		}
	}
}

func rewriteCompositeSideRoles(face *Face, span int) {
	for i, origin := range face.n.Origins {
		role, ok := compositesweep.RewriteSideRole(origin.Role, span)
		if !ok {
			continue
		}
		origin.Role = role
		face.n.Origins[i] = origin
	}
}

func liveCompositeFaces(faces []*topo.Face, body *Body) []*topo.Face {
	out := make([]*topo.Face, 0, 2)
	for _, face := range faces {
		if face == nil || face.Body != bodyNode(body) || slices.Contains(out, face) {
			continue
		}
		out = append(out, face)
	}
	return out
}

type compositeCoedgeUse struct {
	face    *Face
	forward bool
}

// auditCompositeBoundary proves the topology facts introduced by sewing. The
// analytic span builders already prove each face and its outward sense; this
// audit checks that removing caps preserved positive faces, paired every
// two-face edge in opposite directions, and left every vertex's face fan a
// single cycle (a closed solid) or a single path (a surface result's two
// outer rims) — the same manifold-with-boundary invariant
// docs/surface-design.md §9.1 states for the sheet validity audit, restated
// here because this audit runs on the ASSEMBLED composite body rather than
// its recorded topology. The common audit admits free edges for sheets.
// A closed composite solid checks exactly two faces per edge after this audit.
func auditCompositeBoundary(ctx context.Context, body *Body, sewn map[*Edge]struct{}) error {
	if body == nil {
		return fmt.Errorf(`%w: a nil composite sweep body has no boundary`, ErrDegenerate)
	}
	budget := proofbound.NewWorkBudget(ctx)
	uses := make(map[*Edge][]compositeCoedgeUse)
	for _, face := range body.Faces() {
		if err := budget.Step(); err != nil {
			return err
		}
		if proofbound.AdmitAbove(proofbound.MeasuredScalar(face.n.Area, face.n.AreaBound), 0) != proofbound.SurvAdmit {
			return fmt.Errorf(`%w: a composite sweep face is not proven to have positive area`, ErrUnsupported)
		}
		if len(face.n.Loops) == 0 {
			return fmt.Errorf(`%w: a composite sweep face has no boundary loop`, ErrUnsupported)
		}
		for _, loop := range face.n.Loops {
			if len(loop.CoEdges) == 0 {
				return fmt.Errorf(`%w: a composite sweep face has an empty boundary loop`, ErrUnsupported)
			}
			for _, use := range loop.CoEdges {
				if err := budget.Step(); err != nil {
					return err
				}
				if use.Edge == nil {
					return fmt.Errorf(`%w: a composite sweep loop has a nil edge`, ErrUnsupported)
				}
				uses[edgeHandle(use.Edge)] = append(uses[edgeHandle(use.Edge)], compositeCoedgeUse{face: face, forward: use.Forward})
			}
			for i, use := range loop.CoEdges {
				next := loop.CoEdges[(i+1)%len(loop.CoEdges)]
				if coedgeEnd(use) != coedgeStart(next) {
					return fmt.Errorf(`%w: a composite sweep face loop is not endpoint-continuous`, ErrUnsupported)
				}
			}
		}
	}

	for edge, edgeUses := range uses {
		if err := budget.Step(); err != nil {
			return err
		}
		// An edge may have one or two incident faces (a surface result's own
		// free rim edge, or an interior edge on either kind), but never zero or
		// three or more, and the use count read off every face's own loops must
		// agree with Edge.Faces()'s own adjacency exactly either way.
		if len(edge.n.Faces) < 1 || len(edge.n.Faces) > 2 || len(edgeUses) != len(edge.n.Faces) {
			return fmt.Errorf(`%w: a composite sweep edge does not have one or two incident faces`, ErrUnsupported)
		}
		if len(edgeUses) != 2 {
			continue
		}
		if edgeUses[0].face == edgeUses[1].face {
			return fmt.Errorf(`%w: a composite sweep edge is used twice by one face`, ErrUnsupported)
		}
		if _, isSewn := sewn[edge]; isSewn && edgeUses[0].forward == edgeUses[1].forward {
			return fmt.Errorf(`%w: a sewn sweep edge does not have opposite directed uses`, ErrUnsupported)
		}
	}
	if err := auditVertexLinks(budget, uses); err != nil {
		return err
	}
	return budget.Err()
}

// auditVertexLinks proves the manifold-with-boundary invariant at every
// vertex the edges in uses touch: the faces touching it, linked by the
// two-face edges also touching it, form one connected component whose own
// face degrees (how many such link edges reach that face) are all 1 or
// all-but-two 2. Two degree-1 faces make it a PATH — a rim vertex, where
// exactly two of the incident edges are free and end the fan instead of
// continuing it. Zero degree-1 faces make it a CYCLE — an interior vertex,
// exactly the shape the pre-sheet audit already proved. A free (one-face)
// edge contributes its lone face to the vertex's link as a node with no link
// edge of its own, since it has no second face to link that face to there.
//
// This takes no *Body: an earlier version of this function also compared
// len(links) against len(body.Vertices()) and refused on a mismatch, but
// that comparison is a tautology, never a fact about the geometry.
// body.Vertices() derives from Body.Edges(), which walks the identical
// face.loops -> loop.coedges structure uses is itself built from, over the
// SAME edge set (every caller builds uses over every edge of every face's
// every loop, and this function itself refuses any edge with a nil endpoint
// before the counts could ever be compared), so the two vertex sets can
// never disagree. Dropping the parameter along with that leg is also what
// lets this function's second caller (Stitch's curved-body gate) reach it:
// that caller builds its own uses map over a set of faces that has no
// *Body at all until after this audit is meant to run.
func auditVertexLinks(
	budget *proofbound.WorkBudget,
	uses map[*Edge][]compositeCoedgeUse,
) error {
	edges := make(map[*Edge]surfacegroup.LinkEdge[*Face, *Vertex], len(uses))
	for edge, edgeUses := range uses {
		faces := make([]*Face, len(edgeUses))
		for i, use := range edgeUses {
			faces[i] = use.face
		}
		edges[edge] = surfacegroup.LinkEdge[*Face, *Vertex]{
			Start: vertexHandle(edge.n.Start), End: vertexHandle(edge.n.End), Faces: faces, EndpointsValid: edge.n.Start != nil && edge.n.End != nil,
		}
	}
	violation, err := surfacegroup.AuditVertexLinks(budget, edges)
	if err != nil {
		return err
	}
	switch violation {
	case surfacegroup.LinkValid:
		return nil
	case surfacegroup.LinkIncompleteEdge:
		return fmt.Errorf(`%w: a composite sweep edge has incomplete endpoint topology`, ErrUnsupported)
	case surfacegroup.LinkDegree:
		return fmt.Errorf(`%w: a composite sweep vertex link face has degree outside one or two`, ErrUnsupported)
	case surfacegroup.LinkShape:
		return fmt.Errorf(`%w: a composite sweep vertex link is neither a cycle nor a path`, ErrUnsupported)
	case surfacegroup.LinkDisconnected:
		return fmt.Errorf(`%w: a composite sweep vertex link has more than one connected component`, ErrUnsupported)
	default:
		return fmt.Errorf(`%w: an unknown composite sweep vertex link state`, ErrUnsupported)
	}
}
