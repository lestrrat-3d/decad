package decad

import (
	"context"
	"fmt"
	"slices"
	"strings"
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
		for _, origin := range face.origins {
			switch origin.Role {
			case roleCapStart:
				if part.startCap != nil {
					return compositeSpanPart{}, fmt.Errorf(`%w: a sweep span carries more than one start cap`, ErrUnsupported)
				}
				part.startCap = face
			case roleCapEnd:
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
	if len(parts) < 2 {
		return nil, fmt.Errorf(`%w: a composite sweep requires at least two spans`, ErrDegenerate)
	}
	if err := validateCompositeSweepRims(ctx, parts); err != nil {
		return nil, err
	}
	audit := make([]sweepAuditSpan, len(parts))
	for i, part := range parts {
		audit[i] = sweepAuditSpan(part)
	}
	if err := auditCompositeSweep(ctx, audit); err != nil {
		return nil, err
	}

	sewnEdges := make(map[*Edge]struct{})
	for join := 0; join+1 < len(parts); join++ {
		if err := sewCompositeSweepJoin(
			ctx,
			parts[join].endCap,
			parts[join+1].startCap,
			parts[join].body,
			sewnEdges,
		); err != nil {
			return nil, fmt.Errorf(`sweep path join %d: %w`, join+1, err)
		}
	}

	kind := BodySolid
	if surfaceResult {
		kind = BodySheet
	}
	body := &Body{
		doc:    d,
		origin: FeatureRef{producer: ref, Role: roleBody},
		solid:  !surfaceResult,
		kind:   kind,
	}
	var faces []*Face
	if !surfaceResult {
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
			face.body = body
			faces = append(faces, face)
		}
	}
	if !surfaceResult {
		parts[0].startCap.body = body
		parts[len(parts)-1].endCap.body = body
		faces = append(faces, parts[len(parts)-1].endCap)
	}
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	for _, edge := range body.Edges() {
		edge.faces = liveCompositeFaces(edge.faces, body)
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
	return body, nil
}

func validateCompositeSweepRims(ctx context.Context, parts []compositeSpanPart) error {
	budget := newWorkBudget(ctx)
	for join := 0; join+1 < len(parts); join++ {
		if err := budget.err(); err != nil {
			return err
		}
		from, to := parts[join].endCap, parts[join+1].startCap
		if len(from.loops) != len(to.loops) {
			return fmt.Errorf(`%w: adjacent sweep sections have different loop counts`, ErrUnsupported)
		}
		for li := range from.loops {
			if err := budget.step(); err != nil {
				return err
			}
			if len(from.loops[li].coedges) != len(to.loops[li].coedges) {
				return fmt.Errorf(`%w: adjacent sweep section loop %d has different edge counts`, ErrUnsupported, li)
			}
			for ci := range from.loops[li].coedges {
				if err := budget.step(); err != nil {
					return err
				}
				fromUse := from.loops[li].coedges[ci]
				toUse := to.loops[li].coedges[ci]
				fromEdge := fromUse.edge
				toEdge := toUse.edge
				if fromEdge == nil || toEdge == nil || fromEdge.start == nil || fromEdge.end == nil || toEdge.start == nil || toEdge.end == nil {
					return fmt.Errorf(`%w: an adjacent sweep section carries incomplete rim topology`, ErrUnsupported)
				}
				if fromEdge == toEdge {
					return fmt.Errorf(`%w: a span reuses one rim edge at both path ends`, ErrUnsupported)
				}
			}
		}
	}
	return budget.err()
}

func sewCompositeSweepJoin(
	ctx context.Context,
	from, to *Face,
	previous *Body,
	sewn map[*Edge]struct{},
) error {
	for li := range from.loops {
		for ci := range from.loops[li].coedges {
			if err := ctx.Err(); err != nil {
				return err
			}
			fromUse := from.loops[li].coedges[ci]
			toUse := to.loops[li].coedges[ci]
			// The next span's start rim is evaluated directly in the certified
			// transported frame. Keep that edge and replace the preceding span's
			// independently rotated end rim, so the published join reads the
			// exact shared section rather than a second libm image of it.
			shared, discarded := toUse.edge, fromUse.edge
			fromStart, fromEnd := fromUse.Start(), fromUse.End()
			toStart, toEnd := toUse.Start(), toUse.End()
			rewriteCompositeVertex(previous, fromStart, toStart)
			rewriteCompositeVertex(previous, fromEnd, toEnd)
			rewriteCompositeEdge(previous, discarded, shared)
			shared.faces = append(shared.faces, discarded.faces...)
			sewn[shared] = struct{}{}
		}
	}
	return nil
}

func rewriteCompositeVertex(body *Body, old, replacement *Vertex) {
	if old == replacement {
		return
	}
	for _, edge := range body.Edges() {
		if edge.start == old {
			edge.start = replacement
		}
		if edge.end == old {
			edge.end = replacement
		}
	}
}

func rewriteCompositeEdge(body *Body, old, replacement *Edge) {
	for _, face := range body.Faces() {
		for _, loop := range face.loops {
			for i := range loop.coedges {
				if loop.coedges[i].edge == old {
					loop.coedges[i].edge = replacement
				}
			}
		}
	}
}

func rewriteCompositeSideRoles(face *Face, span int) {
	for i, origin := range face.origins {
		suffix, ok := strings.CutPrefix(origin.Role, "side(")
		if !ok {
			continue
		}
		// A single-span reduction has already inserted span 0. Replace it
		// instead of nesting another path index around it.
		if rest, found := strings.CutPrefix(suffix, "0,"); found && strings.Count(suffix, ",") >= 2 {
			suffix = rest
		}
		origin.Role = fmt.Sprintf("side(%d,%s", span, suffix)
		face.origins[i] = origin
	}
}

func liveCompositeFaces(faces []*Face, body *Body) []*Face {
	out := make([]*Face, 0, 2)
	for _, face := range faces {
		if face == nil || face.body != body || slices.Contains(out, face) {
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
// its recorded topology. A solid's every edge has exactly two incident faces
// by construction, so the relaxation below to "one or two" changes nothing a
// solid can ever exercise; it only admits the free edges a surface result's
// two omitted outer caps leave behind.
func auditCompositeBoundary(ctx context.Context, body *Body, sewn map[*Edge]struct{}) error {
	if body == nil {
		return fmt.Errorf(`%w: a nil composite sweep body has no boundary`, ErrDegenerate)
	}
	budget := newWorkBudget(ctx)
	uses := make(map[*Edge][]compositeCoedgeUse)
	for _, face := range body.Faces() {
		if err := budget.step(); err != nil {
			return err
		}
		if admitAbove(measuredScalar(face.area, face.areaBound), 0) != survAdmit {
			return fmt.Errorf(`%w: a composite sweep face is not proven to have positive area`, ErrUnsupported)
		}
		if len(face.loops) == 0 {
			return fmt.Errorf(`%w: a composite sweep face has no boundary loop`, ErrUnsupported)
		}
		for _, loop := range face.loops {
			if len(loop.coedges) == 0 {
				return fmt.Errorf(`%w: a composite sweep face has an empty boundary loop`, ErrUnsupported)
			}
			for _, use := range loop.coedges {
				if err := budget.step(); err != nil {
					return err
				}
				if use.edge == nil {
					return fmt.Errorf(`%w: a composite sweep loop has a nil edge`, ErrUnsupported)
				}
				uses[use.edge] = append(uses[use.edge], compositeCoedgeUse{face: face, forward: use.forward})
			}
			for i, use := range loop.coedges {
				next := loop.coedges[(i+1)%len(loop.coedges)]
				if use.End() != next.Start() {
					return fmt.Errorf(`%w: a composite sweep face loop is not endpoint-continuous`, ErrUnsupported)
				}
			}
		}
	}

	for edge, edgeUses := range uses {
		if err := budget.step(); err != nil {
			return err
		}
		// An edge may have one or two incident faces (a surface result's own
		// free rim edge, or an interior edge on either kind), but never zero or
		// three or more, and the use count read off every face's own loops must
		// agree with Edge.Faces()'s own adjacency exactly either way.
		if len(edge.faces) < 1 || len(edge.faces) > 2 || len(edgeUses) != len(edge.faces) {
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
	if err := auditCompositeVertexLinks(budget, body, uses); err != nil {
		return err
	}
	return budget.err()
}

// auditCompositeVertexLinks proves the manifold-with-boundary invariant at
// every vertex: the faces touching it, linked by the two-face edges also
// touching it, form one connected component whose own face degrees (how many
// such link edges reach that face) are all 1 or all-but-two 2. Two degree-1
// faces make it a PATH — a rim vertex, where exactly two of the incident
// edges are free and end the fan instead of continuing it. Zero degree-1
// faces make it a CYCLE — an interior vertex, exactly the shape the pre-sheet
// audit already proved. A free (one-face) edge contributes its lone face to
// the vertex's link as a node with no link edge of its own, since it has no
// second face to link that face to there.
func auditCompositeVertexLinks(
	budget *workBudget,
	body *Body,
	uses map[*Edge][]compositeCoedgeUse,
) error {
	type vertexLink struct {
		faces     map[*Face]struct{}
		degree    map[*Face]int
		neighbors map[*Face]map[*Face]struct{}
	}
	links := make(map[*Vertex]*vertexLink)
	linkFor := func(vertex *Vertex) *vertexLink {
		link := links[vertex]
		if link == nil {
			link = &vertexLink{
				faces:     make(map[*Face]struct{}),
				degree:    make(map[*Face]int),
				neighbors: make(map[*Face]map[*Face]struct{}),
			}
			links[vertex] = link
		}
		return link
	}
	for edge, edgeUses := range uses {
		if edge.start == nil || edge.end == nil || len(edgeUses) < 1 || len(edgeUses) > 2 {
			return fmt.Errorf(`%w: a composite sweep edge has incomplete endpoint topology`, ErrUnsupported)
		}
		for _, vertex := range []*Vertex{edge.start, edge.end} {
			if err := budget.step(); err != nil {
				return err
			}
			link := linkFor(vertex)
			for _, use := range edgeUses {
				link.faces[use.face] = struct{}{}
			}
			if len(edgeUses) != 2 {
				continue
			}
			a, b := edgeUses[0].face, edgeUses[1].face
			link.degree[a]++
			link.degree[b]++
			if link.neighbors[a] == nil {
				link.neighbors[a] = make(map[*Face]struct{})
			}
			if link.neighbors[b] == nil {
				link.neighbors[b] = make(map[*Face]struct{})
			}
			link.neighbors[a][b] = struct{}{}
			link.neighbors[b][a] = struct{}{}
		}
	}
	if len(links) != len(body.Vertices()) {
		return fmt.Errorf(`%w: a composite sweep vertex is absent from its boundary links`, ErrUnsupported)
	}
	for _, link := range links {
		if err := budget.step(); err != nil {
			return err
		}
		ends := 0
		for face := range link.faces {
			switch d := link.degree[face]; {
			case d == 1:
				ends++
			case d == 2:
			case d == 0 && len(link.faces) == 1:
				// A whole Circle3 rim uses its one vertex TWICE
				// (docs/surface-design.md §5.2): a hole wall's own full-circle
				// free rim closes back on itself there with no OTHER face ever
				// sharing that vertex, so this face has no two-face edge to
				// link it to anything — a lone node, trivially its own single
				// connected component, is exactly what a self-closing free
				// boundary through one point looks like.
			default:
				return fmt.Errorf(`%w: a composite sweep vertex link face has degree outside one or two`, ErrUnsupported)
			}
		}
		if len(link.faces) > 1 && ends != 0 && ends != 2 {
			return fmt.Errorf(`%w: a composite sweep vertex link is neither a cycle nor a path`, ErrUnsupported)
		}
		var seed *Face
		for face := range link.faces {
			seed = face
			break
		}
		seen := map[*Face]struct{}{seed: {}}
		stack := []*Face{seed}
		for len(stack) != 0 {
			last := len(stack) - 1
			face := stack[last]
			stack = stack[:last]
			for neighbor := range link.neighbors[face] {
				if _, ok := seen[neighbor]; ok {
					continue
				}
				seen[neighbor] = struct{}{}
				stack = append(stack, neighbor)
			}
		}
		if len(seen) != len(link.faces) {
			return fmt.Errorf(`%w: a composite sweep vertex link has more than one connected component`, ErrUnsupported)
		}
	}
	return nil
}
