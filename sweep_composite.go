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
func assembleCompositeSweepBody(
	ctx context.Context,
	d *Document,
	ref producerID,
	parts []compositeSpanPart,
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

	body := &Body{
		doc:    d,
		origin: FeatureRef{producer: ref, Role: roleBody},
		solid:  true,
	}
	faces := []*Face{parts[0].startCap}
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
	parts[0].startCap.body = body
	parts[len(parts)-1].endCap.body = body
	faces = append(faces, parts[len(parts)-1].endCap)
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	for _, edge := range body.Edges() {
		edge.faces = liveCompositeFaces(edge.faces, body)
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
// audit checks that removing caps preserved positive faces, paired every edge
// in opposite directions, and left one cyclic fan around every vertex.
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
		if len(edgeUses) != 2 || len(edge.faces) != 2 {
			return fmt.Errorf(`%w: a sewn sweep edge does not have exactly two incident faces`, ErrUnsupported)
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

func auditCompositeVertexLinks(
	budget *workBudget,
	body *Body,
	uses map[*Edge][]compositeCoedgeUse,
) error {
	type vertexLink struct {
		degree    map[*Face]int
		neighbors map[*Face]map[*Face]struct{}
	}
	links := make(map[*Vertex]*vertexLink)
	for edge, edgeUses := range uses {
		if edge.start == nil || edge.end == nil || len(edgeUses) != 2 {
			return fmt.Errorf(`%w: a sewn sweep edge has incomplete endpoint topology`, ErrUnsupported)
		}
		for _, vertex := range []*Vertex{edge.start, edge.end} {
			if err := budget.step(); err != nil {
				return err
			}
			link := links[vertex]
			if link == nil {
				link = &vertexLink{degree: make(map[*Face]int), neighbors: make(map[*Face]map[*Face]struct{})}
				links[vertex] = link
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
		for _, degree := range link.degree {
			if degree != 2 {
				return fmt.Errorf(`%w: a composite sweep vertex link is not a cycle`, ErrUnsupported)
			}
		}
		var seed *Face
		for face := range link.degree {
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
		if len(seen) != len(link.degree) {
			return fmt.Errorf(`%w: a composite sweep vertex link has more than one cycle`, ErrUnsupported)
		}
	}
	return nil
}
