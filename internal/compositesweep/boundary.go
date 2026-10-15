package compositesweep

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegroup"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// CoedgeUse is one face's directed use of an edge, the value AuditVertexLinks
// reads per edge.
type CoedgeUse struct {
	Face    *topo.Face
	Forward bool
}

// AuditBoundary proves the topology facts introduced by sewing. The
// analytic span builders already prove each face and its outward sense; this
// audit checks that removing caps preserved positive faces, paired every
// two-face edge in opposite directions, and left every vertex's face fan a
// single cycle (a closed solid) or a single path (a surface result's two
// outer rims) — the same manifold-with-boundary invariant
// docs/surface-design.md §9.1 states for the sheet validity audit, restated
// here because this audit runs on the ASSEMBLED composite body rather than
// its recorded topology. The common audit admits free edges for sheets.
// A closed composite solid checks exactly two faces per edge after this audit.
func AuditBoundary(ctx context.Context, body *topo.Body, sewn map[*topo.Edge]struct{}) error {
	if body == nil {
		return fmt.Errorf(`%w: a nil composite sweep body has no boundary`, decaderr.ErrDegenerate)
	}
	budget := proofbound.NewWorkBudget(ctx)
	uses := make(map[*topo.Edge][]CoedgeUse)
	for _, face := range topobuild.Faces(body) {
		if err := budget.Step(); err != nil {
			return err
		}
		if proofbound.AdmitAbove(proofbound.MeasuredScalar(face.Area, face.AreaBound), 0) != proofbound.SurvAdmit {
			return fmt.Errorf(`%w: a composite sweep face is not proven to have positive area`, decaderr.ErrUnsupported)
		}
		if len(face.Loops) == 0 {
			return fmt.Errorf(`%w: a composite sweep face has no boundary loop`, decaderr.ErrUnsupported)
		}
		for _, loop := range face.Loops {
			if len(loop.CoEdges) == 0 {
				return fmt.Errorf(`%w: a composite sweep face has an empty boundary loop`, decaderr.ErrUnsupported)
			}
			for _, use := range loop.CoEdges {
				if err := budget.Step(); err != nil {
					return err
				}
				if use.Edge == nil {
					return fmt.Errorf(`%w: a composite sweep loop has a nil edge`, decaderr.ErrUnsupported)
				}
				uses[use.Edge] = append(uses[use.Edge], CoedgeUse{Face: face, Forward: use.Forward})
			}
			for i, use := range loop.CoEdges {
				next := loop.CoEdges[(i+1)%len(loop.CoEdges)]
				if topobuild.CoEdgeEnd(use) != topobuild.CoEdgeStart(next) {
					return fmt.Errorf(`%w: a composite sweep face loop is not endpoint-continuous`, decaderr.ErrUnsupported)
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
		// agree with the edge's own face adjacency exactly either way.
		if len(edge.Faces) < 1 || len(edge.Faces) > 2 || len(edgeUses) != len(edge.Faces) {
			return fmt.Errorf(`%w: a composite sweep edge does not have one or two incident faces`, decaderr.ErrUnsupported)
		}
		if len(edgeUses) != 2 {
			continue
		}
		if edgeUses[0].Face == edgeUses[1].Face {
			return fmt.Errorf(`%w: a composite sweep edge is used twice by one face`, decaderr.ErrUnsupported)
		}
		if _, isSewn := sewn[edge]; isSewn && edgeUses[0].Forward == edgeUses[1].Forward {
			return fmt.Errorf(`%w: a sewn sweep edge does not have opposite directed uses`, decaderr.ErrUnsupported)
		}
	}
	if err := AuditVertexLinks(budget, uses); err != nil {
		return err
	}
	return budget.Err()
}

// AuditVertexLinks proves the manifold-with-boundary invariant at every
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
// This takes no body: comparing len(links) against the body's vertex count
// would be a tautology, never a fact about the geometry. The body's vertices
// derive from its edge walk, which walks the identical face.Loops ->
// loop.CoEdges structure uses is itself built from, over the SAME edge set
// (every caller builds uses over every edge of every face's every loop, and
// this function itself refuses any edge with a nil endpoint before the counts
// could ever be compared), so the two vertex sets can never disagree. Taking
// no body is also what lets this function's second caller (root Stitch's
// curved-body gate) reach it: that caller builds its own uses map over a set
// of faces that has no body at all until after this audit is meant to run.
func AuditVertexLinks(
	budget *proofbound.WorkBudget,
	uses map[*topo.Edge][]CoedgeUse,
) error {
	edges := make(map[*topo.Edge]surfacegroup.LinkEdge[*topo.Face, *topo.Vertex], len(uses))
	for edge, edgeUses := range uses {
		faces := make([]*topo.Face, len(edgeUses))
		for i, use := range edgeUses {
			faces[i] = use.Face
		}
		edges[edge] = surfacegroup.LinkEdge[*topo.Face, *topo.Vertex]{
			Start: edge.Start, End: edge.End, Faces: faces, EndpointsValid: edge.Start != nil && edge.End != nil,
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
		return fmt.Errorf(`%w: a composite sweep edge has incomplete endpoint topology`, decaderr.ErrUnsupported)
	case surfacegroup.LinkDegree:
		return fmt.Errorf(`%w: a composite sweep vertex link face has degree outside one or two`, decaderr.ErrUnsupported)
	case surfacegroup.LinkShape:
		return fmt.Errorf(`%w: a composite sweep vertex link is neither a cycle nor a path`, decaderr.ErrUnsupported)
	case surfacegroup.LinkDisconnected:
		return fmt.Errorf(`%w: a composite sweep vertex link has more than one connected component`, decaderr.ErrUnsupported)
	default:
		return fmt.Errorf(`%w: an unknown composite sweep vertex link state`, decaderr.ErrUnsupported)
	}
}
