package booleaneval

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// View restates the solid a as the faceted payload a mesh Boolean would keep
// of it alone (agent-docs/faceted-source-retention-design.md §3): a's own
// tessellation, meshed by meshes at the chord the pair (a, a) derives, with
// one group per face of a carrying that face's retained tag
// (facetedrecord.SourceTag), its roles and its identity. The payload's vertex
// bounds, volume allowance and area slack are the tessellation's own; nothing
// is cut, welded or moved, so no other term is charged. The returned audit is
// the payload mesh's closure audit, for facetedrecord.BuildWithProof.
//
// A face whose surface SourceTag declines keeps no tag, and the built face
// reads as Faceted.
func View(ctx context.Context, a *topo.Body, meshes OperandMesher) (facetedrecord.Payload, *facetproof.MeshAudit, error) {
	if err := booloperand.RequireVolumeProof(ctx, a, 0); err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	tolMM, dPair, err := booloperand.PairChord(ctx, a, a)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	m, _, err := meshes.TessellateOperand(ctx, a, operandChord(ctx, a, tolMM, dPair, 1))
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	twin, err := unplacedTwin(ctx, a)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	placed, err := placedOperandDepartures(a, twin, m)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	faces := topobuild.Faces(a)
	groups := make([]facetedrecord.Group, 0, len(faces))
	sourceID := make(map[*topo.Face]int, len(faces))
	for i, f := range faces {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return facetedrecord.Payload{}, nil, err
			}
		}
		sourceID[f] = len(groups)
		groups = append(groups, sourceGroup(a, 0, i, f, false, placed))
	}
	src, err := meshbool.SourceIDs(ctx, m.Source, sourceID)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	sym, err := meshbool.OperandSymDiff(m.BoundaryOK, m.SymDiffOK, m.VolSymDiff)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	vertexBound, err := topobuild.VertexBounds(m)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	// The payload owns its slices: the mesher may hand the same mesh to a
	// later caller.
	payload := facetedrecord.Payload{
		Verts:       slices.Clone(m.Vertices),
		VertexBound: vertexBound,
		Tris:        slices.Clone(m.Triangles),
		Src:         src,
		Groups:      groups,
		MeshBound:   facetproof.FacetBoundMax(m.Triangles, vertexBound),
		VolSymDiff:  sym,
		AreaSlack:   m.AreaSlack,
		DPair:       dPair,
		Xform:       r3.Identity(),
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, payload.Verts, payload.Tris)
	if err != nil {
		return facetedrecord.Payload{}, nil, err
	}
	return payload, audit, nil
}
