package topobuild

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/surfacegroup"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// FreeChainCountsByFace counts connected components of each face's recorded
// free edges over their shared vertex nodes. Faces without a free edge are
// absent from the result.
func FreeChainCountsByFace(body *topo.Body) map[*topo.Face]int {
	byFace := map[*topo.Face][][2]*topo.Vertex{}
	for _, edge := range Edges(body) {
		if len(edge.Faces) != 1 {
			continue
		}
		face := edge.Faces[0]
		byFace[face] = append(byFace[face], [2]*topo.Vertex{edge.Start, edge.End})
	}
	counts := make(map[*topo.Face]int, len(byFace))
	for face, edges := range byFace {
		counts[face] = surfacegroup.ChainCount(edges)
	}
	return counts
}

// RequireSheetMesh audits a sheet's free-boundary attribution and vertex links
// in that order. The body's face nodes are numbered in Faces order; a source
// node outside the body gets a fresh number in first-seen triangle order.
// body and mesh must be non-nil.
func RequireSheetMesh(ctx context.Context, body *topo.Body, mesh *topo.Mesh) error {
	faces := Faces(body)
	number := make(map[*topo.Face]int, len(faces))
	for i, face := range faces {
		number[face] = i
	}
	source := make([]int, len(mesh.Source))
	for k, face := range mesh.Source {
		i, ok := number[face]
		if !ok {
			i = len(number)
			number[face] = i
		}
		source[k] = i
	}
	counts := FreeChainCountsByFace(body)
	byNumber := make(map[int]int, len(counts))
	for face, count := range counts {
		byNumber[number[face]] = count
	}
	if err := LiftAuditError(tessellation.RequireSheetBoundary(ctx, tessellation.SheetBoundary{
		Triangles: mesh.Triangles, SourceFaces: source, FreeChainCounts: byNumber,
	})); err != nil {
		return err
	}
	return LiftAuditError(tessellation.RequireSheetVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles))
}
