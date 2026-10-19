package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

func revolveLoftFacesVerticalTraceSplitWedges(ctx context.Context,
	shell *revolveLoftFacesBlankSeamShell, ring revolveLoftFacesShellCapRing,
	section int, nodes []revolveLoftFacesTraceNode, path []revolveLoftFacesTraceEdge,
) (bool, error) {
	read := revolveloftfaces.VerticalTraceSplitReaders[
		revolveLoftFacesTraceNode, revolveLoftFacesTraceEdge, revolveLoftFacesShellTriangleCell]{
		NodeVertex: func(node revolveLoftFacesTraceNode) int { return node.vertex },
		NodeRay:    func(node revolveLoftFacesTraceNode) int { return node.ray },
		EdgeEnds: func(edge revolveLoftFacesTraceEdge) (int, int) {
			return edge.start, edge.end
		},
		EdgeWedge: func(edge revolveLoftFacesTraceEdge) int { return edge.wedge },
		CellLabels: func(cell revolveLoftFacesShellTriangleCell) (int, int, int, int) {
			return cell.section, cell.band, cell.ray, cell.half
		},
		NewCell: func(section, band, ray, half int) revolveLoftFacesShellTriangleCell {
			return revolveLoftFacesShellTriangleCell{
				section: section, band: band, ray: ray, half: half,
			}
		},
	}
	tris, src, cells, built, err := revolveloftfaces.VerticalTraceSplitWedges(ctx,
		&shell.mesh, shell.sectionRings, shell.rayCount, ring.vertices,
		section, nodes, path, shell.triangleCells, read)
	if err != nil || !built {
		return false, err
	}
	shell.mesh.Tris, shell.mesh.Src, shell.triangleCells = tris, src, cells
	return true, nil
}
