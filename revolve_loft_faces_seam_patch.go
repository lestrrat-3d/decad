package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
)

// revolveLoftFacesSeamPatch is one open source-face patch split through a
// common held seam. Complete face bounds need separate curve and twist proofs.
type revolveLoftFacesSeamPatch struct {
	vertices     []r3.Vec
	triangles    [][3]int
	vertexBounds []float64
	origins      []provenance.Ref
	seam         []int
	stationIDs   []int
}

// revolveLoftFacesArcPatchesUnchecked preserves root graph ownership of
// station IDs. Its caller authenticates the live source pair and arc run.
func revolveLoftFacesArcPatchesUnchecked(ctx context.Context, trim *loftFacesTrimRecord,
	band, segment int, section revolveSectionSpan, seam []revolveLoftFacesSeamVertex,
) (revolveLoftFacesSeamPatch, revolveLoftFacesSeamPatch, bool, error) {
	entries := make([]revolveloftfaces.ArcSeamVertex, len(seam))
	for i, entry := range seam {
		entries[i] = revolveloftfaces.ArcSeamVertex{
			Station: entry.station, Exact: entry.exact, Point: entry.point,
			BlankBound: entry.blankBound, LoftBound: entry.loftBound,
		}
	}
	blank, loft, ok, err := revolveloftfaces.ArcPatchesUnchecked(ctx, trim, band, segment, section, entries)
	return revolveLoftFacesSeamPatch{
			vertices: blank.Vertices, triangles: blank.Triangles,
			vertexBounds: blank.VertexBounds, origins: blank.Origins, seam: blank.Seam,
		}, revolveLoftFacesSeamPatch{
			vertices: loft.Vertices, triangles: loft.Triangles,
			vertexBounds: loft.VertexBounds, origins: loft.Origins, seam: loft.Seam,
		}, ok, err
}
