package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
)

// revolveLoftFacesSeamVertex is one shared held point on a certified radial
// source arc. The two bounds belong to the finite Revolve and LoftFaces sides.
type revolveLoftFacesSeamVertex struct {
	station    int
	exact      loftFaceRatVec
	point      r3.Vec
	blankBound float64
	loftBound  float64
}

// revolveLoftFacesArcSeamVerticesUnchecked preserves the root record shape.
// Its caller first authenticates the complete ordered source-arc station run.
func revolveLoftFacesArcSeamVerticesUnchecked(ctx context.Context, trim *loftFacesTrimRecord,
	band int, stations []int, root loftFaceInterval, section revolveSectionSpan,
) ([]revolveLoftFacesSeamVertex, bool, error) {
	vertices, ok, err := revolveloftfaces.ArcSeamVerticesUnchecked(ctx, trim, band, stations, root, section)
	if err != nil || !ok {
		return nil, false, err
	}
	result := make([]revolveLoftFacesSeamVertex, len(vertices))
	for i, vertex := range vertices {
		result[i] = revolveLoftFacesSeamVertex{
			station: vertex.Station, exact: vertex.Exact, point: vertex.Point,
			blankBound: vertex.BlankBound, loftBound: vertex.LoftBound,
		}
	}
	return result, true, nil
}
