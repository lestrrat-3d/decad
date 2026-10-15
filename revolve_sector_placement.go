package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
)

// revolveSectorSourcesSeparate checks each later sector's recorded axis
// placement, charges its mesh bound for that placement, and asks
// facetproof.SectorSourcesSeparate for whole-source separation.
func revolveSectorSourcesSeparate(sectors []facetedPayload) bool {
	sources := make([]facetproof.SectorSource, len(sectors))
	for i, sector := range sectors {
		charge := massmoment.MapCharge{}
		if i != 0 {
			if sector.revolveLoftSector == nil ||
				sector.Xform != sector.revolveLoftSector.motion {
				return false
			}
			var err error
			charge, err = massmoment.XAxisPlacementCharge(sector.revolveLoftSector.motion)
			if err != nil {
				return false
			}
		}
		sources[i] = facetproof.SectorSource{Verts: sector.Verts, MeshBound: sector.MeshBound,
			Bound: massmoment.PlacedBound(sector.MeshBound, charge)}
	}
	return facetproof.SectorSourcesSeparate(sources)
}

// revolveSectorSeam reads the sector loop's first and last root contacts and
// asks facetproof.SectorSeam for a seam outside every certified root sector.
func revolveSectorSeam(path []revolveSectorNode, motions []r3.Transform) (float64, error) {
	if len(path) < 2 || len(motions) == 0 {
		return 0, fmt.Errorf("%w: sector source has no root contacts", ErrUnsupported)
	}
	return facetproof.SectorSeam(path[0].point, path[len(path)-1].point, motions)
}
