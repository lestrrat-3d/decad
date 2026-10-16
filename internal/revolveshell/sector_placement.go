package revolveshell

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
)

// RevolveSectorSourcesSeparate checks each later sector's recorded axis
// placement, charges its mesh bound for that placement, and asks
// facetproof.SectorSourcesSeparate for whole-source separation.
func RevolveSectorSourcesSeparate(sectors []facetedrecord.Payload) bool {
	sources := make([]facetproof.SectorSource, len(sectors))
	for i, sector := range sectors {
		charge := massmoment.MapCharge{}
		if i != 0 {
			if sector.RevolveLoftSector == nil ||
				sector.Xform != sector.RevolveLoftSector.Motion {
				return false
			}
			var err error
			charge, err = massmoment.XAxisPlacementCharge(sector.RevolveLoftSector.Motion)
			if err != nil {
				return false
			}
		}
		sources[i] = facetproof.SectorSource{Verts: sector.Verts, MeshBound: sector.MeshBound,
			Bound: massmoment.PlacedBound(sector.MeshBound, charge)}
	}
	return facetproof.SectorSourcesSeparate(sources)
}

// RevolveSectorSeam reads the sector loop's first and last root contacts and
// asks facetproof.SectorSeam for a seam outside every certified root sector.
func RevolveSectorSeam(path []facetedrecord.RevolveSectorNode, motions []r3.Transform) (float64, error) {
	if len(path) < 2 || len(motions) == 0 {
		return 0, fmt.Errorf("%w: sector source has no root contacts", decaderr.ErrUnsupported)
	}
	return facetproof.SectorSeam(path[0].Point, path[len(path)-1].Point, motions)
}
