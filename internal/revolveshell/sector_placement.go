package revolveshell

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
)

// RevolveSectorSourcesSeparate checks each later sector's recorded placement
// as a rotation about axis (massmoment.AxisPlacementCharge), charges its mesh
// bound for that placement, and asks facetproof for whole-source separation:
// SectorSourcesSeparate for the canonical axis, and SectorSourcesSeparateAbout
// with axis's vector for any other. It needs no drift term, because it proves
// the denoted copies disjoint, and those are the copies the union holds
// (agent-docs/revolve-axis-design.md §5.5).
func RevolveSectorSourcesSeparate(sectors []facetedrecord.Payload, axis coneaxis.Axis) bool {
	sources := make([]facetproof.SectorSource, len(sectors))
	for i, sector := range sectors {
		charge := massmoment.MapCharge{}
		if i != 0 {
			if sector.RevolveLoftSector == nil ||
				sector.Xform != sector.RevolveLoftSector.Motion {
				return false
			}
			var err error
			charge, _, err = massmoment.AxisPlacementCharge(sector.RevolveLoftSector.Motion, axis)
			if err != nil {
				return false
			}
		}
		sources[i] = facetproof.SectorSource{Verts: sector.Verts, MeshBound: sector.MeshBound,
			Bound: massmoment.PlacedBound(sector.MeshBound, charge)}
	}
	if axis.Canonical() {
		return facetproof.SectorSourcesSeparate(sources)
	}
	return facetproof.SectorSourcesSeparateAbout(sources, axis.Vec())
}

// RevolveSectorSeam reads the sector loop's first and last root contacts and
// asks basis's SectorSeam for a seam outside every certified root sector.
func RevolveSectorSeam(basis facetproof.AngleBasis, path []facetedrecord.RevolveSectorNode,
	motions []r3.Transform) (float64, error) {
	if len(path) < 2 || len(motions) == 0 {
		return 0, fmt.Errorf("%w: sector source has no root contacts", decaderr.ErrUnsupported)
	}
	return basis.SectorSeam(path[0].Point, path[len(path)-1].Point, motions)
}
