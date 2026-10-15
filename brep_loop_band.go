package decad

import (
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// brepBandsOccupiedVolumeAdmission is capBlendOccupiedVolumeAdmission's
// question for every band of a route L body (modify-general Table DG's DG3 and
// DG4): each band is admitted as the cap-loop chamfer admits its own loop, as
// a whole turn, or a loop whose every corner is a line-line miter or an
// exactly tangent join. refusal is the first band's reason, nil when every
// band is admitted; err is a budget or context error.
func brepBandsOccupiedVolumeAdmission(budget *proofbound.WorkBudget, bp brepPayload) (error, error) {
	for bi, b := range bp.LoopBands {
		if err := b.Validate(bp, bi); err != nil {
			return nil, err
		}
		if b.Kind == breprecord.BandFillet {
			// Every corner class a fillet band holds is one its strips chord
			// slice by slice; SF1 refused the rest at the build, and the LF6
			// fan's stations are the connector arc's azimuths at every ring
			// (loop-fillet DF5).
			continue
		}
		if b.Selected != nil {
			// The selected chamfer has only straight, planar strips, whose
			// shared vertices are the record's exact contour endpoints.
			continue
		}
		refusal, err := capBlendOccupiedVolumeAdmission(budget, b.TessView(bp.Faces[b.Face], bp.Xform))
		if err != nil || refusal != nil {
			return refusal, err
		}
	}
	return nil, nil //nolint:nilnil // no band refuses and no budget ran out
}
