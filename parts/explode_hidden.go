package parts

import (
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Occlusion says that one part may hide another seen along a view direction.
type Occlusion struct {
	Front, Back string
	Covered     units.Value // Dimensionless: the share of Back's projected box that Front's covers
}

// Hidden lists the pairs of exploded parts where one may hide the other seen along
// view, which points from the camera into the scene: each part's box is projected
// orthographically along view to a rectangle, and a pair whose rectangles overlap
// is listed with the nearer part (the one whose box centre is nearer the camera)
// as Front, and the share of Back's rectangle Front's covers. A pair absent from
// the list cannot hide one another; a listed pair may, since a box is larger than
// its part. Hidden moves nothing. A zero view is decad.ErrDegenerate and a
// non-finite one decad.ErrNotFinite. Pairs come in Parts order.
func (e *Exploded) Hidden(view r3.Vec) ([]Occlusion, error) {
	if e == nil {
		return nil, fmt.Errorf("parts: %w: nil exploded view", decad.ErrDegenerate)
	}
	if !finiteVec(view) {
		return nil, fmt.Errorf("parts: view: %w", decad.ErrNotFinite)
	}
	if view == (r3.Vec{}) {
		return nil, fmt.Errorf("parts: %w: the view has no direction", decad.ErrDegenerate)
	}
	var out []Occlusion
	for a := range e.boxes {
		for b := a + 1; b < len(e.boxes); b++ {
			front, cover := explodeplan.ProjectedCover(e.boxes[a], e.boxes[b], view)
			if cover <= 0 {
				continue
			}
			f, k := e.Parts[a].Name, e.Parts[b].Name
			if front == 1 {
				f, k = k, f
			}
			out = append(out, Occlusion{Front: f, Back: k, Covered: units.Scalar(cover)})
		}
	}
	return out, nil
}
