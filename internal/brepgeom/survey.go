package brepgeom

import (
	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// PullFace is the portion of a BRep face record needed by the undercut survey.
type PullFace struct {
	Frame     r3.Frame
	Transform r3.Transform
	Wall      sectionrecord.CurveSegment
	Planar    bool
	Outward   bool
}

// PullDecision reads one face's placed outward normal against the original
// pull. A planar face contributes its cap normal; a swept face contributes its
// wall's full normal range.
func PullDecision(face PullFace, pull r3.Vec, work *freeform.FreeformWork) (survey2d.PullVerdict, bool) {
	m, ok := survey2d.NewPlacedFrameMap(face.Frame, face.Transform)
	if !ok {
		return 0, false
	}
	if face.Planar {
		sign := -1.0
		if face.Outward {
			sign = 1
		}
		return survey2d.CapNormalDecision(m, pull, sign)
	}
	w, err := boundarywalk.WalkOf(face.Wall, work)
	if err != nil {
		return 0, false
	}
	return survey2d.WallNormalDecision(survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, m, pull)
}
