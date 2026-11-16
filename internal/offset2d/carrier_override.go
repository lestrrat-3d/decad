package offset2d

import (
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CornerCarrier states one side of a corner from a recorded segment's own
// data (agent-docs/carrier-record-design.md §7.2): a line through the
// recorded Start with direction End − Start, or a circle of the recorded
// Center and radius. It replaces the walk-start anchor offset2d otherwise
// reads, which for a narrowed flank is a rounded cut point.
type CornerCarrier struct {
	Line   bool
	SU, SV float64 // line: the recorded Start
	DU, DV float64 // line: End − Start, in the walk's travel sense
	CU, CV float64 // circle: the recorded centre
	R      float64 // circle: the recorded radius (|Start − Center| for an ArcSeg)
}

// LineCarrier is the override of a recorded line, its direction taken in the
// walk's travel sense (the line is the same either way).
func LineCarrier(start, end sectionrecord.Point2, w survey2d.SideWalk) CornerCarrier {
	du, dv := end.U-start.U, end.V-start.V
	if du*w.TanInU+dv*w.TanInV < 0 {
		du, dv = -du, -dv
	}
	return CornerCarrier{Line: true, SU: start.U, SV: start.V, DU: du, DV: dv}
}

// CircleCarrier is the override of a recorded circle or arc carrier.
func CircleCarrier(center sectionrecord.Point2, radius float64) CornerCarrier {
	return CornerCarrier{CU: center.U, CV: center.V, R: radius}
}

// cornerTangent is a walk's unit travel tangent at the corner (px, py) and
// its length. arriving selects the walk's outgoing end tangent. With an
// override the tangent is the recorded line's normalized direction, or a
// circle's rot90 of the normalized corner − Center in the walk's sense.
func cornerTangent(w survey2d.SideWalk, ov *CornerCarrier, px, py float64, arriving bool) (float64, float64, float64) {
	if ov == nil {
		if arriving {
			return Normalize(w.TanOutU, w.TanOutV)
		}
		return Normalize(w.TanInU, w.TanInV)
	}
	if ov.Line {
		return Normalize(ov.DU, ov.DV)
	}
	ux, uy, l := Normalize(px-ov.CU, py-ov.CV)
	if l == 0 {
		return 0, 0, 0
	}
	if w.Th1 < w.Th0 {
		return uy, -ux, l
	}
	return -uy, ux, l
}
