package revolveloftfaces

import (
	"github.com/lestrrat-3d/decad/internal/loftface"
)

func SameCrossing(a, b loftface.RootCrossing) bool {
	return a.Span == b.Span && loftface.RatEqual(a.Lo, b.Lo) &&
		loftface.RatEqual(a.Hi, b.Hi)
}

func SameMappedCell(a, b RevolveLoftFacesMappedCell) bool {
	return a.Segment == b.Segment && a.Span == b.Span && a.Band == b.Band &&
		a.Side == b.Side && a.JointKind == b.JointKind &&
		a.JointPoint == b.JointPoint && a.JointArc == b.JointArc &&
		SameCrossing(a.First, b.First) &&
		SameCrossing(a.Last, b.Last) &&
		loftface.IntervalEqual(a.XFirst, b.XFirst) &&
		loftface.IntervalEqual(a.XLast, b.XLast) &&
		a.XDirection == b.XDirection &&
		loftface.RatEqual(a.SLo, b.SLo) &&
		loftface.RatEqual(a.SHi, b.SHi) &&
		loftface.RatEqual(a.TLo, b.TLo) &&
		loftface.RatEqual(a.THi, b.THi) &&
		a.TLoOpen == b.TLoOpen && a.THiOpen == b.THiOpen
}

func SameArcContact(a, b RevolveLoftFacesMappedCell) bool {
	return a.Segment == b.Segment && a.Span == b.Span && a.Band == b.Band &&
		a.Side == RevolveLoftFacesJointContact && b.Side == a.Side &&
		a.JointKind == RevolveLoftFacesWholeArcContact && b.JointKind == a.JointKind &&
		a.JointArc == b.JointArc && a.JointPoint == b.JointPoint &&
		a.SLo != nil && b.SLo != nil && a.SLo.Cmp(b.SLo) == 0 &&
		a.SHi != nil && b.SHi != nil && a.SHi.Cmp(b.SHi) == 0 &&
		a.TLo != nil && b.TLo != nil && a.TLo.Cmp(b.TLo) == 0 &&
		a.THi != nil && b.THi != nil && a.THi.Cmp(b.THi) == 0 &&
		a.TLoOpen == b.TLoOpen && a.THiOpen == b.THiOpen
}
