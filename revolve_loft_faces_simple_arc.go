package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

func revolveLoftFacesSimpleArc(trim *loftFacesTrimRecord, segment int,
	arc arcSeg, reversed bool) (loftface.SimpleCell, bool) {
	var cell loftface.SimpleCell
	if segment < 0 || segment >= len(trim.Source.source.entities) ||
		arc.TStart != 0 && arc.TStart != 1 ||
		arc.TEnd != 0 && arc.TEnd != 1 ||
		arc.TStart == arc.TEnd || (arc.TStart > arc.TEnd) != reversed {
		return cell, false
	}
	if _, ok := trim.Source.source.entities[segment].(*sketch.Arc); !ok {
		return cell, false
	}
	center := revolveLoftFacesSimplePoint(arc.Center)
	start := revolveLoftFacesSimplePoint(arc.Start)
	end := revolveLoftFacesSimplePoint(arc.End)
	if !revolveLoftFacesSimplePointFinite(center) ||
		!revolveLoftFacesSimplePointFinite(start) ||
		!revolveLoftFacesSimplePointFinite(end) {
		return cell, false
	}
	radius2 := func(point freeform.RatPoint) *big.Rat {
		u := lfSub(point.U, center.U)
		v := lfSub(point.V, center.V)
		return lfAdd(lfMul(u, u), lfMul(v, v))
	}
	if radius2(start).Sign() <= 0 || radius2(start).Cmp(radius2(end)) != 0 {
		return cell, false
	}
	_, sweep, ok := circularbounds.WalkEnclosures(circularbounds.RecordSegment(arc))
	if !ok || sweep.Lo.Sign() <= 0 || sweep.Hi.Cmp(proofbound.PiLower) >= 0 {
		return cell, false
	}
	circularArc, ok := circularbounds.RecordSegment(arc).(circularbounds.ArcSeg)
	if !ok {
		return cell, false
	}
	u, v, ok := circularbounds.ArcBox(circularArc)
	if !ok || u.Lo == nil || u.Hi == nil || v.Lo == nil || v.Hi == nil {
		return cell, false
	}
	cell = loftface.SimpleCell{Segment: segment, Arc: &arc,
		Start: start, End: end,
		Box: [2]loftFaceInterval{{Lo: u.Lo, Hi: u.Hi}, {Lo: v.Lo, Hi: v.Hi}}}
	if reversed {
		cell.Start, cell.End = cell.End, cell.Start
	}
	return cell, true
}

func revolveLoftFacesSimplePoint(point planePoint) freeform.RatPoint {
	return freeform.RatPoint{U: lfF(point.U), V: lfF(point.V)}
}

func revolveLoftFacesSimplePointFinite(point freeform.RatPoint) bool {
	return point.U != nil && point.V != nil
}

// A short arc with both ends on one axis-aligned chord lies strictly away
// from its center in that coordinate throughout its interior. Equal exact
// endpoint radii ensure the arc's denoted endpoint is the recorded endpoint.
func revolveLoftFacesSimpleArcChordSide(arc arcSeg, axis int,
	contact *big.Rat, side int) bool {
	if contact == nil || side != -1 && side != 1 {
		return false
	}
	start := loftface.SimpleCoordinate(revolveLoftFacesSimplePoint(arc.Start), axis)
	end := loftface.SimpleCoordinate(revolveLoftFacesSimplePoint(arc.End), axis)
	center := loftface.SimpleCoordinate(revolveLoftFacesSimplePoint(arc.Center), axis)
	if start == nil || end == nil || center == nil ||
		start.Cmp(contact) != 0 || end.Cmp(contact) != 0 {
		return false
	}
	return side*(center.Cmp(contact)) < 0
}

// A short arc whose endpoints lie strictly in one half-plane stays there:
// leaving and reentering would traverse at least a half-turn. The coordinate
// perpendicular to that half-plane is strictly monotone on the arc.
func revolveLoftFacesSimpleArcMonotoneSide(arc arcSeg, axis int,
	contact *big.Rat, side int) bool {
	if contact == nil || side != -1 && side != 1 {
		return false
	}
	start, end := revolveLoftFacesSimplePoint(arc.Start),
		revolveLoftFacesSimplePoint(arc.End)
	center := revolveLoftFacesSimplePoint(arc.Center)
	if !revolveLoftFacesSimplePointFinite(start) ||
		!revolveLoftFacesSimplePointFinite(end) ||
		!revolveLoftFacesSimplePointFinite(center) {
		return false
	}
	startAxis := loftface.SimpleCoordinate(start, axis)
	endAxis := loftface.SimpleCoordinate(end, axis)
	otherAxis := 1 - axis
	startOrth := lfSub(loftface.SimpleCoordinate(start, otherAxis),
		loftface.SimpleCoordinate(center, otherAxis))
	endOrth := lfSub(loftface.SimpleCoordinate(end, otherAxis),
		loftface.SimpleCoordinate(center, otherAxis))
	if startOrth.Sign() == 0 || startOrth.Sign() != endOrth.Sign() {
		return false
	}
	return startAxis.Cmp(contact) == 0 && side*endAxis.Cmp(contact) > 0 ||
		endAxis.Cmp(contact) == 0 && side*startAxis.Cmp(contact) > 0
}

func revolveLoftFacesSimpleArcSide(arc arcSeg, axis int,
	contact *big.Rat, side int) bool {
	return revolveLoftFacesSimpleArcChordSide(arc, axis, contact, side) ||
		revolveLoftFacesSimpleArcMonotoneSide(arc, axis, contact, side)
}
