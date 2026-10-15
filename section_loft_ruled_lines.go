package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// loftRuledLineSectionRecord intersects the exact triangular walls of a
// direct line-only Loft. Each line pair contributes two triangles with one
// diagonal, so its interior section can contain a kink at that diagonal.
// Zero vertex displacement ties the held triangle corners to the records.
// Placement, source-origin, cap-level, and rewrite-marker guards are reject-only
// for current Loft producers; keep them if those producers gain new routes.
func loftRuledLineSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	if !pl.AuthenticatedReconstruction || pl.SurfaceResult || pl.Xform != r3.Identity() ||
		pl.Delta != 0 || pl.SectionDelta != 0 || pl.Chorded ||
		pl.ConstructionDelta != 0 || len(pl.BlendSegs) != 0 ||
		pl.ThroughBoreCut || pl.AnalyticBore ||
		len(pl.Profile0.Holes) != len(pl.Profile1.Holes) ||
		(len(pl.Alignment) != 0 && len(pl.Alignment) != 1+len(pl.Profile0.Holes)) {
		return profileRecord{}, false
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return profileRecord{}, false
		}
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.Frame0.U() != x || pl.Frame0.V() != y || pl.Frame0.N() != z ||
		pl.Frame1.U() != x || pl.Frame1.V() != y || pl.Frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z ||
		pl.Plane0.U != x || pl.Plane0.V != y ||
		pl.Plane1.U != x || pl.Plane1.V != y ||
		pl.Plane0.Origin != pl.Frame0.Origin() || pl.Plane1.Origin != pl.Frame1.Origin() ||
		pl.Frame0.Origin().X != pl.Frame1.Origin().X ||
		pl.Frame0.Origin().Y != pl.Frame1.Origin().Y ||
		pl.Frame0.Origin().X != plane.Origin().X ||
		pl.Frame0.Origin().Y != plane.Origin().Y {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.Frame0.Origin().Z), proofarith.FloatRat(pl.Frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(cut, z0), new(big.Rat).Sub(z1, z0))
	outer, ok := loftRuledLineSectionLoop(pl.Profile0.Outer, pl.Profile1.Outer, fraction)
	if !ok {
		return profileRecord{}, false
	}
	holes := make([]loopRecord, len(pl.Profile0.Holes))
	for i := range holes {
		holes[i], ok = loftRuledLineSectionLoop(pl.Profile0.Holes[i], pl.Profile1.Holes[i], fraction)
		if !ok {
			return profileRecord{}, false
		}
	}
	return profileRecord{Outer: outer, Holes: holes}, true
}

func loftRuledLineSectionLoop(lower, upper loopRecord, fraction *big.Rat) (loopRecord, bool) {
	if len(lower.Segments) == 0 || len(lower.Segments) != len(upper.Segments) {
		return loopRecord{}, false
	}
	section := loopRecord{Segments: make([]curveSegment, 0, 2*len(lower.Segments))}
	for i, lowerSegment := range lower.Segments {
		v0, v1, ok := loftSectionFullLineEnds(lowerSegment)
		if !ok {
			return loopRecord{}, false
		}
		w0, w1, ok := loftSectionFullLineEnds(upper.Segments[i])
		if !ok {
			return loopRecord{}, false
		}
		start, ok := loftRuledLineSectionPoint(v0, w0, fraction)
		if !ok {
			return loopRecord{}, false
		}
		diagonal, ok := loftRuledLineSectionPoint(v0, w1, fraction)
		if !ok {
			return loopRecord{}, false
		}
		end, ok := loftRuledLineSectionPoint(v1, w1, fraction)
		if !ok || start == diagonal || diagonal == end {
			return loopRecord{}, false
		}
		section.Segments = append(section.Segments,
			lineSeg{Start: start, End: diagonal, TStart: 0, TEnd: 1},
			lineSeg{Start: diagonal, End: end, TStart: 0, TEnd: 1})
	}
	return section, true
}

func loftSectionFullLineEnds(segment curveSegment) (sectionrecord.Point2, sectionrecord.Point2, bool) {
	line, ok := segment.(lineSeg)
	if !ok {
		return sectionrecord.Point2{}, sectionrecord.Point2{}, false
	}
	switch {
	case line.TStart == 0 && line.TEnd == 1:
		return line.Start, line.End, true
	case line.TStart == 1 && line.TEnd == 0:
		return line.End, line.Start, true
	default:
		return sectionrecord.Point2{}, sectionrecord.Point2{}, false
	}
}

func loftRuledLineSectionPoint(a, b sectionrecord.Point2, fraction *big.Rat) (sectionrecord.Point2, bool) {
	u, okU := loftSectionLerpExact(a.U, b.U, fraction)
	v, okV := loftSectionLerpExact(a.V, b.V, fraction)
	return sectionrecord.Point2{U: u, V: v}, okU && okV
}
