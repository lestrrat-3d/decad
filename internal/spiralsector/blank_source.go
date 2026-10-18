package spiralsector

import (
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SixSideBlank identifies the finite six-span blank source in a recorded
// Revolve section. It declines sections with missing or repeated sides.
func SixSideBlank(section facetedrecord.RevolveSectionCertificate) (*facetedrecord.SpiralBlankSource, bool) {
	if len(section.Spans) != 6 || section.AxisSpan < 0 ||
		section.AxisSpan >= len(section.Spans) ||
		section.RootSpan < 0 || section.RootSpan >= len(section.Spans) {
		return nil, false
	}
	result := &facetedrecord.SpiralBlankSource{Section: section,
		RootFace: section.Spans[section.RootSpan].Face}
	if result.RootFace == nil {
		return nil, false
	}
	axis := section.Spans[section.AxisSpan]
	result.ToeAxis, result.HeelAxis = axis.Start, axis.End
	if result.ToeAxis.U > result.HeelAxis.U {
		result.ToeAxis, result.HeelAxis = result.HeelAxis, result.ToeAxis
	}
	var interior []sectionrecord.Point2
	for _, span := range section.Spans {
		if len(span.SourceEdges) != 1 {
			return nil, false
		}
		point := span.Start
		if point.V == 0 {
			continue
		}
		switch point.U {
		case result.ToeAxis.U:
			if result.ToeOuter != (sectionrecord.Point2{}) {
				return nil, false
			}
			result.ToeOuter = point
		case result.HeelAxis.U:
			if result.HeelOuter != (sectionrecord.Point2{}) {
				return nil, false
			}
			result.HeelOuter = point
		default:
			interior = append(interior, point)
		}
	}
	if len(interior) != 2 || result.ToeOuter == (sectionrecord.Point2{}) ||
		result.HeelOuter == (sectionrecord.Point2{}) {
		return nil, false
	}
	if interior[0].U > interior[1].U {
		interior[0], interior[1] = interior[1], interior[0]
	}
	result.ToeRoot, result.Ded = interior[0], interior[1]
	if !(result.ToeRoot.U < result.ToeAxis.U &&
		result.ToeAxis.U < result.Ded.U && result.Ded.U < result.HeelAxis.U) {
		return nil, false
	}
	cycle := [6]sectionrecord.Point2{result.ToeAxis, result.HeelAxis, result.HeelOuter,
		result.Ded, result.ToeRoot, result.ToeOuter}
	var seen [6]bool
	for _, span := range section.Spans {
		matched := false
		for edge := range cycle {
			start, end := cycle[edge], cycle[(edge+1)%len(cycle)]
			forward := span.Start == start && span.End == end
			reverse := span.Start == end && span.End == start
			if !forward && !reverse {
				continue
			}
			if seen[edge] || edge == 3 && span.Face != result.RootFace {
				return nil, false
			}
			seen[edge] = true
			result.EdgeIndex[edge] = span.SourceEdges[0]
			matched = true
			break
		}
		if !matched {
			return nil, false
		}
	}
	return result, seen[3]
}
