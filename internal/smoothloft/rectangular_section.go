package smoothloft

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// RectangularSection retains the exact height curves admitted by a rectangular
// LoftSections builder. Its fields are private so a consumer cannot change the
// source geometry after the held-mesh proof is established.
type RectangularSection struct {
	origin r3.Vec
	spans  []rectangularSectionSpan
}

type rectangularSectionSpan struct {
	start, end   *big.Rat
	width, depth [4]*big.Rat
}

// Origin is the common source-frame origin. Every builder requires world-XY
// axes and the same in-plane origin for its source sections.
func (s *RectangularSection) Origin() r3.Vec { return s.origin }

// At returns the exactly representable half-width and half-height at an
// interior world-Z level. The source builders already proved both dimensions
// positive throughout each span; an unrepresentable requested section refuses.
func (s *RectangularSection) At(z float64) (float64, float64, bool) {
	if s == nil || len(s.spans) == 0 || math.IsNaN(z) || math.IsInf(z, 0) {
		return 0, 0, false
	}
	level := rf(z)
	if level.Cmp(s.spans[0].start) <= 0 || level.Cmp(s.spans[len(s.spans)-1].end) >= 0 {
		return 0, 0, false
	}
	for _, span := range s.spans {
		if level.Cmp(span.start) < 0 || level.Cmp(span.end) > 0 {
			continue
		}
		t := div(sub(level, span.start), sub(span.end, span.start))
		w, errW := exactFloat(fourCubicAt(span.width, t))
		h, errH := exactFloat(fourCubicAt(span.depth, t))
		return w, h, errW == nil && errH == nil && w > 0 && h > 0
	}
	return 0, 0, false
}

func rectangularQuadraticControls(q rectangularQuadratic) [4]*big.Rat {
	middle := sub(mul(rint(2), q.middle), div(add(q.start, q.end), rint(2)))
	return [4]*big.Rat{q.start,
		div(add(q.start, mul(rint(2), middle)), rint(3)),
		div(add(mul(rint(2), middle), q.end), rint(3)), q.end}
}

func threeRectangularSection(origin, end r3.Vec, w, h rectangularQuadratic) *RectangularSection {
	return &RectangularSection{origin: origin, spans: []rectangularSectionSpan{{
		start: rf(origin.Z), end: rf(end.Z),
		width: rectangularQuadraticControls(w), depth: rectangularQuadraticControls(h),
	}}}
}

func fourRectangularSection(origin, end r3.Vec, w, h rectangularCubic) *RectangularSection {
	return &RectangularSection{origin: origin, spans: []rectangularSectionSpan{{
		start: rf(origin.Z), end: rf(end.Z), width: w.controls, depth: h.controls,
	}}}
}

func manyRectangularSection(origin r3.Vec, step *big.Rat, spans []rectangularSpan) *RectangularSection {
	section := &RectangularSection{origin: origin, spans: make([]rectangularSectionSpan, len(spans))}
	for i, span := range spans {
		start := add(rf(origin.Z), mul(step, rint(int64(i))))
		section.spans[i] = rectangularSectionSpan{
			start: start, end: add(start, step), width: span.width.controls, depth: span.depth.controls,
		}
	}
	return section
}
