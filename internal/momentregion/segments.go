package momentregion

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Plan is a free-form segment's converted chain and recorded walk direction.
type Plan struct {
	Spans    []freeform.BezierSpan
	Reversed bool
}

// AddFreeform folds exact span moments into the region's rational and float sums.
func (s State) AddFreeform(spans []freeform.BezierSpan, reversed bool, order freeform.MomentIntegralOrder) {
	exact := freeform.ExactFreeformMoments(spans, reversed, order)
	if extent := freeform.FreeformControlExtent(spans); extent > *s.CoordUpper {
		*s.CoordUpper = extent
	}
	moments := []struct {
		value *float64
		bound *float64
		exact *big.Rat
	}{
		{s.Fields[0].Value, s.Fields[0].Bound, exact.Area},
		{s.Fields[1].Value, s.Fields[1].Bound, exact.Mu},
		{s.Fields[2].Value, s.Fields[2].Bound, exact.Mv},
		{s.Fields[3].Value, s.Fields[3].Bound, exact.Muu},
		{s.Fields[4].Value, s.Fields[4].Bound, exact.Muv},
		{s.Fields[5].Value, s.Fields[5].Bound, exact.Mvv},
	}
	if order < freeform.MomentSecondOrder {
		moments = moments[:3]
	}
	for _, moment := range moments {
		held, _ := moment.exact.Float64()
		Accumulate(moment.value, moment.bound, held, proofarith.RationalFloatError(moment.exact, held))
	}
	s.AddExact(exact)
}

// AddLine folds a recorded line's neutral integral into the region sums.
func (s State) AddLine(seg sectionrecord.LineSeg, anchor sectionrecord.Point2, order freeform.MomentIntegralOrder) {
	shifted := seg
	shifted.Start = sectionrecord.Point2{U: seg.Start.U - anchor.U, V: seg.Start.V - anchor.V}
	shifted.End = sectionrecord.Point2{U: seg.End.U - anchor.U, V: seg.End.V - anchor.V}
	u0, v0 := Lerp2(shifted.Start, shifted.End, shifted.TStart)
	u1, v1 := Lerp2(shifted.Start, shifted.End, shifted.TEnd)
	_, _, coordUpper := boundarywalk.LineWalkBounds(shifted, math.Hypot(u1-u0, v1-v0))
	*s.CoordUpper = math.Max(*s.CoordUpper, coordUpper)

	line := momentline.Line{
		Start:  momentline.Point{U: seg.Start.U, V: seg.Start.V},
		End:    momentline.Point{U: seg.End.U, V: seg.End.V},
		TStart: seg.TStart, TEnd: seg.TEnd,
	}
	values, bounds, exact := momentline.Evaluate(line, momentline.Point{U: anchor.U, V: anchor.V}, order)
	for i, field := range s.Fields {
		if order < freeform.MomentSecondOrder && i >= 3 {
			break
		}
		Accumulate(field.Value, field.Bound, values[i], bounds[i])
	}
	s.AddExact(exact)
}

// AddCircular folds a bounded circular integral into the region sums.
func (s State) AddCircular(
	c sectionrecord.Point2,
	r, th0, th1, radiusUpper, sweepUpper float64,
	areaProof proofbound.RatInterval,
	haveAreaProof bool,
	muProof, mvProof proofbound.RatInterval,
	haveMomentProof bool,
	muuProof, muvProof, mvvProof proofbound.RatInterval,
	haveSecondMomentProof bool,
	order freeform.MomentIntegralOrder,
) {
	// Keep the area expression here so the independent section audit and this
	// integrator produce the same float bits on every supported platform.
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0
	area := 0.5 * (r*r*dth + c.U*r*(sin1-sin0) - c.V*r*(cos1-cos0))
	held := circularbounds.EvaluateFloat(
		circularbounds.RecordPoint(c), r, th0, th1, radiusUpper, sweepUpper, area,
		areaProof, haveAreaProof, muProof, mvProof, haveMomentProof,
		muuProof, muvProof, mvvProof, haveSecondMomentProof, order,
	)
	*s.CoordUpper = math.Max(*s.CoordUpper, held.CoordUpper)
	// Circular integrals have no exact rational because they contain π and trig terms.
	s.DropExact()
	for i, field := range s.Fields {
		if order < freeform.MomentSecondOrder && i >= 3 {
			break
		}
		Accumulate(field.Value, field.Bound, held.Values[i], held.Bounds[i])
	}
}

// Lerp2 returns a line's walked point, preserving natural endpoints verbatim.
func Lerp2(start, end sectionrecord.Point2, t float64) (float64, float64) {
	switch t {
	case 0:
		return start.U, start.V
	case 1:
		return end.U, end.V
	}
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}
