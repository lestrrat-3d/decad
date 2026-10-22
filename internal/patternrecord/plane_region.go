package patternrecord

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

type FitSplineSeg = sectionrecord.FitSplineSeg

// MovePlaneRegion re-records every segment of a region through the exact
// in-plane map pm, keeping each segment's kind, sense and range. It returns
// the moved region beside an upper bound on how far any point of the moved
// record sits from the exact image of the source curve it records: the larger
// endpoint charge for a line, the centre charge plus the radius' scale
// deviation for a circle, and for an arc three times its largest point charge
// plus twice the scale deviation of its radius. A fit spline is charged by
// the exact distance between its refit cubics and the image of its source
// cubics (fitMoveCharge).
//
// The map is a similarity (NewPlaneMap), so the image of a line is a line and
// the image of a circle or arc is a circle or arc. A segment kind with no such
// image is ErrUnsupported.
func MovePlaneRegion(budget *proofbound.WorkBudget, region Region, pm PlaneMap) (Region, float64, error) {
	mv := pm.Motion()
	delta := 0.0
	moveLoop := func(loop LoopRecord) (LoopRecord, error) {
		out := make([]CurveSegment, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return LoopRecord{}, err
			}
			moved, charge, err := movePlaneSegment(seg, pm, mv)
			if err != nil {
				return LoopRecord{}, err
			}
			out[i], delta = moved, max(delta, charge)
		}
		return LoopRecord{Segments: out}, nil
	}
	outer, err := moveLoop(region.Outer)
	if err != nil {
		return Region{}, 0, err
	}
	out := Region{Outer: outer}
	for _, hole := range region.Holes {
		moved, err := moveLoop(hole)
		if err != nil {
			return Region{}, 0, err
		}
		out.Holes = append(out.Holes, moved)
	}
	return out, delta, nil
}

func movePlaneSegment(seg CurveSegment, pm PlaneMap, mv PointMotion) (CurveSegment, float64, error) {
	pts := func(in ...Point2) ([]Point2, float64, error) {
		moved := make([]Point2, len(in))
		worst := 0.0
		for k, p := range in {
			m, e, err := mv(p)
			if err != nil {
				return nil, 0, err
			}
			moved[k], worst = m, math.Max(worst, e)
		}
		return moved, worst, nil
	}
	switch s := seg.(type) {
	case LineSeg:
		p, e, err := pts(s.Start, s.End)
		if err != nil {
			return nil, 0, err
		}
		s.Start, s.End = p[0], p[1]
		return s, e, nil
	case ArcSeg:
		radius := math.Max(dist2(s.Center, s.Start), dist2(s.Center, s.End))
		p, e, err := pts(s.Center, s.Start, s.End)
		if err != nil {
			return nil, 0, err
		}
		s.Center, s.Start, s.End = p[0], p[1], p[2]
		scale := proofbound.ProductUpper(2, proofbound.ProductUpper(radius, pm.ScaleDeviation()))
		return s, proofbound.AbsSumUpper(proofbound.ProductUpper(3, e), scale), nil
	case CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil {
			return nil, 0, err
		}
		p, e, err := pts(s.Center)
		if err != nil {
			return nil, 0, err
		}
		s.Center = p[0]
		return s, proofbound.AbsSumUpper(e, proofbound.ProductUpper(r, pm.ScaleDeviation())), nil
	case FitSplineSeg:
		moved := make([]Point2, len(s.Fit))
		for k, p := range s.Fit {
			m, _, err := mv(p)
			if err != nil {
				return nil, 0, err
			}
			moved[k] = m
		}
		charge, err := fitMoveCharge(s.Fit, moved, pm)
		if err != nil {
			return nil, 0, err
		}
		s.Fit = moved
		return s, charge, nil
	default:
		return nil, 0, fmt.Errorf(`%w: a %T segment has no exact plane motion`, decaderr.ErrUnsupported, seg)
	}
}

// dist2 is a float upper-leaning distance between two held points, used only
// to size a charge term that the caller multiplies by a tiny scale deviation.
func dist2(a, b Point2) float64 {
	return math.Nextafter(math.Hypot(a.U-b.U, a.V-b.V), math.Inf(1))
}

// fitMoveCharge bounds the distance between the refit spline through moved and
// the image under pm of the spline through src, in both directions. Both are
// piecewise cubics with exact rational coefficients (geom.FitInterpolant's
// ExactSpans), matched span for span at equal local parameter u. Every moved
// point sits within the motion's rounding of the image of its source point, so
// the two share a span count. The difference of a matched pair is a cubic in u
// whose Bernstein coefficients enclose it, so
// max over u of its length is at most the root of the sum of the largest
// squared coordinate coefficients. That bound is the Hausdorff distance's upper
// bound for the pairing: it is an enclosure the caller adds to the record's
// displacement, and it admits nothing.
func fitMoveCharge(src, moved []Point2, pm PlaneMap) (float64, error) {
	hold := func(in []Point2) [][2]float64 {
		out := make([][2]float64, len(in))
		for i, p := range in {
			out[i] = [2]float64{p.U, p.V}
		}
		return out
	}
	fs, err := geom.NewFitInterpolant(hold(src))
	if err != nil {
		return 0, fmt.Errorf(`%w: the source fit spline has no interpolant: %s`, decaderr.ErrUnsupported, err)
	}
	fq, err := geom.NewFitInterpolant(hold(moved))
	if err != nil {
		return 0, fmt.Errorf(`%w: the moved fit spline has no interpolant: %s`, decaderr.ErrUnsupported, err)
	}
	ss, err := fs.ExactSpans()
	if err != nil {
		return 0, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, err)
	}
	qs, err := fq.ExactSpans()
	if err != nil {
		return 0, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, err)
	}
	if len(ss) == 0 || len(ss) != len(qs) {
		return 0, fmt.Errorf(`%w: the moved fit spline does not keep the source's %d spans`, decaderr.ErrUnsupported, len(ss))
	}
	worst := new(big.Rat)
	for j := range ss {
		var dx, dy [4]*big.Rat
		for k := range dx {
			ix := proofbound.RatAdd(proofbound.RatMul(pm.A[0][0], ss[j].X[k]), proofbound.RatMul(pm.A[0][1], ss[j].Y[k]))
			iy := proofbound.RatAdd(proofbound.RatMul(pm.A[1][0], ss[j].X[k]), proofbound.RatMul(pm.A[1][1], ss[j].Y[k]))
			if k == 0 {
				ix, iy = proofbound.RatAdd(ix, pm.T[0]), proofbound.RatAdd(iy, pm.T[1])
			}
			dx[k] = new(big.Rat).Sub(qs[j].X[k], ix)
			dy[k] = new(big.Rat).Sub(qs[j].Y[k], iy)
		}
		mx, my := bernsteinMax(dx), bernsteinMax(dy)
		sq := proofbound.RatAdd(proofbound.RatMul(mx, mx), proofbound.RatMul(my, my))
		if sq.Cmp(worst) > 0 {
			worst = sq
		}
	}
	return proofbound.RatSqrtUp(worst), nil
}

// bernsteinMax is the largest absolute Bernstein coefficient of the cubic
// c0 + c1 u + c2 u² + c3 u³ on [0, 1], an upper bound of |cubic| there.
func bernsteinMax(c [4]*big.Rat) *big.Rat {
	third := big.NewRat(1, 3)
	b0 := c[0]
	b1 := proofbound.RatAdd(c[0], proofbound.RatMul(c[1], third))
	b2 := proofbound.RatAdd(c[0], proofbound.RatMul(c[1], big.NewRat(2, 3)), proofbound.RatMul(c[2], third))
	b3 := proofbound.RatAdd(c[0], c[1], c[2], c[3])
	best := new(big.Rat)
	for _, b := range []*big.Rat{b0, b1, b2, b3} {
		if a := new(big.Rat).Abs(b); a.Cmp(best) > 0 {
			best = a
		}
	}
	return best
}
