package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// certifyRecordedCircleArcSeparation isolates the exact intersections of an
// adjacent root-circle and connector's supporting circles. Each candidate
// must lie strictly outside one recorded short arc, proved by interval cross
// products against that arc's endpoint rays.
func certifyRecordedCircleArcSeparation(circle circleSeg, arc arcSeg) error {
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 {
		return fmt.Errorf(`%w: the loft fillet root circle has no finite radius`, ErrUnsupported)
	}
	r, ok := proofbound.RatOf(radius)
	if !ok {
		return fmt.Errorf(`%w: the loft fillet root circle has no finite radius`, ErrUnsupported)
	}
	cx, cy := new(big.Rat).SetFloat64(circle.Center.U), new(big.Rat).SetFloat64(circle.Center.V)
	ax, ay := new(big.Rat).SetFloat64(arc.Center.U), new(big.Rat).SetFloat64(arc.Center.V)
	sx, sy := new(big.Rat).SetFloat64(arc.Start.U), new(big.Rat).SetFloat64(arc.Start.V)
	if cx == nil || cy == nil || ax == nil || ay == nil || sx == nil || sy == nil {
		return fmt.Errorf(`%w: the loft fillet circles have non-finite coordinates`, ErrUnsupported)
	}
	dx, dy := new(big.Rat).Sub(ax, cx), new(big.Rat).Sub(ay, cy)
	d2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	fx, fy := new(big.Rat).Sub(sx, ax), new(big.Rat).Sub(sy, ay)
	arcR2 := new(big.Rat).Add(new(big.Rat).Mul(fx, fx), new(big.Rat).Mul(fy, fy))
	r2 := new(big.Rat).Mul(r, r)
	if d2.Sign() == 0 || arcR2.Sign() == 0 {
		return fmt.Errorf(`%w: the loft fillet root and connector circles are degenerate`, ErrUnsupported)
	}
	projection := new(big.Rat).Quo(new(big.Rat).Add(new(big.Rat).Sub(r2, arcR2), d2),
		new(big.Rat).Mul(big.NewRat(2, 1), d2))
	height2 := new(big.Rat).Sub(r2, new(big.Rat).Mul(new(big.Rat).Mul(projection, projection), d2))
	if height2.Sign() < 0 {
		return nil
	}
	rootSeg := circularbounds.RecordSegment(circle)
	arcSeg := circularbounds.RecordSegment(arc)
	_, arcSweep, arcOK := circularbounds.WalkEnclosures(arcSeg)
	rootLo := min(circle.TStart, circle.TEnd)
	rootHi := max(circle.TStart, circle.TEnd)
	if !arcOK || arcSweep.Lo.Sign() <= 0 || arcSweep.Hi.Cmp(proofbound.PiLower) >= 0 ||
		new(big.Rat).Sub(new(big.Rat).SetFloat64(rootHi),
			new(big.Rat).SetFloat64(rootLo)).Cmp(big.NewRat(1, 2)) >= 0 {
		return fmt.Errorf(`%w: the loft fillet circles have no certified short arcs`, ErrUnsupported)
	}
	rsU, rsV, ok := circularbounds.EndpointInterval(rootSeg, new(big.Rat).SetFloat64(rootLo))
	if !ok {
		return fmt.Errorf(`%w: the loft fillet root-circle start is underivable`, ErrUnsupported)
	}
	reU, reV, ok := circularbounds.EndpointInterval(rootSeg, new(big.Rat).SetFloat64(rootHi))
	if !ok {
		return fmt.Errorf(`%w: the loft fillet root-circle end is underivable`, ErrUnsupported)
	}
	asU, asV, ok := circularbounds.EndpointInterval(arcSeg, big.NewRat(0, 1))
	if !ok {
		return fmt.Errorf(`%w: the loft fillet connector start is underivable`, ErrUnsupported)
	}
	aeU, aeV, ok := circularbounds.EndpointInterval(arcSeg, big.NewRat(1, 1))
	if !ok {
		return fmt.Errorf(`%w: the loft fillet connector end is underivable`, ErrUnsupported)
	}
	baseU := new(big.Rat).Add(cx, new(big.Rat).Mul(projection, dx))
	baseV := new(big.Rat).Add(cy, new(big.Rat).Mul(projection, dy))
	height, ok := proofbound.IntervalSqrt(proofbound.PointInterval(new(big.Rat).Quo(height2, d2)))
	if !ok {
		return fmt.Errorf(`%w: the loft fillet circle intersections are underivable`, ErrUnsupported)
	}
	for _, sign := range []int64{-1, 1} {
		shift := proofbound.IntervalScale(height, big.NewRat(sign, 1))
		pointU := proofbound.IntervalSub(proofbound.PointInterval(baseU),
			proofbound.IntervalScale(shift, dy))
		pointV := proofbound.IntervalAdd(proofbound.PointInterval(baseV),
			proofbound.IntervalScale(shift, dx))
		if !loftOutsideShortArc(pointU, pointV, cx, cy, rsU, rsV, reU, reV) &&
			!loftOutsideShortArc(pointU, pointV, ax, ay, asU, asV, aeU, aeV) {
			return fmt.Errorf(`%w: the loft fillet root and connector arcs may intersect`, ErrUnsupported)
		}
	}
	return nil
}

func loftOutsideShortArc(pU, pV proofbound.RatInterval, cx, cy *big.Rat,
	startU, startV, endU, endV proofbound.RatInterval) bool {
	cU, cV := proofbound.PointInterval(cx), proofbound.PointInterval(cy)
	sU, sV := proofbound.IntervalSub(startU, cU), proofbound.IntervalSub(startV, cV)
	eU, eV := proofbound.IntervalSub(endU, cU), proofbound.IntervalSub(endV, cV)
	pU, pV = proofbound.IntervalSub(pU, cU), proofbound.IntervalSub(pV, cV)
	startCross := proofbound.IntervalSub(proofbound.IntervalMul(sU, pV), proofbound.IntervalMul(sV, pU))
	endCross := proofbound.IntervalSub(proofbound.IntervalMul(pU, eV), proofbound.IntervalMul(pV, eU))
	return startCross.Hi.Sign() < 0 || endCross.Hi.Sign() < 0
}
