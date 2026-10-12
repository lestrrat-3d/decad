package decad

import (
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// A joined-prism fillet contacts the original connector line and circular
// blank. The exact contact coordinates are algebraic; each held point carries
// an interval error obtained from exact dyadic source coordinates.
type prismPatternFilletContact struct {
	part, station int
	line, circle  r3.Vec
	center        r3.Vec
	pointError    float64
	tubeRadius    float64
	circleIV      prismPatternIVPoint
}

type prismPatternIVPoint struct{ x, y proofbound.RatInterval }

func prismPatternIVPointOf(x, y *big.Rat) prismPatternIVPoint {
	return prismPatternIVPoint{proofbound.PointInterval(x), proofbound.PointInterval(y)}
}

func (p prismPatternIVPoint) add(q prismPatternIVPoint) prismPatternIVPoint {
	return prismPatternIVPoint{proofbound.IntervalAdd(p.x, q.x), proofbound.IntervalAdd(p.y, q.y)}
}

func (p prismPatternIVPoint) sub(q prismPatternIVPoint) prismPatternIVPoint {
	return prismPatternIVPoint{proofbound.IntervalSub(p.x, q.x), proofbound.IntervalSub(p.y, q.y)}
}

func (p prismPatternIVPoint) scale(s proofbound.RatInterval) prismPatternIVPoint {
	return prismPatternIVPoint{proofbound.IntervalMul(p.x, s), proofbound.IntervalMul(p.y, s)}
}

func (p prismPatternIVPoint) dot(q prismPatternIVPoint) proofbound.RatInterval {
	return proofbound.IntervalAdd(proofbound.IntervalMul(p.x, q.x), proofbound.IntervalMul(p.y, q.y))
}

func prismPatternIVSqrt(a proofbound.RatInterval) (proofbound.RatInterval, error) {
	value, ok := proofbound.IntervalSqrt(a)
	if !ok || proofbound.IsNonFinite(proofbound.RatFloatUp(value.Hi)) {
		return proofbound.RatInterval{}, fmt.Errorf("%w: joined-prism fillet has no finite square-root bracket", ErrUnsupported)
	}
	return value, nil
}

func prismPatternIVQuo(a, b proofbound.RatInterval) (proofbound.RatInterval, error) {
	value, ok := proofbound.IntervalQuo(a, b)
	if !ok {
		return proofbound.RatInterval{}, fmt.Errorf("%w: joined-prism fillet has no separated denominator", ErrUnsupported)
	}
	return value, nil
}

func prismPatternIVHeld(p prismPatternIVPoint, z float64) (r3.Vec, float64, error) {
	read := func(iv proofbound.RatInterval) float64 {
		mid := new(big.Rat).Add(iv.Lo, iv.Hi)
		mid.Quo(mid, big.NewRat(2, 1))
		value, _ := mid.Float64()
		return value
	}
	x, y := read(p.x), read(p.y)
	point := r3.Vec{X: x, Y: y, Z: z}
	if !proofbound.FiniteVec(point) {
		return r3.Vec{}, 0, fmt.Errorf("%w: joined-prism fillet contact is non-finite", ErrUnsupported)
	}
	errorBound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(p.x, x),
		proofbound.IntervalFloatError(p.y, y))
	return point, errorBound, nil
}

func prismPatternExactPlacedPoint(part prismPatternPart, point Point2) prismPatternIVPoint {
	b := part.prism.xform.Basis()
	c, s := proofarith.FloatRat(b.EX.X), proofarith.FloatRat(b.EX.Y)
	u, v := proofarith.FloatRat(point.U), proofarith.FloatRat(point.V)
	x := new(big.Rat).Sub(new(big.Rat).Mul(c, u), new(big.Rat).Mul(s, v))
	y := new(big.Rat).Add(new(big.Rat).Mul(s, u), new(big.Rat).Mul(c, v))
	return prismPatternIVPointOf(x, y)
}

func prismPatternFilletContactOf(part prismPatternPart, partIndex, station int,
	root arcSeg, rootIndex int, blankRadius, radius float64) (prismPatternFilletContact, error) {
	var result prismPatternFilletContact
	if radius <= 0 || proofbound.IsNonFinite(radius) ||
		station != 0 && station != part.rootStart {
		return result, fmt.Errorf("%w: joined-prism fillet has no root corner or positive radius", ErrUnsupported)
	}
	lineIndex, side := 0, -1.0
	if station == part.rootStart {
		lineIndex, side = rootIndex-1, 1
	}
	if lineIndex < 0 || lineIndex >= len(part.prism.profile.Outer.Segments) {
		return result, fmt.Errorf("%w: joined-prism fillet lost its connector", ErrUnsupported)
	}
	connector, ok := part.prism.profile.Outer.Segments[lineIndex].(lineSeg)
	if !ok {
		return result, fmt.Errorf("%w: joined-prism fillet connector is not straight", ErrUnsupported)
	}
	near, far := connector.Start, connector.End
	if near != root.Start && near != root.End {
		near, far = far, near
	}
	if near != root.Start && near != root.End {
		return result, fmt.Errorf("%w: joined-prism fillet connector misses its root", ErrUnsupported)
	}
	q := prismPatternExactPlacedPoint(part, near)
	f := prismPatternExactPlacedPoint(part, far)
	heldRoot := part.mesh.vertices[2*station]
	rootError := proofbound.AbsSumUpper(proofbound.IntervalFloatError(q.x, heldRoot.X),
		proofbound.IntervalFloatError(q.y, heldRoot.Y))
	if rootError > part.mesh.Bound().Base() {
		return result, fmt.Errorf("%w: joined-prism fillet root is not tied to its source", ErrUnsupported)
	}
	d := f.sub(q)
	length, err := prismPatternIVSqrt(d.dot(d))
	if err != nil || length.Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet connector has no positive length", ErrUnsupported)
	}
	ux, err := prismPatternIVQuo(d.x, length)
	if err != nil {
		return result, err
	}
	uy, err := prismPatternIVQuo(d.y, length)
	if err != nil {
		return result, err
	}
	u := prismPatternIVPoint{ux, uy}
	n := prismPatternIVPoint{proofbound.IntervalScale(uy, proofarith.FloatRat(-side)),
		proofbound.IntervalScale(ux, proofarith.FloatRat(side))}
	r, R := proofarith.FloatRat(radius), proofarith.FloatRat(blankRadius)
	rIV := proofbound.PointInterval(r)
	RIV := proofbound.PointInterval(R)
	L := q.add(n.scale(rIV))
	a := L.dot(u)
	Rplus := proofbound.IntervalAdd(RIV, rIV)
	disc := proofbound.IntervalSub(proofbound.IntervalSquare(a),
		proofbound.IntervalSub(L.dot(L), proofbound.IntervalSquare(Rplus)))
	if disc.Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet offset has no distinct outer contact", ErrUnsupported)
	}
	squareRoot, err := prismPatternIVSqrt(disc)
	if err != nil {
		return result, err
	}
	t := proofbound.IntervalAdd(proofbound.IntervalNeg(a), squareRoot)
	if t.Lo.Sign() <= 0 || proofbound.IntervalSub(length, t).Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet contact escapes its connector", ErrUnsupported)
	}
	center := L.add(u.scale(t))
	lineFoot := q.add(u.scale(t))
	if lineFoot.dot(lineFoot).Lo.Cmp(prismPatternRadiusSq(blankRadius)) <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet line foot stays inside the blank", ErrUnsupported)
	}
	step, err := prismPatternIVQuo(RIV, Rplus)
	if err != nil {
		return result, err
	}
	circleFoot := center.scale(step)
	cross := proofbound.IntervalSub(proofbound.IntervalMul(q.x, circleFoot.y),
		proofbound.IntervalMul(q.y, circleFoot.x))
	if side < 0 && cross.Hi.Sign() >= 0 || side > 0 && cross.Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet contact leaves its circular gap", ErrUnsupported)
	}
	// A source corner, both tangent feet, and the complete radius-r arc fit in
	// one 3r disk. Later clearance and volume proofs charge only these disks.
	centerDistance, err := prismPatternIVSqrt(center.sub(q).dot(center.sub(q)))
	if err != nil || new(big.Rat).Add(centerDistance.Hi, r).Cmp(new(big.Rat).Mul(r, big.NewRat(3, 1))) >= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet escapes its local disk", ErrUnsupported)
	}
	result = prismPatternFilletContact{part: partIndex, station: station,
		tubeRadius: proofbound.RatFloatUp(new(big.Rat).Add(centerDistance.Hi, r)),
		circleIV:   circleFoot}
	result.line, _, err = prismPatternIVHeld(lineFoot, heldRoot.Z)
	if err != nil {
		return prismPatternFilletContact{}, err
	}
	result.circle, _, err = prismPatternIVHeld(circleFoot, heldRoot.Z)
	if err != nil {
		return prismPatternFilletContact{}, err
	}
	result.center, _, err = prismPatternIVHeld(center, heldRoot.Z)
	if err != nil {
		return prismPatternFilletContact{}, err
	}
	_, eLine, _ := prismPatternIVHeld(lineFoot, heldRoot.Z)
	_, eCircle, _ := prismPatternIVHeld(circleFoot, heldRoot.Z)
	_, eCenter, _ := prismPatternIVHeld(center, heldRoot.Z)
	result.pointError = math.Max(eLine, math.Max(eCircle, eCenter))
	return result, nil
}
