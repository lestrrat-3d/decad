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
	lineIV        prismPatternIVPoint
	centerIV      prismPatternIVPoint
	circleIV      prismPatternIVPoint
	ringCut       prismPatternFilletRingCut
}

type prismPatternIVPoint struct{ X, Y proofbound.RatInterval }

func prismPatternIVPointOf(x, y *big.Rat) prismPatternIVPoint {
	return prismPatternIVPoint{proofbound.PointInterval(x), proofbound.PointInterval(y)}
}

func (p prismPatternIVPoint) Add(q prismPatternIVPoint) prismPatternIVPoint {
	return prismPatternIVPoint{proofbound.IntervalAdd(p.X, q.X), proofbound.IntervalAdd(p.Y, q.Y)}
}

func (p prismPatternIVPoint) Sub(q prismPatternIVPoint) prismPatternIVPoint {
	return prismPatternIVPoint{proofbound.IntervalSub(p.X, q.X), proofbound.IntervalSub(p.Y, q.Y)}
}

func (p prismPatternIVPoint) Scale(s proofbound.RatInterval) prismPatternIVPoint {
	return prismPatternIVPoint{proofbound.IntervalMul(p.X, s), proofbound.IntervalMul(p.Y, s)}
}

func (p prismPatternIVPoint) Dot(q prismPatternIVPoint) proofbound.RatInterval {
	return proofbound.IntervalAdd(proofbound.IntervalMul(p.X, q.X), proofbound.IntervalMul(p.Y, q.Y))
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
	x, y := read(p.X), read(p.Y)
	point := r3.Vec{X: x, Y: y, Z: z}
	if !proofbound.FiniteVec(point) {
		return r3.Vec{}, 0, fmt.Errorf("%w: joined-prism fillet contact is non-finite", ErrUnsupported)
	}
	errorBound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(p.X, x),
		proofbound.IntervalFloatError(p.Y, y))
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
	rootError := proofbound.AbsSumUpper(proofbound.IntervalFloatError(q.X, heldRoot.X),
		proofbound.IntervalFloatError(q.Y, heldRoot.Y))
	if rootError > part.mesh.Bound().Base() {
		return result, fmt.Errorf("%w: joined-prism fillet root is not tied to its source", ErrUnsupported)
	}
	d := f.Sub(q)
	length, err := prismPatternIVSqrt(d.Dot(d))
	if err != nil || length.Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet connector has no positive length", ErrUnsupported)
	}
	ux, err := prismPatternIVQuo(d.X, length)
	if err != nil {
		return result, err
	}
	uy, err := prismPatternIVQuo(d.Y, length)
	if err != nil {
		return result, err
	}
	u := prismPatternIVPoint{ux, uy}
	n := prismPatternIVPoint{proofbound.IntervalScale(uy, proofarith.FloatRat(-side)),
		proofbound.IntervalScale(ux, proofarith.FloatRat(side))}
	r, R := proofarith.FloatRat(radius), proofarith.FloatRat(blankRadius)
	rIV := proofbound.PointInterval(r)
	RIV := proofbound.PointInterval(R)
	L := q.Add(n.Scale(rIV))
	a := L.Dot(u)
	Rplus := proofbound.IntervalAdd(RIV, rIV)
	disc := proofbound.IntervalSub(proofbound.IntervalSquare(a),
		proofbound.IntervalSub(L.Dot(L), proofbound.IntervalSquare(Rplus)))
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
	center := L.Add(u.Scale(t))
	lineFoot := q.Add(u.Scale(t))
	if lineFoot.Dot(lineFoot).Lo.Cmp(prismPatternRadiusSq(blankRadius)) <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet line foot stays inside the blank", ErrUnsupported)
	}
	step, err := prismPatternIVQuo(RIV, Rplus)
	if err != nil {
		return result, err
	}
	circleFoot := center.Scale(step)
	cross := proofbound.IntervalSub(proofbound.IntervalMul(q.X, circleFoot.Y),
		proofbound.IntervalMul(q.Y, circleFoot.X))
	if side < 0 && cross.Hi.Sign() >= 0 || side > 0 && cross.Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet contact leaves its circular gap", ErrUnsupported)
	}
	// A source corner, both tangent feet, and the complete radius-r arc fit in
	// one 3r disk. Later clearance and volume proofs charge only these disks.
	centerDistance, err := prismPatternIVSqrt(center.Sub(q).Dot(center.Sub(q)))
	if err != nil || new(big.Rat).Add(centerDistance.Hi, r).Cmp(new(big.Rat).Mul(r, big.NewRat(3, 1))) >= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet escapes its local disk", ErrUnsupported)
	}
	result = prismPatternFilletContact{part: partIndex, station: station,
		tubeRadius: proofbound.RatFloatUp(new(big.Rat).Add(centerDistance.Hi, r)),
		lineIV:     lineFoot, centerIV: center, circleIV: circleFoot}
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
