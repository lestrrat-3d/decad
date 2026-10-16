package prismpattern

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// FilletContact is the certified contact of one rounded root corner. A
// joined-prism fillet contacts the original connector line and circular
// blank. The exact contact coordinates are algebraic; each held point carries
// an interval error obtained from exact dyadic source coordinates.
type FilletContact struct {
	part, station int
	line, circle  r3.Vec
	center        r3.Vec
	pointError    float64
	tubeRadius    float64
	lineIV        IVPoint
	centerIV      IVPoint
	circleIV      IVPoint
	RingCut       FilletRingCut
}

func exactPlacedPoint(part facetedrecord.PrismPatternPart, point sectionrecord.Point2) IVPoint {
	b := part.Prism.Xform.Basis()
	c, s := proofarith.FloatRat(b.EX.X), proofarith.FloatRat(b.EX.Y)
	u, v := proofarith.FloatRat(point.U), proofarith.FloatRat(point.V)
	x := new(big.Rat).Sub(new(big.Rat).Mul(c, u), new(big.Rat).Mul(s, v))
	y := new(big.Rat).Add(new(big.Rat).Mul(s, u), new(big.Rat).Mul(c, v))
	return IVPointOf(x, y)
}

// FilletContactOf brackets the exact connector/circle contact at one root
// station of a copied part, or returns ErrUnsupported.
func FilletContactOf(part facetedrecord.PrismPatternPart, partIndex, station int,
	root sectionrecord.ArcSeg, rootIndex int, blankRadius, radius float64) (FilletContact, error) {
	var result FilletContact
	if radius <= 0 || proofbound.IsNonFinite(radius) ||
		station != 0 && station != part.RootStart {
		return result, fmt.Errorf("%w: joined-prism fillet has no root corner or positive radius", decaderr.ErrUnsupported)
	}
	lineIndex, side := 0, -1.0
	if station == part.RootStart {
		lineIndex, side = rootIndex-1, 1
	}
	if lineIndex < 0 || lineIndex >= len(part.Prism.Profile.Outer.Segments) {
		return result, fmt.Errorf("%w: joined-prism fillet lost its connector", decaderr.ErrUnsupported)
	}
	connector, ok := part.Prism.Profile.Outer.Segments[lineIndex].(sectionrecord.LineSeg)
	if !ok {
		return result, fmt.Errorf("%w: joined-prism fillet connector is not straight", decaderr.ErrUnsupported)
	}
	near, far := connector.Start, connector.End
	if near != root.Start && near != root.End {
		near, far = far, near
	}
	if near != root.Start && near != root.End {
		return result, fmt.Errorf("%w: joined-prism fillet connector misses its root", decaderr.ErrUnsupported)
	}
	q := exactPlacedPoint(part, near)
	f := exactPlacedPoint(part, far)
	heldRoot := part.Mesh.Vertices[2*station]
	rootError := proofbound.AbsSumUpper(proofbound.IntervalFloatError(q.X, heldRoot.X),
		proofbound.IntervalFloatError(q.Y, heldRoot.Y))
	if rootError > units.Millimeters(part.Mesh.Bound).Base() {
		return result, fmt.Errorf("%w: joined-prism fillet root is not tied to its source", decaderr.ErrUnsupported)
	}
	d := f.Sub(q)
	length, err := IVSqrt(d.Dot(d))
	if err != nil || length.Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet connector has no positive length", decaderr.ErrUnsupported)
	}
	ux, err := IVQuo(d.X, length)
	if err != nil {
		return result, err
	}
	uy, err := IVQuo(d.Y, length)
	if err != nil {
		return result, err
	}
	u := IVPoint{X: ux, Y: uy}
	n := IVPoint{X: proofbound.IntervalScale(uy, proofarith.FloatRat(-side)),
		Y: proofbound.IntervalScale(ux, proofarith.FloatRat(side))}
	r, R := proofarith.FloatRat(radius), proofarith.FloatRat(blankRadius)
	rIV := proofbound.PointInterval(r)
	RIV := proofbound.PointInterval(R)
	L := q.Add(n.Scale(rIV))
	a := L.Dot(u)
	Rplus := proofbound.IntervalAdd(RIV, rIV)
	disc := proofbound.IntervalSub(proofbound.IntervalSquare(a),
		proofbound.IntervalSub(L.Dot(L), proofbound.IntervalSquare(Rplus)))
	if disc.Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet offset has no distinct outer contact", decaderr.ErrUnsupported)
	}
	squareRoot, err := IVSqrt(disc)
	if err != nil {
		return result, err
	}
	t := proofbound.IntervalAdd(proofbound.IntervalNeg(a), squareRoot)
	if t.Lo.Sign() <= 0 || proofbound.IntervalSub(length, t).Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet contact escapes its connector", decaderr.ErrUnsupported)
	}
	center := L.Add(u.Scale(t))
	lineFoot := q.Add(u.Scale(t))
	if lineFoot.Dot(lineFoot).Lo.Cmp(RadiusSq(blankRadius)) <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet line foot stays inside the blank", decaderr.ErrUnsupported)
	}
	step, err := IVQuo(RIV, Rplus)
	if err != nil {
		return result, err
	}
	circleFoot := center.Scale(step)
	cross := proofbound.IntervalSub(proofbound.IntervalMul(q.X, circleFoot.Y),
		proofbound.IntervalMul(q.Y, circleFoot.X))
	if side < 0 && cross.Hi.Sign() >= 0 || side > 0 && cross.Lo.Sign() <= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet contact leaves its circular gap", decaderr.ErrUnsupported)
	}
	// A source corner, both tangent feet, and the complete radius-r arc fit in
	// one 3r disk. Later clearance and volume proofs charge only these disks.
	centerDistance, err := IVSqrt(center.Sub(q).Dot(center.Sub(q)))
	if err != nil || new(big.Rat).Add(centerDistance.Hi, r).Cmp(new(big.Rat).Mul(r, big.NewRat(3, 1))) >= 0 {
		return result, fmt.Errorf("%w: joined-prism fillet escapes its local disk", decaderr.ErrUnsupported)
	}
	result = FilletContact{part: partIndex, station: station,
		tubeRadius: proofbound.RatFloatUp(new(big.Rat).Add(centerDistance.Hi, r)),
		lineIV:     lineFoot, centerIV: center, circleIV: circleFoot}
	result.line, _, err = IVHeld(lineFoot, heldRoot.Z)
	if err != nil {
		return FilletContact{}, err
	}
	result.circle, _, err = IVHeld(circleFoot, heldRoot.Z)
	if err != nil {
		return FilletContact{}, err
	}
	result.center, _, err = IVHeld(center, heldRoot.Z)
	if err != nil {
		return FilletContact{}, err
	}
	_, eLine, _ := IVHeld(lineFoot, heldRoot.Z)
	_, eCircle, _ := IVHeld(circleFoot, heldRoot.Z)
	_, eCenter, _ := IVHeld(center, heldRoot.Z)
	result.pointError = math.Max(eLine, math.Max(eCircle, eCenter))
	return result, nil
}
