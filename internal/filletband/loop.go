// Package filletband holds the closed forms a loop fillet's pipe band is
// measured by (docs/loop-fillet-design.md §5): Table CF's strip
// coefficients, the height integrals J_k and H_k, the strip volume and first
// moment, the patch areas, the length of an LF4 quarter ellipse, and each
// patch's extent along a direction. Every reading is a rational interval over
// the loop's recorded coordinates. π enters through proofbound's bracket and
// a square root through its outward-rounded enclosure, so no reading admits
// anything on a residual.
package filletband

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Interval is a closed rational interval.
type Interval = proofbound.RatInterval

// Vec2 is a plane-local vector whose components are intervals.
type Vec2 [2]Interval

// Point is a plane-local point as the record holds it.
type Point struct{ U, V float64 }

// Walk is one coalesced walk of the filleted loop ℓ in the frame of its face
// F, walked with F's material on its left. A straight walk runs from Start to
// End. A circular walk runs about Center, counter-clockwise when CCW, from
// Start to End; a whole circle (Closed) has Start as its seam and Radius as
// its recorded radius.
type Walk struct {
	Circular, Closed, CCW bool
	Start, End, Center    Point
	Radius                float64
}

// Corner is the class Table LF gives one corner of ℓ.
type Corner int

const (
	// Miter is LF4: two straight walks meeting at a convex corner.
	Miter Corner = iota + 1
	// Tangent is LF5: an exactly tangent (G1) join.
	Tangent
	// Reflex is LF6: a reflex corner of F's region.
	Reflex
	// CurvedMiter is LF8: a sharp convex corner involving a circular walk.
	CurvedMiter
)

// Loop is ℓ as the band reads it: its walks, and Corners[k] the class of the
// corner at Walks[k].Start, between walk k−1 and walk k. A loop of one whole
// circle (LF7) has no corner.
type Loop struct {
	Walks   []Walk
	Corners []Corner
}

// WholeTurn reports whether ℓ is one whole circle (LF7).
func (l Loop) WholeTurn() bool { return len(l.Walks) == 1 && l.Walks[0].Closed }

// ratVec is an exact plane-local vector.
type ratVec [2]*big.Rat

func ratOf(f float64) (*big.Rat, error) {
	q := proofarith.FloatRat(f)
	if q == nil {
		return nil, fmt.Errorf(`%w: a loop fillet's recorded coordinate is not finite`, decaderr.ErrNotFinite)
	}
	return q, nil
}

func ratPoint(p Point) (ratVec, error) {
	u, err := ratOf(p.U)
	if err != nil {
		return ratVec{}, err
	}
	v, err := ratOf(p.V)
	if err != nil {
		return ratVec{}, err
	}
	return ratVec{u, v}, nil
}

func (a ratVec) sub(b ratVec) ratVec {
	return ratVec{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1])}
}

func (a ratVec) neg() ratVec { return ratVec{new(big.Rat).Neg(a[0]), new(big.Rat).Neg(a[1])} }

func (a ratVec) dot(b ratVec) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a[0], b[0]), new(big.Rat).Mul(a[1], b[1]))
}

func (a ratVec) cross(b ratVec) *big.Rat {
	return new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0]))
}

// perp is a turned a quarter turn counter-clockwise.
func (a ratVec) perp() ratVec { return ratVec{new(big.Rat).Neg(a[1]), new(big.Rat).Set(a[0])} }

func point(q *big.Rat) Interval { return proofbound.PointInterval(q) }

func pointInt(n int64) Interval { return point(big.NewRat(n, 1)) }

func scale(a Interval, num, den int64) Interval {
	return proofbound.IntervalScale(a, big.NewRat(num, den))
}

func sqrtOf(q *big.Rat) (Interval, error) {
	iv, ok := proofbound.SqrtInterval(point(q))
	if !ok {
		return Interval{}, fmt.Errorf(`%w: a loop fillet's length has no square root enclosure`, decaderr.ErrUnsupported)
	}
	return iv, nil
}

// unit encloses a/|a| for an exact nonzero vector a.
func unit(a ratVec) (Vec2, error) {
	n2 := a.dot(a)
	if n2.Sign() == 0 {
		return Vec2{}, fmt.Errorf(`%w: a loop fillet's walk has no direction`, decaderr.ErrDegenerate)
	}
	n, err := sqrtOf(n2)
	if err != nil {
		return Vec2{}, err
	}
	var out Vec2
	for i := range out {
		q, ok := proofbound.IntervalQuo(point(a[i]), n)
		if !ok {
			return Vec2{}, fmt.Errorf(`%w: a loop fillet's walk has no direction`, decaderr.ErrDegenerate)
		}
		out[i] = q
	}
	return out, nil
}

func (a Vec2) add(b Vec2) Vec2 {
	return Vec2{proofbound.IntervalAdd(a[0], b[0]), proofbound.IntervalAdd(a[1], b[1])}
}

func (a Vec2) mul(s Interval) Vec2 {
	return Vec2{proofbound.IntervalMul(a[0], s), proofbound.IntervalMul(a[1], s)}
}

func (a Vec2) scale(num, den int64) Vec2 {
	return Vec2{scale(a[0], num, den), scale(a[1], num, den)}
}

func ratVec2(a ratVec) Vec2 { return Vec2{point(a[0]), point(a[1])} }

func zeroVec2() Vec2 { return Vec2{pointInt(0), pointInt(0)} }

// inward is the exact, unnormalized inward normal of w at its start (atEnd
// false) or end: the left normal of the walk's direction, into F's material.
// A straight walk's is its direction turned a quarter turn left; a circular
// walk's points toward its centre when the walk is counter-clockwise and away
// from it otherwise.
func inward(w Walk, atEnd bool) (ratVec, error) {
	start, err := ratPoint(w.Start)
	if err != nil {
		return ratVec{}, err
	}
	end, err := ratPoint(w.End)
	if err != nil {
		return ratVec{}, err
	}
	if !w.Circular {
		return end.sub(start).perp(), nil
	}
	c, err := ratPoint(w.Center)
	if err != nil {
		return ratVec{}, err
	}
	p := start
	if atEnd {
		p = end
	}
	radial := p.sub(c)
	if w.CCW {
		return radial.neg(), nil
	}
	return radial, nil
}

// Turn is the exact sign of the turn the loop takes at the corner where prev
// arrives at cur: +1 a left (convex) turn, −1 a right (reflex) turn, 0 a
// tangent join or a cusp. It is the sign of the cross product of the two
// exact tangents there, which a quarter turn of both leaves unchanged, so it
// is read off the inward normals.
func Turn(prev, cur Walk) (int, error) {
	a, err := inward(prev, true)
	if err != nil {
		return 0, err
	}
	b, err := inward(cur, false)
	if err != nil {
		return 0, err
	}
	return a.cross(b).Sign(), nil
}

// Kappa is cot(θ/2) at an LF4 corner, θ the interior angle on the material
// side where the straight walk prev meets the straight walk cur:
// (|a||b| − a·b)/(a × b) over the two exact directions, one enclosed square
// root (docs/loop-fillet-design.md §5.1).
func Kappa(prev, cur Walk) (Interval, error) {
	if prev.Circular || cur.Circular {
		return Interval{}, fmt.Errorf(`%w: an LF4 corner joins two straight walks`, decaderr.ErrUnsupported)
	}
	a, err := direction(prev)
	if err != nil {
		return Interval{}, err
	}
	b, err := direction(cur)
	if err != nil {
		return Interval{}, err
	}
	cr := a.cross(b)
	if cr.Sign() <= 0 {
		return Interval{}, fmt.Errorf(`%w: an LF4 corner turns left`, decaderr.ErrUnsupported)
	}
	ab, err := sqrtOf(new(big.Rat).Mul(a.dot(a), b.dot(b)))
	if err != nil {
		return Interval{}, err
	}
	num := proofbound.IntervalSub(ab, point(a.dot(b)))
	return proofbound.IntervalScale(num, new(big.Rat).Inv(cr)), nil
}

func direction(w Walk) (ratVec, error) {
	start, err := ratPoint(w.Start)
	if err != nil {
		return ratVec{}, err
	}
	end, err := ratPoint(w.End)
	if err != nil {
		return ratVec{}, err
	}
	return end.sub(start), nil
}
