package pointcone

import (
	"errors"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// Errors OffsetMeridian returns. ErrNonFiniteMeridian marks input that is not
// a finite number; ErrUnsupportedMeridian marks a finite input outside the
// class of straight meridians the offset construction handles.
var (
	ErrNonFiniteMeridian   = errors.New("pointcone: meridian input is not finite")
	ErrUnsupportedMeridian = errors.New("pointcone: meridian is not an offsettable straight cone side")
)

// OffsetMeridian returns the float64 apex A and far radius R of a cone whose
// meridian (A,0)-(far,R) lies strictly on side s of the exact line through
// (x0,r0),(x1,r1). It also returns that line's exact slope k and axis X C,
// where the line is x + k*r = C.
//
// The side is the sign of f(x,r) = x + k*r - C. A and R are the float64
// values nearest C and (C-far)/k that lie strictly on that side, so f(A,0)
// and f(far,R) both have sign side and the whole segment lies strictly off
// the line. The result is decided by exact rational arithmetic; no tolerance
// is involved.
//
// The input must be finite, side must be -1 or +1, both radii must be
// positive, x0 != x1, r0 != r1, k > 0 and far must be below both x0 and x1.
// A and R must come out finite with A > far and R > 0.
func OffsetMeridian(x0, r0, x1, r1, far float64, side int) (float64, float64, *big.Rat, *big.Rat, error) {
	for _, v := range [...]float64{x0, r0, x1, r1, far} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return 0, 0, nil, nil, ErrNonFiniteMeridian
		}
	}
	if side != 1 && side != -1 || x0 == x1 || r0 == r1 || r0 <= 0 || r1 <= 0 ||
		far >= x0 || far >= x1 {
		return 0, 0, nil, nil, ErrUnsupportedMeridian
	}
	fx0, fr0 := proofarith.FloatRat(x0), proofarith.FloatRat(r0)
	dx := new(big.Rat).Sub(proofarith.FloatRat(x1), fx0)
	dr := new(big.Rat).Sub(proofarith.FloatRat(r1), fr0)
	k := new(big.Rat).Quo(dx, dr)
	k.Neg(k)
	if k.Sign() <= 0 {
		return 0, 0, nil, nil, ErrUnsupportedMeridian
	}
	c := new(big.Rat).Add(fx0, new(big.Rat).Mul(k, fr0))
	rStar := new(big.Rat).Sub(c, proofarith.FloatRat(far))
	rStar.Quo(rStar, k)
	apex, okApex := strictFloat(c, side)
	radius, okRadius := strictFloat(rStar, side)
	if !okApex || !okRadius || apex <= far || radius <= 0 {
		return 0, 0, nil, nil, ErrUnsupportedMeridian
	}
	return apex, radius, k, c, nil
}

// strictFloat returns the float64 nearest q that is strictly greater than q
// (side +1) or strictly less than q (side -1). When q rounds to infinity, the
// answer is the largest finite float of q's sign if it lies strictly on the
// requested side. It reports false when no finite such float exists.
func strictFloat(q *big.Rat, side int) (float64, bool) {
	f, _ := q.Float64()
	if math.IsNaN(f) {
		return 0, false
	}
	if math.IsInf(f, 0) {
		// q lies beyond the finite range, so only the extreme finite float
		// can be the nearest one strictly on the requested side.
		edge := math.Copysign(math.MaxFloat64, f)
		if exact := proofarith.FloatRat(edge); exact.Cmp(q) == side {
			return edge, true
		}
		return 0, false
	}
	toward := math.Inf(side)
	// Nearest rounding is within one ulp of q, so at most one step is needed;
	// the bound only guards against a malformed input looping.
	for range 4 {
		exact := proofarith.FloatRat(f)
		if exact.Cmp(q) == side {
			return f, true
		}
		f = math.Nextafter(f, toward)
		if math.IsInf(f, 0) {
			return 0, false
		}
	}
	return 0, false
}

// WalkOrientation returns the sign of the exact shoelace area of a closed
// walk: +1 counter-clockwise, -1 clockwise, 0 for zero area, fewer than three
// points or a non-finite coordinate.
func WalkOrientation(points [][2]float64) int {
	if len(points) < 3 {
		return 0
	}
	area := new(big.Rat)
	for i, p := range points {
		q := points[(i+1)%len(points)]
		px, py := proofarith.FloatRat(p[0]), proofarith.FloatRat(p[1])
		qx, qy := proofarith.FloatRat(q[0]), proofarith.FloatRat(q[1])
		if px == nil || py == nil || qx == nil || qy == nil {
			return 0
		}
		area.Add(area, new(big.Rat).Sub(new(big.Rat).Mul(px, qy), new(big.Rat).Mul(qx, py)))
	}
	return area.Sign()
}
