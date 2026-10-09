package filletband

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CurvedMiterPoint encloses a point of the line-circle or circle-circle
// fillet seam in the face's frame. The offset parameter t runs from zero at
// the side level to radius at the cap. Its height is sqrt(2rt-t²). The
// planar components follow the exact intersection of the two offset walls.
// The caller supplies the recorded corner for choosing the continuous root.
func CurvedMiterPoint(prev, cur survey2d.SideWalk, radius, t, cornerU, cornerV float64) ([3]proofbound.RatInterval, bool) {
	r, rt := proofarith.FloatRat(radius), proofarith.FloatRat(t)
	if r == nil || rt == nil || r.Sign() <= 0 || rt.Sign() < 0 || rt.Cmp(r) > 0 {
		return [3]proofbound.RatInterval{}, false
	}
	// The nearest intersection is the continuous corner foot only while
	// the carriers do not fold or exchange roots between zero and t.
	if _, ok := capcontour.MiterLocusSpeedUpper(prev, cur, 0, t, cornerU, cornerV); !ok {
		return [3]proofbound.RatInterval{}, false
	}
	foot, ok := capcontour.LocusFoot(prev, cur, t, cornerU, cornerV)
	if !ok {
		return [3]proofbound.RatInterval{}, false
	}
	heightSq := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(r, rt)),
		new(big.Rat).Mul(rt, rt))
	height, ok := proofbound.SqrtInterval(proofbound.PointInterval(heightSq))
	if !ok {
		return [3]proofbound.RatInterval{}, false
	}
	return [3]proofbound.RatInterval{foot.U, foot.V, height}, true
}

// CurvedMiterLength encloses the seam where a fillet's two pipes meet at a
// non-tangent line-circle or circle-circle corner. The corner foot P(t) is
// the intersection of the two offset carriers. At meridian angle phi, its
// offset is t = r(1-cos(phi)) and its height is r sin(phi). Thus its speed is
// r sqrt(|P'(t)|² sin²(phi) + cos²(phi)). The locus-speed enclosure covers
// every t in [0,r], so half a turn times r max(1, speed) bounds its length.
// Its axial travel r is a lower bound. A false result means the offset locus
// cannot be enclosed without a fold or the radius cannot be read exactly.
func CurvedMiterLength(prev, cur survey2d.SideWalk, radius float64) (proofbound.RatInterval, bool) {
	r := proofarith.FloatRat(radius)
	if r == nil || r.Sign() <= 0 {
		return proofbound.RatInterval{}, false
	}
	speed, ok := capcontour.MiterLocusSpeedUpper(prev, cur, 0, radius, cur.StartU, cur.StartV)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	s := proofarith.FloatRat(speed)
	if s == nil {
		return proofbound.RatInterval{}, false
	}
	if s.Cmp(big.NewRat(1, 1)) < 0 {
		s = big.NewRat(1, 1)
	}
	upper := new(big.Rat).Mul(r, s)
	upper.Mul(upper, proofbound.HalfPiInterval().Hi)
	return proofbound.Interval(r, upper), true
}

// CurvedMiterChordGap bounds the distance from any seam point to the chord
// between the seam's endpoints in one of n equal meridian-angle intervals.
// The speed bound used for CurvedMiterLength is global, and each point is at
// most half an interval from one of its endpoints. This is a conservative
// first-order bound for mesh sampling; endpoint placement errors are separate.
func CurvedMiterChordGap(prev, cur survey2d.SideWalk, radius float64, n int) (*big.Rat, bool) {
	if n <= 0 {
		return nil, false
	}
	length, ok := CurvedMiterLength(prev, cur, radius)
	if !ok {
		return nil, false
	}
	count := new(big.Int).Mul(big.NewInt(int64(n)), big.NewInt(2))
	return new(big.Rat).Quo(length.Hi, new(big.Rat).SetInt(count)), true
}
