package filletband

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CurvedMiterMassAllowance bounds the difference between the strip assembled
// from the two untrimmed wall pieces and the strip whose carriers meet at the
// curved miter. Every changed boundary segment has endpoints at an offset
// normal foot or the miter foot. The latter travels at most speed*r from the
// recorded corner; straight connectors stay in that disk. A circular
// connector stays there too when the speed bound rules out a half-turn.
// Otherwise its entire offset circle is at most 2R+r from the corner,
// plus the recorded endpoint's stated error. Thus the
// section difference is contained in the disk of radius reach at every
// height. Its volume has absolute value at most pi*reach²*r, and each
// in-plane first moment at most that volume times |corner coordinate|+reach.
// The bound allows either sign, so the caller widens its polynomial strip
// readings rather than assuming overlap or gap.
func CurvedMiterMassAllowance(prev, cur survey2d.SideWalk, radius proofbound.RatInterval) (
	volume, momentU, momentV *big.Rat, ok bool,
) {
	reach, ok := CurvedMiterReachUpper(prev, cur, radius)
	if !ok {
		return nil, nil, nil, false
	}
	d := proofarith.FloatRat(reach)
	cu, cv := proofarith.FloatRat(cur.StartU), proofarith.FloatRat(cur.StartV)
	if d == nil || cu == nil || cv == nil {
		return nil, nil, nil, false
	}
	volume = new(big.Rat).Mul(proofbound.PiUpper, new(big.Rat).Mul(d, d))
	volume.Mul(volume, radius.Hi)
	u := new(big.Rat).Add(new(big.Rat).Abs(cu), d)
	v := new(big.Rat).Add(new(big.Rat).Abs(cv), d)
	return volume, new(big.Rat).Mul(volume, u), new(big.Rat).Mul(volume, v), true
}

// CurvedMiterReachUpper encloses every changed part of the two wall patches
// in a planar disk about their shared recorded corner. It includes both the
// miter foot's travel and any circular connector between that foot and the
// original normal foot.
func CurvedMiterReachUpper(prev, cur survey2d.SideWalk, radius proofbound.RatInterval) (float64, bool) {
	if radius.Lo.Sign() <= 0 {
		return 0, false
	}
	r := proofbound.RatFloatUp(radius.Hi)
	if proofbound.IsNonFinite(r) {
		return 0, false
	}
	speed, ok := capcontour.MiterLocusSpeedUpper(prev, cur, 0, r, cur.StartU, cur.StartV)
	if !ok {
		return 0, false
	}
	cornerError := math.Max(proofbound.WalkEndBoundAllow(prev.EndBound),
		proofbound.WalkEndBoundAllow(cur.StartBound))
	reach := proofbound.AbsSumUpper(proofbound.ProductUpper(r, proofbound.AbsSumUpper(speed, 1)), cornerError)
	if proofbound.IsNonFinite(reach) {
		return 0, false
	}
	for i, wall := range []survey2d.SideWalk{prev, cur} {
		if !wall.IsCircular() {
			continue
		}
		rad := proofbound.AbsSumUpper(math.Abs(wall.Radius), wall.RadiusBound)
		end := proofbound.WalkEndBoundAllow(wall.StartBound)
		if i == 0 {
			end = proofbound.WalkEndBoundAllow(wall.EndBound)
		}
		// The connected trim from the original radial foot to the miter foot
		// can cross a half-turn only by passing the antipode. If even the
		// antipode's nearest possible position lies beyond the foot's proven
		// travel, every point of that trim stays no farther from the corner
		// than its two endpoints. Otherwise the entire offset circle is the
		// conservative enclosure.
		if !curvedMiterShortArc(wall, radius, reach, end) {
			reach = math.Max(reach, proofbound.AbsSumUpper(proofbound.ProductUpper(2, rad), r, end))
		}
	}
	if proofbound.IsNonFinite(reach) {
		return 0, false
	}
	return reach, true
}

// curvedMiterShortArc certifies that the continuously trimmed offset circle
// never reaches the antipode of its original radial foot. A path whose foot
// stays strictly closer than the antipode cannot pass a half turn.
func curvedMiterShortArc(w survey2d.SideWalk, radius proofbound.RatInterval, reach, endError float64) bool {
	R, Rbound := proofarith.FloatRat(w.Radius), proofarith.FloatRat(w.RadiusBound)
	E, reachRat := proofarith.FloatRat(endError), proofarith.FloatRat(reach)
	if R == nil || Rbound == nil || E == nil || reachRat == nil {
		return false
	}
	antipode := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Sub(R, Rbound))
	antipode.Sub(antipode, radius.Hi)
	antipode.Sub(antipode, E)
	return reachRat.Cmp(antipode) < 0
}

// CurvedMiterPatchAreaAllowance bounds the area change on one pipe patch at
// a sharp circular corner. A straight patch's generator can move along its
// axis by no more than speed*r; integrating its quarter-circle area element
// gives the first branch. A circular patch is bounded by one complete turn
// of its torus, with radial reach at most R+2r. A certified short arc uses
// the chord-to-arc ratio pi/2 instead. Both its original angular interval
// and the trimmed interval occupy at most one turn, so the full-turn fallback
// bounds their area difference regardless of which interval is larger.
func CurvedMiterPatchAreaAllowance(piece Piece, affected, prev, cur survey2d.SideWalk, atStart bool,
	radius proofbound.RatInterval) (*big.Rat, bool) {
	if radius.Lo.Sign() <= 0 {
		return nil, false
	}
	r := proofbound.RatFloatUp(radius.Hi)
	if proofbound.IsNonFinite(r) {
		return nil, false
	}
	speed, ok := capcontour.MiterLocusSpeedUpper(prev, cur, 0, r, cur.StartU, cur.StartV)
	if !ok {
		return nil, false
	}
	if piece.Kind == Cylinder {
		out := new(big.Rat).Mul(radius.Hi, radius.Hi)
		out.Mul(out, proofarith.FloatRat(speed))
		out.Mul(out, proofbound.HalfPiInterval().Hi)
		return out, true
	}
	if piece.Kind != InnerTorus && piece.Kind != OuterTorus {
		return nil, false
	}
	if !affected.IsCircular() {
		return nil, false
	}
	endError := proofbound.WalkEndBoundAllow(affected.EndBound)
	if atStart {
		endError = proofbound.WalkEndBoundAllow(affected.StartBound)
	}
	reach := proofbound.AbsSumUpper(proofbound.ProductUpper(r, proofbound.AbsSumUpper(speed, 1)), endError)
	if curvedMiterShortArc(affected, radius, reach, endError) {
		// On a minor circular arc, arc/chord <= pi/2. The chord from the
		// original normal foot to the miter foot is at most (speed+1)*r.
		out := new(big.Rat).Mul(radius.Hi, radius.Hi)
		out.Mul(out, new(big.Rat).Add(proofarith.FloatRat(speed), big.NewRat(1, 1)))
		out.Mul(out, proofbound.HalfPiInterval().Hi)
		out.Mul(out, proofbound.HalfPiInterval().Hi)
		return out, true
	}
	R := proofbound.AbsSumUpper(math.Abs(affected.Radius), affected.RadiusBound)
	if proofbound.IsNonFinite(R) {
		return nil, false
	}
	radial := new(big.Rat).Add(proofarith.FloatRat(R), new(big.Rat).Mul(radius.Hi, big.NewRat(2, 1)))
	out := new(big.Rat).Mul(radius.Hi, radial)
	out.Mul(out, proofbound.HalfPiInterval().Hi)
	out.Mul(out, proofbound.TwoPiInterval().Hi)
	return out, true
}

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
