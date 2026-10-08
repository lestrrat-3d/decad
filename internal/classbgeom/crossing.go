package classbgeom

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// CrossingOffsetUpper is a proven upper bound on the distance from a keyed
// vertex to the exact crossing it names (docs/general-boolean-design.md §5,
// §10): the crossing of a cylinder, whose section is seg, with a plane along
// its axis. Read in the plane across the axis, the vertex sits on the plane's
// trace at its exact level along, at the free coordinate free; the crossing
// on the vertex's side of the centre sits at
// centerFree ± √(R² − (along − centerAlong)²) on that trace. The vertex is
// that far from the crossing:
//
//	| |free − centerFree| − √b |  =  |a² − b| / (a + √b),
//
// with a = |free − centerFree| and b = R² − (along − centerAlong)², both
// exact rationals over the recorded floats, and √b replaced by its value
// rounded down (proofbound.RatSqrtDown), which only enlarges the quotient. R²
// is a circle's recorded radius squared, or an arc's squared distance from
// its centre to its Start, the radius it denotes.
//
// The scene that cut the trace placed the vertex through a recorded cut
// parameter, and that parameter is no closer to the crossing than the float
// arithmetic of the coordinates allows, so far from the plane origin the
// vertex sits about ulp(|free|) away whatever the trace's length. Every face
// pinned at the vertex inherits that distance, so it is the vertex's
// displacement. A trace that misses the cylinder (b < 0) or a bound that
// cannot be stated answers +Inf, which the caller refuses on.
func CrossingOffsetUpper(seg sectionrecord.CurveSegment, free, centerFree, along, centerAlong float64) float64 {
	r2, ok := radiusSquared(seg)
	if !ok {
		return math.Inf(1)
	}
	a := exactDelta(free, centerFree)
	h := exactDelta(along, centerAlong)
	if a == nil || h == nil {
		return math.Inf(1)
	}
	a.Abs(a)
	b := new(big.Rat).Sub(r2, new(big.Rat).Mul(h, h))
	if b.Sign() < 0 {
		return math.Inf(1)
	}
	num := new(big.Rat).Sub(new(big.Rat).Mul(a, a), b)
	if num.Sign() == 0 {
		return 0
	}
	num.Abs(num)
	root := proofarith.FloatRat(proofbound.RatSqrtDown(b))
	if root == nil {
		return math.Inf(1)
	}
	den := new(big.Rat).Add(a, root)
	if den.Sign() <= 0 {
		return math.Inf(1)
	}
	up := proofbound.RatFloatUp(num.Quo(num, den))
	if proofbound.IsNonFinite(up) {
		return math.Inf(1)
	}
	return up
}

// radiusSquared is the exact squared radius a circular section segment
// denotes.
func radiusSquared(seg sectionrecord.CurveSegment) (*big.Rat, bool) {
	switch s := seg.(type) {
	case sectionrecord.CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		rr := proofarith.FloatRat(r)
		if rr == nil {
			return nil, false
		}
		return rr.Mul(rr, rr), true
	case sectionrecord.ArcSeg:
		du := exactDelta(s.Start.U, s.Center.U)
		dv := exactDelta(s.Start.V, s.Center.V)
		if du == nil || dv == nil {
			return nil, false
		}
		return du.Add(du.Mul(du, du), new(big.Rat).Mul(dv, dv)), true
	default:
		return nil, false
	}
}

// exactDelta is a − b over exact rationals, or nil where either is not
// finite.
func exactDelta(a, b float64) *big.Rat {
	ra, rb := proofarith.FloatRat(a), proofarith.FloatRat(b)
	if ra == nil || rb == nil {
		return nil
	}
	return ra.Sub(ra, rb)
}
