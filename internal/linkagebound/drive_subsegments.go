package linkagebound

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// DriverSubsegment is one stretch of a driven loop's schedule whose value
// keeps one sign. A mixed-unit zero crossing adds a straddle between cuts.
type DriverSubsegment struct {
	Idx      int
	Lo, Hi   *big.Rat
	Near     *big.Rat
	NearZero bool
	Held     bool
	Side     int
	Straddle bool
}

// DriverSubsegments cuts a loop driver's schedule into sub-segments
// (docs/linkage-check-design.md §15.8): each segment once, or twice at the
// fraction where its driver value crosses 0. That fraction is exact when both
// of the segment's waypoints are whole turns, or both radians or lengths.
// Between waypoints stated in mixed terms it depends on π, and the segment is
// cut three times instead: up to a rational cut below the crossing, the
// straddle holding it, and on from a rational cut above it (crossingCuts).
// A sub-segment holding the driver at 0 is read on any side a moving one
// uses.
func DriverSubsegments(points []motionbound.MotionParam, values []units.Value, linkIndex int) ([]DriverSubsegment, error) {
	n := len(points) - 1
	signs := make([]int, len(values))
	for w, v := range values {
		c, ok := motionbound.ParamCompare(v, units.New(0, v.Unit()))
		if !ok {
			return nil, fmt.Errorf(`%w: the sign of link %d's waypoint %s cannot be decided`, decaderr.ErrUnsupported, linkIndex, v)
		}
		signs[w] = c
	}
	side := func(sign int) int {
		if sign < 0 {
			return 1
		}
		return 0
	}
	var subs []DriverSubsegment
	add := func(sub DriverSubsegment) {
		sub.Idx = len(subs)
		subs = append(subs, sub)
	}
	for j := range n {
		a, b := big.NewRat(int64(j), int64(n)), big.NewRat(int64(j+1), int64(n))
		pa, pb := points[j], points[j+1]
		sa, sb := signs[j], signs[j+1]
		switch {
		case pa.Turn.Cmp(pb.Turn) == 0 && pa.Base.Cmp(pb.Base) == 0:
			add(DriverSubsegment{Lo: a, Hi: b, Near: a, NearZero: sa == 0, Held: true, Side: side(sa)})
		case sa*sb < 0:
			var num, den *big.Rat
			switch {
			case pa.Turn.Sign() == 0 && pb.Turn.Sign() == 0:
				num, den = pa.Base, new(big.Rat).Sub(pa.Base, pb.Base)
			case pa.Base.Sign() == 0 && pb.Base.Sign() == 0:
				num, den = pa.Turn, new(big.Rat).Sub(pa.Turn, pb.Turn)
			default:
				lo, hi, ok := crossingCuts(pa, pb, a, b, sa)
				if !ok {
					return nil, fmt.Errorf(`%w: link %d's driver crosses 0 between waypoints %s and %s stated in mixed terms, where the crossing cannot be bracketed`,
						decaderr.ErrUnsupported, linkIndex, values[j], values[j+1])
				}
				add(DriverSubsegment{Lo: a, Hi: lo, Near: lo, Side: side(sa)})
				add(DriverSubsegment{Lo: new(big.Rat).Set(lo), Hi: hi, Near: new(big.Rat).Set(lo), Side: side(sa), Straddle: true})
				add(DriverSubsegment{Lo: new(big.Rat).Set(hi), Hi: b, Near: new(big.Rat).Set(hi), Side: side(sb)})
				continue
			}
			t := new(big.Rat).Quo(num, den)
			s0 := new(big.Rat).Sub(b, a)
			s0.Mul(s0, t).Add(s0, a)
			add(DriverSubsegment{Lo: a, Hi: s0, Near: s0, NearZero: true, Side: side(sa)})
			add(DriverSubsegment{Lo: new(big.Rat).Set(s0), Hi: b, Near: new(big.Rat).Set(s0), NearZero: true, Side: side(sb)})
		default:
			sign := sa + sb
			sub := DriverSubsegment{Lo: a, Hi: b, Near: a, Side: side(sign)}
			// The near end has the smaller |q|: the smaller value of a
			// positive stretch, the larger of a negative one.
			ends, ok := motionbound.ParamCompare(values[j], values[j+1])
			if !ok {
				return nil, fmt.Errorf(`%w: link %d's waypoints %s and %s cannot be ordered`, decaderr.ErrUnsupported, linkIndex, values[j], values[j+1])
			}
			if ends*sign > 0 {
				sub.Near = b
			}
			sub.NearZero = (sub.Near == a && sa == 0) || (sub.Near == b && sb == 0)
			add(sub)
		}
	}
	// A stretch that holds 0 needs a zero pose, which every side's E0 is;
	// it takes a side some moving stretch uses.
	used := -1
	for _, sub := range subs {
		if !sub.Held || !sub.NearZero {
			used = sub.Side
			break
		}
	}
	for n := range subs {
		if subs[n].Held && subs[n].NearZero && used >= 0 {
			subs[n].Side = used
		}
	}
	return subs, nil
}

// crossingCuts brackets the irrational fraction where a driver crosses 0
// between waypoints pa at a and pb at b stated in mixed terms: two rationals
// lo < hi inside (a, b), the driver's value at lo proven to have pa's sign sa
// and at hi pb's, for every π in its enclosure. The crossing's local
// fraction q_a/(q_a − q_b) is read at both ends of π's enclosure, the two
// readings widened outward by their gap, and the signs then checked exactly;
// ok is false when a check fails.
func crossingCuts(pa, pb motionbound.MotionParam, a, b *big.Rat, sa int) (*big.Rat, *big.Rat, bool) {
	twoPi := proofbound.TwoPiInterval()
	var ts []*big.Rat
	for _, tp := range []*big.Rat{twoPi.Lo, twoPi.Hi} {
		qa := new(big.Rat).Mul(pa.Turn, tp)
		qa.Add(qa, pa.Base)
		qb := new(big.Rat).Mul(pb.Turn, tp)
		qb.Add(qb, pb.Base)
		den := new(big.Rat).Sub(qa, qb)
		if den.Sign() == 0 {
			return nil, nil, false
		}
		ts = append(ts, new(big.Rat).Quo(qa, den))
	}
	tlo, thi := ts[0], ts[1]
	if tlo.Cmp(thi) > 0 {
		tlo, thi = thi, tlo
	}
	gap := new(big.Rat).Sub(thi, tlo)
	tlo = new(big.Rat).Sub(tlo, gap)
	thi = new(big.Rat).Add(thi, gap)
	at := func(t *big.Rat) *big.Rat {
		out := new(big.Rat).Sub(b, a)
		return out.Add(out.Mul(out, t), a)
	}
	lo, hi := at(tlo), at(thi)
	if lo.Cmp(a) <= 0 || hi.Cmp(b) >= 0 || lo.Cmp(hi) >= 0 {
		return nil, nil, false
	}
	signed := func(t *big.Rat, want int) bool {
		q := pa.Lerp(pb, t)
		if want < 0 {
			return motionbound.ParamUpper(q).Sign() < 0
		}
		return motionbound.ParamLower(q).Sign() > 0
	}
	if !signed(tlo, sa) || !signed(thi, -sa) {
		return nil, nil, false
	}
	return lo, hi, true
}
