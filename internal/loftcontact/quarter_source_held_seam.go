package loftcontact

import (
	"context"
	"errors"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// QuarterSourceHeldY1 bounds the selected held y=1 chain against the source
// quarter arc. It proves no other seam, output shell, or Boolean result.
type QuarterSourceHeldY1 struct {
	Reason string
	Bound  *big.Rat
	Pieces int
}

func quarterQuadraticValue(m loftHullMath, a, b, c, z *big.Rat) (*big.Rat, error) {
	az, err := m.mul(a, z)
	if err != nil {
		return nil, err
	}
	az, err = m.add(az, b)
	if err != nil {
		return nil, err
	}
	az, err = m.mul(az, z)
	if err != nil {
		return nil, err
	}
	return m.add(az, c)
}

// The minimum of a quadratic on a closed interval is at an endpoint or,
// when its leading coefficient is positive, at its stationary point.
func quarterQuadraticNonnegative(m loftHullMath, a, b, c, lo, hi *big.Rat) (bool, error) {
	for _, z := range []*big.Rat{lo, hi} {
		v, err := quarterQuadraticValue(m, a, b, c, z)
		if err != nil {
			return false, err
		}
		sign, err := m.sign(v)
		if err != nil || sign < 0 {
			return false, err
		}
	}
	positive, err := m.sign(a)
	if err != nil || positive <= 0 {
		return positive <= 0, err
	}
	twice, err := m.mul(big.NewRat(2, 1), a)
	if err != nil {
		return false, err
	}
	negative, err := m.sub(big.NewRat(0, 1), b)
	if err != nil {
		return false, err
	}
	stationary, err := m.quotient(negative, twice)
	if err != nil {
		return false, err
	}
	aboveLo, err := m.cmp(stationary, lo)
	if err != nil {
		return false, err
	}
	belowHi, err := m.cmp(stationary, hi)
	if err != nil || aboveLo <= 0 || belowHi >= 0 {
		return belowHi >= 0 || aboveLo <= 0, err
	}
	v, err := quarterQuadraticValue(m, a, b, c, stationary)
	if err != nil {
		return false, err
	}
	sign, err := m.sign(v)
	return sign >= 0, err
}

func quarterSourceHeldPiece(ctx context.Context, m loftHullMath,
	piece heldPiece, bound *big.Rat) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	zero, ten := big.NewRat(0, 1), big.NewRat(10, 1)
	for _, p := range []heldPoint{piece.lo, piece.hi} {
		if p.x == nil || p.y == nil || p.z == nil {
			return false, nil
		}
		if err := m.step(); err != nil {
			return false, err
		}
		if p.y.Cmp(big.NewRat(1, 1)) != 0 || p.z.Cmp(zero) < 0 || p.z.Cmp(ten) > 0 {
			return false, nil
		}
		lower, err := m.sub(p.x, bound)
		if err != nil {
			return false, err
		}
		upper, err := m.add(p.x, bound)
		if err != nil {
			return false, err
		}
		left, err := m.cmp(lower, big.NewRat(2, 1))
		if err != nil || left <= 0 {
			return false, err
		}
		right, err := m.cmp(upper, big.NewRat(6, 1))
		if err != nil || right >= 0 {
			return false, err
		}
	}
	dz, err := m.sub(piece.hi.z, piece.lo.z)
	if err != nil {
		return false, err
	}
	if dz.Sign() <= 0 {
		return false, nil
	}
	dx, err := m.sub(piece.hi.x, piece.lo.x)
	if err != nil {
		return false, err
	}
	slope, err := m.quotient(dx, dz)
	if err != nil {
		return false, err
	}
	part, err := m.mul(slope, piece.lo.z)
	if err != nil {
		return false, err
	}
	intercept, err := m.sub(piece.lo.x, part)
	if err != nil {
		return false, err
	}
	slopeSquare, err := m.mul(slope, slope)
	if err != nil {
		return false, err
	}
	for _, upper := range []bool{false, true} {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		var shifted *big.Rat
		if upper {
			shifted, err = m.add(intercept, bound)
		} else {
			shifted, err = m.sub(intercept, bound)
		}
		if err != nil {
			return false, err
		}
		cross, err := m.mul(slope, shifted)
		if err != nil {
			return false, err
		}
		cross, err = m.mul(big.NewRat(2, 1), cross)
		if err != nil {
			return false, err
		}
		constant, err := m.mul(shifted, shifted)
		if err != nil {
			return false, err
		}
		var a, b, c *big.Rat
		if upper {
			a, err = m.sub(slopeSquare, big.NewRat(1, 400))
			if err == nil {
				b, err = m.sub(cross, big.NewRat(1, 2))
			}
			if err == nil {
				c, err = m.sub(constant, big.NewRat(24, 1))
			}
		} else {
			a, err = m.sub(big.NewRat(1, 400), slopeSquare)
			if err == nil {
				b, err = m.sub(big.NewRat(1, 2), cross)
			}
			if err == nil {
				c, err = m.sub(big.NewRat(24, 1), constant)
			}
		}
		if err != nil {
			return false, err
		}
		ok, err := quarterQuadraticNonnegative(m, a, b, c, piece.lo.z, piece.hi.z)
		if err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}

// CertifyQuarterSourceHeldY1 rereads the live original pair, then compares
// every selected y=1 held piece with the exact source radius at each height.
// The caller must separately authenticate the source cap junction witnesses.
func CertifyQuarterSourceHeldY1(ctx context.Context, budget *proofbound.WorkBudget,
	prism, loft *topo.Body, arcSegment int, radii [2]*big.Rat) (QuarterSourceHeldY1, bool, error) {
	report := QuarterSourceHeldY1{}
	decline := func(reason string) (QuarterSourceHeldY1, bool, error) {
		report.Reason = reason
		return report, false, nil
	}
	if err := ctx.Err(); err != nil {
		return report, false, err
	}
	if budget == nil || !loftHullFits(radii[0]) || !loftHullFits(radii[1]) ||
		radii[0].Cmp(big.NewRat(5, 1)) != 0 || radii[1].Cmp(big.NewRat(11, 2)) != 0 {
		return decline("source_radius")
	}
	m := loftHullMath{budget: budget}
	selected, accepted, err := CertifyQuarterHeldPair(ctx, budget, prism, loft)
	if err != nil || !accepted {
		if err == nil {
			return decline("held_pair." + selected.Reason)
		}
		return report, false, err
	}
	actualArc, ok, err := quarterPairArc(m, loft)
	if err != nil || !ok || actualArc != arcSegment {
		return report, false, err
	}
	pl, payloadOK := loft.Payload.(loftrecord.Payload)
	if !payloadOK {
		return decline("source_payload")
	}
	for capIndex, radius := range radii {
		if err := m.step(); err != nil {
			return report, false, err
		}
		loop := pl.Profile0.Outer
		if capIndex == 1 {
			loop = pl.Profile1.Outer
		}
		arc, isArc := loop.Segments[arcSegment].(sectionrecord.ArcSeg)
		if !isArc {
			return decline("source_arc")
		}
		lift, valid := proofbound.RatOf(arc.Start.U)
		if !valid || lift.Cmp(radius) != 0 {
			return decline("source_radius")
		}
	}
	bound := big.NewRat(1, 4)
	pieces := selected.Y1.pieces
	if len(pieces) == 0 || len(pieces) > 512 ||
		selected.Y1.BottomX == nil || selected.Y1.TopX == nil {
		return decline("held_chain")
	}
	bottom, err := m.cmp(selected.Y1.BottomX, pieces[0].lo.x)
	if err != nil {
		return report, false, err
	}
	top, err := m.cmp(selected.Y1.TopX, pieces[len(pieces)-1].hi.x)
	if err != nil || bottom != 0 || top != 0 {
		return report, false, err
	}
	for _, piece := range pieces {
		ok, checkErr := quarterSourceHeldPiece(ctx, m, piece, bound)
		if checkErr != nil || !ok {
			if errors.Is(checkErr, errLoftHullSize) {
				return decline("arithmetic_size")
			}
			if checkErr != nil {
				return report, false, checkErr
			}
			return decline("source_held_bound")
		}
	}
	if err := budget.Err(); err != nil {
		return report, false, err
	}
	if err := ctx.Err(); err != nil {
		return report, false, err
	}
	return QuarterSourceHeldY1{Bound: bound, Pieces: len(pieces)}, true, nil
}
