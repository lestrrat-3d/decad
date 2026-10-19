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

// QuarterSourceHeldSeam bounds one selected held chain against the source
// quarter arc. It proves no output shell or Boolean result.
type QuarterSourceHeldSeam struct {
	Reason string
	Bound  *big.Rat
	Pieces int
}

// QuarterSourceHeldY1 names the selected lower-wall result.
type QuarterSourceHeldY1 = QuarterSourceHeldSeam

// QuarterSourceHeldY3 names the selected upper-wall result.
type QuarterSourceHeldY3 = QuarterSourceHeldSeam

// QuarterSourceHeldPair certifies the two selected source-to-held seam bands.
// Accepted pairs have strictly separated bands at every common height.
type QuarterSourceHeldPair struct {
	Reason string
	Y1, Y3 QuarterSourceHeldSeam
	Slabs  int
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
	piece heldPiece, carrier, sourceConstant, bound *big.Rat) (bool, error) {
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
		if p.y.Cmp(carrier) != 0 || p.z.Cmp(zero) < 0 || p.z.Cmp(ten) > 0 {
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
				c, err = m.sub(constant, sourceConstant)
			}
		} else {
			a, err = m.sub(big.NewRat(1, 400), slopeSquare)
			if err == nil {
				b, err = m.sub(big.NewRat(1, 2), cross)
			}
			if err == nil {
				c, err = m.sub(sourceConstant, constant)
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

func quarterSourceHeldSeparation(ctx context.Context, m loftHullMath,
	y1, y3 []heldPiece, bound *big.Rat) (int, bool, error) {
	if len(y1) == 0 || len(y3) == 0 || len(y1) > 512 || len(y3) > 512 {
		return 0, false, nil
	}
	z, ten := big.NewRat(0, 1), big.NewRat(10, 1)
	i, j, slabs := 0, 0, 0
	for i < len(y1) && j < len(y3) {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		if err := m.step(); err != nil {
			return 0, false, err
		}
		outer, inner := y1[i], y3[j]
		for _, piece := range []heldPiece{outer, inner} {
			lo, err := m.cmp(piece.lo.z, z)
			if err != nil || lo > 0 {
				return 0, false, err
			}
		}
		end := outer.hi.z
		cmp, err := m.cmp(inner.hi.z, end)
		if err != nil {
			return 0, false, err
		}
		if cmp < 0 {
			end = inner.hi.z
		}
		cmp, err = m.cmp(z, end)
		if err != nil || cmp >= 0 {
			return 0, false, err
		}
		for _, height := range []*big.Rat{z, end} {
			outerX, err := quarterHeldAtZ(m, outer, height)
			if err != nil {
				return 0, false, err
			}
			innerX, err := quarterHeldAtZ(m, inner, height)
			if err != nil {
				return 0, false, err
			}
			upper, err := m.add(innerX, bound)
			if err != nil {
				return 0, false, err
			}
			lower, err := m.sub(outerX, bound)
			if err != nil {
				return 0, false, err
			}
			cmp, err := m.cmp(upper, lower)
			if err != nil || cmp >= 0 {
				return 0, false, err
			}
		}
		slabs++
		z = end
		cmp, err = m.cmp(outer.hi.z, end)
		if err != nil {
			return 0, false, err
		}
		if cmp == 0 {
			i++
		}
		cmp, err = m.cmp(inner.hi.z, end)
		if err != nil {
			return 0, false, err
		}
		if cmp == 0 {
			j++
		}
	}
	cmp, err := m.cmp(z, ten)
	return slabs, err == nil && cmp == 0 && i == len(y1) && j == len(y3), err
}

// CertifyQuarterSourceHeldSelected rereads one original pair and checks one
// or both selected seams. The caller authenticates both source cap witnesses.
func CertifyQuarterSourceHeldSelected(ctx context.Context, budget *proofbound.WorkBudget,
	prism, loft *topo.Body, arcSegment int, radii [2]*big.Rat,
	selectedMask uint8) (QuarterSourceHeldPair, bool, error) {
	report := QuarterSourceHeldPair{}
	decline := func(reason string) (QuarterSourceHeldPair, bool, error) {
		return QuarterSourceHeldPair{Reason: reason}, false, nil
	}
	if err := ctx.Err(); err != nil {
		return report, false, err
	}
	if (selectedMask == 0 || selectedMask > 3) || budget == nil ||
		!loftHullFits(radii[0]) || !loftHullFits(radii[1]) {
		return decline("source_radius")
	}
	m := loftHullMath{budget: budget}
	for capIndex, want := range [2]*big.Rat{big.NewRat(5, 1), big.NewRat(11, 2)} {
		cmp, err := m.cmp(radii[capIndex], want)
		if errors.Is(err, errLoftHullSize) {
			return decline("arithmetic_size")
		}
		if err != nil {
			return report, false, err
		}
		if cmp != 0 {
			return decline("source_radius")
		}
	}
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
		if !valid {
			return decline("source_radius")
		}
		cmp, err := m.cmp(lift, radius)
		if errors.Is(err, errLoftHullSize) {
			return decline("arithmetic_size")
		}
		if err != nil {
			return QuarterSourceHeldPair{}, false, err
		}
		if cmp != 0 {
			return decline("source_radius")
		}
	}
	bound := big.NewRat(1, 4)
	for index, seam := range [2]QuarterHeldSeam{selected.Y1, selected.Y3} {
		if selectedMask&uint8(1<<index) == 0 {
			continue
		}
		carrier := big.NewRat(int64(1+2*index), 1)
		sourceConstant := big.NewRat(int64(24-8*index), 1)
		pieces := seam.pieces
		if len(pieces) == 0 || len(pieces) > 512 ||
			seam.BottomX == nil || seam.TopX == nil {
			return decline("held_chain")
		}
		bottom, err := m.cmp(seam.BottomX, pieces[0].lo.x)
		if err != nil {
			return QuarterSourceHeldPair{}, false, err
		}
		top, err := m.cmp(seam.TopX, pieces[len(pieces)-1].hi.x)
		if err != nil || bottom != 0 || top != 0 {
			return QuarterSourceHeldPair{}, false, err
		}
		for _, piece := range pieces {
			ok, checkErr := quarterSourceHeldPiece(ctx, m, piece, carrier, sourceConstant, bound)
			if checkErr != nil || !ok {
				if errors.Is(checkErr, errLoftHullSize) {
					return decline("arithmetic_size")
				}
				if checkErr != nil {
					return QuarterSourceHeldPair{}, false, checkErr
				}
				return decline("source_held_bound")
			}
		}
		if index == 0 {
			report.Y1 = QuarterSourceHeldSeam{Bound: bound, Pieces: len(pieces)}
		} else {
			report.Y3 = QuarterSourceHeldSeam{Bound: bound, Pieces: len(pieces)}
		}
	}
	if selectedMask == 3 {
		slabs, separated, err := quarterSourceHeldSeparation(ctx, m,
			selected.Y1.pieces, selected.Y3.pieces, bound)
		if errors.Is(err, errLoftHullSize) {
			return decline("arithmetic_size")
		}
		if err != nil {
			return QuarterSourceHeldPair{}, false, err
		}
		if !separated {
			return decline("band_separation")
		}
		report.Slabs = slabs
	}
	live, err := quarterPairLive(ctx, m, loft, prism)
	if err != nil {
		return QuarterSourceHeldPair{}, false, err
	}
	if !live {
		return decline("live_pair")
	}
	if err := budget.Err(); err != nil {
		return QuarterSourceHeldPair{}, false, err
	}
	if err := ctx.Err(); err != nil {
		return QuarterSourceHeldPair{}, false, err
	}
	return report, true, nil
}

// CertifyQuarterSourceHeldY1 bounds the selected y=1 chain.
func CertifyQuarterSourceHeldY1(ctx context.Context, budget *proofbound.WorkBudget,
	prism, loft *topo.Body, arcSegment int, radii [2]*big.Rat) (QuarterSourceHeldY1, bool, error) {
	pair, accepted, err := CertifyQuarterSourceHeldSelected(ctx, budget, prism, loft, arcSegment, radii, 1)
	if err != nil || !accepted {
		return QuarterSourceHeldSeam{Reason: pair.Reason}, false, err
	}
	return pair.Y1, true, nil
}

// CertifyQuarterSourceHeldY3 bounds the selected y=3 chain.
func CertifyQuarterSourceHeldY3(ctx context.Context, budget *proofbound.WorkBudget,
	prism, loft *topo.Body, arcSegment int, radii [2]*big.Rat) (QuarterSourceHeldY3, bool, error) {
	pair, accepted, err := CertifyQuarterSourceHeldSelected(ctx, budget, prism, loft, arcSegment, radii, 2)
	if err != nil || !accepted {
		return QuarterSourceHeldSeam{Reason: pair.Reason}, false, err
	}
	return pair.Y3, true, nil
}
