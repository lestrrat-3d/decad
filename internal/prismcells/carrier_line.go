package prismcells

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// SameLineCarrier compares complete supporting lines in exact dyadic arithmetic.
// A shared carrier needs a separate source-span proof before scene construction.
func SameLineCarrier(a, b sectionrecord.LineSeg) bool {
	carrier := newLineCarrier(a)
	var scratch carrierScratch
	return carrier.holds(b, &scratch)
}

// OwnedLine is one operand's whole source line: Owner names the operand.
type OwnedLine struct {
	Owner int
	Line  sectionrecord.LineSeg
}

// SharedLineCarrier reports whether two lines of different owners lie on one
// infinite line, each pair decided exactly as SameLineCarrier decides it. It
// visits each line against every earlier line of another owner, in slice
// order, charges budget one step per visited pair before deciding it, and
// stops at the first shared pair.
func SharedLineCarrier(budget *proofbound.WorkBudget, lines []OwnedLine) (bool, error) {
	carriers := make([]lineCarrier, len(lines))
	for i, line := range lines {
		carriers[i] = newLineCarrier(line.Line)
	}
	var scratch carrierScratch
	for i, line := range lines {
		for j, prior := range lines[:i] {
			if prior.Owner == line.Owner {
				continue
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			if carriers[j].holds(line.Line, &scratch) {
				return true, nil
			}
		}
	}
	return false, nil
}

// lineCarrier is the supporting line of one source segment, prepared for
// repeated containment tests. Its exact rationals are lifted on first need,
// once per line rather than once per pair.
type lineCarrier struct {
	seg    sectionrecord.LineSeg
	finite bool
	lifted bool
	startU *big.Rat
	startV *big.Rat
	dx     *big.Rat
	dy     *big.Rat
}

func newLineCarrier(seg sectionrecord.LineSeg) lineCarrier {
	return lineCarrier{seg: seg, finite: finitePoint(seg.Start) && finitePoint(seg.End)}
}

// carrierScratch holds the rationals the exact fallback rewrites per endpoint.
type carrierScratch struct {
	pu, pv, ex, ey, left, right big.Rat
}

// holds reports whether both endpoints of b lie on c's supporting line: the
// cross product (End − Start) × (p − Start) of c's segment is exactly zero at
// each endpoint p, tested in b's Start, End order.
//
// For finite coordinates proof.OrientSignFloat decides each sign. Its points
// are c's Start and End and p at z = 0, and c's Start lifted to z = 1. The
// fourth point's differences from the first are exactly (0, 0, 1), so the
// determinant det[End − Start, p − Start, (0, 0, 1)] is the planar cross
// product itself. OrientSignFloat's certain answer is the exact sign: its
// float filter clears a proven error bound, and its fallback is an exact
// expansion. An uncertain answer, or a non-finite coordinate, falls back to
// exact rational arithmetic over the same cross product.
func (c *lineCarrier) holds(b sectionrecord.LineSeg, s *carrierScratch) bool {
	for _, p := range [2]sectionrecord.Point2{b.Start, b.End} {
		if c.crossSign(p, s) != 0 {
			return false
		}
	}
	return true
}

// crossSign is the exact sign of (End − Start) × (p − Start) for c's segment.
func (c *lineCarrier) crossSign(p sectionrecord.Point2, s *carrierScratch) int {
	if c.finite && finitePoint(p) {
		start, end := c.seg.Start, c.seg.End
		sign, certain := proofarith.OrientSignFloat(
			r3.Vec{X: start.U, Y: start.V}, r3.Vec{X: end.U, Y: end.V},
			r3.Vec{X: p.U, Y: p.V}, r3.Vec{X: start.U, Y: start.V, Z: 1})
		if certain {
			return sign
		}
	}
	c.lift()
	ex := proofarith.SubRat(&s.ex, proofarith.SetFloatRat(&s.pu, p.U), c.startU)
	ey := proofarith.SubRat(&s.ey, proofarith.SetFloatRat(&s.pv, p.V), c.startV)
	return proofarith.MulRat(&s.left, c.dx, ey).Cmp(proofarith.MulRat(&s.right, c.dy, ex))
}

// lift sets c's exact start and direction once.
func (c *lineCarrier) lift() {
	if c.lifted {
		return
	}
	c.startU, c.startV = proofarith.FloatRat(c.seg.Start.U), proofarith.FloatRat(c.seg.Start.V)
	c.dx = proofarith.SubRat(new(big.Rat), proofarith.FloatRat(c.seg.End.U), c.startU)
	c.dy = proofarith.SubRat(new(big.Rat), proofarith.FloatRat(c.seg.End.V), c.startV)
	c.lifted = true
}

func finitePoint(p sectionrecord.Point2) bool {
	return !math.IsNaN(p.U) && !math.IsInf(p.U, 0) && !math.IsNaN(p.V) && !math.IsInf(p.V, 0)
}
