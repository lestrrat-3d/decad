package loftrecord

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// DirectSectionLineSupportsBoundary proves the authenticated section's entire
// line-and-arc boundary stays on one side of the named whole line and meets
// that line only on the named segment. It does not classify the filled region,
// any other operand, or Boolean contact.
func (p Payload) DirectSectionLineSupportsBoundary(ctx context.Context, body *topo.Body, section, segment int,
	budget *proofbound.WorkBudget) (bool, error) {
	_, _, ok, err := p.directSectionLine(ctx, body, section, segment, budget)
	if err != nil || !ok {
		return false, err
	}
	profile := p.Profile0
	if section == 1 {
		profile = p.Profile1
	}
	// The source boundary must be an exact closed walk. Sketch endpoints can
	// otherwise agree only within their solver tolerance.
	var first, previous sectionrecord.Point2
	for i, raw := range profile.Outer.Segments {
		if err := budget.Step(); err != nil {
			return false, err
		}
		var start, end sectionrecord.Point2
		var t0, t1 float64
		switch s := raw.(type) {
		case sectionrecord.LineSeg:
			start, end, t0, t1 = s.Start, s.End, s.TStart, s.TEnd
		case sectionrecord.ArcSeg:
			start, end, t0, t1 = s.Start, s.End, s.TStart, s.TEnd
		default:
			return false, nil
		}
		if !wholeDirectRange(t0, t1) {
			return false, nil
		}
		if t0 > t1 {
			start, end = end, start
		}
		if !proofbound.FiniteVec(r3.Vec{X: start.U, Y: start.V}) ||
			!proofbound.FiniteVec(r3.Vec{X: end.U, Y: end.V}) || i > 0 && start != previous {
			return false, nil
		}
		if i == 0 {
			first = start
		}
		previous = end
	}
	if previous != first {
		return false, nil
	}
	line, ok := profile.Outer.Segments[segment].(sectionrecord.LineSeg)
	if !ok {
		return false, nil
	}
	type point [2]*big.Rat
	read := func(q sectionrecord.Point2) (point, bool) {
		if !proofbound.FiniteVec(r3.Vec{X: q.U, Y: q.V}) {
			return point{}, false
		}
		return point{new(big.Rat).SetFloat64(q.U), new(big.Rat).SetFloat64(q.V)}, true
	}
	a, oka := read(line.Start)
	b, okb := read(line.End)
	if !oka || !okb {
		return false, nil
	}
	dx, dy := new(big.Rat).Sub(b[0], a[0]), new(big.Rat).Sub(b[1], a[1])
	length := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	if length.Sign() == 0 {
		return false, nil
	}
	det := func(q point) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(dx, new(big.Rat).Sub(q[1], a[1])),
			new(big.Rat).Mul(dy, new(big.Rat).Sub(q[0], a[0])))
	}
	equal := func(q, r point) bool { return q[0].Cmp(r[0]) == 0 && q[1].Cmp(r[1]) == 0 }
	endpoint := func(q point) bool { return equal(q, a) || equal(q, b) }
	side := 0
	claim := func(value *big.Rat) bool {
		s := value.Sign()
		if s == 0 {
			return true
		}
		if side == 0 {
			side = s
		}
		return s == side
	}
	for i, raw := range profile.Outer.Segments {
		if err := budget.ReserveBytes(64 << 10); err != nil {
			return false, err
		}
		if err := budget.Step(); err != nil {
			return false, err
		}
		if i == segment {
			continue
		}
		switch s := raw.(type) {
		case sectionrecord.LineSeg:
			if !wholeDirectRange(s.TStart, s.TEnd) {
				return false, nil
			}
			x, xok := read(s.Start)
			y, yok := read(s.End)
			if !xok || !yok || equal(x, y) {
				return false, nil
			}
			sx, sy := det(x), det(y)
			if !claim(sx) || !claim(sy) || sx.Sign() == 0 && (!endpoint(x) || sy.Sign() == 0) ||
				sy.Sign() == 0 && !endpoint(y) {
				return false, nil
			}
		case sectionrecord.ArcSeg:
			if !wholeDirectRange(s.TStart, s.TEnd) {
				return false, nil
			}
			c, cok := read(s.Center)
			x, xok := read(s.Start)
			y, yok := read(s.End)
			if !cok || !xok || !yok {
				return false, nil
			}
			radiusSquared := func(q point) *big.Rat {
				u, v := new(big.Rat).Sub(q[0], c[0]), new(big.Rat).Sub(q[1], c[1])
				return new(big.Rat).Add(new(big.Rat).Mul(u, u), new(big.Rat).Mul(v, v))
			}
			r := radiusSquared(x)
			if r.Sign() == 0 || r.Cmp(radiusSquared(y)) != 0 {
				return false, nil
			}
			centerSide := det(c)
			if centerSide.Sign() == 0 || !claim(centerSide) {
				return false, nil
			}
			cmp := new(big.Rat).Mul(centerSide, centerSide).Cmp(new(big.Rat).Mul(length, r))
			if cmp < 0 {
				return false, nil
			}
			if cmp == 0 {
				// The entire circle touches the line at only one point. Requiring
				// a pinned endpoint there locates it on the named segment.
				if (det(x).Sign() != 0 || !endpoint(x)) && (det(y).Sign() != 0 || !endpoint(y)) {
					return false, nil
				}
			}
		default:
			return false, nil
		}
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	return side != 0, nil
}

func wholeDirectRange(start, end float64) bool {
	return start == 0 && end == 1 || start == 1 && end == 0
}
