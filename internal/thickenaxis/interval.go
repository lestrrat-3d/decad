package thickenaxis

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Radial is the revolve arm's radial-axis gate (docs/surface-design.md
// §16.5): the plane-local axis every swept offset must keep a strictly
// positive radius from, held as exact rationals so each comparison below is
// decided rather than measured. A surface of revolution's own normal lies in
// its meridian plane, so offsetting the meridian IS offsetting the surface,
// and the swept offset folds exactly where the offset meridian reaches the
// axis.
type Radial struct{ aU, aV, dU, dV *big.Rat }

// NewRadial holds one exact plane-local revolve axis for the interval gate.
func NewRadial(aU, aV, dU, dV *big.Rat) Radial {
	return Radial{aU: aU, aV: aV, dU: dU, dV: dV}
}

// Rho is axisFrame.toAxis's own radial coordinate, taken over the rationals.
func (r Radial) Rho(u, v *big.Rat) *big.Rat {
	du := new(big.Rat).Sub(u, r.aU)
	dv := new(big.Rat).Sub(v, r.aV)
	return new(big.Rat).Sub(new(big.Rat).Mul(dv, r.dU), new(big.Rat).Mul(du, r.dV))
}

// leastOverBox is the least radius any point of one exact box reaches. ρ is
// affine in (u, v), so its minimum over a box sits at a corner.
func (r Radial) leastOverBox(b thickenExactBox) *big.Rat {
	least := r.Rho(b.minU, b.minV)
	for _, corner := range [][2]*big.Rat{{b.minU, b.maxV}, {b.maxU, b.minV}, {b.maxU, b.maxV}} {
		if got := r.Rho(corner[0], corner[1]); got.Cmp(least) < 0 {
			least = got
		}
	}
	return least
}

// Require refuses a swept offset whose least radius is not proven positive.
func (r Radial) Require(least *big.Rat) error {
	if least.Sign() <= 0 {
		return fmt.Errorf(`%w: the swept offset reaches the revolve axis (least radius %s mm)`,
			decaderr.ErrUnsupported, least.FloatString(9))
	}
	return nil
}

// thickenAffine is an exact coordinate a+bτ. Every axis-line endpoint and
// corner-arc endpoint has this form because right-angle miter coefficients
// and axis normals are integers. No intermediate offset is constructed.
type thickenAffine struct{ a, b *big.Rat }

type thickenMovingPoint struct{ u, v thickenAffine }

type thickenMovingPiece struct {
	line       bool
	horizontal bool
	start, end thickenMovingPoint
	center     thickenExactPoint
}

type thickenExactBox struct{ minU, maxU, minV, maxV *big.Rat }

func thickenAffineCoord(x float64, step int) thickenAffine {
	return thickenAffine{a: proofarith.FloatRat(x), b: big.NewRat(int64(step), 1)}
}

func thickenMovingOffset(u, v float64, du, dv int) thickenMovingPoint {
	return thickenMovingPoint{u: thickenAffineCoord(u, du), v: thickenAffineCoord(v, dv)}
}

func thickenAffineSub(a, b thickenAffine) freeform.RatPoly {
	return freeform.RatPoly{new(big.Rat).Sub(a.a, b.a), new(big.Rat).Sub(a.b, b.b)}
}

func thickenAffineConst(a thickenAffine, b *big.Rat) freeform.RatPoly {
	return freeform.RatPoly{new(big.Rat).Sub(a.a, b), new(big.Rat).Set(a.b)}
}

func thickenAffineAt(a thickenAffine, at *big.Rat) *big.Rat {
	return new(big.Rat).Add(a.a, new(big.Rat).Mul(a.b, at))
}

func thickenRange(values ...*big.Rat) (*big.Rat, *big.Rat) {
	lo, hi := values[0], values[0]
	for _, v := range values[1:] {
		if v.Cmp(lo) < 0 {
			lo = v
		}
		if v.Cmp(hi) > 0 {
			hi = v
		}
	}
	return lo, hi
}

// thickenPieceBox encloses a piece over every τ in [0, limit] using only
// affine endpoint extrema. A corner arc stays in its fixed radial quadrant.
func thickenPieceBox(p thickenMovingPiece, limit *big.Rat) thickenExactBox {
	if p.line {
		zero := new(big.Rat)
		minU, maxU := thickenRange(thickenAffineAt(p.start.u, zero), thickenAffineAt(p.end.u, zero),
			thickenAffineAt(p.start.u, limit), thickenAffineAt(p.end.u, limit))
		minV, maxV := thickenRange(thickenAffineAt(p.start.v, zero), thickenAffineAt(p.end.v, zero),
			thickenAffineAt(p.start.v, limit), thickenAffineAt(p.end.v, limit))
		return thickenExactBox{minU: minU, maxU: maxU, minV: minV, maxV: maxV}
	}
	sx, sy := p.start.u.b, p.start.v.b
	if sx.Sign() == 0 {
		sx = p.end.u.b
	}
	if sy.Sign() == 0 {
		sy = p.end.v.b
	}
	minU, maxU := thickenRange(p.center.u, new(big.Rat).Add(p.center.u, new(big.Rat).Mul(sx, limit)))
	minV, maxV := thickenRange(p.center.v, new(big.Rat).Add(p.center.v, new(big.Rat).Mul(sy, limit)))
	return thickenExactBox{minU: minU, maxU: maxU, minV: minV, maxV: maxV}
}

func thickenBoxesDisjoint(a, b thickenExactBox) bool {
	return a.maxU.Cmp(b.minU) < 0 || b.maxU.Cmp(a.minU) < 0 ||
		a.maxV.Cmp(b.minV) < 0 || b.maxV.Cmp(a.minV) < 0
}

func thickenContactEvent(ctx context.Context, p freeform.RatPoly, limit *big.Rat) (bool, error) {
	p = freeform.RpSquareFree(p)
	if freeform.RpDeg(p) < 1 {
		return false, nil
	}
	chain, err := freeform.SturmChainIntContext(ctx, p)
	if err != nil {
		return false, err
	}
	return freeform.SturmCount(chain, new(big.Rat), limit) > 0, nil
}

// AxisIntervalClear isolates every possible first contact event of
// nonadjacent moving pieces. Their supporting equations have degree at most
// two in τ: line endpoints and carriers are affine, and a corner radius is τ.
// A root of a supporting equation can be a false contact on a trimmed piece;
// refusing it is conservative. If no such root lies in (0, amount], no actual
// contact can begin there. The audited endpoint decides the final winding.
func AxisIntervalClear(ctx context.Context, walks []survey2d.SideWalk, dirs []AxisDir,
	sense int, amount float64, budget *proofbound.WorkBudget, radial *Radial) error {
	n := len(dirs)
	joins := make([]struct {
		arc              bool
		m, before, after thickenMovingPoint
	}, n)
	for i := range dirs {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		prev, cur := dirs[(i+n-1)%n], dirs[i]
		v := walks[i]
		j := &joins[i]
		j.arc = prev.u*cur.v-prev.v*cur.u == -sense
		j.before = thickenMovingOffset(v.StartU, v.StartV, sense*-prev.v, sense*prev.u)
		j.after = thickenMovingOffset(v.StartU, v.StartV, sense*-cur.v, sense*cur.u)
		j.m = thickenMovingOffset(v.StartU, v.StartV,
			sense*(-prev.v-cur.v), sense*(prev.u+cur.u))
	}
	var pieces []thickenMovingPiece
	for i, dir := range dirs {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		start, end := joins[i].m, joins[(i+1)%n].m
		if joins[i].arc {
			start = joins[i].after
		}
		if joins[(i+1)%n].arc {
			end = joins[(i+1)%n].before
		}
		pieces = append(pieces, thickenMovingPiece{line: true, horizontal: dir.u != 0,
			start: start, end: end})
		if joins[(i+1)%n].arc {
			v := walks[(i+1)%n]
			pieces = append(pieces, thickenMovingPiece{
				start: joins[(i+1)%n].before, end: joins[(i+1)%n].after,
				center: thickenExactPoint{u: proofarith.FloatRat(v.StartU), v: proofarith.FloatRat(v.StartV)},
			})
		}
	}
	return thickenPiecesIntervalClear(ctx, pieces, proofarith.FloatRat(amount), budget, radial)
}

// thickenPiecesIntervalClear is the interval scan itself, over one CLOSED ring
// of moving pieces in loop order: consecutive pieces are the joins the
// construction prescribes and are excluded, and every other pair is isolated
// exactly.
func thickenPiecesIntervalClear(ctx context.Context, pieces []thickenMovingPiece,
	limit *big.Rat, budget *proofbound.WorkBudget, radial *Radial) error {
	boxes := make([]thickenExactBox, len(pieces))
	for i, piece := range pieces {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		boxes[i] = thickenPieceBox(piece, limit)
	}
	if radial != nil {
		// Each box encloses its own piece over every τ in [0, amount], so the
		// least radius the whole swept family reaches is the least any box
		// corner reaches. The boxes at τ = 0 are the SOURCE walks, so one scan
		// certifies the source section, the requested offset, and every
		// intermediate offset between them.
		for _, box := range boxes {
			if err := radial.Require(radial.leastOverBox(box)); err != nil {
				return err
			}
		}
	}
	check := func(p freeform.RatPoly) error {
		contact, err := thickenContactEvent(ctx, p, limit)
		if err != nil {
			return err
		}
		if contact {
			return fmt.Errorf(`%w: the offset interval contains a possible nonadjacent contact`, decaderr.ErrUnsupported)
		}
		return nil
	}
	for i, a := range pieces {
		if a.line {
			advance := thickenAffineSub(a.end.v, a.start.v)
			if a.horizontal {
				advance = thickenAffineSub(a.end.u, a.start.u)
			}
			if err := check(advance); err != nil {
				return err
			}
		}
		for j := i + 1; j < len(pieces); j++ {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return err
			}
			if j == i+1 || (i == 0 && j == len(pieces)-1) {
				continue
			}
			if thickenBoxesDisjoint(boxes[i], boxes[j]) {
				continue
			}
			b := pieces[j]
			if a.line && b.line {
				if thickenLineLineContact(a, b, limit) {
					return fmt.Errorf(`%w: the offset interval contains a nonadjacent line contact`, decaderr.ErrUnsupported)
				}
				continue
			}
			var candidates []freeform.RatPoly
			switch {
			case a.line:
				candidates = thickenLineArcEvents(a, b)
			case b.line:
				candidates = thickenLineArcEvents(b, a)
			default:
				du := new(big.Rat).Sub(a.center.u, b.center.u)
				dv := new(big.Rat).Sub(a.center.v, b.center.v)
				d2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
				candidates = []freeform.RatPoly{{new(big.Rat).Neg(d2), new(big.Rat), big.NewRat(4, 1)}}
			}
			for _, p := range candidates {
				if err := check(p); err != nil {
					return err
				}
			}
		}
	}
	return nil
}

// A first finite line-line contact occurs when one endpoint reaches the
// other's support, or parallel supports become equal. These parameters are
// rational; test finite segment inclusion at each exact root.
func thickenLineLineContact(a, b thickenMovingPiece, limit *big.Rat) bool {
	for _, p := range thickenLineLineEvents(a, b) {
		p = freeform.RpTrim(p)
		if len(p) != 2 || p[1].Sign() == 0 {
			continue
		}
		root := new(big.Rat).Quo(new(big.Rat).Neg(p[0]), p[1])
		if root.Sign() <= 0 || root.Cmp(limit) > 0 {
			continue
		}
		if thickenLinesTouchAt(a, b, root) {
			return true
		}
	}
	return false
}

func thickenRangesOverlap(a0, a1, b0, b1 *big.Rat) bool {
	aLo, aHi := thickenRange(a0, a1)
	bLo, bHi := thickenRange(b0, b1)
	return aLo.Cmp(bHi) <= 0 && bLo.Cmp(aHi) <= 0
}

func thickenWithin(value, a, b *big.Rat) bool {
	lo, hi := thickenRange(a, b)
	return lo.Cmp(value) <= 0 && value.Cmp(hi) <= 0
}

func thickenLinesTouchAt(a, b thickenMovingPiece, at *big.Rat) bool {
	au0, av0 := thickenAffineAt(a.start.u, at), thickenAffineAt(a.start.v, at)
	au1, av1 := thickenAffineAt(a.end.u, at), thickenAffineAt(a.end.v, at)
	bu0, bv0 := thickenAffineAt(b.start.u, at), thickenAffineAt(b.start.v, at)
	bu1, bv1 := thickenAffineAt(b.end.u, at), thickenAffineAt(b.end.v, at)
	if a.horizontal && b.horizontal {
		return av0.Cmp(bv0) == 0 && thickenRangesOverlap(au0, au1, bu0, bu1)
	}
	if !a.horizontal && !b.horizontal {
		return au0.Cmp(bu0) == 0 && thickenRangesOverlap(av0, av1, bv0, bv1)
	}
	if a.horizontal {
		return thickenWithin(bu0, au0, au1) && thickenWithin(av0, bv0, bv1)
	}
	return thickenWithin(au0, bu0, bu1) && thickenWithin(bv0, av0, av1)
}

func thickenLineLineEvents(a, b thickenMovingPiece) []freeform.RatPoly {
	if a.horizontal && b.horizontal {
		return []freeform.RatPoly{
			thickenAffineSub(a.start.v, b.start.v),
			thickenAffineSub(a.start.u, b.start.u), thickenAffineSub(a.start.u, b.end.u),
			thickenAffineSub(a.end.u, b.start.u), thickenAffineSub(a.end.u, b.end.u),
		}
	}
	if !a.horizontal && !b.horizontal {
		return []freeform.RatPoly{
			thickenAffineSub(a.start.u, b.start.u),
			thickenAffineSub(a.start.v, b.start.v), thickenAffineSub(a.start.v, b.end.v),
			thickenAffineSub(a.end.v, b.start.v), thickenAffineSub(a.end.v, b.end.v),
		}
	}
	horizontal, vertical := a, b
	if !a.horizontal {
		horizontal, vertical = b, a
	}
	return []freeform.RatPoly{
		thickenAffineSub(vertical.start.u, horizontal.start.u),
		thickenAffineSub(vertical.start.u, horizontal.end.u),
		thickenAffineSub(horizontal.start.v, vertical.start.v),
		thickenAffineSub(horizontal.start.v, vertical.end.v),
	}
}

func thickenLineArcEvents(line, arc thickenMovingPiece) []freeform.RatPoly {
	var distance freeform.RatPoly
	var arcStart, arcEnd, lineCoord thickenAffine
	if line.horizontal {
		distance = thickenAffineConst(line.start.v, arc.center.v)
		arcStart, arcEnd = arc.start.v, arc.end.v
		lineCoord = line.start.v
	} else {
		distance = thickenAffineConst(line.start.u, arc.center.u)
		arcStart, arcEnd = arc.start.u, arc.end.u
		lineCoord = line.start.u
	}
	tau := freeform.RatPoly{new(big.Rat), big.NewRat(1, 1)}
	contact := freeform.RpSub(freeform.RpMul(distance, distance), freeform.RpMul(tau, tau))
	endpoint := func(p thickenMovingPoint) freeform.RatPoly {
		du := thickenAffineConst(p.u, arc.center.u)
		dv := thickenAffineConst(p.v, arc.center.v)
		return freeform.RpSub(freeform.RpAdd(freeform.RpMul(du, du), freeform.RpMul(dv, dv)), freeform.RpMul(tau, tau))
	}
	return []freeform.RatPoly{contact, endpoint(line.start), endpoint(line.end),
		thickenAffineSub(arcStart, lineCoord), thickenAffineSub(arcEnd, lineCoord)}
}
