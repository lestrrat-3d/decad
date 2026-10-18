package loftcontact

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// QuarterHeldPair certifies only the selected outer arc-wall chains of the
// original faceted quarter Loft at the two authenticated prism walls.
type QuarterHeldPair struct {
	Reason string
	Y1, Y3 QuarterHeldSeam
}

func quarterPairLive(ctx context.Context, m loftHullMath, loft, prism *topo.Body) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if loft == nil || prism == nil || loft.Doc == nil || loft.Doc != prism.Doc ||
		loft.Owner == nil || prism.Owner == nil || !loft.Solid || !prism.Solid ||
		loft.Kind != topo.BodySolid || prism.Kind != topo.BodySolid ||
		loft.Origin.Role != provenance.RoleBody || prism.Origin.Role != provenance.RoleBody ||
		loft.Origin.Producer != 0 || prism.Origin.Producer != 1 {
		return false, nil
	}
	loft.Doc.LiveMu.RLock()
	if len(loft.Doc.LiveSet) > 2 {
		loft.Doc.LiveMu.RUnlock()
		return false, nil
	}
	live := append([]*topo.Body(nil), loft.Doc.LiveSet...)
	loft.Doc.LiveMu.RUnlock()
	if len(live) != 2 {
		return false, nil
	}
	for i, want := range [2]*topo.Body{loft, prism} {
		if err := m.step(); err != nil {
			return false, err
		}
		if live[i] != want || live[i].Doc != loft.Doc {
			return false, nil
		}
	}
	return true, nil
}

func quarterPairPrism(m loftHullMath, prism *topo.Body) (float64, float64, bool, error) {
	pp, ok := prism.Payload.(prismrecord.Payload)
	if !ok || pp.Xform != r3.Identity() || pp.Frame.Origin() != (r3.Vec{}) ||
		pp.Frame.U() != (r3.Vec{X: 1}) || pp.Frame.V() != (r3.Vec{Y: 1}) ||
		pp.Z0 != 0 || pp.Z1 != 12 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		pp.SectionDelta != 0 || pp.SurfaceResult || pp.BlendKind != "" || len(pp.BlendSegs) != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 4 {
		return 0, 0, false, nil
	}
	corners := [4]sectionrecord.Point2{{U: 2, V: 1}, {U: 6, V: 1}, {U: 6, V: 3}, {U: 2, V: 3}}
	var walked [4][2]sectionrecord.Point2
	for i, raw := range pp.Profile.Outer.Segments {
		if err := m.step(); err != nil {
			return 0, 0, false, err
		}
		line, lineOK := raw.(sectionrecord.LineSeg)
		if !lineOK {
			return 0, 0, false, nil
		}
		switch {
		case line.TStart == 0 && line.TEnd == 1:
			walked[i] = [2]sectionrecord.Point2{line.Start, line.End}
		case line.TStart == 1 && line.TEnd == 0:
			walked[i] = [2]sectionrecord.Point2{line.End, line.Start}
		default:
			return 0, 0, false, nil
		}
	}
	start := -1
	for i, corner := range corners {
		if err := m.step(); err != nil {
			return 0, 0, false, err
		}
		if walked[0][0] == corner {
			start = i
			break
		}
	}
	if start < 0 {
		return 0, 0, false, nil
	}
	for i := range walked {
		if err := m.step(); err != nil {
			return 0, 0, false, err
		}
		if walked[i][0] != corners[(start+i)%4] || walked[i][1] != corners[(start+i+1)%4] {
			return 0, 0, false, nil
		}
	}
	// Both values come from the authenticated full horizontal wall records.
	var y1, y3 float64
	for _, edge := range walked {
		if err := m.step(); err != nil {
			return 0, 0, false, err
		}
		if edge[0].V != edge[1].V {
			continue
		}
		if edge[0].V == corners[0].V {
			y1 = edge[0].V
		} else if edge[0].V == corners[2].V {
			y3 = edge[0].V
		}
	}
	return y1, y3, y1 == 1 && y3 == 3, nil
}

func quarterPairArc(m loftHullMath, loft *topo.Body) (int, bool, error) {
	pl, ok := loft.Payload.(loftrecord.Payload)
	if !ok || pl.Plane0.Origin.Z != 0 || pl.Plane1.Origin.Z != 10 ||
		pl.Frame0.U() != (r3.Vec{X: 1}) || pl.Frame0.V() != (r3.Vec{Y: 1}) ||
		pl.Frame1.U() != (r3.Vec{X: 1}) || pl.Frame1.V() != (r3.Vec{Y: 1}) ||
		pl.Plane0.Origin.X != 0 || pl.Plane0.Origin.Y != 0 ||
		pl.Plane1.Origin.X != 0 || pl.Plane1.Origin.Y != 0 ||
		len(pl.Profile0.Outer.Segments) == 0 || len(pl.Profile0.Outer.Segments) > 512 ||
		len(pl.Profile0.Outer.Segments) != len(pl.Profile1.Outer.Segments) {
		return -1, false, nil
	}
	arc := -1
	for i, raw := range pl.Profile0.Outer.Segments {
		if err := m.step(); err != nil {
			return -1, false, err
		}
		if _, isArc := raw.(sectionrecord.ArcSeg); !isArc {
			continue
		}
		if _, isArc := pl.Profile1.Outer.Segments[i].(sectionrecord.ArcSeg); !isArc || arc >= 0 {
			return -1, false, nil
		}
		arc = i
	}
	return arc, arc >= 0, nil
}

func quarterHeldAtZ(m loftHullMath, piece heldPiece, z *big.Rat) (*big.Rat, error) {
	dz, err := m.sub(piece.hi.z, piece.lo.z)
	if err != nil {
		return nil, err
	}
	part, err := m.sub(z, piece.lo.z)
	if err != nil {
		return nil, err
	}
	t, err := m.quotient(part, dz)
	if err != nil {
		return nil, err
	}
	dx, err := m.sub(piece.hi.x, piece.lo.x)
	if err != nil {
		return nil, err
	}
	part, err = m.mul(t, dx)
	if err != nil {
		return nil, err
	}
	return m.add(piece.lo.x, part)
}

func quarterHeldPairOrder(m loftHullMath, y1, y3 []heldPiece) (bool, error) {
	if len(y1) == 0 || len(y3) == 0 || len(y1) > 512 || len(y3) > 512 {
		return false, nil
	}
	zero, ten, two, six := big.NewRat(0, 1), big.NewRat(10, 1), big.NewRat(2, 1), big.NewRat(6, 1)
	z := zero
	i, j := 0, 0
	for i < len(y1) && j < len(y3) {
		if err := m.step(); err != nil {
			return false, err
		}
		p, q := y1[i], y3[j]
		for _, lo := range []*big.Rat{p.lo.z, q.lo.z} {
			cmp, err := m.cmp(lo, z)
			if err != nil || cmp > 0 {
				return false, err
			}
		}
		end := p.hi.z
		cmp, err := m.cmp(q.hi.z, end)
		if err != nil {
			return false, err
		}
		if cmp < 0 {
			end = q.hi.z
		}
		cmp, err = m.cmp(z, end)
		if err != nil || cmp >= 0 {
			return false, err
		}
		for _, height := range []*big.Rat{z, end} {
			outer, err := quarterHeldAtZ(m, p, height)
			if err != nil {
				return false, err
			}
			inner, err := quarterHeldAtZ(m, q, height)
			if err != nil {
				return false, err
			}
			for _, operands := range [][2]*big.Rat{{two, inner}, {inner, outer}, {outer, six}} {
				cmp, err := m.cmp(operands[0], operands[1])
				if err != nil || cmp >= 0 {
					return false, err
				}
			}
		}
		z = end
		cmp, err = m.cmp(p.hi.z, end)
		if err != nil {
			return false, err
		}
		if cmp == 0 {
			i++
		}
		cmp, err = m.cmp(q.hi.z, end)
		if err != nil {
			return false, err
		}
		if cmp == 0 {
			j++
		}
	}
	cmp, err := m.cmp(z, ten)
	return err == nil && cmp == 0 && i == len(y1) && j == len(y3), err
}

// CertifyQuarterHeldPair proves the two selected original held outer-wall
// chains under one shared bounded output budget. It proves no curved source
// seam, whole-wall coverage, volume, measurement or public Boolean result.
func CertifyQuarterHeldPair(ctx context.Context, budget *proofbound.WorkBudget,
	prism, loft *topo.Body) (QuarterHeldPair, bool, error) {
	decline := func(reason string) (QuarterHeldPair, bool, error) {
		return QuarterHeldPair{Reason: reason}, false, nil
	}
	if err := ctx.Err(); err != nil {
		return QuarterHeldPair{}, false, err
	}
	if budget == nil {
		return decline("budget")
	}
	if err := budget.Err(); err != nil {
		return QuarterHeldPair{}, false, err
	}
	m := loftHullMath{budget: proofbound.NewWallWorkBudgetWithOperation(65_536, budget)}
	live, err := quarterPairLive(ctx, m, loft, prism)
	if err != nil {
		return QuarterHeldPair{}, false, err
	}
	if !live {
		return decline("live_pair")
	}
	y1, y3, prismOK, err := quarterPairPrism(m, prism)
	if err != nil {
		return QuarterHeldPair{}, false, err
	}
	if !prismOK {
		return decline("prism_record")
	}
	arc, arcOK, err := quarterPairArc(m, loft)
	if err != nil {
		return QuarterHeldPair{}, false, err
	}
	if !arcOK {
		return decline("loft_arc")
	}
	a, accepted, err := CertifyQuarterHeldSeam(ctx, m.budget, loft, arc, y1)
	if err != nil {
		return QuarterHeldPair{}, false, err
	}
	if !accepted {
		return decline("y1." + a.Reason)
	}
	b, accepted, err := CertifyQuarterHeldSeam(ctx, m.budget, loft, arc, y3)
	if err != nil {
		return QuarterHeldPair{}, false, err
	}
	if !accepted {
		return decline("y3." + b.Reason)
	}
	ordered, err := quarterHeldPairOrder(m, a.pieces, b.pieces)
	if err != nil {
		return QuarterHeldPair{}, false, err
	}
	if !ordered {
		return decline("held_order")
	}
	// The two reader scans rechecked Loft liveness. This final charged snapshot
	// also catches a prism retired while either scan was in flight.
	live, err = quarterPairLive(ctx, m, loft, prism)
	if err != nil {
		return QuarterHeldPair{}, false, err
	}
	if !live {
		return decline("live_pair")
	}
	if err := m.budget.Err(); err != nil {
		return QuarterHeldPair{}, false, err
	}
	return QuarterHeldPair{Y1: a, Y3: b}, true, nil
}
