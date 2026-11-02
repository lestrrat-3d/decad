package prismboolean

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ReplayWholeCircleMatch proves exact source-circle identity under a signed
// section map. The caller separately authenticates the records and their levels.
func ReplayWholeCircleMatch(ctx context.Context, group, leaf momentinput.Profile, groupPlacement, leafPlacement prismplacement.Operand, budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	m, ok, err := prismplacement.ReplayParallelSignedMap(ctx, groupPlacement, leafPlacement, budget)
	if err != nil || !ok {
		return false, err
	}
	ok, err = prismplacement.ReplaySameShiftedInterval(ctx, groupPlacement, leafPlacement, m.Shift, budget)
	if err != nil || !ok {
		return false, err
	}
	if err := budget.SpendWork(ctx, 16); err != nil {
		return false, err
	}
	if len(group.Holes) != 0 || len(leaf.Holes) != 0 || len(group.Outer.Segments) != 1 || len(leaf.Outer.Segments) != 1 {
		return false, nil
	}
	g, ok := group.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok {
		return false, nil
	}
	l, ok := leaf.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok {
		return false, nil
	}
	if m.OffsetU == nil || m.OffsetV == nil || l.Radius != g.Radius || l.TStart != 0 || l.TEnd != 1 || g.TStart != 0 || g.TEnd != 1 {
		return false, nil
	}
	radius := l.Radius.Base()
	if math.IsNaN(radius) || math.IsInf(radius, 0) || radius <= 0 {
		return false, nil
	}
	if (m.U != 1 && m.U != -1 && m.U != 2 && m.U != -2) || (m.V != 1 && m.V != -1 && m.V != 2 && m.V != -2) || m.U == m.V || m.U == -m.V {
		return false, nil
	}
	orientation := 1
	if m.U < 0 {
		orientation = -orientation
	}
	if m.V < 0 {
		orientation = -orientation
	}
	if m.U == 2 || m.U == -2 {
		orientation = -orientation
	}
	if (l.CCW != g.CCW) != (orientation < 0) {
		return false, nil
	}
	mapped := [2]*big.Rat{m.OffsetU, m.OffsetV}
	for i, val := range [2]float64{l.Center.U, l.Center.V} {
		q, err := proof.ReplayFloatRat(ctx, val, budget)
		if err != nil {
			return false, err
		}
		direction := m.U
		if i == 1 {
			direction = m.V
		}
		index := direction
		if index < 0 {
			index = -index
		}
		index--
		op := byte('+')
		if direction < 0 {
			op = '-'
		}
		mapped[index], err = proof.ReplayRatArithmetic(ctx, mapped[index], q, op, budget)
		if err != nil {
			return false, err
		}
	}
	for i, val := range [2]float64{g.Center.U, g.Center.V} {
		q, err := proof.ReplayFloatRat(ctx, val, budget)
		if err != nil {
			return false, err
		}
		cmp, err := proof.ReplayRatCompare(ctx, mapped[i], q, budget)
		if err != nil {
			return false, err
		}
		if cmp != 0 {
			return false, nil
		}
	}
	return true, nil
}
