// Package thickenaxis certifies exact axis-parallel section offsets.
package thickenaxis

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// AxisDir is an exact unit direction along one recorded plane axis.
type AxisDir struct{ u, v int }

// NewAxisDir records a unit axis direction.
func NewAxisDir(u, v int) AxisDir { return AxisDir{u: u, v: v} }

// U returns the direction's U component.
func (d AxisDir) U() int { return d.u }

// V returns the direction's V component.
func (d AxisDir) V() int { return d.v }

type thickenExactPoint struct{ u, v *big.Rat }

type thickenAxisJoin struct {
	arc              bool
	m, before, after thickenExactPoint
}

// AxisDirections checks one closed walk and returns its exact unit axis directions.
func AxisDirections(walks []survey2d.SideWalk, budget *proofbound.WorkBudget) ([]AxisDir, error) {
	n := len(walks)
	if n < 4 {
		return nil, fmt.Errorf(`%w: an axis-parallel prism loop needs at least four walks`, decaderr.ErrUnsupported)
	}
	dirs := make([]AxisDir, n)
	for i, w := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		next := walks[(i+1)%n]
		if w.StartBound.U != 0 || w.StartBound.V != 0 || w.EndBound.U != 0 || w.EndBound.V != 0 {
			return nil, fmt.Errorf(`%w: a source line endpoint has an unresolved coordinate bound`, decaderr.ErrUnsupported)
		}
		if w.EndU != next.StartU || w.EndV != next.StartV {
			return nil, fmt.Errorf(`%w: the prism loop has no exact adjacent joins`, decaderr.ErrUnsupported)
		}
		switch {
		case w.StartU == w.EndU && w.StartV < w.EndV:
			dirs[i] = AxisDir{v: 1}
		case w.StartU == w.EndU && w.StartV > w.EndV:
			dirs[i] = AxisDir{v: -1}
		case w.StartV == w.EndV && w.StartU < w.EndU:
			dirs[i] = AxisDir{u: 1}
		case w.StartV == w.EndV && w.StartU > w.EndU:
			dirs[i] = AxisDir{u: -1}
		default:
			return nil, fmt.Errorf(`%w: the prism loop is not axis-parallel`, decaderr.ErrUnsupported)
		}
		if proofarith.FloatRat(w.StartU) == nil || proofarith.FloatRat(w.StartV) == nil ||
			proofarith.FloatRat(w.EndU) == nil || proofarith.FloatRat(w.EndV) == nil {
			return nil, fmt.Errorf(`%w: a prism boundary coordinate is not finite`, decaderr.ErrUnsupported)
		}
	}
	for i := range dirs {
		p, q := dirs[(i+n-1)%n], dirs[i]
		if p.u*q.v-p.v*q.u == 0 {
			return nil, fmt.Errorf(`%w: a prism corner is not a right angle`, decaderr.ErrUnsupported)
		}
	}
	return dirs, nil
}

// CertifyAxisOffset checks generated segments against exact axis offsets.
func CertifyAxisOffset(walks []survey2d.SideWalk, dirs []AxisDir, generated sectionrecord.LoopRecord,
	sense int, amount float64, budget *proofbound.WorkBudget) error {
	n := len(dirs)
	t := proofarith.FloatRat(amount)
	joins := make([]thickenAxisJoin, n)
	for i := range dirs {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		prev, cur := dirs[(i+n-1)%n], dirs[i]
		turn := prev.u*cur.v - prev.v*cur.u
		vertex := walks[i]
		j := &joins[i]
		j.arc = turn == -sense
		j.before = thickenExactOffset(vertex.StartU, vertex.StartV, sense*(-prev.v), sense*prev.u, t)
		j.after = thickenExactOffset(vertex.StartU, vertex.StartV, sense*(-cur.v), sense*cur.u, t)
		j.m = thickenExactOffset(vertex.StartU, vertex.StartV,
			sense*(-prev.v-cur.v), sense*(prev.u+cur.u), t)
	}
	idx := 0
	for i := range dirs {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		if idx >= len(generated.Segments) {
			return fmt.Errorf(`%w: the offset dropped a line walk`, decaderr.ErrUnsupported)
		}
		line, ok := generated.Segments[idx].(sectionrecord.LineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return fmt.Errorf(`%w: the offset line is not the generated walk`, decaderr.ErrUnsupported)
		}
		start, end := joins[i].m, joins[(i+1)%n].m
		if joins[i].arc {
			start = joins[i].after
		}
		if joins[(i+1)%n].arc {
			end = joins[(i+1)%n].before
		}
		if !thickenPointIsExact(line.Start, start) || !thickenPointIsExact(line.End, end) {
			return fmt.Errorf(`%w: a generated offset line coordinate is rounded`, decaderr.ErrUnsupported)
		}
		idx++
		corner := (i + 1) % n
		if !joins[corner].arc {
			continue
		}
		if idx >= len(generated.Segments) {
			return fmt.Errorf(`%w: the offset dropped a corner arc`, decaderr.ErrUnsupported)
		}
		arc, ok := generated.Segments[idx].(sectionrecord.ArcSeg)
		if !ok {
			return fmt.Errorf(`%w: the offset corner is not an arc`, decaderr.ErrUnsupported)
		}
		before, after := joins[corner].before, joins[corner].after
		if sense > 0 {
			before, after = after, before
		}
		v := walks[corner]
		center := thickenExactOffset(v.StartU, v.StartV, 0, 0, t)
		if !thickenPointIsExact(arc.Center, center) ||
			!thickenPointIsExact(arc.Start, before) || !thickenPointIsExact(arc.End, after) ||
			(sense < 0 && (arc.TStart != 0 || arc.TEnd != 1)) ||
			(sense > 0 && (arc.TStart != 1 || arc.TEnd != 0)) {
			return fmt.Errorf(`%w: a generated corner arc coordinate is rounded`, decaderr.ErrUnsupported)
		}
		idx++
	}
	if idx != len(generated.Segments) {
		return fmt.Errorf(`%w: the offset added an unexpected feature`, decaderr.ErrUnsupported)
	}
	return nil
}

func thickenExactOffset(u, v float64, du, dv int, t *big.Rat) thickenExactPoint {
	return thickenExactPoint{
		u: new(big.Rat).Add(proofarith.FloatRat(u), new(big.Rat).Mul(big.NewRat(int64(du), 1), t)),
		v: new(big.Rat).Add(proofarith.FloatRat(v), new(big.Rat).Mul(big.NewRat(int64(dv), 1), t)),
	}
}

func thickenPointIsExact(got sectionrecord.Point2, want thickenExactPoint) bool {
	return proofarith.RationalFloatError(want.u, got.U) == 0 && proofarith.RationalFloatError(want.v, got.V) == 0
}
