package prismcells

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// ClassifySplit uses Classify's orientation and
// propagation rule for the target alone. A sheet tool has no material side;
// its edges only divide cells. In particular a circular tool may leave a
// holed target cell, so every published loop participates in the link map.
// A span sketch resolved between a tool entity and a target entity on one
// carrier (CoincidentEdges) is the target's boundary even where it is named
// under the tool's entity: the cell walking it takes its target side directly
// from the target entity it shares, and nothing propagates across it.
func ClassifySplit(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile) ([]bool, error) {
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	type occurrence struct {
		cell int
		isB  bool
		both bool
	}
	member := make([]membership, len(profiles))
	occ := map[edgeKey][]occurrence{}
	coincident, ok, err := CoincidentEdges(budget, tags, profiles)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf(`%w: a span the tool shares with the target cannot be read`, decaderr.ErrUnsupported)
	}
	setTarget := func(i int, match bool) error {
		if member[i].known && member[i].val != match {
			return fmt.Errorf(`%w: a split cell has conflicting target-side labels`, decaderr.ErrUnsupported)
		}
		member[i] = membership{known: true, val: match}
		return nil
	}
	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if !p.Valid {
			return nil, fmt.Errorf(`%w: a cell the split resolution depends on is invalid`, decaderr.ErrUnsupported)
		}
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, edge := range loop {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				origin, ok := tags[edge.Entity]
				if !ok {
					return nil, fmt.Errorf(`%w: a split cell edge traces to neither operand`, decaderr.ErrUnsupported)
				}
				if !origin.IsB {
					if err := setTarget(i, edge.Reversed == origin.AuthoredReversed); err != nil {
						return nil, err
					}
				}
				c, shared := coincident.Lookup(edge.Entity, edge.TStart, edge.TEnd)
				if shared {
					partner, ok := tags[c.Partner]
					if !ok || partner.IsB == origin.IsB {
						return nil, fmt.Errorf(`%w: a shared split span traces to one operand`, decaderr.ErrUnsupported)
					}
					if !partner.IsB {
						if err := setTarget(i, (edge.Reversed != c.Opposite) == partner.AuthoredReversed); err != nil {
							return nil, err
						}
					}
				}
				key := edgeKey{entity: edge.Entity, t0: edge.TStart, t1: edge.TEnd}
				occ[key] = append(occ[key], occurrence{cell: i, isB: origin.IsB, both: shared})
			}
		}
	}
	var links []link
	for _, uses := range occ {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		switch len(uses) {
		case 1:
		case 2:
			if uses[0].cell == uses[1].cell || uses[0].isB != uses[1].isB {
				return nil, fmt.Errorf(`%w: a split edge has inconsistent cell ownership`, decaderr.ErrUnsupported)
			}
			links = append(links, link{a: uses[0].cell, b: uses[1].cell, isB: uses[0].isB, both: uses[0].both})
		default:
			return nil, fmt.Errorf(`%w: a split edge belongs to more than two cells`, decaderr.ErrUnsupported)
		}
	}
	if err := propagate(budget, links, true, member); err != nil {
		return nil, err
	}
	for _, link := range links {
		if link.isB && !link.both && member[link.a].known && member[link.b].known && member[link.a].val != member[link.b].val {
			return nil, fmt.Errorf(`%w: adjacent split cells disagree on the target side of a tool edge`, decaderr.ErrUnsupported)
		}
	}
	matter := make([]bool, len(profiles))
	for i, m := range member {
		if !m.known {
			return nil, fmt.Errorf(`%w: a split cell has no target-side label`, decaderr.ErrUnsupported)
		}
		matter[i] = m.val
	}
	return matter, nil
}
