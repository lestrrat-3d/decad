package prismboolean

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is agent-docs/general-boolean-design.md §3's class A1: a Union of two
// co-directional prisms (or stacked prisms) whose sweep intervals overlap or
// touch without being equal. The distinct levels of both operands, compared
// exactly, cut the union into slabs. A slab one operand reaches takes that
// operand's region verbatim. A slab both reach takes the select-all merge of
// prism-boolean §4.2, or the containing operand's region verbatim when the
// clean-nesting match proves one region holds the other's whole. For
// a stackedrecord.Payload, each differing interface uses that match: the
// narrower outer must come back whole as a hole of the wider region's
// cell, whose exposed record is that wider region with the hole.
//
// Admission is reject-only and silent up to the first private scene: G1-G4
// on the two operands' outer prisms, G4 and G6 (hole-free) on every slab
// region, prism-boolean's trimmed-circular refusal, and the interval relation
// `z0_b' <= z1_a && z0_a <= z1_b'` over big.Rat. Past that point an
// unresolved topology still falls back to the mesh path, as prism-boolean
// §4.4 states: a disjoint overlap, and a cell the match cannot find.
// When a stacked record cannot state an interface, or an operand requires
// a BRep, internal/stackedunion/brep.go rebuilds result slabs from the
// admitted operand slabs and classifies their exposed interfaces. A pair
// that build does not cover takes the mesh path.

// TryStackedUnion is general-boolean §3 A1. The payload is a
// stackedrecord.Payload when brepOnly is false and every interface matches.
// Direct BRep routing or an interface miss can produce a breprecord.Payload.
func TryStackedUnion(ctx context.Context, a, b topo.Payload) (topo.Payload, bool, error) {
	va, aok := stackedunion.OperandFromPayload(a)
	vb, bok := stackedunion.OperandFromPayload(b)
	if !aok || !bok {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	// G1-G4 on the outer prisms; G4, G6 and the trimmed-circular refusal on
	// every region, since a stacked operand's proxy names its first slab only.
	if _, _, ok, err := prismcells.AdmitPairBudget(budget, meshbool.OpUnion, va.Proxy, vb.Proxy); err != nil || !ok {
		return nil, false, err
	}
	// A brep operand, or a slab holding several regions, is the brep build's
	// alone: the stacked record states one region per slab.
	brepOnly := va.Brep || vb.Brep
	for _, op := range []stackedunion.Operand{va, vb} {
		for _, slab := range op.Slabs {
			if len(slab.Regions) == 0 {
				return nil, false, nil
			}
			if len(slab.Regions) != 1 {
				brepOnly = true
			}
			for _, region := range slab.Regions {
				if len(region.Holes) != 0 { // G6
					return nil, false, nil
				}
				analytic, err := prismcells.ProfileAnalytic(budget, region) // G4
				if err != nil || !analytic {
					return nil, false, err
				}
				trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, region.Outer, region.Holes)
				if err != nil || trimmed {
					return nil, false, err
				}
			}
		}
	}
	shift := prismplacement.ZShift(va.Proxy.PlacementOperand(), vb.Proxy.PlacementOperand())
	levels, ok := stackedrecord.UnionLevels(va.Slabs, vb.Slabs, shift)
	if !ok || len(levels) < 3 {
		// Equal intervals are prism-boolean §3.2's Union row, not a stack.
		return nil, false, nil
	}
	a0 := proofarith.FloatRat(va.Slabs[0].Z0)
	a1 := proofarith.FloatRat(va.Slabs[len(va.Slabs)-1].Z1)
	b0 := proofarith.FloatRat(vb.Slabs[0].Z0)
	b1 := proofarith.FloatRat(vb.Slabs[len(vb.Slabs)-1].Z1)
	b0.Add(b0, shift)
	b1.Add(b1, shift)
	if b0.Cmp(a1) > 0 || a0.Cmp(b1) > 0 { // G5 for a stacked union: the intervals overlap or touch
		return nil, false, nil
	}
	reexpress, err := prismcells.NewReexpression(va.Proxy.PlacementOperand(), vb.Proxy.PlacementOperand())
	if err != nil {
		return nil, false, err
	}

	return stackedunion.Join(ctx, budget, va, vb, reexpress, levels, shift, brepOnly)
}
