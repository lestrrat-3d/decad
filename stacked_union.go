package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is docs/general-boolean-design.md §3's class A1: a Union of two
// co-directional prisms (or stacked prisms) whose sweep intervals overlap or
// touch without being equal. The distinct levels of both operands, compared
// exactly, cut the union into slabs. A slab one operand reaches takes that
// operand's region verbatim. A slab both reach takes the select-all merge of
// prism-boolean §4.2, or the containing operand's region verbatim when the
// clean-nesting match proves one region holds the other's whole. For
// a stackedPrismPayload, each differing interface uses that match: the
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

// stackedUnionOperand is one operand of the stacked union read as slabs: a
// prism is one slab over its own interval; a stacked prism is its own slabs.
// proxy carries the frame, placement and section displacement the gates read.
type stackedUnionOperand struct {
	proxy prismPayload
	slabs []stackedrecord.Slab
	// brep marks an A1 result read through its stack: its slabs may hold
	// several regions, and only the brep build arranges them.
	brep bool
}

func stackedUnionOperandOf(b *Body) (stackedUnionOperand, bool) {
	switch p := b.n.Payload.(type) {
	case prismPayload:
		return stackedUnionOperand{proxy: p, slabs: []stackedrecord.Slab{{
			Regions: []profileRecord{p.Profile},
			Z0:      p.Z0, Z1: p.Z1, Z0Delta: p.Z0Delta, Z1Delta: p.Z1Delta,
		}}}, true
	case stackedPrismPayload:
		return stackedUnionOperand{proxy: p.OuterPrism(), slabs: p.Slabs}, true
	case brepPayload:
		// An A1 result keeps the slabs it was built from; a class-B result
		// keeps none and is no operand here.
		if p.Stack == nil || len(p.Stack.Slabs) == 0 {
			return stackedUnionOperand{}, false
		}
		first, last := p.Stack.Slabs[0], p.Stack.Slabs[len(p.Stack.Slabs)-1]
		return stackedUnionOperand{proxy: prismrecord.Payload{
			Profile: profileRecord{Outer: first.Regions[0].Outer},
			Frame:   p.Faces[0].Frame, Xform: p.Xform, SectionDelta: p.Stack.Delta,
			Z0: first.Z0, Z0Delta: first.Z0Delta, Z1: last.Z1, Z1Delta: last.Z1Delta,
		}, slabs: p.Stack.Slabs, brep: true}, true
	default:
		return stackedUnionOperand{}, false
	}
}

// tryStackedUnion is general-boolean §3 A1. ok=false with a nil error is a
// silent miss: the caller takes the mesh path. A non-nil error is a genuine
// refusal the caller propagates (prism-boolean §3.4). The payload is a
// stackedPrismPayload when brepOnly is false and every interface matches.
// Direct BRep routing or an interface miss can produce a brepPayload.
func tryStackedUnion(ctx context.Context, a, b *Body) (topo.Payload, bool, error) {
	va, aok := stackedUnionOperandOf(a)
	vb, bok := stackedUnionOperandOf(b)
	if !aok || !bok {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	// G1-G4 on the outer prisms; G4, G6 and the trimmed-circular refusal on
	// every region, since a stacked operand's proxy names its first slab only.
	if _, _, ok, err := admitPrismPairBudget(budget, meshbool.OpUnion,
		bornBody(&Body{n: topo.Body{Payload: va.proxy}}), bornBody(&Body{n: topo.Body{Payload: vb.proxy}})); err != nil || !ok {
		return nil, false, err
	}
	// A brep operand, or a slab holding several regions, is the brep build's
	// alone: the stacked record states one region per slab.
	brepOnly := va.brep || vb.brep
	for _, op := range []stackedUnionOperand{va, vb} {
		for _, slab := range op.slabs {
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
	shift := prismplacement.ZShift(va.proxy.PlacementOperand(), vb.proxy.PlacementOperand())
	levels, ok := stackedrecord.UnionLevels(va.slabs, vb.slabs, shift)
	if !ok || len(levels) < 3 {
		// Equal intervals are prism-boolean §3.2's Union row, not a stack.
		return nil, false, nil
	}
	a0 := proofarith.FloatRat(va.slabs[0].Z0)
	a1 := proofarith.FloatRat(va.slabs[len(va.slabs)-1].Z1)
	b0 := proofarith.FloatRat(vb.slabs[0].Z0)
	b1 := proofarith.FloatRat(vb.slabs[len(vb.slabs)-1].Z1)
	b0.Add(b0, shift)
	b1.Add(b1, shift)
	if b0.Cmp(a1) > 0 || a0.Cmp(b1) > 0 { // G5 for a stacked union: the intervals overlap or touch
		return nil, false, nil
	}
	reexpress, err := prismcells.NewReexpression(va.proxy.PlacementOperand(), vb.proxy.PlacementOperand())
	if err != nil {
		return nil, false, err
	}

	return stackedunion.Join(ctx, budget, stackedunion.OperandOf(va.proxy, va.slabs),
		stackedunion.OperandOf(vb.proxy, vb.slabs), reexpress, levels, shift, brepOnly)
}
