package stackedrecord

import (
	"context"
	"fmt"
	"math"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Slab is one recorded section over one axial interval.
type Slab struct {
	Regions                  []momentinput.Profile
	Z0, Z1, Z0Delta, Z1Delta float64
}

// Interface records the exposed patches on either side of a slab boundary.
type Interface struct {
	LowerExposed, UpperExposed []momentinput.Profile
}

// Record is the section and interface record checked before topology builds.
type Record struct {
	Slabs      []Slab
	Interfaces []Interface
}

// OuterRuns groups consecutive slabs with the same outer loop. A prism group
// contributes one run per disjoint region. Each run keeps only its outer loop
// because bounds and geometric caps read the body's outer envelope.
func OuterRuns(slabs []Slab) []Slab {
	var runs []Slab
	if len(slabs) == 1 && len(slabs[0].Regions) >= 2 {
		for _, region := range slabs[0].Regions {
			run := slabs[0]
			run.Regions = []momentinput.Profile{{Outer: region.Outer}}
			runs = append(runs, run)
		}
		return runs
	}
	for k, slab := range slabs {
		if k > 0 && equalLoop(slabs[k-1].Regions[0].Outer, slab.Regions[0].Outer) {
			last := &runs[len(runs)-1]
			last.Z1, last.Z1Delta = slab.Z1, slab.Z1Delta
			continue
		}
		slab.Regions = []momentinput.Profile{{Outer: slab.Regions[0].Outer}}
		runs = append(runs, slab)
	}
	return runs
}

func (sp Record) isGroup() bool {
	return len(sp.Slabs) == 1 && len(sp.Slabs[0].Regions) >= 2
}

func equalLoop(a, b sectionrecord.LoopRecord) bool {
	return reflect.DeepEqual(a, b)
}

// Derive derives every interface's exposed records from the slabs
// alone, so a rewrite that moves or rebuilds the slab regions (the mirror
// join, a pattern instance) re-derives its interfaces here rather than moving
// the old ones, and Falsify's I7 holds by construction and is
// still checked. Where the two outers match, each exclusive hole of one side,
// reversed, is the material the other side exposes. Where they differ (§2.2's
// union reading), the one patch is the wider region with the narrower outer
// reversed as its hole, on the side prior records: which side is wider is a
// fact of the stack's construction, and a rigid motion of both regions keeps
// it.
func Derive(ctx context.Context, slabs []Slab, prior []Interface) ([]Interface, error) {
	out := make([]Interface, len(slabs)-1)
	for i := range out {
		if wideLower, lining := LiningSides(slabs[i], slabs[i+1]); lining {
			narrow := slabs[i].Regions
			if wideLower {
				narrow = slabs[i+1].Regions
			}
			exposed, err := liningExposed(ctx, narrow)
			if err != nil {
				return nil, err
			}
			if wideLower {
				out[i] = Interface{LowerExposed: exposed}
			} else {
				out[i] = Interface{UpperExposed: exposed}
			}
			continue
		}
		lowerRegion, upperRegion := slabs[i].Regions[0], slabs[i+1].Regions[0]
		same := equalLoop(lowerRegion.Outer, upperRegion.Outer)
		if !same {
			if i >= len(prior) {
				return nil, fmt.Errorf(`%w: interface %d changes the outer loop and has no prior record of its exposed side`, decaderr.ErrDegenerate, i)
			}
			if len(prior[i].LowerExposed) == 1 && len(prior[i].UpperExposed) == 0 {
				lower, err := UnionExposed(ctx, lowerRegion, upperRegion)
				if err != nil {
					return nil, err
				}
				out[i] = Interface{LowerExposed: lower}
				continue
			}
			upper, err := UnionExposed(ctx, upperRegion, lowerRegion)
			if err != nil {
				return nil, err
			}
			out[i] = Interface{UpperExposed: upper}
			continue
		}
		lowerOnly, upperOnly := ExclusiveHoles(lowerRegion, upperRegion)
		lower, err := Exposed(ctx, upperOnly)
		if err != nil {
			return nil, err
		}
		upper, err := Exposed(ctx, lowerOnly)
		if err != nil {
			return nil, err
		}
		out[i] = Interface{LowerExposed: lower, UpperExposed: upper}
	}
	return out, nil
}

// falsifyPrismGroup is §2.2's I1/I2 reading for a prism group: one slab with
// two or more non-empty regions over a finite interval, and no interface.
// That the regions are pairwise disjoint is proven when the group is built,
// by sketch's arrangement of every region's outer (prismcells.ProveGroupDisjoint);
// this audit compares records and proves no geometry.
func falsifyPrismGroup(ctx context.Context, sp Record) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(sp.Interfaces) != 0 {
		return fmt.Errorf(`%w: a prism group has one slab and no interface`, decaderr.ErrUnsupported)
	}
	slab := sp.Slabs[0]
	if math.IsNaN(slab.Z0) || math.IsNaN(slab.Z1) ||
		math.IsInf(slab.Z0, 0) || math.IsInf(slab.Z1, 0) || slab.Z0 >= slab.Z1 {
		return fmt.Errorf(`%w: slab 0 has an empty interval`, decaderr.ErrDegenerate)
	}
	for r, region := range slab.Regions {
		if len(region.Outer.Segments) == 0 {
			return fmt.Errorf(`%w: region %d of the prism group has no outer loop`, decaderr.ErrDegenerate, r)
		}
	}
	return nil
}

// Falsify refuses a slab record whose interfaces cannot be built from its loops.
func Falsify(ctx context.Context, sp Record) error {
	if sp.isGroup() {
		return falsifyPrismGroup(ctx, sp)
	}
	if len(sp.Slabs) < 2 || len(sp.Interfaces) != len(sp.Slabs)-1 {
		return fmt.Errorf(`%w: a stacked prism needs two slabs and one interface between each pair`, decaderr.ErrUnsupported)
	}
	for i, slab := range sp.Slabs {
		if err := ctx.Err(); err != nil {
			return err
		}
		if len(slab.Regions) == 0 {
			return fmt.Errorf(`%w: slab %d has no region`, decaderr.ErrUnsupported, i)
		}
		if math.IsNaN(slab.Z0) || math.IsNaN(slab.Z1) ||
			math.IsInf(slab.Z0, 0) || math.IsInf(slab.Z1, 0) || slab.Z0 >= slab.Z1 {
			return fmt.Errorf(`%w: slab %d has an empty interval`, decaderr.ErrDegenerate, i)
		}
		if i == 0 {
			continue
		}
		prev := sp.Slabs[i-1]
		if prev.Z1 != slab.Z0 || prev.Z1Delta != slab.Z0Delta {
			return fmt.Errorf(`%w: slabs %d and %d do not share one level`, decaderr.ErrDegenerate, i-1, i)
		}
		if len(prev.Regions) != 1 || len(slab.Regions) != 1 {
			// I2: a slab holds several regions only as the narrow side of a
			// lining interface (§2.2's lining reading).
			if err := falsifyLiningInterface(ctx, prev, slab, sp.Interfaces[i-1], i-1); err != nil {
				return err
			}
			continue
		}
		equal := equalLoop(prev.Regions[0].Outer, slab.Regions[0].Outer)
		if !equal {
			if err := falsifyUnionInterface(ctx, prev.Regions[0], slab.Regions[0], sp.Interfaces[i-1], i-1); err != nil {
				return err
			}
			continue
		}
		lowerOnly, upperOnly := ExclusiveHoles(prev.Regions[0], slab.Regions[0])
		if len(lowerOnly) != 0 && len(upperOnly) != 0 {
			return fmt.Errorf(`%w: both sides of interface %d have exclusive holes`, decaderr.ErrUnsupported, i-1)
		}
		got := sp.Interfaces[i-1]
		lowerOK, err := exposedMatches(ctx, got.LowerExposed, upperOnly)
		if err != nil {
			return err
		}
		upperOK, err := exposedMatches(ctx, got.UpperExposed, lowerOnly)
		if err != nil {
			return err
		}
		if !lowerOK || !upperOK {
			return fmt.Errorf(`%w: interface %d does not record its exposed material`, decaderr.ErrDegenerate, i-1)
		}
	}
	return nil
}

// exposedMatches is I7 for a monotone interface: got holds exactly one
// hole-free record per exclusive hole, in order, each one's outer the hole's
// interior. A record states that interior either as reverse(hole) — the
// spelling Exposed derives — or as the loop whose reverse is the hole,
// which is how a shell records the offset loop it built the hole from
// (loopReversesRecord).
func exposedMatches(ctx context.Context, got []momentinput.Profile, holes []sectionrecord.LoopRecord) (bool, error) {
	if len(got) != len(holes) {
		return false, nil
	}
	for e, hole := range holes {
		if len(got[e].Holes) != 0 {
			return false, nil
		}
		ok, err := loopReversesRecord(ctx, got[e].Outer, hole)
		if err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}

// loopReversesRecord reports whether loop a is loop b walked the other way,
// as records: a equals reverse(b), or reverse(a) equals b. Reversal rebuilds
// each segment from its walk, so the two spellings need not agree bit for bit,
// and either one names the same loop.
func loopReversesRecord(ctx context.Context, a, b sectionrecord.LoopRecord) (bool, error) {
	rb, err := offset2d.ReverseLoopRecordContext(ctx, b)
	if err != nil {
		return false, err
	}
	if equalLoop(a, rb) {
		return true, nil
	}
	ra, err := offset2d.ReverseLoopRecordContext(ctx, a)
	if err != nil {
		return false, err
	}
	return equalLoop(ra, b), nil
}

// LiningSides reports whether the interface between lower and upper
// takes §2.2's lining reading — one side holds several regions — and, if so,
// whether the lower side is the wide one (the side holding one region).
func LiningSides(lower, upper Slab) (bool, bool) {
	switch {
	case len(lower.Regions) == 1 && len(upper.Regions) > 1:
		return true, true
	case len(upper.Regions) == 1 && len(lower.Regions) > 1:
		return false, true
	default:
		return false, false
	}
}

// liningExposed derives the one exposed record of a lining interface
// from its narrow regions: the wide region's material they do not cover. Its outer is
// the reverse of the first narrow region's one hole, and its holes are the
// reverses of every other narrow region's outer, in order.
func liningExposed(ctx context.Context, narrow []momentinput.Profile) ([]momentinput.Profile, error) {
	if len(narrow) == 0 || len(narrow[0].Holes) != 1 {
		return nil, fmt.Errorf(`%w: a lining interface's first narrow region has no single hole`, decaderr.ErrUnsupported)
	}
	outer, err := offset2d.ReverseLoopRecordContext(ctx, narrow[0].Holes[0])
	if err != nil {
		return nil, err
	}
	record := momentinput.Profile{Outer: outer}
	for _, region := range narrow[1:] {
		hole, err := offset2d.ReverseLoopRecordContext(ctx, region.Outer)
		if err != nil {
			return nil, err
		}
		record.Holes = append(record.Holes, hole)
	}
	return []momentinput.Profile{record}, nil
}

// falsifyLiningInterface is §2.2's lining reading of I5-I7, for an
// interface where one side holds several regions. The wide side holds one
// region W with k holes; the narrow side holds 1 + k regions: the first
// shares W's outer and has one hole of its own, and region m >= 1 has W's
// hole m-1 as its one hole and an outer of its own. Only the wide side
// records exposed material, one record: the first narrow region's hole
// reversed as its outer and every other narrow region's outer reversed as
// its holes (loopReversesRecord's either spelling). That each narrow region
// lies in W, and that the narrow regions are pairwise disjoint, is proven
// when the shell builds them, by the offset section's §5 audit; this audit
// compares records and proves no geometry.
func falsifyLiningInterface(ctx context.Context, lower, upper Slab, boundary Interface, index int) error {
	wideLower, lining := LiningSides(lower, upper)
	if !lining {
		return fmt.Errorf(`%w: both sides of interface %d hold several regions`, decaderr.ErrUnsupported, index)
	}
	wide, narrow := upper.Regions[0], lower.Regions
	got, other := boundary.UpperExposed, boundary.LowerExposed
	if wideLower {
		wide, narrow = lower.Regions[0], upper.Regions
		got, other = boundary.LowerExposed, boundary.UpperExposed
	}
	if len(narrow) != 1+len(wide.Holes) || len(narrow[0].Holes) != 1 {
		return fmt.Errorf(`%w: interface %d does not line each loop of its wide region`, decaderr.ErrUnsupported, index)
	}
	same := equalLoop(narrow[0].Outer, wide.Outer)
	if !same {
		return fmt.Errorf(`%w: interface %d's first narrow region does not share the wide outer`, decaderr.ErrUnsupported, index)
	}
	for m, region := range narrow[1:] {
		if len(region.Holes) != 1 {
			return fmt.Errorf(`%w: interface %d's narrow region %d has no single hole`, decaderr.ErrUnsupported, index, m+1)
		}
		same := equalLoop(region.Holes[0], wide.Holes[m])
		if !same {
			return fmt.Errorf(`%w: interface %d's narrow region %d does not line wide hole %d`, decaderr.ErrUnsupported, index, m+1, m)
		}
	}
	if len(other) != 0 || len(got) != 1 || len(got[0].Holes) != len(narrow)-1 {
		return fmt.Errorf(`%w: interface %d does not record its exposed material`, decaderr.ErrDegenerate, index)
	}
	ok, err := loopReversesRecord(ctx, got[0].Outer, narrow[0].Holes[0])
	if err != nil {
		return err
	}
	for m := 1; ok && m < len(narrow); m++ {
		ok, err = loopReversesRecord(ctx, got[0].Holes[m-1], narrow[m].Outer)
		if err != nil {
			return err
		}
	}
	if !ok {
		return fmt.Errorf(`%w: interface %d does not record its exposed material`, decaderr.ErrDegenerate, index)
	}
	return nil
}

// falsifyUnionInterface is §2.2's union reading of I5-I7 for an
// interface whose two outer loops differ. Both regions must be hole-free, and
// exactly one side records exactly one exposed patch: the wider region with
// the narrower outer reversed as its hole (UnionExposed). The audit
// re-derives that record from the two regions and the recorded side and
// compares it record for record. That the narrower outer lies inside the
// wider one is proven at construction by the clean-nesting match, as I6's
// monotone holes are; this audit compares records and proves no geometry.
func falsifyUnionInterface(ctx context.Context, lower, upper momentinput.Profile, boundary Interface, index int) error {
	if len(lower.Holes) != 0 || len(upper.Holes) != 0 {
		return fmt.Errorf(`%w: interface %d changes the outer loop between holed regions`, decaderr.ErrUnsupported, index)
	}
	var got []momentinput.Profile
	var want []momentinput.Profile
	var err error
	switch {
	case len(boundary.LowerExposed) == 1 && len(boundary.UpperExposed) == 0:
		got = boundary.LowerExposed
		want, err = UnionExposed(ctx, lower, upper)
	case len(boundary.UpperExposed) == 1 && len(boundary.LowerExposed) == 0:
		got = boundary.UpperExposed
		want, err = UnionExposed(ctx, upper, lower)
	default:
		return fmt.Errorf(`%w: interface %d changes the outer loop without exactly one exposed patch`, decaderr.ErrDegenerate, index)
	}
	if err != nil {
		return err
	}
	if !reflect.DeepEqual(got, want) {
		return fmt.Errorf(`%w: interface %d does not record its exposed material`, decaderr.ErrDegenerate, index)
	}
	return nil
}
