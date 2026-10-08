package decad

import (
	"context"
	"fmt"
	"math"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A slab is one constant section over an axial interval. Its one region is
// kept in a slice so the record can later admit disjoint sections per slab.
type prismSlab struct {
	regions          []ProfileRecord
	z0, z1           float64
	z0Delta, z1Delta float64
}

type prismSlabInterface struct {
	lowerExposed []ProfileRecord
	upperExposed []ProfileRecord
}

type stackedPrismPayload struct {
	slabs        []prismSlab
	interfaces   []prismSlabInterface
	frame        r3.Frame
	xform        r3.Transform
	sectionDelta float64
}

func (sp stackedPrismPayload) transform() r3.Transform { return sp.xform }

func (sp stackedPrismPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	sp.xform = composed
	return evalStackedContext(ctx, d, ref, sp)
}

func (sp stackedPrismPayload) axialDelta() float64 {
	var delta float64
	for _, slab := range sp.slabs {
		delta = max(delta, slab.z0Delta, slab.z1Delta)
	}
	return delta
}

func (sp stackedPrismPayload) outerPrism() prismPayload {
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	return prismPayload{
		profile: ProfileRecord{Outer: first.regions[0].Outer},
		frame:   sp.frame, xform: sp.xform, sectionDelta: sp.sectionDelta,
		z0: first.z0, z0Delta: first.z0Delta,
		z1: last.z1, z1Delta: last.z1Delta,
	}
}

// outerRuns is one outer-only prism per maximal run of consecutive slabs that
// carry one outer loop record, over that run's own interval. A cut-built stack
// is one run, the outer-only prism on the full interval. A union-built stack
// has one run per distinct outer, and every run's prism is material of the
// body, since a union-built slab region is hole-free (§2.2's union reading).
// The union of the runs' prisms is the body's outer envelope, which is what
// Bounds, extentAlong, the centroid's geometric cap and the tolerance gate's
// witnesses read.
func (sp stackedPrismPayload) outerRuns() ([]prismPayload, error) {
	base := sp.outerPrism()
	var runs []prismPayload
	if sp.isGroup() {
		// A prism group's regions are disjoint lumps over one interval: one
		// run per region.
		slab := sp.slabs[0]
		for _, region := range slab.regions {
			run := base
			run.profile = ProfileRecord{Outer: region.Outer}
			runs = append(runs, run)
		}
		return runs, nil
	}
	for k, slab := range sp.slabs {
		if k > 0 {
			same, err := loopRecordsEqual(nil, sp.slabs[k-1].regions[0].Outer, slab.regions[0].Outer)
			if err != nil {
				return nil, err
			}
			if same {
				last := &runs[len(runs)-1]
				last.z1, last.z1Delta = slab.z1, slab.z1Delta
				continue
			}
		}
		run := base
		run.profile = ProfileRecord{Outer: slab.regions[0].Outer}
		run.z0, run.z0Delta = slab.z0, slab.z0Delta
		run.z1, run.z1Delta = slab.z1, slab.z1Delta
		runs = append(runs, run)
	}
	return runs, nil
}

func (sp stackedPrismPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	runs, err := sp.outerRuns()
	if err != nil {
		return 0, 0, 0, err
	}
	var lo, hi, bound float64
	for i, run := range runs {
		rlo, rhi, rbound, err := run.extentAlong(g)
		if err != nil {
			return 0, 0, 0, err
		}
		if i == 0 {
			lo, hi, bound = rlo, rhi, rbound
			continue
		}
		lo, hi, bound = min(lo, rlo), max(hi, rhi), max(bound, rbound)
	}
	return lo, hi, bound, nil
}

// stackedBoundsContext is the union of every outer run's box. Each run's box
// bounds its own extremes, so the union's extreme along each axis is one
// run's, and the largest run bound covers it. outerDelta is the largest
// column displacement any run's outer loop carries beyond sectionDelta
// (stackedPlan.columnDelta), charged to every run as its section displacement.
func stackedBoundsContext(ctx context.Context, sp stackedPrismPayload, outerDelta float64, work *freeform.FreeformWork) (Box, error) {
	runs, err := sp.outerRuns()
	if err != nil {
		return Box{}, err
	}
	var out Box
	bound := 0.0
	for i, run := range runs {
		if outerDelta > 0 {
			run.sectionDelta = proofbound.AbsSumUpper(run.sectionDelta, outerDelta)
		}
		box, err := prismBoundsContext(ctx, run, work, nil)
		if err != nil {
			return Box{}, err
		}
		bound = max(bound, box.Bound.Base())
		if i == 0 {
			out = box
			continue
		}
		out.Min = r3.NewVec(min(out.Min.X, box.Min.X), min(out.Min.Y, box.Min.Y), min(out.Min.Z, box.Min.Z))
		out.Max = r3.NewVec(max(out.Max.X, box.Max.X), max(out.Max.Y, box.Max.Y), max(out.Max.Z, box.Max.Z))
	}
	out.Exactness = exactnessOf(bound)
	out.Bound = units.Millimeters(bound)
	return out, nil
}

// stackedExclusiveHoles derives the holes present on only one side of an
// interface. Admission proves their nesting; this function compares records.
func stackedExclusiveHoles(lower, upper ProfileRecord) (lowerOnly, upperOnly []LoopRecord, err error) {
	for _, hole := range lower.Holes {
		found := false
		for _, other := range upper.Holes {
			equal, e := loopRecordsEqual(nil, hole, other)
			if e != nil {
				return nil, nil, e
			}
			if equal {
				found = true
				break
			}
		}
		if !found {
			lowerOnly = append(lowerOnly, hole)
		}
	}
	for _, hole := range upper.Holes {
		found := false
		for _, other := range lower.Holes {
			equal, e := loopRecordsEqual(nil, hole, other)
			if e != nil {
				return nil, nil, e
			}
			if equal {
				found = true
				break
			}
		}
		if !found {
			upperOnly = append(upperOnly, hole)
		}
	}
	return lowerOnly, upperOnly, nil
}

func stackedExposed(ctx context.Context, holes []LoopRecord) ([]ProfileRecord, error) {
	out := make([]ProfileRecord, 0, len(holes))
	for _, hole := range holes {
		reversed, err := reverseLoopRecordContext(ctx, hole)
		if err != nil {
			return nil, err
		}
		out = append(out, ProfileRecord{Outer: reversed})
	}
	return out, nil
}

// stackedInterfaces derives every interface's exposed records from the slabs
// alone, so a rewrite that moves or rebuilds the slab regions (the mirror
// join, a pattern instance) re-derives its interfaces here rather than moving
// the old ones, and falsifyStackedPayload's I7 holds by construction and is
// still checked. Where the two outers match, each exclusive hole of one side,
// reversed, is the material the other side exposes. Where they differ (§2.2's
// union reading), the one patch is the wider region with the narrower outer
// reversed as its hole, on the side prior records: which side is wider is a
// fact of the stack's construction, and a rigid motion of both regions keeps
// it.
func stackedInterfaces(ctx context.Context, slabs []prismSlab, prior []prismSlabInterface) ([]prismSlabInterface, error) {
	out := make([]prismSlabInterface, len(slabs)-1)
	for i := range out {
		if wideLower, lining := stackedLiningSides(slabs[i], slabs[i+1]); lining {
			narrow := slabs[i].regions
			if wideLower {
				narrow = slabs[i+1].regions
			}
			exposed, err := stackedLiningExposed(ctx, narrow)
			if err != nil {
				return nil, err
			}
			if wideLower {
				out[i] = prismSlabInterface{lowerExposed: exposed}
			} else {
				out[i] = prismSlabInterface{upperExposed: exposed}
			}
			continue
		}
		lowerRegion, upperRegion := slabs[i].regions[0], slabs[i+1].regions[0]
		same, err := loopRecordsEqual(nil, lowerRegion.Outer, upperRegion.Outer)
		if err != nil {
			return nil, err
		}
		if !same {
			if i >= len(prior) {
				return nil, fmt.Errorf(`%w: interface %d changes the outer loop and has no prior record of its exposed side`, ErrDegenerate, i)
			}
			if len(prior[i].lowerExposed) == 1 && len(prior[i].upperExposed) == 0 {
				lower, err := stackedUnionExposed(ctx, lowerRegion, upperRegion)
				if err != nil {
					return nil, err
				}
				out[i] = prismSlabInterface{lowerExposed: lower}
				continue
			}
			upper, err := stackedUnionExposed(ctx, upperRegion, lowerRegion)
			if err != nil {
				return nil, err
			}
			out[i] = prismSlabInterface{upperExposed: upper}
			continue
		}
		lowerOnly, upperOnly, err := stackedExclusiveHoles(lowerRegion, upperRegion)
		if err != nil {
			return nil, err
		}
		lower, err := stackedExposed(ctx, upperOnly)
		if err != nil {
			return nil, err
		}
		upper, err := stackedExposed(ctx, lowerOnly)
		if err != nil {
			return nil, err
		}
		out[i] = prismSlabInterface{lowerExposed: lower, upperExposed: upper}
	}
	return out, nil
}

// falsifyStackedPayload refuses any record whose interfaces cannot be built
// from whole, shared loop columns. It is run before topology and chording.
// isGroup reports whether the payload is a prism group
// (docs/mirror-pattern-design.md §6.3): one slab holding two or more regions.
func (sp stackedPrismPayload) isGroup() bool {
	return len(sp.slabs) == 1 && len(sp.slabs[0].regions) >= 2
}

// falsifyPrismGroup is §2.2's I1/I2 reading for a prism group: one slab with
// two or more non-empty regions over a finite interval, and no interface.
// That the regions are pairwise disjoint is proven when the group is built,
// by sketch's arrangement of every region's outer (provePrismRegionsDisjoint);
// this audit compares records and proves no geometry.
func falsifyPrismGroup(ctx context.Context, sp stackedPrismPayload) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(sp.interfaces) != 0 {
		return fmt.Errorf(`%w: a prism group has one slab and no interface`, ErrUnsupported)
	}
	slab := sp.slabs[0]
	if math.IsNaN(slab.z0) || math.IsNaN(slab.z1) ||
		math.IsInf(slab.z0, 0) || math.IsInf(slab.z1, 0) || slab.z0 >= slab.z1 {
		return fmt.Errorf(`%w: slab 0 has an empty interval`, ErrDegenerate)
	}
	for r, region := range slab.regions {
		if len(region.Outer.Segments) == 0 {
			return fmt.Errorf(`%w: region %d of the prism group has no outer loop`, ErrDegenerate, r)
		}
	}
	return nil
}

func falsifyStackedPayload(ctx context.Context, sp stackedPrismPayload) error {
	if sp.isGroup() {
		return falsifyPrismGroup(ctx, sp)
	}
	if len(sp.slabs) < 2 || len(sp.interfaces) != len(sp.slabs)-1 {
		return fmt.Errorf(`%w: a stacked prism needs two slabs and one interface between each pair`, ErrUnsupported)
	}
	for i, slab := range sp.slabs {
		if err := ctx.Err(); err != nil {
			return err
		}
		if len(slab.regions) == 0 {
			return fmt.Errorf(`%w: slab %d has no region`, ErrUnsupported, i)
		}
		if math.IsNaN(slab.z0) || math.IsNaN(slab.z1) ||
			math.IsInf(slab.z0, 0) || math.IsInf(slab.z1, 0) || slab.z0 >= slab.z1 {
			return fmt.Errorf(`%w: slab %d has an empty interval`, ErrDegenerate, i)
		}
		if i == 0 {
			continue
		}
		prev := sp.slabs[i-1]
		if prev.z1 != slab.z0 || prev.z1Delta != slab.z0Delta {
			return fmt.Errorf(`%w: slabs %d and %d do not share one level`, ErrDegenerate, i-1, i)
		}
		if len(prev.regions) != 1 || len(slab.regions) != 1 {
			// I2: a slab holds several regions only as the narrow side of a
			// lining interface (§2.2's lining reading).
			if err := falsifyStackedLiningInterface(ctx, prev, slab, sp.interfaces[i-1], i-1); err != nil {
				return err
			}
			continue
		}
		equal, err := loopRecordsEqual(nil, prev.regions[0].Outer, slab.regions[0].Outer)
		if err != nil {
			return err
		}
		if !equal {
			if err := falsifyStackedUnionInterface(ctx, prev.regions[0], slab.regions[0], sp.interfaces[i-1], i-1); err != nil {
				return err
			}
			continue
		}
		lowerOnly, upperOnly, err := stackedExclusiveHoles(prev.regions[0], slab.regions[0])
		if err != nil {
			return err
		}
		if len(lowerOnly) != 0 && len(upperOnly) != 0 {
			return fmt.Errorf(`%w: both sides of interface %d have exclusive holes`, ErrUnsupported, i-1)
		}
		got := sp.interfaces[i-1]
		lowerOK, err := stackedExposedMatches(ctx, got.lowerExposed, upperOnly)
		if err != nil {
			return err
		}
		upperOK, err := stackedExposedMatches(ctx, got.upperExposed, lowerOnly)
		if err != nil {
			return err
		}
		if !lowerOK || !upperOK {
			return fmt.Errorf(`%w: interface %d does not record its exposed material`, ErrDegenerate, i-1)
		}
	}
	return nil
}

// stackedExposedMatches is I7 for a monotone interface: got holds exactly one
// hole-free record per exclusive hole, in order, each one's outer the hole's
// interior. A record states that interior either as reverse(hole) — the
// spelling stackedExposed derives — or as the loop whose reverse is the hole,
// which is how a shell records the offset loop it built the hole from
// (loopReversesRecord).
func stackedExposedMatches(ctx context.Context, got []ProfileRecord, holes []LoopRecord) (bool, error) {
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
func loopReversesRecord(ctx context.Context, a, b LoopRecord) (bool, error) {
	rb, err := reverseLoopRecordContext(ctx, b)
	if err != nil {
		return false, err
	}
	if same, err := loopRecordsEqual(nil, a, rb); err != nil || same {
		return same, err
	}
	ra, err := reverseLoopRecordContext(ctx, a)
	if err != nil {
		return false, err
	}
	return loopRecordsEqual(nil, ra, b)
}

// stackedLiningSides reports whether the interface between lower and upper
// takes §2.2's lining reading — one side holds several regions — and, if so,
// whether the lower side is the wide one (the side holding one region).
func stackedLiningSides(lower, upper prismSlab) (bool, bool) {
	switch {
	case len(lower.regions) == 1 && len(upper.regions) > 1:
		return true, true
	case len(upper.regions) == 1 && len(lower.regions) > 1:
		return false, true
	default:
		return false, false
	}
}

// stackedLiningExposed derives the one exposed record of a lining interface
// from its narrow regions: the wide region's material they do not cover. Its outer is
// the reverse of the first narrow region's one hole, and its holes are the
// reverses of every other narrow region's outer, in order.
func stackedLiningExposed(ctx context.Context, narrow []ProfileRecord) ([]ProfileRecord, error) {
	if len(narrow) == 0 || len(narrow[0].Holes) != 1 {
		return nil, fmt.Errorf(`%w: a lining interface's first narrow region has no single hole`, ErrUnsupported)
	}
	outer, err := reverseLoopRecordContext(ctx, narrow[0].Holes[0])
	if err != nil {
		return nil, err
	}
	record := ProfileRecord{Outer: outer}
	for _, region := range narrow[1:] {
		hole, err := reverseLoopRecordContext(ctx, region.Outer)
		if err != nil {
			return nil, err
		}
		record.Holes = append(record.Holes, hole)
	}
	return []ProfileRecord{record}, nil
}

// falsifyStackedLiningInterface is §2.2's lining reading of I5-I7, for an
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
func falsifyStackedLiningInterface(ctx context.Context, lower, upper prismSlab, boundary prismSlabInterface, index int) error {
	wideLower, lining := stackedLiningSides(lower, upper)
	if !lining {
		return fmt.Errorf(`%w: both sides of interface %d hold several regions`, ErrUnsupported, index)
	}
	wide, narrow := upper.regions[0], lower.regions
	got, other := boundary.upperExposed, boundary.lowerExposed
	if wideLower {
		wide, narrow = lower.regions[0], upper.regions
		got, other = boundary.lowerExposed, boundary.upperExposed
	}
	if len(narrow) != 1+len(wide.Holes) || len(narrow[0].Holes) != 1 {
		return fmt.Errorf(`%w: interface %d does not line each loop of its wide region`, ErrUnsupported, index)
	}
	same, err := loopRecordsEqual(nil, narrow[0].Outer, wide.Outer)
	if err != nil {
		return err
	}
	if !same {
		return fmt.Errorf(`%w: interface %d's first narrow region does not share the wide outer`, ErrUnsupported, index)
	}
	for m, region := range narrow[1:] {
		if len(region.Holes) != 1 {
			return fmt.Errorf(`%w: interface %d's narrow region %d has no single hole`, ErrUnsupported, index, m+1)
		}
		same, err := loopRecordsEqual(nil, region.Holes[0], wide.Holes[m])
		if err != nil {
			return err
		}
		if !same {
			return fmt.Errorf(`%w: interface %d's narrow region %d does not line wide hole %d`, ErrUnsupported, index, m+1, m)
		}
	}
	if len(other) != 0 || len(got) != 1 || len(got[0].Holes) != len(narrow)-1 {
		return fmt.Errorf(`%w: interface %d does not record its exposed material`, ErrDegenerate, index)
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
		return fmt.Errorf(`%w: interface %d does not record its exposed material`, ErrDegenerate, index)
	}
	return nil
}

// falsifyStackedUnionInterface is §2.2's union reading of I5-I7 for an
// interface whose two outer loops differ. Both regions must be hole-free, and
// exactly one side records exactly one exposed patch: the wider region with
// the narrower outer reversed as its hole (stackedUnionExposed). The audit
// re-derives that record from the two regions and the recorded side and
// compares it record for record. That the narrower outer lies inside the
// wider one is proven at construction by the clean-nesting match, as I6's
// monotone holes are; this audit compares records and proves no geometry.
func falsifyStackedUnionInterface(ctx context.Context, lower, upper ProfileRecord, boundary prismSlabInterface, index int) error {
	if len(lower.Holes) != 0 || len(upper.Holes) != 0 {
		return fmt.Errorf(`%w: interface %d changes the outer loop between holed regions`, ErrUnsupported, index)
	}
	var got []ProfileRecord
	var want []ProfileRecord
	var err error
	switch {
	case len(boundary.lowerExposed) == 1 && len(boundary.upperExposed) == 0:
		got = boundary.lowerExposed
		want, err = stackedUnionExposed(ctx, lower, upper)
	case len(boundary.upperExposed) == 1 && len(boundary.lowerExposed) == 0:
		got = boundary.upperExposed
		want, err = stackedUnionExposed(ctx, upper, lower)
	default:
		return fmt.Errorf(`%w: interface %d changes the outer loop without exactly one exposed patch`, ErrDegenerate, index)
	}
	if err != nil {
		return err
	}
	if !reflect.DeepEqual(got, want) {
		return fmt.Errorf(`%w: interface %d does not record its exposed material`, ErrDegenerate, index)
	}
	return nil
}

// stackedPatchLoop is one loop of a planar patch: the column whose ring it
// reads, which ring (the column's top or its bottom), and whether the loop is
// the patch's outer boundary.
type stackedPatchLoop struct {
	column int
	top    bool
	outer  bool
}

// stackedPatch is one exposed planar patch at an interface: a floor (material
// below only, outward +N) or a ceiling (material above only, outward -N).
type stackedPatch struct {
	role   string
	floor  bool
	record ProfileRecord
	loops  []stackedPatchLoop
}

// stackedInterfacePatches lists interface k's exposed patches with the column
// rings that bound them. Where the two outers match (§2.2's I5), each patch is
// one exclusive hole's column. Where they differ (the union reading), the one
// patch is bounded by the wider side's outer column and, as its hole, the
// narrower side's outer column. A lining interface's one patch is bounded by
// the first narrow region's hole column, as its outer, and by every other
// narrow region's outer column, as its holes. The body build and the
// tessellator both read this list, so both name the same face by the same
// role and the same rings.
func stackedInterfacePatches(sp stackedPrismPayload, columns []stackedColumn, bySlab [][]stackedSlabLoop, k int) ([]stackedPatch, error) {
	boundary := sp.interfaces[k]
	if wideLower, lining := stackedLiningSides(sp.slabs[k], sp.slabs[k+1]); lining {
		return stackedLiningPatches(sp, columns, bySlab, k, wideLower)
	}
	lower, upper := sp.slabs[k].regions[0], sp.slabs[k+1].regions[0]
	same, err := loopRecordsEqual(nil, lower.Outer, upper.Outer)
	if err != nil {
		return nil, err
	}
	var patches []stackedPatch
	if !same {
		lowerOuter, upperOuter := stackedSlabColumn(bySlab[k], 0, 0), stackedSlabColumn(bySlab[k+1], 0, 0)
		if lowerOuter < 0 || upperOuter < 0 || columns[lowerOuter].end != k || columns[upperOuter].start != k+1 {
			return nil, fmt.Errorf(`%w: interface %d changes the outer loop but a wall column crosses it`, ErrDegenerate, k)
		}
		for e, exposed := range boundary.lowerExposed {
			patches = append(patches, stackedPatch{role: fmt.Sprintf("floor(%d,%d)", k, e), floor: true, record: exposed,
				loops: []stackedPatchLoop{{column: lowerOuter, top: true, outer: true}, {column: upperOuter}}})
		}
		for e, exposed := range boundary.upperExposed {
			patches = append(patches, stackedPatch{role: fmt.Sprintf("ceiling(%d,%d)", k, e), record: exposed,
				loops: []stackedPatchLoop{{column: upperOuter, outer: true}, {column: lowerOuter, top: true}}})
		}
		return patches, nil
	}
	lowerOnly, upperOnly, err := stackedExclusiveHoles(lower, upper)
	if err != nil {
		return nil, err
	}
	for e, hole := range upperOnly {
		ci, err := stackedHoleColumn(columns, stackedHoleColumns(bySlab[k+1]), hole, k+1, true)
		if err != nil {
			return nil, err
		}
		patches = append(patches, stackedPatch{role: fmt.Sprintf("floor(%d,%d)", k, e), floor: true,
			record: boundary.lowerExposed[e], loops: []stackedPatchLoop{{column: ci, outer: true}}})
	}
	for e, hole := range lowerOnly {
		ci, err := stackedHoleColumn(columns, stackedHoleColumns(bySlab[k]), hole, k, false)
		if err != nil {
			return nil, err
		}
		patches = append(patches, stackedPatch{role: fmt.Sprintf("ceiling(%d,%d)", k, e),
			record: boundary.upperExposed[e], loops: []stackedPatchLoop{{column: ci, top: true, outer: true}}})
	}
	return patches, nil
}

// stackedHoleColumns lists the columns carrying the first region's holes in
// one slab's entries.
func stackedHoleColumns(entries []stackedSlabLoop) []int {
	var out []int
	for _, e := range entries {
		if e.region == 0 && e.loop != 0 {
			out = append(out, e.column)
		}
	}
	return out
}

// stackedLiningPatches is stackedInterfacePatches for a lining interface. The
// narrow side's own loops — the first region's hole and every other region's
// outer — have no equal loop on the wide side, so each one's column starts
// (narrow side above) or ends (narrow side below) at this interface, and the
// patch reads that end's ring.
func stackedLiningPatches(sp stackedPrismPayload, columns []stackedColumn, bySlab [][]stackedSlabLoop, k int, wideLower bool) ([]stackedPatch, error) {
	narrowSlab, top := k, true
	exposed, role := sp.interfaces[k].upperExposed, fmt.Sprintf("ceiling(%d,0)", k)
	if wideLower {
		narrowSlab, top = k+1, false
		exposed, role = sp.interfaces[k].lowerExposed, fmt.Sprintf("floor(%d,0)", k)
	}
	if len(exposed) != 1 {
		return nil, fmt.Errorf(`%w: lining interface %d records no single exposed patch`, ErrDegenerate, k)
	}
	ring := func(region, loop int, outer bool) (stackedPatchLoop, error) {
		ci := stackedSlabColumn(bySlab[narrowSlab], region, loop)
		if ci < 0 || (top && columns[ci].end != k) || (!top && columns[ci].start != k+1) {
			return stackedPatchLoop{}, fmt.Errorf(`%w: lining interface %d has no wall column ending at it`, ErrDegenerate, k)
		}
		return stackedPatchLoop{column: ci, top: top, outer: outer}, nil
	}
	outer, err := ring(0, 1, true)
	if err != nil {
		return nil, err
	}
	patch := stackedPatch{role: role, floor: wideLower, record: exposed[0], loops: []stackedPatchLoop{outer}}
	for m := 1; m < len(sp.slabs[narrowSlab].regions); m++ {
		hole, err := ring(m, 0, false)
		if err != nil {
			return nil, err
		}
		patch.loops = append(patch.loops, hole)
	}
	return []stackedPatch{patch}, nil
}

// stackedPatchArea is a patch record's area: its outer loop's enclosed area
// less each hole's, with sectionDisplacementArea over every loop's walks and
// proven perimeter folded into the bound (§4), and each loop's own column
// displacement (colDelta) over that loop alone.
func stackedPatchArea(ctx context.Context, sectionDelta float64, colDelta []float64, columns []stackedColumn, patch stackedPatch) (proofbound.BoundedScalar, error) {
	area, err := loopEnclosedAreaContext(ctx, patch.record.Outer)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	for _, hole := range patch.record.Holes {
		holeArea, err := loopEnclosedAreaContext(ctx, hole)
		if err != nil {
			return proofbound.BoundedScalar{}, err
		}
		area = proofbound.BoundedSub(area, holeArea)
	}
	walks := 0
	perimeter := 0.0
	for _, pl := range patch.loops {
		col := columns[pl.column]
		walks += len(col.loop.Segments)
		loopPerimeter := proofbound.AbsSumUpper(col.perimeter.Value, col.perimeter.Bound)
		perimeter = proofbound.AbsSumUpper(perimeter, loopPerimeter)
		if delta := colDelta[pl.column]; delta > 0 {
			area.Bound = proofbound.AbsSumUpper(area.Bound,
				proofbound.SectionDisplacementArea(delta, len(col.loop.Segments), loopPerimeter))
		}
	}
	area.Bound = proofbound.AbsSumUpper(area.Bound, proofbound.SectionDisplacementArea(sectionDelta, walks, perimeter))
	return area, nil
}

type stackedColumn struct {
	loop       LoopRecord
	start, end int
	region     int
	loopIndex  int
	bottom     []coedge
	top        []coedge
	perimeter  proofbound.BoundedScalar
}

func stackedLoops(region ProfileRecord) []LoopRecord {
	return append([]LoopRecord{region.Outer}, region.Holes...)
}

// stackedSlabLoop is one loop of one slab's region: the column that carries
// it, the region it belongs to in that slab, and its index in that region
// (0 the outer, i >= 1 hole i-1).
type stackedSlabLoop struct {
	column, region, loop int
}

// stackedSlabColumn is the column carrying one region's loop, by region and
// loop index, in one slab's entries, or -1.
func stackedSlabColumn(entries []stackedSlabLoop, region, loop int) int {
	for _, e := range entries {
		if e.region == region && e.loop == loop {
			return e.column
		}
	}
	return -1
}

// stackedColumns names each uninterrupted wall once. bySlab lists every
// region loop of each slab, region by region, with the column carrying it,
// including shared through holes. A loop continues the column of an equal
// loop record in the slab below, whichever region of either slab holds it.
func stackedColumns(sp stackedPrismPayload) ([]stackedColumn, [][]stackedSlabLoop, error) {
	var columns []stackedColumn
	bySlab := make([][]stackedSlabLoop, len(sp.slabs))
	for k, slab := range sp.slabs {
		for r, region := range slab.regions {
			for i, loop := range stackedLoops(region) {
				found := -1
				// A prism group has one slab, so no column continues.
				if k != 0 {
					for _, prev := range bySlab[k-1] {
						equal, err := loopRecordsEqual(nil, loop, columns[prev.column].loop)
						if err != nil {
							return nil, nil, err
						}
						if equal {
							found = prev.column
							break
						}
					}
				}
				switch {
				case found < 0:
					found = len(columns)
					columns = append(columns, stackedColumn{loop: loop, start: k, end: k, region: r, loopIndex: i})
				case columns[found].end == k:
					return nil, nil, fmt.Errorf(`%w: slab %d carries one loop record twice`, ErrDegenerate, k)
				default:
					columns[found].end = k
				}
				bySlab[k] = append(bySlab[k], stackedSlabLoop{column: found, region: r, loop: i})
			}
		}
	}
	return columns, bySlab, nil
}

// stackedPlan names a stacked body's faces and states the displacement each
// wall column's own loop carries beside the payload-wide sectionDelta. The
// payload's own plan (stackedOwnPlan) mints §3's roles and charges no column
// displacement. A cup (shell_cup.go) builds through its own plan: it keeps the
// cup's public roles and charges its offset loops their proven offset
// displacement, loop by loop.
type stackedPlan interface {
	// wallRoleLoop is the loop index buildLoopSidesAs mints into column
	// col's side(i,j) roles, and nameWalls rewrites the roles minted on that
	// column's walls.
	wallRoleLoop(col stackedColumn) int
	nameWalls(ctx context.Context, col stackedColumn, faces []*Face, ref producerID) error
	// capRole names the bottom (top false) or top cap of one region of the
	// first or the last slab.
	capRole(slab, region int, top bool) string
	// patchRole names an exposed interface patch.
	patchRole(patch stackedPatch) string
	// columnDelta is the section displacement column col's loop carries
	// beyond the payload's sectionDelta.
	columnDelta(col stackedColumn) float64
	// order is the body's face order, from the build order.
	order(faces []*Face) []*Face
}

// stackedOwnPlan is the stacked payload's own naming (§3).
type stackedOwnPlan struct{}

func (stackedOwnPlan) wallRoleLoop(col stackedColumn) int { return col.loopIndex }

func (stackedOwnPlan) nameWalls(_ context.Context, col stackedColumn, faces []*Face, _ producerID) error {
	for _, face := range faces {
		for i := range face.origins {
			face.origins[i].Role = fmt.Sprintf("slab(%d).region(%d).%s", col.start, col.region, face.origins[i].Role)
		}
	}
	return nil
}

func (stackedOwnPlan) capRole(_, _ int, top bool) string {
	if top {
		return roleCapEnd
	}
	return roleCapStart
}

func (stackedOwnPlan) patchRole(patch stackedPatch) string { return patch.role }

func (stackedOwnPlan) columnDelta(stackedColumn) float64 { return 0 }

func (stackedOwnPlan) order(faces []*Face) []*Face { return faces }

func evalStackedContext(ctx context.Context, d *Document, ref producerID, sp stackedPrismPayload) (*Body, error) {
	return evalStackedPlanContext(ctx, d, ref, sp, stackedOwnPlan{})
}

// stackedPart is one (slab, region) prism of a stacked body: its bounded
// section area and its centroid lifted to the slab's bounded midpoint.
type stackedPart struct {
	slab, region  int
	area          proofbound.BoundedScalar
	centroid      r3.Vec
	centroidBound float64
}

// stackedRegionPart integrates one slab region. The region's area carries
// sectionDisplacementArea for the payload's sectionDelta over all its walks,
// and each loop's own column displacement over that loop alone; a region with
// a displaced loop also widens its first moments by that displacement area
// times the farthest coordinate the displaced boundary can reach, as
// displacedRegionIntegrals does, so the centroid quotient covers it.
func stackedRegionPart(ctx context.Context, sp stackedPrismPayload, base prismPayload, columns []stackedColumn, colDelta []float64,
	entries []stackedSlabLoop, k, r int, work *freeform.FreeformWork) (stackedPart, error) {
	slab := sp.slabs[k]
	region := slab.regions[r]
	ig, err := region.EvaluatorIntegralsContext(ctx, freeform.MomentFirstOrder, work)
	if err != nil {
		return stackedPart{}, err
	}
	if ig.Area <= 0 {
		return stackedPart{}, fmt.Errorf(`%w: slab %d region %d has no material area`, ErrDegenerate, k, r)
	}
	perimeter := proofbound.BoundedScalar{}
	walks := 0
	loopArea, loopReach := 0.0, 0.0
	for _, e := range entries {
		if e.region != r {
			continue
		}
		col := columns[e.column]
		perimeter = proofbound.BoundedAdd(perimeter, col.perimeter)
		walks += len(col.loop.Segments)
		if delta := colDelta[e.column]; delta > 0 {
			loopArea = proofbound.AbsSumUpper(loopArea, proofbound.SectionDisplacementArea(delta, len(col.loop.Segments),
				proofbound.AbsSumUpper(col.perimeter.Value, col.perimeter.Bound)))
			loopReach = max(loopReach, delta)
		}
	}
	if loopArea > 0 {
		coord, err := profileCoordinateEnvelope(region, work, nil)
		if err != nil {
			return stackedPart{}, err
		}
		moment := proofbound.ProductUpper(loopArea, proofbound.AbsSumUpper(coord, loopReach))
		ig.AreaBound = proofbound.AbsSumUpper(ig.AreaBound, loopArea)
		ig.MuBound = proofbound.AbsSumUpper(ig.MuBound, moment)
		ig.MvBound = proofbound.AbsSumUpper(ig.MvBound, moment)
	}
	part := stackedPart{slab: k, region: r, area: proofbound.MeasuredScalar(ig.Area, proofbound.AbsSumUpper(ig.AreaBound,
		proofbound.SectionDisplacementArea(sp.sectionDelta, walks, proofbound.AbsSumUpper(perimeter.Value, perimeter.Bound))))}
	u := proofbound.BoundedQuotient(ig.Mu, ig.MuBound, ig.Area, ig.AreaBound)
	v := proofbound.BoundedQuotient(ig.Mv, ig.MvBound, ig.Area, ig.AreaBound)
	u.Bound = proofbound.AbsSumUpper(u.Bound, sp.sectionDelta)
	v.Bound = proofbound.AbsSumUpper(v.Bound, sp.sectionDelta)
	mid := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.MeasuredScalar(slab.z0, slab.z0Delta),
		proofbound.MeasuredScalar(slab.z1, slab.z1Delta)), proofbound.ExactScalar(2))
	part.centroid = base.point(u.Value, v.Value, mid.Value)
	part.centroidBound = prismPointBound(base, u, v, mid)
	return part, nil
}

// evalStackedPlanContext builds a stacked body under plan's naming and column
// displacements (§3, §4).
func evalStackedPlanContext(ctx context.Context, d *Document, ref producerID, sp stackedPrismPayload, plan stackedPlan) (*Body, error) {
	if err := falsifyStackedPayload(ctx, sp); err != nil {
		return nil, err
	}
	columns, bySlab, err := stackedColumns(sp)
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	work := freeform.NewFreeformWork()
	base := sp.outerPrism()
	colDelta := make([]float64, len(columns))
	maxColDelta := 0.0
	var faces []*Face
	for ci := range columns {
		col := &columns[ci]
		colDelta[ci] = plan.columnDelta(*col)
		maxColDelta = max(maxColDelta, colDelta[ci])
		first, last := sp.slabs[col.start], sp.slabs[col.end]
		pp := base
		pp.z0, pp.z0Delta = first.z0, first.z0Delta
		pp.z1, pp.z1Delta = last.z1, last.z1Delta
		if colDelta[ci] > 0 {
			pp.sectionDelta = proofbound.AbsSumUpper(sp.sectionDelta, colDelta[ci])
		}
		wallFaces, bottom, top, perimeter, err := buildLoopSidesAs(ctx, body, ref, pp,
			plan.wallRoleLoop(*col), col.loopIndex != 0, col.loop, work, nil, levelToken{}, levelToken{}, false)
		if err != nil {
			return nil, err
		}
		if err := plan.nameWalls(ctx, *col, wallFaces, ref); err != nil {
			return nil, err
		}
		col.bottom, col.top, col.perimeter = bottom, top, perimeter
		faces = append(faces, wallFaces...)
	}

	// One part per (slab, region).
	var parts []stackedPart
	for k, slab := range sp.slabs {
		for r := range slab.regions {
			part, err := stackedRegionPart(ctx, sp, base, columns, colDelta, bySlab[k], k, r, work)
			if err != nil {
				return nil, err
			}
			parts = append(parts, part)
		}
	}

	newPlane := func(z, axial float64, flip bool, role string, area proofbound.BoundedScalar) (*Face, error) {
		frame, err := capFrame(base, z, flip)
		if err != nil {
			return nil, err
		}
		return &Face{surface: Plane{Frame: frame}, origins: []FeatureRef{{producer: ref, Role: role}},
			body: body, area: area.Value, areaBound: area.Bound,
			axialDelta: axial, hasAxialDelta: true}, nil
	}
	// One bottom cap per region of the first slab and one top cap per region
	// of the last, so CapStart(group) selects every lump's bottom face
	// (docs/mirror-pattern-design.md §6.3).
	first, last := sp.slabs[0], sp.slabs[len(sp.slabs)-1]
	var planar []*Face
	capArea := proofbound.BoundedScalar{}
	for _, part := range parts {
		for _, end := range []struct {
			slab int
			z, d float64
			flip bool
			top  bool
		}{{0, first.z0, first.z0Delta, true, false}, {len(sp.slabs) - 1, last.z1, last.z1Delta, false, true}} {
			if part.slab != end.slab {
				continue
			}
			f, err := newPlane(end.z, end.d, end.flip, plan.capRole(end.slab, part.region, end.top), part.area)
			if err != nil {
				return nil, err
			}
			for _, e := range bySlab[end.slab] {
				if e.region != part.region {
					continue
				}
				coedges := columns[e.column].bottom
				if end.top {
					coedges = columns[e.column].top
				}
				f.loops = append(f.loops, &Loop{coedges: coedges, outer: e.loop == 0})
			}
			if len(planar) == 0 {
				capArea = part.area
			} else {
				capArea = proofbound.BoundedAdd(capArea, part.area)
			}
			planar = append(planar, f)
		}
	}
	capCount := len(planar)
	for k := range sp.interfaces {
		z, axial := sp.slabs[k].z1, sp.slabs[k].z1Delta
		patches, err := stackedInterfacePatches(sp, columns, bySlab, k)
		if err != nil {
			return nil, err
		}
		for _, patch := range patches {
			area, err := stackedPatchArea(ctx, sp.sectionDelta, colDelta, columns, patch)
			if err != nil {
				return nil, err
			}
			f, err := newPlane(z, axial, !patch.floor, plan.patchRole(patch), area)
			if err != nil {
				return nil, err
			}
			for _, pl := range patch.loops {
				coedges := columns[pl.column].bottom
				if pl.top {
					coedges = columns[pl.column].top
				}
				f.loops = append(f.loops, &Loop{coedges: coedges, outer: pl.outer})
			}
			planar = append(planar, f)
		}
	}
	if err := attachFaceLoopsContext(ctx, planar); err != nil {
		return nil, err
	}
	faces = append(faces, planar...)
	body.lumps = sheetLumps(plan.order(faces))

	volume := proofbound.BoundedScalar{}
	area := capArea
	var moment [3]proofbound.BoundedScalar
	for _, part := range parts {
		slab := sp.slabs[part.slab]
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(slab.z1, slab.z1Delta), proofbound.MeasuredScalar(slab.z0, slab.z0Delta))
		mass := proofbound.BoundedMul(part.area, height)
		volume = proofbound.BoundedAdd(volume, mass)
		point := part.centroid
		pointBound := part.centroidBound
		for i, value := range []float64{point.X, point.Y, point.Z} {
			moment[i] = proofbound.BoundedAdd(moment[i], proofbound.BoundedMul(mass, proofbound.MeasuredScalar(value, pointBound)))
		}
	}
	for _, col := range columns {
		a, b := sp.slabs[col.start], sp.slabs[col.end]
		height := proofbound.BoundedSub(proofbound.MeasuredScalar(b.z1, b.z1Delta), proofbound.MeasuredScalar(a.z0, a.z0Delta))
		area = proofbound.BoundedAdd(area, proofbound.BoundedMul(col.perimeter, height))
	}
	for _, face := range planar[capCount:] {
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(face.area, face.areaBound))
	}
	if volume.Value <= 0 {
		return nil, fmt.Errorf(`%w: a stacked prism encloses no volume`, ErrDegenerate)
	}
	body.volume = Measurement{Value: units.CubicMillimeters(volume.Value), Exactness: exactnessOf(volume.Bound),
		Bound: units.CubicMillimeters(volume.Bound)}
	body.area = Measurement{Value: units.SquareMillimeters(area.Value), Exactness: exactnessOf(area.Bound),
		Bound: units.SquareMillimeters(area.Bound)}
	x, y, z := proofbound.BoundedDiv(moment[0], volume), proofbound.BoundedDiv(moment[1], volume), proofbound.BoundedDiv(moment[2], volume)
	centroid := r3.Vec{X: x.Value, Y: y.Value, Z: z.Value}
	centroidBound := proofbound.Radius3D(max(x.Bound, y.Bound, z.Bound))
	if sp.sectionDelta > 0 || maxColDelta > 0 {
		runs, err := sp.outerRuns()
		if err != nil {
			return nil, err
		}
		// Every material point lies in one run's prism, so the largest run
		// envelope caps the centroid's distance from the held point.
		geometryBound := 0.0
		for _, run := range runs {
			runBound, err := prismCentroidGeometryBound(run, run.profile, centroid, work, nil)
			if err != nil {
				return nil, err
			}
			geometryBound = max(geometryBound, runBound)
		}
		if sp.sectionDelta > 0 {
			centroidBound = max(centroidBound, proofbound.AbsSumUpper(geometryBound,
				sp.sectionDelta, sp.axialDelta()))
		} else {
			// The column displacements are already charged into every part's
			// moments, so the envelope is a ceiling on that sound answer. It is
			// read off the recorded runs; the denoted body reaches up to the
			// largest column displacement farther in the plane and each level's
			// axial one along the normal, carried through the rigid map's 3·L1
			// factor, as the cup's own envelope is.
			geometryBound = proofbound.AbsSumUpper(geometryBound, proofbound.ProductUpper(3, proofbound.AbsSumUpper(
				proofbound.ProductUpper(proofbound.AbsSumUpper(vecL1(base.frame.U()), vecL1(base.frame.V())), maxColDelta),
				proofbound.ProductUpper(vecL1(base.frame.N()), sp.axialDelta()),
			)))
			centroidBound = math.Min(centroidBound, geometryBound)
		}
	}
	body.centroid = VecMeasurement{Value: centroid, Exactness: exactnessOf(centroidBound), Bound: units.Millimeters(centroidBound)}
	outerDelta := 0.0
	for _, entries := range bySlab {
		for _, e := range entries {
			if e.loop == 0 && (e.region == 0 || sp.isGroup()) {
				outerDelta = max(outerDelta, colDelta[e.column])
			}
		}
	}
	bounds, err := stackedBoundsContext(ctx, sp, outerDelta, work)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = sp
	return body, nil
}
