package pointquery

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
)

// LocateStacked classifies the world point p against the solid an
// undisplaced stacked prism record denotes: the union of every slab's
// regions swept over that slab's interval, carried through the record's
// frame and placement (agent-docs/point-containment-design.md §9). decided
// is false for a displaced record or a point the route leaves open. p must
// be finite.
func LocateStacked(ctx context.Context, p r3.Vec, sp stackedrecord.Payload) (Location, bool, error) {
	if err := ctx.Err(); err != nil {
		return Undecided, false, err
	}
	if sp.SectionDelta != 0 {
		return Undecided, false, nil
	}
	return locateSlabs(ctx, p, sp.Slabs, sp.Frame, sp.Xform)
}

// LocateStackBrep classifies the world point p against a BRep that keeps the
// slabs a stacked Union built it from (agent-docs/general-boolean-design.md
// §4.1), when its faces are exactly those slabs' loops at their levels: a
// zero stack and face displacement, no blend face, loop band or shell band,
// and a first face that is a planar cap in the reference frame the stack is
// stated in (agent-docs/point-containment-design.md §9.1). Every other BRep,
// and a point the route leaves open, reports decided false. A line-only A7
// Cut BRep instead combines its transient exact source prisms through §7.
// p must be finite.
func LocateStackBrep(ctx context.Context, p r3.Vec, bp breprecord.Payload) (Location, bool, error) {
	if err := ctx.Err(); err != nil {
		return Undecided, false, err
	}
	if cut := bp.PointQueryCut; cut != nil {
		if bp.Stack != nil || !cut.Target.Profile.AllLines() || !cut.Tool.Profile.AllLines() ||
			len(cut.Target.Profile.Holes) != 1 || len(cut.Tool.Profile.Holes) != 0 ||
			cut.Target.SectionDelta != 0 || cut.Tool.SectionDelta != 0 ||
			len(bp.Faces) == 0 || len(bp.LoopBands) != 0 || bp.BossShell != nil || bp.PocketShell != nil {
			return Undecided, false, nil
		}
		for _, face := range bp.Faces {
			if face.Blend != "" {
				return Undecided, false, nil
			}
		}
		target, targetOK, err := LocatePrism(ctx, p, cut.Target)
		if err != nil || !targetOK {
			return Undecided, false, err
		}
		tool, toolOK, err := LocatePrism(ctx, p, cut.Tool)
		if err != nil || !toolOK {
			return Undecided, false, err
		}
		result := meet(target, complement(tool))
		return result, result != Undecided, nil
	}
	stack := bp.Stack
	if stack == nil || stack.Delta != 0 || len(bp.Faces) == 0 ||
		len(bp.LoopBands) != 0 || bp.BossShell != nil || bp.PocketShell != nil {
		return Undecided, false, nil
	}
	for _, face := range bp.Faces {
		if face.Delta != 0 || face.Z0Delta != 0 || face.Z1Delta != 0 || face.Blend != "" {
			return Undecided, false, nil
		}
		for _, splits := range [...][]brepgeom.Split{face.Side0, face.Side1} {
			for _, split := range splits {
				if split.ZDelta != 0 {
					return Undecided, false, nil
				}
			}
		}
	}
	if first := bp.Faces[0]; !first.Planar() || first.Sweep != (r3.Vec{}) {
		return Undecided, false, nil
	}
	// The BRep joins each junction at one canonical vertex
	// (agent-docs/general-boolean-design.md §4.1); the route reads only
	// loops whose junctions close at two equal ends, where that vertex is
	// the recorded end itself.
	budget := proofbound.NewWorkBudget(ctx)
	for _, slab := range stack.Slabs {
		for _, region := range slab.Regions {
			rings, ok, err := readProfile(budget, region)
			if err != nil || !ok || !rings.closed() {
				return Undecided, false, err
			}
		}
	}
	return locateSlabs(ctx, p, stack.Slabs, bp.Faces[0].Frame, bp.Xform)
}

// locateSlabs classifies p against the union of every slab's regions swept
// over its interval in the plane coordinates frame and xform place
// (§9.2). Every level must carry zero displacement, and the record must pass
// the reject-only stack checks: a slab, a region per slab, Z0 < Z1, and one
// held level shared by consecutive slabs.
func locateSlabs(ctx context.Context, p r3.Vec, slabs []stackedrecord.Slab, frame r3.Frame, xform r3.Transform) (Location, bool, error) {
	if len(slabs) == 0 {
		return Undecided, false, nil
	}
	levels := make([]*big.Rat, 0, len(slabs)+1)
	for i, slab := range slabs {
		if slab.Z0Delta != 0 || slab.Z1Delta != 0 || len(slab.Regions) == 0 {
			return Undecided, false, nil
		}
		z0, z1 := proof.FloatRat(slab.Z0), proof.FloatRat(slab.Z1)
		if z0 == nil || z1 == nil || z0.Cmp(z1) >= 0 {
			return Undecided, false, nil
		}
		if i == 0 {
			levels = append(levels, z0)
		} else if levels[i].Cmp(z0) != 0 {
			return Undecided, false, nil
		}
		levels = append(levels, z1)
	}
	local, ok := placedLocal(p, frame, xform)
	if !ok {
		return Undecided, false, nil
	}
	z, q := local[2], planePoint{u: local[0], v: local[1]}
	n := len(slabs)
	if z.Cmp(levels[0]) < 0 || z.Cmp(levels[n]) > 0 {
		return Outside, true, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	slabClass := func(i int) (Location, error) {
		if i < 0 || i >= n {
			return Outside, nil
		}
		acc := Outside
		for _, region := range slabs[i].Regions {
			if err := budget.Step(); err != nil {
				return Undecided, err
			}
			class, err := locateInProfile(budget, region, q)
			if err != nil {
				return Undecided, err
			}
			acc = join(acc, regionLocation(class))
		}
		return acc, nil
	}
	var result Location
	for k := range levels {
		cmp := z.Cmp(levels[k])
		if cmp == 0 {
			below, err := slabClass(k - 1)
			if err != nil {
				return Undecided, false, err
			}
			above, err := slabClass(k)
			if err != nil {
				return Undecided, false, err
			}
			result = atLevel(below, above)
			break
		}
		if cmp < 0 {
			class, err := slabClass(k - 1)
			if err != nil {
				return Undecided, false, err
			}
			result = class
			break
		}
	}
	return result, result != Undecided, nil
}

// atLevel classifies a point on a level between two slabs from its plane
// class in the slab below and the slab above, a missing slab reading
// outside (§9.2's table). Near the point the material is the lower region
// under the level and the upper region over it. On either region's boundary
// the point is material with non-material points across that boundary on
// its own side of the level. Inside one and outside the other, the points
// straight across the level on the outside side are not material. Equal
// decided classes carry over, and any other pair is Undecided.
func atLevel(below, above Location) Location {
	switch {
	case below == OnBoundary || above == OnBoundary:
		return OnBoundary
	case below == Undecided || above == Undecided:
		return Undecided
	case below == above:
		return below
	}
	return OnBoundary
}
