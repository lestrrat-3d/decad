package stackedunion

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/lestrrat-3d/sketch"
)

// maxAuditRotations bounds how many ring starts holedRegionAudit tries.
const maxAuditRotations = 8

// sceneHasHoles reports whether some record of the scene has a hole.
func (b *ubBuild) sceneHasHoles(sc *ubScene) bool {
	for _, refs := range [][]ubRef{sc.refsA, sc.refsB} {
		for _, ref := range refs {
			if len(b.record(ref).Holes) != 0 {
				return true
			}
		}
	}
	return false
}

// holedSlabRegions is one result slab's regions when some record reaching it
// has holes: the cells inside some record, merged. A cell inside no record
// must lie inside a hole of some record (an edge of a hole loop with the cell
// on that loop's void side); any other is a new enclosed void and a miss. One
// outer loop takes every hole; several outer loops with a hole are a miss,
// and several without one are proven apart as the hole-free merge's are.
func (b *ubBuild) holedSlabRegions(ctx context.Context, sc *ubScene) ([]profileRecord, error) {
	if sc.matter == nil {
		matter, resolved, err := prismcells.ClassifyRegions(b.st.budget, sc.tags, sc.profiles)
		if err != nil {
			return nil, err
		}
		if !resolved {
			return nil, errUBMiss
		}
		sc.matter = matter
	}
	var selected []*sketch.Profile
	for i, cell := range sc.profiles {
		inside := false
		for _, m := range sc.matter {
			inside = inside || m[i]
		}
		if inside {
			selected = append(selected, cell)
			continue
		}
		own, err := b.inOwnHole(sc, cell)
		if err != nil {
			return nil, err
		}
		if !own {
			return nil, errUBMiss
		}
	}
	if len(selected) == 0 {
		return nil, errUBMiss
	}
	if ok, err := sc.delta.SharedSpansBounded(b.st.budget, sc.profiles); err != nil || !ok {
		if err != nil {
			return nil, err
		}
		return nil, errUBMiss
	}
	loops, cutDelta, resolved, err := prismcells.MergeLoopsExactLineCuts(b.st.budget, sc.sketch, selected, "union")
	if err != nil {
		return nil, err
	}
	if !resolved {
		return nil, errUBMiss
	}
	b.cutDelta = math.Max(b.cutDelta, cutDelta)
	var outers, holes []sectionrecord.LoopRecord
	for _, loop := range loops {
		area, err := classbgeom.LoopSignedArea(loop)
		if err != nil {
			return nil, err
		}
		switch {
		case area > 0:
			outers = append(outers, loop)
		case area < 0:
			holes = append(holes, loop)
		default:
			return nil, errUBMiss
		}
	}
	switch {
	case len(outers) == 0:
		return nil, errUBMiss
	case len(outers) > 1 && len(holes) != 0:
		return nil, errUBMiss
	case len(outers) > 1:
		out := make([]profileRecord, len(outers))
		for i, loop := range outers {
			out[i] = profileRecord{Outer: loop}
			if err := prismcells.AuditMergeSection(b.st.budget, prismrecord.Payload{Profile: out[i]}, out[i]); err != nil {
				return nil, err
			}
		}
		if err := b.proveApart(ctx, out); err != nil {
			return nil, err
		}
		return out, nil
	}
	region, err := b.holedRegionAudit(profileRecord{Outer: outers[0], Holes: holes})
	if err != nil {
		return nil, err
	}
	return []profileRecord{region}, nil
}

// inOwnHole reports whether a cell inside no record has an edge of some
// record's hole loop with the cell on that loop's void side.
func (b *ubBuild) inOwnHole(sc *ubScene, cell *sketch.Profile) (bool, error) {
	for _, loop := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
		for _, e := range loop {
			if err := b.st.budget.Step(); err != nil {
				return false, err
			}
			origin, ok := sc.tags[e.Entity]
			if !ok {
				return false, errUBMiss
			}
			if origin.Hole >= 0 && e.Reversed != origin.AuthoredReversed {
				return true, nil
			}
		}
	}
	return false, nil
}

// holedRegionAudit audits a holed region by ring rotation: for k = 0 up to
// min(8, the longest ring) it rotates every ring to start at its (k mod
// len)-th segment and keeps the first rotation prismcells.AuditMergeSection
// passes. Rotation changes no geometry; it only changes which loop point the
// nesting audit classifies, and S7 rules out contact, so every point of a
// loop classifies alike. With no passing rotation it returns the k = 0
// audit's error.
func (b *ubBuild) holedRegionAudit(region profileRecord) (profileRecord, error) {
	longest := len(region.Outer.Segments)
	for _, hole := range region.Holes {
		longest = max(longest, len(hole.Segments))
	}
	var first error
	for k := range min(maxAuditRotations, longest) {
		rotated := profileRecord{Outer: stackedbrep.RotateRing(region.Outer, k)}
		for _, hole := range region.Holes {
			rotated.Holes = append(rotated.Holes, stackedbrep.RotateRing(hole, k))
		}
		err := prismcells.AuditMergeSection(b.st.budget, prismrecord.Payload{Profile: rotated}, rotated)
		if err == nil {
			return rotated, nil
		}
		if k == 0 {
			first = err
		}
	}
	if first == nil {
		first = errUBMiss
	}
	return profileRecord{}, first
}
