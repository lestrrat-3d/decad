package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is the face view that states a prism or a stacked prism as a
// brepPayload, the analytically trimmed face record of
// docs/general-boolean-design.md §4 (§4.1). internal/breprecord owns the
// record, its audit, the body build with its topology and roles (§4.2) and the
// measurements (§4.3); tessellate_brep.go owns the mesh and its
// occupied-volume proof (§4.4).

// brepOfPrism is a prism's face view (§4.1): one swept face per recorded
// segment, then the start and end caps. It is built on demand and never stored
// for a prism. A surface result has no closed face view and is ErrUnsupported.
func brepOfPrism(pp prismPayload) (brepPayload, error) {
	bp, _, err := brepOfPrismWithRoles(pp)
	return bp, err
}

// brepOfPrismWithRoles also maps record faces to the source prism's public
// roles, so an asymmetric chamfer can resolve its reference face.
func brepOfPrismWithRoles(pp prismPayload) (brepPayload, []string, error) {
	if pp.SurfaceResult {
		return brepPayload{}, nil, fmt.Errorf(`%w: a surface-result prism has no closed face view`, ErrUnsupported)
	}
	profile, allow, err := brepJoinProfile(pp.Profile)
	if err != nil {
		return brepPayload{}, nil, err
	}
	pp.Profile = profile
	if allow > 0 {
		pp.SectionDelta = proofbound.AbsSumUpper(pp.SectionDelta, allow)
	}
	bp := brepPayload{Xform: pp.Xform}
	var sourceRoles []string
	for li, loop := range append([]loopRecord{pp.Profile.Outer}, pp.Profile.Holes...) {
		for si, seg := range loop.Segments {
			bp.Faces = append(bp.Faces, brepFace{
				Frame: pp.Frame, Wall: seg, Z0: pp.Z0, Z1: pp.Z1,
				Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta, Delta: pp.SectionDelta,
			})
			sourceRoles = append(sourceRoles, fmt.Sprintf("side(%d,%d)", li, si))
		}
	}
	region := pp.Profile
	bp.Faces = append(bp.Faces,
		brepFace{Frame: pp.Frame, Region: &region, Z0: pp.Z0, Z1: pp.Z0,
			Z0Delta: pp.Z0Delta, Z1Delta: pp.Z0Delta, Delta: pp.SectionDelta},
		brepFace{Frame: pp.Frame, Region: &region, Outward: true, Z0: pp.Z1, Z1: pp.Z1,
			Z0Delta: pp.Z1Delta, Z1Delta: pp.Z1Delta, Delta: pp.SectionDelta},
	)
	sourceRoles = append(sourceRoles, provenance.RoleCapStart, provenance.RoleCapEnd)
	bp.AssignRoles()
	return bp, sourceRoles, nil
}

// brepOfStacked is a stacked prism's face view (§4.1): one swept face per
// segment of every loop column (docs/stacked-prism-design.md §2.3), then the
// two caps and every interface's exposed floors and ceilings. The record is
// audited first, so a stack its own build refuses has no face view either. A
// prism group (one slab of several disjoint regions) has no face view and is
// ErrUnsupported: the record states one region per cap. So is a stack
// enclosing a cavity (a closed shell, EnclosesCavity).
func brepOfStacked(ctx context.Context, sp stackedPrismPayload) (brepPayload, error) {
	bp, _, err := brepOfStackedWithRoles(ctx, sp)
	return bp, err
}

// brepOfStackedWithRoles also returns each record face's role on the source
// stacked body. Chamfer uses the roles only while resolving a public face.
func brepOfStackedWithRoles(ctx context.Context, sp stackedPrismPayload) (brepPayload, []string, error) {
	if sp.IsGroup() {
		return brepPayload{}, nil, fmt.Errorf(`%w: a prism group of %d disjoint regions has no face view`, ErrUnsupported, len(sp.Slabs[0].Regions))
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
		return brepPayload{}, nil, err
	}
	cavity, err := sp.EnclosesCavity()
	if err != nil {
		return brepPayload{}, nil, err
	}
	if cavity {
		// A brep record's lumps are its connected face sets, so a cavity's
		// faces would read as a second solid.
		return brepPayload{}, nil, fmt.Errorf(`%w: a stacked prism enclosing a cavity has no face view`, ErrUnsupported)
	}
	sp, err = brepJoinStacked(sp)
	if err != nil {
		return brepPayload{}, nil, err
	}
	columns, _, err := sp.Columns()
	if err != nil {
		return brepPayload{}, nil, err
	}
	bp := brepPayload{Xform: sp.Xform}
	var sourceRoles []string
	for _, col := range columns {
		first, last := sp.Slabs[col.Start], sp.Slabs[col.End]
		for segment, seg := range col.Loop.Segments {
			bp.Faces = append(bp.Faces, brepFace{
				Frame: sp.Frame, Wall: seg, Z0: first.Z0, Z1: last.Z1,
				Z0Delta: first.Z0Delta, Z1Delta: last.Z1Delta, Delta: sp.SectionDelta,
			})
			sourceRoles = append(sourceRoles, fmt.Sprintf("slab(%d).region(%d).side(%d,%d)",
				col.Start, col.Region, col.LoopIndex, segment))
		}
	}
	addPlanar := func(region profileRecord, z, zDelta float64, outward bool, sourceRole string) {
		bp.Faces = append(bp.Faces, brepFace{Frame: sp.Frame, Region: &region, Outward: outward,
			Z0: z, Z1: z, Z0Delta: zDelta, Z1Delta: zDelta, Delta: sp.SectionDelta})
		sourceRoles = append(sourceRoles, sourceRole)
	}
	first, last := sp.Slabs[0], sp.Slabs[len(sp.Slabs)-1]
	addPlanar(first.Regions[0], first.Z0, first.Z0Delta, false, provenance.RoleCapStart)
	addPlanar(last.Regions[0], last.Z1, last.Z1Delta, true, provenance.RoleCapEnd)
	for k, boundary := range sp.Interfaces {
		z, zDelta := sp.Slabs[k].Z1, sp.Slabs[k].Z1Delta
		for e, region := range boundary.LowerExposed {
			addPlanar(region, z, zDelta, true, fmt.Sprintf("floor(%d,%d)", k, e))
		}
		for e, region := range boundary.UpperExposed {
			addPlanar(region, z, zDelta, false, fmt.Sprintf("ceiling(%d,%d)", k, e))
		}
	}
	bp.AssignRoles()
	return bp, sourceRoles, nil
}

// brepJoinProfile adapts a region to brepgeom.JoinProfile.
func brepJoinProfile(p profileRecord) (profileRecord, float64, error) {
	joined, allow, err := brepgeom.JoinProfile(brepgeom.Profile{Outer: p.Outer, Holes: p.Holes})
	return profileRecord{Outer: joined.Outer, Holes: joined.Holes}, allow, err
}

// brepJoinStacked joins every region and exposed record of a
// stack, charging the largest allow to its section displacement. Equal loops
// join alike, so the columns Payload.Columns derives are unchanged.
func brepJoinStacked(sp stackedPrismPayload) (stackedPrismPayload, error) {
	allow := 0.0
	join := func(p profileRecord) (profileRecord, error) {
		out, a, err := brepJoinProfile(p)
		allow = math.Max(allow, a)
		return out, err
	}
	out := sp
	out.Slabs = make([]stackedrecord.Slab, len(sp.Slabs))
	for k, slab := range sp.Slabs {
		out.Slabs[k] = slab
		out.Slabs[k].Regions = make([]profileRecord, len(slab.Regions))
		for r, region := range slab.Regions {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.Slabs[k].Regions[r] = joined
		}
	}
	out.Interfaces = make([]stackedrecord.Interface, len(sp.Interfaces))
	for k, boundary := range sp.Interfaces {
		for _, region := range boundary.LowerExposed {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.Interfaces[k].LowerExposed = append(out.Interfaces[k].LowerExposed, joined)
		}
		for _, region := range boundary.UpperExposed {
			joined, err := join(region)
			if err != nil {
				return stackedPrismPayload{}, err
			}
			out.Interfaces[k].UpperExposed = append(out.Interfaces[k].UpperExposed, joined)
		}
	}
	if allow > 0 {
		out.SectionDelta = proofbound.AbsSumUpper(sp.SectionDelta, allow)
	}
	return out, nil
}
