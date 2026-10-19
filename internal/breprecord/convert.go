package breprecord

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the face view that states a prism or a stacked prism as a
// Payload, the analytically trimmed face record of
// docs/general-boolean-design.md §4 (§4.1), and the other builders of a
// Payload from a record or an engine: a side opening, an A1 stack and the P8
// plate. Payload's audit, the body build with its topology and roles (§4.2)
// and the measurements (§4.3) live beside it; internal/brepbandmesh owns the
// mesh and its occupied-volume proof (§4.4).

// OfPrism is a prism's face view (§4.1): one swept face per recorded
// segment, then the start and end caps. It is built on demand and never stored
// for a prism. A surface result has no closed face view and is ErrUnsupported.
func OfPrism(pp prismrecord.Payload) (Payload, error) {
	bp, _, err := OfPrismWithRoles(pp)
	return bp, err
}

// OfPrismWithRoles also maps record faces to the source prism's public
// roles, so an asymmetric chamfer can resolve its reference face.
func OfPrismWithRoles(pp prismrecord.Payload) (Payload, []string, error) {
	if pp.SurfaceResult {
		return Payload{}, nil, fmt.Errorf(`%w: a surface-result prism has no closed face view`, decaderr.ErrUnsupported)
	}
	profile, allow, err := JoinProfile(pp.Profile)
	if err != nil {
		return Payload{}, nil, err
	}
	pp.Profile = profile
	if allow > 0 {
		pp.SectionDelta = proofbound.AbsSumUpper(pp.SectionDelta, allow)
	}
	bp := Payload{Xform: pp.Xform}
	var sourceRoles []string
	for li, loop := range append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...) {
		for si, seg := range loop.Segments {
			bp.Faces = append(bp.Faces, Face{
				Frame: pp.Frame, Wall: seg, Z0: pp.Z0, Z1: pp.Z1,
				Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta, Delta: pp.SectionDelta,
			})
			sourceRoles = append(sourceRoles, fmt.Sprintf("side(%d,%d)", li, si))
		}
	}
	region := pp.Profile
	bp.Faces = append(bp.Faces,
		Face{Frame: pp.Frame, Region: &region, Z0: pp.Z0, Z1: pp.Z0,
			Z0Delta: pp.Z0Delta, Z1Delta: pp.Z0Delta, Delta: pp.SectionDelta},
		Face{Frame: pp.Frame, Region: &region, Outward: true, Z0: pp.Z1, Z1: pp.Z1,
			Z0Delta: pp.Z1Delta, Z1Delta: pp.Z1Delta, Delta: pp.SectionDelta},
	)
	sourceRoles = append(sourceRoles, provenance.RoleCapStart, provenance.RoleCapEnd)
	bp.AssignRoles()
	return bp, sourceRoles, nil
}

// OfStacked is a stacked prism's face view (§4.1): one swept face per
// segment of every loop column (docs/stacked-prism-design.md §2.3), then the
// two caps and every interface's exposed floors and ceilings. The record is
// audited first, so a stack its own build refuses has no face view either. A
// prism group (one slab of several disjoint regions) has no face view and is
// ErrUnsupported: the record states one region per cap. So is a stack
// enclosing a cavity (a closed shell, EnclosesCavity).
func OfStacked(ctx context.Context, sp stackedrecord.Payload) (Payload, error) {
	bp, _, err := OfStackedWithRoles(ctx, sp)
	return bp, err
}

// OfStackedWithRoles also returns each record face's role on the source
// stacked body. Chamfer uses the roles only while resolving a public face.
func OfStackedWithRoles(ctx context.Context, sp stackedrecord.Payload) (Payload, []string, error) {
	if sp.IsGroup() {
		return Payload{}, nil, fmt.Errorf(`%w: a prism group of %d disjoint regions has no face view`, decaderr.ErrUnsupported, len(sp.Slabs[0].Regions))
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
		return Payload{}, nil, err
	}
	cavity, err := sp.EnclosesCavity()
	if err != nil {
		return Payload{}, nil, err
	}
	if cavity {
		// A brep record's lumps are its connected face sets, so a cavity's
		// faces would read as a second solid.
		return Payload{}, nil, fmt.Errorf(`%w: a stacked prism enclosing a cavity has no face view`, decaderr.ErrUnsupported)
	}
	sp, err = joinStacked(sp)
	if err != nil {
		return Payload{}, nil, err
	}
	columns, _, err := sp.Columns()
	if err != nil {
		return Payload{}, nil, err
	}
	bp := Payload{Xform: sp.Xform}
	var sourceRoles []string
	for _, col := range columns {
		first, last := sp.Slabs[col.Start], sp.Slabs[col.End]
		for segment, seg := range col.Loop.Segments {
			bp.Faces = append(bp.Faces, Face{
				Frame: sp.Frame, Wall: seg, Z0: first.Z0, Z1: last.Z1,
				Z0Delta: first.Z0Delta, Z1Delta: last.Z1Delta, Delta: sp.SectionDelta,
			})
			sourceRoles = append(sourceRoles, fmt.Sprintf("slab(%d).region(%d).side(%d,%d)",
				col.Start, col.Region, col.LoopIndex, segment))
		}
	}
	addPlanar := func(region momentinput.Profile, z, zDelta float64, outward bool, sourceRole string) {
		bp.Faces = append(bp.Faces, Face{Frame: sp.Frame, Region: &region, Outward: outward,
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

// JoinProfile adapts a region to brepgeom.JoinProfile.
func JoinProfile(p momentinput.Profile) (momentinput.Profile, float64, error) {
	joined, allow, err := brepgeom.JoinProfile(brepgeom.Profile{Outer: p.Outer, Holes: p.Holes})
	return momentinput.Profile{Outer: joined.Outer, Holes: joined.Holes}, allow, err
}

// joinStacked joins every region and exposed record of a
// stack, charging the largest allow to its section displacement. Equal loops
// join alike, so the columns Payload.Columns derives are unchanged.
func joinStacked(sp stackedrecord.Payload) (stackedrecord.Payload, error) {
	allow := 0.0
	join := func(p momentinput.Profile) (momentinput.Profile, error) {
		out, a, err := JoinProfile(p)
		allow = math.Max(allow, a)
		return out, err
	}
	out := sp
	out.Slabs = make([]stackedrecord.Slab, len(sp.Slabs))
	for k, slab := range sp.Slabs {
		out.Slabs[k] = slab
		out.Slabs[k].Regions = make([]momentinput.Profile, len(slab.Regions))
		for r, region := range slab.Regions {
			joined, err := join(region)
			if err != nil {
				return stackedrecord.Payload{}, err
			}
			out.Slabs[k].Regions[r] = joined
		}
	}
	out.Interfaces = make([]stackedrecord.Interface, len(sp.Interfaces))
	for k, boundary := range sp.Interfaces {
		for _, region := range boundary.LowerExposed {
			joined, err := join(region)
			if err != nil {
				return stackedrecord.Payload{}, err
			}
			out.Interfaces[k].LowerExposed = append(out.Interfaces[k].LowerExposed, joined)
		}
		for _, region := range boundary.UpperExposed {
			joined, err := join(region)
			if err != nil {
				return stackedrecord.Payload{}, err
			}
			out.Interfaces[k].UpperExposed = append(out.Interfaces[k].UpperExposed, joined)
		}
	}
	if allow > 0 {
		out.SectionDelta = proofbound.AbsSumUpper(sp.SectionDelta, allow)
	}
	return out, nil
}

// StackedRecord states an engine's slabs and horizontal faces as a
// Payload in reference frame ref under xform, every face carrying the
// section displacement delta: the horizontal faces, every curved or oblique
// wall piece (stackedbrep.Engine.sweptFaces) and every axis-aligned wall plane
// (stackedbrep.Engine.wallFaces). Each planar face record then faces modify §5's audit
// (simplicity, orientation and nesting; general-boolean §5), roles are
// assigned, and the record must close by counting (§4.2). geom has read
// every slab loop and horizontal face and recorded its junctions. A topology
// the engine does not cover is brepgeom.ErrStackedWallMiss.
func StackedRecord(ctx context.Context, budget *proofbound.WorkBudget, geom *stackedbrep.Engine, levels []stackedbrep.Level, ref r3.Frame, xform r3.Transform, delta float64) (Payload, error) {
	faces, err := geom.AssembleFaces(levels, ref, delta)
	if err != nil {
		return Payload{}, err
	}
	out := Payload{Xform: xform}
	for _, f := range faces {
		var region *momentinput.Profile
		if f.Region != nil {
			p := momentinput.Profile{Outer: f.Region.Outer, Holes: f.Region.Holes}
			region = &p
		}
		out.Faces = append(out.Faces, Face{
			Frame: f.Frame, Region: region, Outward: f.Outward, Sweep: f.Sweep, Wall: f.Wall,
			Z0: f.Z0, Z1: f.Z1, Z0Delta: f.Z0Delta, Z1Delta: f.Z1Delta,
			Side0: f.Side0, Side1: f.Side1, Delta: f.Delta,
		})
	}
	// Modify §5's audit per planar face record (general-boolean §5):
	// simplicity, orientation and nesting. It refuses; it admits nothing.
	for _, f := range out.Faces {
		if f.Region == nil {
			continue
		}
		if err := prismcells.AuditMergeSection(budget, prismrecord.Payload{Profile: *f.Region}, *f.Region); err != nil {
			return Payload{}, err
		}
	}
	out.AssignRoles()
	// The record must close by counting (§4.2).
	if _, err := TopologyContext(ctx, out); err != nil {
		return Payload{}, err
	}
	return out, nil
}

// SideOpening records the stack's built faces under the receiver frame and
// placement. The section and slab construction runs in internal/prismshell.
func SideOpening(ctx context.Context, budget *proofbound.WorkBudget, pp prismrecord.Payload,
	sec prismshell.Section, removedStart, removedEnd bool, s, tmm, tDelta float64) (Payload, error) {
	geom, levels, delta, err := prismshell.Stack(budget, prismshell.StackInput{
		Section: sec, Z0: pp.Z0, Z1: pp.Z1, Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta,
		RemovedStart: removedStart, RemovedEnd: removedEnd, Sense: s,
		Thickness: tmm, ThicknessDelta: tDelta,
	})
	var bp Payload
	if err == nil {
		bp, err = StackedRecord(ctx, budget, geom, levels, pp.Frame, pp.Xform, delta)
	}
	if errors.Is(err, brepgeom.ErrStackedWallMiss) {
		return Payload{}, fmt.Errorf(`%w: the stacked record build does not state this side opening's faces (shell-opening SO5)`, decaderr.ErrUnsupported)
	}
	return bp, err
}

// IsP8Rounded compares every stored field, including segment domains,
// displacements, placement, and Boolean trim loops. It accepts only the
// exact 40x20x20 radius-3 plate with one centered y-through radius-3 bore.
// A future wider route needs its own source and clearance proof.
func IsP8Rounded(bp Payload) bool {
	want, ok := p8RoundedRecord()
	return ok && sectionrecord.IdenticalRecord(bp, want)
}

func p8RoundedRecord() (Payload, bool) {
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		return Payload{}, false
	}
	xz, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		return Payload{}, false
	}
	point := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line := func(x0, y0, x1, y1 float64) sectionrecord.CurveSegment {
		return sectionrecord.LineSeg{Start: point(x0, y0), End: point(x1, y1), TEnd: 1}
	}
	arc := func(cx, cy, x0, y0, x1, y1 float64) sectionrecord.CurveSegment {
		return sectionrecord.ArcSeg{Center: point(cx, cy), Start: point(x0, y0), End: point(x1, y1), TEnd: 1}
	}
	outer := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(3, 0, 37, 0), arc(37, 3, 37, 0, 40, 3),
		line(40, 3, 40, 17), arc(37, 17, 40, 17, 37, 20),
		line(37, 20, 3, 20), arc(3, 17, 3, 20, 0, 17),
		line(0, 17, 0, 3), arc(3, 3, 0, 3, 3, 0),
	}}
	hole := sectionrecord.CircleSeg{Center: point(20, 10), Radius: units.Millimeters(3), TStart: 1}
	front := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(3, 0, 37, 0), line(37, 0, 37, 20),
		line(37, 20, 3, 20), line(3, 20, 3, 0),
	}}, Holes: []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{hole}}}}
	back := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(37, 0, 37, 20), line(37, 20, 3, 20),
		line(3, 20, 3, 0), line(3, 0, 37, 0),
	}}, Holes: []sectionrecord.LoopRecord{{Segments: []sectionrecord.CurveSegment{hole}}}}
	faces := []Face{
		{Frame: xy, Wall: outer.Segments[1], Z1: 20},
		{Frame: xy, Wall: outer.Segments[2], Z1: 20},
		{Frame: xy, Wall: outer.Segments[3], Z1: 20},
		{Frame: xy, Wall: outer.Segments[5], Z1: 20},
		{Frame: xy, Wall: outer.Segments[6], Z1: 20},
		{Frame: xy, Wall: outer.Segments[7], Z1: 20},
		{Frame: xy, Region: &momentinput.Profile{Outer: outer}},
		{Frame: xy, Region: &momentinput.Profile{Outer: outer}, Outward: true, Z0: 20, Z1: 20},
		{Frame: xz, Region: &back, Sweep: r3.NewVec(0, 0, 1), Z0: -20, Z1: -20},
		{Frame: xz, Region: &front, Outward: true, Sweep: r3.NewVec(0, 0, 1)},
		{Frame: xz, Wall: hole, Z0: -20},
	}
	want := Payload{Faces: faces, Xform: r3.Identity()}
	want.AssignRoles()
	return want, true
}

// RequireExactSection is brep-modify SB1 (Table RB's RB3): a brep or
// stacked receiver whose record carries a section displacement on any face is
// ErrUnsupported, naming the displacement. It is prismrecord.Payload.RequireExactSection's rule
// over the whole record: a rewrite of a record that denotes its body only to
// within a displacement has no proven displacement of its own. Level
// displacements (z0Delta/z1Delta) are not read; no brep-modify construction
// moves a level.
func RequireExactSection(bp Payload, op string) error {
	delta := bp.SectionDelta()
	if delta == 0 {
		return nil
	}
	return fmt.Errorf(
		`%w: this evaluator %s a brep or stacked receiver whose recorded faces are the faces it denotes only; this body's faces carry a proven section displacement of %g mm from the ones its construction denotes (brep-modify SB1)`,
		decaderr.ErrUnsupported, op, delta,
	)
}

// ThroughCapsCount is the number of planar faces across reference axis k.
func ThroughCapsCount(bp Payload, embeds []brepgeom.Embed, k int) int {
	n := 0
	for fi, f := range bp.Faces {
		if f.Planar() && embeds[fi].Axis[2] == k {
			n++
		}
	}
	return n
}
