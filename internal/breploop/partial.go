package breploop

import (
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// This file is route L's partial-loop arm: a Fillet of a selected
// straight-edge chain on one planar loop, and a Chamfer of one or two
// adjacent straight walks (docs/modify-general-design.md §4.3a). Route E
// prepares the selected edges' end-face corners; these readers move only the
// selected walls, offset only their walks of the common cap loop, and pin each
// chain end's terminal to the cap contour's foot.

// MarkChainTerminals marks each blend end that no other selected blend shares
// as a chain terminal, and every shared end as interior.
func MarkChainTerminals(blends []*breprecord.EdgeBlend) {
	selectedAt := map[[3]float64]int{}
	for _, eb := range blends {
		for _, v := range eb.V {
			selectedAt[v]++
		}
	}
	for _, eb := range blends {
		for k, v := range eb.V {
			eb.Terminal[k] = selectedAt[v] == 1
		}
	}
}

// PartialFilletLines reads the straight walks of the partial loop sel. It
// returns, in loop order, the selection ordinal of each selected straight walk
// whose edge a selected edge matched, and the chain-end vertices where a
// selected straight walk meets an unselected one.
func (r *State) PartialFilletLines(sel Sel, selected []bool) ([]int, map[[3]float64]bool, error) {
	var ordinals []int
	terminals := map[[3]float64]bool{}
	n := len(selected)
	for i, on := range selected {
		if !on {
			continue
		}
		ui := r.LoopUse[[3]int{sel.Face, sel.Loop, i}]
		u := r.Topology.Uses[ui]
		if !u.Walk.IsLine() {
			continue
		}
		if !selected[(i+n-1)%n] {
			terminals[u.From] = true
		}
		if !selected[(i+1)%n] {
			terminals[u.To] = true
		}
		for ordinal, ei := range r.Matched {
			if ei == r.Topology.EdgeOf[ui] {
				ordinals = append(ordinals, ordinal)
				break
			}
		}
	}
	if len(ordinals) == 0 {
		return nil, nil, r.Refuse("SL1", `the selected partial loop has no straight walks`)
	}
	return ordinals, terminals, nil
}

// AdmitPartialChamfer is §4.3a's selection gate on the partial loop sel: the
// call's edges selected edges are one or two adjacent straight walks of a
// loop of at least four walks, each recorded segment its own walk.
func (r *State) AdmitPartialChamfer(sel Sel, selected []bool, edges int) error {
	if edges < 1 || edges > 2 || len(selected) < 4 {
		return r.Refuse("SL1", `a partial chamfer requires one or two adjacent edges with two free ends`)
	}
	var walks []int
	for i, on := range selected {
		if on {
			walks = append(walks, i)
		}
	}
	if len(walks) != edges ||
		len(walks) == 2 &&
			(walks[0]+1)%len(selected) != walks[1] && (walks[1]+1)%len(selected) != walks[0] {
		return r.Refuse("SL1", `a partial chamfer's selected walks are not one chain`)
	}
	cl, err := capblend.OneLoopCornerLoop(r.Budget, r.Payload.Faces[sel.Face].RegionLoop(sel.Loop), freeform.NewFreeformWork())
	if err != nil {
		return err
	}
	if len(cl.Walks) != len(selected) {
		return r.Refuse("SL1", `a partial chamfer's face coalesces recorded walks`)
	}
	for i, on := range selected {
		if on && !cl.Walks[i].IsLine() {
			return r.Refuse("SL1", `a partial chamfer selects a curved walk`)
		}
	}
	return nil
}

// RequirePartialRestated checks that route E's restatement of the straight
// walls kept the partial loop sel: the face's loop keeps its walk count, and
// the matched edges select exactly the walks of selected on it.
func (r *State) RequirePartialRestated(sel Sel, selected []bool) error {
	planar := r.Topology.Planar[sel.Face]
	if sel.Loop >= len(planar) || len(planar[sel.Loop]) != len(selected) {
		return r.Refuse("SL1", `restating the straight walls changed the selected planar loop`)
	}
	again := make([]bool, len(selected))
	for _, ei := range r.Matched {
		if ei < 0 {
			return r.Refuse("SL1", `restating the straight walls lost a selected edge`)
		}
		found := false
		for _, ui := range r.Topology.Edges[ei] {
			u := r.Topology.Uses[ui]
			if u.Face == sel.Face && u.Loop == sel.Loop && u.Part == brepgeom.LoopSeg &&
				u.Seg >= 0 && u.Seg < len(again) {
				again[u.Seg] = true
				found = true
			}
		}
		if !found {
			return r.Refuse("SL1", `restating the straight walls moved a selected edge off its planar loop`)
		}
	}
	if !slices.Equal(selected, again) {
		return r.Refuse("SL1", `restating the straight walls changed the selected partial loop`)
	}
	return nil
}

// RewritePartialFillet records a selected-walk band. The cap contour comes
// from the same per-walk sharp offset used by a subset draft. End walls take
// route E's terminal arcs; selected walls end at the band's side level.
func (r *State) RewritePartialFillet(sel Sel, selected []bool, blends []*breprecord.EdgeBlend) (breprecord.Payload, error) {
	if r.Setback == nil {
		return breprecord.Payload{}, r.Refuse("SL1", `the partial-loop fillet has no radius`)
	}
	if err := r.ClassifyLoop(&sel); err != nil {
		return breprecord.Payload{}, err
	}
	active := sel
	active.Beside = nil
	for i, on := range selected {
		if on {
			active.Beside = append(active.Beside, sel.Beside[i])
		}
	}
	if err := r.RequireBandReach([]Sel{active}); err != nil {
		return breprecord.Payload{}, err
	}
	f := r.Payload.Faces[sel.Face]
	orig := f.RegionLoop(sel.Loop)
	cl, err := capblend.OneLoopCornerLoop(r.Budget, orig, freeform.NewFreeformWork())
	if err != nil {
		return breprecord.Payload{}, err
	}
	if len(cl.Walks) != len(selected) {
		return breprecord.Payload{}, r.Refuse("SL2", `the partial loop coalesces two recorded walks`)
	}
	amounts := make([]float64, len(selected))
	for i, on := range selected {
		if on {
			amounts[i] = r.Setback.Dc
		}
	}
	fr, err := breprecord.FilletLoopOf(r.Budget, orig, r.Setback.Dc, f.Role, freeform.NewFreeformWork())
	if err != nil {
		return breprecord.Payload{}, err
	}
	sphere := make([]bool, len(selected))
	for i, on := range selected {
		if !on {
			continue
		}
		if cl.Walks[i].IsCircular() && !filletband.SphereWalk(cl.Walks[i], amounts[i]) {
			return breprecord.Payload{}, r.Refuse("SL2", `a selected circular walk has no supported partial-loop patch`)
		}
		if !filletband.SphereWalk(cl.Walks[i], amounts[i]) {
			continue
		}
		next := (i + 1) % len(selected)
		if fr.Loop.Corners[i] != filletband.Tangent || fr.Loop.Corners[next] != filletband.Tangent {
			return breprecord.Payload{}, r.Refuse("SL2", `a selected sphere walk has a corner that is not tangent`)
		}
		sphere[i] = true
	}
	segs, joins, capWalk, capArc, err := offset2d.PartialFilletContour(
		r.Budget, cl.Walks, selected, sphere, amounts, offset2d.ShellTol)
	if err != nil {
		return breprecord.Payload{}, r.WrapFace(sel.Face, err)
	}
	profile := momentinput.Profile{Outer: sectionrecord.CloneLoopRecord(f.Region.Outer)}
	for _, h := range f.Region.Holes {
		profile.Holes = append(profile.Holes, sectionrecord.CloneLoopRecord(h))
	}
	if sel.Loop == 0 {
		profile.Outer = sectionrecord.LoopRecord{Segments: segs}
	} else {
		profile.Holes[sel.Loop-1] = sectionrecord.LoopRecord{Segments: segs}
	}
	if err := offset2d.AuditOffsetSectionBudget(r.Budget, *f.Region, profile); err != nil {
		return breprecord.Payload{}, r.WrapFace(sel.Face, err)
	}
	readings := make([]capcontour.Join, len(joins))
	for i, j := range joins {
		readings[i] = capcontour.Join{Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV,
			M:  sectionrecord.Point2{U: j.M.U, V: j.M.V},
			PA: sectionrecord.Point2{U: j.PA.U, V: j.PA.V}, PB: sectionrecord.Point2{U: j.PB.U, V: j.PB.V}}
	}
	delta, err := capband.PartialFilletContourDelta(
		cl.Walks, readings, amounts, sphere, r.Setback.DcDelta, offset2d.ShellTol)
	if err != nil {
		return breprecord.Payload{}, r.WrapFace(sel.Face, err)
	}

	faces := slices.Clone(r.Payload.Faces)
	restore := map[int]breprecord.Face{sel.Face: r.Payload.Faces[sel.Face]}
	faces[sel.Face].Region = &profile
	faces[sel.Face].Delta = math.Max(faces[sel.Face].Delta, delta)
	band := breprecord.LoopBand{Face: sel.Face, Loop: sel.Loop, Orig: sectionrecord.CloneLoopRecord(orig),
		Setback: *r.Setback, Sigma: sel.Sigma, Kind: breprecord.BandFillet,
		Selected: slices.Clone(selected), CapWalk: capWalk, CapArc: capArc, Restore: restore}
	eF := r.Topology.Embeds[sel.Face]
	n := eF.Axis[2]
	sideZ, sideDelta := band.SideLevel(f)
	sideRef := eF.Canon(0, 0, sideZ)[n]
	trims := map[int][]map[int]*offset2d.Blend{}
	for i, on := range selected {
		if !on {
			continue
		}
		b := sel.Beside[i]
		pu := r.Topology.Uses[b.Use]
		if _, ok := restore[pu.Face]; !ok {
			restore[pu.Face] = r.Payload.Faces[pu.Face]
		}
		a := &faces[pu.Face]
		if b.Planar {
			if err := r.TrimPlanar(trims, pu, n, sideRef, r.Setback.Ds); err != nil {
				return breprecord.Payload{}, err
			}
			a.Delta = math.Max(a.Delta, sideDelta)
			continue
		}
		eA := r.Topology.Embeds[pu.Face]
		local := eA.Sign[2]*sideRef + 0
		if pu.Part == brepgeom.Rim0 {
			a.Z0, a.Z0Delta = local, proofbound.AbsSumUpper(math.Max(a.Z0Delta, f.Z0Delta), sideDelta)
		} else {
			a.Z1, a.Z1Delta = local, proofbound.AbsSumUpper(math.Max(a.Z1Delta, f.Z0Delta), sideDelta)
		}
	}

	for _, eb := range blends {
		for k := range eb.V {
			if !eb.Terminal[k] {
				continue
			}
			capLoop := profile.Outer
			if sel.Loop > 0 {
				capLoop = profile.Holes[sel.Loop-1]
			}
			if err := r.partialTerminalBlend(sel, orig, capLoop, capWalk, eb, k, sideRef); err != nil {
				return breprecord.Payload{}, err
			}
			end := eb.End[k]
			if _, ok := restore[end.Face]; !ok {
				restore[end.Face] = r.Payload.Faces[end.Face]
			}
			if trims[end.Face] == nil {
				loops, err := offset2d.ProfileCornerLoopsBudget(r.Budget, *r.Payload.Faces[end.Face].Region)
				if err != nil {
					return breprecord.Payload{}, err
				}
				trims[end.Face] = make([]map[int]*offset2d.Blend, len(loops))
				for i := range trims[end.Face] {
					trims[end.Face][i] = map[int]*offset2d.Blend{}
				}
			}
			if trims[end.Face][end.Loop][end.Corner] != nil {
				return breprecord.Payload{}, r.Refuse("SL2", `two terminal arcs claim one end-face corner`)
			}
			trims[end.Face][end.Loop][end.Corner] = eb.Blend[k]
		}
	}
	var order []int
	for fi := range trims {
		order = append(order, fi)
	}
	slices.Sort(order)
	for _, fi := range order {
		old := r.Payload.Faces[fi]
		loops, err := offset2d.ProfileCornerLoopsBudget(r.Budget, *r.Payload.Faces[fi].Region)
		if err != nil {
			return breprecord.Payload{}, err
		}
		region, connectors, err := offset2d.RewriteProfileBudget(r.Budget, *old.Region, loops, trims[fi])
		if err != nil {
			return breprecord.Payload{}, err
		}
		if err := offset2d.AuditRewriteBudget(r.Budget, *old.Region, region, loops, trims[fi]); err != nil {
			return breprecord.Payload{}, r.WrapFace(fi, err)
		}
		faces[fi].Region = &region
		for li, segs := range connectors {
			for si := range segs {
				band.Terminals = append(band.Terminals, breprecord.BandTerminal{Face: fi, Loop: li, Seg: si})
			}
		}
	}
	if len(band.Terminals) == 0 {
		return breprecord.Payload{}, r.Refuse("SL1", `the selected partial loop has no terminal arcs`)
	}
	out := breprecord.Payload{Faces: faces, Xform: r.Payload.Xform,
		LoopBands: append(slices.Clone(r.Payload.LoopBands), band)}
	out.AssignRoles()
	return out, nil
}

// partialTerminalBlend pins a chain end to the cap contour's actual foot.
// At a hole mouth the selected cap walks move away from the hole, whereas a
// standalone route E corner would trim the end wall toward the hole. The
// terminal arc must use the cap foot shared with the open band's cylinder.
func (r *State) partialTerminalBlend(sel Sel, orig, contour sectionrecord.LoopRecord, capWalk []int,
	eb *breprecord.EdgeBlend, k int, sideLevelRef float64) error {
	v := eb.V[k]
	seg := -1
	for _, ui := range r.Topology.Edges[r.Matched[eb.Ordinal]] {
		u := r.Topology.Uses[ui]
		if u.Face == sel.Face && u.Loop == sel.Loop && u.Part == brepgeom.LoopSeg {
			seg = u.Seg
			break
		}
	}
	if seg < 0 || seg >= len(orig.Segments) || seg >= len(capWalk) ||
		capWalk[seg] < 0 || capWalk[seg] >= len(contour.Segments) {
		return r.Refuse("SL2", `a terminal has no selected cap segment`)
	}
	work := freeform.NewFreeformWork()
	ow, err := boundarywalk.WalkOf(orig.Segments[seg], work)
	if err != nil {
		return err
	}
	cw, err := boundarywalk.WalkOf(contour.Segments[capWalk[seg]], work)
	if err != nil {
		return err
	}
	f := r.Payload.Faces[sel.Face]
	eF := r.Topology.Embeds[sel.Face]
	var capRef [3]float64
	switch {
	case eF.Canon(ow.StartU, ow.StartV, f.Z0) == v:
		capRef = eF.Canon(cw.StartU, cw.StartV, f.Z0)
	case eF.Canon(ow.EndU, ow.EndV, f.Z0) == v:
		capRef = eF.Canon(cw.EndU, cw.EndV, f.Z0)
	default:
		return r.Refuse("SL2", `a terminal vertex is not an end of its cap segment`)
	}
	n := eF.Axis[2]
	sideRef := v
	sideRef[n] = sideLevelRef
	centreRef := capRef
	centreRef[n] = sideRef[n]
	endFace := r.Payload.Faces[eb.End[k].Face]
	eEnd := r.Topology.Embeds[eb.End[k].Face]
	capLocal, sideLocal, centreLocal := eEnd.Local(capRef), eEnd.Local(sideRef), eEnd.Local(centreRef)
	if capLocal[2] != endFace.Z0 || sideLocal[2] != endFace.Z0 || centreLocal[2] != endFace.Z0 {
		return r.Refuse("SL2", `a terminal cap foot leaves its end face`)
	}
	capPoint := sectionrecord.Point2{U: capLocal[0], V: capLocal[1]}
	sidePoint := sectionrecord.Point2{U: sideLocal[0], V: sideLocal[1]}
	centre := sectionrecord.Point2{U: centreLocal[0], V: centreLocal[1]}
	if math.Abs(math.Hypot(sidePoint.U-centre.U, sidePoint.V-centre.V)-r.Setback.Dc) > sectionaudit.Tolerance {
		return r.Refuse("SL2", `a terminal wall meets the selected pipe along a noncircular curve`)
	}
	blend := eb.Blend[k]
	oldA := eEnd.Canon(blend.FA.U, blend.FA.V, endFace.Z0)
	oldB := eEnd.Canon(blend.FB.U, blend.FB.V, endFace.Z0)
	levelRef := v[n]
	switch {
	case oldA[n] == levelRef && oldB[n] == sideRef[n]:
		blend.FA, blend.FB = capPoint, sidePoint
	case oldB[n] == levelRef && oldA[n] == sideRef[n]:
		blend.FA, blend.FB = sidePoint, capPoint
	default:
		return r.Refuse("SL2", `a terminal blend has no cap and side feet`)
	}
	corner := eEnd.Local(v)
	blend.CutbackA = math.Hypot(blend.FA.U-corner[0], blend.FA.V-corner[1])
	blend.CutbackB = math.Hypot(blend.FB.U-corner[0], blend.FB.V-corner[1])
	ax, ay := blend.FA.U-centre.U, blend.FA.V-centre.V
	bx, by := blend.FB.U-centre.U, blend.FB.V-centre.V
	cross := ax*by - ay*bx
	if cross == 0 {
		return r.Refuse("SL2", `a terminal arc has no turn`)
	}
	blend.Connector = offset2d.ArcSegment(centre, blend.FA, blend.FB, cross > 0)
	return nil
}

// RewritePartialChamfer records §4.3a's selected-walk chamfer band. Each
// selected wall ends at the band's side level, the planar face keeps the
// contour with only the selected walks offset, and each free end takes a
// straight terminal line on its end face.
func (r *State) RewritePartialChamfer(sel Sel, selected []bool,
	blends []*breprecord.EdgeBlend) (breprecord.Payload, error) {
	if r.Setback == nil {
		return breprecord.Payload{}, r.Refuse("SL1", `the partial chamfer has no setback`)
	}
	if err := r.ClassifyLoop(&sel); err != nil {
		return breprecord.Payload{}, err
	}
	active := sel
	active.Beside = nil
	for i, on := range selected {
		if on {
			active.Beside = append(active.Beside, sel.Beside[i])
		}
	}
	if err := r.RequireBandReach([]Sel{active}); err != nil {
		return breprecord.Payload{}, err
	}
	f := r.Payload.Faces[sel.Face]
	orig := f.RegionLoop(sel.Loop)
	cl, err := capblend.OneLoopCornerLoop(r.Budget, orig, freeform.NewFreeformWork())
	if err != nil {
		return breprecord.Payload{}, err
	}
	amounts := make([]float64, len(selected))
	for i, on := range selected {
		if on {
			amounts[i] = r.Setback.Dc
		}
	}
	segs, joins, capWalk, capArc, err := offset2d.PartialChamferContour(
		r.Budget, cl.Walks, selected, amounts, offset2d.ShellTol)
	if err != nil {
		return breprecord.Payload{}, r.WrapFace(sel.Face, err)
	}
	for _, arc := range capArc {
		if arc >= 0 {
			return breprecord.Payload{}, r.Refuse("SL1", `a partial chamfer has a curved connector`)
		}
	}
	profile := momentinput.Profile{Outer: sectionrecord.CloneLoopRecord(f.Region.Outer)}
	for _, h := range f.Region.Holes {
		profile.Holes = append(profile.Holes, sectionrecord.CloneLoopRecord(h))
	}
	if sel.Loop == 0 {
		profile.Outer = sectionrecord.LoopRecord{Segments: segs}
	} else {
		profile.Holes[sel.Loop-1] = sectionrecord.LoopRecord{Segments: segs}
	}
	if err := offset2d.AuditOffsetSectionBudget(r.Budget, *f.Region, profile); err != nil {
		return breprecord.Payload{}, r.WrapFace(sel.Face, err)
	}
	readings := make([]capcontour.Join, len(joins))
	for i, j := range joins {
		readings[i] = capcontour.Join{Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV,
			M:  sectionrecord.Point2{U: j.M.U, V: j.M.V},
			PA: sectionrecord.Point2{U: j.PA.U, V: j.PA.V}, PB: sectionrecord.Point2{U: j.PB.U, V: j.PB.V}}
	}
	delta, err := capband.AmountsContourDisplacement(cl.Walks, readings, amounts,
		r.Setback.DcDelta, offset2d.ShellTol)
	if err != nil {
		return breprecord.Payload{}, r.WrapFace(sel.Face, err)
	}
	faces := slices.Clone(r.Payload.Faces)
	restore := map[int]breprecord.Face{sel.Face: r.Payload.Faces[sel.Face]}
	faces[sel.Face].Region = &profile
	faces[sel.Face].Delta = math.Max(faces[sel.Face].Delta, delta)
	band := breprecord.LoopBand{Face: sel.Face, Loop: sel.Loop, Orig: sectionrecord.CloneLoopRecord(orig),
		Setback: *r.Setback, Sigma: sel.Sigma, Kind: breprecord.BandChamfer,
		Selected: slices.Clone(selected), CapWalk: capWalk, CapArc: capArc, Restore: restore}
	eF := r.Topology.Embeds[sel.Face]
	n := eF.Axis[2]
	sideZ, sideDelta := band.SideLevel(f)
	sideRef := eF.Canon(0, 0, sideZ)[n]
	trims := map[int][]map[int]*offset2d.Blend{}
	for i, on := range selected {
		if !on {
			continue
		}
		b := sel.Beside[i]
		pu := r.Topology.Uses[b.Use]
		if _, ok := restore[pu.Face]; !ok {
			restore[pu.Face] = r.Payload.Faces[pu.Face]
		}
		a := &faces[pu.Face]
		if b.Planar {
			if err := r.TrimPlanar(trims, pu, n, sideRef, r.Setback.Ds); err != nil {
				return breprecord.Payload{}, err
			}
			a.Delta = math.Max(a.Delta, sideDelta)
			continue
		}
		eA := r.Topology.Embeds[pu.Face]
		local := eA.Sign[2] * sideRef
		if pu.Part == brepgeom.Rim0 {
			a.Z0, a.Z0Delta = local, proofbound.AbsSumUpper(math.Max(a.Z0Delta, f.Z0Delta), sideDelta)
		} else {
			a.Z1, a.Z1Delta = local, proofbound.AbsSumUpper(math.Max(a.Z1Delta, f.Z0Delta), sideDelta)
		}
	}
	for _, eb := range blends {
		for k := range eb.V {
			if !eb.Terminal[k] {
				continue
			}
			capLoop := profile.Outer
			if sel.Loop > 0 {
				capLoop = profile.Holes[sel.Loop-1]
			}
			if err := r.partialTerminalChamfer(sel, orig, capLoop, capWalk, eb, k, sideRef); err != nil {
				return breprecord.Payload{}, err
			}
			end := eb.End[k]
			if _, ok := restore[end.Face]; !ok {
				restore[end.Face] = r.Payload.Faces[end.Face]
			}
			if trims[end.Face] == nil {
				loops, err := offset2d.ProfileCornerLoopsBudget(r.Budget, *r.Payload.Faces[end.Face].Region)
				if err != nil {
					return breprecord.Payload{}, err
				}
				trims[end.Face] = make([]map[int]*offset2d.Blend, len(loops))
				for i := range trims[end.Face] {
					trims[end.Face][i] = map[int]*offset2d.Blend{}
				}
			}
			if trims[end.Face][end.Loop][end.Corner] != nil {
				return breprecord.Payload{}, r.Refuse("SL1", `two terminal lines claim one end-face corner`)
			}
			trims[end.Face][end.Loop][end.Corner] = eb.Blend[k]
		}
	}
	var order []int
	for fi := range trims {
		order = append(order, fi)
	}
	slices.Sort(order)
	for _, fi := range order {
		old := r.Payload.Faces[fi]
		loops, err := offset2d.ProfileCornerLoopsBudget(r.Budget, *old.Region)
		if err != nil {
			return breprecord.Payload{}, err
		}
		region, connectors, err := offset2d.RewriteProfileBudget(r.Budget, *old.Region, loops, trims[fi])
		if err != nil {
			return breprecord.Payload{}, err
		}
		if err := offset2d.AuditRewriteBudget(r.Budget, *old.Region, region, loops, trims[fi]); err != nil {
			return breprecord.Payload{}, r.WrapFace(fi, err)
		}
		faces[fi].Region = &region
		for li, segs := range connectors {
			for si := range segs {
				band.Terminals = append(band.Terminals, breprecord.BandTerminal{Face: fi, Loop: li, Seg: si})
			}
		}
	}
	if len(band.Terminals) != 2 {
		return breprecord.Payload{}, r.Refuse("SL1", `a two-edge partial chamfer has no two terminal lines`)
	}
	out := breprecord.Payload{Faces: faces, Xform: r.Payload.Xform,
		LoopBands: append(slices.Clone(r.Payload.LoopBands), band)}
	out.AssignRoles()
	return out, nil
}

// partialTerminalChamfer places an end-face line on the selected band's two
// directrices. It replaces route E's independent corner feet after the
// common-face contour has fixed the cap foot.
func (r *State) partialTerminalChamfer(sel Sel, orig, contour sectionrecord.LoopRecord,
	capWalk []int, eb *breprecord.EdgeBlend, k int, sideRef float64) error {
	v := eb.V[k]
	seg := -1
	for _, ui := range r.Topology.Edges[r.Matched[eb.Ordinal]] {
		u := r.Topology.Uses[ui]
		if u.Face == sel.Face && u.Loop == sel.Loop && u.Part == brepgeom.LoopSeg {
			seg = u.Seg
			break
		}
	}
	if seg < 0 || capWalk[seg] < 0 || capWalk[seg] >= len(contour.Segments) {
		return r.Refuse("SL1", `a terminal has no selected cap segment`)
	}
	work := freeform.NewFreeformWork()
	ow, err := capblend.OneLoopCornerLoop(r.Budget,
		sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{orig.Segments[seg]}}, work)
	if err != nil {
		return err
	}
	cw, err := capblend.OneLoopCornerLoop(r.Budget,
		sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{contour.Segments[capWalk[seg]]}}, work)
	if err != nil {
		return err
	}
	if len(ow.Walks) != 1 || len(cw.Walks) != 1 {
		return r.Refuse("SL1", `a terminal is not a single straight walk`)
	}
	f := r.Payload.Faces[sel.Face]
	eF := r.Topology.Embeds[sel.Face]
	var capRef [3]float64
	switch {
	case eF.Canon(ow.Walks[0].StartU, ow.Walks[0].StartV, f.Z0) == v:
		capRef = eF.Canon(cw.Walks[0].StartU, cw.Walks[0].StartV, f.Z0)
	case eF.Canon(ow.Walks[0].EndU, ow.Walks[0].EndV, f.Z0) == v:
		capRef = eF.Canon(cw.Walks[0].EndU, cw.Walks[0].EndV, f.Z0)
	default:
		return r.Refuse("SL1", `a terminal vertex is not an end of its cap segment`)
	}
	n := eF.Axis[2]
	side := v
	side[n] = sideRef
	endFace := r.Payload.Faces[eb.End[k].Face]
	eEnd := r.Topology.Embeds[eb.End[k].Face]
	capLocal, sideLocal := eEnd.Local(capRef), eEnd.Local(side)
	if capLocal[2] != endFace.Z0 || sideLocal[2] != endFace.Z0 {
		return r.Refuse("SL1", `a terminal line leaves its end face`)
	}
	blend := eb.Blend[k]
	oldA, oldB := eEnd.Canon(blend.FA.U, blend.FA.V, endFace.Z0),
		eEnd.Canon(blend.FB.U, blend.FB.V, endFace.Z0)
	level := v[n]
	capPoint := sectionrecord.Point2{U: capLocal[0], V: capLocal[1]}
	sidePoint := sectionrecord.Point2{U: sideLocal[0], V: sideLocal[1]}
	switch {
	case oldA[n] == level && oldB[n] == sideRef:
		blend.FA, blend.FB = capPoint, sidePoint
	case oldB[n] == level && oldA[n] == sideRef:
		blend.FA, blend.FB = sidePoint, capPoint
	default:
		return r.Refuse("SL1", `a terminal blend has no cap and side feet`)
	}
	blend.Connector = sectionrecord.LineSeg{Start: blend.FA, End: blend.FB, TStart: 0, TEnd: 1}
	corner := eEnd.Local(v)
	blend.CutbackA = math.Hypot(blend.FA.U-corner[0], blend.FA.V-corner[1])
	blend.CutbackB = math.Hypot(blend.FB.U-corner[0], blend.FB.V-corner[1])
	return nil
}
