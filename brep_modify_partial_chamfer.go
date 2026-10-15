package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// brepChamferPartialLoop admits one or two adjacent selected straight walks
// on a planar face. The chain's two vertices end on unselected straight walks.
func brepChamferPartialLoop(ctx context.Context, d *Document, bp brepPayload,
	call brepModifyRequest, sel brepLoopSel, selected []bool) (*Body, error) {
	initial, err := newBrepLoopRead(ctx, bp, call)
	if err != nil {
		return nil, err
	}
	if err := initial.matchEdges(); err != nil {
		return nil, err
	}
	if len(call.edges) < 1 || len(call.edges) > 2 || len(selected) < 4 {
		return nil, initial.refuse("SL1", `a partial chamfer requires one or two adjacent edges with two free ends`)
	}
	var walks []int
	for i, on := range selected {
		if on {
			walks = append(walks, i)
		}
	}
	if len(walks) != len(call.edges) ||
		len(walks) == 2 &&
			(walks[0]+1)%len(selected) != walks[1] && (walks[1]+1)%len(selected) != walks[0] {
		return nil, initial.refuse("SL1", `a partial chamfer's selected walks are not one chain`)
	}
	cl, err := capblend.OneLoopCornerLoop(initial.budget, bp.Faces[sel.face].RegionLoop(sel.loop), freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if len(cl.Walks) != len(selected) {
		return nil, initial.refuse("SL1", `a partial chamfer's face coalesces recorded walks`)
	}
	for i, on := range selected {
		if on && !cl.Walks[i].IsLine() {
			return nil, initial.refuse("SL1", `a partial chamfer selects a curved walk`)
		}
	}
	// The two selected edges must be a proper chain with two terminal vertices.
	// Route E supplies the two end-face corner rewrites and its topology gates.
	er, blends, err := prepareBrepEdgeBlends(ctx, bp, call, true, nil)
	if err != nil {
		return nil, err
	}
	selectedAt := map[[3]float64]int{}
	for _, eb := range blends {
		for _, v := range eb.v {
			selectedAt[v]++
		}
	}
	for _, eb := range blends {
		for k, v := range eb.v {
			eb.terminal[k] = selectedAt[v] == 1
		}
	}
	r, err := newBrepLoopRead(ctx, er.bp, call)
	if err != nil {
		return nil, err
	}
	if err := r.matchEdges(); err != nil {
		return nil, err
	}
	planar := r.btopo.Planar[sel.face]
	if sel.loop >= len(planar) || len(planar[sel.loop]) != len(selected) {
		return nil, r.refuse("SL1", `restating the straight walls changed the selected partial loop`)
	}
	again := make([]bool, len(selected))
	for _, ei := range r.matched {
		if ei < 0 {
			return nil, r.refuse("SL1", `restating the straight walls lost a selected edge`)
		}
		found := false
		for _, ui := range r.btopo.Edges[ei] {
			u := r.btopo.Uses[ui]
			if u.Face == sel.face && u.Loop == sel.loop && u.Part == brepLoopSeg &&
				u.Seg >= 0 && u.Seg < len(again) {
				again[u.Seg] = true
				found = true
			}
		}
		if !found {
			return nil, r.refuse("SL1", `restating the straight walls moved a selected edge off its planar loop`)
		}
	}
	if !slices.Equal(selected, again) {
		return nil, r.refuse("SL1", `restating the straight walls changed the selected partial loop`)
	}
	out, err := r.rewritePartialChamfer(sel, selected, blends)
	if err != nil {
		return nil, err
	}
	body, err := bodyResult(breprecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), out))
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the partial-loop chamfer's rewritten brep record`, err)
	}
	return body, nil
}

func (r *brepLoopRead) rewritePartialChamfer(sel brepLoopSel, selected []bool,
	blends []*brepEdgeBlend) (brepPayload, error) {
	if r.call.loop == nil {
		return brepPayload{}, r.refuse("SL1", `the partial chamfer has no setback`)
	}
	if err := r.classifyLoop(&sel); err != nil {
		return brepPayload{}, err
	}
	active := sel
	active.beside = nil
	for i, on := range selected {
		if on {
			active.beside = append(active.beside, sel.beside[i])
		}
	}
	if err := r.requireBandReach([]brepLoopSel{active}); err != nil {
		return brepPayload{}, err
	}
	f := r.bp.Faces[sel.face]
	orig := f.RegionLoop(sel.loop)
	cl, err := capblend.OneLoopCornerLoop(r.budget, orig, freeform.NewFreeformWork())
	if err != nil {
		return brepPayload{}, err
	}
	amounts := make([]float64, len(selected))
	for i, on := range selected {
		if on {
			amounts[i] = r.call.loop.Dc
		}
	}
	segs, joins, capWalk, capArc, err := offset2d.PartialChamferContour(
		r.budget, cl.Walks, selected, amounts, shellTol)
	if err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
	}
	for _, arc := range capArc {
		if arc >= 0 {
			return brepPayload{}, r.refuse("SL1", `a partial chamfer has a curved connector`)
		}
	}
	profile := profileRecord{Outer: cloneLoopRecord(f.Region.Outer)}
	for _, h := range f.Region.Holes {
		profile.Holes = append(profile.Holes, cloneLoopRecord(h))
	}
	if sel.loop == 0 {
		profile.Outer = loopRecord{Segments: segs}
	} else {
		profile.Holes[sel.loop-1] = loopRecord{Segments: segs}
	}
	if err := offset2d.AuditOffsetSectionBudget(r.budget, *f.Region, profile); err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
	}
	readings := make([]capcontour.Join, len(joins))
	for i, j := range joins {
		readings[i] = capcontour.Join{Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV,
			M:  planePoint{U: j.M.U, V: j.M.V},
			PA: planePoint{U: j.PA.U, V: j.PA.V}, PB: planePoint{U: j.PB.U, V: j.PB.V}}
	}
	delta, err := capband.AmountsContourDisplacement(cl.Walks, readings, amounts,
		r.call.loop.DcDelta, shellTol)
	if err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
	}
	faces := slices.Clone(r.bp.Faces)
	restore := map[int]brepFace{sel.face: r.bp.Faces[sel.face]}
	faces[sel.face].Region = &profile
	faces[sel.face].Delta = math.Max(faces[sel.face].Delta, delta)
	band := brepLoopBand{Face: sel.face, Loop: sel.loop, Orig: cloneLoopRecord(orig),
		Setback: *r.call.loop, Sigma: sel.sigma, Kind: breprecord.BandChamfer,
		Selected: slices.Clone(selected), CapWalk: capWalk, CapArc: capArc, Restore: restore}
	eF := r.btopo.Embeds[sel.face]
	n := eF.Axis[2]
	sideZ, sideDelta := band.SideLevel(f)
	sideRef := eF.Canon(0, 0, sideZ)[n]
	trims := map[int][]map[int]*cornerBlend{}
	for i, on := range selected {
		if !on {
			continue
		}
		b := sel.beside[i]
		pu := r.btopo.Uses[b.use]
		if _, ok := restore[pu.Face]; !ok {
			restore[pu.Face] = r.bp.Faces[pu.Face]
		}
		a := &faces[pu.Face]
		if b.planar {
			if err := r.trimPlanar(trims, pu, n, sideRef, r.call.loop.Ds); err != nil {
				return brepPayload{}, err
			}
			a.Delta = math.Max(a.Delta, sideDelta)
			continue
		}
		eA := r.btopo.Embeds[pu.Face]
		local := eA.Sign[2] * sideRef
		if pu.Part == brepRim0 {
			a.Z0, a.Z0Delta = local, proofbound.AbsSumUpper(math.Max(a.Z0Delta, f.Z0Delta), sideDelta)
		} else {
			a.Z1, a.Z1Delta = local, proofbound.AbsSumUpper(math.Max(a.Z1Delta, f.Z0Delta), sideDelta)
		}
	}
	for _, eb := range blends {
		for k := range eb.v {
			if !eb.terminal[k] {
				continue
			}
			capLoop := profile.Outer
			if sel.loop > 0 {
				capLoop = profile.Holes[sel.loop-1]
			}
			if err := r.partialTerminalChamfer(sel, orig, capLoop, capWalk, eb, k, sideRef); err != nil {
				return brepPayload{}, err
			}
			end := eb.end[k]
			if _, ok := restore[end.face]; !ok {
				restore[end.face] = r.bp.Faces[end.face]
			}
			if trims[end.face] == nil {
				loops, err := offset2d.ProfileCornerLoopsBudget(r.budget, *r.bp.Faces[end.face].Region)
				if err != nil {
					return brepPayload{}, err
				}
				trims[end.face] = make([]map[int]*cornerBlend, len(loops))
				for i := range trims[end.face] {
					trims[end.face][i] = map[int]*cornerBlend{}
				}
			}
			if trims[end.face][end.loop][end.corner] != nil {
				return brepPayload{}, r.refuse("SL1", `two terminal lines claim one end-face corner`)
			}
			trims[end.face][end.loop][end.corner] = eb.blend[k]
		}
	}
	var order []int
	for fi := range trims {
		order = append(order, fi)
	}
	slices.Sort(order)
	for _, fi := range order {
		old := r.bp.Faces[fi]
		loops, err := offset2d.ProfileCornerLoopsBudget(r.budget, *old.Region)
		if err != nil {
			return brepPayload{}, err
		}
		region, connectors, err := rewriteProfileBudget(r.budget, *old.Region, loops, trims[fi])
		if err != nil {
			return brepPayload{}, err
		}
		if err := offset2d.AuditRewriteBudget(r.budget, *old.Region, region, loops, trims[fi]); err != nil {
			return brepPayload{}, r.wrapFace(fi, err)
		}
		faces[fi].Region = &region
		for li, segs := range connectors {
			for si := range segs {
				band.Terminals = append(band.Terminals, brepBandTerminal{Face: fi, Loop: li, Seg: si})
			}
		}
	}
	if len(band.Terminals) != 2 {
		return brepPayload{}, r.refuse("SL1", `a two-edge partial chamfer has no two terminal lines`)
	}
	out := brepPayload{Faces: faces, Xform: r.bp.Xform,
		LoopBands: append(slices.Clone(r.bp.LoopBands), band)}
	out.AssignRoles()
	return out, nil
}

// partialTerminalChamfer places an end-face line on the selected band's two
// directrices. It replaces route E's independent corner feet after the
// common-face contour has fixed the cap foot.
func (r *brepLoopRead) partialTerminalChamfer(sel brepLoopSel, orig, contour loopRecord,
	capWalk []int, eb *brepEdgeBlend, k int, sideRef float64) error {
	v := eb.v[k]
	seg := -1
	for _, ui := range r.btopo.Edges[r.matched[eb.ordinal]] {
		u := r.btopo.Uses[ui]
		if u.Face == sel.face && u.Loop == sel.loop && u.Part == brepLoopSeg {
			seg = u.Seg
			break
		}
	}
	if seg < 0 || capWalk[seg] < 0 || capWalk[seg] >= len(contour.Segments) {
		return r.refuse("SL1", `a terminal has no selected cap segment`)
	}
	work := freeform.NewFreeformWork()
	ow, err := capblend.OneLoopCornerLoop(r.budget, loopRecord{Segments: []curveSegment{orig.Segments[seg]}}, work)
	if err != nil {
		return err
	}
	cw, err := capblend.OneLoopCornerLoop(r.budget, loopRecord{Segments: []curveSegment{contour.Segments[capWalk[seg]]}}, work)
	if err != nil {
		return err
	}
	if len(ow.Walks) != 1 || len(cw.Walks) != 1 {
		return r.refuse("SL1", `a terminal is not a single straight walk`)
	}
	f := r.bp.Faces[sel.face]
	eF := r.btopo.Embeds[sel.face]
	var capRef [3]float64
	switch {
	case eF.Canon(ow.Walks[0].StartU, ow.Walks[0].StartV, f.Z0) == v:
		capRef = eF.Canon(cw.Walks[0].StartU, cw.Walks[0].StartV, f.Z0)
	case eF.Canon(ow.Walks[0].EndU, ow.Walks[0].EndV, f.Z0) == v:
		capRef = eF.Canon(cw.Walks[0].EndU, cw.Walks[0].EndV, f.Z0)
	default:
		return r.refuse("SL1", `a terminal vertex is not an end of its cap segment`)
	}
	n := eF.Axis[2]
	side := v
	side[n] = sideRef
	endFace := r.bp.Faces[eb.end[k].face]
	eEnd := r.btopo.Embeds[eb.end[k].face]
	capLocal, sideLocal := eEnd.Local(capRef), eEnd.Local(side)
	if capLocal[2] != endFace.Z0 || sideLocal[2] != endFace.Z0 {
		return r.refuse("SL1", `a terminal line leaves its end face`)
	}
	blend := eb.blend[k]
	oldA, oldB := eEnd.Canon(blend.FA.U, blend.FA.V, endFace.Z0),
		eEnd.Canon(blend.FB.U, blend.FB.V, endFace.Z0)
	level := v[n]
	switch {
	case oldA[n] == level && oldB[n] == sideRef:
		blend.FA, blend.FB = planePoint{U: capLocal[0], V: capLocal[1]}, planePoint{U: sideLocal[0], V: sideLocal[1]}
	case oldB[n] == level && oldA[n] == sideRef:
		blend.FA, blend.FB = planePoint{U: sideLocal[0], V: sideLocal[1]}, planePoint{U: capLocal[0], V: capLocal[1]}
	default:
		return r.refuse("SL1", `a terminal blend has no cap and side feet`)
	}
	blend.Connector = sectionrecord.LineSeg{Start: blend.FA, End: blend.FB, TStart: 0, TEnd: 1}
	corner := eEnd.Local(v)
	blend.CutbackA = math.Hypot(blend.FA.U-corner[0], blend.FA.V-corner[1])
	blend.CutbackB = math.Hypot(blend.FB.U-corner[0], blend.FB.V-corner[1])
	return nil
}
