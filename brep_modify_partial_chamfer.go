package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// brepChamferPartialLoop admits one pair of adjacent selected straight walks
// on a planar face. The other two vertices end on unselected straight walks.
func brepChamferPartialLoop(ctx context.Context, d *Document, bp brepPayload,
	call brepModifyRequest, sel brepLoopSel, selected []bool) (*Body, error) {
	initial, err := newBrepLoopRead(ctx, bp, call)
	if err != nil {
		return nil, err
	}
	if err := initial.matchEdges(); err != nil {
		return nil, err
	}
	if len(call.edges) != 2 || len(selected) < 4 {
		return nil, initial.refuse("SL1", `a partial chamfer requires exactly two adjacent edges with two free ends`)
	}
	var walks []int
	for i, on := range selected {
		if on {
			walks = append(walks, i)
		}
	}
	if len(walks) != 2 ||
		(walks[0]+1)%len(selected) != walks[1] && (walks[1]+1)%len(selected) != walks[0] {
		return nil, initial.refuse("SL1", `a partial chamfer's two walks are not adjacent`)
	}
	cl, err := oneLoopCornerLoop(initial.budget, bp.faces[sel.face].regionLoop(sel.loop), freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if len(cl.walks) != len(selected) {
		return nil, initial.refuse("SL1", `a partial chamfer's face coalesces recorded walks`)
	}
	for i, on := range selected {
		if on && !cl.walks[i].IsLine() {
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
	againSel, again, ok := r.partialSelectedLoop()
	if !ok || againSel.face != sel.face || againSel.loop != sel.loop || !slices.Equal(selected, again) {
		return nil, r.refuse("SL1", `restating the straight walls changed the selected partial loop`)
	}
	out, err := r.rewritePartialChamfer(sel, selected, blends)
	if err != nil {
		return nil, err
	}
	body, err := evalBrepContext(ctx, d, d.nextProducerID(), out)
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
	f := r.bp.faces[sel.face]
	orig := f.regionLoop(sel.loop)
	cl, err := oneLoopCornerLoop(r.budget, orig, freeform.NewFreeformWork())
	if err != nil {
		return brepPayload{}, err
	}
	amounts := make([]float64, len(selected))
	for i, on := range selected {
		if on {
			amounts[i] = r.call.loop.dc
		}
	}
	segs, joins, capWalk, capArc, err := offset2d.PartialFilletContour(
		r.budget, cl.walks, selected, make([]bool, len(selected)), amounts, shellTol)
	if err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
	}
	for _, arc := range capArc {
		if arc >= 0 {
			return brepPayload{}, r.refuse("SL1", `a partial chamfer has a curved connector`)
		}
	}
	profile := profileRecord{Outer: cloneLoopRecord(f.region.Outer)}
	for _, h := range f.region.Holes {
		profile.Holes = append(profile.Holes, cloneLoopRecord(h))
	}
	if sel.loop == 0 {
		profile.Outer = loopRecord{Segments: segs}
	} else {
		profile.Holes[sel.loop-1] = loopRecord{Segments: segs}
	}
	if err := auditOffsetSectionBudget(r.budget, *f.region, profile); err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
	}
	readings := make([]capcontour.Join, len(joins))
	for i, j := range joins {
		readings[i] = capcontour.Join{Arc: j.Arc, G1: j.G1, VU: j.VertU, VV: j.VertV,
			M:  Point2{U: j.M.U, V: j.M.V},
			PA: Point2{U: j.PA.U, V: j.PA.V}, PB: Point2{U: j.PB.U, V: j.PB.V}}
	}
	delta, err := capband.AmountsContourDisplacement(cl.walks, readings, amounts,
		r.call.loop.dcDelta, shellTol)
	if err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
	}
	faces := slices.Clone(r.bp.faces)
	restore := map[int]brepFace{sel.face: r.bp.faces[sel.face]}
	faces[sel.face].region = &profile
	faces[sel.face].delta = math.Max(faces[sel.face].delta, delta)
	band := brepLoopBand{face: sel.face, loop: sel.loop, orig: cloneLoopRecord(orig),
		setback: *r.call.loop, sigma: sel.sigma, kind: brepBandChamfer,
		selected: slices.Clone(selected), capWalk: capWalk, capArc: capArc, restore: restore}
	eF := r.topo.embeds[sel.face]
	n := eF.Axis[2]
	sideZ, sideDelta := band.sideLevel(f)
	sideRef := eF.Canon(0, 0, sideZ)[n]
	trims := map[int][]map[int]*cornerBlend{}
	for i, on := range selected {
		if !on {
			continue
		}
		b := sel.beside[i]
		pu := r.topo.uses[b.use]
		if _, ok := restore[pu.Face]; !ok {
			restore[pu.Face] = r.bp.faces[pu.Face]
		}
		a := &faces[pu.Face]
		if b.planar {
			if err := r.trimPlanar(trims, pu, n, sideRef, r.call.loop.ds); err != nil {
				return brepPayload{}, err
			}
			a.delta = math.Max(a.delta, sideDelta)
			continue
		}
		eA := r.topo.embeds[pu.Face]
		local := eA.Sign[2] * sideRef
		if pu.Part == brepRim0 {
			a.z0, a.z0Delta = local, proofbound.AbsSumUpper(math.Max(a.z0Delta, f.z0Delta), sideDelta)
		} else {
			a.z1, a.z1Delta = local, proofbound.AbsSumUpper(math.Max(a.z1Delta, f.z0Delta), sideDelta)
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
				restore[end.face] = r.bp.faces[end.face]
			}
			if trims[end.face] == nil {
				loops, err := profileCornerLoopsBudget(r.budget, *r.bp.faces[end.face].region)
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
		old := r.bp.faces[fi]
		loops, err := profileCornerLoopsBudget(r.budget, *old.region)
		if err != nil {
			return brepPayload{}, err
		}
		region, connectors, err := rewriteProfileBudget(r.budget, *old.region, loops, trims[fi])
		if err != nil {
			return brepPayload{}, err
		}
		if err := auditRewriteBudget(r.budget, *old.region, region, loops, trims[fi]); err != nil {
			return brepPayload{}, r.wrapFace(fi, err)
		}
		faces[fi].region = &region
		for li, segs := range connectors {
			for si := range segs {
				band.terminals = append(band.terminals, brepBandTerminal{face: fi, loop: li, seg: si})
			}
		}
	}
	if len(band.terminals) != 2 {
		return brepPayload{}, r.refuse("SL1", `a two-edge partial chamfer has no two terminal lines`)
	}
	out := brepPayload{faces: faces, xform: r.bp.xform,
		loopBands: append(slices.Clone(r.bp.loopBands), band)}
	out.assignRoles()
	return out, nil
}

// partialTerminalChamfer places an end-face line on the selected band's two
// directrices. It replaces route E's independent corner feet after the
// common-face contour has fixed the cap foot.
func (r *brepLoopRead) partialTerminalChamfer(sel brepLoopSel, orig, contour loopRecord,
	capWalk []int, eb *brepEdgeBlend, k int, sideRef float64) error {
	v := eb.v[k]
	seg := -1
	for _, ui := range r.topo.edges[r.matched[eb.ordinal]] {
		u := r.topo.uses[ui]
		if u.Face == sel.face && u.Loop == sel.loop && u.Part == brepLoopSeg {
			seg = u.Seg
			break
		}
	}
	if seg < 0 || capWalk[seg] < 0 || capWalk[seg] >= len(contour.Segments) {
		return r.refuse("SL1", `a terminal has no selected cap segment`)
	}
	work := freeform.NewFreeformWork()
	ow, err := oneLoopCornerLoop(r.budget, loopRecord{Segments: []curveSegment{orig.Segments[seg]}}, work)
	if err != nil {
		return err
	}
	cw, err := oneLoopCornerLoop(r.budget, loopRecord{Segments: []curveSegment{contour.Segments[capWalk[seg]]}}, work)
	if err != nil {
		return err
	}
	if len(ow.walks) != 1 || len(cw.walks) != 1 {
		return r.refuse("SL1", `a terminal is not a single straight walk`)
	}
	f := r.bp.faces[sel.face]
	eF := r.topo.embeds[sel.face]
	var capRef [3]float64
	switch {
	case eF.Canon(ow.walks[0].StartU, ow.walks[0].StartV, f.z0) == v:
		capRef = eF.Canon(cw.walks[0].StartU, cw.walks[0].StartV, f.z0)
	case eF.Canon(ow.walks[0].EndU, ow.walks[0].EndV, f.z0) == v:
		capRef = eF.Canon(cw.walks[0].EndU, cw.walks[0].EndV, f.z0)
	default:
		return r.refuse("SL1", `a terminal vertex is not an end of its cap segment`)
	}
	n := eF.Axis[2]
	side := v
	side[n] = sideRef
	endFace := r.bp.faces[eb.end[k].face]
	eEnd := r.topo.embeds[eb.end[k].face]
	capLocal, sideLocal := eEnd.Local(capRef), eEnd.Local(side)
	if capLocal[2] != endFace.z0 || sideLocal[2] != endFace.z0 {
		return r.refuse("SL1", `a terminal line leaves its end face`)
	}
	blend := eb.blend[k]
	oldA, oldB := eEnd.Canon(blend.FA.U, blend.FA.V, endFace.z0),
		eEnd.Canon(blend.FB.U, blend.FB.V, endFace.z0)
	level := v[n]
	switch {
	case oldA[n] == level && oldB[n] == sideRef:
		blend.FA, blend.FB = Point2{U: capLocal[0], V: capLocal[1]}, Point2{U: sideLocal[0], V: sideLocal[1]}
	case oldB[n] == level && oldA[n] == sideRef:
		blend.FA, blend.FB = Point2{U: sideLocal[0], V: sideLocal[1]}, Point2{U: capLocal[0], V: capLocal[1]}
	default:
		return r.refuse("SL1", `a terminal blend has no cap and side feet`)
	}
	blend.Connector = sectionrecord.LineSeg{Start: blend.FA, End: blend.FB, TStart: 0, TEnd: 1}
	corner := eEnd.Local(v)
	blend.CutbackA = math.Hypot(blend.FA.U-corner[0], blend.FA.V-corner[1])
	blend.CutbackB = math.Hypot(blend.FB.U-corner[0], blend.FB.V-corner[1])
	return nil
}
