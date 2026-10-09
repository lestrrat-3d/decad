package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// selectedStraightEdgesShareVertex gates the restatement retry to straight
// edge selections whose topology has a common corner.
func selectedStraightEdgesShareVertex(edges []*Edge) bool {
	for i, edge := range edges {
		if _, ok := edge.curve.(Line3); !ok {
			return false
		}
		for _, earlier := range edges[:i] {
			if edge.start == earlier.start || edge.start == earlier.end ||
				edge.end == earlier.start || edge.end == earlier.end {
				return true
			}
		}
	}
	return false
}

// partialSelectedLoop finds one planar loop that contains every selected edge
// as a proper subset. Route E has already taken independent straight edges,
// so a successful reading contains at least one shared vertex.
func (r *brepLoopRead) partialSelectedLoop() (brepLoopSel, []bool, bool) {
	if len(r.matched) < 2 || r.matched[0] < 0 {
		return brepLoopSel{}, nil, false
	}
	for _, firstUse := range r.topo.edges[r.matched[0]] {
		u := r.topo.uses[firstUse]
		if u.Part != brepLoopSeg || !r.bp.faces[u.Face].planar() {
			continue
		}
		selected := make([]bool, len(r.topo.planar[u.Face][u.Loop]))
		valid := true
		for _, ei := range r.matched {
			if ei < 0 {
				valid = false
				break
			}
			found := false
			for _, ui := range r.topo.edges[ei] {
				v := r.topo.uses[ui]
				if v.Part == brepLoopSeg && v.Face == u.Face && v.Loop == u.Loop {
					selected[v.Seg] = true
					found = true
				}
			}
			if !found {
				valid = false
				break
			}
		}
		if !valid || len(r.matched) >= len(selected) {
			continue
		}
		return brepLoopSel{face: u.Face, loop: u.Loop}, selected, true
	}
	return brepLoopSel{}, nil, false
}

// brepFilletPartialLoop reads each selected edge through route E, then moves
// only the selected walls and offsets only their walks of the common cap
// loop. The band's patches close the record's open cap, side and terminal
// arcs during the body build.
func brepFilletPartialLoop(ctx context.Context, d *Document, bp brepPayload, call brepModifyRequest,
	sel brepLoopSel, selected []bool) (*Body, error) {
	initial, err := newBrepLoopRead(ctx, bp, call)
	if err != nil {
		return nil, err
	}
	if err := initial.matchEdges(); err != nil {
		return nil, err
	}
	lineCall := call
	lineCall.edges = nil
	var ordinals []int
	terminals := map[[3]float64]bool{}
	n := len(selected)
	for i, on := range selected {
		if !on {
			continue
		}
		ui := initial.loopUse[[3]int{sel.face, sel.loop, i}]
		u := initial.topo.uses[ui]
		if !u.Walk.IsLine() {
			continue
		}
		if !selected[(i+n-1)%n] {
			terminals[u.From] = true
		}
		if !selected[(i+1)%n] {
			terminals[u.To] = true
		}
		for ordinal, ei := range initial.matched {
			if ei == initial.topo.edgeOf[ui] {
				lineCall.edges = append(lineCall.edges, call.edges[ordinal])
				ordinals = append(ordinals, ordinal)
				break
			}
		}
	}
	if len(lineCall.edges) == 0 {
		return nil, initial.refuse("SL1", `the selected partial loop has no straight walks`)
	}
	partialTerminals := terminals
	if len(lineCall.edges) == len(call.edges) {
		partialTerminals = nil
	}
	er, blends, err := prepareBrepEdgeBlends(ctx, bp, lineCall, true, partialTerminals)
	if err != nil {
		return nil, err
	}
	if partialTerminals == nil {
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
	}
	for i, eb := range blends {
		eb.ordinal = ordinals[i]
	}
	r, err := newBrepLoopRead(ctx, er.bp, call)
	if err != nil {
		return nil, err
	}
	if err := r.matchEdges(); err != nil {
		return nil, err
	}
	_, again, ok := r.partialSelectedLoop()
	if !ok || !slices.Equal(selected, again) {
		return nil, r.refuse("SL1", `restating the straight walls changed the selected partial loop`)
	}
	return r.buildPartialFillet(ctx, d, sel, selected, blends)
}

func (r *brepLoopRead) buildPartialFillet(ctx context.Context, d *Document, sel brepLoopSel,
	selected []bool, blends []*brepEdgeBlend) (*Body, error) {
	out, err := r.rewritePartialFillet(sel, selected, blends)
	if err != nil {
		return nil, err
	}
	body, err := evalBrepContext(ctx, d, d.nextProducerID(), out)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf(`%w; the partial-loop fillet's rewritten brep record; selector %s matched [%s]`,
			err, r.call.sel, selectedEdgesContext(r.call.edges))
	}
	if body.volume.Value.Base() <= 0 {
		return nil, fmt.Errorf(`%w: the partial-loop fillet encloses no volume`, ErrDegenerate)
	}
	return body, nil
}

// rewritePartialFillet records a selected-walk band. The cap contour comes
// from the same per-walk sharp offset used by a subset draft. End walls take
// route E's terminal arcs; selected walls end at the band's side level.
func (r *brepLoopRead) rewritePartialFillet(sel brepLoopSel, selected []bool, blends []*brepEdgeBlend) (brepPayload, error) {
	if r.call.loop == nil {
		return brepPayload{}, r.refuse("SL1", `the partial-loop fillet has no radius`)
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
	if len(cl.walks) != len(selected) {
		return brepPayload{}, r.refuse("SL2", `the partial loop coalesces two recorded walks`)
	}
	amounts := make([]float64, len(selected))
	for i, on := range selected {
		if on {
			amounts[i] = r.call.loop.dc
		}
	}
	fr, err := filletLoopOf(r.budget, orig, r.call.loop.dc, f.role, freeform.NewFreeformWork())
	if err != nil {
		return brepPayload{}, err
	}
	sphere := make([]bool, len(selected))
	for i, on := range selected {
		if !on {
			continue
		}
		if cl.walks[i].IsCircular() && !filletband.SphereWalk(cl.walks[i], amounts[i]) {
			return brepPayload{}, r.refuse("SL2", `a selected circular walk has no supported partial-loop patch`)
		}
		if !filletband.SphereWalk(cl.walks[i], amounts[i]) {
			continue
		}
		next := (i + 1) % len(selected)
		if fr.loop.Corners[i] != filletband.Tangent || fr.loop.Corners[next] != filletband.Tangent {
			return brepPayload{}, r.refuse("SL2", `a selected sphere walk has a corner that is not tangent`)
		}
		sphere[i] = true
	}
	segs, joins, capWalk, capArc, err := offset2d.PartialFilletContour(
		r.budget, cl.walks, selected, sphere, amounts, shellTol)
	if err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
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
	delta, err := capband.PartialFilletContourDelta(
		cl.walks, readings, amounts, sphere, r.call.loop.dcDelta, shellTol)
	if err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
	}

	faces := slices.Clone(r.bp.faces)
	restore := map[int]brepFace{sel.face: r.bp.faces[sel.face]}
	faces[sel.face].region = &profile
	faces[sel.face].delta = math.Max(faces[sel.face].delta, delta)
	band := brepLoopBand{face: sel.face, loop: sel.loop, orig: cloneLoopRecord(orig),
		setback: *r.call.loop, sigma: sel.sigma, kind: brepBandFillet,
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
		local := eA.Sign[2]*sideRef + 0
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
			if err := r.partialTerminalBlend(sel, orig, capLoop, capWalk, eb, k, sideRef); err != nil {
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
				return brepPayload{}, r.refuse("SL2", `two terminal arcs claim one end-face corner`)
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
		loops, err := profileCornerLoopsBudget(r.budget, *r.bp.faces[fi].region)
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
	if len(band.terminals) == 0 {
		return brepPayload{}, r.refuse("SL1", `the selected partial loop has no terminal arcs`)
	}
	out := brepPayload{faces: faces, xform: r.bp.xform,
		loopBands: append(slices.Clone(r.bp.loopBands), band)}
	out.assignRoles()
	return out, nil
}

// partialTerminalBlend pins a chain end to the cap contour's actual foot.
// At a hole mouth the selected cap walks move away from the hole, whereas a
// standalone route E corner would trim the end wall toward the hole. The
// terminal arc must use the cap foot shared with the open band's cylinder.
func (r *brepLoopRead) partialTerminalBlend(sel brepLoopSel, orig, contour loopRecord, capWalk []int,
	eb *brepEdgeBlend, k int, sideLevelRef float64) error {
	v := eb.v[k]
	seg := -1
	for _, ui := range r.topo.edges[r.matched[eb.ordinal]] {
		u := r.topo.uses[ui]
		if u.Face == sel.face && u.Loop == sel.loop && u.Part == brepLoopSeg {
			seg = u.Seg
			break
		}
	}
	if seg < 0 || seg >= len(orig.Segments) || seg >= len(capWalk) ||
		capWalk[seg] < 0 || capWalk[seg] >= len(contour.Segments) {
		return r.refuse("SL2", `a terminal has no selected cap segment`)
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
	f := r.bp.faces[sel.face]
	eF := r.topo.embeds[sel.face]
	var capRef [3]float64
	switch {
	case eF.Canon(ow.StartU, ow.StartV, f.z0) == v:
		capRef = eF.Canon(cw.StartU, cw.StartV, f.z0)
	case eF.Canon(ow.EndU, ow.EndV, f.z0) == v:
		capRef = eF.Canon(cw.EndU, cw.EndV, f.z0)
	default:
		return r.refuse("SL2", `a terminal vertex is not an end of its cap segment`)
	}
	n := eF.Axis[2]
	sideRef := v
	sideRef[n] = sideLevelRef
	centreRef := capRef
	centreRef[n] = sideRef[n]
	endFace := r.bp.faces[eb.end[k].face]
	eEnd := r.topo.embeds[eb.end[k].face]
	capLocal, sideLocal, centreLocal := eEnd.Local(capRef), eEnd.Local(sideRef), eEnd.Local(centreRef)
	if capLocal[2] != endFace.z0 || sideLocal[2] != endFace.z0 || centreLocal[2] != endFace.z0 {
		return r.refuse("SL2", `a terminal cap foot leaves its end face`)
	}
	capPoint := Point2{U: capLocal[0], V: capLocal[1]}
	sidePoint := Point2{U: sideLocal[0], V: sideLocal[1]}
	centre := Point2{U: centreLocal[0], V: centreLocal[1]}
	if math.Abs(math.Hypot(sidePoint.U-centre.U, sidePoint.V-centre.V)-r.call.loop.dc) > filletTol {
		return r.refuse("SL2", `a terminal wall meets the selected pipe along a noncircular curve`)
	}
	blend := eb.blend[k]
	oldA := eEnd.Canon(blend.FA.U, blend.FA.V, endFace.z0)
	oldB := eEnd.Canon(blend.FB.U, blend.FB.V, endFace.z0)
	levelRef := v[n]
	switch {
	case oldA[n] == levelRef && oldB[n] == sideRef[n]:
		blend.FA, blend.FB = capPoint, sidePoint
	case oldB[n] == levelRef && oldA[n] == sideRef[n]:
		blend.FA, blend.FB = sidePoint, capPoint
	default:
		return r.refuse("SL2", `a terminal blend has no cap and side feet`)
	}
	corner := eEnd.Local(v)
	blend.CutbackA = math.Hypot(blend.FA.U-corner[0], blend.FA.V-corner[1])
	blend.CutbackB = math.Hypot(blend.FB.U-corner[0], blend.FB.V-corner[1])
	ax, ay := blend.FA.U-centre.U, blend.FA.V-centre.V
	bx, by := blend.FB.U-centre.U, blend.FB.V-centre.V
	cross := ax*by - ay*bx
	if cross == 0 {
		return r.refuse("SL2", `a terminal arc has no turn`)
	}
	blend.Connector = offset2d.ArcSegment(centre, blend.FA, blend.FB, cross > 0)
	return nil
}
