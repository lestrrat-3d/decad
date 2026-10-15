package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
)

// selectedStraightEdgesShareVertex gates the restatement retry to straight
// edge selections whose topology has a common corner.
func selectedStraightEdgesShareVertex(edges []*Edge) bool {
	for i, edge := range edges {
		if _, ok := edge.n.Curve.(surfacegeom.Line3); !ok {
			return false
		}
		for _, earlier := range edges[:i] {
			if edge.n.Start == earlier.n.Start || edge.n.Start == earlier.n.End ||
				edge.n.End == earlier.n.Start || edge.n.End == earlier.n.End {
				return true
			}
		}
	}
	return false
}

// partialSelectedLoop finds one planar loop that contains every selected edge
// as a proper subset. A Fillet of one route E edge whose endpoint-face
// corner has the wrong material wedge may also reach this reading.
func (r *brepLoopRead) partialSelectedLoop() (brepLoopSel, []bool, bool) {
	if len(r.matched) == 0 || r.matched[0] < 0 {
		return brepLoopSel{}, nil, false
	}
	for _, firstUse := range r.btopo.Edges[r.matched[0]] {
		u := r.btopo.Uses[firstUse]
		if u.Part != brepLoopSeg || !r.bp.Faces[u.Face].Planar() {
			continue
		}
		selected := make([]bool, len(r.btopo.Planar[u.Face][u.Loop]))
		valid := true
		for _, ei := range r.matched {
			if ei < 0 {
				valid = false
				break
			}
			found := false
			for _, ui := range r.btopo.Edges[ei] {
				v := r.btopo.Uses[ui]
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
		u := initial.btopo.Uses[ui]
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
			if ei == initial.btopo.EdgeOf[ui] {
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
	planar := r.btopo.Planar[sel.face]
	if sel.loop >= len(planar) || len(planar[sel.loop]) != len(selected) {
		return nil, r.refuse("SL1", `restating the straight walls changed the selected planar loop`)
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
	if body.n.Volume.Value.Base() <= 0 {
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
	f := r.bp.Faces[sel.face]
	orig := f.RegionLoop(sel.loop)
	cl, err := capblend.OneLoopCornerLoop(r.budget, orig, freeform.NewFreeformWork())
	if err != nil {
		return brepPayload{}, err
	}
	if len(cl.Walks) != len(selected) {
		return brepPayload{}, r.refuse("SL2", `the partial loop coalesces two recorded walks`)
	}
	amounts := make([]float64, len(selected))
	for i, on := range selected {
		if on {
			amounts[i] = r.call.loop.Dc
		}
	}
	fr, err := filletLoopOf(r.budget, orig, r.call.loop.Dc, f.Role, freeform.NewFreeformWork())
	if err != nil {
		return brepPayload{}, err
	}
	sphere := make([]bool, len(selected))
	for i, on := range selected {
		if !on {
			continue
		}
		if cl.Walks[i].IsCircular() && !filletband.SphereWalk(cl.Walks[i], amounts[i]) {
			return brepPayload{}, r.refuse("SL2", `a selected circular walk has no supported partial-loop patch`)
		}
		if !filletband.SphereWalk(cl.Walks[i], amounts[i]) {
			continue
		}
		next := (i + 1) % len(selected)
		if fr.Loop.Corners[i] != filletband.Tangent || fr.Loop.Corners[next] != filletband.Tangent {
			return brepPayload{}, r.refuse("SL2", `a selected sphere walk has a corner that is not tangent`)
		}
		sphere[i] = true
	}
	segs, joins, capWalk, capArc, err := offset2d.PartialFilletContour(
		r.budget, cl.Walks, selected, sphere, amounts, shellTol)
	if err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
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
	delta, err := capband.PartialFilletContourDelta(
		cl.Walks, readings, amounts, sphere, r.call.loop.DcDelta, shellTol)
	if err != nil {
		return brepPayload{}, r.wrapFace(sel.face, err)
	}

	faces := slices.Clone(r.bp.Faces)
	restore := map[int]brepFace{sel.face: r.bp.Faces[sel.face]}
	faces[sel.face].Region = &profile
	faces[sel.face].Delta = math.Max(faces[sel.face].Delta, delta)
	band := brepLoopBand{Face: sel.face, Loop: sel.loop, Orig: cloneLoopRecord(orig),
		Setback: *r.call.loop, Sigma: sel.sigma, Kind: brepBandFillet,
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
		local := eA.Sign[2]*sideRef + 0
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
			if err := r.partialTerminalBlend(sel, orig, capLoop, capWalk, eb, k, sideRef); err != nil {
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
		old := r.bp.Faces[fi]
		loops, err := offset2d.ProfileCornerLoopsBudget(r.budget, *r.bp.Faces[fi].Region)
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
	if len(band.Terminals) == 0 {
		return brepPayload{}, r.refuse("SL1", `the selected partial loop has no terminal arcs`)
	}
	out := brepPayload{Faces: faces, Xform: r.bp.Xform,
		LoopBands: append(slices.Clone(r.bp.LoopBands), band)}
	out.AssignRoles()
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
	for _, ui := range r.btopo.Edges[r.matched[eb.ordinal]] {
		u := r.btopo.Uses[ui]
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
	f := r.bp.Faces[sel.face]
	eF := r.btopo.Embeds[sel.face]
	var capRef [3]float64
	switch {
	case eF.Canon(ow.StartU, ow.StartV, f.Z0) == v:
		capRef = eF.Canon(cw.StartU, cw.StartV, f.Z0)
	case eF.Canon(ow.EndU, ow.EndV, f.Z0) == v:
		capRef = eF.Canon(cw.EndU, cw.EndV, f.Z0)
	default:
		return r.refuse("SL2", `a terminal vertex is not an end of its cap segment`)
	}
	n := eF.Axis[2]
	sideRef := v
	sideRef[n] = sideLevelRef
	centreRef := capRef
	centreRef[n] = sideRef[n]
	endFace := r.bp.Faces[eb.end[k].face]
	eEnd := r.btopo.Embeds[eb.end[k].face]
	capLocal, sideLocal, centreLocal := eEnd.Local(capRef), eEnd.Local(sideRef), eEnd.Local(centreRef)
	if capLocal[2] != endFace.Z0 || sideLocal[2] != endFace.Z0 || centreLocal[2] != endFace.Z0 {
		return r.refuse("SL2", `a terminal cap foot leaves its end face`)
	}
	capPoint := planePoint{U: capLocal[0], V: capLocal[1]}
	sidePoint := planePoint{U: sideLocal[0], V: sideLocal[1]}
	centre := planePoint{U: centreLocal[0], V: centreLocal[1]}
	if math.Abs(math.Hypot(sidePoint.U-centre.U, sidePoint.V-centre.V)-r.call.loop.Dc) > filletTol {
		return r.refuse("SL2", `a terminal wall meets the selected pipe along a noncircular curve`)
	}
	blend := eb.blend[k]
	oldA := eEnd.Canon(blend.FA.U, blend.FA.V, endFace.Z0)
	oldB := eEnd.Canon(blend.FB.U, blend.FB.V, endFace.Z0)
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
