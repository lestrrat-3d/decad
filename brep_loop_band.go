package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// This file is route L's band (docs/modify-general-design.md §4.2 step 5 and
// §4.3, "modify-general §N" below): the record a brepPayload keeps for each
// chamfered loop, the band boundary edges the record bounds on one side only,
// the adapter that hands both directrices to modify-reach §8.3's buildCapBand
// in the loop's own face frame, and the band's mass terms, read through
// modify-reach §8.4's readers (readBandMass).

// brepLoopBand is one chamfered loop of a route L result. face is the index
// of the planar face F that holds the loop, and loop the index of the cap
// contour in F's region (0 the outer loop, 1+i hole i). orig is the loop as
// the receiver recorded it, in F's frame: the band's side directrix is orig
// moved along F's normal to the side level. setback is the band's two
// setbacks, dc = ds = d (modify-general §7). sigma is −1 when the walls beside
// the loop descend into the body, so the band removes a wedge, and +1 when
// they rise off F, so the band fills the concave corner (Table LB, LB6). kind
// is the band's surface family (docs/loop-fillet-design.md §4 step 3).
type brepLoopBand struct {
	face, loop int
	orig       loopRecord
	setback    capSetback
	sigma      float64
	kind       brepBandKind
	selected   []bool
	capWalk    []int
	capArc     []int
	terminals  []brepBandTerminal
	restore    map[int]brepFace
}

// brepBandTerminal names an open terminal arc on an ordinary planar face of
// a partial fillet band. The arc pairs that face with one band patch.
type brepBandTerminal struct{ face, loop, seg int }

// brepBandKind is a loop band's surface family: a chamfer's ruled patches or a
// fillet's pipe patches (docs/loop-fillet-design.md).
type brepBandKind string

const (
	brepBandChamfer brepBandKind = "chamfer"
	brepBandFillet  brepBandKind = "fillet"
)

// matSign is the band's material sense in F's frame, the sign
// buildCapBand reads: the side level is F's level plus matSign·ds. A
// descending loop's side level lies against F's outward normal, and a rising
// loop's along it.
func (b brepLoopBand) matSign(f brepFace) float64 {
	outward := -1.0
	if f.outward {
		outward = 1
	}
	return b.sigma * outward
}

// sideLevel is the band's side level in F's frame, F's level plus
// matSign·ds, beside its displacement: F's own level displacement plus the
// setback's conversion and the float sum's rounding (capBandLevelDelta).
func (b brepLoopBand) sideLevel(f brepFace) (float64, float64) {
	m := b.matSign(f)
	return f.z0 + m*b.setback.ds, proofbound.AbsSumUpper(f.z0Delta, capBandLevelDelta(f.z0, m, b.setback))
}

// view is the band read as a one-loop cap blend in F's own frame: its
// profile is orig, both levels are F's, and it is chamfered on the cap its
// material sense names. buildCapBand and readBandMass read the band through
// it exactly as they read a prism's cap band.
func (b brepLoopBand) view(f brepFace, xform r3.Transform) capBlendPayload {
	cbp := capBlendPayload{
		profile: profileRecord{Outer: b.orig}, frame: f.frame, xform: xform,
		z0: f.z0, z1: f.z0, z0Delta: f.z0Delta, z1Delta: f.z0Delta,
		start: b.setback, end: b.setback,
		startLoops: map[int]bool{}, endLoops: map[int]bool{},
		fillet: b.kind == brepBandFillet,
	}
	if b.matSign(f) > 0 {
		cbp.startLoops[0] = true
	} else {
		cbp.endLoops[0] = true
	}
	return cbp
}

// patchRole is the role of the band's patch p: chamferLoop(f,l,p) for a
// chamfer band's face f and loop l (modify-general BG2), filletLoop(f,l,p) for
// a fillet band's (docs/loop-fillet-design.md BF1).
func (b brepLoopBand) patchRole(p int) string {
	return fmt.Sprintf("%sLoop(%d,%d,%d)", b.kind, b.face, b.loop, p)
}

// validate refuses a band no route L build records: one naming no planar face
// or no loop of it, a sigma other than ±1, setbacks that are not one positive
// finite d, or an empty loop.
func (b brepLoopBand) validate(bp brepPayload, bi int) error {
	refuse := func(why string) error {
		return fmt.Errorf(`%w: loop band %d of a brep record %s`, ErrUnsupported, bi, why)
	}
	switch {
	case b.face < 0 || b.face >= len(bp.faces) || !bp.faces[b.face].planar():
		return refuse(`names no planar face of the record`)
	case b.loop < 0 || b.loop > len(bp.faces[b.face].region.Holes):
		return refuse(`names no loop of its face`)
	case b.kind != brepBandChamfer && b.kind != brepBandFillet:
		return refuse(`is neither a chamfer nor a fillet band, the two kinds route L builds`)
	case b.sigma != 1 && b.sigma != -1:
		return refuse(`states no side of its face`)
	case !(b.setback.dc > 0) || b.setback.dc != b.setback.ds || math.IsInf(b.setback.dc, 1):
		return refuse(`states no one positive finite setback`)
	case len(b.orig.Segments) == 0:
		return refuse(`holds no loop`)
	case b.selected != nil && len(b.selected) != len(b.orig.Segments):
		return refuse(`has no selection for every original segment`)
	case b.selected != nil && (len(b.capWalk) != len(b.selected) || len(b.capArc) != len(b.selected)):
		return refuse(`has no cap contour map for every selected segment`)
	}
	for _, t := range b.terminals {
		if t.face < 0 || t.face >= len(bp.faces) || !bp.faces[t.face].planar() ||
			t.loop < 0 || t.loop > len(bp.faces[t.face].region.Holes) ||
			t.seg < 0 || t.seg >= len(bp.faces[t.face].regionLoop(t.loop).Segments) {
			return refuse(`names no terminal segment of a planar face`)
		}
	}
	return nil
}

// regionLoop is loop li of a planar face's region: 0 the outer loop, 1+i
// hole i.
func (f brepFace) regionLoop(li int) loopRecord {
	if li == 0 {
		return f.region.Outer
	}
	return f.region.Holes[li-1]
}

// loopBandKeys is the set of edge keys the record bounds on one side only
// (modify-general §4.2 step 6): every band's cap contour, the segments of
// F's loop at F's level, and its side contour, orig's segments at the side
// level. walk is the topology's own segment walk, so each key is the one the
// face beside it states. The set is empty for a record with no band.
func (bp brepPayload) loopBandKeys(embeds []brepEmbed, walk func(curveSegment) (survey2d.SegmentWalk, error)) (map[brepgeom.EdgeKey]struct{}, error) {
	open := map[brepgeom.EdgeKey]struct{}{}
	if len(bp.loopBands) == 0 {
		return open, nil
	}
	add := func(e brepEmbed, seg curveSegment, z float64) error {
		w, err := walk(seg)
		if err != nil {
			return err
		}
		key, _ := brepgeom.CurveKey(e, w, z)
		if _, dup := open[key]; dup {
			return fmt.Errorf(`%w: two chamfer band boundary edges of a brep record coincide`, ErrUnsupported)
		}
		open[key] = struct{}{}
		return nil
	}
	for bi, b := range bp.loopBands {
		if err := b.validate(bp, bi); err != nil {
			return nil, err
		}
		f := bp.faces[b.face]
		capSegs := f.regionLoop(b.loop).Segments
		if b.selected == nil {
			for _, seg := range capSegs {
				if err := add(embeds[b.face], seg, f.z0); err != nil {
					return nil, err
				}
			}
		} else {
			for i, on := range b.selected {
				if !on {
					continue
				}
				if b.capWalk[i] < 0 || b.capWalk[i] >= len(capSegs) {
					return nil, fmt.Errorf(`%w: a partial fillet band's cap walk is missing`, ErrUnsupported)
				}
				if err := add(embeds[b.face], capSegs[b.capWalk[i]], f.z0); err != nil {
					return nil, err
				}
				if k := (i + 1) % len(b.selected); b.capArc[k] >= 0 {
					if b.capArc[k] >= len(capSegs) {
						return nil, fmt.Errorf(`%w: a partial fillet band's cap arc is missing`, ErrUnsupported)
					}
					if err := add(embeds[b.face], capSegs[b.capArc[k]], f.z0); err != nil {
						return nil, err
					}
				}
			}
		}
		sideZ, _ := b.sideLevel(f)
		for i, seg := range b.orig.Segments {
			if b.selected != nil && !b.selected[i] {
				continue
			}
			if err := add(embeds[b.face], seg, sideZ); err != nil {
				return nil, err
			}
		}
		for _, t := range b.terminals {
			face := bp.faces[t.face]
			if err := add(embeds[t.face], face.regionLoop(t.loop).Segments[t.seg], face.z0); err != nil {
				return nil, err
			}
		}
	}
	return open, nil
}

// brepOpenSet is every band boundary edge of one body build: coedge is the
// record face's coedge for each open use, and cap and side hold, per band,
// the coedges buildCapBand reads as the cap contour (F's loop, in loop order)
// and the side contour (orig at the side level, in orig's walk order), each
// walked forward along its loop.
type brepOpenSet struct {
	coedge    map[int]coedge
	cap, side [][]coedge
	capBySeg  []map[int]*Edge
	terminal  []map[brepBandTerminal]*Edge
}

// brepOpenEdges builds every band boundary edge (modify-general §4.2 step 5):
// the cap contour's edges are F's own loop segments, directed along F's walk,
// and the side contour's are orig's segments at the side level, directed
// along orig's walk, which the face beside each walks forward or back. Each
// edge is built in F's frame by the prism rim construction
// (buildWallGeometry) over the face's own displacements, and takes the
// walked-boundary convexity of the loop it runs along (topology.go's
// Edge.IsConvex): a circular walk by its own turn, a straight one by the
// loop's role. Every non-closed end is a vertex the record's own use already
// placed; a whole circle's seam is placed here at its walk's start, which is
// where buildCapBand reads the band's seam.
func brepOpenEdges(ctx context.Context, bp brepPayload, topo *brepTopology, placeVertex func(c [3]float64, faceDelta, levelDelta, endAllow float64) *Vertex) (brepOpenSet, error) {
	out := brepOpenSet{coedge: map[int]coedge{}, cap: make([][]coedge, len(bp.loopBands)),
		side: make([][]coedge, len(bp.loopBands)), capBySeg: make([]map[int]*Edge, len(bp.loopBands)),
		terminal: make([]map[brepBandTerminal]*Edge, len(bp.loopBands))}
	if len(bp.loopBands) == 0 {
		return out, nil
	}
	openAt := map[brepgeom.EdgeKey]int{}
	for _, ui := range topo.open {
		openAt[topo.uses[ui].Key] = ui
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	for bi, b := range bp.loopBands {
		if err := ctx.Err(); err != nil {
			return brepOpenSet{}, err
		}
		f := bp.faces[b.face]
		e := topo.embeds[b.face]
		hole := b.loop != 0
		capView := f.view(bp.xform)
		capSelected := map[int]bool{}
		if b.selected != nil {
			out.capBySeg[bi] = map[int]*Edge{}
			for i, on := range b.selected {
				if on {
					capSelected[b.capWalk[i]] = true
					if arc := b.capArc[(i+1)%len(b.selected)]; arc >= 0 {
						capSelected[arc] = true
					}
				}
			}
		}
		for _, ui := range topo.faceUses[b.face] {
			u := topo.uses[ui]
			if u.Loop != b.loop {
				continue
			}
			if b.selected != nil && !capSelected[u.Seg] {
				continue
			}
			if topo.edgeOf[ui] >= 0 {
				return brepOpenSet{}, fmt.Errorf(`%w: chamfer band %d's cap contour pairs with a face of the record`, ErrUnsupported, bi)
			}
			start, end := placeVertex(u.DirFrom, 0, 0, 0), placeVertex(u.DirTo, 0, 0, 0)
			if u.Key.Closed {
				start = placeVertex(u.DirFrom, f.delta, u.LevelDelta, proofbound.WalkEndBoundAllow(u.StartBound()))
				end = start
			}
			edge, err := brepBandEdge(capView, u.Walk, hole, start, end)
			if err != nil {
				return brepOpenSet{}, err
			}
			co := coedge{edge: edge, forward: true}
			out.coedge[ui] = co
			out.cap[bi] = append(out.cap[bi], co)
			if b.selected != nil {
				out.capBySeg[bi][u.Seg] = edge
			}
		}

		sideZ, _ := b.sideLevel(f)
		cl, err := oneLoopCornerLoop(budget, b.orig, work)
		if err != nil {
			return brepOpenSet{}, err
		}
		if len(cl.walks) != len(b.orig.Segments) {
			return brepOpenSet{}, fmt.Errorf(`%w: chamfer band %d's loop holds two consecutive segments on one carrier`, ErrUnsupported, bi)
		}
		for i, w := range cl.walks {
			if b.selected != nil && !b.selected[i] {
				continue
			}
			key, ccw := brepgeom.CurveKey(e, w.SegmentWalk, sideZ)
			ui, ok := openAt[key]
			if !ok {
				return brepOpenSet{}, fmt.Errorf(`%w: chamfer band %d's side contour is not a boundary of the record`, ErrUnsupported, bi)
			}
			u := topo.uses[ui]
			beside := bp.faces[u.Face]
			from, to := e.Canon(w.StartU, w.StartV, sideZ), e.Canon(w.EndU, w.EndV, sideZ)
			var start, end *Vertex
			forward := u.From == from && u.To == to
			if u.Key.Closed {
				seg := b.orig.Segments[w.Segs[0]]
				start = placeVertex(from, beside.delta, u.LevelDelta,
					proofbound.WalkEndBoundAllow(boundarywalk.DenotedStartBound(seg, w.SegmentWalk)))
				end = start
				forward = u.Sense == ccw
			} else {
				start, end = placeVertex(from, 0, 0, 0), placeVertex(to, 0, 0, 0)
			}
			sideView := f.view(bp.xform)
			sideView.z0, sideView.z1 = sideZ, sideZ
			sideView.z0Delta, sideView.z1Delta = u.LevelDelta, u.LevelDelta
			sideView.sectionDelta = beside.delta
			edge, err := brepBandEdge(sideView, w.SegmentWalk, hole, start, end)
			if err != nil {
				return brepOpenSet{}, err
			}
			out.coedge[ui] = coedge{edge: edge, forward: forward}
			out.side[bi] = append(out.side[bi], coedge{edge: edge, forward: true})
		}
		if len(b.terminals) > 0 {
			out.terminal[bi] = map[brepBandTerminal]*Edge{}
		}
		for _, t := range b.terminals {
			ui := -1
			for _, candidate := range topo.faceUses[t.face] {
				u := topo.uses[candidate]
				if u.Part == brepLoopSeg && u.Loop == t.loop && u.Seg == t.seg {
					ui = candidate
					break
				}
			}
			if ui < 0 || topo.edgeOf[ui] >= 0 {
				return brepOpenSet{}, fmt.Errorf(`%w: fillet band %d's terminal is not an open planar edge`, ErrUnsupported, bi)
			}
			u := topo.uses[ui]
			start, end := placeVertex(u.DirFrom, 0, 0, 0), placeVertex(u.DirTo, 0, 0, 0)
			view := bp.faces[t.face].view(bp.xform)
			edge, err := brepBandEdge(view, u.Walk, t.loop != 0, start, end)
			if err != nil {
				return brepOpenSet{}, err
			}
			out.coedge[ui] = coedge{edge: edge, forward: true}
			out.terminal[bi][t] = edge
		}
	}
	if len(out.coedge) != len(topo.open) {
		return brepOpenSet{}, fmt.Errorf(`%w: the record's chamfer bands bound %d of its %d one-sided edges`, ErrUnsupported, len(out.coedge), len(topo.open))
	}
	return out, nil
}

// brepBandEdge builds one band boundary edge: w walked at view's level from
// start to end, sharing one seam vertex when w is a whole circle, with the
// walked-boundary convexity of a loop whose role hole names. Its length
// carries view's section displacement, as a brep loop segment's does
// (brepEdge).
func brepBandEdge(view prismPayload, w survey2d.SegmentWalk, hole bool, start, end *Vertex) (*Edge, error) {
	convex := !hole
	if w.IsCircular() {
		convex = w.Th1 >= w.Th0
	}
	edge, _, _, _, err := buildWallGeometry(view, survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}, convex, w.Closed, start, end, start, end) //nolint:dogsled // only the rim at view's level is read
	if err != nil {
		return nil, err
	}
	edge.lengthBound = proofbound.AbsSumUpper(w.LengthBound, proofbound.SectionDisplacementLength(view.sectionDelta, 1))
	return edge, nil
}

// brepBandMass is what the bands add to a brep body's divergence sums
// (modify-general §4.3), in the record's reference frame: vol3 is the patches'
// share of 3V, moments their share of the first moment along each reference
// axis, and area their summed areas.
type brepBandMass struct {
	vol3    proofbound.RatInterval
	moments [3]proofbound.RatInterval
	area    proofbound.BoundedScalar
}

// zeroBrepBandMass is the sums of a body with no band.
func zeroBrepBandMass() brepBandMass {
	zero := proofbound.PointInterval(new(big.Rat))
	return brepBandMass{vol3: zero, moments: [3]proofbound.RatInterval{zero, zero, zero}}
}

func (m *brepBandMass) add(o brepBandMass) {
	m.vol3 = proofbound.IntervalAdd(m.vol3, o.vol3)
	for i := range m.moments {
		m.moments[i] = proofbound.IntervalAdd(m.moments[i], o.moments[i])
	}
	m.area = proofbound.BoundedAdd(m.area, o.area)
}

// brepBandsBuilt is every band a body build attached: the patch faces, in
// band order and each band's own patch order, and their mass sums.
type brepBandsBuilt struct {
	patches []*Face
	// geom is each band's patch roles beside their geometry, in band order.
	geom [][]capPatch
	mass brepBandMass
}

// attachBrepLoopBands attaches every band of the record to the body
// (modify-general §4.2 step 5): buildCapBand runs in F's own frame over the
// band's cap blend view, handed the cap contour and the side contour as the
// record's own coedges (brepOpenEdges), and mints the slant rulings, the
// reflex connectors' apex cones and the patches between. Its orientation
// reference is the wall's outward normal turned toward the cap; beside a
// rising loop the wall's material lies on the loop's other side, so the
// reference is the true normal negated and every patch is turned over. Each
// patch carries chamferLoop(f,l,p) for the band's face f, loop l and its own
// index p in the band. The band's mass terms are read as brepBandMassOf
// states. A fillet band is attachFilletBand's, and its geom entry is nil.
func attachBrepLoopBands(ctx context.Context, body *Body, ref producerID, bp brepPayload, open brepOpenSet) (brepBandsBuilt, error) {
	out := brepBandsBuilt{mass: zeroBrepBandMass()}
	if len(bp.loopBands) == 0 {
		return out, nil
	}
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		return brepBandsBuilt{}, err
	}
	work := freeform.NewFreeformWork()
	for bi, b := range bp.loopBands {
		f := bp.faces[b.face]
		if b.kind == brepBandFillet {
			var patches []*Face
			var mass brepBandMass
			var err error
			if b.selected != nil {
				patches, mass, err = attachPartialFilletBand(ctx, body, ref, bp, bi, open, embeds[b.face], work)
			} else {
				patches, mass, err = attachFilletBand(ctx, body, ref, bp, bi, open, embeds[b.face], work)
			}
			if err != nil {
				return brepBandsBuilt{}, err
			}
			out.patches = append(out.patches, patches...)
			out.geom = append(out.geom, nil)
			out.mass.add(mass)
			continue
		}
		cbp := b.view(f, bp.xform)
		band, err := buildCapBand(ctx, body, ref, cbp, 0, b.orig, f.z0, b.matSign(f), open.side[bi], open.cap[bi], work)
		if err != nil {
			return brepBandsBuilt{}, err
		}
		geom := make([]capPatch, len(band.patches))
		for p, patch := range band.patches {
			role := b.patchRole(p)
			patch.origins = []FeatureRef{{producer: ref, Role: role}}
			if b.sigma > 0 {
				patch.reversed = !patch.reversed
			}
			geom[p] = capPatch{role: role, geom: band.geom[p]}
		}
		out.patches = append(out.patches, band.patches...)
		out.geom = append(out.geom, geom)
		mass, err := brepBandMassOf(ctx, b, f, embeds[b.face], cbp, band, work)
		if err != nil {
			return brepBandsBuilt{}, err
		}
		out.mass.add(mass)
	}
	return out, nil
}

// brepBandMassOf is one band's terms in the record's divergence sums
// (modify-general §4.3). readBandMass, capband.BandVolume and
// capband.BandMoment read the closed band region B — the band's patches
// closed by the cap contour's disk at F's level and orig's disk at the side
// level — with its volume, its first moments about F's frame origin, the
// chord-versus-locus terms, the closure slivers and the side level's
// displacement. The patches' own share of the body's sums is B's less its two
// disks, each disk's flux and axial moment taken exactly against B's outward
// normal (along −matSign at the cap level, +matSign at the side level), and
// signed by whether B is material of the body: B is material beside an outer
// loop that descends and a hole that rises, and void beside a hole that
// descends and an outer loop that rises, so the sign is −sigma times orig's
// orientation. Every face frame shares the reference origin bit for bit
// (brepEmbeds), so F's first moments map onto the reference axes by F's
// signed permutation.
func brepBandMassOf(ctx context.Context, b brepLoopBand, f brepFace, e brepEmbed, cbp capBlendPayload, band capBandResult, work *freeform.FreeformWork) (brepBandMass, error) {
	m := b.matSign(f)
	in, err := readBandMass(ctx, 0, b.orig, cbp, band.geom, f.z0, m, band.delta, band.closure)
	if err != nil {
		return brepBandMass{}, err
	}
	volume, err := capband.BandVolume(in, work)
	if err != nil {
		return brepBandMass{}, err
	}
	mu, mv, mz, err := capband.BandMoment(in, work)
	if err != nil {
		return brepBandMass{}, err
	}
	enclose := func(s proofbound.BoundedScalar) (proofbound.RatInterval, error) {
		return brepgeom.Enclosure(s.Value, s.Bound, nil)
	}
	capArea, err := enclose(in.CapArea)
	if err != nil {
		return brepBandMass{}, err
	}
	sideArea, err := enclose(in.SideArea)
	if err != nil {
		return brepBandMass{}, err
	}
	capZ, sideZ := proofarith.FloatRat(in.CapLevel.Value), proofarith.FloatRat(in.SideLevel.Value)
	if capZ == nil || sideZ == nil {
		return brepBandMass{}, fmt.Errorf(`%w: a chamfer band's levels are not finite`, ErrNotFinite)
	}
	mRat, eps := big.NewRat(int64(m), 1), big.NewRat(int64(-b.sigma*in.Orientation), 1)
	negM, half := new(big.Rat).Neg(mRat), big.NewRat(1, 2)
	disks := proofbound.IntervalAdd(
		proofbound.IntervalScale(capArea, proofbound.RatMul(negM, capZ)),
		proofbound.IntervalScale(sideArea, proofbound.RatMul(mRat, sideZ)))
	diskMoment := proofbound.IntervalAdd(
		proofbound.IntervalScale(capArea, proofbound.RatMul(half, negM, capZ, capZ)),
		proofbound.IntervalScale(sideArea, proofbound.RatMul(half, mRat, sideZ, sideZ)))

	volumeIv, err := enclose(volume)
	if err != nil {
		return brepBandMass{}, err
	}
	minus := big.NewRat(-1, 1)
	out := zeroBrepBandMass()
	out.vol3 = proofbound.IntervalScale(proofbound.IntervalAdd(
		proofbound.IntervalScale(volumeIv, big.NewRat(3, 1)), proofbound.IntervalScale(disks, minus)), eps)
	var local [3]proofbound.RatInterval
	for i, s := range [2]proofbound.BoundedScalar{mu, mv} {
		iv, err := enclose(s)
		if err != nil {
			return brepBandMass{}, err
		}
		local[i] = iv
	}
	mzIv, err := enclose(mz)
	if err != nil {
		return brepBandMass{}, err
	}
	local[2] = proofbound.IntervalAdd(mzIv, proofbound.IntervalScale(diskMoment, minus))
	for i := range local {
		scale := proofbound.RatMul(eps, big.NewRat(int64(e.Sign[i]), 1))
		out.moments[e.Axis[i]] = proofbound.IntervalScale(local[i], scale)
	}
	for _, g := range band.geom {
		pa, pb := capband.AreaOf(g)
		out.area = proofbound.BoundedAdd(out.area, proofbound.MeasuredScalar(pa, pb))
	}
	return out, nil
}

// brepBandsOccupiedVolumeAdmission is capBlendOccupiedVolumeAdmission's
// question for every band of a route L body (modify-general Table DG's DG3 and
// DG4): each band is admitted as the cap-loop chamfer admits its own loop, as
// a whole turn, or a loop whose every corner is a line-line miter or an
// exactly tangent join. refusal is the first band's reason, nil when every
// band is admitted; err is a budget or context error.
func brepBandsOccupiedVolumeAdmission(budget *proofbound.WorkBudget, bp brepPayload) (error, error) {
	for bi, b := range bp.loopBands {
		if err := b.validate(bp, bi); err != nil {
			return nil, err
		}
		if b.kind == brepBandFillet {
			// Every corner class a fillet band holds is one its strips chord
			// slice by slice; SF1 refused the rest at the build, and the LF6
			// fan's stations are the connector arc's azimuths at every ring
			// (loop-fillet DF5).
			continue
		}
		refusal, err := capBlendOccupiedVolumeAdmission(budget, b.tessView(bp.faces[b.face], bp.xform))
		if err != nil || refusal != nil {
			return refusal, err
		}
	}
	return nil, nil //nolint:nilnil // no band refuses and no budget ran out
}
