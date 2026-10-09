package decad

import (
	"context"
	"fmt"
	"reflect"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is route S's rim assembly (docs/modify-general-design.md §3.3
// steps 5 and 6, "modify-general §N" below). Each removed face R is stated as
// a planar face, and its rim is R's region less the cavity's trace in R's
// plane: R's outer loop holding every cavity face there as a hole, and one
// band per hole of R between that hole and the cavity hole it partners.
// Where the cavity's trace runs along R's own boundary — a removed wall run
// beside a removed cap, each rim reaching the other's edge — the coincident
// pieces cancel and the rest chain into the rim's loops. Every comparison is
// exact on recorded floats, and every loop the assembly cannot state refuses
// (SG7).

// throughCutRims assembles the shell's record (modify-general §3.3 steps 5
// and 6): the receiver's faces but the removed ones, verbatim; the cavity's
// faces reversed, except those lying in a removed face's plane; and, per
// removed face R, its rim: R's region less every cavity face Q in R's plane
// (throughRimRegions), and for each hole h of R the band between h and the
// hole of some Q that partners it — at a cap the hole of the eroded section
// of h's index, at a pierced wall the section of the tool through h dilated
// by t. A loop with no partner is SG7. Every rim region and band runs modify
// §5's audit (S8, S7, S9). A cap's rim carries the cavity's section
// displacement; a wall's rim, whose region states the cavity's levels and the
// receiver's cap levels as in-plane coordinates, carries the largest of that
// and every cavity level's displacement.
func throughCutRims(ctx context.Context, budget *proofbound.WorkBudget, bp brepPayload, tc throughCut, cavity brepPayload, eroded ProfileRecord, dilated []ProfileRecord, rm throughRemoval) (brepPayload, error) {
	frames := make([]r3.Frame, 0, 1+len(cavity.faces))
	frames = append(frames, bp.faces[0].frame)
	for _, f := range cavity.faces {
		frames = append(frames, f.frame)
	}
	embeds, err := brepgeom.Embeds(frames, ErrUnsupported)
	if err != nil {
		return brepPayload{}, err
	}
	cavEmbeds := embeds[1:]
	joined, _, err := brepJoinProfile(eroded)
	if err != nil {
		return brepPayload{}, err
	}
	charge := cavity.sectionDelta()
	for _, f := range cavity.faces {
		charge = max(charge, f.z0Delta, f.z1Delta)
	}

	var removed []int
	if rm.bottom {
		removed = append(removed, tc.caps.bottom)
	}
	if rm.top {
		removed = append(removed, tc.caps.top)
	}
	removed = append(removed, rm.walls...)
	gone := map[int]struct{}{}
	inPlane := map[int]struct{}{}
	var rims []brepFace
	for _, fi := range removed {
		if err := ctx.Err(); err != nil {
			return brepPayload{}, err
		}
		gone[fi] = struct{}{}
		rp, err := tc.rimPlane(bp, fi)
		if err != nil {
			return brepPayload{}, err
		}
		r := rp.face
		partner, err := tc.rimPartner(ctx, fi, rp, joined, dilated)
		if err != nil {
			return brepPayload{}, err
		}
		var qOuters []LoopRecord
		var bands []ProfileRecord
		partnered := map[int]struct{}{}
		// Each cavity face in R's plane: a planar face across R's axis at its
		// level, or at a wall, a straight cavity wall along k on R's carrier.
		for ci, q := range cavity.faces {
			eQ := cavEmbeds[ci]
			if !q.planar() {
				if !rp.wall || eQ.Axis[2] != tc.k {
					continue
				}
				outer, ok := throughSweptTrace(q, eQ, rp, tc.k)
				if !ok {
					continue
				}
				inPlane[ci] = struct{}{}
				qOuters = append(qOuters, outer)
				continue
			}
			if eQ.Axis[2] != rp.axis || brepLevel(q, eQ) != rp.level {
				continue
			}
			inPlane[ci] = struct{}{}
			inR, ok := brepgeom.NewPrismMap(eQ, rp.e).Region(*q.region)
			if !ok {
				return brepPayload{}, throughRimError(r, "a cavity face in its plane does not map into its frame")
			}
			inF, ok := brepgeom.NewPrismMap(eQ, tc.caps.eF).Region(*q.region)
			if !ok && !rp.wall {
				return brepPayload{}, throughRimError(r, "a cavity face in its plane does not map into the prism's frame")
			}
			qOuters = append(qOuters, inR.Outer)
			for qh, hole := range inR.Holes {
				var holeF LoopRecord
				if !rp.wall {
					holeF = inF.Holes[qh]
				}
				hi, err := partner(hole, holeF)
				if err != nil {
					return brepPayload{}, err
				}
				if _, dup := partnered[hi]; dup {
					return brepPayload{}, throughRimError(r, "a cavity hole partners no hole of the removed face, or two")
				}
				partnered[hi] = struct{}{}
				band, err := offset2d.ReverseLoopRecordContext(ctx, hole)
				if err != nil {
					return brepPayload{}, err
				}
				bands = append(bands, ProfileRecord{Outer: band, Holes: []LoopRecord{r.region.Holes[hi]}})
			}
		}
		if len(partnered) != len(r.region.Holes) {
			return brepPayload{}, throughRimError(r, "a hole of the removed face has no cavity hole to partner")
		}
		regions, err := throughRimRegions(ctx, budget, r, r.region.Outer, qOuters)
		if err != nil {
			return brepPayload{}, err
		}
		delta := cavity.sectionDelta()
		if rp.wall {
			delta = charge
		}
		for _, region := range append(regions, bands...) {
			if err := auditThroughRim(budget, region); err != nil {
				return brepPayload{}, err
			}
			region := region
			rims = append(rims, brepFace{frame: r.frame, region: &region, outward: r.outward, sweep: r.sweep,
				z0: r.z0, z1: r.z0, z0Delta: r.z0Delta, z1Delta: r.z0Delta, delta: delta})
		}
	}

	out := brepPayload{xform: bp.xform}
	for fi, f := range bp.faces {
		if _, removed := gone[fi]; removed {
			continue
		}
		out.faces = append(out.faces, f)
	}
	for ci, f := range cavity.faces {
		if _, in := inPlane[ci]; in {
			continue
		}
		out.faces = append(out.faces, f.reversed())
	}
	out.faces = append(out.faces, rims...)
	return out, nil
}

// throughRimPlane is one removed face stated as a planar face: face is the
// statement (frame, region, outward, level, sweep), e its frame's embed, axis
// and level its plane in reference coordinates, and wall whether it is a wall
// of A rather than a cap.
type throughRimPlane struct {
	face  brepFace
	e     brepEmbed
	axis  int
	level float64
	wall  bool
}

// rimPlane states removed face fi as a planar face. A cap and a pierced wall
// are planar faces already and are taken as recorded. A wall along k is a
// straight wall along a section axis (removedFaces admits no other): its
// statement is the rectangle it sweeps over [zlo, zhi] in the signed
// permutation frame across its normal axis whose normal is its outward
// normal, restating a wall along k as a planar face does (brepFace.sweep).
func (tc throughCut) rimPlane(bp brepPayload, fi int) (throughRimPlane, error) {
	f, e := bp.faces[fi], tc.embeds[fi]
	if f.planar() {
		return throughRimPlane{face: f, e: e, axis: e.Axis[2], level: brepLevel(f, e), wall: tc.kinds[fi] == throughPierced}, nil
	}
	from, to, ok := brepgeom.NaturalLine(f.wall)
	if !ok {
		return throughRimPlane{}, throughRimError(f, "its wall is no straight line over its natural range")
	}
	a, b := e.Canon(from.U, from.V, 0), e.Canon(to.U, to.V, 0)
	j := -1
	for i := range 3 {
		if i != tc.k && a[i] == b[i] {
			j = i
		}
	}
	if j < 0 {
		return throughRimPlane{}, throughRimError(f, "its wall lies along no section axis")
	}
	sign := 1.0
	if normal := e.Canon(to.V-from.V, from.U-to.U, 0); normal[j] < 0 {
		sign = -1
	}
	frame, eR, err := brepgeom.PlanarFrame(bp.faces[0].frame, j, sign)
	if err != nil {
		return throughRimPlane{}, throughRimError(f, "its plane has no exact frame")
	}
	level := a[j]
	region := ProfileRecord{Outer: throughRimRect(eR, a, b, tc.k, tc.caps.zlo, tc.caps.zhi)}
	z := sign*level + 0
	face := brepFace{frame: frame, region: &region, outward: true, sweep: tc.caps.frame.N(), z0: z, z1: z, role: f.role}
	return throughRimPlane{face: face, e: eR, axis: j, level: level, wall: true}, nil
}

// throughSweptTrace is the rectangle a straight cavity wall along k sweeps,
// as a counter-clockwise loop in R's frame, when the wall lies on R's
// carrier: both its ends at R's level along R's axis. It reports false for
// any other cavity wall.
func throughSweptTrace(q brepFace, eQ brepEmbed, rp throughRimPlane, k int) (LoopRecord, bool) {
	from, to, ok := brepgeom.NaturalLine(q.wall)
	if !ok {
		return LoopRecord{}, false
	}
	a, b := eQ.Canon(from.U, from.V, 0), eQ.Canon(to.U, to.V, 0)
	if a[rp.axis] != rp.level || b[rp.axis] != rp.level {
		return LoopRecord{}, false
	}
	l0, l1 := eQ.Sign[2]*q.z0+0, eQ.Sign[2]*q.z1+0
	return throughRimRect(rp.e, a, b, k, min(l0, l1), max(l0, l1)), true
}

// throughRimRect is the rectangle the segment from a to b (reference
// coordinates; their coordinates along k are ignored) sweeps over [lo, hi]
// along k, as a counter-clockwise loop of natural-range LineSegs in the frame
// with embed e.
func throughRimRect(e brepEmbed, a, b [3]float64, k int, lo, hi float64) LoopRecord {
	corners := [4][3]float64{a, b, b, a}
	corners[0][k], corners[1][k], corners[2][k], corners[3][k] = lo, lo, hi, hi
	pts := make([]Point2, 4)
	for i, c := range corners {
		l := e.Local(c)
		pts[i] = Point2{U: l[0], V: l[1]}
	}
	area := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		area += pts[i].U*pts[j].V - pts[j].U*pts[i].V
	}
	if area < 0 {
		slices.Reverse(pts)
	}
	segs := make([]CurveSegment, len(pts))
	for i := range pts {
		segs[i] = LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return LoopRecord{Segments: segs}
}

// rimPartner returns the hole partnering of removed face fi: given a cavity
// face's hole in R's frame (and, at a cap, in the prism frame F), the index
// of the hole of R it partners, or SG7. At a cap the cavity hole must equal
// the eroded section's hole of some index (S-1's rule: offsetProfile keeps
// loop order, and the joined record is what class B states), and R's hole of
// that section index partners it. At a pierced wall the cavity hole must
// equal the section of the tool through one of R's holes dilated by t, as the
// tool's own cut states it: the joined dilated section carried into R's frame
// and reversed, compared as throughLoopsSame reads two loops. A wall along k holds no hole, so any cavity hole there is SG7.
func (tc throughCut) rimPartner(ctx context.Context, fi int, rp throughRimPlane, joined ProfileRecord, dilated []ProfileRecord) (func(inR, inF LoopRecord) (int, error), error) {
	r := rp.face
	miss := func(what string) (int, error) { return -1, throughRimError(r, what) }
	if !rp.wall {
		// The section hole index of each of R's region holes.
		holes := make([]int, len(r.region.Holes))
		for hi := range holes {
			holes[hi] = hi
			if fi == tc.caps.bottom {
				holes[hi] = tc.caps.bottomLoop[1+hi] - 1
			}
		}
		return func(_, inF LoopRecord) (int, error) {
			si := slices.IndexFunc(joined.Holes, func(want LoopRecord) bool { return brepgeom.LoopsEqual(want, inF) })
			if si < 0 {
				return miss("a cavity face in its plane holds a loop that is neither its outer loop nor a hole of the eroded section")
			}
			hi := slices.Index(holes, si)
			if hi < 0 {
				return miss("a hole of the eroded section partners no hole of the removed face")
			}
			return hi, nil
		}, nil
	}
	type want struct {
		loop LoopRecord
		hi   int
	}
	var wants []want
	for i, tool := range tc.tools {
		var hi int
		switch fi {
		case tool.w0:
			hi = tool.loop0 - 1
		case tool.w1:
			hi = tool.loop1 - 1
		default:
			continue
		}
		section, _, err := brepJoinProfile(dilated[i])
		if err != nil {
			return nil, err
		}
		inR, ok := brepgeom.NewPrismMap(tool.frameEmb, rp.e).Loop(section.Outer)
		if !ok {
			return nil, throughRimError(r, "a dilated tool's section does not map into its frame")
		}
		loop, err := offset2d.ReverseLoopRecordContext(ctx, inR)
		if err != nil {
			return nil, err
		}
		wants = append(wants, want{loop: loop, hi: hi})
	}
	return func(inR, _ LoopRecord) (int, error) {
		for _, w := range wants {
			if throughLoopsSame(w.loop, inR) {
				return w.hi, nil
			}
		}
		return miss("a cavity face in its plane holds a loop that is neither its outer loop nor the dilated section of a tool through it")
	}, nil
}

// throughLoopsSame is brepLoopsEqual with one widening: two natural-range
// LineSegs that walk the same two ends in the same order are the same piece,
// whichever way each states its range. A reversed loop writes a line
// {End, Start, 0 → 1} where the class-B cut writes {Start, End, 1 → 0}; both
// denote one directed segment bit for bit. Every other segment compares as
// recorded.
func throughLoopsSame(a, b LoopRecord) bool {
	n := len(a.Segments)
	if n != len(b.Segments) || n == 0 {
		return false
	}
	same := func(x, y CurveSegment) bool {
		xf, xt, xok := brepgeom.NaturalLine(x)
		yf, yt, yok := brepgeom.NaturalLine(y)
		if xok && yok {
			return xf == yf && xt == yt
		}
		return reflect.DeepEqual(x, y)
	}
	for r := range n {
		ok := true
		for i := range n {
			if !same(a.Segments[i], b.Segments[(i+r)%n]) {
				ok = false
				break
			}
		}
		if ok {
			return true
		}
	}
	return false
}

// throughRimPiece is one directed piece of a rim's boundary in R's frame:
// its record and walked ends, whether it is a LineSeg along a frame axis over
// its natural range, whether it was cancelled, the loop it comes from (0 R's
// outer loop, 1 + i cavity loop i reversed) and its index there. A curved
// piece's ends are its line neighbours' where it has them (fromKnown,
// toKnown).
type throughRimPiece struct {
	seg                CurveSegment
	from, to           Point2
	fromKnown, toKnown bool
	line, gone         bool
	src, seqIndex      int
}

// throughRimRegions is R's region less the cavity faces in its plane, whose
// outer loops cavity holds in R's frame. Where no cavity loop shares a
// boundary piece with R's outer loop, the rim is §3.3 step 5's record: R's
// outer loop holding each cavity loop reversed. Otherwise every axis-aligned
// line of one loop is split at each line end of another lying strictly
// inside it, each piece of R's outer loop cancels the cavity piece walking
// the same two ends the other way, and the rest chain into loops: a piece
// continues along its own loop where its successor survives, and otherwise
// into the one surviving piece of any loop starting at its end whose own
// predecessor was cancelled. A line split where nothing cancelled keeps the
// split vertex, which the closure count refuses where no neighbouring face
// shares it. One counter-clockwise loop with clockwise ones is one region with
// holes; counter-clockwise loops alone are one region each. Any other result —
// an end with no or several continuations, a loop with no area, two outer
// loops beside a hole — is SG7. Nothing is admitted on a residual: every
// split point is a recorded vertex lying exactly on the line it splits.
func throughRimRegions(ctx context.Context, budget *proofbound.WorkBudget, r brepFace, outer LoopRecord, cavity []LoopRecord) ([]ProfileRecord, error) {
	holes := make([]LoopRecord, len(cavity))
	for i, q := range cavity {
		var err error
		if holes[i], err = offset2d.ReverseLoopRecordContext(ctx, q); err != nil {
			return nil, err
		}
	}
	loops := append([]LoopRecord{outer}, holes...)
	seqs := make([][]*throughRimPiece, len(loops))
	for li, loop := range loops {
		for _, seg := range loop.Segments {
			p := &throughRimPiece{seg: seg, src: li}
			if from, to, ok := brepgeom.NaturalLine(seg); ok && (from.U == to.U) != (from.V == to.V) {
				p.from, p.to, p.line, p.fromKnown, p.toKnown = from, to, true, true, true
			}
			seqs[li] = append(seqs[li], p)
		}
	}
	for li := range seqs {
		var split []*throughRimPiece
		for _, p := range seqs[li] {
			split = append(split, p.splitAt(seqs, li)...)
		}
		seqs[li] = split
	}
	cancelled := 0
	for _, p := range seqs[0] {
		for _, other := range seqs[1:] {
			if i := slices.IndexFunc(other, func(q *throughRimPiece) bool {
				return p.line && q.line && !q.gone && q.from == p.to && q.to == p.from
			}); i >= 0 {
				p.gone, other[i].gone = true, true
				cancelled++
				break
			}
		}
	}
	if cancelled == 0 {
		return []ProfileRecord{{Outer: outer, Holes: holes}}, nil
	}
	for _, seq := range seqs {
		n := len(seq)
		for i, p := range seq {
			p.seqIndex = i
			if p.line {
				continue
			}
			if prev := seq[(i+n-1)%n]; prev.line {
				p.from, p.fromKnown = prev.to, true
			}
			if next := seq[(i+1)%n]; next.line {
				p.to, p.toKnown = next.from, true
			}
		}
	}
	chained, err := throughRimChain(seqs)
	if err != nil {
		return nil, throughRimError(r, err.Error())
	}
	var outs, inner []LoopRecord
	for _, loop := range chained {
		var rec LoopRecord
		for _, p := range loop {
			rec.Segments = append(rec.Segments, p.seg)
		}
		area, err := loopSignedAreaBudget(budget, rec)
		if err != nil {
			return nil, shellCancelCause(err)
		}
		switch {
		case area > 0:
			outs = append(outs, rec)
		case area < 0:
			inner = append(inner, rec)
		default:
			return nil, throughRimError(r, "a loop left by the cavity's trace encloses no area")
		}
	}
	switch {
	case len(inner) == 0:
		out := make([]ProfileRecord, len(outs))
		for i, o := range outs {
			out[i] = ProfileRecord{Outer: o}
		}
		return out, nil
	case len(outs) == 1:
		return []ProfileRecord{{Outer: outs[0], Holes: inner}}, nil
	}
	return nil, throughRimError(r, "the loops left by the cavity's trace are not one outer loop with holes")
}

// splitAt splits a line piece of loop li at every line end of another loop
// lying strictly inside it on its carrier, compared exactly, in walk order.
func (p *throughRimPiece) splitAt(seqs [][]*throughRimPiece, li int) []*throughRimPiece {
	if !p.line {
		return []*throughRimPiece{p}
	}
	along := 0 // the coordinate the line runs along: 0 for U, 1 for V
	if p.from.U == p.to.U {
		along = 1
	}
	coord := func(c Point2) (float64, float64) {
		if along == 0 {
			return c.U, c.V
		}
		return c.V, c.U
	}
	a, level := coord(p.from)
	b, _ := coord(p.to)
	lo, hi := min(a, b), max(a, b)
	var cuts []float64
	for lj, other := range seqs {
		if lj == li {
			continue
		}
		for _, q := range other {
			if !q.line {
				continue
			}
			for _, c := range []Point2{q.from, q.to} {
				x, y := coord(c)
				if y == level && lo < x && x < hi && !slices.Contains(cuts, x) {
					cuts = append(cuts, x)
				}
			}
		}
	}
	if len(cuts) == 0 {
		return []*throughRimPiece{p}
	}
	slices.Sort(cuts)
	if a > b {
		slices.Reverse(cuts)
	}
	point := func(x float64) Point2 {
		if along == 0 {
			return Point2{U: x, V: level}
		}
		return Point2{U: level, V: x}
	}
	out := make([]*throughRimPiece, 0, len(cuts)+1)
	from := p.from
	for _, x := range append(cuts, b) {
		to := point(x)
		if x == b {
			to = p.to
		}
		out = append(out, &throughRimPiece{seg: LineSeg{Start: from, End: to, TStart: 0, TEnd: 1},
			from: from, to: to, fromKnown: true, toKnown: true, line: true, src: p.src})
		from = to
	}
	return out
}

// throughRimChain chains the surviving pieces into loops, starting from R's
// outer loop in walk order.
func throughRimChain(seqs [][]*throughRimPiece) ([][]*throughRimPiece, error) {
	next := func(p *throughRimPiece) (*throughRimPiece, error) {
		seq := seqs[p.src]
		if s := seq[(p.seqIndex+1)%len(seq)]; !s.gone {
			return s, nil
		}
		var found *throughRimPiece
		for _, other := range seqs {
			for _, q := range other {
				prev := other[(q.seqIndex+len(other)-1)%len(other)]
				if q.gone || !prev.gone || !q.fromKnown || q.from != p.to {
					continue
				}
				if found != nil {
					return nil, fmt.Errorf("two pieces continue the cavity's trace from one vertex")
				}
				found = q
			}
		}
		if found == nil {
			return nil, fmt.Errorf("a piece left by the cavity's trace has no continuation")
		}
		return found, nil
	}
	visited := map[*throughRimPiece]struct{}{}
	var loops [][]*throughRimPiece
	for _, seq := range seqs {
		for _, start := range seq {
			if _, seen := visited[start]; start.gone || seen {
				continue
			}
			var loop []*throughRimPiece
			for p := start; ; {
				if _, seen := visited[p]; seen {
					return nil, fmt.Errorf("the cavity's trace leaves a loop that does not close")
				}
				visited[p] = struct{}{}
				loop = append(loop, p)
				q, err := next(p)
				if err != nil {
					return nil, err
				}
				if q == start {
					break
				}
				p = q
			}
			loops = append(loops, loop)
		}
	}
	return loops, nil
}

// throughRimError is SG7, naming the removed face.
func throughRimError(r brepFace, what string) error {
	return fmt.Errorf(`%w: the shell's rim at %s does not partner the cavity's trace: %s (modify-general SG7)`, ErrUnsupported, r.role, what)
}

// auditThroughRim is modify §5's audit on one rim region or band: S8 (the
// outer loop walks counter-clockwise and every hole clockwise, each with a
// non-zero area), then S7 (no crossing or contact) and S9 (nesting) through
// the shared audit.
func auditThroughRim(budget *proofbound.WorkBudget, region ProfileRecord) error {
	for li, loop := range append([]LoopRecord{region.Outer}, region.Holes...) {
		area, err := loopSignedAreaBudget(budget, loop)
		if err != nil {
			return shellCancelCause(err)
		}
		if (li == 0 && !(area > 0)) || (li > 0 && !(area < 0)) {
			return fmt.Errorf(`%w: the shell's rim loop %d is walked against its material`, ErrDegenerate, li)
		}
	}
	if err := auditOffsetSectionBudget(budget, region, region); err != nil {
		return shellCancelCause(err)
	}
	return nil
}
