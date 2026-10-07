package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the mirror join of docs/mirror-pattern-design.md §5: Mirrored
// with WithJoin returns the union of a prism or stacked receiver and its own
// mirror image as one body, by rewriting the receiver's recorded section in
// its own frame. No boolean runs and no sketch scene is built: §5.2 states why
// this 2D answer is decad's own. The join reflects the record's own
// coordinates in exact rational arithmetic, removes the walls on the mirror
// line by exact identity, and hands the assembled record to the modify §5
// audit, which can only refuse.

// identMirrorJoin is WithJoin's option identity.
type identMirrorJoin struct{}

type mirrorJoinOption struct{ option.Interface }

func (mirrorJoinOption) mirrorOption() {}

// WithJoin makes [Body.Mirrored] return the union of the receiver and its
// mirror image as one body: the symmetric-part operation
// (docs/mirror-pattern-design.md §5). The plane must be a [MirrorFace] of the
// receiver itself naming one or more planar walls on one line of its section;
// the selector then asserts at least one face rather than exactly one.
//
// The join rewrites the receiver's own recorded section and never runs a
// boolean. It admits a straight prism or a stacked prism (a blind-cut plate)
// whose every other boundary segment is a whole line, a whole arc or a whole
// circle lying on the receiver's side of the mirror line, and whose recorded
// section carries no displacement. Every other receiver is refused with the
// sentinel §5.1 states — ErrUnsupported for a shape the join does not build,
// ErrDegenerate for a selection that names no single mirror line — and is
// never handed to Union instead. A repeated WithJoin() is idempotent.
func WithJoin() MirrorOption {
	return mirrorJoinOption{option.New(identMirrorJoin{}, struct{}{})}
}

// joinWall is one selected wall of the receiver: the (loop, segment) its role
// names, in slab slab's region for a stacked receiver and -1 for a prism.
type joinWall struct {
	slab, loop, seg int
}

// mirroredJoin is Mirrored under WithJoin. It runs Placed's receiver gates,
// then §5.1's admission J1-J7 in order, assembles the joined record (§5.2),
// audits it (§5.3) and commits the result in place of the receiver.
func (b *Body) mirroredJoin(ctx context.Context, plane MirrorPlane) (*Body, error) {
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot mirror a body it did not build`, ErrUnsupported)
	}
	// J1: a straight prism or a stacked prism, and a solid.
	switch b.payload.(type) {
	case prismPayload, stackedPrismPayload:
	default:
		return nil, fmt.Errorf(`%w: a mirror join rewrites a prism's or a stacked prism's section; join any other body with MirroredCopy and Union`, ErrUnsupported)
	}
	if b.Kind() == BodySheet {
		return nil, fmt.Errorf(`%w: a mirror join builds a solid and does not accept a sheet body`, ErrUnsupported)
	}
	walls, err := d.resolveJoinWalls(b, plane)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	ref := d.nextProducerID()
	var body *Body
	switch p := b.payload.(type) {
	case prismPayload:
		joined, err := joinPrismPayload(budget, p, walls)
		if err != nil {
			return nil, err
		}
		body, err = evalPrismContext(ctx, d, ref, joined, freeform.NewFreeformWork())
		if err != nil {
			return nil, err
		}
	case stackedPrismPayload:
		joined, err := joinStackedPayload(ctx, budget, p, walls)
		if err != nil {
			return nil, err
		}
		body, err = evalStackedContext(ctx, d, ref, joined)
		if err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b)
	return body, nil
}

// resolveJoinWalls is J2's selection half: the plane must be a MirrorFace of
// the receiver, its selector resolves to at least one face, and every face
// must be a wall the receiver's own producer minted a side role for. Whether
// that wall is planar is read off the record by the payload's own join.
func (d *Document) resolveJoinWalls(b *Body, plane MirrorPlane) ([]joinWall, error) {
	var mf MirrorFace
	switch p := plane.(type) {
	case MirrorFace:
		mf = p
	case *MirrorFace:
		if p == nil {
			return nil, errNilMirrorPlane
		}
		mf = *p
	case *MirrorFrame:
		if p == nil {
			return nil, errNilMirrorPlane
		}
		return nil, errJoinNeedsFace
	case MirrorFrame:
		return nil, errJoinNeedsFace
	default:
		return nil, errNilMirrorPlane
	}
	if mf.Body == nil {
		return nil, fmt.Errorf(`%w: a mirror face names no body to resolve against`, ErrDegenerate)
	}
	if err := d.requireLive(mf.Body); err != nil {
		return nil, err
	}
	if mf.Body != b {
		return nil, fmt.Errorf(`%w: a mirror join needs its mirror line in the receiver's own record, so the mirror face must be a face of the receiver`, ErrUnsupported)
	}
	faces, err := selectAtLeastOneFace(b, mf.Face, "the mirror face")
	if err != nil {
		return nil, err
	}
	_, stacked := b.payload.(stackedPrismPayload)
	producer := b.originProducer()
	var walls []joinWall
	for fi, f := range faces {
		found := false
		for _, o := range f.origins {
			if o.producer != producer {
				continue
			}
			var k, i, j int
			if n, _ := fmt.Sscanf(o.Role, "slab(%d).region(0).side(%d,%d)", &k, &i, &j); n == 3 && stacked {
				walls = append(walls, joinWall{slab: k, loop: i, seg: j})
				found = true
				continue
			}
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &i, &j); n == 2 && !stacked {
				walls = append(walls, joinWall{slab: -1, loop: i, seg: j})
				found = true
			}
		}
		if !found {
			return nil, fmt.Errorf(`%w: selected face %d is not a wall of the receiver (a cap or a floor states no mirror line in the section)`, ErrDegenerate, fi)
		}
	}
	return walls, nil
}

// errJoinNeedsFace is J2's refusal of a MirrorFrame: the join reads its
// mirror line from a recorded wall, which a caller-built frame does not name.
var errJoinNeedsFace = fmt.Errorf(`%w: a mirror join needs the mirror line as a recorded wall; name it with a MirrorFace of the receiver, not a MirrorFrame`, ErrUnsupported)

// selectAtLeastOneFace resolves sel against body asserting at least one
// match: an unasserted zero is ErrCardinality with Expected "at least 1",
// and a failed assertion of the selector's own is returned unchanged.
func selectAtLeastOneFace(body *Body, sel FaceSelector, what string) ([]*Face, error) {
	q, err := builtinFaceQuery(sel, what)
	if err != nil {
		return nil, err
	}
	faces, err := q.SelectFaces(body)
	if err != nil {
		if errors.Is(err, ErrCardinality) {
			return nil, err
		}
		if errors.Is(err, ErrNoMatch) {
			return nil, q.selectionError(body, 0, "at least 1", ErrCardinality)
		}
		return nil, err
	}
	return faces, nil
}

// joinRegion is one recorded region under the join: its loops (outer first)
// and, per loop, the segment indices that are selected walls.
type joinRegion struct {
	loops []LoopRecord
	sel   []map[int]struct{}
}

func newJoinRegion(p ProfileRecord) joinRegion {
	loops := append([]LoopRecord{p.Outer}, p.Holes...)
	sel := make([]map[int]struct{}, len(loops))
	for i := range sel {
		sel[i] = map[int]struct{}{}
	}
	return joinRegion{loops: loops, sel: sel}
}

// joinPrismPayload runs J2-J7 over a prism receiver and returns the joined
// payload: the same frame, placement, interval and axial displacements over
// the assembled record, with δ_mirror as its section displacement.
func joinPrismPayload(budget *proofbound.WorkBudget, pp prismPayload, walls []joinWall) (prismPayload, error) {
	region := newJoinRegion(pp.profile)
	segs := make([]LineSeg, 0, len(walls))
	for _, w := range walls {
		seg, err := joinWallSegment(region.loops, w)
		if err != nil {
			return prismPayload{}, err
		}
		segs = append(segs, seg)
		region.sel[w.loop][w.seg] = struct{}{}
	}
	line, err := admitMirrorLine(segs)
	if err != nil {
		return prismPayload{}, err
	}
	if err := line.admitRegions(budget, []joinRegion{region}); err != nil {
		return prismPayload{}, err
	}
	if pp.sectionDelta != 0 {
		return prismPayload{}, errJoinDisplaced
	}
	joined, delta, err := line.assemble(budget, region)
	if err != nil {
		return prismPayload{}, err
	}
	out := pp
	out.profile = joined
	out.sectionDelta = delta
	out.walks = nil
	out.blendSegs = nil
	out.blendKind = ""
	return out, nil
}

// joinStackedPayload runs J2-J7 over every slab's region of a stacked
// receiver, rewrites each region by the same splice, and re-derives every
// interface's exposed records from the rewritten regions. Equal loops rewrite
// to equal loops, so the outer wall stays one column and each interface stays
// monotone; evalStackedContext's falsifyStackedPayload re-checks both.
func joinStackedPayload(ctx context.Context, budget *proofbound.WorkBudget, sp stackedPrismPayload, walls []joinWall) (stackedPrismPayload, error) {
	regions := make([]joinRegion, len(sp.slabs))
	for k, slab := range sp.slabs {
		if len(slab.regions) != 1 {
			return stackedPrismPayload{}, fmt.Errorf(`%w: slab %d has no single region`, ErrUnsupported, k)
		}
		regions[k] = newJoinRegion(slab.regions[0])
	}
	segs := make([]LineSeg, 0, len(walls))
	for _, w := range walls {
		if w.slab < 0 || w.slab >= len(regions) {
			return stackedPrismPayload{}, fmt.Errorf(`%w: a selected wall names slab %d, which the receiver does not have`, ErrDegenerate, w.slab)
		}
		seg, err := joinWallSegment(regions[w.slab].loops, w)
		if err != nil {
			return stackedPrismPayload{}, err
		}
		segs = append(segs, seg)
		// The wall's face spans its whole column: mark the same loop and
		// segment wherever an equal loop appears.
		wallLoop := regions[w.slab].loops[w.loop]
		for _, region := range regions {
			for li, loop := range region.loops {
				equal, err := loopRecordsEqual(budget, wallLoop, loop)
				if err != nil {
					return stackedPrismPayload{}, err
				}
				if equal {
					region.sel[li][w.seg] = struct{}{}
				}
			}
		}
	}
	line, err := admitMirrorLine(segs)
	if err != nil {
		return stackedPrismPayload{}, err
	}
	if err := line.admitRegions(budget, regions); err != nil {
		return stackedPrismPayload{}, err
	}
	if sp.sectionDelta != 0 {
		return stackedPrismPayload{}, errJoinDisplaced
	}
	out := sp
	out.slabs = make([]prismSlab, len(sp.slabs))
	for k, region := range regions {
		joined, delta, err := line.assemble(budget, region)
		if err != nil {
			return stackedPrismPayload{}, err
		}
		out.slabs[k] = sp.slabs[k]
		out.slabs[k].regions = []ProfileRecord{joined}
		out.sectionDelta = math.Max(out.sectionDelta, delta)
	}
	out.interfaces = make([]prismSlabInterface, len(out.slabs)-1)
	for i := range out.interfaces {
		lowerOnly, upperOnly, err := stackedExclusiveHoles(out.slabs[i].regions[0], out.slabs[i+1].regions[0])
		if err != nil {
			return stackedPrismPayload{}, err
		}
		lower, err := stackedExposed(ctx, upperOnly)
		if err != nil {
			return stackedPrismPayload{}, err
		}
		upper, err := stackedExposed(ctx, lowerOnly)
		if err != nil {
			return stackedPrismPayload{}, err
		}
		out.interfaces[i] = prismSlabInterface{lowerExposed: lower, upperExposed: upper}
	}
	return out, nil
}

// errJoinDisplaced is J7: the §5.3 audit proves closure on the recorded
// coordinates, and a displaced record does not state the section it denotes.
var errJoinDisplaced = fmt.Errorf(`%w: the receiver's section carries a displacement from the section it denotes, so the join cannot prove its splice closes; join a receiver drawn directly (J7)`, ErrUnsupported)

// joinWallSegment is J2's record half: the selected wall's segment, which
// must be a line. A curved wall names no mirror plane.
func joinWallSegment(loops []LoopRecord, w joinWall) (LineSeg, error) {
	if w.loop < 0 || w.loop >= len(loops) || w.seg < 0 || w.seg >= len(loops[w.loop].Segments) {
		return LineSeg{}, fmt.Errorf(`%w: a selected wall names segment (%d, %d), which the receiver's record does not have`, ErrDegenerate, w.loop, w.seg)
	}
	seg, err := normalizeSegment(loops[w.loop].Segments[w.seg])
	if err != nil {
		return LineSeg{}, err
	}
	line, ok := seg.(LineSeg)
	if !ok {
		return LineSeg{}, fmt.Errorf(`%w: selected wall (%d, %d) is a %T, and a curved wall names no mirror plane`, ErrDegenerate, w.loop, w.seg, seg)
	}
	return line, nil
}

// mirrorLine is the join's mirror line in the section's plane: the carrier of
// the first selected wall, oriented along that wall's walk, so the material
// lies on its left. Every coordinate is held as an exact rational.
type mirrorLine struct {
	pu, pv *big.Rat // the first wall's walk start
	du, dv *big.Rat // its walk direction, end minus start
	dd     *big.Rat // du² + dv²
}

func ratOf(v float64) *big.Rat { return proofarith.FloatRat(v) }

// lineWalkEnds is a whole line's walk: Start to End, or End to Start when its
// range runs backwards.
func lineWalkEnds(s LineSeg) (Point2, Point2) {
	if s.TStart > s.TEnd {
		return s.End, s.Start
	}
	return s.Start, s.End
}

// admitMirrorLine runs J3 and J4 over the selected walls and returns the
// line they lie on. J3: every wall's two recorded endpoints lie on the first
// wall's carrier, decided by exact rational cross products. J4: every wall is
// whole.
func admitMirrorLine(walls []LineSeg) (mirrorLine, error) {
	first := walls[0]
	start, end := lineWalkEnds(first)
	l := mirrorLine{pu: ratOf(start.U), pv: ratOf(start.V)}
	eu, ev := ratOf(end.U), ratOf(end.V)
	l.du = new(big.Rat).Sub(eu, l.pu)
	l.dv = new(big.Rat).Sub(ev, l.pv)
	l.dd = proofbound.RatAdd(proofbound.RatMul(l.du, l.du), proofbound.RatMul(l.dv, l.dv))
	if l.dd.Sign() == 0 {
		return mirrorLine{}, fmt.Errorf(`%w: the selected wall has zero length and names no mirror line`, ErrDegenerate)
	}
	for i, s := range walls {
		if l.side(s.Start) != 0 || l.side(s.End) != 0 {
			return mirrorLine{}, fmt.Errorf(`%w: selected wall %d does not lie on the first selected wall's line, so the selection names no single mirror plane (J3)`, ErrDegenerate, i)
		}
	}
	for i, s := range walls {
		if !prismcells.WholeSegmentRange(s.TStart, s.TEnd) {
			return mirrorLine{}, fmt.Errorf(`%w: selected wall %d is a fragment of a longer recorded line, and the join mirrors only a wall the record states whole (J4)`, ErrUnsupported, i)
		}
	}
	return l, nil
}

// cross is D × (X − P), exactly: positive on the material side, zero on the
// line.
func (l mirrorLine) cross(p Point2) *big.Rat {
	xu, xv := ratOf(p.U), ratOf(p.V)
	ru := new(big.Rat).Sub(xu, l.pu)
	rv := new(big.Rat).Sub(xv, l.pv)
	return new(big.Rat).Sub(proofbound.RatMul(l.du, rv), proofbound.RatMul(l.dv, ru))
}

// side is the sign of cross.
func (l mirrorLine) side(p Point2) int { return l.cross(p).Sign() }

// circleOnSide reports whether the whole circle about c of squared radius r2
// lies in the closed material half-plane: c on the material side, and its
// distance to the line, cross/|D|, at least the radius. Both sides are
// squared, so the test is exact.
func (l mirrorLine) circleOnSide(c Point2, r2 *big.Rat) bool {
	x := l.cross(c)
	if x.Sign() < 0 {
		return false
	}
	return proofbound.RatMul(x, x).Cmp(proofbound.RatMul(r2, l.dd)) >= 0
}

// reflect returns the image of p across the line, each coordinate computed
// exactly and rounded to the nearest float once, beside the rounding it
// committed: the sum of the two coordinates' errors, which bounds the
// distance from the held image to the exact one. m(X) = X − 2·c/(D·D)·D⊥
// with c = D × (X − P) and D⊥ = (−dv, du); no square root is taken.
func (l mirrorLine) reflect(p Point2) (Point2, float64, error) {
	c := l.cross(p)
	k := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), c), l.dd)
	u := new(big.Rat).Add(ratOf(p.U), new(big.Rat).Mul(k, l.dv))
	v := new(big.Rat).Sub(ratOf(p.V), new(big.Rat).Mul(k, l.du))
	hu, _ := u.Float64()
	hv, _ := v.Float64()
	if math.IsInf(hu, 0) || math.IsInf(hv, 0) {
		return Point2{}, 0, fmt.Errorf(`%w: a mirrored coordinate overflows a float`, ErrNotFinite)
	}
	held := Point2{U: hu, V: hv}
	eu, ev := proofarith.RationalFloatError(u, hu), proofarith.RationalFloatError(v, hv)
	switch {
	case eu == 0:
		// A coordinate that rounded nothing adds nothing, so a line parallel
		// to an axis charges the other coordinate's rounding exactly.
		return held, ev, nil
	case ev == 0:
		return held, eu, nil
	default:
		return held, proofbound.AbsSumUpper(eu, ev), nil
	}
}

// admitRegions runs J5 and J6 over every region, in that order. J5: every
// selected wall walks the first wall's way (so the material lies on one
// side), every region's outer loop holds a selected wall, every other segment
// is a line, arc or circle in the closed material half-plane (an arc or
// circle with its whole circle there), and a segment end on the line is the
// junction with a selected wall. J6: no arc or circle is recorded over a
// narrowed range.
func (l mirrorLine) admitRegions(budget *proofbound.WorkBudget, regions []joinRegion) error {
	work := freeform.NewFreeformWork()
	for _, region := range regions {
		if len(region.sel[0]) == 0 {
			return fmt.Errorf(`%w: the mirror line bounds no part of a region's outer loop, so the region lies across or away from it (J5)`, ErrUnsupported)
		}
		for li, loop := range region.loops {
			n := len(loop.Segments)
			for si, raw := range loop.Segments {
				if err := budget.Step(); err != nil {
					return err
				}
				seg, err := normalizeSegment(raw)
				if err != nil {
					return err
				}
				if _, ok := region.sel[li][si]; ok {
					s, ok := seg.(LineSeg)
					if !ok {
						return fmt.Errorf(`%w: a selected wall is a %T, and a curved wall names no mirror plane`, ErrDegenerate, seg)
					}
					start, end := lineWalkEnds(s)
					d := proofbound.RatAdd(
						proofbound.RatMul(new(big.Rat).Sub(ratOf(end.U), ratOf(start.U)), l.du),
						proofbound.RatMul(new(big.Rat).Sub(ratOf(end.V), ratOf(start.V)), l.dv))
					if d.Sign() <= 0 {
						return fmt.Errorf(`%w: selected walls on one line bound material on both of its sides (J5)`, ErrUnsupported)
					}
					continue
				}
				ends, err := l.admitSegment(seg, work)
				if err != nil {
					return fmt.Errorf(`loop %d segment %d: %w`, li, si, err)
				}
				_, prevSel := region.sel[li][(si+n-1)%n]
				_, nextSel := region.sel[li][(si+1)%n]
				if ends.closed {
					continue
				}
				if l.side(ends.start) == 0 && !prevSel {
					return fmt.Errorf(`%w: loop %d segment %d starts on the mirror line away from a selected wall (J5)`, ErrUnsupported, li, si)
				}
				if l.side(ends.end) == 0 && !nextSel {
					return fmt.Errorf(`%w: loop %d segment %d ends on the mirror line away from a selected wall (J5)`, ErrUnsupported, li, si)
				}
			}
		}
	}
	for _, region := range regions {
		trimmed, err := prismProfileHasTrimmedCircularSource(budget, ProfileRecord{Outer: region.loops[0], Holes: region.loops[1:]})
		if err != nil {
			return err
		}
		if trimmed {
			return fmt.Errorf(`%w: an arc or circle of the receiver is recorded over a narrowed range (J6)`, ErrUnsupported)
		}
	}
	return nil
}

// joinEnds is a segment's walk ends as the join reads them; computedStart
// and computedEnd mark an end the evaluator computed (a narrowed line's
// walked end) rather than one the record states.
type joinEnds struct {
	start, end                 Point2
	computedStart, computedEnd bool
	closed                     bool
}

// segmentJoinEnds reads seg's walk ends. A whole line's or arc's are its own
// recorded points; a narrowed line's come from walkOf, which evaluates the
// carrier at the recorded parameter.
func segmentJoinEnds(seg CurveSegment, work *freeform.FreeformWork) (joinEnds, error) {
	switch s := seg.(type) {
	case LineSeg:
		if prismcells.WholeSegmentRange(s.TStart, s.TEnd) {
			start, end := lineWalkEnds(s)
			return joinEnds{start: start, end: end}, nil
		}
		w, err := walkOf(s, work)
		if err != nil {
			return joinEnds{}, err
		}
		natural := func(t float64) bool { return t == 0 || t == 1 }
		return joinEnds{
			start:         Point2{U: w.StartU, V: w.StartV},
			end:           Point2{U: w.EndU, V: w.EndV},
			computedStart: !natural(s.TStart),
			computedEnd:   !natural(s.TEnd),
		}, nil
	case ArcSeg:
		if s.TStart > s.TEnd {
			return joinEnds{start: s.End, end: s.Start}, nil
		}
		return joinEnds{start: s.Start, end: s.End}, nil
	case CircleSeg:
		return joinEnds{closed: true}, nil
	default:
		return joinEnds{}, fmt.Errorf(`%w: a %T segment has no exact mirror image (J5)`, ErrUnsupported, seg)
	}
}

// admitSegment is J5's side test for one unselected segment: every point the
// test reads must have a non-negative exact cross product. A line's two
// recorded carrier ends bound its walk by convexity, and a narrowed line's
// walked ends are read too, since the build uses them. An arc or circle needs
// its whole circle on the side; an arc's squared radius is the larger of its
// two recorded radii, so a three-point arc whose ends sit at slightly
// different radii is tested at the farther one.
func (l mirrorLine) admitSegment(seg CurveSegment, work *freeform.FreeformWork) (joinEnds, error) {
	ends, err := segmentJoinEnds(seg, work)
	if err != nil {
		return joinEnds{}, err
	}
	wrongSide := fmt.Errorf(`%w: a boundary segment reaches across the mirror line, which the join would have to arrange (J5)`, ErrUnsupported)
	switch s := seg.(type) {
	case LineSeg:
		for _, p := range []Point2{s.Start, s.End, ends.start, ends.end} {
			if l.side(p) < 0 {
				return joinEnds{}, wrongSide
			}
		}
	case ArcSeg:
		if l.side(s.Start) < 0 || l.side(s.End) < 0 {
			return joinEnds{}, wrongSide
		}
		r2 := slices.MaxFunc([]*big.Rat{sqDist(s.Start, s.Center), sqDist(s.End, s.Center)}, func(a, b *big.Rat) int { return a.Cmp(b) })
		if !l.circleOnSide(s.Center, r2) {
			return joinEnds{}, wrongSide
		}
	case CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil {
			return joinEnds{}, fmt.Errorf(`%w: a circle radius is not a length: %s`, ErrUnsupported, err)
		}
		rr := ratOf(r)
		if rr == nil || !l.circleOnSide(s.Center, proofbound.RatMul(rr, rr)) {
			return joinEnds{}, wrongSide
		}
	}
	return ends, nil
}

// sqDist is |a − b|², exactly.
func sqDist(a, b Point2) *big.Rat {
	du := new(big.Rat).Sub(ratOf(a.U), ratOf(b.U))
	dv := new(big.Rat).Sub(ratOf(a.V), ratOf(b.V))
	return proofbound.RatAdd(proofbound.RatMul(du, du), proofbound.RatMul(dv, dv))
}

// mirrorReversedRun is a run of segments' image across the line walked back:
// rewindLoop's rule (prism_boolean.go) under the exact reflection, so the
// image of the run's last segment comes first and walks from the run's end
// back to its start. Per kind, a line becomes the whole line from m(walk end)
// to m(walk start), an arc {C, S, E} becomes {m(C), m(E), m(S)} over the same
// range, and a circle keeps its radius, CCW flag and range about m(C): the
// reflection reverses its winding and the reversed walk reverses it back.
//
// It returns the images beside how far any point of a held image can sit from
// the exact image of the segment's denoted walk: the largest point rounding,
// tripled when the run holds an arc (its centre and radius both move with the
// rounding of its three points, the argument offsetSectionDelta states for a
// recorded arc), plus the walk charge of any narrowed line, whose walked
// endpoints were computed.
func (l mirrorLine) mirrorReversedRun(budget *proofbound.WorkBudget, run []CurveSegment) ([]CurveSegment, float64, error) {
	pointMax := 0.0
	reflect := func(p Point2) (Point2, error) {
		m, e, err := l.reflect(p)
		pointMax = math.Max(pointMax, e)
		return m, err
	}
	img, walk, err := rewindLoop(budget, LoopRecord{Segments: run}, reflect)
	if err != nil {
		return nil, 0, err
	}
	for _, seg := range run {
		if _, ok := seg.(ArcSeg); ok {
			pointMax = proofbound.ProductUpper(3, pointMax)
			break
		}
	}
	if walk == 0 {
		return img.Segments, pointMax, nil
	}
	return img.Segments, proofbound.AbsSumUpper(pointMax, walk), nil
}

// assemble is §5.2's splice over one region. A loop with selected walls is
// cut at them into runs; each run closes with its own image walked back, so
// a loop with k selected walls yields k closed loops. The closed loop of
// largest positive signed area is the outer, and the others are holes. A
// loop with no selected wall stays, and its image joins it as another hole.
// It returns the assembled record, audited (§5.3), beside δ_mirror: the
// largest charge any image segment carries.
func (l mirrorLine) assemble(budget *proofbound.WorkBudget, region joinRegion) (ProfileRecord, float64, error) {
	work := freeform.NewFreeformWork()
	var closed, kept, images []LoopRecord
	delta := 0.0
	for li, loop := range region.loops {
		if err := budget.Step(); err != nil {
			return ProfileRecord{}, 0, err
		}
		segs := make([]CurveSegment, len(loop.Segments))
		for i, raw := range loop.Segments {
			seg, err := normalizeSegment(raw)
			if err != nil {
				return ProfileRecord{}, 0, err
			}
			segs[i] = seg
		}
		if len(region.sel[li]) == 0 {
			img, charge, err := l.mirrorReversedRun(budget, segs)
			if err != nil {
				return ProfileRecord{}, 0, err
			}
			kept = append(kept, LoopRecord{Segments: segs})
			images = append(images, LoopRecord{Segments: img})
			delta = math.Max(delta, charge)
			continue
		}
		cuts := make([]int, 0, len(region.sel[li]))
		for si := range region.sel[li] {
			cuts = append(cuts, si)
		}
		slices.Sort(cuts)
		n := len(segs)
		for t, a := range cuts {
			b := cuts[(t+1)%len(cuts)]
			var run []CurveSegment
			for i := (a + 1) % n; i != b; i = (i + 1) % n {
				run = append(run, segs[i])
			}
			if len(run) == 0 {
				continue
			}
			img, charge, err := l.mirrorReversedRun(budget, run)
			if err != nil {
				return ProfileRecord{}, 0, err
			}
			closed = append(closed, LoopRecord{Segments: append(slices.Clone(run), img...)})
			delta = math.Max(delta, charge)
		}
	}
	outer := -1
	best := 0.0
	for i, loop := range closed {
		area, err := loopSignedAreaBudget(budget, loop)
		if err != nil {
			return ProfileRecord{}, 0, err
		}
		if area > best {
			outer, best = i, area
		}
	}
	if outer < 0 {
		return ProfileRecord{}, 0, fmt.Errorf(`%w: the join's spliced loops enclose no material`, ErrDegenerate)
	}
	out := ProfileRecord{Outer: closed[outer]}
	for i, loop := range closed {
		if i != outer {
			out.Holes = append(out.Holes, loop)
		}
	}
	out.Holes = append(out.Holes, kept...)
	out.Holes = append(out.Holes, images...)
	if err := auditJoinedSection(budget, out, work); err != nil {
		return ProfileRecord{}, 0, err
	}
	return out, delta, nil
}

// auditJoinedSection is §5.3: the modify §5 audit with an empty blend map,
// in prism-boolean §6's order. The junction falsifier runs on every junction
// of every loop (RB9, ErrUnrecordableProfile); S8 reads each loop's signed
// area — the outer positive, every hole negative (RB3, ErrDegenerate); S7
// refuses a crossing or contact of non-adjacent segments within the
// diameter-anchored floor (RB4); S9 proves every hole inside the outer and
// outside every other hole (RB5/RB6). Every check only refuses.
func auditJoinedSection(budget *proofbound.WorkBudget, p ProfileRecord, work *freeform.FreeformWork) error {
	loops := append([]LoopRecord{p.Outer}, p.Holes...)
	for li, loop := range loops {
		joins := make([]loopJoin, len(loop.Segments))
		for si, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return err
			}
			ends, err := segmentJoinEnds(seg, work)
			if err != nil {
				return err
			}
			joins[si] = sketchrecord.RecordedJoin(ends.start, ends.end, ends.computedStart, ends.computedEnd, ends.closed)
		}
		if err := falsifyLoopJoins(fmt.Sprintf("mirror join loop %d", li), joins); err != nil {
			return err
		}
	}
	for li, loop := range loops {
		area, err := loopSignedAreaBudget(budget, loop)
		if err != nil {
			return err
		}
		if (li == 0) != (area > 0) || area == 0 {
			return fmt.Errorf(`%w: the join's spliced loop %d winds the wrong way or encloses nothing`, ErrDegenerate, li)
		}
	}
	segs, err := buildSegEntriesBudget(budget, loops)
	if err != nil {
		return err
	}
	if err := crossingAuditBudget(budget, segs); err != nil {
		return err
	}
	return nestingAuditBudget(budget, segs, len(loops))
}
