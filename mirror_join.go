package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/mirrorjoin"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-go/option/v3"
)

// This file adapts internal/mirrorjoin's record rewrite to prism and stacked
// payloads, then runs the modify §5 audit on each assembled region.

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
	if sp, ok := b.payload.(stackedPrismPayload); ok && sp.isGroup() {
		// A prism group's lumps are separate regions, which this join does
		// not splice one by one.
		return nil, fmt.Errorf(`%w: a mirror join does not rewrite a prism group (J1)`, ErrUnsupported)
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
	// J1: the join re-derives each interface from exclusive holes, which a
	// union-built stack's changing outer loop does not record.
	if runs, err := sp.outerRuns(); err != nil {
		return stackedPrismPayload{}, err
	} else if len(runs) != 1 {
		return stackedPrismPayload{}, fmt.Errorf(`%w: a union-built stack changes its outer loop between slabs, which the join does not rewrite; join the operands before the union (J1)`, ErrUnsupported)
	}
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
	interfaces, err := stackedInterfaces(ctx, out.slabs, sp.interfaces)
	if err != nil {
		return stackedPrismPayload{}, err
	}
	out.interfaces = interfaces
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

// mirrorLine adapts root callers to the record admission and splice.
type mirrorLine struct{ line mirrorjoin.Line }

func admitMirrorLine(walls []LineSeg) (mirrorLine, error) {
	line, err := mirrorjoin.AdmitLine(walls)
	return mirrorLine{line: line}, err
}

func (l mirrorLine) admitRegions(budget *proofbound.WorkBudget, regions []joinRegion) error {
	records := make([]mirrorjoin.Region, len(regions))
	for i, region := range regions {
		records[i] = mirrorjoin.Region{Loops: region.loops, Sel: region.sel}
	}
	return l.line.AdmitRegions(budget, records)
}

func (l mirrorLine) mirrorReversedRun(budget *proofbound.WorkBudget, run []CurveSegment) ([]CurveSegment, float64, error) {
	return l.line.ReverseRun(budget, run)
}

func segmentJoinEnds(seg CurveSegment, work *freeform.FreeformWork) (mirrorjoin.Ends, error) {
	return mirrorjoin.SegmentEnds(seg, work)
}

// assemble adapts §5.2's record splice and runs §5.3's section audit.
func (l mirrorLine) assemble(budget *proofbound.WorkBudget, region joinRegion) (ProfileRecord, float64, error) {
	work := freeform.NewFreeformWork()
	spliced, err := l.line.Splice(budget, mirrorjoin.Region{Loops: region.loops, Sel: region.sel})
	if err != nil {
		return ProfileRecord{}, 0, err
	}
	out := ProfileRecord{Outer: spliced.Outer, Holes: spliced.Holes}
	if err := auditJoinedSection(budget, out, work); err != nil {
		return ProfileRecord{}, 0, err
	}
	return out, spliced.Delta, nil
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
			joins[si] = sketchrecord.RecordedJoin(
				ends.Start, ends.End, ends.ComputedStart, ends.ComputedEnd, ends.Closed)
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
