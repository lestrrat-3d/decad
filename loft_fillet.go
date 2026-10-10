package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// filletStraightLoft rounds recorded line/arc corners of an untwisted loft.
// Both profiles must state the same section, so replacing each with the same
// rewritten record preserves their one-to-one loft correspondence. Spline
// segments remain the original records; only the analytic corner is trimmed.
func (b *Body) filletStraightLoft(ctx context.Context, sel EdgeSelector, edges []*Edge, pl loftPayload, radius float64) (*Body, error) {
	if pl.surfaceResult || len(pl.profile0.Holes) != 0 || len(pl.profile1.Holes) != 0 ||
		!sectionrecord.IdenticalRecord(pl.profile0, pl.profile1) ||
		pl.plane0.U != pl.plane1.U || pl.plane0.V != pl.plane1.V {
		return nil, fmt.Errorf(`%w: a loft fillet needs matching, parallel, solid sections without holes`, ErrUnsupported)
	}
	for _, off := range pl.alignment {
		if off != 0 {
			return nil, fmt.Errorf(`%w: a loft fillet needs the two sections in the same segment order`, ErrUnsupported)
		}
	}
	shift := pl.plane1.Origin.Sub(pl.plane0.Origin)
	if shift.Dot(pl.plane0.U) != 0 || shift.Dot(pl.plane0.V) != 0 {
		return nil, fmt.Errorf(`%w: a loft fillet needs an axial section translation`, ErrUnsupported)
	}
	segments := pl.profile0.Outer.Segments
	n := len(segments)
	if n < 2 {
		return nil, fmt.Errorf(`%w: a loft fillet needs an analytic section corner`, ErrUnsupported)
	}
	walks := make([]survey2d.SideWalk, n)
	analytic := make([]bool, n)
	for i, seg := range segments {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		switch seg.(type) {
		case lineSeg, arcSeg:
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return nil, err
			}
			walks[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
			analytic[i] = true
		}
	}
	blends := make(map[int]*cornerBlend, len(edges))
	for _, edge := range edges {
		if _, ok := edge.curve.(Line3); !ok || edge.start == nil || edge.end == nil {
			return nil, fmt.Errorf(`%w: the selected loft edge is not an axial corner`, ErrUnsupported)
		}
		matched := -1
		for i := range segments {
			prev := (i + n - 1) % n
			if !analytic[prev] || !analytic[i] {
				continue
			}
			w := walks[i]
			bottom := pl.xform.Apply(pl.frame0.ToWorldUV(w.StartU, w.StartV))
			top := pl.xform.Apply(pl.frame1.ToWorldUV(w.StartU, w.StartV))
			if (edge.start.position == bottom && edge.end.position == top) ||
				(edge.start.position == top && edge.end.position == bottom) {
				if matched >= 0 {
					return nil, fmt.Errorf(`%w: a loft edge matches more than one section corner`, ErrUnsupported)
				}
				matched = i
			}
		}
		if matched < 0 {
			return nil, fmt.Errorf(`%w: a selected loft edge has no analytic section corner`, ErrUnsupported)
		}
		if _, exists := blends[matched]; exists {
			return nil, fmt.Errorf(`%w: a loft corner was selected twice`, ErrDegenerate)
		}
		cb, err := offset2d.Fillet(walks, matched, radius, filletTol)
		if err != nil {
			return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, sel, matched)
		}
		blends[matched] = cb
	}

	rewritten := make([]curveSegment, 0, n+len(blends))
	changed := make(map[int]bool, len(blends)*3)
	blendSegs := make(map[int]struct{}, len(blends))
	for i, original := range segments {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		start, end := blends[i], blends[(i+1)%n]
		if start != nil || end != nil {
			if !analytic[i] {
				return nil, fmt.Errorf(`%w: a loft fillet cannot trim a free-form carrier`, ErrUnsupported)
			}
			w := walks[i]
			from := sectionrecord.Point2{U: w.StartU, V: w.StartV}
			to := sectionrecord.Point2{U: w.EndU, V: w.EndV}
			if start != nil {
				from = start.FB
			}
			if end != nil {
				to = end.FA
			}
			cut := 0.0
			if start != nil {
				cut += start.CutbackB
			}
			if end != nil {
				cut += end.CutbackA
			}
			if cut >= w.Length-filletTol || offset2d.WalkConsumed(w, from, to, filletTol) {
				return nil, fmt.Errorf(`%w: loft fillets consume section segment %d`, ErrUnsupported, i)
			}
			rewritten = append(rewritten, offset2d.OriginalSegment(w, from.U, from.V, to.U, to.V))
			changed[len(rewritten)-1] = true
		} else {
			rewritten = append(rewritten, original)
		}
		if end != nil {
			rewritten = append(rewritten, end.Connector)
			changed[len(rewritten)-1] = true
			blendSegs[len(rewritten)-1] = struct{}{}
		}
	}
	profile := profileRecord{Outer: loopRecord{Segments: rewritten}}
	budget := proofbound.NewWorkBudget(ctx)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	limit := loftmesh.StationWorkLimit(0, uint64(len(rewritten)))
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	if err := auditLoftFilletProfile(budget, profile, changed, work0); err != nil {
		return nil, err
	}
	area0, err := profile.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work0)
	if err != nil {
		return nil, err
	}
	area1, err := profile.EvaluatorIntegralsContext(ctx, freeform.MomentAreaOrder, work1)
	if err != nil {
		return nil, err
	}
	if area0.Area <= 0 || area1.Area <= 0 {
		return nil, fmt.Errorf(`%w: the loft fillet consumes its section`, ErrDegenerate)
	}
	next := pl
	next.profile0, next.profile1 = profile, profile
	next.blendSegs = []map[int]struct{}{blendSegs}
	next.recordArea = [2]float64{area0.Area, area1.Area}
	next.verts, next.tris, next.walls = nil, nil, 0
	next.cell, next.side, next.capStartCount = nil, nil, 0
	next.proof = loftmesh.MeshProof{}
	ref := b.doc.nextProducerID()
	body, err := evalLoft(ctx, b.doc, ref, next, budget, work0, work1)
	if err != nil {
		return nil, err
	}
	return commitModifyResult(ctx, b, body)
}
