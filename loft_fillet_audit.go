package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type loftSectionBox struct{ u, v proofbound.RatInterval }

func loftLineCoordinate(a, b, t float64) *big.Rat {
	ra, _ := proofbound.RatOf(a)
	rb, _ := proofbound.RatOf(b)
	rt, _ := proofbound.RatOf(t)
	return new(big.Rat).Add(ra, new(big.Rat).Mul(rt, new(big.Rat).Sub(rb, ra)))
}

func loftPointInterval(a, b *big.Rat) proofbound.RatInterval {
	return proofbound.Interval(proofbound.RatMin(a, b), proofbound.RatMax(a, b))
}

func loftSectionSegmentBox(seg curveSegment, w survey2d.SegmentWalk) (loftSectionBox, error) {
	switch seg := seg.(type) {
	case lineSeg:
		return loftSectionBox{
			loftPointInterval(loftLineCoordinate(seg.Start.U, seg.End.U, seg.TStart),
				loftLineCoordinate(seg.Start.U, seg.End.U, seg.TEnd)),
			loftPointInterval(loftLineCoordinate(seg.Start.V, seg.End.V, seg.TStart),
				loftLineCoordinate(seg.Start.V, seg.End.V, seg.TEnd)),
		}, nil
	case arcSeg:
		record := circularbounds.RecordSegment(seg)
		arc, ok := record.(circularbounds.ArcSeg)
		if !ok {
			return loftSectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound an arc`, ErrUnsupported)
		}
		u, v, ok := circularbounds.ArcBox(arc)
		if !ok {
			return loftSectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound an arc`, ErrUnsupported)
		}
		return loftSectionBox{u, v}, nil
	case circleSeg:
		record := circularbounds.RecordSegment(seg)
		circle, ok := record.(circularbounds.CircleSeg)
		if !ok {
			return loftSectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound a circle`, ErrUnsupported)
		}
		u, v, ok := circularbounds.CircleBox(circle)
		if !ok {
			return loftSectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound a circle`, ErrUnsupported)
		}
		return loftSectionBox{u, v}, nil
	default:
		if w.Kind != survey2d.WalkFreeform || len(w.Spans) == 0 {
			return loftSectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound a section segment`, ErrUnsupported)
		}
		var out loftSectionBox
		first := true
		for _, span := range w.Spans {
			for _, pt := range span {
				if first {
					out.u = proofbound.PointInterval(pt.U)
					out.v = proofbound.PointInterval(pt.V)
					first = false
					continue
				}
				out.u.Lo = proofbound.RatMin(out.u.Lo, pt.U)
				out.u.Hi = proofbound.RatMax(out.u.Hi, pt.U)
				out.v.Lo = proofbound.RatMin(out.v.Lo, pt.V)
				out.v.Hi = proofbound.RatMax(out.v.Hi, pt.V)
			}
		}
		return out, nil
	}
}

func loftBoxSeparated(a, b loftSectionBox, gap *big.Rat) bool {
	separated := func(a, b proofbound.RatInterval) bool {
		return new(big.Rat).Add(a.Hi, gap).Cmp(b.Lo) < 0 ||
			new(big.Rat).Add(b.Hi, gap).Cmp(a.Lo) < 0
	}
	return separated(a.u, b.u) || separated(a.v, b.v)
}

// auditLoftFilletProfile keeps the existing authenticated free-form pieces
// verbatim. Only new analytic pieces can introduce a new contact with them.
// Their complete circular range is enclosed by ArcBox; a Bézier lies in its
// exact control hull. A pair whose enclosing boxes do not separate is refused.
func auditLoftFilletProfile(ctx context.Context, budget *proofbound.WorkBudget,
	profile profileRecord, changed map[int]bool, constructionDelta float64,
	work *freeform.FreeformWork) error {
	if constructionDelta < 0 || proofbound.IsNonFinite(constructionDelta) {
		return fmt.Errorf(`%w: the loft fillet construction departure is underivable`, ErrUnsupported)
	}
	segments := profile.Outer.Segments
	boxes := make([]loftSectionBox, len(segments))
	free := make([]bool, len(segments))
	var analytic []sectionaudit.Entry
	for i, seg := range segments {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return err
		}
		free[i] = w.Kind == survey2d.WalkFreeform
		if !free[i] {
			analytic = append(analytic, sectionaudit.NewEntry(0, i, len(segments), w))
		}
		boxes[i], err = loftSectionSegmentBox(seg, w)
		if err != nil {
			return err
		}
	}
	minU, minV := boxes[0].u.Lo, boxes[0].v.Lo
	maxU, maxV := boxes[0].u.Hi, boxes[0].v.Hi
	for _, box := range boxes[1:] {
		minU = proofbound.RatMin(minU, box.u.Lo)
		minV = proofbound.RatMin(minV, box.v.Lo)
		maxU = proofbound.RatMax(maxU, box.u.Hi)
		maxV = proofbound.RatMax(maxV, box.v.Hi)
	}
	du, dv := new(big.Rat).Sub(maxU, minU), new(big.Rat).Sub(maxV, minV)
	d2 := proofbound.RatAdd(proofbound.RatMul(du, du), proofbound.RatMul(dv, dv))
	diameter := proofbound.RatSqrtUp(d2)
	if math.IsInf(diameter, 0) || math.IsNaN(diameter) {
		return fmt.Errorf(`%w: the loft fillet's section diameter cannot be bounded`, ErrUnsupported)
	}
	contactEps, _ := proofbound.RatOf(math.Nextafter(sectionaudit.ContactEps, math.Inf(1)))
	delta := new(big.Rat).SetFloat64(constructionDelta)
	twoDelta := new(big.Rat).Mul(delta, big.NewRat(2, 1))
	threeDelta := new(big.Rat).Mul(delta, big.NewRat(3, 1))
	gap := proofbound.RatAdd(proofbound.RatMul(
		new(big.Rat).Add(new(big.Rat).SetFloat64(diameter), threeDelta), contactEps), twoDelta)
	if err := sectionaudit.CrossingWithFloor(budget, analytic, proofbound.RatFloatUp(gap)); err != nil {
		return err
	}
	for i := range segments {
		if !changed[i] {
			continue
		}
		for j := range segments {
			if i == j {
				continue
			}
			if !free[j] && constructionDelta == 0 {
				continue // the existing analytic crossing audit covers this pair
			}
			if j == (i+1)%len(segments) || i == (j+1)%len(segments) {
				if !free[j] {
					continue
				}
				if arc, isArc := segments[i].(arcSeg); isArc {
					if fit, isFit := segments[j].(fitSplineSeg); isFit {
						if err := certifyRecordedFitArcSeparation(ctx, budget, fit, arc,
							j == (i+len(segments)-1)%len(segments), work); err != nil {
							return fmt.Errorf("%w; connector %d, fit %d", err, i, j)
						}
					}
				}
				continue
			}
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return err
			}
			if !loftBoxSeparated(boxes[i], boxes[j], gap) {
				return fmt.Errorf(`%w: loft fillet segment %d may contact segment %d`, ErrUnsupported, i, j)
			}
		}
	}
	for i, segment := range segments {
		if !changed[i] {
			continue
		}
		arc, ok := segment.(arcSeg)
		if !ok {
			continue
		}
		neighbors := []int{(i + len(segments) - 1) % len(segments), (i + 1) % len(segments)}
		_, fitBefore := segments[neighbors[0]].(fitSplineSeg)
		_, fitAfter := segments[neighbors[1]].(fitSplineSeg)
		if !fitBefore && !fitAfter {
			continue
		}
		for _, j := range neighbors {
			circle, isCircle := segments[j].(circleSeg)
			if isCircle {
				if err := certifyRecordedCircleArcSeparation(circle, arc); err != nil {
					return err
				}
			}
		}
	}
	return nil
}
