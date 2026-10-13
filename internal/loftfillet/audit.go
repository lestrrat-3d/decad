package loftfillet

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type sectionBox struct{ u, v proofbound.RatInterval }

func lineCoordinate(a, b, t float64) *big.Rat {
	ra, _ := proofbound.RatOf(a)
	rb, _ := proofbound.RatOf(b)
	rt, _ := proofbound.RatOf(t)
	return new(big.Rat).Add(ra, new(big.Rat).Mul(rt, new(big.Rat).Sub(rb, ra)))
}

func pointInterval(a, b *big.Rat) proofbound.RatInterval {
	return proofbound.Interval(proofbound.RatMin(a, b), proofbound.RatMax(a, b))
}

func sectionSegmentBox(seg sectionrecord.CurveSegment, w survey2d.SegmentWalk) (sectionBox, error) {
	switch seg := seg.(type) {
	case sectionrecord.LineSeg:
		return sectionBox{
			pointInterval(lineCoordinate(seg.Start.U, seg.End.U, seg.TStart),
				lineCoordinate(seg.Start.U, seg.End.U, seg.TEnd)),
			pointInterval(lineCoordinate(seg.Start.V, seg.End.V, seg.TStart),
				lineCoordinate(seg.Start.V, seg.End.V, seg.TEnd)),
		}, nil
	case sectionrecord.ArcSeg:
		record := circularbounds.RecordSegment(seg)
		arc, ok := record.(circularbounds.ArcSeg)
		if !ok {
			return sectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound an arc`, decaderr.ErrUnsupported)
		}
		u, v, ok := circularbounds.ArcBox(arc)
		if !ok {
			return sectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound an arc`, decaderr.ErrUnsupported)
		}
		return sectionBox{u, v}, nil
	case sectionrecord.CircleSeg:
		record := circularbounds.RecordSegment(seg)
		circle, ok := record.(circularbounds.CircleSeg)
		if !ok {
			return sectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound a circle`, decaderr.ErrUnsupported)
		}
		u, v, ok := circularbounds.CircleBox(circle)
		if !ok {
			return sectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound a circle`, decaderr.ErrUnsupported)
		}
		return sectionBox{u, v}, nil
	default:
		if w.Kind != survey2d.WalkFreeform || len(w.Spans) == 0 {
			return sectionBox{}, fmt.Errorf(`%w: the loft fillet cannot bound a section segment`, decaderr.ErrUnsupported)
		}
		var out sectionBox
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

func boxSeparated(a, b sectionBox, gap *big.Rat) bool {
	separated := func(a, b proofbound.RatInterval) bool {
		return new(big.Rat).Add(a.Hi, gap).Cmp(b.Lo) < 0 ||
			new(big.Rat).Add(b.Hi, gap).Cmp(a.Lo) < 0
	}
	return separated(a.u, b.u) || separated(a.v, b.v)
}

// AuditProfile keeps the existing authenticated free-form pieces
// verbatim. Only new analytic pieces can introduce a new contact with them.
// Their complete circular range is enclosed by ArcBox; a Bézier lies in its
// exact control hull. A pair whose enclosing boxes do not separate is refused.
func AuditProfile(ctx context.Context, budget *proofbound.WorkBudget,
	profile momentinput.Profile, changed map[int]bool, constructionDelta float64,
	work *freeform.FreeformWork) error {
	if constructionDelta < 0 || proofbound.IsNonFinite(constructionDelta) {
		return fmt.Errorf(`%w: the loft fillet construction departure is underivable`, decaderr.ErrUnsupported)
	}
	segments := profile.Outer.Segments
	boxes := make([]sectionBox, len(segments))
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
		boxes[i], err = sectionSegmentBox(seg, w)
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
		return fmt.Errorf(`%w: the loft fillet's section diameter cannot be bounded`, decaderr.ErrUnsupported)
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
				if arc, isArc := segments[i].(sectionrecord.ArcSeg); isArc {
					if fit, isFit := segments[j].(sectionrecord.FitSplineSeg); isFit {
						if err := recordedFitArcSeparation(ctx, budget, fit, arc,
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
			if !boxSeparated(boxes[i], boxes[j], gap) {
				return fmt.Errorf(`%w: loft fillet segment %d may contact segment %d`, decaderr.ErrUnsupported, i, j)
			}
		}
	}
	for i, segment := range segments {
		if !changed[i] {
			continue
		}
		arc, ok := segment.(sectionrecord.ArcSeg)
		if !ok {
			continue
		}
		neighbors := []int{(i + len(segments) - 1) % len(segments), (i + 1) % len(segments)}
		_, fitBefore := segments[neighbors[0]].(sectionrecord.FitSplineSeg)
		_, fitAfter := segments[neighbors[1]].(sectionrecord.FitSplineSeg)
		if !fitBefore && !fitAfter {
			continue
		}
		for _, j := range neighbors {
			circle, isCircle := segments[j].(sectionrecord.CircleSeg)
			if isCircle {
				if err := recordedCircleArcSeparation(circle, arc); err != nil {
					return err
				}
			}
		}
	}
	return nil
}
