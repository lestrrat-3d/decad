package decad

import (
	"context"
	"fmt"
	"math"

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
	if pl.constructionDelta > 0 || len(pl.constructionArcUpper) != 0 || len(pl.constructionLengthExtra) != 0 {
		return nil, fmt.Errorf(`%w: a certified fitted-spline loft fillet cannot be filleted again`, ErrUnsupported)
	}
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
	walkWork := freeform.NewFreeformWork()
	walkWork.RaiseLimit(loftmesh.StationWorkLimit(0, uint64(n)))
	for i, seg := range segments {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		w, err := boundarywalk.WalkOf(seg, walkWork)
		if err != nil {
			return nil, err
		}
		walks[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
		switch seg.(type) {
		case lineSeg, arcSeg:
			analytic[i] = true
		}
	}
	type loftCornerBlend struct {
		blend                 *cornerBlend
		fitParam, circleParam float64
		fitCircle             bool
		certificate           loftFitRootCertificate
		constructionDelta     float64
		idealArcLength        float64
		fitCarrierDeparture   float64
		rootCarrierDeparture  float64
		circleIdealParam      proofbound.RatInterval
	}
	blends := make(map[int]loftCornerBlend, len(edges))
	for _, edge := range edges {
		if _, ok := edge.curve.(Line3); !ok || edge.start == nil || edge.end == nil {
			return nil, fmt.Errorf(`%w: the selected loft edge is not an axial corner`, ErrUnsupported)
		}
		matched := -1
		for i := range segments {
			prev := (i + n - 1) % n
			if !(analytic[prev] && analytic[i]) && !loftFitCirclePair(segments[prev], segments[i]) {
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
		prev := (matched + n - 1) % n
		if loftFitCirclePair(segments[prev], segments[matched]) {
			cb, fitParam, circleParam, err := loftFitCircleBlend(segments, walks, matched, radius)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, sel, matched)
			}
			fitArrives := false
			fit, isFit := segments[prev].(fitSplineSeg)
			circle, isCircle := segments[matched].(circleSeg)
			if isFit {
				fitArrives = true
			} else {
				circle, isCircle = segments[prev].(circleSeg)
				fit, isFit = segments[matched].(fitSplineSeg)
			}
			if !isFit || !isCircle {
				return nil, fmt.Errorf(`%w: the loft fit-circle carrier changed kind`, ErrUnsupported)
			}
			arrive, leave := walks[prev], walks[matched]
			cross := arrive.TanOutU*leave.TanInV - arrive.TanOutV*leave.TanInU
			if cross == 0 || math.IsNaN(cross) || math.IsInf(cross, 0) {
				return nil, fmt.Errorf(`%w: the loft fit-circle corner has no certified turn`, ErrUnsupported)
			}
			connector, ok := cb.Connector.(arcSeg)
			if !ok {
				return nil, fmt.Errorf(`%w: the loft fit-circle connector is not an arc`, ErrUnsupported)
			}
			cert, err := certifyLoftFitCircleRoot(ctx, fit, circle, connector, fitParam, radius,
				fitArrives, math.Copysign(1, cross))
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, sel, matched)
			}
			departure, idealLength, err := loftFitArcDeparture(cert, connector, fitArrives, radius)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, sel, matched)
			}
			fitCarrier, rootCarrier, circleIdealParam, err := loftFitCarrierDeparture(cert, circle,
				fitParam, circleParam, fitArrives)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, sel, matched)
			}
			blends[matched] = loftCornerBlend{
				blend: cb, fitParam: fitParam, circleParam: circleParam, fitCircle: true,
				certificate: cert, constructionDelta: departure, idealArcLength: idealLength,
				fitCarrierDeparture: fitCarrier, rootCarrierDeparture: rootCarrier,
				circleIdealParam: circleIdealParam,
			}
		} else {
			cb, err := offset2d.Fillet(walks, matched, radius, filletTol)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, sel, matched)
			}
			blends[matched] = loftCornerBlend{blend: cb}
		}
	}

	rewritten := make([]curveSegment, 0, n+len(blends))
	changed := make(map[int]bool, len(blends)*3)
	blendSegs := make(map[int]struct{}, len(blends))
	constructionDelta := 0.0
	constructionArcUpper := make(map[int]float64, len(blends))
	constructionLengthExtra := make(map[int]float64, len(blends)*2)
	for i, original := range segments {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		start, hasStart := blends[i]
		end, hasEnd := blends[(i+1)%n]
		if hasStart && hasEnd && start.fitCircle && end.fitCircle {
			startParam, endParam := start.certificate.Param, end.certificate.Param
			if _, circle := original.(circleSeg); circle {
				startParam, endParam = start.circleIdealParam, end.circleIdealParam
			}
			increasing := false
			switch segment := original.(type) {
			case fitSplineSeg:
				increasing = segment.TStart < segment.TEnd
			case circleSeg:
				increasing = segment.TStart < segment.TEnd
			default:
				return nil, fmt.Errorf(`%w: a two-ended loft fillet has no certified carrier`, ErrUnsupported)
			}
			if increasing && startParam.Hi.Cmp(endParam.Lo) >= 0 ||
				!increasing && startParam.Lo.Cmp(endParam.Hi) <= 0 {
				return nil, fmt.Errorf(`%w: ideal loft fillets consume section segment %d`, ErrUnsupported, i)
			}
		}
		carrierDeparture := 0.0
		if _, fit := original.(fitSplineSeg); fit {
			if hasStart && start.fitCircle {
				carrierDeparture = math.Max(carrierDeparture, start.fitCarrierDeparture)
			}
			if hasEnd && end.fitCircle {
				carrierDeparture = math.Max(carrierDeparture, end.fitCarrierDeparture)
			}
		} else if _, circle := original.(circleSeg); circle {
			if hasStart && start.fitCircle {
				carrierDeparture = math.Max(carrierDeparture, start.rootCarrierDeparture)
			}
			if hasEnd && end.fitCircle {
				carrierDeparture = math.Max(carrierDeparture, end.rootCarrierDeparture)
			}
		}
		if hasStart || hasEnd {
			switch original := original.(type) {
			case fitSplineSeg:
				if hasStart {
					if !start.fitCircle {
						return nil, fmt.Errorf(`%w: a loft fillet cannot trim this fit-spline carrier`, ErrUnsupported)
					}
					original.TStart = start.fitParam
				}
				if hasEnd {
					if !end.fitCircle {
						return nil, fmt.Errorf(`%w: a loft fillet cannot trim this fit-spline carrier`, ErrUnsupported)
					}
					original.TEnd = end.fitParam
				}
				if original.TStart == original.TEnd {
					return nil, fmt.Errorf(`%w: loft fillets consume section segment %d`, ErrUnsupported, i)
				}
				rewritten = append(rewritten, original)
			case circleSeg:
				if hasStart {
					if !start.fitCircle {
						return nil, fmt.Errorf(`%w: a loft fillet cannot trim this circle carrier`, ErrUnsupported)
					}
					original.TStart = start.circleParam
				}
				if hasEnd {
					if !end.fitCircle {
						return nil, fmt.Errorf(`%w: a loft fillet cannot trim this circle carrier`, ErrUnsupported)
					}
					original.TEnd = end.circleParam
				}
				if original.CCW != (original.TStart < original.TEnd) {
					return nil, fmt.Errorf(`%w: loft fillets consume section segment %d`, ErrUnsupported, i)
				}
				rewritten = append(rewritten, original)
			case lineSeg, arcSeg:
				w := walks[i]
				from := sectionrecord.Point2{U: w.StartU, V: w.StartV}
				to := sectionrecord.Point2{U: w.EndU, V: w.EndV}
				if hasStart {
					from = start.blend.FB
				}
				if hasEnd {
					to = end.blend.FA
				}
				cut := 0.0
				if hasStart {
					cut += start.blend.CutbackB
				}
				if hasEnd {
					cut += end.blend.CutbackA
				}
				if cut >= w.Length-filletTol || offset2d.WalkConsumed(w, from, to, filletTol) {
					return nil, fmt.Errorf(`%w: loft fillets consume section segment %d`, ErrUnsupported, i)
				}
				rewritten = append(rewritten, offset2d.OriginalSegment(w, from.U, from.V, to.U, to.V))
			default:
				return nil, fmt.Errorf(`%w: a loft fillet cannot trim this section carrier`, ErrUnsupported)
			}
			// A trimmed fit spline is a subset of its authenticated source.
			// Only newly made analytic pieces need the free-form contact audit.
			if _, fit := original.(fitSplineSeg); !fit {
				changed[len(rewritten)-1] = true
			}
		} else {
			rewritten = append(rewritten, original)
		}
		if carrierDeparture > 0 {
			constructionDelta = math.Max(constructionDelta, carrierDeparture)
			constructionLengthExtra[len(rewritten)-1] = proofbound.ProductUpper(2, carrierDeparture)
		}
		if hasEnd {
			rewritten = append(rewritten, end.blend.Connector)
			changed[len(rewritten)-1] = true
			blendSegs[len(rewritten)-1] = struct{}{}
			if end.fitCircle {
				constructionDelta = math.Max(constructionDelta, end.constructionDelta)
				constructionArcUpper[len(rewritten)-1] = end.idealArcLength
			}
		}
	}
	profile := profileRecord{Outer: loopRecord{Segments: rewritten}}
	budget := proofbound.NewWorkBudget(ctx)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	pl.raiseReconstructionLimit(work0, work1)
	limit := pl.rewriteWorkLimit(uint64(len(rewritten)))
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	if err := auditLoftFilletProfile(ctx, budget, profile, changed, constructionDelta, work0); err != nil {
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
	next.constructionDelta = constructionDelta
	next.constructionArcUpper = constructionArcUpper
	next.constructionLengthExtra = constructionLengthExtra
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

func loftFitCirclePair(a, b curveSegment) bool {
	_, aFit := a.(fitSplineSeg)
	_, bFit := b.(fitSplineSeg)
	_, aCircle := a.(circleSeg)
	_, bCircle := b.(circleSeg)
	return (aFit && bCircle) || (aCircle && bFit)
}
