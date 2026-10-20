package loftrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftfillet"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// FilletStraight rounds recorded line/arc corners of an untwisted loft.
// Both profiles must state the same section, so replacing each with the same
// rewritten record preserves their one-to-one loft correspondence. Spline
// segments remain the original records; only the analytic corner is trimmed.
func FilletStraight(ctx context.Context, env topo.Env, selector string, edges []*topo.Edge,
	pl Payload, radius float64) (*topo.Body, error) {
	if pl.ConstructionDelta > 0 || len(pl.ConstructionArcUpper) != 0 || len(pl.ConstructionLengthExtra) != 0 {
		return nil, fmt.Errorf(`%w: a certified fitted-spline loft fillet cannot be filleted again`, decaderr.ErrUnsupported)
	}
	if pl.SurfaceResult || len(pl.Profile0.Holes) != 0 || len(pl.Profile1.Holes) != 0 ||
		!sectionrecord.IdenticalRecord(pl.Profile0, pl.Profile1) ||
		pl.Plane0.U != pl.Plane1.U || pl.Plane0.V != pl.Plane1.V {
		return nil, fmt.Errorf(`%w: a loft fillet needs matching, parallel, solid sections without holes`, decaderr.ErrUnsupported)
	}
	for _, off := range pl.Alignment {
		if off != 0 {
			return nil, fmt.Errorf(`%w: a loft fillet needs the two sections in the same segment order`, decaderr.ErrUnsupported)
		}
	}
	shift := pl.Plane1.Origin.Sub(pl.Plane0.Origin)
	if shift.Dot(pl.Plane0.U) != 0 || shift.Dot(pl.Plane0.V) != 0 {
		return nil, fmt.Errorf(`%w: a loft fillet needs an axial section translation`, decaderr.ErrUnsupported)
	}
	segments := pl.Profile0.Outer.Segments
	n := len(segments)
	if n < 2 {
		return nil, fmt.Errorf(`%w: a loft fillet needs an analytic section corner`, decaderr.ErrUnsupported)
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
		case sectionrecord.LineSeg, sectionrecord.ArcSeg:
			analytic[i] = true
		}
	}
	type loftCornerBlend struct {
		blend                 *offset2d.Blend
		fitParam, circleParam float64
		fitCircle             bool
		certificate           loftfillet.RootCertificate
		constructionDelta     float64
		idealArcLength        float64
		fitCarrierDeparture   float64
		rootCarrierDeparture  float64
		circleIdealParam      proofbound.RatInterval
	}
	blends := make(map[int]loftCornerBlend, len(edges))
	for _, edge := range edges {
		if _, ok := edge.Curve.(surfacegeom.Line3); !ok || edge.Start == nil || edge.End == nil {
			return nil, fmt.Errorf(`%w: the selected loft edge is not an axial corner`, decaderr.ErrUnsupported)
		}
		matched := -1
		for i := range segments {
			prev := (i + n - 1) % n
			if (!analytic[prev] || !analytic[i]) && !loftfillet.FitCirclePair(segments[prev], segments[i]) {
				continue
			}
			w := walks[i]
			bottom := pl.Xform.Apply(pl.Frame0.ToWorldUV(w.StartU, w.StartV))
			top := pl.Xform.Apply(pl.Frame1.ToWorldUV(w.StartU, w.StartV))
			if (edge.Start.Position == bottom && edge.End.Position == top) ||
				(edge.Start.Position == top && edge.End.Position == bottom) {
				if matched >= 0 {
					return nil, fmt.Errorf(`%w: a loft edge matches more than one section corner`, decaderr.ErrUnsupported)
				}
				matched = i
			}
		}
		if matched < 0 {
			return nil, fmt.Errorf(`%w: a selected loft edge has no analytic section corner`, decaderr.ErrUnsupported)
		}
		if _, exists := blends[matched]; exists {
			return nil, fmt.Errorf(`%w: a loft corner was selected twice`, decaderr.ErrDegenerate)
		}
		prev := (matched + n - 1) % n
		if loftfillet.FitCirclePair(segments[prev], segments[matched]) {
			cb, fitParam, circleParam, err := loftfillet.CircleBlend(segments, walks, matched, radius)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, selector, matched)
			}
			fitArrives := false
			fit, isFit := segments[prev].(sectionrecord.FitSplineSeg)
			circle, isCircle := segments[matched].(sectionrecord.CircleSeg)
			if isFit {
				fitArrives = true
			} else {
				circle, isCircle = segments[prev].(sectionrecord.CircleSeg)
				fit, isFit = segments[matched].(sectionrecord.FitSplineSeg)
			}
			if !isFit || !isCircle {
				return nil, fmt.Errorf(`%w: the loft fit-circle carrier changed kind`, decaderr.ErrUnsupported)
			}
			arrive, leave := walks[prev], walks[matched]
			cross := arrive.TanOutU*leave.TanInV - arrive.TanOutV*leave.TanInU
			if cross == 0 || math.IsNaN(cross) || math.IsInf(cross, 0) {
				return nil, fmt.Errorf(`%w: the loft fit-circle corner has no certified turn`, decaderr.ErrUnsupported)
			}
			connector, ok := cb.Connector.(sectionrecord.ArcSeg)
			if !ok {
				return nil, fmt.Errorf(`%w: the loft fit-circle connector is not an arc`, decaderr.ErrUnsupported)
			}
			cert, err := loftfillet.CertifyRoot(ctx, fit, circle, connector, fitParam, radius,
				fitArrives, math.Copysign(1, cross))
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, selector, matched)
			}
			departure, idealLength, err := loftfillet.ArcDeparture(cert, connector, fitArrives, radius)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, selector, matched)
			}
			fitCarrier, rootCarrier, circleIdealParam, err := loftfillet.CarrierDeparture(cert, circle,
				fitParam, circleParam, fitArrives)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, selector, matched)
			}
			blends[matched] = loftCornerBlend{
				blend: cb, fitParam: fitParam, circleParam: circleParam, fitCircle: true,
				certificate: cert, constructionDelta: departure, idealArcLength: idealLength,
				fitCarrierDeparture: fitCarrier, rootCarrierDeparture: rootCarrier,
				circleIdealParam: circleIdealParam,
			}
		} else {
			cb, err := offset2d.Fillet(walks, matched, radius, sectionaudit.Tolerance)
			if err != nil {
				return nil, fmt.Errorf(`%w; selector %s, section corner %d`, err, selector, matched)
			}
			blends[matched] = loftCornerBlend{blend: cb}
		}
	}

	rewritten := make([]sectionrecord.CurveSegment, 0, n+len(blends))
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
			if _, circle := original.(sectionrecord.CircleSeg); circle {
				startParam, endParam = start.circleIdealParam, end.circleIdealParam
			}
			increasing := false
			switch segment := original.(type) {
			case sectionrecord.FitSplineSeg:
				increasing = segment.TStart < segment.TEnd
			case sectionrecord.CircleSeg:
				increasing = segment.TStart < segment.TEnd
			default:
				return nil, fmt.Errorf(`%w: a two-ended loft fillet has no certified carrier`, decaderr.ErrUnsupported)
			}
			if increasing && startParam.Hi.Cmp(endParam.Lo) >= 0 ||
				!increasing && startParam.Lo.Cmp(endParam.Hi) <= 0 {
				return nil, fmt.Errorf(`%w: ideal loft fillets consume section segment %d`, decaderr.ErrUnsupported, i)
			}
		}
		carrierDeparture := 0.0
		if _, fit := original.(sectionrecord.FitSplineSeg); fit {
			if hasStart && start.fitCircle {
				carrierDeparture = math.Max(carrierDeparture, start.fitCarrierDeparture)
			}
			if hasEnd && end.fitCircle {
				carrierDeparture = math.Max(carrierDeparture, end.fitCarrierDeparture)
			}
		} else if _, circle := original.(sectionrecord.CircleSeg); circle {
			if hasStart && start.fitCircle {
				carrierDeparture = math.Max(carrierDeparture, start.rootCarrierDeparture)
			}
			if hasEnd && end.fitCircle {
				carrierDeparture = math.Max(carrierDeparture, end.rootCarrierDeparture)
			}
		}
		if hasStart || hasEnd {
			switch original := original.(type) {
			case sectionrecord.FitSplineSeg:
				if hasStart {
					if !start.fitCircle {
						return nil, fmt.Errorf(`%w: a loft fillet cannot trim this fit-spline carrier`, decaderr.ErrUnsupported)
					}
					original.TStart = start.fitParam
				}
				if hasEnd {
					if !end.fitCircle {
						return nil, fmt.Errorf(`%w: a loft fillet cannot trim this fit-spline carrier`, decaderr.ErrUnsupported)
					}
					original.TEnd = end.fitParam
				}
				if original.TStart == original.TEnd {
					return nil, fmt.Errorf(`%w: loft fillets consume section segment %d`, decaderr.ErrUnsupported, i)
				}
				rewritten = append(rewritten, original)
			case sectionrecord.CircleSeg:
				if hasStart {
					if !start.fitCircle {
						return nil, fmt.Errorf(`%w: a loft fillet cannot trim this circle carrier`, decaderr.ErrUnsupported)
					}
					original.TStart = start.circleParam
				}
				if hasEnd {
					if !end.fitCircle {
						return nil, fmt.Errorf(`%w: a loft fillet cannot trim this circle carrier`, decaderr.ErrUnsupported)
					}
					original.TEnd = end.circleParam
				}
				if original.CCW != (original.TStart < original.TEnd) {
					return nil, fmt.Errorf(`%w: loft fillets consume section segment %d`, decaderr.ErrUnsupported, i)
				}
				rewritten = append(rewritten, original)
			case sectionrecord.LineSeg, sectionrecord.ArcSeg:
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
				if cut >= w.Length-sectionaudit.Tolerance || offset2d.WalkConsumed(w, from, to, sectionaudit.Tolerance) {
					return nil, fmt.Errorf(`%w: loft fillets consume section segment %d`, decaderr.ErrUnsupported, i)
				}
				rewritten = append(rewritten, offset2d.OriginalSegment(w, from.U, from.V, to.U, to.V))
			default:
				return nil, fmt.Errorf(`%w: a loft fillet cannot trim this section carrier`, decaderr.ErrUnsupported)
			}
			// A trimmed fit spline is a subset of its authenticated source.
			// Only newly made analytic pieces need the free-form contact audit.
			if _, fit := original.(sectionrecord.FitSplineSeg); !fit {
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
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: rewritten}}
	budget := proofbound.NewWorkBudget(ctx)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	pl.RaiseReconstructionLimit(work0, work1)
	limit := pl.RewriteWorkLimit(uint64(len(rewritten)))
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	if err := loftfillet.AuditProfile(ctx, budget, profile, changed, constructionDelta, work0); err != nil {
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
		return nil, fmt.Errorf(`%w: the loft fillet consumes its section`, decaderr.ErrDegenerate)
	}
	next := pl
	next.Profile0, next.Profile1 = profile, profile
	next.BlendSegs = []map[int]struct{}{blendSegs}
	next.ConstructionDelta = constructionDelta
	next.ConstructionArcUpper = constructionArcUpper
	next.ConstructionLengthExtra = constructionLengthExtra
	next.RecordArea = [2]float64{area0.Area, area1.Area}
	next.Verts, next.Tris, next.Walls = nil, nil, 0
	next.Cell, next.Side, next.CapStartCount = nil, nil, 0
	next.Proof = loftmesh.MeshProof{}
	return Build(ctx, env, next, budget, work0, work1)
}
