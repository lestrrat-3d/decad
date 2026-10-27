package smoothloft

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

type fitSectionCell struct {
	patch fitPatch
	seg   int
}

type fitSectionPart struct {
	cells    []fitSectionCell
	arc      []sectionrecord.Point2
	arcRound float64
	arcGap   float64
	arcWalk  survey2d.SegmentWalk
	arcSeg   sectionrecord.ArcSeg
	arcR     float64
}

func fitWalkSpans(w survey2d.SegmentWalk) []freeform.BezierSpan {
	return fitSpansInWalkOrder(w.Spans, w.Reversed)
}

func fitSpansInWalkOrder(spans []freeform.BezierSpan, reversed bool) []freeform.BezierSpan {
	if !reversed {
		return spans
	}
	out := make([]freeform.BezierSpan, len(spans))
	for i, span := range spans {
		controls := slices.Clone(span)
		slices.Reverse(controls)
		out[len(spans)-1-i] = controls
	}
	return out
}

func fitCommonCount(a, b, c int) (int, bool) {
	const maxCells = 512
	if a < 1 || b < 1 || c < 1 {
		return 0, false
	}
	gcd := func(x, y int) int {
		for y != 0 {
			x, y = y, x%y
		}
		return x
	}
	ab := a / gcd(a, b)
	if ab > maxCells/b {
		return 0, false
	}
	lcm := ab * b
	if lcm/gcd(lcm, c) > maxCells/c {
		return 0, false
	}
	return lcm / gcd(lcm, c) * c, true
}

func fitArcRadius(seg sectionrecord.ArcSeg) (float64, bool) {
	if seg.Center != (sectionrecord.Point2{}) || seg.TStart != 0 || seg.TEnd != 1 {
		return 0, false
	}
	r0 := add(mul(rf(seg.Start.U), rf(seg.Start.U)), mul(rf(seg.Start.V), rf(seg.Start.V)))
	r1 := add(mul(rf(seg.End.U), rf(seg.End.U)), mul(rf(seg.End.V), rf(seg.End.V)))
	if r0.Sign() <= 0 || r0.Cmp(r1) != 0 {
		return 0, false
	}
	r := proofbound.RatSqrtUp(r0)
	return r, !proofbound.IsNonFinite(r)
}

func fitArcWalkForward(seg sectionrecord.ArcSeg, w survey2d.SegmentWalk) bool {
	return !w.Reversed && w.Kind == survey2d.WalkCircular &&
		w.StartU == seg.Start.U && w.StartV == seg.Start.V &&
		w.EndU == seg.End.U && w.EndV == seg.End.V
}

func fitEndpoint(p freeform.RatPoint, q sectionrecord.Point2) bool {
	return p.U.Cmp(rf(q.U)) == 0 && p.V.Cmp(rf(q.V)) == 0
}

func fitSectionPlanes(planes [3]sectionrecord.PlaneRecord) (*big.Rat, error) {
	for i := range planes {
		if !proofbound.FiniteVec(planes[i].Origin) ||
			planes[i].U != (r3.Vec{X: 1}) || planes[i].V != (r3.Vec{Y: 1}) ||
			planes[i].Origin.X != planes[0].Origin.X || planes[i].Origin.Y != planes[0].Origin.Y {
			return nil, fmt.Errorf("%w: fitted sections need common world-XY axes and origin", decaderr.ErrUnsupported)
		}
	}
	z0, z1, z2 := rf(planes[0].Origin.Z), rf(planes[1].Origin.Z), rf(planes[2].Origin.Z)
	if z0.Cmp(z1) >= 0 || z1.Cmp(z2) >= 0 || mul(rint(2), z1).Cmp(add(z0, z2)) != 0 {
		return nil, fmt.Errorf("%w: fitted sections need ascending, equally spaced levels", decaderr.ErrUnsupported)
	}
	return sub(z2, z0), nil
}

func fitSectionParts(ctx context.Context, profiles [3]momentinput.Profile,
	work [3]*freeform.FreeformWork) ([4]fitSectionPart, error) {
	var parts [4]fitSectionPart
	for _, profile := range profiles {
		if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 4 {
			return parts, fmt.Errorf("%w: fitted sections need one four-edge outer loop", decaderr.ErrUnsupported)
		}
	}
	var walks [3]*momentinput.ProfileWalks
	for i := range profiles {
		var err error
		walks[i], err = momentinput.ResolveProfileWalks(profiles[i], work[i])
		if err != nil {
			return parts, err
		}
	}
	fitCount, arcCount := 0, 0
	budget := proofbound.NewWorkBudget(ctx)
	for j, raw := range profiles[0].Outer.Segments {
		if err := ctx.Err(); err != nil {
			return parts, err
		}
		switch seg := raw.(type) {
		case sectionrecord.FitSplineSeg:
			fitCount++
			if _, arc := profiles[0].Outer.Segments[(j+1)%4].(sectionrecord.ArcSeg); !arc {
				return parts, fmt.Errorf("%w: fitted flanks must alternate with fixed arcs", decaderr.ErrUnsupported)
			}
			var spans [3][]freeform.BezierSpan
			for i := range profiles {
				other, ok := profiles[i].Outer.Segments[j].(sectionrecord.FitSplineSeg)
				if !ok || other.TStart != seg.TStart || other.TEnd != seg.TEnd ||
					!((other.TStart == 0 && other.TEnd == 1) || (other.TStart == 1 && other.TEnd == 0)) {
					return parts, fmt.Errorf("%w: fitted flank ranges or kinds differ", decaderr.ErrUnsupported)
				}
				spans[i] = fitWalkSpans(walks[i].At(0, j))
			}
			common, ok := fitCommonCount(len(spans[0]), len(spans[1]), len(spans[2]))
			if !ok {
				return parts, fmt.Errorf("%w: fitted flank common span count exceeds 512", decaderr.ErrUnsupported)
			}
			for i := range spans {
				var err error
				spans[i], err = freeform.RefineSpanChain(spans[i], common/len(spans[i]), work[i])
				if err != nil {
					return parts, err
				}
			}
			for k := range common {
				patch, err := fitPatchOf([3]freeform.BezierSpan{spans[0][k], spans[1][k], spans[2][k]})
				if err != nil {
					return parts, err
				}
				if err := fitPatchPolarPositive(ctx, budget, patch); err != nil {
					return parts, err
				}
				parts[j].cells = append(parts[j].cells, fitSectionCell{patch: patch, seg: j})
			}
		case sectionrecord.ArcSeg:
			arcCount++
			r, ok := fitArcRadius(seg)
			if !ok {
				return parts, fmt.Errorf("%w: fixed arcs need origin-centered CCW whole arcs with exact equal radii", decaderr.ErrUnsupported)
			}
			for i := 1; i < 3; i++ {
				other, sameKind := profiles[i].Outer.Segments[j].(sectionrecord.ArcSeg)
				if !sameKind || other != seg {
					return parts, fmt.Errorf("%w: fitted sections need identical circular arcs", decaderr.ErrUnsupported)
				}
			}
			for i := range profiles {
				if !fitArcWalkForward(seg, walks[i].At(0, j)) {
					return parts, fmt.Errorf("%w: fixed arc walk must follow its positive polar direction", decaderr.ErrUnsupported)
				}
			}
			w := walks[0].At(0, j)
			stations, _, sagitta, matched, round, err := loftmesh.CircularCellPoints(w, w, seg, seg, 0.005)
			if err != nil {
				return parts, err
			}
			gap := sagitta
			for _, d := range matched {
				gap = math.Max(gap, d)
			}
			parts[j].arc, parts[j].arcRound, parts[j].arcGap = stations, round, gap
			parts[j].arcWalk, parts[j].arcSeg, parts[j].arcR = w, seg, r
		default:
			return parts, fmt.Errorf("%w: fitted sections need alternating fit splines and arcs", decaderr.ErrUnsupported)
		}
	}
	if fitCount != 2 || arcCount != 2 {
		return parts, fmt.Errorf("%w: fitted sections need two fitted flanks and two fixed arcs", decaderr.ErrUnsupported)
	}
	for i := range profiles {
		for j := range 4 {
			_, isFit := profiles[i].Outer.Segments[j].(sectionrecord.FitSplineSeg)
			if !isFit {
				continue
			}
			spans := fitWalkSpans(walks[i].At(0, j))
			if len(spans) == 0 {
				return parts, fmt.Errorf("%w: a fitted flank has no curve spans", decaderr.ErrUnsupported)
			}
			prev, okPrev := profiles[i].Outer.Segments[(j+3)%4].(sectionrecord.ArcSeg)
			next, okNext := profiles[i].Outer.Segments[(j+1)%4].(sectionrecord.ArcSeg)
			if !okPrev || !okNext || !fitEndpoint(spans[0][0], prev.End) ||
				!fitEndpoint(spans[len(spans)-1][3], next.Start) {
				return parts, fmt.Errorf("%w: fitted flank and fixed arc endpoints differ", decaderr.ErrUnsupported)
			}
		}
	}
	return parts, nil
}
