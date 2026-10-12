package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/r3"
)

// prismPatternSource is a bounded source relation for a circular prism and
// rigid copies of one adjacent prism section. It does not prove a Union:
// the shared wall and the caps still need a constructed boundary and volume
// proof before any result can be published.
type prismPatternSource struct {
	blank    prismPayload
	section  prismPayload
	rootArc  arcSeg
	rootGap  float64
	sections []*Body
}

// prismPatternSourceOf checks only recorded Extrude sections and Copy
// placements. The first tool and its copies must share the very same source
// record; a profile merely similar to that source cannot enter this path.
func prismPatternSourceOf(ctx context.Context, operands []*Body) (*prismPatternSource, bool, error) {
	if len(operands) < 3 || len(operands) > 33 {
		return nil, false, nil
	}
	blank, ok := operands[0].payload.(prismPayload)
	if !ok || len(blank.profile.Holes) != 0 || len(blank.profile.Outer.Segments) != 1 ||
		blank.xform != r3.Identity() {
		return nil, false, nil
	}
	circle, ok := blank.profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center != (Point2{}) || circle.Radius.Base() <= 0 ||
		!prismPatternWholeRange(circle.TStart, circle.TEnd) {
		return nil, false, nil
	}
	first, ok := operands[1].payload.(prismPayload)
	if !ok || first.frame != blank.frame || first.z0 != blank.z0 || first.z1 != blank.z1 ||
		first.xform != r3.Identity() || len(first.profile.Holes) != 0 ||
		len(first.profile.Outer.Segments) < 4 {
		return nil, false, nil
	}
	segments := first.profile.Outer.Segments
	var root arcSeg
	rootIndex := -1
	for i, segment := range segments {
		arc, isArc := segment.(arcSeg)
		if !isArc || arc.Center != circle.Center || !prismPatternWholeRange(arc.TStart, arc.TEnd) {
			continue
		}
		gap, good := prismPatternArcGap(arc, circle.Radius.Base(), r3.Identity())
		if !good || gap >= 0.01 {
			continue
		}
		if rootIndex >= 0 {
			return nil, true, fmt.Errorf("%w: copied prism section has multiple possible circular joins", ErrUnsupported)
		}
		root, rootIndex = arc, i
	}
	if rootIndex < 0 {
		return nil, false, nil
	}
	if err := prismPatternOutsideRoot(ctx, segments, rootIndex, root, circle.Radius.Base()); err != nil {
		return nil, true, err
	}
	gap := 0.0
	for i, body := range operands[1:] {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		section, isPrism := body.payload.(prismPayload)
		if !isPrism || section.frame != first.frame || section.z0 != first.z0 ||
			section.z1 != first.z1 || !prismPatternAxialRotation(section.xform) ||
			len(section.profile.Outer.Segments) != len(segments) ||
			&section.profile.Outer.Segments[0] != &segments[0] {
			return nil, true, fmt.Errorf("%w: section %d is not a radial copy of the first prism", ErrUnsupported, i+1)
		}
		localGap, good := prismPatternArcGap(root, circle.Radius.Base(), section.xform)
		if !good {
			return nil, true, fmt.Errorf("%w: section %d has no finite circular-root bound", ErrUnsupported, i+1)
		}
		gap = max(gap, localGap)
	}
	if gap >= 0.01 {
		return nil, true, fmt.Errorf("%w: copied prism root arcs exceed the circular-wall bound", ErrUnsupported)
	}
	return &prismPatternSource{blank: blank, section: first, rootArc: root,
		rootGap: gap, sections: operands[1:]}, true, nil
}

func prismPatternWholeRange(start, end float64) bool {
	return start == 0 && end == 1 || start == 1 && end == 0
}

func prismPatternAxialRotation(t r3.Transform) bool {
	if !t.IsValid() || t.IsReflection() || t.Translation() != (r3.Vec{}) {
		return false
	}
	b := t.Basis()
	return b.EZ == (r3.Vec{Z: 1}) && b.EX.Z == 0 && b.EY.Z == 0 &&
		b.EX.X == b.EY.Y && b.EX.Y == -b.EY.X
}

// prismPatternArcGap bounds the difference between a placed arc's exact
// radius and the blank's radius. The arc's denoted radius comes from Start;
// its pinned End can differ, so the caller must charge that endpoint too.
func prismPatternArcGap(arc arcSeg, radius float64, motion r3.Transform) (float64, bool) {
	b := motion.Basis()
	c, s := proofarith.FloatRat(b.EX.X), proofarith.FloatRat(b.EX.Y)
	scale2 := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	x := new(big.Rat).Sub(proofarith.FloatRat(arc.Start.U), proofarith.FloatRat(arc.Center.U))
	y := new(big.Rat).Sub(proofarith.FloatRat(arc.Start.V), proofarith.FloatRat(arc.Center.V))
	r2 := new(big.Rat).Mul(scale2,
		new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
	lo, hi := proofbound.RatSqrtDown(r2), proofbound.RatSqrtUp(r2)
	if lo <= 0 || proofbound.IsNonFinite(hi) {
		return 0, false
	}
	r := proofarith.FloatRat(radius)
	d0 := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(lo), r))
	d1 := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(hi), r))
	if d1.Cmp(d0) > 0 {
		d0 = d1
	}
	gap := proofbound.AbsSumUpper(proofbound.RatFloatUp(d0), circularbounds.ArcRadialResidualUpper(arc))
	return gap, !proofbound.IsNonFinite(gap)
}

func prismPatternOutsideRoot(ctx context.Context, segments []curveSegment, rootIndex int,
	root arcSeg, radius float64) error {
	for i, segment := range segments {
		if err := ctx.Err(); err != nil {
			return err
		}
		if i == rootIndex {
			continue
		}
		switch seg := segment.(type) {
		case lineSeg:
			if i == (rootIndex+1)%len(segments) || (i+1)%len(segments) == rootIndex {
				if !prismPatternRootConnector(seg, root, radius) {
					return fmt.Errorf("%w: prism section connector %d has no outward radial proof", ErrUnsupported, i)
				}
				continue
			}
			if !prismPatternLineOutside(seg, radius) {
				return fmt.Errorf("%w: prism section line %d is not proven outside the circular wall", ErrUnsupported, i)
			}
		case fitSplineSeg:
			spans, err := splinebezier.FitSplineBezierSpans(seg, freeform.NewFreeformWork())
			if err != nil {
				return err
			}
			for _, span := range spans {
				if !prismPatternSpanOutside(span, radius) {
					return fmt.Errorf("%w: prism section spline %d is not proven outside the circular wall", ErrUnsupported, i)
				}
			}
		case arcSeg:
			if seg.Center != (Point2{}) || prismPatternRadialSq(seg.Start).Cmp(prismPatternRadiusSq(radius)) <= 0 {
				return fmt.Errorf("%w: prism section arc %d is not proven outside the circular wall", ErrUnsupported, i)
			}
		default:
			return fmt.Errorf("%w: prism section curve %d has no circular-wall proof", ErrUnsupported, i)
		}
	}
	return nil
}

func prismPatternRootConnector(line lineSeg, root arcSeg, radius float64) bool {
	near, far := line.Start, line.End
	if near != root.Start && near != root.End {
		near, far = far, near
	}
	if near != root.Start && near != root.End {
		return false
	}
	nu, nv := proofarith.FloatRat(near.U), proofarith.FloatRat(near.V)
	du := new(big.Rat).Sub(proofarith.FloatRat(far.U), nu)
	dv := new(big.Rat).Sub(proofarith.FloatRat(far.V), nv)
	radial := new(big.Rat).Add(new(big.Rat).Mul(nu, du), new(big.Rat).Mul(nv, dv))
	return radial.Sign() > 0 && prismPatternRadialSq(far).Cmp(prismPatternRadiusSq(radius)) > 0
}

func prismPatternRadiusSq(radius float64) *big.Rat {
	r := proofarith.FloatRat(radius)
	return new(big.Rat).Mul(r, r)
}

func prismPatternRadialSq(p Point2) *big.Rat {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	return new(big.Rat).Add(new(big.Rat).Mul(u, u), new(big.Rat).Mul(v, v))
}

func prismPatternLineOutside(line lineSeg, radius float64) bool {
	a, b := prismPatternRadialSq(line.Start), prismPatternRadialSq(line.End)
	r2 := prismPatternRadiusSq(radius)
	if a.Cmp(r2) <= 0 || b.Cmp(r2) <= 0 {
		return false
	}
	// The dot with one endpoint direction is a conservative linear support
	// certificate for every point on this line.
	return prismPatternControlOutside([]freeform.RatPoint{
		{U: proofarith.FloatRat(line.Start.U), V: proofarith.FloatRat(line.Start.V)},
		{U: proofarith.FloatRat(line.End.U), V: proofarith.FloatRat(line.End.V)},
	}, radius)
}

func prismPatternSpanOutside(span freeform.BezierSpan, radius float64) bool {
	return prismPatternControlOutside(span, radius)
}

// A Bezier span lies in its controls' convex hull. One separating line that
// leaves all controls beyond a circle proves the entire span outside it.
func prismPatternControlOutside(points []freeform.RatPoint, radius float64) bool {
	if len(points) == 0 {
		return false
	}
	du := new(big.Rat).Add(points[0].U, points[len(points)-1].U)
	dv := new(big.Rat).Add(points[0].V, points[len(points)-1].V)
	norm2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	if norm2.Sign() == 0 {
		return false
	}
	threshold := new(big.Rat).Mul(prismPatternRadiusSq(radius), norm2)
	for _, point := range points {
		dot := new(big.Rat).Add(new(big.Rat).Mul(du, point.U), new(big.Rat).Mul(dv, point.V))
		if dot.Sign() <= 0 || new(big.Rat).Mul(dot, dot).Cmp(threshold) <= 0 {
			return false
		}
	}
	return true
}
