// Package prismpattern holds the prism-pattern Union's exact radial source
// tests and joined-prism fillet contact arithmetic: blank and section arc
// gaps, outward wall certificates, interval contact points, polar angles and
// ring points. See docs/prism-pattern-union-design.md.
package prismpattern

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// SourceOf checks only recorded Extrude sections and Copy placements. The
// first tool and its copies must share the same source record; a merely
// similar profile cannot enter this path.
func SourceOf(ctx context.Context, operands []*topo.Body) (*facetedrecord.PrismPatternSource, bool, error) {
	if len(operands) < 3 || len(operands) > 33 {
		return nil, false, nil
	}
	blank, ok := operands[0].Payload.(prismrecord.Payload)
	if !ok || len(blank.Profile.Holes) != 0 || len(blank.Profile.Outer.Segments) != 1 ||
		blank.Xform != r3.Identity() {
		return nil, false, nil
	}
	circle, ok := blank.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || circle.Center != (sectionrecord.Point2{}) || circle.Radius.Base() <= 0 ||
		!WholeRange(circle.TStart, circle.TEnd) {
		return nil, false, nil
	}
	first, ok := operands[1].Payload.(prismrecord.Payload)
	if !ok || first.Frame != blank.Frame || first.Z0 != blank.Z0 || first.Z1 != blank.Z1 ||
		first.Xform != r3.Identity() || len(first.Profile.Holes) != 0 ||
		len(first.Profile.Outer.Segments) < 4 {
		return nil, false, nil
	}
	segments := first.Profile.Outer.Segments
	var root sectionrecord.ArcSeg
	rootIndex := -1
	for i, segment := range segments {
		arc, isArc := segment.(sectionrecord.ArcSeg)
		if !isArc || arc.Center != circle.Center || !WholeRange(arc.TStart, arc.TEnd) {
			continue
		}
		gap, good := ArcGap(arc, circle.Radius.Base(), r3.Identity())
		if !good || gap >= 0.01 {
			continue
		}
		if rootIndex >= 0 {
			return nil, true, fmt.Errorf("%w: copied prism section has multiple possible circular joins", decaderr.ErrUnsupported)
		}
		root, rootIndex = arc, i
	}
	if rootIndex < 0 {
		return nil, false, nil
	}
	gap := 0.0
	leastScale2 := big.NewRat(1, 1)
	blankRadiusSq := RadiusSq(circle.Radius.Base())
	for i, body := range operands[1:] {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		section, isPrism := body.Payload.(prismrecord.Payload)
		if !isPrism || section.Frame != first.Frame || section.Z0 != first.Z0 ||
			section.Z1 != first.Z1 || !AxialRotation(section.Xform) ||
			len(section.Profile.Outer.Segments) != len(segments) ||
			&section.Profile.Outer.Segments[0] != &segments[0] {
			return nil, true, fmt.Errorf("%w: section %d is not a radial copy of the first prism", decaderr.ErrUnsupported, i+1)
		}
		localGap, good := ArcGap(root, circle.Radius.Base(), section.Xform)
		if !good {
			return nil, true, fmt.Errorf("%w: section %d has no finite circular-root bound", decaderr.ErrUnsupported, i+1)
		}
		gap = max(gap, localGap)
		scale2 := Scale2(section.Xform)
		if scale2.Sign() <= 0 {
			return nil, true, fmt.Errorf("%w: section %d has no positive radial placement scale", decaderr.ErrUnsupported, i+1)
		}
		if !RootInside(root, blankRadiusSq, scale2) {
			return nil, true, fmt.Errorf("%w: section %d circular root is not strictly inside the blank",
				decaderr.ErrUnsupported, i+1)
		}
		if scale2.Cmp(leastScale2) < 0 {
			leastScale2 = scale2
		}
	}
	if gap >= 0.01 {
		return nil, true, fmt.Errorf("%w: copied prism root arcs exceed the circular-wall bound", decaderr.ErrUnsupported)
	}
	// The exact dyadic placement can differ from a unit rotation by one or
	// more ulps. Prove every non-root source outside the blank after the
	// smallest radial scale among all copies, not only in source coordinates.
	outsideRadiusSq := new(big.Rat).Quo(blankRadiusSq, leastScale2)
	if err := OutsideRoot(ctx, segments, rootIndex, root, outsideRadiusSq); err != nil {
		return nil, true, err
	}
	return &facetedrecord.PrismPatternSource{Blank: blank, BlankRadius: circle.Radius.Base(), Section: first,
		RootArc: root, RootIndex: rootIndex,
		RootGap: gap, Sections: append([]*topo.Body(nil), operands[1:]...)}, true, nil
}

// RootInside reports whether both recorded endpoints of the source arc lie
// strictly inside the blank after the radial scale.
// The source arc's denoted radius comes from Start. Its pinned End can
// disagree slightly, so both recorded endpoints must lie inside the blank.
// This exact overlap is needed before the join can remove the root wall.
func RootInside(root sectionrecord.ArcSeg, blankRadiusSq, scale2 *big.Rat) bool {
	for _, endpoint := range []sectionrecord.Point2{root.Start, root.End} {
		endpointSq := new(big.Rat).Mul(RadialSq(endpoint), scale2)
		if endpointSq.Cmp(blankRadiusSq) >= 0 {
			return false
		}
	}
	return true
}

// WholeRange reports whether a segment's parameter range is the whole
// segment, walked either way.
func WholeRange(start, end float64) bool {
	return start == 0 && end == 1 || start == 1 && end == 0
}

// AxialRotation reports whether t is a valid, non-reflecting rotation about
// the sweep axis with no translation.
func AxialRotation(t r3.Transform) bool {
	if !t.IsValid() || t.IsReflection() || t.Translation() != (r3.Vec{}) {
		return false
	}
	b := t.Basis()
	return b.EZ == (r3.Vec{Z: 1}) && b.EX.Z == 0 && b.EY.Z == 0 &&
		b.EX.X == b.EY.Y && b.EX.Y == -b.EY.X
}

// ArcGap bounds the difference between a placed arc's exact
// radius and the blank's radius. The arc's denoted radius comes from Start;
// its pinned End can differ, so the caller must charge that endpoint too.
func ArcGap(arc sectionrecord.ArcSeg, radius float64, motion r3.Transform) (float64, bool) {
	scale2 := Scale2(motion)
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
	endGap := proofbound.ProductUpper(proofbound.RatSqrtUp(scale2), circularbounds.ArcRadialResidualUpper(arc))
	gap := proofbound.AbsSumUpper(proofbound.RatFloatUp(d0), endGap)
	return gap, !proofbound.IsNonFinite(gap)
}

// Scale2 returns the exact squared radial scale of a placement transform.
func Scale2(motion r3.Transform) *big.Rat {
	b := motion.Basis()
	c, s := proofarith.FloatRat(b.EX.X), proofarith.FloatRat(b.EX.Y)
	return new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
}

// OutsideRoot proves every source segment other than the root outside the
// circle of squared radius outsideRadiusSq.
func OutsideRoot(ctx context.Context, segments []sectionrecord.CurveSegment, rootIndex int,
	root sectionrecord.ArcSeg, outsideRadiusSq *big.Rat) error {
	for i, segment := range segments {
		if err := ctx.Err(); err != nil {
			return err
		}
		if i == rootIndex {
			continue
		}
		switch seg := segment.(type) {
		case sectionrecord.LineSeg:
			if i == (rootIndex+1)%len(segments) || (i+1)%len(segments) == rootIndex {
				if !rootConnector(seg, root, outsideRadiusSq) {
					return fmt.Errorf("%w: prism section connector %d has no outward radial proof",
						decaderr.ErrUnsupported, i)
				}
				continue
			}
			if !lineOutside(seg, outsideRadiusSq) {
				return fmt.Errorf("%w: prism section line %d is not proven outside the circular wall",
					decaderr.ErrUnsupported, i)
			}
		case sectionrecord.FitSplineSeg:
			spans, err := splinebezier.FitSplineBezierSpans(seg, freeform.NewFreeformWork())
			if err != nil {
				return err
			}
			for _, span := range spans {
				if !spanOutside(span, outsideRadiusSq) {
					return fmt.Errorf("%w: prism section spline %d is not proven outside the circular wall",
						decaderr.ErrUnsupported, i)
				}
			}
		case sectionrecord.ArcSeg:
			if seg.Center != (sectionrecord.Point2{}) || RadialSq(seg.Start).Cmp(outsideRadiusSq) <= 0 {
				return fmt.Errorf("%w: prism section arc %d is not proven outside the circular wall",
					decaderr.ErrUnsupported, i)
			}
		default:
			return fmt.Errorf("%w: prism section curve %d has no circular-wall proof", decaderr.ErrUnsupported, i)
		}
	}
	return nil
}

func rootConnector(line sectionrecord.LineSeg, root sectionrecord.ArcSeg, outsideRadiusSq *big.Rat) bool {
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
	return radial.Sign() > 0 && RadialSq(far).Cmp(outsideRadiusSq) > 0
}

// RadiusSq returns the exact square of a float radius.
func RadiusSq(radius float64) *big.Rat {
	r := proofarith.FloatRat(radius)
	return new(big.Rat).Mul(r, r)
}

// RadialSq returns the exact squared distance of p from the origin.
func RadialSq(p sectionrecord.Point2) *big.Rat {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	return new(big.Rat).Add(new(big.Rat).Mul(u, u), new(big.Rat).Mul(v, v))
}

func lineOutside(line sectionrecord.LineSeg, outsideRadiusSq *big.Rat) bool {
	a, b := RadialSq(line.Start), RadialSq(line.End)
	if a.Cmp(outsideRadiusSq) <= 0 || b.Cmp(outsideRadiusSq) <= 0 {
		return false
	}
	// The dot with one endpoint direction is a conservative linear support
	// certificate for every point on this line.
	return controlOutside([]freeform.RatPoint{
		{U: proofarith.FloatRat(line.Start.U), V: proofarith.FloatRat(line.Start.V)},
		{U: proofarith.FloatRat(line.End.U), V: proofarith.FloatRat(line.End.V)},
	}, outsideRadiusSq)
}

func spanOutside(span freeform.BezierSpan, outsideRadiusSq *big.Rat) bool {
	return controlOutside(span, outsideRadiusSq)
}

// A Bezier span lies in its controls' convex hull. One separating line that
// leaves all controls beyond a circle proves the entire span outside it.
func controlOutside(points []freeform.RatPoint, outsideRadiusSq *big.Rat) bool {
	if len(points) == 0 {
		return false
	}
	du := new(big.Rat).Add(points[0].U, points[len(points)-1].U)
	dv := new(big.Rat).Add(points[0].V, points[len(points)-1].V)
	norm2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	if norm2.Sign() == 0 {
		return false
	}
	threshold := new(big.Rat).Mul(outsideRadiusSq, norm2)
	for _, point := range points {
		dot := new(big.Rat).Add(new(big.Rat).Mul(du, point.U), new(big.Rat).Mul(dv, point.V))
		if dot.Sign() <= 0 || new(big.Rat).Mul(dot, dot).Cmp(threshold) <= 0 {
			return false
		}
	}
	return true
}
