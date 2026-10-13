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
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/r3"
)

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
