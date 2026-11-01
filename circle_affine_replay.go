package decad

import (
	"context"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type circleExactAffine [3][4]*big.Rat

func circleReplayAffine(ctx context.Context, x r3.Transform, input circleExactAffine, b *proofbound.ReplayBudget) (circleExactAffine, error) {
	var out circleExactAffine
	basis := x.Basis()
	cols := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	tr := x.Translation()
	tv := [3]float64{tr.X, tr.Y, tr.Z}
	var left [3][3]*big.Rat
	for j, v := range cols {
		xyz := [3]float64{v.X, v.Y, v.Z}
		for i := range 3 {
			var err error
			left[i][j], err = proof.ReplayFloatRat(ctx, xyz[i], b)
			if err != nil {
				return out, err
			}
		}
	}
	for i := range 3 {
		translation, err := proof.ReplayFloatRat(ctx, tv[i], b)
		if err != nil {
			return out, err
		}
		for j := range 4 {
			sum, err := proof.ReplayFloatRat(ctx, 0, b)
			if err != nil {
				return out, err
			}
			for k := range 3 {
				term, err := proof.ReplayRatArithmetic(ctx, left[i][k], input[k][j], '*', b)
				if err != nil {
					return out, err
				}
				sum, err = proof.ReplayRatArithmetic(ctx, sum, term, '+', b)
				if err != nil {
					return out, err
				}
			}
			if j == 3 {
				sum, err = proof.ReplayRatArithmetic(ctx, sum, translation, '+', b)
				if err != nil {
					return out, err
				}
			}
			out[i][j] = sum
		}
	}
	return out, nil
}

// exactAffineAllowance compares the exact complete source and held affine maps.
// The sum of absolute coefficient differences bounds the operator norm; the
// corresponding translation sum plus radius times that norm bounds every
// source point's movement along the linear interpolation.
func (s *circleSourceSnapshot) exactAffineAllowance(ctx context.Context, b *proofbound.ReplayBudget) (float64, float64, error) {
	if err := s.circleReplayCheck(ctx, b); err != nil {
		return 0, 0, err
	}
	p, ok := s.root.n.Payload.(prismrecord.Payload)
	if !ok || len(p.Profile.Outer.Segments) != 1 {
		return 0, 0, errCirclePlacementReplay
	}
	seg, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok {
		return 0, 0, errCirclePlacementReplay
	}
	radius, err := seg.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || proofbound.IsNonFinite(radius) {
		return 0, 0, errCirclePlacementReplay
	}
	v := [4]r3.Vec{p.Frame.U(), p.Frame.V(), p.Frame.N(), p.Frame.Origin()}
	var frame circleExactAffine
	for j, vec := range v {
		xyz := [3]float64{vec.X, vec.Y, vec.Z}
		for i := range 3 {
			frame[i][j], err = proof.ReplayFloatRat(ctx, xyz[i], b)
			if err != nil {
				return 0, 0, err
			}
		}
	}
	held, err := circleReplayAffine(ctx, p.Xform, frame, b)
	if err != nil {
		return 0, 0, err
	}
	exact := frame
	for _, step := range slices.Backward(s.steps) {
		exact, err = circleReplayAffine(ctx, step.motion, exact, b)
		if err != nil {
			return 0, 0, err
		}
	}
	linear, origin := 0.0, 0.0
	for i := range 3 {
		for j := range 4 {
			d, e := proof.ReplayRationalRatError(ctx, exact[i][j], held[i][j], b)
			if e != nil {
				return 0, 0, e
			}
			if j == 3 {
				origin = proofbound.AbsSumUpper(origin, d)
			} else {
				linear = proofbound.AbsSumUpper(linear, d)
			}
		}
	}
	extent := math.Max(proofbound.AbsSumUpper(seg.Center.U, radius), math.Max(proofbound.AbsSumUpper(seg.Center.V, radius), math.Max(math.Abs(p.Z0), math.Abs(p.Z1))))
	displacement := proofbound.AbsSumUpper(origin, proofbound.ProductUpper(linear, extent))
	if proofbound.IsNonFinite(displacement) || proofbound.IsNonFinite(linear) {
		return 0, 0, errCirclePlacementReplay
	}
	return displacement, linear, nil
}
