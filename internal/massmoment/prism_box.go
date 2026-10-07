package massmoment

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CardinalBoxMoments computes the mass and world-axis centroidal inertia of an exact rectangular prism.
func CardinalBoxMoments(ctx context.Context, segments []sectionrecord.CurveSegment, z0, z1 float64,
	frame r3.Frame, xform r3.Transform, density units.Value) (*big.Rat, [3]*big.Rat, error) {
	firstLine, ok := segments[0].(sectionrecord.LineSeg)
	if !ok {
		return nil, [3]*big.Rat{}, fmt.Errorf("%w: box section is not a line loop", decaderr.ErrUnsupported)
	}
	first := firstLine.Start
	minU, maxU, minV, maxV := first.U, first.U, first.V, first.V
	for _, segment := range segments {
		if err := ctx.Err(); err != nil {
			return nil, [3]*big.Rat{}, err
		}
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok {
			return nil, [3]*big.Rat{}, fmt.Errorf("%w: box section is not a line loop", decaderr.ErrUnsupported)
		}
		point := line.Start
		if line.TStart == 1 {
			point = line.End
		}
		minU, maxU = math.Min(minU, point.U), math.Max(maxU, point.U)
		minV, maxV = math.Min(minV, point.V), math.Max(maxV, point.V)
	}
	u := proofarith.DySubScalar(proofarith.MustDyOf(maxU), proofarith.MustDyOf(minU))
	v := proofarith.DySubScalar(proofarith.MustDyOf(maxV), proofarith.MustDyOf(minV))
	z := proofarith.DySubScalar(proofarith.MustDyOf(z1), proofarith.MustDyOf(z0))
	if u.Sign() <= 0 || v.Sign() <= 0 || z.Sign() <= 0 {
		return nil, [3]*big.Rat{}, fmt.Errorf("%w: source box has no positive volume", decaderr.ErrUnsupported)
	}
	densityBase := proofarith.DyMul(proofarith.MustDyOf(density.Mag()), proofarith.MustDyOf(density.Unit().Factor()))
	mass := proofarith.DyMul(densityBase, proofarith.DyMul(u, proofarith.DyMul(v, z)))
	if mass.Sign() <= 0 {
		return nil, [3]*big.Rat{}, fmt.Errorf("%w: mass is not positive", decaderr.ErrUnsupported)
	}

	var dimensions [3]proofarith.Dyadic
	for i, axis := range []r3.Vec{
		xform.ApplyDir(frame.U()),
		xform.ApplyDir(frame.V()),
		xform.ApplyDir(frame.N()),
	} {
		dimension := []proofarith.Dyadic{u, v, z}[i]
		switch {
		case math.Abs(axis.X) == 1 && axis.Y == 0 && axis.Z == 0:
			dimensions[0] = dimension
		case axis.X == 0 && math.Abs(axis.Y) == 1 && axis.Z == 0:
			dimensions[1] = dimension
		case axis.X == 0 && axis.Y == 0 && math.Abs(axis.Z) == 1:
			dimensions[2] = dimension
		default:
			return nil, [3]*big.Rat{}, fmt.Errorf("%w: box orientation has no exact cardinal axes", decaderr.ErrUnsupported)
		}
	}
	squared := [3]proofarith.Dyadic{}
	for i, length := range dimensions {
		squared[i] = proofarith.DyMul(length, length)
	}
	inertia := func(a, c proofarith.Dyadic) *big.Rat {
		numerator := proofarith.DyMul(mass, proofarith.DyAdd(a, c))
		return new(big.Rat).Quo(numerator.Rat(), big.NewRat(12, 1))
	}
	readings := [3]*big.Rat{
		inertia(squared[1], squared[2]),
		inertia(squared[0], squared[2]),
		inertia(squared[0], squared[1]),
	}
	return mass.Rat(), readings, nil
}
