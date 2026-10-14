package twistsweep

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
)

// TwistCardinalPathOf admits the cardinal two-span path (a positive-Z line then a
// quarter arc) a composite twist sweeps and returns its exact record.
func TwistCardinalPathOf(path *sweepinput.Path, plane planeRecord, poly TwistPolygon) (TwistCardinalPath, error) {
	if path == nil || len(path.Records) != 2 ||
		plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) ||
		plane.V != r3.NewVec(0, 1, 0) || path.Start != (r3.Vec{}) {
		return TwistCardinalPath{}, fmt.Errorf(`%w: composite twist needs an origin XY sketch and two path spans`, decaderr.ErrUnsupported)
	}
	line, arc := path.Records[0], path.Records[1]
	if line.Arc != nil || arc.Arc == nil || line.End.X != 0 || line.End.Y != 0 || line.End.Z <= 0 ||
		arc.Start != line.End {
		return TwistCardinalPath{}, fmt.Errorf(`%w: composite twist needs a positive Z line followed by an arc`, decaderr.ErrUnsupported)
	}
	r := arc.End.X
	h := line.End.Z
	carrier := arc.Arc
	if r <= 0 || arc.End.Y != 0 || arc.End.Z != h+r ||
		carrier.Center[0].Cmp(TwistRat(r)) != 0 || carrier.Center[1].Sign() != 0 ||
		carrier.Center[2].Cmp(TwistRat(h)) != 0 ||
		carrier.RadiusStart[0].Cmp(new(big.Rat).Neg(TwistRat(r))) != 0 ||
		carrier.RadiusStart[1].Sign() != 0 || carrier.RadiusStart[2].Sign() != 0 ||
		carrier.RadiusEnd[0].Sign() != 0 || carrier.RadiusEnd[1].Sign() != 0 ||
		carrier.RadiusEnd[2].Cmp(TwistRat(r)) != 0 ||
		carrier.Axis[0].Sign() != 0 || carrier.Axis[1].Sign() <= 0 || carrier.Axis[2].Sign() != 0 ||
		arc.ArcAngle.Turn == nil || arc.ArcAngle.Turn.Cmp(big.NewRat(1, 4)) != 0 {
		return TwistCardinalPath{}, fmt.Errorf(`%w: composite twist needs a positive cardinal quarter arc in XZ`, decaderr.ErrUnsupported)
	}
	// The profile stays on the inside of the arc's polar radius. This also
	// separates the true arc from the preceding line except at their join.
	if !proofbound.FiniteVec(arc.End) ||
		TwistRat(r).Cmp(new(big.Rat).Mul(big.NewRat(8, 1), TwistRat(poly.MaxRadiusL1))) <= 0 {
		return TwistCardinalPath{}, fmt.Errorf(`%w: the twisted profile approaches the arc axis`, decaderr.ErrUnsupported)
	}
	return TwistCardinalPath{Height: h, Radius: r}, nil
}
