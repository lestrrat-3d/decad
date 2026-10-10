package sweeparc

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// Geometry is the resolved axis and bounded angle of one recorded sweep arc.
type Geometry struct {
	Line  revolveaxis.Line2
	Phi   float64
	Angle revolveangle.Angle
}

// Derive checks the arc carrier against the profile plane before its angle
// reaches the evaluator. The path's first tangent must follow the plane's
// positive normal, and the circle's axis must lie in that plane.
func Derive(start r3.Vec, arc *Record, phi float64, angle revolveangle.Angle,
	plane sectionrecord.PlaneRecord) (Geometry, error) {
	normal := proofarith.DvCross(proofarith.DyVec(plane.U), proofarith.DyVec(plane.V))
	relStart := proofarith.DvSub(proofarith.DyVec(start), proofarith.DyVec(plane.Origin))
	if !proofarith.DvDot(relStart, normal).IsZero() {
		return Geometry{}, fmt.Errorf(`%w: the sweep path must start in the profile plane`, decaderr.ErrDegenerate)
	}
	if arc == nil {
		return Geometry{}, fmt.Errorf(`%w: a sweep arc path record holds no circular carrier`, decaderr.ErrDegenerate)
	}

	center := arc.Center
	r0, rm, r1 := arc.RadiusStart, arc.RadiusMiddle, arc.RadiusEnd
	axis := arc.Axis
	tangent := Cross(axis, r0)
	normalRat := FromDyadic(normal)
	if !IsZero(Cross(tangent, normalRat)) || Dot(tangent, normalRat).Sign() <= 0 {
		return Geometry{}, fmt.Errorf(`%w: the sweep path's initial tangent must follow the profile plane's positive normal`, decaderr.ErrDegenerate)
	}
	if Dot(Sub(center, VecOf(plane.Origin)), normalRat).Sign() != 0 {
		return Geometry{}, fmt.Errorf(`%w: the sweep arc's derived axis does not lie in the profile plane`, decaderr.ErrDegenerate)
	}

	// The carrier was solved from all three points. This equality catches an
	// arithmetic regression before a derived angle reaches the evaluator.
	radiusSquared := Dot(r0, r0)
	if Dot(rm, rm).Cmp(radiusSquared) != 0 || Dot(r1, r1).Cmp(radiusSquared) != 0 {
		return Geometry{}, fmt.Errorf(`%w: the sweep arc points do not share one exact carrier`, decaderr.ErrUnsupported)
	}
	line, err := AxisLine(center, axis, plane.Origin, plane.U, plane.V)
	if err != nil {
		return Geometry{}, err
	}
	return Geometry{Line: line, Phi: phi, Angle: angle}, nil
}
