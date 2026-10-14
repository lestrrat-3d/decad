package sweepinput

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

type chainRecord = sectionrecord.ChainRecord

type lineSeg = sectionrecord.LineSeg

type planeRecord = sectionrecord.PlaneRecord

// ValidateSweepPathStart is Table S row S5: the path starts in the profile
// plane and its first tangent follows the plane's positive normal. It reads
// no internal join.
func ValidateSweepPathStart(path *Path, plane planeRecord) error {
	return ValidateStart(path.Start, plane, path.Records[0].TangentIn)
}

// ValidateSweepPathGeometry checks the recorded path starts in the profile plane
// and that every join follows the tangent rules.
func ValidateSweepPathGeometry(path *Path, plane planeRecord) error {
	if err := ValidateSweepPathStart(path, plane); err != nil {
		return err
	}
	tangents := make([]Tangents, len(path.Records))
	for i, record := range path.Records {
		tangents[i] = Tangents{In: record.TangentIn, Out: record.TangentOut}
	}
	return ValidateJoins(tangents)
}

// ValidateStraightSweepPath admits a one-span straight path and returns its
// height and its bound along the frame normal.
func ValidateStraightSweepPath(path *Path, frame r3.Frame) (float64, float64, error) {
	start := path.Start
	end := path.End
	if SamePathPoint(start, end) {
		return 0, 0, fmt.Errorf(`%w: closed sweep paths are not implemented`, decaderr.ErrUnsupported)
	}
	segments := path.Segments
	if len(segments) != 1 {
		return 0, 0, fmt.Errorf(`%w: this evaluator sweeps one straight path span only`, decaderr.ErrUnsupported)
	}
	ok := !segments[0].Arc
	if !ok {
		return 0, 0, fmt.Errorf(`%w: the sweep path span is not straight`, decaderr.ErrUnsupported)
	}
	return StraightSpan(start, end, frame.N())
}

// SamePathPoint reports whether a and b are the same point, component by
// component.
func SamePathPoint(a, b r3.Vec) bool {
	return a.X == b.X && a.Y == b.Y && a.Z == b.Z
}

// CardinalChainSweepClass admits the source-backed class whose line and arc
// spans lie on opposite sides of their common XY section. Every equality is
// checked over the exact binary-rational values of the public inputs; rounded
// arithmetic cannot turn a near-half-circle into this exact class.
func CardinalChainSweepClass(chain chainRecord, plane planeRecord, path *Path) error {
	if plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) ||
		plane.V != r3.NewVec(0, 1, 0) || len(chain.Segments) != 1 ||
		len(path.Records) != 2 || path.Start != (r3.Vec{}) {
		return fmt.Errorf(`%w: composite chain Sweep requires one XY line and a cardinal two-span path`, decaderr.ErrUnsupported)
	}
	line, ok := chain.Segments[0].(lineSeg)
	if !ok || line.Start.U != 0 || line.Start.V != 0 || line.End.V != 0 ||
		line.TStart != 0 || line.TEnd != 1 {
		return fmt.Errorf(`%w: composite chain Sweep requires one forward radial source line`, decaderr.ErrUnsupported)
	}
	width := line.End.U
	straight := path.Records[0]
	arc := path.Records[1]
	arcSegment, ok := path.Segments[1], path.Segments[1].Arc
	if !ok {
		return fmt.Errorf(`%w: composite chain Sweep requires an ArcThrough second span`, decaderr.ErrUnsupported)
	}
	radius := arcSegment.Through.X
	height := straight.End.Z
	if width <= 0 || radius <= width || height <= 0 || straight.Arc != nil || arc.Arc == nil ||
		straight.End.X != 0 || straight.End.Y != 0 ||
		arc.Start != straight.End || arcSegment.Through.Y != 0 || arc.End.Y != 0 || arc.End.Z != height ||
		!dyadicSumEquals(height, radius, arcSegment.Through.Z) ||
		!dyadicSumEquals(radius, radius, arc.End.X) ||
		!dyadicSumEquals(radius-width, width, radius) ||
		!dyadicSumEquals(arc.End.X-width, width, arc.End.X) {
		return fmt.Errorf(`%w: composite chain Sweep requires a positive-Z line, tangent XZ half-circle, and radial source inside its radius`, decaderr.ErrUnsupported)
	}
	return nil
}

func dyadicSumEquals(a, b, sum float64) bool {
	return new(big.Rat).Add(new(big.Rat).SetFloat64(a), new(big.Rat).SetFloat64(b)).Cmp(
		new(big.Rat).SetFloat64(sum)) == 0
}
