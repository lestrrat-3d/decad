package sweepinput

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

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

// CompositeChainSweepPath is Table SC row SC7's path gate for a composite
// chain Sweep (agent-docs/sweep-design.md §15.8). Every arc must be an exact
// quarter or half turn about an axis parallel to a coordinate axis: those are
// the turns whose sine and cosine are exact, so the transported section frames
// can carry zero displacement. The gate reads the recorded exact axis and turn
// fraction, never a rounded angle, so a near-quarter arc cannot pass it. The
// caller still refuses any transported frame with nonzero displacement, every
// span reruns its own builder gates, and the separation audit decides contact.
func CompositeChainSweepPath(path *Path) error {
	if len(path.Records) < 2 {
		return fmt.Errorf(`%w: a composite chain Sweep path has %d spans, fewer than two`,
			decaderr.ErrUnsupported, len(path.Records))
	}
	for i, record := range path.Records {
		if record.Arc == nil {
			continue
		}
		if _, _, ok := cardinalAxis(record.Arc.Axis); !ok {
			return fmt.Errorf(`%w: composite chain Sweep span %d is an arc about a non-coordinate axis`,
				decaderr.ErrUnsupported, i)
		}
		if !quarterOrHalfTurn(record.ArcAngle) {
			return fmt.Errorf(`%w: composite chain Sweep span %d is not an exact quarter or half turn`,
				decaderr.ErrUnsupported, i)
		}
	}
	return nil
}
