// Package embossplan decides the sweep of an emboss tool from the face it
// lies on (agent-docs/emboss-design.md). It holds plain frame arithmetic and
// touches no body, document or sketch.
package embossplan

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// PlaneTol is the largest distance, in millimetres, between the sketch plane
// origin and the face plane that Plan accepts, and ParallelTol the largest
// length of the cross product of the two unit normals. A sketch farther off
// names a different plane, so it is refused rather than moved.
const (
	PlaneTol    = 1e-6
	ParallelTol = 1e-9
)

// Margin is how far, in millimetres, every tool reaches past the face on the
// side it leaves open, so no tool face lies in the face plane (a tangent
// contact the boolean refuses).
const Margin = 1

// Sweep is the two sides of an emboss tool, in millimetres, measured from the
// sketch plane. Along is the distance swept along the sketch normal and
// Against the distance swept against it.
type Sweep struct {
	Along, Against float64
}

// Plan returns the sweep of a tool for a relief of height on the face whose
// plane contains sketch. outward is the face's outward unit normal. A raised
// tool reaches height outward and Margin inward. An engraved tool reaches
// height inward and Margin outward.
func Plan(sketch r3.Frame, face r3.Frame, outward r3.Vec, height float64, raised bool) (Sweep, error) {
	return PlanCurved(sketch, face, outward, height, raised, 0)
}

// PlanCurved is Plan for a face that curves away from its tangent plane: extra
// millimetres more are added to the side the tool leaves open, so the tool still
// overlaps the body at the edge of its section, where the wall has moved from the
// plane by up to extra less Margin.
func PlanCurved(sketch r3.Frame, face r3.Frame, outward r3.Vec, height float64, raised bool, extra float64) (Sweep, error) {
	if !(extra >= 0) || math.IsInf(extra, 0) {
		return Sweep{}, fmt.Errorf(`%w: the extra reach must be finite and not negative`, decaderr.ErrDegenerate)
	}
	if !(height > 0) || math.IsInf(height, 0) {
		return Sweep{}, fmt.Errorf(`%w: the relief height must be positive and finite`, decaderr.ErrDegenerate)
	}
	n := sketch.N()
	if off := math.Abs(sketch.Origin().Sub(face.Origin()).Dot(outward)); off > PlaneTol {
		return Sweep{}, fmt.Errorf(`%w: the sketch plane lies %g mm off the face plane`, decaderr.ErrDegenerate, off)
	}
	if cross := n.Cross(outward); math.Sqrt(cross.Dot(cross)) > ParallelTol {
		return Sweep{}, fmt.Errorf(`%w: the sketch plane is not parallel to the face plane`, decaderr.ErrDegenerate)
	}
	out, in := height, float64(Margin)+extra
	if !raised {
		out, in = float64(Margin)+extra, height
	}
	if n.Dot(outward) > 0 {
		return Sweep{Along: out, Against: in}, nil
	}
	return Sweep{Along: in, Against: out}, nil
}

// RemovesNothing reports whether a volume reading of after, value ± bound, is
// certifiably no smaller than a reading of before. A true answer is certain; a
// false answer proves nothing.
func RemovesNothing(before, beforeBound, after, afterBound float64) bool {
	return after-afterBound >= before+beforeBound
}
