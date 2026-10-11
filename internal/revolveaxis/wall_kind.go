package revolveaxis

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// WallKind names the surface an axis-coordinate walk sweeps.
type WallKind int

const (
	// WallAxis sweeps no face because its line lies on the axis.
	WallAxis WallKind = iota
	WallCylinder
	WallPlane
	WallCone
	WallSphere
	WallTorus
	WallFreeform
)

// Classify names the surface of revolution one walk sweeps.
func Classify(w survey2d.SegmentWalk, snapTol float64) WallKind {
	if w.Kind == survey2d.WalkFreeform {
		return WallFreeform
	}
	if w.IsCircular() {
		if math.Abs(w.CV) <= snapTol {
			return WallSphere
		}
		return WallTorus
	}
	if w.StartV == 0 && w.EndV == 0 {
		return WallAxis
	}
	dz, dr := w.EndU-w.StartU, w.EndV-w.StartV
	l := math.Hypot(dz, dr)
	if math.Abs(dr) <= 1e-9*l {
		return WallCylinder
	}
	if math.Abs(dz) <= 1e-9*l {
		return WallPlane
	}
	return WallCone
}
