package loftmesh

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// BodyReadings are the published scalar readings of a lofted body as plain
// numbers: each value with the bound it carries, in base units.
type BodyReadings struct {
	Volume, VolumeBound  float64
	Area                 float64
	Centroid             r3.Vec
	CentroidBound        float64
	BoundsMin, BoundsMax r3.Vec
	BoundsBound          float64
}

// ValidateBodyReadings is loftrecord.Build's own finiteness gate (design O2).
// Volume, Centroid and Bounds must be fully finite, exactly as every other
// analytic payload requires — but Area's Bound is deliberately NOT checked:
// §8 requires a saturated Area bound to publish +Inf as a proof term (a wall
// set whose areas approach float64's own ceiling), and checking it here
// would refuse the very body that fixture constructs. Only Area's Value is
// checked for finiteness.
func ValidateBodyReadings(r BodyReadings) error {
	if !allFinite(r.Volume, r.VolumeBound) {
		return fmt.Errorf(`%w: the loft's volume measurement is not finite`, decaderr.ErrNotFinite)
	}
	if !allFinite(r.Area) {
		return fmt.Errorf(`%w: the loft's area value is not finite`, decaderr.ErrNotFinite)
	}
	if !allFinite(r.Centroid.X, r.Centroid.Y, r.Centroid.Z, r.CentroidBound) {
		return fmt.Errorf(`%w: the loft's centroid measurement is not finite`, decaderr.ErrNotFinite)
	}
	if !allFinite(
		r.BoundsMin.X, r.BoundsMin.Y, r.BoundsMin.Z,
		r.BoundsMax.X, r.BoundsMax.Y, r.BoundsMax.Z,
		r.BoundsBound,
	) {
		return fmt.Errorf(`%w: the loft's bounds measurement is not finite`, decaderr.ErrNotFinite)
	}
	return nil
}

func allFinite(values ...float64) bool {
	return !slices.ContainsFunc(values, proofbound.IsNonFinite)
}
