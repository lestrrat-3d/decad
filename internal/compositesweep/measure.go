package compositesweep

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Span holds one built analytic span's bounded volume, centroid, and box.
type Span struct {
	Solid         bool
	Volume        proofbound.BoundedScalar
	Centroid      r3.Vec
	CentroidBound float64
	Min, Max      r3.Vec
	BoxBound      float64
}

// Measurements are the bounded readings of the assembled sweep.
type Measurements struct {
	Volume, Area proofbound.BoundedScalar
	Centroid     [3]proofbound.BoundedScalar
	Min, Max     [3]proofbound.BoundedScalar
}

// Aggregate combines spans and published faces. The omitted endpoint caps of
// a surface result enter the area sum and are then subtracted with their own
// bounds, preserving the same bounded arithmetic as the solid path.
func Aggregate(budget *proofbound.WorkBudget, spans []Span, faces []proofbound.BoundedScalar,
	surfaceResult bool, startCap, endCap proofbound.BoundedScalar) (Measurements, error) {
	if len(spans) == 0 {
		return Measurements{}, fmt.Errorf(`%w: a composite sweep requires at least one built span`, decaderr.ErrDegenerate)
	}
	volume := proofbound.ExactScalar(0)
	var moments [3]proofbound.BoundedScalar
	var minCoord, maxCoord [3]proofbound.BoundedScalar
	for i, span := range spans {
		if err := budget.Step(); err != nil {
			return Measurements{}, err
		}
		if !span.Solid {
			return Measurements{}, fmt.Errorf(`%w: composite sweep span %d is not a solid`, decaderr.ErrDegenerate, i)
		}
		volume = proofbound.BoundedAdd(volume, span.Volume)
		centroid := [3]float64{span.Centroid.X, span.Centroid.Y, span.Centroid.Z}
		lo := [3]float64{span.Min.X, span.Min.Y, span.Min.Z}
		hi := [3]float64{span.Max.X, span.Max.Y, span.Max.Z}
		for axis := range 3 {
			moments[axis] = proofbound.BoundedAdd(moments[axis], proofbound.BoundedMul(
				span.Volume, proofbound.MeasuredScalar(centroid[axis], span.CentroidBound)))
			minValue := proofbound.MeasuredScalar(lo[axis], span.BoxBound)
			maxValue := proofbound.MeasuredScalar(hi[axis], span.BoxBound)
			if i == 0 {
				minCoord[axis], maxCoord[axis] = minValue, maxValue
				continue
			}
			minCoord[axis] = proofbound.BoundedMin(minCoord[axis], minValue)
			maxCoord[axis] = proofbound.BoundedNeg(proofbound.BoundedMin(
				proofbound.BoundedNeg(maxCoord[axis]), proofbound.BoundedNeg(maxValue)))
		}
	}
	if proofbound.AdmitAbove(volume, 0) != proofbound.SurvAdmit {
		return Measurements{}, fmt.Errorf(`%w: a composite sweep's volume is not proven positive`, decaderr.ErrUnsupported)
	}
	result := Measurements{Volume: volume, Min: minCoord, Max: maxCoord}
	for axis := range 3 {
		result.Centroid[axis] = proofbound.BoundedDiv(moments[axis], volume)
	}
	area := proofbound.ExactScalar(0)
	for _, face := range faces {
		if err := budget.Step(); err != nil {
			return Measurements{}, err
		}
		area = proofbound.BoundedAdd(area, face)
	}
	if surfaceResult {
		area = proofbound.BoundedAdd(area, startCap)
		area = proofbound.BoundedAdd(area, endCap)
		area = proofbound.BoundedSub(area, startCap)
		area = proofbound.BoundedSub(area, endCap)
	}
	result.Area = area
	return result, budget.Err()
}
