package compositesweep

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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

// AggregateBody publishes the union readings after the
// caller has proved that span interiors are disjoint and has assembled their
// shared sections into body. Volume and first moments add over the spans,
// each of which stays a solid regardless of surfaceResult (Assemble
// keeps every span's own two cap roles for the pairing and sewing that
// already ran); bounds are the componentwise union of the span boxes.
//
// Area is a plain sum over body's own published faces, which already excludes
// every internal cap and, for a surface result, the two outer caps too. For a
// surface result, collapsing to that sum alone would hold the right VALUE but
// an uncomposed bound — §4.3 requires a sheet's area to be the full
// (cap-inclusive) sum minus the two omitted caps, each already-computed
// quantity contributing its own bound. So the two omitted caps are summed
// into the running total and then subtracted back out, exactly as
// internal/prismrecord/build.go and internal/revolveplan/build.go compose theirs, rather than being left out
// of the sum from the start.
func AggregateBody(ctx context.Context, body *topo.Body, parts []SpanPart, surfaceResult bool) error {
	if body == nil || len(parts) == 0 {
		return fmt.Errorf(`%w: a composite sweep requires at least one built span`, decaderr.ErrDegenerate)
	}
	budget := proofbound.NewWorkBudget(ctx)
	spans := make([]Span, len(parts))
	for i, part := range parts {
		span := part.Body
		if span == nil {
			continue
		}
		spans[i] = Span{
			Solid: span.Solid, Volume: measurementScalar(span.Volume),
			Centroid: span.Centroid.Value, CentroidBound: span.Centroid.Bound.Base(),
			Min: span.Bounds.Min, Max: span.Bounds.Max, BoxBound: span.Bounds.Bound.Base(),
		}
	}
	faces := topobuild.Faces(body)
	areas := make([]proofbound.BoundedScalar, len(faces))
	for i, face := range faces {
		areas[i] = proofbound.MeasuredScalar(face.Area, face.AreaBound)
	}
	var startCap, endCap proofbound.BoundedScalar
	if surfaceResult {
		startCap = proofbound.MeasuredScalar(parts[0].StartCap.Area, parts[0].StartCap.AreaBound)
		endCap = proofbound.MeasuredScalar(parts[len(parts)-1].EndCap.Area, parts[len(parts)-1].EndCap.AreaBound)
	}
	readings, err := Aggregate(budget, spans, areas, surfaceResult, startCap, endCap)
	if err != nil {
		return err
	}
	centroidBound := proofbound.Radius3D(max(readings.Centroid[0].Bound, readings.Centroid[1].Bound, readings.Centroid[2].Bound))
	body.Volume = ScalarMeasurement(readings.Volume, units.CubicMillimeter)
	body.Area = ScalarMeasurement(readings.Area, units.SquareMillimeter)
	body.Centroid = measurement.VecMeasurement{
		Value:     r3.NewVec(readings.Centroid[0].Value, readings.Centroid[1].Value, readings.Centroid[2].Value),
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	body.Bounds = boundsBox(readings.Min[0], readings.Min[1], readings.Min[2],
		readings.Max[0], readings.Max[1], readings.Max[2])
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return fmt.Errorf(`%w: a composite sweep computed a non-finite measurement`, decaderr.ErrUnsupported)
	}
	return budget.Err()
}

func measurementScalar(m measurement.Measurement) proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(m.Value.Base(), m.Bound.Base())
}

// ScalarMeasurement publishes a bounded scalar as a measurement in unit.
func ScalarMeasurement(value proofbound.BoundedScalar, unit units.Unit) measurement.Measurement {
	return measurement.Measurement{
		Value:     units.New(value.Value, unit),
		Exactness: measurement.ExactnessOf(value.Bound),
		Bound:     units.New(value.Bound, unit),
	}
}

func boundsBox(minX, minY, minZ, maxX, maxY, maxZ proofbound.BoundedScalar) measurement.Box {
	bound := max(minX.Bound, minY.Bound, minZ.Bound, maxX.Bound, maxY.Bound, maxZ.Bound)
	if bound != 0 {
		bound = math.Nextafter(bound, math.Inf(1))
	}
	return measurement.Box{
		Min:       r3.NewVec(minX.Value, minY.Value, minZ.Value),
		Max:       r3.NewVec(maxX.Value, maxY.Value, maxZ.Value),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}
}
