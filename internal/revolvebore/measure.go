package revolvebore

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ApplyMeasurements subtracts a fully contained, flush tool from a fresh
// target. AddBoundary must have already attached the new faces.
func ApplyMeasurements(body, tool *topo.Body, mer Meridian) error {
	faces := resultFacesByRole(body)
	low := faces[fmt.Sprintf("side(0,%d)", mer.LowCapSegment)]
	high := faces[fmt.Sprintf("side(0,%d)", mer.HighCapSegment)]
	toolFaces := topobuild.FacesByRole(tool)
	toolCap := toolFaces["capStart"]
	if low == nil || high == nil || toolCap == nil {
		return fmt.Errorf("%w: bore measurements need both end faces", decaderr.ErrUnsupported)
	}
	holeArea := proofbound.MeasuredScalar(toolCap.Area, toolCap.AreaBound)
	for _, end := range []*topo.Face{low, high} {
		area := proofbound.BoundedSub(proofbound.MeasuredScalar(end.Area, end.AreaBound), holeArea)
		if proofbound.AdmitAbove(area, 0) != proofbound.SurvAdmit {
			return fmt.Errorf("%w: bore end area is not proven positive", decaderr.ErrUnsupported)
		}
		end.Area, end.AreaBound = area.Value, area.Bound
	}
	area := proofbound.BoundedScalar{}
	for _, face := range topobuild.Faces(body) {
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(face.Area, face.AreaBound))
	}
	hostVolume := proofbound.MeasuredScalar(body.Volume.Value.Base(), body.Volume.Bound.Base())
	toolVolume := proofbound.MeasuredScalar(tool.Volume.Value.Base(), tool.Volume.Bound.Base())
	volume := proofbound.BoundedSub(hostVolume, toolVolume)
	if proofbound.AdmitAbove(volume, 0) != proofbound.SurvAdmit {
		return fmt.Errorf("%w: bore volume is not proven positive", decaderr.ErrUnsupported)
	}
	var centroid [3]proofbound.BoundedScalar
	hostXYZ := [3]float64{body.Centroid.Value.X, body.Centroid.Value.Y, body.Centroid.Value.Z}
	toolXYZ := [3]float64{tool.Centroid.Value.X, tool.Centroid.Value.Y, tool.Centroid.Value.Z}
	for i := range centroid {
		hostMoment := proofbound.BoundedMul(hostVolume,
			proofbound.MeasuredScalar(hostXYZ[i], body.Centroid.Bound.Base()))
		toolMoment := proofbound.BoundedMul(toolVolume,
			proofbound.MeasuredScalar(toolXYZ[i], tool.Centroid.Bound.Base()))
		centroid[i] = proofbound.BoundedDiv(proofbound.BoundedSub(hostMoment, toolMoment), volume)
	}
	bound := proofbound.AbsSumUpper(centroid[0].Bound, centroid[1].Bound, centroid[2].Bound)
	if proofbound.IsNonFinite(bound) {
		return fmt.Errorf("%w: bore centroid has no finite bound", decaderr.ErrUnsupported)
	}
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(volume.Value),
		Bound: units.CubicMillimeters(volume.Bound), Exactness: measurement.ExactnessOf(volume.Bound)}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(area.Value),
		Bound: units.SquareMillimeters(area.Bound), Exactness: measurement.ExactnessOf(area.Bound)}
	body.Centroid = measurement.VecMeasurement{Value: r3.Vec{X: centroid[0].Value, Y: centroid[1].Value, Z: centroid[2].Value},
		Bound: units.Millimeters(bound), Exactness: measurement.ExactnessOf(bound)}
	return topobuild.ValidateAnalyticMeasurements(body)
}
