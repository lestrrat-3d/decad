package decad

import "github.com/lestrrat-3d/decad/internal/record"

// Structural curve records are defined in internal/record. These aliases
// expose their definitions through decad.

// PlaneRecord is a recorded sketch plane. See docs/sketch-seam-design.md §2.
type PlaneRecord = record.PlaneRecord

// Point2 is a plane-local coordinate in millimetres.
type Point2 = record.Point2

// ProfileRecord is a structural plane-local region: one outer loop and its
// holes. The evaluator defines its measurement methods in the root package.
type ProfileRecord struct {
	Outer LoopRecord   `json:"outer"`
	Holes []LoopRecord `json:"holes,omitempty"`
}

// LoopRecord is one closed directed boundary walk.
type LoopRecord = record.LoopRecord

// ChainRecord is one open directed boundary walk.
type ChainRecord = record.ChainRecord

// CurveSegment is a recorded curve with a sealed variant set.
type CurveSegment = record.CurveSegment

// LineSeg records a line and its parameter range.
type LineSeg = record.LineSeg

// CircleSeg records a circle and its parameter range.
type CircleSeg = record.CircleSeg

// ArcSeg records an arc and its parameter range.
type ArcSeg = record.ArcSeg

// EllipseSeg records an ellipse and its parameter range.
type EllipseSeg = record.EllipseSeg

// EllipticalArcSeg records an elliptical arc and its parameter range.
type EllipticalArcSeg = record.EllipticalArcSeg

// SplineSeg records a spline and its parameter range.
type SplineSeg = record.SplineSeg

// NURBSSeg records a NURBS curve and its parameter range.
type NURBSSeg = record.NURBSSeg

// ClosedSplineSeg records a closed spline and its parameter range.
type ClosedSplineSeg = record.ClosedSplineSeg

// FitSplineSeg records a fit spline and its parameter range.
type FitSplineSeg = record.FitSplineSeg

// ConicSeg records a conic and its parameter range.
type ConicSeg = record.ConicSeg

func cloneLoopRecord(loop LoopRecord) LoopRecord  { return record.CloneLoopRecord(loop) }
func finiteSegmentValue(value float64) bool       { return record.FiniteSegmentValue(value) }
func validateNURBSSegment(segment NURBSSeg) error { return record.ValidateNURBSSegment(segment) }
func validateNURBSSegmentSizes(segment NURBSSeg) error {
	return record.ValidateNURBSSegmentSizes(segment)
}
func validateNURBSSegmentContent(segment NURBSSeg) error {
	return record.ValidateNURBSSegmentContent(segment)
}
func validateSegment(segment CurveSegment) error { return record.ValidateSegment(segment) }
func normalizeSegment(segment CurveSegment) (CurveSegment, error) {
	return record.NormalizeSegment(segment)
}

var errNilSegment = record.ErrNilSegment
