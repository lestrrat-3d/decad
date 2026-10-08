package decad

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Structural curve records are defined in internal/sectionrecord. These aliases
// expose their definitions through decad.

// PlaneRecord is a recorded sketch plane. See docs/sketch-seam-design.md §2.
type PlaneRecord = sectionrecord.PlaneRecord

// Point2 is a plane-local coordinate in millimetres.
type Point2 = sectionrecord.Point2

// ProfileRecord is a structural plane-local region: one outer loop and its
// holes. Its measurement methods integrate validated records in momentinput.
type ProfileRecord = momentinput.Profile

// LoopRecord is one closed directed boundary walk.
type LoopRecord = sectionrecord.LoopRecord

// ChainRecord is one open directed boundary walk.
type ChainRecord = sectionrecord.ChainRecord

// CurveSegment is a recorded curve with a sealed variant set.
type CurveSegment = sectionrecord.CurveSegment

// LineSeg records a line and its parameter range.
type LineSeg = sectionrecord.LineSeg

// CircleSeg records a circle and its parameter range.
type CircleSeg = sectionrecord.CircleSeg

// ArcSeg records an arc and its parameter range.
type ArcSeg = sectionrecord.ArcSeg

// EllipseSeg records an ellipse and its parameter range.
type EllipseSeg = sectionrecord.EllipseSeg

// EllipticalArcSeg records an elliptical arc and its parameter range.
type EllipticalArcSeg = sectionrecord.EllipticalArcSeg

// SplineSeg records a spline and its parameter range.
type SplineSeg = sectionrecord.SplineSeg

// NURBSSeg records a NURBS curve and its parameter range.
type NURBSSeg = sectionrecord.NURBSSeg

// ClosedSplineSeg records a closed spline and its parameter range.
type ClosedSplineSeg = sectionrecord.ClosedSplineSeg

// FitSplineSeg records a fit spline and its parameter range.
type FitSplineSeg = sectionrecord.FitSplineSeg

// ConicSeg records a conic and its parameter range.
type ConicSeg = sectionrecord.ConicSeg

func cloneLoopRecord(loop LoopRecord) LoopRecord  { return sectionrecord.CloneLoopRecord(loop) }
func validateNURBSSegment(segment NURBSSeg) error { return sectionrecord.ValidateNURBSSegment(segment) }
func validateSegment(segment CurveSegment) error  { return sectionrecord.ValidateSegment(segment) }
func normalizeSegment(segment CurveSegment) (CurveSegment, error) {
	return sectionrecord.NormalizeSegment(segment)
}
