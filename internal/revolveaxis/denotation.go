package revolveaxis

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// DenotationInput is the plain data a revolve's wall and cap denotations read:
// the plane lift, the axis's proven bounds, the placement and the section
// displacement with its whole-section flag.
type DenotationInput struct {
	Lift         revolvemesh.RevolveLift
	Axis         revolvemesh.AxisBound
	Xform        r3.Transform
	SectionWhole bool
	SectionDelta float64
}

// WallDenotation encloses the surface the wall w denotes: its recorded
// segments, read off the plane-local walks they were re-expressed from, swept
// about the recorded axis and placed (surfacenormal.Revolved). A straight
// wall reads each recorded segment's own two ends, in walk order; a circular
// wall reads each segment's recorded centre and radius. Face.NormalAt judges
// the wall's normal against it, and Stitch's flux path admits the wall only
// where its tag is exactly this surface (agent-docs/evaluator-design.md §6,
// agent-docs/surface-design.md §6.4).
func (d DenotationInput) WallDenotation(w survey2d.SideWalk, kind WallKind,
	plane []survey2d.SegmentWalk) *surfacenormal.Revolved {
	if kind == WallFreeform {
		return nil
	}
	lift, ab := d.Lift, d.Axis
	section := SectionWholeCharges(d.SectionWhole, d.SectionDelta)
	var den surfacenormal.Revolved
	switch kind {
	case WallSphere, WallTorus:
		circles := make([]revolvemesh.RecordedMeridian, len(w.Segs))
		for i, si := range w.Segs {
			pw := plane[si]
			circles[i] = revolvemesh.RecordedMeridian{
				U: pw.CU, V: pw.CV, UV: DenotedBound(proofbound.WalkEndBound{}, section),
				R: pw.Radius, RBound: DenotedRadiusBound(pw.RadiusBound, section),
			}
		}
		den = lift.CircularWallNormal(ab, d.Xform, circles)
	default:
		ends := make([][2]revolvemesh.RecordedMeridian, len(w.Segs))
		for i, si := range w.Segs {
			pw := plane[si]
			ends[i] = [2]revolvemesh.RecordedMeridian{
				{U: pw.StartU, V: pw.StartV, UV: DenotedBound(pw.StartBound, section)},
				{U: pw.EndU, V: pw.EndV, UV: DenotedBound(pw.EndBound, section)},
			}
		}
		den = lift.StraightWallNormal(ab, d.Xform, ends)
	}
	return &den
}

// CapDenotation encloses the plane a partial sweep's cap at end denotes: the
// plane through the recorded axis at the angle end's record states, placed
// (revolvemesh.RevolveLift.CapNormal). An end with no denotation — a
// ToFaceAngular stop, or a payload literal built without one — states no
// angle, so it returns nil and the cap keeps its tag's proof composed with
// its normalBound, which is +Inf there.
func (d DenotationInput) CapDenotation(den revolveangle.Angle, phi float64, start bool) *surfacenormal.Revolved {
	if !den.Valid() {
		return nil
	}
	sin, cos, ok := den.SinCosFor(phi)
	if !ok {
		return nil
	}
	out := d.Lift.CapNormal(d.Axis, d.Xform, sin, cos, start)
	return &out
}

// CapArcCurveBound is an Edge's curveBound for a partial sweep's cap copy of
// a recorded circular walk: plane holds the plane-local walks of the
// recorded segments the copy covers, every one on one circle, whose centre
// is recorded and whose radius sits within its walk's RadiusBound; a
// whole-section displacement widens both (revolvemesh.RevolveLift.CapArcGap).
// An end with no denotation, a walk set that is not one circle, or a bound
// not below half the held radius answers false.
func (d DenotationInput) CapArcCurveBound(plane []survey2d.SegmentWalk, den revolveangle.Angle, phi float64,
	center, axis r3.Vec, radius float64) (float64, bool) {
	if len(plane) == 0 || !den.Valid() {
		return 0, false
	}
	sin, cos, ok := den.SinCosFor(phi)
	if !ok {
		return 0, false
	}
	first := plane[0]
	radiusBound := 0.0
	for _, pw := range plane {
		if !pw.IsCircular() || pw.CU != first.CU || pw.CV != first.CV || pw.Radius != first.Radius {
			return 0, false
		}
		radiusBound = math.Max(radiusBound, pw.RadiusBound)
	}
	charge := SectionWholeCharges(d.SectionWhole, d.SectionDelta)
	if charge.U != 0 {
		radiusBound = proofbound.AbsSumUpper(radiusBound, d.SectionDelta)
	}
	gap := d.Lift.CapArcGap(d.Axis, d.Xform, first.CU, first.CV, charge, first.Radius, radiusBound, sin, cos, center, axis, radius)
	if proofbound.IsNonFinite(gap) || !(proofbound.ProductUpper(2, gap) < radius) {
		return 0, false
	}
	return gap, true
}
