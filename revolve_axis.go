package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"

	"github.com/lestrrat-3d/r3"
)

// This file resolves a revolve's axis into the sketch plane and decides what
// the profile may do around it: revolveaxis.Line2, the 2D line the axis projects to;
// axisFrame, the axis-local frame every later reading is taken in; and the
// two gates that refuse a profile crossing or touching the axis where the
// sweep would be degenerate.
//
// wallKind is decided here because it follows from the axis alone: a segment
// parallel to the axis sweeps a cylinder, one meeting it at an angle a cone,
// one perpendicular an annulus. A segment whose kind the axis cannot decide
// exactly refuses rather than being assigned the nearest one. See
// docs/evaluator-design.md §6.

// axisInPlane reads the public axis variant and resolves its coordinates.
func axisInPlane(a Axis, frame r3.Frame) (revolveaxis.Line2, error) {
	var input revolveaxis.AxisInput
	switch value := a.(type) {
	case SketchLine:
		input = revolveaxis.SketchLine{
			StartU: value.Start.U, StartV: value.Start.V,
			EndU: value.End.U, EndV: value.End.V,
		}
	case ConstructionAxis:
		input = revolveaxis.ConstructionAxis{Origin: value.Origin, Dir: value.Dir}
	default:
		return revolveaxis.Line2{}, fmt.Errorf(`%w: axis %T is not supported by this evaluator`, ErrUnsupported, a)
	}
	return revolveaxis.AxisInPlane(input, frame)
}

// The payload holds the internal axis frame and its profile proof charges.
type axisFrame = revolveaxis.Frame
type regionSnapAllow = revolveaxis.SnapAllow

// wallKind classifies what one boundary walk sweeps.
type wallKind = revolveaxis.WallKind

const (
	// wallAxis is a line lying along the axis: it sweeps a zero-area set
	// and emits no face — the neighboring segments' faces close the solid.
	wallAxis = revolveaxis.WallAxis
	// wallCylinder is a line parallel to the axis.
	wallCylinder = revolveaxis.WallCylinder
	// wallPlane is a line perpendicular to the axis: a planar annulus, or a
	// disk when it reaches the axis.
	wallPlane = revolveaxis.WallPlane
	// wallCone is an inclined line; an endpoint on the axis is its apex.
	wallCone = revolveaxis.WallCone
	// wallSphere is a circular walk whose center lies on the axis.
	wallSphere = revolveaxis.WallSphere
	// wallTorus is a circular walk whose center lies off the axis.
	wallTorus    = revolveaxis.WallTorus
	wallFreeform = revolveaxis.WallFreeform
)

// resolveAxisSide scans the record, asks revolveaxis to decide its side, then
// audits each walk for axis contact and charges any snapped endpoint.
func resolveAxisSide(ctx context.Context, profile profileRecord, line revolveaxis.Line2, work *freeform.FreeformWork) (axisFrame, float64, error) {
	nU, nV := -line.DV, line.DU
	rlo, rhi, rBound, err := boundaryExtremesBoundedContext(ctx, profile, nU, nV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	zlo, zhi, zBound, err := boundaryExtremesBoundedContext(ctx, profile, line.DU, line.DV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}

	// The side gate charges the scan's dot-product rounding separately from
	// the positional bounds returned with the extremes.
	coordUpper, err := momentinput.CoordinateEnvelope(profile, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	resolved, err := revolveaxis.ResolveSide(line, revolveaxis.SideExtremes{
		RLo: rlo, RHi: rhi, RBound: rBound,
		ZLo: zlo, ZHi: zhi, ZBound: zBound,
		CoordUpper: coordUpper,
	})
	if err != nil {
		return axisFrame{}, 0, err
	}
	ax := resolved.Frame
	ax.RadialAdmitAllow = resolved.RadialAdmitAllow
	ax.RadialProof = resolved.RadialProof
	ax.AxialExtentUpper = resolved.AxialExtentUpper
	snap, err := revolveaxis.AuditProfileAxisContact(ax, profile, work)
	if err != nil {
		return axisFrame{}, 0, err
	}
	ax.Snap = snap
	return ax, resolved.Side, nil
}
