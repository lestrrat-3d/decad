package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/revolveaxis"

	"github.com/lestrrat-3d/r3"
)

// This file holds axisInPlane and the aliases axisFrame and wallKind. The
// profile gates live in revolveplan.ResolveAxisSide
// (internal/revolveplan/axis_side.go). See docs/evaluator-design.md §6.

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

// revolveWalks is the shared plane and axis-coordinate resolution.
type revolveWalks = revolveaxis.ResolvedWalks

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
)
