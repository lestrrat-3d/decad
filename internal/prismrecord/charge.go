package prismrecord

import (
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ChargeMap widens every reading a plane-coordinate build published —
// the volume, the body's and each face's area, each edge's length — to
// cover the image under the map the prism family denotes through:
// L = B·[U V N], the frame's held U, V and N and the placement's held basis
// read as exact rationals (massmoment.PrismRotation, the leaves
// proofbound.ExactFrameLiftRound compares every lifted vertex against).
// Every prism-family build integrates its readings in plane coordinates,
// and r3 keeps L orthonormal only to rounding (docs/evaluator-design.md §5).
// The centroid and the box need no charge: an affine map carries a centroid
// to its image's centroid, and the box reads the linear functional g·L over
// the plane-coordinate body. An exactly orthonormal L changes nothing.
func ChargeMap(body *topo.Body, frame r3.Frame, xform r3.Transform) error {
	l, err := massmoment.PrismRotation(frame, xform)
	if err != nil {
		return err
	}
	c, err := massmoment.MapChargeOf(l)
	if err != nil {
		return err
	}
	ChargeBodyMap(body, c)
	return nil
}

// ChargeBodyMap is ChargeMap's widening for an already-read charge.
func ChargeBodyMap(body *topo.Body, c massmoment.MapCharge) {
	if c == (massmoment.MapCharge{}) {
		return
	}
	if body.Solid {
		v := c.VolumeOf(proofbound.MeasuredScalar(body.Volume.Value.Base(), body.Volume.Bound.Base()))
		body.Volume.Bound = units.CubicMillimeters(v.Bound)
		body.Volume.Exactness = measurement.ExactnessOf(v.Bound)
	}
	a := c.AreaOf(proofbound.MeasuredScalar(body.Area.Value.Base(), body.Area.Bound.Base()))
	body.Area.Bound = units.SquareMillimeters(a.Bound)
	body.Area.Exactness = measurement.ExactnessOf(a.Bound)
	for _, f := range topobuild.Faces(body) {
		f.AreaBound = c.AreaOf(proofbound.MeasuredScalar(f.Area, f.AreaBound)).Bound
	}
	for _, e := range topobuild.Edges(body) {
		e.LengthBound = c.LengthBound(e.Length, e.LengthBound)
	}
}
