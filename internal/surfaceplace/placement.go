package surfaceplace

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// TransformSurface carries a face's descriptive vectors through a rigid
// placement with Apply for positions and ApplyDir for directions. It uses the
// built geometry rather than re-deriving a surface from its source record.
// NURBSSurface and Faceted are opaque markers and pass through unchanged.
func TransformSurface(s surfacegeom.Surface, xf r3.Transform) (surfacegeom.Surface, error) {
	switch v := s.(type) {
	case surfacegeom.Plane:
		origin := xf.Apply(v.Frame.Origin())
		u := xf.ApplyDir(v.Frame.U())
		vv := xf.ApplyDir(v.Frame.V())
		f, err := r3.NewFrame(origin, u, vv)
		if err != nil {
			return nil, fmt.Errorf(`%w: a placed stitch face's plane is degenerate: %s`, decaderr.ErrUnsupported, err)
		}
		return surfacegeom.Plane{Frame: f}, nil
	case surfacegeom.Cylinder:
		return surfacegeom.Cylinder{Origin: xf.Apply(v.Origin), Axis: xf.ApplyDir(v.Axis), Radius: v.Radius}, nil
	case surfacegeom.Cone:
		return surfacegeom.Cone{Origin: xf.Apply(v.Origin), Axis: xf.ApplyDir(v.Axis), Radius: v.Radius, HalfAngle: v.HalfAngle}, nil
	case surfacegeom.Sphere:
		return surfacegeom.Sphere{Center: xf.Apply(v.Center), Radius: v.Radius}, nil
	case surfacegeom.Torus:
		return surfacegeom.Torus{Center: xf.Apply(v.Center), Axis: xf.ApplyDir(v.Axis), Major: v.Major, Minor: v.Minor}, nil
	case surfacegeom.NURBSSurface:
		return v, nil
	case surfacegeom.Faceted:
		return v, nil
	default:
		return nil, fmt.Errorf(`%w: Stitch cannot transform surface kind %T`, decaderr.ErrUnsupported, s)
	}
}

// TransformCurve carries an edge's geometry through a rigid placement.
func TransformCurve(c surfacegeom.Curve, xf r3.Transform) (surfacegeom.Curve, error) {
	switch v := c.(type) {
	case surfacegeom.Line3:
		return surfacegeom.Line3{}, nil
	case surfacegeom.Circle3:
		return surfacegeom.Circle3{Center: xf.Apply(v.Center), Axis: xf.ApplyDir(v.Axis), Radius: v.Radius}, nil
	case surfacegeom.Arc3:
		return surfacegeom.Arc3{Center: xf.Apply(v.Center), Axis: xf.ApplyDir(v.Axis), Radius: v.Radius}, nil
	case surfacegeom.Ellipse3:
		// A reflection reverses the sense of rotation, so the placed arc stays
		// counter-clockwise from its start to its end about the negated axis.
		axis := xf.ApplyDir(v.Axis)
		if xf.IsReflection() {
			axis = axis.Scale(-1)
		}
		return surfacegeom.Ellipse3{
			Center: xf.Apply(v.Center), Axis: axis, Major: xf.ApplyDir(v.Major),
			SemiMajor: v.SemiMajor, SemiMinor: v.SemiMinor,
		}, nil
	case surfacegeom.FilletMiter3:
		return v, nil
	case surfacegeom.NURBSCurve:
		return v, nil
	case surfacegeom.FacetedCurve:
		return v, nil
	default:
		return nil, fmt.Errorf(`%w: Stitch cannot transform curve kind %T`, decaderr.ErrUnsupported, c)
	}
}

// PlacedCurveBound carries oldBound through xf onto the placed held circle.
// For old center c, axis a, radius ρ, placement basis B, and orthonormality
// defect e, a denoted point within κ of c+ρw maps within (1+e)κ of its held
// image. The held image lies within |B·c+t−c′|+2ρ|B·w·â′|+ρe of the placed
// circle, using |B·w·a′| ≤ (1+e)|a′−B·a|+e|a|. The identity placement
// returns oldBound unchanged. An unbounded old edge, a noncircular curve, or
// a bound that reaches half the radius returns false.
func PlacedCurveBound(old surfacegeom.Curve, oldBound float64, bounded bool, placed surfacegeom.Curve, xf r3.Transform) (float64, bool) {
	if !bounded {
		return 0, false
	}
	if xf == r3.Identity() {
		return oldBound, true
	}
	var oldCenter, oldAxis, newCenter, newAxis r3.Vec
	var radius float64
	switch c := old.(type) {
	case surfacegeom.Circle3:
		oldCenter, oldAxis, radius = c.Center, c.Axis, c.Radius.Base()
	case surfacegeom.Arc3:
		oldCenter, oldAxis, radius = c.Center, c.Axis, c.Radius.Base()
	default:
		return 0, false
	}
	switch c := placed.(type) {
	case surfacegeom.Circle3:
		newCenter, newAxis = c.Center, c.Axis
	case surfacegeom.Arc3:
		newCenter, newAxis = c.Center, c.Axis
	default:
		return 0, false
	}
	basis, err := massmoment.PlacementRotation(xf)
	if err != nil {
		return 0, false
	}
	charge, err := massmoment.MapChargeOf(basis)
	if err != nil {
		return 0, false
	}
	b := xf.Basis()
	for _, v := range [...]r3.Vec{oldCenter, oldAxis, newCenter, newAxis, b.EX, b.EY, b.EZ, xf.Translation()} {
		if !proofbound.FiniteVec(v) {
			return 0, false
		}
	}
	centerGap := proofbound.ExactRigidRound(b, xf.Translation(), proofarith.DyVec(oldCenter), newCenter)
	ex, ey, ez := proofarith.DyVec(b.EX), proofarith.DyVec(b.EY), proofarith.DyVec(b.EZ)
	a := proofarith.DyVec(oldAxis)
	var ba proofarith.DyV3
	for i := range 3 {
		ba[i] = proofarith.DyAdd(proofarith.DyAdd(proofarith.DyMul(ex[i], a[0]), proofarith.DyMul(ey[i], a[1])), proofarith.DyMul(ez[i], a[2]))
	}
	na := proofarith.DyVec(newAxis)
	diff := proofarith.DvSub(na, ba)
	newLow := proofarith.DySqrtDown(proofarith.DvDot(na, na))
	if !(newLow > 0) {
		return 0, false
	}
	onePlus := proofbound.AbsSumUpper(1, charge.Stretch)
	tilt := proofbound.AbsSumUpper(
		proofbound.ProductUpper(onePlus, proofarith.DySqrtUp(proofarith.DvDot(diff, diff))),
		proofbound.ProductUpper(charge.Stretch, proofarith.DySqrtUp(proofarith.DvDot(a, a))),
	)
	tilt = proofbound.UpRound(tilt / newLow)
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(onePlus, oldBound),
		centerGap,
		proofbound.ProductUpper(2, proofbound.ProductUpper(radius, tilt)),
		proofbound.ProductUpper(radius, charge.Stretch),
	)
	if proofbound.IsNonFinite(bound) || !(proofbound.ProductUpper(2, bound) < radius) {
		return 0, false
	}
	return bound, true
}
