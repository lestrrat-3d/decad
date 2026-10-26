package facetedrecord

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ScaleTag returns the published image of a retained Plane, Cylinder, Sphere or
// Torus tag under the positive uniform scale x -> anchor + s*(x - anchor), and
// the normal departure the image gains from the exact similar surface
// (agent-docs/retained-modify-reach-design.md §7). It reports false for any
// other tag, and for a tag whose image has no stated bound: a plane whose
// frame is not representable, or a curved tag whose radii have no finite
// millimetre reading or whose displacement reaches the A4 limit.
//
// A uniform scale keeps every normal, so the exact image of the tag is the
// true image of its patch (Claim R) with the same normals at corresponding
// points. Only the arithmetic departs from it:
//
//   - Plane. The origin is rounded (gap P from anchor + s*(o - anchor)), and
//     NewFrame may round the unchanged frame vectors again (largest gap
//     alpha from U and V). The charge is 8*alpha + P, the plane charge of
//     PlacedTagDepartureBy with the identity as the map of directions.
//   - Cylinder, Sphere, Torus. The axis is unchanged. The origin or centre is
//     rounded (gap P) and each rounded radius departs from s times its source
//     by rho. The tag surface lies within P + rho of the exact image (a torus
//     adds the gaps of both radii), and that displacement enters the held term
//     of retention §3.3 at twice its size, against the published radii.
func ScaleTag(tag surfacegeom.Surface, anchor r3.Vec, s float64) (surfacegeom.Surface, float64, bool) {
	linear := [3][3]*big.Rat{}
	factor := ratOfPointPlane(s)
	zero := new(big.Rat)
	for i := range linear {
		linear[i] = [3]*big.Rat{zero, zero, zero}
		linear[i][i] = factor
	}
	var translation [3]*big.Rat
	one := big.NewRat(1, 1)
	for i, c := range [3]float64{anchor.X, anchor.Y, anchor.Z} {
		translation[i] = pointPlaneMul(new(big.Rat).Sub(one, factor), ratOfPointPlane(c))
	}
	around := func(p r3.Vec) r3.Vec { return anchor.Add(p.Sub(anchor).Scale(s)) }
	switch t := tag.(type) {
	case surfacegeom.Plane:
		origin := around(t.Frame.Origin())
		frame, err := r3.NewFrame(origin, t.Frame.U(), t.Frame.V())
		if err != nil {
			return nil, 0, false
		}
		alpha := max(imageGap(identityRat(), t.Frame.U(), frame.U()), imageGap(identityRat(), t.Frame.V(), frame.V()))
		p := pointGap(linear, translation, t.Frame.Origin(), origin)
		return surfacegeom.Plane{Frame: frame}, proofbound.AbsSumUpper(proofbound.ProductUpper(8, alpha), p), true
	case surfacegeom.Cylinder:
		radius, rho, ok := scaledRadius(t.Radius, s)
		if !ok {
			return nil, 0, false
		}
		moved := surfacegeom.Cylinder{Origin: around(t.Origin), Axis: t.Axis, Radius: radius}
		p := pointGap(linear, translation, t.Origin, moved.Origin)
		add, ok := curvedDeparture(moved, 0, 0, proofbound.AbsSumUpper(p, rho), moved.Origin, nil, 0)
		return moved, add, ok
	case surfacegeom.Sphere:
		radius, rho, ok := scaledRadius(t.Radius, s)
		if !ok {
			return nil, 0, false
		}
		moved := surfacegeom.Sphere{Center: around(t.Center), Radius: radius}
		p := pointGap(linear, translation, t.Center, moved.Center)
		add, ok := curvedDeparture(moved, 0, 0, proofbound.AbsSumUpper(p, rho), moved.Center, nil, 0)
		return moved, add, ok
	case surfacegeom.Torus:
		major, rhoMajor, ok := scaledRadius(t.Major, s)
		if !ok {
			return nil, 0, false
		}
		minor, rhoMinor, ok := scaledRadius(t.Minor, s)
		if !ok {
			return nil, 0, false
		}
		moved := surfacegeom.Torus{Center: around(t.Center), Axis: t.Axis, Major: major, Minor: minor}
		p := pointGap(linear, translation, t.Center, moved.Center)
		shift := proofbound.AbsSumUpper(p, rhoMajor, rhoMinor)
		add, ok := curvedDeparture(moved, 0, 0, shift, moved.Center, nil, 0)
		return moved, add, ok
	}
	return nil, 0, false
}

// scaledRadius returns the radius rounded from s times the source radius and
// the exact gap of that rounding.
func scaledRadius(source units.Value, s float64) (units.Value, float64, bool) {
	r, ok := millimeters(source)
	if !ok {
		return units.Value{}, 0, false
	}
	rounded := s * r
	if proofbound.IsNonFinite(rounded) {
		return units.Value{}, 0, false
	}
	exact := pointPlaneMul(ratOfPointPlane(s), ratOfPointPlane(r))
	return units.Millimeters(rounded), proofbound.RatAbsDiff(exact, rounded), true
}

func identityRat() [3][3]*big.Rat {
	var m [3][3]*big.Rat
	for i := range m {
		for j := range m[i] {
			m[i][j] = new(big.Rat)
		}
		m[i][i].SetInt64(1)
	}
	return m
}
