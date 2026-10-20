package facetedrecord

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// SourceTag reads the group fields a mesh Boolean copies from the operand face
// f (agent-docs/faceted-source-retention-design.md §3). flip is true for the faces
// of the tool of a Cut, whose natural normal points into the result. The
// second result is false when A1–A2 decline the face, and the group then
// carries no surface. f must not be nil.
func SourceTag(f *topo.Face, flip bool) (Group, bool) {
	switch f.Surface.(type) {
	case surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Cone, surfacegeom.Sphere, surfacegeom.Torus:
	default:
		return Group{}, false
	}
	g := Group{
		Surface:  f.Surface,
		Retained: true,
		Reversed: f.Reversed != flip,
		Denoted:  f.Denoted,
	}
	switch {
	case f.Denoted != nil:
		// NormalAt judges a denoted face against its denoted surface, which
		// covers the tag's whole departure; NormalBound is carried as the
		// operand states it, for the readers that gate on it.
		g.NormalBound = f.NormalBound
	case f.RetainedTag:
		g.NormalBound = f.TagDeparture
	case f.NormalBound != 0:
		// A built face with a stated tilt departs from its tag by more than
		// the tilt: the tag is a ruled stand-in, not the face's geometry.
		return Group{}, false
	}
	if _, plane := f.Surface.(surfacegeom.Plane); plane && f.CapRecord != nil && f.HasAxialDelta {
		g.CapRecord, g.AxialDelta, g.HasAxialDelta = f.CapRecord, f.AxialDelta, true
	}
	return g, true
}

// HeldNormalTerm returns the normal bound a retained tag adds for facets that
// sit within delta of the true patch (agent-docs/faceted-source-retention-design.md
// §3.3), and whether the tag is admitted (A4). rhoLower is a lower bound on the
// distance from the axis of every true point of a Cone patch; the other
// surfaces ignore it. The term is zero for a plane. For the others it is
// 2·delta/R times a surface factor, capped at 2, and the tag is declined once
// the term reaches 1.
func HeldNormalTerm(tag surfacegeom.Surface, delta, rhoLower float64) (float64, bool) {
	var term float64
	switch s := tag.(type) {
	case surfacegeom.Plane:
		return 0, true
	case surfacegeom.Cylinder:
		r, ok := millimeters(s.Radius)
		if !ok {
			return 0, false
		}
		term = 2 * delta / r
	case surfacegeom.Sphere:
		r, ok := millimeters(s.Radius)
		if !ok {
			return 0, false
		}
		term = 2 * delta / r
	case surfacegeom.Cone:
		half, err := s.HalfAngle.In(units.Radian)
		if err != nil || !(rhoLower > 0) || math.IsInf(rhoLower, 0) {
			return 0, false
		}
		term = math.Abs(math.Cos(half)) * 2 * delta / rhoLower
	case surfacegeom.Torus:
		major, ok := millimeters(s.Major)
		if !ok {
			return 0, false
		}
		minor, ok := millimeters(s.Minor)
		if !ok || !(major > minor) {
			return 0, false
		}
		// The tube centre moves with the radial direction, which a true
		// point's distance from the axis (at least major − minor) bounds.
		term = 2 * delta * (1 + 2*major/(major-minor)) / minor
	default:
		return 0, false
	}
	term = math.Nextafter(term, math.Inf(1))
	if !(term < 1) {
		return 0, false
	}
	return term, true
}

func millimeters(v units.Value) (float64, bool) {
	r, err := v.In(units.Millimeter)
	if err != nil || !(r > 0) || math.IsInf(r, 0) {
		return 0, false
	}
	return r, true
}

// ConeAxialMin returns the smallest coordinate along the cone's axis,
// measured from its origin, over the vertices of the triangle (a, b, c).
func ConeAxialMin(cone surfacegeom.Cone, a, b, c r3.Vec) float64 {
	along := func(x r3.Vec) float64 { return x.Sub(cone.Origin).Dot(cone.Axis) }
	return min(along(a), along(b), along(c))
}

// ConeRhoLower returns a lower bound on the distance from the axis of every
// true point of a cone patch whose held facets reach no lower than axial
// coordinate sMin and sit within delta of the true patch. The cone's radius
// grows linearly along its axis, so the bound is the radius at the lowest
// axial coordinate a true point can have, less the rounding of the
// coordinates. A non-positive result means the patch may reach the axis.
func ConeRhoLower(cone surfacegeom.Cone, sMin, delta float64) float64 {
	r0, radiusErr := cone.Radius.In(units.Millimeter)
	half, angleErr := cone.HalfAngle.In(units.Radian)
	if radiusErr != nil || angleErr != nil || r0 < 0 || math.IsInf(r0, 0) {
		return 0
	}
	scale := max(math.Abs(sMin), math.Abs(r0)) + delta
	slope := math.Tan(half) * (1 - 16*0x1p-52)
	return r0 + slope*(sMin-delta-16*0x1p-52*scale) - 16*0x1p-52*scale
}

// coneAxialMin returns, for each face whose retained group is a Cone, the
// smallest axial coordinate over its facets' vertices.
func coneAxialMin(groups []Group, faceGroup map[*topo.Face]int, verts []r3.Vec, tris [][3]int,
	facetFace []*topo.Face) map[*topo.Face]float64 {
	out := map[*topo.Face]float64{}
	for i, t := range tris {
		f := facetFace[i]
		group := groups[faceGroup[f]]
		cone, ok := group.Surface.(surfacegeom.Cone)
		if !ok || !group.Retained {
			continue
		}
		s := ConeAxialMin(cone, verts[t[0]], verts[t[1]], verts[t[2]])
		if prior, seen := out[f]; !seen || s < prior {
			out[f] = s
		}
	}
	return out
}
