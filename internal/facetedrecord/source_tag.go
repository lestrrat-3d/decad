package facetedrecord

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// SourceTag reads the group fields a mesh Boolean copies from the operand face
// f (docs/faceted-source-retention-design.md §3). flip is true for the faces
// of the tool of a Cut, whose natural normal points into the result. The
// second result is false when A1–A2 decline the face, and the group then
// carries no surface. f must not be nil.
func SourceTag(f *topo.Face, flip bool) (Group, bool) {
	if f.Denoted != nil {
		return Group{}, false
	}
	switch f.Surface.(type) {
	case surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Sphere:
	default:
		return Group{}, false
	}
	departure := f.NormalBound
	if f.RetainedTag {
		departure = f.TagDeparture
	} else if departure != 0 {
		// A built face with a stated tilt departs from its tag by more than
		// the tilt: the tag is a ruled stand-in, not the face's geometry.
		return Group{}, false
	}
	return Group{
		Surface:     f.Surface,
		Retained:    true,
		Reversed:    f.Reversed != flip,
		NormalBound: departure,
	}, true
}

// RetainedNormalBound returns the NormalBound of a retained face whose tag
// departs by departure and whose facets sit within delta of the true patch
// (docs/faceted-source-retention-design.md §3.3), and whether the tag is
// admitted (A4). A plane adds nothing to departure; a cylinder or sphere of
// radius R adds 2·delta/R, capped at 2, and is declined once that reaches 1.
func RetainedNormalBound(tag surfacegeom.Surface, departure, delta float64) (float64, bool) {
	var radius units.Value
	switch s := tag.(type) {
	case surfacegeom.Plane:
		return departure, true
	case surfacegeom.Cylinder:
		radius = s.Radius
	case surfacegeom.Sphere:
		radius = s.Radius
	default:
		return 0, false
	}
	r, err := radius.In(units.Millimeter)
	if err != nil || !(r > 0) || math.IsInf(r, 0) {
		return 0, false
	}
	term := math.Nextafter(2*delta/r, math.Inf(1))
	if !(term < 1) {
		return 0, false
	}
	return math.Nextafter(departure+term, math.Inf(1)), true
}
