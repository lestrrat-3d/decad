package booleaneval

import (
	"math"

	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// retainedCylinderOf names the exact cylinder of each facet's source face when
// that face is a retained Cylinder whose tag is the operand's own surface: no
// stated tag departure, so no placement or similarity left the tag a float
// image (agent-docs/faceted-source-retention-design.md §11, Claim C). By Claim
// R the face's true piece lies on that surface.
//
// A face with a denoted revolve patch names its cylinder only when the tag is
// exactly the cylinder the patch denotes, decided in exact arithmetic
// (stitchflux.TagIsDenoted), as retainedPlaneOf decides a plane. The edge
// curves are not passed: they bear on the flux arms' rims, not on where the
// true piece lies.
//
// The cylinder's Delta is the face's wall charge (topo.Face.AxialDelta): the
// true wall lies within it of the tag. A negative or non-finite charge names no
// cylinder.
//
// The returned function is not safe for concurrent use: it memoises the
// denoted-cylinder decision per source face.
func retainedCylinderOf(m *topo.Mesh) func(facet int) (meshbool.SlabCylinder, bool) {
	denoted := map[*topo.Face]bool{}
	return func(facet int) (meshbool.SlabCylinder, bool) {
		if facet >= len(m.Source) {
			return meshbool.SlabCylinder{}, false
		}
		f := m.Source[facet]
		if f == nil || !f.RetainedTag || f.TagDeparture != 0 {
			return meshbool.SlabCylinder{}, false
		}
		cyl, ok := f.Surface.(surfacegeom.Cylinder)
		if !ok {
			return meshbool.SlabCylinder{}, false
		}
		if f.Denoted != nil {
			exact, seen := denoted[f]
			if !seen {
				exact = stitchflux.TagIsDenoted(stitchflux.TaggedFace{Denoted: f.Denoted, Surface: f.Surface})
				denoted[f] = exact
			}
			if !exact {
				return meshbool.SlabCylinder{}, false
			}
		}
		radius, err := cyl.Radius.In(units.Millimeter)
		if err != nil {
			return meshbool.SlabCylinder{}, false
		}
		delta := 0.0
		if f.HasAxialDelta {
			delta = f.AxialDelta
		}
		if !(delta >= 0) || math.IsInf(delta, 0) {
			return meshbool.SlabCylinder{}, false
		}
		return meshbool.SlabCylinder{Origin: cyl.Origin, Axis: cyl.Axis, Radius: radius, Delta: delta}, true
	}
}

// denotedCylinderFaces counts the distinct source faces of m whose slab
// reading runs stitchflux.TagIsDenoted: retained Cylinder faces with no stated
// tag departure that carry a denoted patch.
func denotedCylinderFaces(m *topo.Mesh) uint64 {
	seen := map[*topo.Face]struct{}{}
	for _, f := range m.Source {
		if f == nil || !f.RetainedTag || f.TagDeparture != 0 || f.Denoted == nil {
			continue
		}
		if _, ok := f.Surface.(surfacegeom.Cylinder); ok {
			seen[f] = struct{}{}
		}
	}
	return uint64(len(seen))
}
