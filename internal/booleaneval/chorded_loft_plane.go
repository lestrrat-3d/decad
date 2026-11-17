package booleaneval

import (
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// loftWallRolePrefix starts the role of every wall face of a loft: side(i,j,k).
const loftWallRolePrefix = "side("

// chordedLoftPlane states, on the retained group of a wall face of a chorded
// loft, how far the face's true piece lies from the tag.
//
// A wall face of a loft is a Plane through its own three held vertices. For a
// circular or free-form pair that triangle only chords a ruled patch
// (loftrecord.Payload.Chorded), so the plane is not the surface the loft
// denotes. A retained tag claims the plane IS the true surface, and the
// retained-plane slab (retainedPlaneOf) would read a facet on it as lying in
// its plane. The group states the loft's own facet departure as its
// AxialDelta instead, so the slab charges it
// (agent-docs/faceted-source-retention-design.md §11, "Displaced plane").
//
// A group that carries no tag, a face that is not a loft wall, and a loft with
// no chorded cell are returned as they are. A chorded loft whose departure is
// not a finite non-negative number states nothing a slab could charge, so its
// group keeps no tag.
func chordedLoftPlane(group facetedrecord.Group, owner *topo.Body, f *topo.Face) facetedrecord.Group {
	if _, plane := group.Surface.(surfacegeom.Plane); !plane || !group.Retained {
		return group
	}
	lp, ok := owner.Payload.(loftrecord.Payload)
	if !ok || !lp.Chorded {
		return group
	}
	if !slices.ContainsFunc(f.Origins, func(o provenance.Ref) bool { return strings.HasPrefix(o.Role, loftWallRolePrefix) }) {
		return group
	}
	delta := lp.Proof.FacetDeparture
	if !(delta >= 0) || math.IsInf(delta, 0) {
		group.Surface, group.Retained, group.Reversed, group.NormalBound = nil, false, false, 0
		group.CapRecord, group.AxialDelta, group.HasAxialDelta = nil, 0, false
		group.ExtentDelta = 0
		return group
	}
	group.AxialDelta, group.HasAxialDelta = max(group.AxialDelta, delta), true
	return group
}
