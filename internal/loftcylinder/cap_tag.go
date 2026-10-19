package loftcylinder

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// TagCaps gives the two already audited cap groups of pp the result's own cap
// roles under producer ref and the source blank's exact planar surface. The
// joined cap audit proves each disk tiles its outline; the bore paths audit
// each annulus before calling it. It refuses unless, for each cap, exactly
// one face origin of blank names the blank's cap role, that face is an
// outward axial plane at the cap's level, and the cap group has facets that
// all lie on that level. The mesh's remaining groups keep their faceted
// surface identity.
func TagCaps(pp *facetedrecord.Payload, blank *topo.Body, groups [2]int,
	levels [2]float64, ref provenance.Producer) error {
	if blank == nil || len(pp.Tris) != len(pp.Src) || groups[0] == groups[1] {
		return fmt.Errorf("%w: loft/cylinder caps have no separate source groups", decaderr.ErrUnsupported)
	}
	roles := [2]string{provenance.RoleCapStart, provenance.RoleCapEnd}
	faces := topobuild.Faces(blank)
	for cap, group := range groups {
		if group < 0 || group >= len(pp.Groups) {
			return fmt.Errorf("%w: loft/cylinder cap names no source group", decaderr.ErrUnsupported)
		}
		want := provenance.Ref{Producer: blank.Origin.Producer, Role: roles[cap]}
		var source *topo.Face
		for _, face := range faces {
			for _, origin := range face.Origins {
				if origin != want {
					continue
				}
				if source != nil {
					return fmt.Errorf("%w: loft/cylinder blank has duplicate cap roles", decaderr.ErrUnsupported)
				}
				source = face
			}
		}
		if source == nil {
			return fmt.Errorf("%w: loft/cylinder blank has no cap source", decaderr.ErrUnsupported)
		}
		plane, ok := source.Surface.(surfacegeom.Plane)
		if !ok || plane.Frame.Origin().Z != levels[cap] {
			return fmt.Errorf("%w: loft/cylinder blank cap has no matching plane", decaderr.ErrUnsupported)
		}
		normal := float64(1)
		if cap == 0 {
			normal = -1
		}
		if plane.Frame.N() != (r3.Vec{Z: normal}) {
			return fmt.Errorf("%w: loft/cylinder blank cap has no axial normal", decaderr.ErrUnsupported)
		}
		count := 0
		for i, tri := range pp.Tris {
			if pp.Src[i] != group {
				continue
			}
			count++
			for _, index := range tri {
				if index < 0 || index >= len(pp.Verts) || pp.Verts[index].Z != levels[cap] {
					return fmt.Errorf("%w: loft/cylinder cap facet leaves its source plane", decaderr.ErrUnsupported)
				}
			}
		}
		if count == 0 {
			return fmt.Errorf("%w: loft/cylinder cap group has no facets", decaderr.ErrUnsupported)
		}
		g := &pp.Groups[group]
		g.Origins = append(append([]provenance.Ref(nil), g.Origins...),
			provenance.Ref{Producer: ref, Role: roles[cap]})
		g.Surface = plane
		g.Planar = true
		g.NormalBound = source.NormalBound
		g.Reversed = source.Reversed
	}
	return nil
}

// TagBridgedTop drops the exact top Plane tag that TagCaps gave group
// topGroup of pp when any axialBridge shift is positive, because an original
// tooth then ends below the held cap. The cap role stays;
// facetedrecord.BuildWithProof supplies the source-to-held displacement on its
// surface.
func TagBridgedTop(pp *facetedrecord.Payload, topGroup int, axialBridge []float64) {
	for _, shift := range axialBridge {
		if shift > 0 {
			top := &pp.Groups[topGroup]
			top.Surface = nil
			top.Planar = false
			return
		}
	}
}
