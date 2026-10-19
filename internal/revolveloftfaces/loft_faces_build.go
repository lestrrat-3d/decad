package revolveloftfaces

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// LoftFacesSection is one section of a LoftFaces body. A selected section is
// a complete Split cap: Mesh is its slab's payload, Slice the slab's
// point-plane record, Limit the cap's plane limit, Group the cap's facet
// group in Mesh, Ring its station ring and Surface its face's surface. A
// constructed section (the cone-tangent end) carries only Constructed, its
// exact cap map, its Ring, and the first selected section's Slice, read only
// for its base mesh.
type LoftFacesSection struct {
	Mesh        facetedrecord.Payload
	Slice       *facetedrecord.PointPlaneSliceRecord
	Limit       *facetedrecord.PointPlaneLimit
	Group       int
	Ring        []r3.Vec
	Surface     surfacegeom.Surface
	Constructed *loftface.Map
}

// BuildLoftFaces lofts through the collected sections of one point-loft
// source and builds the certified body in env; env.Ref names its side and cap
// roles. It proves the cap embeddings, audits crossings, orients the mesh,
// proves its boundary and occupied-volume bounds against the held volume, and
// records the cap maps for later Booleans. A non-nil tangent means the last
// section is the constructed cone-tangent end it describes.
func BuildLoftFaces(ctx context.Context, env topo.Env, sections []LoftFacesSection,
	source *facetedrecord.PointSectionRecord, tangent *loftface.TangentEnd) (*topo.Body, error) {
	mesh, err := assembleLoftFaces(ctx, env.Ref, sections, source)
	if err != nil {
		return nil, err
	}
	embedSections := make([]EmbeddingSection, len(sections))
	for i, section := range sections {
		embedSections[i] = EmbeddingSection{
			Mesh: section.Mesh, Limit: section.Limit, Constructed: section.Constructed,
		}
	}
	embedding, err := ProveEmbedding(ctx, embedSections, source)
	if err != nil {
		return nil, err
	}
	base := sections[0].Slice.Base
	if layout, ok := loftmesh.LoftFacesCapLayout(base.Tris[:len(sections[0].Ring)],
		len(sections)-1, len(mesh.Tris)); ok {
		err = loftmesh.LoftCrossingAuditWithCaps(ctx, mesh.Verts, mesh.Tris, layout)
	} else {
		err = loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris)
	}
	if err != nil {
		return nil, err
	}
	if err := Orient(&mesh, base, source, embedding.Sign); err != nil {
		return nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.Verts, mesh.Tris)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	_, _, heldRat, err := facetproof.VolumeFromAudit(audit, 0)
	if ctxErr := ctx.Err(); ctxErr != nil {
		return nil, ctxErr
	}
	if err != nil {
		return nil, err
	}
	boundSections := make([]BoundsSection, len(sections))
	for i, section := range sections {
		boundSections[i] = BoundsSection{Ring: section.Ring, BaseMeshBound: section.Slice.Base.MeshBound}
	}
	mesh, err = ProveBounds(ctx, boundSections, source, embedding.Maps, embedding.JacobianSum, mesh, base)
	if err != nil {
		return nil, err
	}
	capBounds := make([]facetedrecord.LoftFaceCapBounds, len(sections))
	for i := range sections {
		capBounds[i], err = CapBounds(ctx, embedding.Maps[i], boundSections[i], source)
		if err != nil {
			return nil, err
		}
	}
	if facetedrecord.VolFloor(heldRat, mesh.VolSymDiff) <= 0 {
		return nil, fmt.Errorf("%w: face-fed loft occupied-volume proof exceeds the held volume", decaderr.ErrUnsupported)
	}
	if len(base.Src) < len(source.SourceStation) {
		return nil, fmt.Errorf("%w: loft source cell roles are incomplete", decaderr.ErrUnsupported)
	}
	mesh.LoftFacesPrimitive = true
	mesh.LoftFacesCap = &facetedrecord.LoftFacesCapRecord{Source: source,
		Maps:         append([]loftface.Map(nil), embedding.Maps...),
		Bounds:       capBounds,
		StationGroup: append([]int(nil), base.Src[:len(source.SourceStation)]...),
		Final:        embedding.Maps[len(embedding.Maps)-1], Group: len(mesh.Groups) - 1,
		Tangent: tangent}
	return facetedrecord.BuildWithProof(ctx, env, mesh, audit, measurement.Measurement{}, nil)
}

func assembleLoftFaces(ctx context.Context, ref provenance.Producer,
	sections []LoftFacesSection, source *facetedrecord.PointSectionRecord) (facetedrecord.Payload, error) {
	n := len(sections[0].Ring)
	base := sections[0].Slice.Base
	segmentCount := len(source.Profile.Outer.Segments)
	if len(base.Src) < n || len(base.Tris) < n || len(base.Groups) != segmentCount+1 {
		return facetedrecord.Payload{}, fmt.Errorf("%w: LoftFaces source fan has no complete side correspondence",
			decaderr.ErrUnsupported)
	}
	verts := make([]r3.Vec, 0, len(sections)*n)
	for i, section := range sections {
		if err := ctx.Err(); err != nil {
			return facetedrecord.Payload{}, err
		}
		if len(section.Ring) != n {
			return facetedrecord.Payload{}, fmt.Errorf("%w: LoftFaces section %d has different stations",
				decaderr.ErrUnsupported, i)
		}
		verts = append(verts, section.Ring...)
	}
	tris := make([][3]int, 0, 2*n*(len(sections)-1)+2*len(base.Tris[n:]))
	src := make([]int, 0, cap(tris))
	groups := make([]facetedrecord.Group, (len(sections)-1)*segmentCount+2)
	for band := range len(sections) - 1 {
		for segment := range segmentCount {
			groups[band*segmentCount+segment] = facetedrecord.Group{Origins: []provenance.Ref{{Producer: ref,
				Role: fmt.Sprintf("side(%d,%d)", band, segment)}}}
		}
		for j := range n {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return facetedrecord.Payload{}, err
				}
			}
			side := base.Tris[j]
			if side[0] != 0 || side[1] <= 0 || side[2] <= 0 {
				return facetedrecord.Payload{}, fmt.Errorf("%w: LoftFaces source fan has no ordered side",
					decaderr.ErrUnsupported)
			}
			b, c := side[1]-1, side[2]-1
			group := base.Src[j]
			if group < 0 || group >= segmentCount {
				return facetedrecord.Payload{}, fmt.Errorf("%w: LoftFaces source side has no segment",
					decaderr.ErrUnsupported)
			}
			lo, hi := band*n, (band+1)*n
			tris = append(tris, [3]int{lo + b, hi + b, hi + c}, [3]int{lo + b, hi + c, lo + c})
			src = append(src, band*segmentCount+group, band*segmentCount+group)
		}
	}
	startGroup, endGroup := len(groups)-2, len(groups)-1
	startOrigins := append([]provenance.Ref(nil), sections[0].Mesh.Groups[sections[0].Group].Origins...)
	last := sections[len(sections)-1]
	groups[startGroup] = facetedrecord.Group{
		Origins: append(startOrigins, provenance.Ref{Producer: ref, Role: provenance.RoleCapStart}),
		Surface: sections[0].Surface, Planar: true, Reversed: true, NormalBound: 2}
	if last.Constructed != nil {
		plane, err := constructedPlane(*last.Constructed)
		if err != nil {
			return facetedrecord.Payload{}, err
		}
		groups[endGroup] = facetedrecord.Group{Origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Surface: plane, Planar: true, NormalBound: 2}
	} else {
		endOrigins := append([]provenance.Ref(nil), last.Mesh.Groups[last.Group].Origins...)
		groups[endGroup] = facetedrecord.Group{
			Origins: append(endOrigins, provenance.Ref{Producer: ref, Role: provenance.RoleCapEnd}),
			Surface: last.Surface, Planar: true, NormalBound: 2}
	}
	for _, capTri := range base.Tris[n:] {
		if capTri[0] == 0 || capTri[1] == 0 || capTri[2] == 0 {
			return facetedrecord.Payload{}, fmt.Errorf("%w: LoftFaces source cap includes the apex", decaderr.ErrUnsupported)
		}
		tris = append(tris,
			[3]int{capTri[0] - 1, capTri[2] - 1, capTri[1] - 1},
			[3]int{(len(sections)-1)*n + capTri[0] - 1, (len(sections)-1)*n + capTri[1] - 1,
				(len(sections)-1)*n + capTri[2] - 1})
		src = append(src, startGroup, endGroup)
	}
	return facetedrecord.Payload{Verts: verts, Tris: tris, Src: src, Groups: groups, Xform: r3.Identity()}, nil
}

// constructedPlane returns the rounded Plane tag of a constructed planar end
// map. The tag is for reporting only; the exact map is the proof.
func constructedPlane(m loftface.Map) (surfacegeom.Plane, error) {
	u, v := loftface.RoundPoint(m.QU), loftface.RoundPoint(m.QV)
	frame, err := r3.NewFrame(loftface.RoundPoint(m.Constant), u, v)
	if err != nil {
		return surfacegeom.Plane{}, fmt.Errorf("%w: the constructed loft end has no plane frame: %s",
			decaderr.ErrUnsupported, err)
	}
	return surfacegeom.Plane{Frame: frame}, nil
}
