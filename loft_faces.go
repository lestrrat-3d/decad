package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

const maxLoftFacesSections = 32

// LoftFaces builds one solid through ordered complete caps of the same
// authenticated point-loft Split source (docs/loft-face-sections-design.md).
func LoftFaces(ctx context.Context, d *Document, sections ...*Face) (*Body, error) {
	if ctx == nil || d == nil || len(sections) < 2 || len(sections) > maxLoftFacesSections {
		return nil, fmt.Errorf("%w: LoftFaces needs a context, document and 2 to %d cap faces",
			ErrDegenerate, maxLoftFacesSections)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	selected, source, err := collectLoftFaces(d, sections)
	if err != nil {
		return nil, err
	}
	return buildLoftFaces(ctx, d, selected, source, nil)
}

// buildLoftFaces lofts through the collected sections and commits the body.
// A non-nil tangent means the last section is the constructed cone-tangent
// end described by tangent; every earlier section is a selected cap.
func buildLoftFaces(ctx context.Context, d *Document, selected []loftFaceSection,
	source *pointSectionRecord, tangent *loftface.TangentEnd) (*Body, error) {
	ref := d.nextProducerID()
	mesh, err := assembleLoftFaces(ctx, ref, selected, source)
	if err != nil {
		return nil, err
	}
	embedding, err := proveLoftFaceEmbedding(ctx, selected, source)
	if err != nil {
		return nil, err
	}
	if layout, ok := loftmesh.LoftFacesCapLayout(selected[0].slice.Base.Tris[:len(selected[0].ring)],
		len(selected)-1, len(mesh.Tris)); ok {
		err = loftmesh.LoftCrossingAuditWithCaps(ctx, mesh.Verts, mesh.Tris, layout)
	} else {
		err = loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris)
	}
	if err != nil {
		return nil, err
	}
	if err := orientLoftFaces(&mesh, selected[0].slice.Base, source, embedding.sign); err != nil {
		return nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.Verts, mesh.Tris)
	if err != nil {
		return nil, err
	}
	_, heldRat, err := meshVolumeMeasurement(ctx, audit, 0)
	if err != nil {
		return nil, err
	}
	boundSections := make([]revolveloftfaces.BoundsSection, len(selected))
	for i, section := range selected {
		boundSections[i] = revolveloftfaces.BoundsSection{
			Ring: section.ring, BaseMeshBound: section.slice.Base.MeshBound,
		}
	}
	mesh, err = revolveloftfaces.ProveBounds(ctx, boundSections, source,
		embedding.maps, embedding.jacobianSum, mesh, selected[0].slice.Base)
	if err != nil {
		return nil, err
	}
	capBounds := make([]loftFaceCapBounds, len(selected))
	for i := range selected {
		capBounds[i], err = revolveloftfaces.CapBounds(ctx, embedding.maps[i], boundSections[i], source)
		if err != nil {
			return nil, err
		}
	}
	if facetedrecord.VolFloor(heldRat, mesh.VolSymDiff) <= 0 {
		return nil, fmt.Errorf("%w: face-fed loft occupied-volume proof exceeds the held volume", ErrUnsupported)
	}
	if len(selected[0].slice.Base.Src) < len(source.SourceStation) {
		return nil, fmt.Errorf("%w: loft source cell roles are incomplete", ErrUnsupported)
	}
	mesh.LoftFacesPrimitive = true
	mesh.LoftFacesCap = &loftFacesCapRecord{Source: source,
		Maps:         append([]loftFaceMap(nil), embedding.maps...),
		Bounds:       capBounds,
		StationGroup: append([]int(nil), selected[0].slice.Base.Src[:len(source.SourceStation)]...),
		Final:        embedding.maps[len(embedding.maps)-1], Group: len(mesh.Groups) - 1,
		Tangent: tangent}
	body, err := bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(d), ref), mesh, audit, measurementToInternal(Measurement{}), nil))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

type loftFaceSection struct {
	face  *Face
	body  *Body
	mesh  facetedPayload
	slice *pointPlaneSliceRecord
	limit *pointPlaneLimit
	group int
	ring  []r3.Vec
	// constructed is the exact cap map of a section that no selected cap
	// supplies (the cone-tangent end). Then face, body, mesh and limit are
	// unset, and slice is the first selected section's, read only for its
	// base mesh bound.
	constructed *loftFaceMap
}

func collectLoftFaces(d *Document, faces []*Face) ([]loftFaceSection, *pointSectionRecord, error) {
	sections := make([]loftFaceSection, len(faces))
	var source *pointSectionRecord
	seen := make(map[*Body]bool, len(faces)-1)
	for i, face := range faces {
		if face == nil || face.n.Body == nil {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not a body face", ErrDegenerate, i)
		}
		body := bodyHandle(face.n.Body)
		if err := d.requireLive(body); err != nil {
			return nil, nil, err
		}
		fp, ok := body.n.Payload.(facetedPayload)
		if !ok || fp.PointPlane == nil || fp.PointPlane.Base.PointSection == nil ||
			len(fp.Verts) == 0 || len(fp.FaceOf) != len(fp.Tris) {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has no authenticated Split cap", ErrUnsupported, i)
		}
		record := fp.PointPlane.Base.PointSection
		if !facetedrecord.PointSourceCurrent(record) {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has a stale source profile", ErrStaleProfile, i)
		}
		if source == nil {
			source = record
		} else if source != record {
			return nil, nil, fmt.Errorf("%w: LoftFaces sections have different point-loft sources", ErrUnsupported)
		}
		if fp.PointSimilarity != nil && fp.PointSimilarity.Slice != fp.PointPlane {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has broken similarity provenance", ErrUnsupported, i)
		}
		faceIndex := -1
		for j, candidate := range body.Faces() {
			if candidate == face {
				faceIndex = j
				break
			}
		}
		if faceIndex < 0 {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not in its body", ErrUnsupported, i)
		}
		group := -1
		for j, owner := range fp.FaceOf {
			if owner == faceIndex {
				group = fp.Src[j]
				break
			}
		}
		want := len(fp.Groups) - 1
		if i == 0 {
			want--
		}
		if group != want || (i == 0 && fp.PointPlane.Lower == nil) {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not the required Split cap", ErrUnsupported, i)
		}
		if complete, ok := pointCapGroupFace(body, fp, want); !ok || complete != face {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not one complete cap patch", ErrUnsupported, i)
		}
		if i == 1 && body != bodyHandle(faces[0].n.Body) {
			return nil, nil, fmt.Errorf("%w: LoftFaces first toe and heel must share one slab", ErrUnsupported)
		}
		if i >= 1 {
			if seen[body] {
				return nil, nil, fmt.Errorf("%w: LoftFaces repeats a heel slab", ErrUnsupported)
			}
			seen[body] = true
		}
		n := len(fp.PointPlane.Base.Verts) - 1
		if n < 3 || len(fp.Verts) != 2*n || len(fp.PointPlane.Base.Tris) <= n {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has no complete station ring", ErrUnsupported, i)
		}
		ring := fp.Verts[n:]
		limit := fp.PointPlane.Upper
		if i == 0 {
			ring = fp.Verts[:n]
			limit = fp.PointPlane.Lower
		}
		sections[i] = loftFaceSection{face: face, body: body, mesh: fp,
			slice: fp.PointPlane, limit: limit, group: group, ring: ring}
	}
	return sections, source, nil
}

func assembleLoftFaces(ctx context.Context, ref producerID,
	sections []loftFaceSection, source *pointSectionRecord) (facetedPayload, error) {
	n := len(sections[0].ring)
	base := sections[0].slice.Base
	segmentCount := len(source.Profile.Outer.Segments)
	if len(base.Src) < n || len(base.Tris) < n || len(base.Groups) != segmentCount+1 {
		return facetedPayload{}, fmt.Errorf("%w: LoftFaces source fan has no complete side correspondence", ErrUnsupported)
	}
	verts := make([]r3.Vec, 0, len(sections)*n)
	for i, section := range sections {
		if err := ctx.Err(); err != nil {
			return facetedPayload{}, err
		}
		if len(section.ring) != n {
			return facetedPayload{}, fmt.Errorf("%w: LoftFaces section %d has different stations", ErrUnsupported, i)
		}
		verts = append(verts, section.ring...)
	}
	tris := make([][3]int, 0, 2*n*(len(sections)-1)+2*len(base.Tris[n:]))
	src := make([]int, 0, cap(tris))
	groups := make([]facetGroup, (len(sections)-1)*segmentCount+2)
	for band := range len(sections) - 1 {
		for segment := range segmentCount {
			groups[band*segmentCount+segment] = facetGroup{Origins: []provenance.Ref{{Producer: ref,
				Role: fmt.Sprintf("side(%d,%d)", band, segment)}}}
		}
		for j := range n {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return facetedPayload{}, err
				}
			}
			side := base.Tris[j]
			if side[0] != 0 || side[1] <= 0 || side[2] <= 0 {
				return facetedPayload{}, fmt.Errorf("%w: LoftFaces source fan has no ordered side", ErrUnsupported)
			}
			b, c := side[1]-1, side[2]-1
			group := base.Src[j]
			if group < 0 || group >= segmentCount {
				return facetedPayload{}, fmt.Errorf("%w: LoftFaces source side has no segment", ErrUnsupported)
			}
			lo, hi := band*n, (band+1)*n
			tris = append(tris, [3]int{lo + b, hi + b, hi + c}, [3]int{lo + b, hi + c, lo + c})
			src = append(src, band*segmentCount+group, band*segmentCount+group)
		}
	}
	startGroup, endGroup := len(groups)-2, len(groups)-1
	startOrigins := append([]provenance.Ref(nil), sections[0].mesh.Groups[sections[0].group].Origins...)
	last := sections[len(sections)-1]
	groups[startGroup] = facetGroup{Origins: append(startOrigins, provenance.Ref{Producer: ref, Role: provenance.RoleCapStart}),
		Surface: sections[0].face.n.Surface, Planar: true, Reversed: true, NormalBound: 2}
	if last.constructed != nil {
		plane, err := loftFacesConstructedPlane(*last.constructed)
		if err != nil {
			return facetedPayload{}, err
		}
		groups[endGroup] = facetGroup{Origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Surface: plane, Planar: true, NormalBound: 2}
	} else {
		endOrigins := append([]provenance.Ref(nil), last.mesh.Groups[last.group].Origins...)
		groups[endGroup] = facetGroup{Origins: append(endOrigins, provenance.Ref{Producer: ref, Role: provenance.RoleCapEnd}),
			Surface: last.face.n.Surface, Planar: true, NormalBound: 2}
	}
	for _, capTri := range base.Tris[n:] {
		if capTri[0] == 0 || capTri[1] == 0 || capTri[2] == 0 {
			return facetedPayload{}, fmt.Errorf("%w: LoftFaces source cap includes the apex", ErrUnsupported)
		}
		tris = append(tris,
			[3]int{capTri[0] - 1, capTri[2] - 1, capTri[1] - 1},
			[3]int{(len(sections)-1)*n + capTri[0] - 1, (len(sections)-1)*n + capTri[1] - 1,
				(len(sections)-1)*n + capTri[2] - 1})
		src = append(src, startGroup, endGroup)
	}
	return facetedPayload{Verts: verts, Tris: tris, Src: src, Groups: groups, Xform: r3.Identity()}, nil
}

// loftFacesConstructedPlane returns the rounded Plane tag of a constructed
// planar end map. The tag is for reporting only; the exact map is the proof.
func loftFacesConstructedPlane(m loftFaceMap) (surfacegeom.Plane, error) {
	u, v := loftface.RoundPoint(m.QU), loftface.RoundPoint(m.QV)
	frame, err := r3.NewFrame(loftface.RoundPoint(m.Constant), u, v)
	if err != nil {
		return surfacegeom.Plane{}, fmt.Errorf("%w: the constructed loft end has no plane frame: %s", ErrUnsupported, err)
	}
	return surfacegeom.Plane{Frame: frame}, nil
}
