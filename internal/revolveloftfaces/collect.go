package revolveloftfaces

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// Collect authenticates the cap faces LoftFaces lofts through and returns one
// section per face plus the point-loft source they share. live is the owning
// document's liveness gate for a section's body; it runs before that section's
// other gates.
func Collect(faces []*topo.Face, live func(*topo.Body) error) ([]LoftFacesSection, *facetedrecord.PointSectionRecord, error) {
	sections := make([]LoftFacesSection, len(faces))
	var source *facetedrecord.PointSectionRecord
	seen := make(map[*topo.Body]bool, len(faces)-1)
	for i, face := range faces {
		if face == nil || face.Body == nil {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not a body face", decaderr.ErrDegenerate, i)
		}
		body := face.Body
		if err := live(body); err != nil {
			return nil, nil, err
		}
		fp, ok := body.Payload.(facetedrecord.Payload)
		if !ok || fp.PointPlane == nil || fp.PointPlane.Base.PointSection == nil ||
			len(fp.Verts) == 0 || len(fp.FaceOf) != len(fp.Tris) {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has no authenticated Split cap", decaderr.ErrUnsupported, i)
		}
		record := fp.PointPlane.Base.PointSection
		if !facetedrecord.PointSourceCurrent(record) {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has a stale source profile", decaderr.ErrStaleProfile, i)
		}
		if source == nil {
			source = record
		} else if source != record {
			return nil, nil, fmt.Errorf("%w: LoftFaces sections have different point-loft sources", decaderr.ErrUnsupported)
		}
		if fp.PointSimilarity != nil && fp.PointSimilarity.Slice != fp.PointPlane {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has broken similarity provenance", decaderr.ErrUnsupported, i)
		}
		bodyFaces := topobuild.Faces(body)
		faceIndex := -1
		for j, candidate := range bodyFaces {
			if candidate == face {
				faceIndex = j
				break
			}
		}
		if faceIndex < 0 {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not in its body", decaderr.ErrUnsupported, i)
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
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not the required Split cap", decaderr.ErrUnsupported, i)
		}
		index, ok := facetedtopology.GroupFace(fp.Src, fp.FaceOf, len(bodyFaces), want)
		if !ok || bodyFaces[index] != face {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d is not one complete cap patch", decaderr.ErrUnsupported, i)
		}
		if i == 1 && body != faces[0].Body {
			return nil, nil, fmt.Errorf("%w: LoftFaces first toe and heel must share one slab", decaderr.ErrUnsupported)
		}
		if i >= 1 {
			if seen[body] {
				return nil, nil, fmt.Errorf("%w: LoftFaces repeats a heel slab", decaderr.ErrUnsupported)
			}
			seen[body] = true
		}
		n := len(fp.PointPlane.Base.Verts) - 1
		if n < 3 || len(fp.Verts) != 2*n || len(fp.PointPlane.Base.Tris) <= n {
			return nil, nil, fmt.Errorf("%w: LoftFaces section %d has no complete station ring", decaderr.ErrUnsupported, i)
		}
		ring := fp.Verts[n:]
		limit := fp.PointPlane.Upper
		if i == 0 {
			ring = fp.Verts[:n]
			limit = fp.PointPlane.Lower
		}
		sections[i] = LoftFacesSection{Mesh: fp, Slice: fp.PointPlane, Limit: limit,
			Group: group, Ring: ring, Surface: face.Surface}
	}
	return sections, source, nil
}
