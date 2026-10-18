package facetedrecord

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/shellseparation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// FacetedCopyLineage records the source of a faceted body's rigid image.
// Copy issues it after rebuilding the held mesh. A Boolean result does not
// inherit it. The source must still be live when a copied-tool proof uses it.
type FacetedCopyLineage struct {
	Source          *topo.Body
	SourceProducer  provenance.Producer
	SourcePlacement r3.Transform
	TargetPlacement r3.Transform
	Motion          r3.Transform
	// Direct is true only when placed applied motion to the source payload
	// itself, so the image's true solid is exactly motion of the source's.
	Direct bool
	// Exact is the rational rotation the image denotes, set only for a direct
	// copy made with WithCopyRotation. A Transform, a copy of a copy and every
	// ordinary placement leave it nil.
	Exact *exactrotation.Rotation
}

// FacetedCopySet binds a source and its live rigid copies to their actual
// held shells and one-to-one face owners. The shell proof establishes that
// source solids and held shells are pairwise separate; it says nothing about
// their contact with a separate Union blank.
type FacetedCopySet struct {
	Source *topo.Body
	Tools  []*topo.Body
	Moves  []r3.Transform
	// Exact holds each copy's exact rotation record (index 0 and every ordinary
	// copy nil). Maps holds the matching true linear maps for shellseparation:
	// the record's rotation where one exists, else nil for the float move.
	Exact     []*exactrotation.Rotation
	Maps      [][3][3]*big.Rat
	Axis      shellseparation.Axis
	Held      []shellseparation.Shell
	FaceOwner []map[*topo.Face]*topo.Face // source face to the corresponding live copy face
}

// CopyFaceOwners matches each live source face to one copied face while
// preserving the source and image body face order and provenance.
func CopyFaceOwners(source, image *topo.Body, first, placed Payload) (map[*topo.Face]*topo.Face, bool) {
	if len(first.Tris) != len(placed.Tris) || len(first.Src) != len(first.Tris) ||
		len(placed.Src) != len(placed.Tris) || len(first.FaceOf) != len(first.Tris) ||
		len(placed.FaceOf) != len(placed.Tris) || len(first.Verts) != len(placed.Verts) {
		return nil, false
	}
	sourceFaces, imageFaces := topobuild.Faces(source), topobuild.Faces(image)
	owners := make(map[*topo.Face]*topo.Face, len(sourceFaces))
	reverse := make(map[*topo.Face]*topo.Face, len(imageFaces))
	for i, triangle := range first.Tris {
		want := triangle
		if placed.CopyLineage.Motion.IsReflection() {
			want[1], want[2] = want[2], want[1]
		}
		if placed.Tris[i] != want || first.Src[i] != placed.Src[i] ||
			first.FaceOf[i] < 0 || first.FaceOf[i] >= len(sourceFaces) ||
			placed.FaceOf[i] < 0 || placed.FaceOf[i] >= len(imageFaces) {
			return nil, false
		}
		original, copyFace := sourceFaces[first.FaceOf[i]], imageFaces[placed.FaceOf[i]]
		if owner := owners[original]; owner != nil && owner != copyFace {
			return nil, false
		}
		if prior := reverse[copyFace]; prior != nil && prior != original {
			return nil, false
		}
		owners[original], reverse[copyFace] = copyFace, original
	}
	if len(owners) != len(sourceFaces) || len(reverse) != len(imageFaces) {
		return nil, false
	}
	for original, copyFace := range owners {
		copyOrigins := append([]provenance.Ref(nil), copyFace.Origins...)
		for _, sourceOrigin := range append([]provenance.Ref(nil), original.Origins...) {
			if !slices.Contains(copyOrigins, sourceOrigin) {
				return nil, false
			}
		}
	}
	return owners, true
}
