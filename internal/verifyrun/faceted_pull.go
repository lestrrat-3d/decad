package verifyrun

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshthick"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// facetedUndercuts reads the undercuts of a faceted body against a pull from
// rays through its held mesh (agent-docs/mesh-wall-survey-design.md §6). A face
// holding a triangle whose release ray is blocked by the body is a proven
// undercut. The rays sample one point per triangle, so they cannot show a face
// free: a body with a blocked face lists those faces and stays undecided, and a
// body with none is undecided with nothing listed.
func facetedUndercuts(budget *proofbound.WorkBudget, b *topo.Body, pl facetedrecord.Payload, pull r3.Vec) (undercutOutcome, error) {
	dir, ok := pull.Normalize()
	if !ok || len(pl.FaceOf) != len(pl.Tris) {
		return undercutOutcome{reason: surveyFacetedUnsupported}, nil
	}
	tree, err := meshthick.NewTree(budget, pl.Verts, pl.Tris)
	if err != nil || tree.Empty() {
		return undercutOutcome{reason: surveyFacetedUnsupported}, err
	}
	clearance := 2*pl.MeshBound + facetedWallSlack*math.Max(pl.Diameter, tree.Diameter())
	blocked, err := meshthick.Undercuts(budget, tree, dir, clearance)
	if err != nil {
		return undercutOutcome{}, err
	}
	faces := topobuild.Faces(b)
	var listed []*topo.Face
	seen := map[int]struct{}{}
	for _, tri := range blocked {
		fi := pl.FaceOf[tri]
		if _, dup := seen[fi]; dup || fi < 0 || fi >= len(faces) {
			continue
		}
		seen[fi] = struct{}{}
		listed = append(listed, faces[fi])
	}
	if len(listed) == 0 {
		return undercutOutcome{}, nil
	}
	return undercutOutcome{faces: listed, ok: true, undecided: true}, nil
}
