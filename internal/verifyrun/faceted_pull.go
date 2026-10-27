package verifyrun

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshthick"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
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
	if out, ok := orthogonalUndercuts(b, pl, dir); ok {
		return out, nil
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

// orthogonalUndercuts reads the undercuts of a faceted body every face of which squares to a world axis, with no vertex
// bound, against a pull along that axis, exactly (agent-docs/orthogonal-wall-design.md): a face square to the pull is
// blocked when material lies beyond it on any line over it, tested on the solid's grid, so the survey lists the blocked
// faces and, when there is none, proves the pull free. ok is false when the body is outside that class or the pull is
// not exactly a world axis, and the rays take it.
func orthogonalUndercuts(b *topo.Body, pl facetedrecord.Payload, dir r3.Vec) (undercutOutcome, bool) {
	for _, beta := range pl.VertexBound {
		if beta != 0 {
			return undercutOutcome{}, false
		}
	}
	axis := -1
	for k, c := range [3]float64{dir.X, dir.Y, dir.Z} {
		switch c {
		case 0:
		case 1, -1:
			if axis >= 0 {
				return undercutOutcome{}, false
			}
			axis = k
		default:
			return undercutOutcome{}, false
		}
	}
	if axis < 0 {
		return undercutOutcome{}, false
	}
	blocked, err := orthoshell.Undercuts(pl.Verts, pl.Tris, axis)
	if err != nil {
		return undercutOutcome{}, false
	}
	faces := topobuild.Faces(b)
	listed := []*topo.Face{} // non-nil: every face is decided
	seen := map[int]struct{}{}
	for tri, isBlocked := range blocked {
		if !isBlocked {
			continue
		}
		fi := pl.FaceOf[tri]
		if _, dup := seen[fi]; dup || fi < 0 || fi >= len(faces) {
			continue
		}
		seen[fi] = struct{}{}
		listed = append(listed, faces[fi])
	}
	return undercutOutcome{faces: listed, ok: true}, true
}
