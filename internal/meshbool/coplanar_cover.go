package meshbool

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// keepOptions carries the coplanar side evidence one keepSideWith pass reads
// (agent-docs/coplanar-contact-design.md §3.4). The zero value is the plain
// pass.
type keepOptions struct {
	// Coplanar maps each facet of m to the facets of the other operand that
	// form an admitted coplanar pair with it.
	Coplanar map[int][]int
	// Op is the boolean operation, and Second says m is its second operand.
	Op     OperationKind
	Second bool
}

// ContactKeys is what the stitch needs to tell a contact pinch from a broken
// mesh (agent-docs/coplanar-contact-design.md §5).
type ContactKeys struct {
	// Owned holds the exact key of every point the coplanar pass accepted.
	Owned map[string]struct{}
	// Legacy is the refusal the pair pass returns with the switch off.
	Legacy error
}

// coplanarKeep is §3.4's table: whether a region covered by coincident
// patches keeps its copy from operand A or B. Same facing keeps A's copy for
// Union and Intersect and no copy for Cut; opposite facing keeps no copy for
// Union and Intersect and A's copy for Cut. B's copy is never kept, which makes
// the choice independent of facet order and of Cut flipping B's facets.
func coplanarKeep(op OperationKind, second, sameFacing bool) bool {
	if second {
		return false
	}
	switch op {
	case OpUnion, OpIntersect:
		return sameFacing
	case OpCut:
		return !sameFacing
	default:
		return false
	}
}

// coverOf returns the facets of partners whose closed triangle contains the
// exact probe p, a point of the plane every partner shares. A probe on a
// patch-boundary edge of a partner is ErrBooleanFailed: those edges were
// inserted as cuts, so no region's interior touches them. A probe on a
// triangulation diagonal of the partner's patch lies in both facets sharing it,
// and both are partners, so both are covers.
func coverOf(other *BoolMesh, partners []int, p proof.Xpt) ([]int, error) {
	var cover []int
	for _, j := range partners {
		xt := XtriCorners(other, j)
		nb := other.Norms[j]
		if !PointOnTri(p, xt, nb) {
			continue
		}
		for k := range 3 {
			a, b := xt[k], xt[(k+1)%3]
			if PlaneSide(a, b, p, nb) != 0 || !PointOnSegment3D(a, b, p) {
				continue
			}
			boundary, err := patchBoundaryEdge(other, j, k)
			if err != nil {
				return nil, err
			}
			if boundary {
				return nil, fmt.Errorf(`%w: a region probe lies on a coplanar patch boundary`, decaderr.ErrBooleanFailed)
			}
		}
		cover = append(cover, j)
	}
	return cover, nil
}

// coverFacing reads the facing of facet i of m against every covering facet of
// other: same is true when the exact normals point the same way. Covers that
// disagree are ErrUnclassifiableContact.
func coverFacing(m *BoolMesh, i int, other *BoolMesh, cover []int) (bool, error) {
	same := false
	for k, j := range cover {
		s := proof.XdotSign(m.Norms[i], other.Norms[j]) > 0
		if k > 0 && s != same {
			return false, ErrUnclassifiableContact(`mixed coplanar side evidence`)
		}
		same = s
	}
	return same, nil
}

// subdivideBatch is subdivideCutFacets for facets that may have no contact
// segment: a facet with only coplanar partners is one region, the facet
// itself with its centroid as probe, and CutTriangle is not called for it.
func subdivideBatch(ctx context.Context, m *BoolMesh, cuts map[int][]Xseg, facets []int) ([]cutFacetResult, error) {
	var cut []int
	for _, i := range facets {
		if len(cuts[i]) > 0 {
			cut = append(cut, i)
		}
	}
	if len(cut) == len(facets) {
		return subdivideCutFacets(ctx, m, cuts, facets)
	}
	cutResults, err := subdivideCutFacets(ctx, m, cuts, cut)
	if err != nil {
		return nil, err
	}
	results := make([]cutFacetResult, len(facets))
	next := 0
	for slot, i := range facets {
		if len(cuts[i]) > 0 {
			results[slot] = cutResults[next]
			next++
			continue
		}
		x := XtriCorners(m, i)
		results[slot] = cutFacetResult{regions: []CutRegion{{Tris: [][3]proof.Xpt{x}, Probe: XCentroid(x[0], x[1], x[2])}}}
	}
	return results, nil
}
