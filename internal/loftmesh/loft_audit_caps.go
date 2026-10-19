package loftmesh

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// LoftCapLayout is a caller's own record of which triangles of an assembled
// mesh are walls and which are caps, read from the index structure it built,
// never inferred from geometry.
type LoftCapLayout struct {
	Walls    int     // tris[:Walls] are the wall triangles
	Cap0     []int   // indices into tris of the first cap's triangles, each >= Walls
	Cap1     []int   // indices of the second cap's triangles; empty for an apex fan
	Loops0   [][]int // vertex-index cycles bounding Cap0
	Loops1   [][]int // cycles bounding Cap1, or [][]int{{apex}} for an apex fan
	Interior []int   // vertices of interior section rings (LoftAuditStructure.Interior)
}

// LoftCrossingAuditWithCaps returns what
// LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris) returns,
// provided ctx is not cancelled during the call: then the verdict and the
// refusal text are a direct generic call's. When the layout is well formed,
// at least one cap is exactly planar and the mesh is small enough that the
// generic audit cannot reach its streamed row preflight or any S8 ceiling
// (capsAuditFits), it first runs the structured audit over the triangles
// reordered as walls, Cap0, Cap1, so each planar cap is decided by
// CapFamilyProof. A structured pass that succeeds is the generic audit's
// verdict; any error reruns the generic audit on the same budget. Under
// cancellation the result may be the cancellation error in place of a
// geometric refusal: a context that expires during the structured pass makes
// the rerun's entry poll report it, where a direct generic call could have
// reached its refusal first. The audit never accepts a mesh the generic audit
// refuses.
func LoftCrossingAuditWithCaps(ctx context.Context, verts []r3.Vec, tris [][3]int, layout LoftCapLayout) error {
	_, _, err := LoftCrossingAuditWithCapsWork(ctx, verts, tris, layout)
	return err
}

// LoftCrossingAuditWithCapsWork is LoftCrossingAuditWithCaps with the
// structured pass's work, for tests. structured is false when the call took
// the generic audit directly.
func LoftCrossingAuditWithCapsWork(ctx context.Context, verts []r3.Vec, tris [][3]int,
	layout LoftCapLayout) (LoftAuditWork, bool, error) {
	return loftCrossingAuditWithCaps(proofbound.NewWorkBudget(ctx), verts, tris, layout)
}

// loftCrossingAuditWithCaps runs both passes on one budget, so the generic
// rerun after a structured pass that stopped on cancellation sees that
// cancellation the way a direct generic call would: a context stays
// cancelled, and the rerun never starts from a fresh, uncancelled count.
func loftCrossingAuditWithCaps(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	layout LoftCapLayout) (LoftAuditWork, bool, error) {
	generic := func() error {
		_, err := loftCrossingAuditLimited(budget, verts, tris, nil, loftGenericShortcuts,
			proofbound.MaxFacetPairTestsPerCall, MaxLoftAuditTriangles, true)
		return err
	}
	if !capsAuditFits(len(tris)) {
		return LoftAuditWork{}, false, generic()
	}
	order, ok := capsReorder(len(verts), tris, layout)
	if !ok || !capsPlanar(verts, order, layout) {
		return LoftAuditWork{}, false, generic()
	}
	structure := LoftAuditStructure{Walls: layout.Walls, CapStartCount: len(layout.Cap0),
		Loops0: layout.Loops0, Loops1: layout.Loops1, Interior: layout.Interior}
	work, err := loftCrossingAuditLimited(budget, verts, order, &structure,
		loftStructuredShortcuts, proofbound.MaxFacetPairTestsPerCall, MaxLoftAuditTriangles, true)
	if err == nil {
		return work, true, nil
	}
	return work, true, generic()
}

// capsAuditFits reports that f triangles have at most
// loftAuditCandidateBatchSize pairs. The generic audit streams its rows, and
// runs the row preflight that charges tree nodes beside pairs, only for more
// candidates than that, and its candidate count and enumeration work are each
// at most C(f, 2), far below the S8 ceiling. So for such a mesh the generic
// audit has no resource refusal the structured pass could skip.
func capsAuditFits(f int) bool {
	if f < 2 {
		return true
	}
	n := uint64(f)
	return n*(n-1)/2 <= loftAuditCandidateBatchSize
}

// capsReorder validates the layout and returns a private copy of tris ordered
// as walls, Cap0, Cap1. ok is false unless every triangle vertex index is
// below vertexCount and the layout names every triangle once.
func capsReorder(vertexCount int, tris [][3]int, layout LoftCapLayout) ([][3]int, bool) {
	for _, tri := range tris {
		for _, v := range tri {
			if v < 0 || v >= vertexCount {
				return nil, false
			}
		}
	}
	f := len(tris)
	walls := layout.Walls
	if walls < 0 || walls > f || walls+len(layout.Cap0)+len(layout.Cap1) != f {
		return nil, false
	}
	order := make([][3]int, 0, f)
	order = append(order, tris[:walls]...)
	seen := make([]bool, f)
	for _, capIndices := range [2][]int{layout.Cap0, layout.Cap1} {
		for _, t := range capIndices {
			if t < walls || t >= f || seen[t] {
				return nil, false
			}
			seen[t] = true
			order = append(order, tris[t])
		}
	}
	return order, true
}

// capsPlanar reports that Cap0 or Cap1 (the triangles of order after the
// walls) is exactly planar. It only routes: CapFamilyProof checks (a) again,
// and a mesh whose caps are not exactly planar (an oblique sketch plane
// rounds its cap vertices off the plane) stays on the generic audit, which
// may stream and run in parallel for large meshes.
func capsPlanar(verts []r3.Vec, order [][3]int, layout LoftCapLayout) bool {
	first := layout.Walls
	return capExactlyPlanar(verts, order[first:first+len(layout.Cap0)]) ||
		capExactlyPlanar(verts, order[first+len(layout.Cap0):])
}

// capExactlyPlanar reports that tris is non-empty and every vertex of every
// triangle has exact sign 0 against the plane of tris[0]. A zero normal is
// not planar.
func capExactlyPlanar(verts []r3.Vec, tris [][3]int) bool {
	if len(tris) == 0 {
		return false
	}
	a, b, c := proof.XptOf(verts[tris[0][0]]), proof.XptOf(verts[tris[0][1]]), proof.XptOf(verts[tris[0][2]])
	n := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
	if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
		return false
	}
	plane := NewLoftExactPlane(a, n)
	var sum, term big.Int
	for _, tri := range tris {
		for _, v := range tri {
			if plane.Sign(proof.XptOf(verts[v]), &sum, &term) != 0 {
				return false
			}
		}
	}
	return true
}

// FanRing returns the ring order of an apex fan's far stations: sides are the
// fan's n side triangles (0, b, c) with 1 <= b, c <= n, and the result lists
// b-1 values from 0 following each side's b -> c. ok is false unless the
// sides form one cycle through all n stations.
func FanRing(sides [][3]int) ([]int, bool) {
	n := len(sides)
	if n < 3 {
		return nil, false
	}
	next := make([]int, n)
	for i := range next {
		next[i] = -1
	}
	for _, side := range sides {
		b, c := side[1], side[2]
		if side[0] != 0 || b < 1 || b > n || c < 1 || c > n || b == c || next[b-1] != -1 {
			return nil, false
		}
		next[b-1] = c - 1
	}
	ring := make([]int, 0, n)
	seen := make([]bool, n)
	v := 0
	for range n {
		if v < 0 || seen[v] {
			return nil, false
		}
		seen[v] = true
		ring = append(ring, v)
		v = next[v]
	}
	if v != 0 {
		return nil, false
	}
	return ring, true
}

// PointPlaneSlabCapLayout records the wall and cap triangle order emitted by
// a point-plane slab: n apex walls or 2n walls between two planes, followed by
// far-cap triangles interleaved between upper and lower caps when both exist.
func PointPlaneSlabCapLayout(sides [][3]int, triCount, upperOffset int, hasApex bool) (LoftCapLayout, bool) {
	ring, ok := FanRing(sides)
	if !ok {
		return LoftCapLayout{}, false
	}
	n := len(sides)
	shift := func(offset int) []int {
		out := make([]int, len(ring))
		for i, r := range ring {
			out[i] = offset + r
		}
		return out
	}
	if hasApex {
		if triCount < n {
			return LoftCapLayout{}, false
		}
		layout := LoftCapLayout{Walls: n, Loops0: [][]int{shift(upperOffset)}, Loops1: [][]int{{0}}}
		for t := n; t < triCount; t++ {
			layout.Cap0 = append(layout.Cap0, t)
		}
		return layout, true
	}
	if triCount < 2*n || (triCount-2*n)%2 != 0 {
		return LoftCapLayout{}, false
	}
	layout := LoftCapLayout{Walls: 2 * n, Loops0: [][]int{shift(upperOffset)}, Loops1: [][]int{shift(0)}}
	for t := 2 * n; t < triCount; t += 2 {
		layout.Cap0 = append(layout.Cap0, t)
		layout.Cap1 = append(layout.Cap1, t+1)
	}
	return layout, true
}

// LoftFacesCapLayout is the wall and cap structure
// revolveloftfaces.BuildLoftFaces assembles: 2n walls per band, then
// start/end cap triangles interleaved. The start ring is
// vertices [0, n), the end ring [bands*n, (bands+1)*n), and the rings between
// them are Interior. sides are the source fan's side triangles.
func LoftFacesCapLayout(sides [][3]int, bands, triCount int) (LoftCapLayout, bool) {
	ring, ok := FanRing(sides)
	if !ok || bands < 1 {
		return LoftCapLayout{}, false
	}
	n := len(sides)
	walls := 2 * n * bands
	if triCount < walls || (triCount-walls)%2 != 0 {
		return LoftCapLayout{}, false
	}
	shift := func(offset int) []int {
		out := make([]int, len(ring))
		for i, r := range ring {
			out[i] = offset + r
		}
		return out
	}
	layout := LoftCapLayout{Walls: walls,
		Loops0: [][]int{shift(0)}, Loops1: [][]int{shift(bands * n)}}
	for t := walls; t < triCount; t += 2 {
		layout.Cap0 = append(layout.Cap0, t)
		layout.Cap1 = append(layout.Cap1, t+1)
	}
	for v := n; v < bands*n; v++ {
		layout.Interior = append(layout.Interior, v)
	}
	return layout, true
}
