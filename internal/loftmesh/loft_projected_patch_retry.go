package loftmesh

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// projectedPatchRetryMaxInverted caps the triangles whose projected sign
// differs from the patch's majority sign that a retry accepts. A cone cap of a
// straight-bevel sector join has a handful of inverted slivers at its root
// seam (4 to 9); a patch with many more is folded along a run, not at a seam,
// and its reduced patch would not prove, so the retry is not tried.
const projectedPatchRetryMaxInverted = 32

// provenProjectedPatch returns the triangle family of patch that the audit may
// treat as one projected graph, and reports whether there is one.
//
// It first runs the projected-patch proof on patch. When that fails only on
// orientation, it runs the same proof once more on a reduced patch: patch
// without its inverted triangles (those whose projected sign is not the
// strict majority sign, zero included) and without every triangle that shares
// a vertex with one of them (one vertex ring). Only a patch whose proof
// returns ProjectedPatchHolds is reported.
//
// The retry keeps the audit exact because the audit never takes the proof as
// a statement about triangles outside the patch it was given. The proof's
// theorem (ProjectedPatchProof) holds for any set of triangles, so it holds
// for the reduced set: its pairs meet only at indexed common edges and
// vertices, and the audit omits exactly those pairs. Every dropped triangle is
// outside every family, so it stays in the audit with every other triangle,
// the retained triangles of this patch included. A failed retry leaves the
// whole patch in the audit, as a failed first proof does. The ring and the
// cap on inverted triangles decide only how often the retry succeeds, never
// whether the result is sound: widening the ring or the cap, or dropping the
// ring, changes which patches prove and no verdict.
//
// remaining is the proof's shared boundary-check allowance and budget the
// caller's work budget; both are charged as for the first proof. A canceled
// budget or an exhausted allowance makes the retry report false, and the
// caller's own budget check then returns the error.
func provenProjectedPatch(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	patch ProjectedPatch, remaining *uint64) (ProjectedPatch, bool) {
	switch projectedPatchProofLimited(budget, verts, tris, patch, remaining) {
	case ProjectedPatchHolds:
		return patch, true
	case ProjectedPatchOrientation:
	default:
		return ProjectedPatch{}, false
	}
	reduced, ok := withoutInvertedTriangles(budget, verts, tris, patch)
	if !ok {
		return ProjectedPatch{}, false
	}
	if projectedPatchProofLimited(budget, verts, tris, reduced, remaining) != ProjectedPatchHolds {
		return ProjectedPatch{}, false
	}
	return reduced, true
}

// withoutInvertedTriangles returns patch without its inverted triangles and
// one vertex ring around them. It reports false when the patch has no strict
// majority sign, has no inverted triangle, has more than
// projectedPatchRetryMaxInverted of them, names an index or vertex outside
// the mesh, or would be left empty.
func withoutInvertedTriangles(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	patch ProjectedPatch) (ProjectedPatch, bool) {
	u, v := patch.Axes[0], patch.Axes[1]
	if u < 0 || u > 2 || v < 0 || v > 2 || u == v {
		return ProjectedPatch{}, false
	}
	signs := make([]int, len(patch.Triangles))
	pos, neg := 0, 0
	for k, index := range patch.Triangles {
		if budget.Step() != nil || index < 0 || index >= len(tris) {
			return ProjectedPatch{}, false
		}
		for _, vertex := range tris[index] {
			if vertex < 0 || vertex >= len(verts) {
				return ProjectedPatch{}, false
			}
		}
		signs[k] = projectedTriangleSign(verts, tris[index], patch.Axes)
		switch signs[k] {
		case 1:
			pos++
		case -1:
			neg++
		}
	}
	major := 1
	inverted := len(patch.Triangles) - pos
	if neg > pos {
		major, inverted = -1, len(patch.Triangles)-neg
	}
	if pos == neg || inverted == 0 || inverted > projectedPatchRetryMaxInverted {
		return ProjectedPatch{}, false
	}
	ring := make(map[int]struct{}, 3*inverted)
	for k, index := range patch.Triangles {
		if signs[k] == major {
			continue
		}
		for _, vertex := range tris[index] {
			ring[vertex] = struct{}{}
		}
	}
	reduced := ProjectedPatch{Axes: patch.Axes, Triangles: make([]int, 0, len(patch.Triangles)-inverted)}
	for k, index := range patch.Triangles {
		if signs[k] != major {
			continue
		}
		tri := tris[index]
		_, a := ring[tri[0]]
		_, b := ring[tri[1]]
		_, c := ring[tri[2]]
		if !a && !b && !c {
			reduced.Triangles = append(reduced.Triangles, index)
		}
	}
	if len(reduced.Triangles) == 0 {
		return ProjectedPatch{}, false
	}
	return reduced, true
}
