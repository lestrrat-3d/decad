package loftmesh

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ProjectedPatch names one triangle family that may form a graph over a
// coordinate plane. Triangles are indices into the complete joined mesh.
// Axes are distinct coordinate indices in [0, 3). A successful proof decides
// only pairs inside this family; all pairs with other triangles stay audited.
type ProjectedPatch struct {
	Triangles []int
	Axes      [2]int
}

// ProjectedPatchCondition records the first failed condition of a projected
// patch proof. A failed proof never removes a triangle pair from the audit.
type ProjectedPatchCondition uint8

const (
	ProjectedPatchHolds ProjectedPatchCondition = iota
	ProjectedPatchStructure
	ProjectedPatchOrientation
	ProjectedPatchBoundary
	ProjectedPatchSelfContact
)

type patchEdge struct{ a, b int }

type patchEdgeUse struct {
	from, to, count, net int
}

func patchEdgeKey(a, b int) patchEdge {
	if a > b {
		a, b = b, a
	}
	return patchEdge{a: a, b: b}
}

func projectedCoord(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// projectedTriangleSign takes the same conservative fast path as
// meshbool.Cross2xSign, then lifts undecided float corners exactly.
func projectedTriangleSign(verts []r3.Vec, tri [3]int, axes [2]int) int {
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	au, av := projectedCoord(a, axes[0]), projectedCoord(a, axes[1])
	bu, bv := projectedCoord(b, axes[0]), projectedCoord(b, axes[1])
	cu, cv := projectedCoord(c, axes[0]), projectedCoord(c, axes[1])
	if proofbound.IsNonFinite(au) || proofbound.IsNonFinite(av) ||
		proofbound.IsNonFinite(bu) || proofbound.IsNonFinite(bv) ||
		proofbound.IsNonFinite(cu) || proofbound.IsNonFinite(cv) {
		return 0
	}
	det := (bu-au)*(cv-av) - (bv-av)*(cu-au)
	scale := (math.Abs(bu)+math.Abs(au))*(math.Abs(cv)+math.Abs(av)) +
		(math.Abs(bv)+math.Abs(av))*(math.Abs(cu)+math.Abs(au))
	err := 1e-12 * scale
	if !proofbound.IsNonFinite(det) && !proofbound.IsNonFinite(err) && err > 0 {
		if det > err {
			return 1
		}
		if det < -err {
			return -1
		}
	}
	ax := meshbool.NewXP2FromXpt(proof.XptOf(a), axes[0], axes[1])
	bx := meshbool.NewXP2FromXpt(proof.XptOf(b), axes[0], axes[1])
	cx := meshbool.NewXP2FromXpt(proof.XptOf(c), axes[0], axes[1])
	return meshbool.Cross2xSign(ax, bx, cx)
}

func patchProjectedPoint(v r3.Vec, axes [2]int) meshbool.Xp2 {
	return meshbool.NewXP2FromXpt(proof.XptOf(v), axes[0], axes[1])
}

func patchSegmentsMeet(a, b, c, d meshbool.Xp2) bool {
	abc := meshbool.Cross2xSign(a, b, c)
	abd := meshbool.Cross2xSign(a, b, d)
	cda := meshbool.Cross2xSign(c, d, a)
	cdb := meshbool.Cross2xSign(c, d, b)
	if abc*abd < 0 && cda*cdb < 0 {
		return true
	}
	if abc == 0 {
		if on, _ := meshbool.OnSegment2(a, b, c); on {
			return true
		}
	}
	if abd == 0 {
		if on, _ := meshbool.OnSegment2(a, b, d); on {
			return true
		}
	}
	if cda == 0 {
		if on, _ := meshbool.OnSegment2(c, d, a); on {
			return true
		}
	}
	if cdb == 0 {
		if on, _ := meshbool.OnSegment2(c, d, b); on {
			return true
		}
	}
	return false
}

func patchAdjacentEdgesMeetOnlyAtVertex(a, b, c meshbool.Xp2) bool {
	if a.U.Cmp(c.U) == 0 && a.V.Cmp(c.V) == 0 {
		return false
	}
	if on, interior := meshbool.OnSegment2(a, b, c); on && interior {
		return false
	}
	if on, interior := meshbool.OnSegment2(b, c, a); on && interior {
		return false
	}
	return true
}

// ProjectedPatchProof proves that the held triangles of one family are a
// single non-self-touching graph over Axes. The proof uses an exact directed
// edge census, strict projected orientation, and a simple projected boundary.
// The oriented 2-chain then has the boundary polygon's winding number almost
// everywhere. Every projected triangle has the same sign, so no two projected
// interiors overlap. Exact boundary simplicity and nonzero triangle areas
// also exclude an unindexed edge or vertex contact. Since each 3D triangle
// is a graph over its projected triangle, its only same-family contacts are
// those at indexed common edges or vertices.
func ProjectedPatchProof(budget *proofbound.WorkBudget, verts []r3.Vec,
	tris [][3]int, patch ProjectedPatch) ProjectedPatchCondition {
	remaining := uint64(100_000_000)
	return projectedPatchProofLimited(budget, verts, tris, patch, &remaining)
}

func projectedPatchProofLimited(budget *proofbound.WorkBudget, verts []r3.Vec,
	tris [][3]int, patch ProjectedPatch, remaining *uint64) ProjectedPatchCondition {
	u, v := patch.Axes[0], patch.Axes[1]
	if u < 0 || u > 2 || v < 0 || v > 2 || u == v || len(patch.Triangles) == 0 {
		return ProjectedPatchStructure
	}
	use := make(map[patchEdge]patchEdgeUse, 3*len(patch.Triangles)/2)
	sigma := 0
	for _, index := range patch.Triangles {
		if budget.Step() != nil || index < 0 || index >= len(tris) {
			return ProjectedPatchStructure
		}
		tri := tris[index]
		for _, vertex := range tri {
			if vertex < 0 || vertex >= len(verts) {
				return ProjectedPatchStructure
			}
		}
		sign := projectedTriangleSign(verts, tri, patch.Axes)
		if sign == 0 || (sigma != 0 && sign != sigma) {
			return ProjectedPatchOrientation
		}
		sigma = sign
		for k, a := range tri {
			b := tri[(k+1)%3]
			if a == b {
				return ProjectedPatchStructure
			}
			key := patchEdgeKey(a, b)
			entry := use[key]
			entry.from, entry.to = a, b
			entry.count++
			if a < b {
				entry.net++
			} else {
				entry.net--
			}
			if entry.count > 2 {
				return ProjectedPatchStructure
			}
			use[key] = entry
		}
	}
	next := make(map[int]int)
	in := make(map[int]int)
	for _, edge := range use {
		switch {
		case edge.count == 2 && edge.net == 0:
			continue
		case edge.count != 1 || (edge.net != 1 && edge.net != -1):
			return ProjectedPatchStructure
		}
		if _, exists := next[edge.from]; exists {
			return ProjectedPatchBoundary
		}
		next[edge.from] = edge.to
		in[edge.to]++
		if in[edge.to] > 1 {
			return ProjectedPatchBoundary
		}
	}
	if len(next) < 3 {
		return ProjectedPatchBoundary
	}
	start := -1
	for vertex := range next {
		start = vertex
		break
	}
	loop := make([]int, 0, len(next))
	seen := make(map[int]struct{}, len(next))
	for vertex := start; ; vertex = next[vertex] {
		if _, exists := seen[vertex]; exists {
			if vertex != start || len(loop) != len(next) {
				return ProjectedPatchBoundary
			}
			break
		}
		if _, exists := next[vertex]; !exists || in[vertex] != 1 {
			return ProjectedPatchBoundary
		}
		seen[vertex] = struct{}{}
		loop = append(loop, vertex)
	}
	points := make([]meshbool.Xp2, len(loop))
	for i, vertex := range loop {
		points[i] = patchProjectedPoint(verts[vertex], patch.Axes)
	}
	area := new(big.Rat)
	origin := meshbool.NewXP2(big.NewRat(0, 1), big.NewRat(0, 1))
	for i := range loop {
		if budget.Step() != nil {
			return ProjectedPatchStructure
		}
		j := (i + 1) % len(loop)
		if points[i].U.Cmp(points[j].U) == 0 && points[i].V.Cmp(points[j].V) == 0 {
			return ProjectedPatchBoundary
		}
		area.Add(area, meshbool.Cross2x(origin, points[i], points[j]))
	}
	if area.Sign() != sigma {
		return ProjectedPatchBoundary
	}
	for i := range loop {
		a, b := points[i], points[(i+1)%len(loop)]
		for j := i + 1; j < len(loop); j++ {
			if *remaining == 0 || budget.Step() != nil {
				return ProjectedPatchStructure
			}
			*remaining -= 1
			c, d := points[j], points[(j+1)%len(loop)]
			if math.Max(a.Fu, b.Fu) < math.Min(c.Fu, d.Fu) ||
				math.Max(c.Fu, d.Fu) < math.Min(a.Fu, b.Fu) ||
				math.Max(a.Fv, b.Fv) < math.Min(c.Fv, d.Fv) ||
				math.Max(c.Fv, d.Fv) < math.Min(a.Fv, b.Fv) {
				continue
			}
			adjacent := j == i+1 || (i == 0 && j == len(loop)-1)
			if adjacent {
				if j == i+1 && !patchAdjacentEdgesMeetOnlyAtVertex(a, b, d) {
					return ProjectedPatchSelfContact
				}
				if i == 0 && j == len(loop)-1 && !patchAdjacentEdgesMeetOnlyAtVertex(c, d, b) {
					return ProjectedPatchSelfContact
				}
				continue
			}
			if patchSegmentsMeet(a, b, c, d) {
				return ProjectedPatchSelfContact
			}
		}
	}
	return ProjectedPatchHolds
}
