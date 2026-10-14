package loftmesh

import (
	"cmp"
	"iter"
	"math"
	"math/big"
	"slices"

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
		proof.AddRat(area, area, meshbool.Cross2x(origin, points[i], points[j]))
	}
	if area.Sign() != sigma {
		return ProjectedPatchBoundary
	}
	return patchBoundarySimple(budget, points, remaining)
}

// patchBoundarySimple proves that no two boundary edges meet except
// adjacent edges at their shared vertex. Two edges whose projected u
// ranges are strictly apart cannot meet, so they are skipped. The range of
// an edge is [min, max] of its end points' u, taken from the held float Fu,
// and a skip is allowed only when every Fu and Fv is exactly the point's
// rational U or V (a float64 vertex lifted by proof.XptOf always is): then
// the float comparison is the exact comparison of the rational ranges. Any other
// boundary, and any non-finite coordinate, takes the quadratic check, where
// the sweep's ordering is undefined or inexact. The edges are taken in order
// of their low u end, and each is tested against the later edges whose low
// u end does not pass its own high u end (see patchUOverlapPairs): exactly
// the pairs the u test of the quadratic check does not reject, touching
// ranges included. Each tested pair uses one unit of the shared check
// allowance and one budget step.
func patchBoundarySimple(budget *proofbound.WorkBudget, points []meshbool.Xp2,
	remaining *uint64) ProjectedPatchCondition {
	n := len(points)
	lo, hi, order, ok := patchUSweepOrder(points)
	if !ok {
		return patchBoundarySimpleQuadratic(budget, points, remaining)
	}
	for x, y := range patchUOverlapPairs(lo, hi, order) {
		if *remaining == 0 || budget.Step() != nil {
			return ProjectedPatchStructure
		}
		*remaining--
		i, j := int(min(x, y)), int(max(x, y))
		a, b := points[i], points[(i+1)%n]
		c, d := points[j], points[(j+1)%n]
		if math.Max(a.Fv, b.Fv) < math.Min(c.Fv, d.Fv) ||
			math.Max(c.Fv, d.Fv) < math.Min(a.Fv, b.Fv) {
			continue
		}
		// The adjacency and crossing tests below are those of the quadratic check.
		adjacent := j == i+1 || (i == 0 && j == n-1)
		if adjacent {
			if j == i+1 && !patchAdjacentEdgesMeetOnlyAtVertex(a, b, d) {
				return ProjectedPatchSelfContact
			}
			if i == 0 && j == n-1 && !patchAdjacentEdgesMeetOnlyAtVertex(c, d, b) {
				return ProjectedPatchSelfContact
			}
			continue
		}
		if patchSegmentsMeet(a, b, c, d) {
			return ProjectedPatchSelfContact
		}
	}
	return ProjectedPatchHolds
}

// patchUSweepOrder returns each edge's u range and the edges ordered by low
// u end (ties by index). It reports false when some coordinate is
// non-finite or some Fu or Fv is not exactly the point's rational U or V.
func patchUSweepOrder(points []meshbool.Xp2) (lo, hi []float64, order []int32, ok bool) {
	n := len(points)
	lo, hi = make([]float64, n), make([]float64, n)
	for i := range n {
		a, b := points[i].Fu, points[(i+1)%n].Fu
		if math.IsNaN(a) || math.IsNaN(b) || math.IsInf(a, 0) || math.IsInf(b, 0) ||
			math.IsNaN(points[i].Fv) || math.IsInf(points[i].Fv, 0) ||
			points[i].U == nil || points[i].V == nil ||
			new(big.Rat).SetFloat64(a).Cmp(points[i].U) != 0 ||
			new(big.Rat).SetFloat64(points[i].Fv).Cmp(points[i].V) != 0 {
			return nil, nil, nil, false
		}
		lo[i], hi[i] = math.Min(a, b), math.Max(a, b)
	}
	order = make([]int32, n)
	for i := range order {
		order[i] = int32(i)
	}
	slices.SortFunc(order, func(x, y int32) int {
		if c := cmp.Compare(lo[x], lo[y]); c != 0 {
			return c
		}
		return cmp.Compare(x, y)
	})
	return lo, hi, order, true
}

// patchUOverlapPairs yields every pair of edges, as (x, y) with x earlier in
// order, whose u ranges [lo, hi] are not strictly apart. order lists the
// edges by low u end. A pair is cut only when lo[y] > hi[x], a comparison of
// two finite floats with no rounding; touching ranges are yielded.
func patchUOverlapPairs(lo, hi []float64, order []int32) iter.Seq2[int32, int32] {
	return func(yield func(int32, int32) bool) {
		for k, x := range order {
			for _, y := range order[k+1:] {
				if lo[y] > hi[x] {
					break
				}
				if !yield(x, y) {
					return
				}
			}
		}
	}
}

// patchBoundarySimpleQuadratic tests every pair of boundary edges. It is
// the fallback for a boundary whose projected coordinates are not all
// finite, where the sweep's ordering is undefined.
func patchBoundarySimpleQuadratic(budget *proofbound.WorkBudget, points []meshbool.Xp2,
	remaining *uint64) ProjectedPatchCondition {
	n := len(points)
	for i := range n {
		a, b := points[i], points[(i+1)%n]
		for j := i + 1; j < n; j++ {
			if *remaining == 0 || budget.Step() != nil {
				return ProjectedPatchStructure
			}
			*remaining--
			c, d := points[j], points[(j+1)%n]
			if math.Max(a.Fu, b.Fu) < math.Min(c.Fu, d.Fu) ||
				math.Max(c.Fu, d.Fu) < math.Min(a.Fu, b.Fu) ||
				math.Max(a.Fv, b.Fv) < math.Min(c.Fv, d.Fv) ||
				math.Max(c.Fv, d.Fv) < math.Min(a.Fv, b.Fv) {
				continue
			}
			adjacent := j == i+1 || (i == 0 && j == n-1)
			if adjacent {
				if j == i+1 && !patchAdjacentEdgesMeetOnlyAtVertex(a, b, d) {
					return ProjectedPatchSelfContact
				}
				if i == 0 && j == n-1 && !patchAdjacentEdgesMeetOnlyAtVertex(c, d, b) {
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
