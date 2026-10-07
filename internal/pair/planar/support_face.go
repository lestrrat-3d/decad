package planar

import (
	"math/big"
	"slices"
	"sort"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// VertexFaceIDs lists the distinct face ids of the triangles holding vertex v.
func VertexFaceIDs(solid *PlanarSolid, v int) []int {
	var ids []int
	for t, tri := range solid.Tris {
		if tri[0] != v && tri[1] != v && tri[2] != v {
			continue
		}
		if id := solid.Faces[t]; !slices.Contains(ids, id) {
			ids = append(ids, id)
		}
	}
	sort.Ints(ids)
	return ids
}

// SupportFace is the part of a solid on its support plane, projected by
// dropping the normal's largest component, with edges bounding the triangles.
type SupportFace struct {
	ID    int
	Drop  int
	tris  [][3][2]*big.Rat
	edges [][2][2]*big.Rat
}

// BuildSupportFace collects triangles on the support plane. They must all
// belong to one face of the solid, which the manifold names.
func BuildSupportFace(solid *PlanarSolid, points []proofarith.DyV3,
	normal, origin proofarith.DyV3) (SupportFace, bool) {
	if solid.Faces == nil {
		return SupportFace{}, false
	}
	drop := 0
	for k := 1; k < 3; k++ {
		if proofarith.DyCmp(proofarith.DyAbs(normal[k]), proofarith.DyAbs(normal[drop])) > 0 {
			drop = k
		}
	}
	project := func(v proofarith.DyV3) [2]*big.Rat {
		i, j := (drop+1)%3, (drop+2)%3
		return [2]*big.Rat{v[i].Rat(), v[j].Rat()}
	}
	face := SupportFace{ID: -1, Drop: drop}
	directed := make(map[[2]int]struct{})
	var held [][3]int
	for t, tri := range solid.Tris {
		onPlane := true
		for _, v := range tri {
			if proofarith.DvDot(normal, proofarith.DvSub(points[v], origin)).Sign() != 0 {
				onPlane = false
				break
			}
		}
		if !onPlane {
			continue
		}
		a := points[tri[0]]
		n := proofarith.DvCross(proofarith.DvSub(points[tri[1]], a),
			proofarith.DvSub(points[tri[2]], a))
		if proofarith.DvDot(n, normal).Sign() <= 0 {
			return SupportFace{}, false
		}
		if face.ID >= 0 && solid.Faces[t] != face.ID {
			return SupportFace{}, false
		}
		face.ID = solid.Faces[t]
		held = append(held, tri)
		face.tris = append(face.tris, [3][2]*big.Rat{project(points[tri[0]]),
			project(points[tri[1]]), project(points[tri[2]])})
		for i := range 3 {
			directed[[2]int{tri[i], tri[(i+1)%3]}] = struct{}{}
		}
	}
	if face.ID < 0 {
		return SupportFace{}, false
	}
	for _, tri := range held {
		for i := range 3 {
			from, to := tri[i], tri[(i+1)%3]
			if _, shared := directed[[2]int{to, from}]; shared {
				continue
			}
			face.edges = append(face.edges, [2][2]*big.Rat{project(points[from]),
				project(points[to])})
		}
	}
	return face, true
}

// HoldsBox reports whether a closed box in the face's projected coordinates
// lies inside the face: it meets no bounding edge and has a corner inside
// one of the face's triangles.
func (face *SupportFace) HoldsBox(lo, hi [2]*big.Rat) bool {
	inside := false
	for _, tri := range face.tris {
		if pointInTriangle(lo, tri) {
			inside = true
			break
		}
	}
	if !inside {
		return false
	}
	for _, edge := range face.edges {
		if segmentMeetsBox(edge[0], edge[1], lo, hi) {
			return false
		}
	}
	return true
}

func orient(a, b, c [2]*big.Rat) int {
	left := new(big.Rat).Mul(new(big.Rat).Sub(b[0], a[0]), new(big.Rat).Sub(c[1], a[1]))
	right := new(big.Rat).Mul(new(big.Rat).Sub(b[1], a[1]), new(big.Rat).Sub(c[0], a[0]))
	return left.Cmp(right)
}

// pointInTriangle tests the closed triangle in either winding.
func pointInTriangle(p [2]*big.Rat, tri [3][2]*big.Rat) bool {
	sign := orient(tri[0], tri[1], tri[2])
	if sign == 0 {
		return false
	}
	for k := range 3 {
		if orient(tri[k], tri[(k+1)%3], p)*sign < 0 {
			return false
		}
	}
	return true
}

// segmentMeetsBox is the separating-axis test of a segment and a closed
// axis-aligned box: the two box axes and the segment's normal.
func segmentMeetsBox(a, b, lo, hi [2]*big.Rat) bool {
	for axis := range 2 {
		if proofbound.RatMax(a[axis], b[axis]).Cmp(lo[axis]) < 0 || proofbound.RatMin(a[axis], b[axis]).Cmp(hi[axis]) > 0 {
			return false
		}
	}
	positive, negative := false, false
	for _, corner := range [4][2]*big.Rat{{lo[0], lo[1]}, {hi[0], lo[1]}, {lo[0], hi[1]}, {hi[0], hi[1]}} {
		switch orient(a, b, corner) {
		case 1:
			positive = true
		case -1:
			negative = true
		default:
			return true
		}
	}
	return positive && negative
}
