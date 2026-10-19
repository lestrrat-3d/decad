package facetedtopology

import (
	"context"

	"github.com/lestrrat-3d/r3"
)

// GroupedMesh is a held mesh with each facet's source group and face index.
type GroupedMesh struct {
	Verts  []r3.Vec
	Tris   [][3]int
	Src    []int
	FaceOf []int
}

type capTriangle [3]r3.Vec

func capTriangleOf(v []r3.Vec, t [3]int) capTriangle {
	out := capTriangle{v[t[0]], v[t[1]], v[t[2]]}
	less := func(a, b r3.Vec) bool {
		if a.X != b.X {
			return a.X < b.X
		}
		if a.Y != b.Y {
			return a.Y < b.Y
		}
		return a.Z < b.Z
	}
	if less(out[1], out[0]) {
		out[0], out[1] = out[1], out[0]
	}
	if less(out[2], out[1]) {
		out[1], out[2] = out[2], out[1]
	}
	if less(out[1], out[0]) {
		out[0], out[1] = out[1], out[0]
	}
	return out
}

// UnchangedGroupPatch reports whether the facets of base with source index group survive in
// result as the same triangle multiset on one result face, and returns that
// face's index. The base group must sit on one face; result facets of that
// face's index are compared. A nil result is no match. Face lists must name
// each face once, so equal indices mean the same face. A Cut may keep only
// part of a cap: this requires the exact held triangle multiset and one
// unchanged source patch. (From unchangedLoftCap.)
func UnchangedGroupPatch(ctx context.Context, base GroupedMesh, groupCount, group, baseFaceCount int,
	result *GroupedMesh, resultFaceCount int) (int, bool, error) {
	if group < 0 || group >= groupCount || len(base.FaceOf) != len(base.Tris) {
		return -1, false, nil
	}
	sourceIndex := -1
	want := map[capTriangle]int{}
	for i, tri := range base.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return -1, false, err
			}
		}
		if base.Src[i] != group {
			continue
		}
		faceIndex := base.FaceOf[i]
		if faceIndex < 0 || faceIndex >= baseFaceCount {
			return -1, false, nil
		}
		if sourceIndex >= 0 && sourceIndex != faceIndex {
			return -1, false, nil
		}
		sourceIndex = faceIndex
		want[capTriangleOf(base.Verts, tri)]++
	}
	if sourceIndex < 0 {
		return -1, false, nil
	}
	if result == nil {
		return -1, false, nil
	}
	p := *result
	resultIndex := -1
	count := 0
	for i, tri := range p.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return -1, false, err
			}
		}
		if p.Src[i] != sourceIndex {
			continue
		}
		faceIndex := p.FaceOf[i]
		if faceIndex < 0 || faceIndex >= resultFaceCount {
			return -1, false, nil
		}
		if resultIndex >= 0 && resultIndex != faceIndex {
			return -1, false, nil
		}
		resultIndex = faceIndex
		key := capTriangleOf(p.Verts, tri)
		if want[key] == 0 {
			return -1, false, nil
		}
		want[key]--
		count++
	}
	for _, remaining := range want {
		if remaining != 0 {
			return -1, false, nil
		}
	}
	if count == 0 {
		return -1, false, nil
	}
	return resultIndex, true, nil
}

// SortedTriangle returns triangle's vertex indices in ascending order and
// the sign of the sorting permutation: +1 for even, -1 for odd.
func SortedTriangle(triangle [3]int) ([3]int, int) {
	sign := 1
	if triangle[0] > triangle[1] {
		triangle[0], triangle[1] = triangle[1], triangle[0]
		sign = -sign
	}
	if triangle[1] > triangle[2] {
		triangle[1], triangle[2] = triangle[2], triangle[1]
		sign = -sign
	}
	if triangle[0] > triangle[1] {
		triangle[0], triangle[1] = triangle[1], triangle[0]
		sign = -sign
	}
	return triangle, sign
}

// CapsOppose reports whether the lowerGroup facets of the lower mesh and the
// upperGroup facets of the upper mesh, mapped through upperMap, are the same
// triangles with opposite winding.
func CapsOppose(lowerTris [][3]int, lowerSrc []int, upperTris [][3]int, upperSrc []int,
	upperMap []int, lowerGroup, upperGroup int) bool {
	indices := make(map[[3]int]int)
	for i, triangle := range lowerTris {
		if lowerSrc[i] != lowerGroup {
			continue
		}
		key, sign := SortedTriangle(triangle)
		if indices[key] != 0 {
			return false
		}
		indices[key] = sign
	}
	if len(indices) == 0 {
		return false
	}
	for i, triangle := range upperTris {
		if upperSrc[i] != upperGroup {
			continue
		}
		for corner, index := range triangle {
			if index < 0 || index >= len(upperMap) {
				return false
			}
			triangle[corner] = upperMap[index]
		}
		key, sign := SortedTriangle(triangle)
		if indices[key] != -sign {
			return false
		}
		delete(indices, key)
	}
	return len(indices) == 0
}
