package decad

import (
	"context"
	"slices"
	"sort"
)

// revolveLoftFacesHeldPair is one live source-face switch in an already
// audited held Boolean result. Its edges retain the result's exact vertex
// indices; the source certificate must still justify each complete chain.
type revolveLoftFacesHeldPair struct {
	blankFace, trimmedFace *Face
	edges                  [][2]int
}

// revolveLoftFacesHeldPairs enumerates all cross-operand result edges from
// actual triangle ownership. It has no allowed-role table and admits no
// contact: callers must prove each returned pair against authenticated
// source geometry, then cover every near contact omitted by output edges.
func revolveLoftFacesHeldPairs(ctx context.Context, blank, trimmed *Body,
	result facetedPayload) ([]revolveLoftFacesHeldPair, bool, error) {
	if ctx == nil || blank == nil || trimmed == nil ||
		len(result.tris) == 0 || len(result.tris) > 1<<22 ||
		len(result.src) != len(result.tris) {
		return nil, false, nil
	}
	blankFaces, trimmedFaces := blank.Faces(), trimmed.Faces()
	faces := append(append([]*Face(nil), blankFaces...), trimmedFaces...)
	if len(result.groups) != len(faces) {
		return nil, false, nil
	}
	for i, face := range faces {
		if !slices.Equal(result.groups[i].origins, face.Origins()) ||
			result.groups[i].planar != face.isPlanar() {
			return nil, false, nil
		}
	}
	type edge struct{ a, b int }
	type owners struct {
		count uint8
		faces [2]int
	}
	seams := make(map[edge]owners, len(result.tris)*3/2)
	for i, tri := range result.tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
		}
		if result.src[i] < 0 || result.src[i] >= len(faces) {
			return nil, false, nil
		}
		for j := range 3 {
			a, b := tri[j], tri[(j+1)%3]
			if a < 0 || b < 0 || a >= len(result.verts) || b >= len(result.verts) || a == b {
				return nil, false, nil
			}
			if a > b {
				a, b = b, a
			}
			key := edge{a, b}
			entry := seams[key]
			if entry.count >= 2 {
				return nil, false, nil
			}
			entry.faces[entry.count] = result.src[i]
			entry.count++
			seams[key] = entry
		}
	}
	var pairs []revolveLoftFacesHeldPair
	index := map[[2]int]int{}
	for key, owner := range seams {
		if owner.count != 2 {
			return nil, false, nil
		}
		a, b := owner.faces[0], owner.faces[1]
		if a >= len(blankFaces) && b < len(blankFaces) {
			a, b = b, a
		}
		if a >= len(blankFaces) || b < len(blankFaces) {
			continue
		}
		b -= len(blankFaces)
		if a < 0 || b < 0 || b >= len(trimmedFaces) {
			return nil, false, nil
		}
		role := [2]int{a, b}
		position, found := index[role]
		if !found {
			position = len(pairs)
			index[role] = position
			pairs = append(pairs, revolveLoftFacesHeldPair{
				blankFace: blankFaces[a], trimmedFace: trimmedFaces[b]})
		}
		pairs[position].edges = append(pairs[position].edges, [2]int{key.a, key.b})
	}
	if len(pairs) == 0 {
		return nil, false, nil
	}
	sort.Slice(pairs, func(i, j int) bool {
		a, b := indexOfFace(blankFaces, pairs[i].blankFace),
			indexOfFace(blankFaces, pairs[j].blankFace)
		if a != b {
			return a < b
		}
		return indexOfFace(trimmedFaces, pairs[i].trimmedFace) <
			indexOfFace(trimmedFaces, pairs[j].trimmedFace)
	})
	return pairs, true, nil
}
