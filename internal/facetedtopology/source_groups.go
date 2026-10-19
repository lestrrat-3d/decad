package facetedtopology

// UniqueGroupFace returns the one face index that every facet of group maps
// to through faceOf. It reports false when group is negative, src and faceOf
// differ in length, a matching facet's index lies outside [0, faceCount), two
// matching facets name different faces, or no facet carries group. The face
// list must name each face once, so equal indices mean the same face.
// (From uniqueResultSourceFace.)
func UniqueGroupFace(src, faceOf []int, faceCount, group int) (int, bool) {
	if group < 0 || len(faceOf) != len(src) {
		return -1, false
	}
	face := -1
	for i, g := range src {
		if g != group {
			continue
		}
		index := faceOf[i]
		if index < 0 || index >= faceCount {
			return -1, false
		}
		if face >= 0 && face != index {
			return -1, false
		}
		face = index
	}
	return face, face >= 0
}

// GroupFace returns the face index shared by every facet of group. The first
// matching facet fixes the index; a later facet naming another index refuses.
// The fixed index must lie in [0, faceCount). faceOf must be at least as long
// as src. (From pointCapGroupFace.)
func GroupFace(src, faceOf []int, faceCount, group int) (int, bool) {
	index := -1
	for i, source := range src {
		if source != group {
			continue
		}
		if index == -1 {
			index = faceOf[i]
		} else if index != faceOf[i] {
			return -1, false
		}
	}
	if index < 0 || index >= faceCount {
		return -1, false
	}
	return index, true
}

// TrimGroupFace returns the face index shared by every facet of group, or -1
// when src and faceOf differ in length, two facets name different faces, or
// the index is at least faceCount. spiralsector.TrimFaceGroup reads it.
func TrimGroupFace(src, faceOf []int, faceCount, group int) int {
	if len(faceOf) != len(src) {
		return -1
	}
	face := -1
	for i, g := range src {
		if g != group {
			continue
		}
		if face >= 0 && face != faceOf[i] {
			return -1
		}
		face = faceOf[i]
	}
	if face >= faceCount {
		return -1
	}
	return face
}

// CapGroupsPresent reports whether toe and heel are distinct groups below
// groupCount that each own at least one facet.
func CapGroupsPresent(src, faceOf []int, groupCount, toe, heel int) bool {
	if toe < 0 || heel < 0 ||
		toe == heel ||
		toe >= groupCount || heel >= groupCount ||
		len(faceOf) != len(src) {
		return false
	}
	toeSeen, heelSeen := false, false
	for _, group := range src {
		toeSeen = toeSeen || group == toe
		heelSeen = heelSeen || group == heel
	}
	return toeSeen && heelSeen
}

// CapGroupsPure reports whether the toe and heel cap groups each own one
// whole face of their own. A retained cone cap owns its entire live result
// face. A face shared with a different source group cannot be treated as an
// isolated tool-cap boundary.
func CapGroupsPure(src, faceOf []int, triCount, faceCount, toe, heel int) bool {
	if len(src) != triCount || len(faceOf) != triCount {
		return false
	}
	capFaces := [2]int{-1, -1}
	capGroups := [2]int{toe, heel}
	for i, group := range src {
		for capIndex, capGroup := range capGroups {
			if group != capGroup {
				continue
			}
			if faceOf[i] < 0 || faceOf[i] >= faceCount ||
				capFaces[capIndex] >= 0 && capFaces[capIndex] != faceOf[i] {
				return false
			}
			capFaces[capIndex] = faceOf[i]
		}
	}
	if capFaces[0] < 0 || capFaces[1] < 0 || capFaces[0] == capFaces[1] {
		return false
	}
	for i, face := range faceOf {
		if face == capFaces[0] && src[i] != capGroups[0] ||
			face == capFaces[1] && src[i] != capGroups[1] {
			return false
		}
	}
	return true
}
