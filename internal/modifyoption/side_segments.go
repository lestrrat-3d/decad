package modifyoption

import "fmt"

// SideSegments returns the recorded segments of loop li named by roles of the
// exact form "side(li,j)". Any other role, including one with trailing text
// or a different loop, is skipped.
func SideSegments(roles []string, li int) map[int]struct{} {
	out := map[int]struct{}{}
	for _, role := range roles {
		var i, j int
		if n, err := fmt.Sscanf(role, "side(%d,%d)", &i, &j); err != nil || n != 2 || i != li {
			continue
		}
		if role != fmt.Sprintf("side(%d,%d)", i, j) {
			continue
		}
		out[j] = struct{}{}
	}
	return out
}

// WalkHoldsAny reports whether any recorded segment of a coalesced walk is
// in segs.
func WalkHoldsAny(walkSegs []int, segs map[int]struct{}) bool {
	for _, s := range walkSegs {
		if _, ok := segs[s]; ok {
			return true
		}
	}
	return false
}
