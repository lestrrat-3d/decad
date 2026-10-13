package revolveshell

import "fmt"

// SideSegments returns the recorded meridian segments j named by the
// "side(0,j)" roles in roles, in order. Roles that are not of that form, or
// that name another loop, are skipped. ok is false when no role names a
// segment, so the face is not a generated side face of the revolve.
func SideSegments(roles []string) ([]int, bool) {
	var segs []int
	for _, role := range roles {
		var li, j int
		if n, _ := fmt.Sscanf(role, "side(%d,%d)", &li, &j); n != 2 || li != 0 {
			continue
		}
		segs = append(segs, j)
	}
	return segs, len(segs) > 0
}
