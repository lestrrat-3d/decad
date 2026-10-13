package selectorquery

// RoleIndex maps each provenance role to the index of the last face that
// names it. faceRoles holds one role list per face, in face order.
func RoleIndex(faceRoles [][]string) map[string]int {
	index := make(map[string]int)
	for i, roles := range faceRoles {
		for _, role := range roles {
			index[role] = i
		}
	}
	return index
}
