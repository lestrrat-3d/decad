package meshthick

import "slices"

// sortByKey orders the triangle indices by key.
func sortByKey(indices []int, key func(int) float64) {
	slices.SortFunc(indices, func(a, b int) int {
		ka, kb := key(a), key(b)
		switch {
		case ka < kb:
			return -1
		case ka > kb:
			return 1
		}
		return a - b
	})
}
