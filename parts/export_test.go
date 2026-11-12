package parts

import "sync/atomic"

// CountProjections makes a derived explode add one to n for every projection it builds.
func CountProjections(n *atomic.Int64) ExplodeOption {
	return option{func(c *config) error { c.projCount = n; return nil }}
}

// BoxOnlyBlocking makes a derived explode block every part as its box.
func BoxOnlyBlocking() ExplodeOption {
	return option{func(c *config) error { c.boxOnly = true; return nil }}
}
