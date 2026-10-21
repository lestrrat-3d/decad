// Package seal holds the key that lets decad's own producers reach the
// report types' sealed fields. Callers outside this module cannot name it.
package seal

// Key opens a sealed accessor on a report type.
type Key struct{}
