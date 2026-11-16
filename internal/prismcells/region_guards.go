package prismcells

import (
	"slices"
	"sync/atomic"
)

// disabledGuards lets a test switch one of the extended Union's arrangement
// guards (V1, V2, V3; agent-docs/carrier-record-design.md §5.3) off to show
// that the guard is what refuses a scene. Production code never sets it.
var disabledGuards atomic.Pointer[[]string]

// guardOn reports whether the named guard runs.
func guardOn(name string) bool {
	off := disabledGuards.Load()
	return off == nil || !slices.Contains(*off, name)
}
