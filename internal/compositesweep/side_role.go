package compositesweep

import (
	"fmt"
	"strings"
)

// RewriteSideRole returns role renumbered as a side wall of span. A role that
// is not a "side(" role is returned unchanged with false. A single-span
// reduction has already inserted span 0, so a leading "0," path index with
// more indices behind it is replaced instead of nested.
func RewriteSideRole(role string, span int) (string, bool) {
	suffix, ok := strings.CutPrefix(role, "side(")
	if !ok {
		return role, false
	}
	if rest, found := strings.CutPrefix(suffix, "0,"); found && strings.Count(suffix, ",") >= 2 {
		suffix = rest
	}
	return fmt.Sprintf("side(%d,%s", span, suffix), true
}
