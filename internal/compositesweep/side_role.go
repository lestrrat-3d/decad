package compositesweep

import (
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/topo"
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

// SpanFaceRole returns the role a composite span face publishes: a "side("
// role is renumbered as a side wall of span, and, when reverseCaps is set, the
// start and end cap roles trade places. Any other role is returned unchanged.
func SpanFaceRole(role string, span int, reverseCaps bool, capStart, capEnd string) string {
	if suffix, ok := strings.CutPrefix(role, "side("); ok {
		return fmt.Sprintf("side(%d,%s", span, suffix)
	}
	if reverseCaps {
		return SwapCapRole(role, capStart, capEnd)
	}
	return role
}

// SwapCapRole returns capEnd for capStart, capStart for capEnd, and every
// other role unchanged.
func SwapCapRole(role, capStart, capEnd string) string {
	switch role {
	case capStart:
		return capEnd
	case capEnd:
		return capStart
	}
	return role
}

// RewriteFaceSideRoles renumbers each "side(" role of face as a side wall of
// span (RewriteSideRole); every other role stays.
func RewriteFaceSideRoles(face *topo.Face, span int) {
	for i, origin := range face.Origins {
		role, ok := RewriteSideRole(origin.Role, span)
		if !ok {
			continue
		}
		origin.Role = role
		face.Origins[i] = origin
	}
}
