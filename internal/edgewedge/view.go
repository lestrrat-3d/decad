package edgewedge

import "github.com/lestrrat-3d/decad/internal/selectorquery"

// View is the edge view the root selector resolves against: the node facts
// selectorquery.EdgeNode exposes, plus the wedge reading. It lives here, not
// in selectorquery, because the reading calls topobuild.NormalAt and
// topobuild already imports selectorquery.
type View struct {
	selectorquery.EdgeNode
}

// SelectorWedge reports whether the edge's wedge is decided, and whether a
// decided wedge is reflex.
func (v View) SelectorWedge() (bool, bool) {
	switch Of(v.Edge) {
	case Reflex:
		return true, true
	case Salient:
		return true, false
	default:
		return false, false
	}
}
