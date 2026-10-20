package loftcontact

import (
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// MatchingHeldCapOuters requires one shared held polygon at both axial
// levels, compared as edge sets in the start frame's plane coordinates with
// each edge's endpoints in a canonical order. This is stronger than equal input
// records: asymmetric station alignment or a different floating-point cap
// sample must not be silently replaced by the other end's polygon when the
// cap band is built.
func MatchingHeldCapOuters(frame r3.Frame, start, end *topo.Face) bool {
	if len(start.Loops) == 0 || len(end.Loops) == 0 ||
		!start.Loops[0].Outer || !end.Loops[0].Outer ||
		len(start.Loops[0].CoEdges) != len(end.Loops[0].CoEdges) {
		return false
	}
	keyOf := func(ce topo.CoEdge) [2]sectionrecord.Point2 {
		a, b := frame.ToLocal(topobuild.CoEdgeStart(ce).Position), frame.ToLocal(topobuild.CoEdgeEnd(ce).Position)
		p := sectionrecord.Point2{U: a.X, V: a.Y}
		q := sectionrecord.Point2{U: b.X, V: b.Y}
		if p.U > q.U || (p.U == q.U && p.V > q.V) {
			p, q = q, p
		}
		return [2]sectionrecord.Point2{p, q}
	}
	counts := make(map[[2]sectionrecord.Point2]int, len(start.Loops[0].CoEdges))
	for _, ce := range start.Loops[0].CoEdges {
		counts[keyOf(ce)]++
	}
	for _, ce := range end.Loops[0].CoEdges {
		key := keyOf(ce)
		if counts[key] == 0 {
			return false
		}
		counts[key]--
	}
	return true
}
