package stackedbrep

import (
	"maps"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// maxRingRotations bounds how many ring starts auditedRegion tries.
const maxRingRotations = 8

// CarrierOf is the carrier a record segment lies on, as LoopOf reads it
// before any alias applies.
func CarrierOf(seg CurveSegment) (Carrier, error) {
	w, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return Carrier{}, err
	}
	if err := requireStackedAnalyticWalk(w); err != nil {
		return Carrier{}, err
	}
	return ubCarrierOf(seg, w)
}

// SetHoledUnion marks the build as a holed union. audit is the nesting audit
// FaceRecord tries against each rotation of a holed region's rings.
func (b *Engine) SetHoledUnion(audit func(brepgeom.Profile) error) {
	b.holedUnion = true
	b.audit = audit
}

// SetAlias replaces the carrier alias with a copy of alias. LoopOf reads each
// segment's carrier through it.
func (b *Engine) SetAlias(alias map[Carrier]Carrier) {
	b.alias = maps.Clone(alias)
	if b.alias == nil {
		b.alias = map[Carrier]Carrier{}
	}
}

// Canonical is the carrier c stands for: its alias, or c itself.
func (b *Engine) Canonical(c Carrier) Carrier {
	if to, ok := b.alias[c]; ok {
		return to
	}
	return c
}

// AddSpanEnd records p as the end of a shared span on the canonical carrier
// of c.
func (b *Engine) AddSpanEnd(c Carrier, p Point2) {
	c = b.Canonical(c)
	if b.spanEnds[c] == nil {
		b.spanEnds[c] = map[Point2]struct{}{}
	}
	b.spanEnds[c][p] = struct{}{}
}

// spanEndJunction reads the junction of two circle carriers whose walked ends
// differ, where one end is a cut and the other a natural end at a recorded
// span end. It returns the natural end with its allowance widened by the
// distance between the two walked points, and reports whether the junction
// stands.
//
// The two segments are consecutive in a loop sketch returned, so they meet at
// one sketch vertex. Sketch reported that vertex as the shared span's end,
// which is the natural end's recorded point. The cut end's walk lies within
// its own allowance of the point its carrier denotes there, so the natural
// point lies within the returned allowance of that carrier. The distance is
// charged, never compared to a threshold. The junction rests on the
// one-carrier reading of the span: both of its carriers are one canonical
// carrier.
func (b *Engine) spanEndJunction(ci Carrier, ei ubEnd, cj Carrier, sj ubEnd) (ubEnd, bool) {
	if !b.holedUnion || ci.Kind != ubCircle || cj.Kind != ubCircle || ei.cut == sj.cut {
		return ubEnd{}, false
	}
	nat, cut, cutCarrier := ei, sj, cj
	if ei.cut {
		nat, cut, cutCarrier = sj, ei, ci
	}
	if _, ok := b.spanEnds[b.Canonical(cutCarrier)][nat.p]; !ok {
		return ubEnd{}, false
	}
	ru, rv := proof.FloatRat(ei.p.U), proof.FloatRat(ei.p.V)
	su, sv := proof.FloatRat(sj.p.U), proof.FloatRat(sj.p.V)
	if ru == nil || rv == nil || su == nil || sv == nil {
		return ubEnd{}, false
	}
	sep := boundarywalk.RatL1Upper(new(big.Rat).Sub(su, ru), new(big.Rat).Sub(sv, rv))
	if proofbound.IsNonFinite(sep) {
		return ubEnd{}, false
	}
	nat.allow = proofbound.AbsSumUpper(max(nat.allow, cut.allow), sep)
	return nat, true
}

// RotateRing restates loop starting at its (k mod len)-th segment.
func RotateRing(loop LoopRecord, k int) LoopRecord {
	n := len(loop.Segments)
	if n == 0 {
		return loop
	}
	k %= n
	segs := make([]CurveSegment, 0, n)
	segs = append(segs, loop.Segments[k:]...)
	segs = append(segs, loop.Segments[:k]...)
	return LoopRecord{Segments: segs}
}

// auditedRegion returns region under the first ring rotation audit passes.
// Rotation changes no geometry; it only changes which loop point the nesting
// audit classifies. S7 rules out contact, so every point of a loop classifies
// alike, and a pass under any rotation is a pass of the region. With no
// passing rotation it returns region unchanged, for the caller's own audit to
// refuse. It tries nothing unless the build is a holed union with an audit
// and the region has holes.
func (b *Engine) auditedRegion(region brepgeom.Profile) brepgeom.Profile {
	if !b.holedUnion || b.audit == nil || len(region.Holes) == 0 {
		return region
	}
	longest := len(region.Outer.Segments)
	for _, hole := range region.Holes {
		longest = max(longest, len(hole.Segments))
	}
	for k := range min(maxRingRotations, longest) {
		rotated := brepgeom.Profile{Outer: RotateRing(region.Outer, k)}
		for _, hole := range region.Holes {
			rotated.Holes = append(rotated.Holes, RotateRing(hole, k))
		}
		if b.audit(rotated) == nil {
			return rotated
		}
	}
	return region
}
