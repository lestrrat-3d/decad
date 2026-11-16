package brepmodify

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file holds what happens after the corners are read: CB5's widened
// cutbacks, §7.7's result record and CB7's audit transfer.

// carrierWiden is CB5: S6 reads every selected corner's cutbacks widened by
// 2·T(δ′), and every unselected junction with δ > 0 at the end of a walk a
// selected corner trims widens that walk by 2·T(δ + a).
func carrierWiden(cr prismrecord.CarrierRecord, sections []sectionrecord.LoopRecord, loops []survey2d.CornerLoop,
	blendAt []map[int]*offset2d.Blend, corners []map[int]*cbCorner) ([]map[int]*offset2d.Blend, error) {
	out := make([]map[int]*offset2d.Blend, len(blendAt))
	for li, at := range blendAt {
		out[li] = make(map[int]*offset2d.Blend, len(at))
		walks := loops[li].Walks
		n := len(walks)
		nseg := len(sections[li].Segments)
		for ci, cb := range at {
			c := corners[li][ci]
			xA := math.Max(math.Max(proofbound.AbsSumUpper(c.deltaJ, c.aJ), math.Max(c.dfA, c.dfB)), c.a.eps)
			xB := math.Max(math.Max(proofbound.AbsSumUpper(c.deltaJ, c.aJ), math.Max(c.dfA, c.dfB)), c.b.eps)
			tA, tB := c.a.transport(xA), c.b.transport(xB)
			if (math.IsInf(tA, 1) || math.IsInf(tB, 1)) && rowOn("CB5") {
				return nil, cbRefusal("CB5", c.where(), `a displacement %g reaches a circular carrier's radius, so S6's cutback comparison cannot be transferred`, math.Max(xA, xB))
			}
			wide := *cb
			wide.CutbackA = sumUp(cb.CutbackA, tA, tA)
			wide.CutbackB = sumUp(cb.CutbackB, tB, tB)
			out[li][ci] = &wide
		}
		if len(at) == 0 {
			continue
		}
		for ci := range walks {
			if _, selected := at[ci]; selected {
				continue
			}
			prev, next := walks[(ci+n-1)%n], walks[ci]
			prevEps, _, deltaJ, ok := walkEntries(cr, li, prev, nseg)
			if !ok || deltaJ == 0 {
				continue
			}
			nextEps, _, _, _ := walkEntries(cr, li, next, nseg)
			_, trimsPrev := at[(ci+n-1)%n]
			_, trimsNext := at[(ci+1)%n]
			if !trimsPrev && !trimsNext {
				continue
			}
			x := proofbound.AbsSumUpper(deltaJ, math.Max(proofbound.WalkEndBoundAllow(prev.EndBound), proofbound.WalkEndBoundAllow(next.StartBound)))
			wide := &offset2d.Blend{}
			if trimsPrev {
				cp, okP := newCBCarrier(sections[li].Segments[prev.Segs[len(prev.Segs)-1]], prev, prevEps)
				if !okP {
					return nil, cbRefusal("CB5", "", `a carrier beside a displaced junction is not a finite line or circle`)
				}
				t := cp.transport(x)
				if math.IsInf(t, 1) && rowOn("CB5") {
					return nil, cbRefusal("CB5", "", `a junction displacement %g reaches a circular carrier's radius`, x)
				}
				wide.CutbackA = sumUp(t, t)
			}
			if trimsNext {
				cn, okN := newCBCarrier(sections[li].Segments[next.Segs[0]], next, nextEps)
				if !okN {
					return nil, cbRefusal("CB5", "", `a carrier beside a displaced junction is not a finite line or circle`)
				}
				t := cn.transport(x)
				if math.IsInf(t, 1) && rowOn("CB5") {
					return nil, cbRefusal("CB5", "", `a junction displacement %g reaches a circular carrier's radius`, x)
				}
				wide.CutbackB = sumUp(t, t)
			}
			out[li][ci] = wide
		}
	}
	return out, nil
}

// emitted is one segment offset2d.RewriteLoop emits, with its record entries.
type emitted struct {
	seg       sectionrecord.CurveSegment
	eps       float64
	sd, ed    float64 // displacement of its start and end read points
	unchanged bool
	connector bool
	seam      seamMove // closeSeams' move of this segment's start
}

// carrierRewriteRecord is §7.7: the rewritten section's carrier record, in
// offset2d.RewriteLoop's segment order (each walk, then the connector of the
// corner at its end). A loop without a blend is cloned and keeps its entries.
func carrierRewriteRecord(cr prismrecord.CarrierRecord, sections []sectionrecord.LoopRecord, loops []survey2d.CornerLoop,
	blendAt []map[int]*offset2d.Blend, corners []map[int]*cbCorner, rewritten momentinput.Profile,
	seams [][]seamMove) (prismrecord.CarrierRecord, error) {
	out := prismrecord.ZeroCarrierRecord(rewritten)
	// The record's profile must equal the payload's loop for loop, and
	// momentinput.ExactProfileEqual tells a nil hole list from an empty one.
	if rewritten.Holes != nil && out.Profile.Holes == nil {
		out.Profile.Holes = []sectionrecord.LoopRecord{}
	}
	newLoops := append([]sectionrecord.LoopRecord{rewritten.Outer}, rewritten.Holes...)
	for li := range sections {
		if len(blendAt[li]) == 0 {
			copy(out.Segment[li], cr.Segment[li])
			copy(out.Junction[li], cr.Junction[li])
			continue
		}
		walks := loops[li].Walks
		n := len(walks)
		nseg := len(sections[li].Segments)
		var seq []emitted
		for i, w := range walks {
			eps, startJ, endJ, _ := walkEntries(cr, li, w, nseg)
			cs, ce := corners[li][i], corners[li][(i+1)%n]
			if len(seq) >= len(newLoops[li].Segments) {
				return prismrecord.CarrierRecord{}, cbRefusal("CB0", "", `the rewritten loop is shorter than its walks`)
			}
			e := emitted{seg: newLoops[li].Segments[len(seq)], seam: seams[li][len(seq)]}
			e.unchanged = cs == nil && ce == nil && len(w.Segs) == 1 && wholeSegment(sections[li].Segments[w.Segs[0]])
			e.sd = proofbound.AbsSumUpper(startJ, proofbound.WalkEndBoundAllow(w.StartBound))
			if cs != nil {
				e.sd = cs.dfB
			}
			e.ed = proofbound.AbsSumUpper(endJ, proofbound.WalkEndBoundAllow(w.EndBound))
			if ce != nil {
				e.ed = ce.dfA
			}
			e.sd = proofbound.AbsSumUpper(e.sd, e.seam.Read)
			switch {
			case e.unchanged:
				e.eps = proofbound.AbsSumUpper(eps, e.seam.Carrier)
			case isArc(e.seg):
				addRadialResidual(&e)
				ds := e.sd
				if !naturalArc(e.seg) {
					ds = e.ed
				}
				e.eps = proofbound.AbsSumUpper(eps, eps, ds, e.seam.Carrier)
			default:
				e.eps = math.Max(e.sd, e.ed)
			}
			seq = append(seq, e)
			if ce != nil && blendAt[li][(i+1)%n].Connector != nil {
				if len(seq) >= len(newLoops[li].Segments) {
					return prismrecord.CarrierRecord{}, cbRefusal("CB0", "", `the rewritten loop is shorter than its connectors`)
				}
				sm := seams[li][len(seq)]
				k := emitted{seg: newLoops[li].Segments[len(seq)], eps: proofbound.AbsSumUpper(ce.connEps, sm.Carrier),
					sd: proofbound.AbsSumUpper(ce.dfA, sm.Read), ed: ce.dfB, connector: true, seam: sm}
				if isArc(k.seg) {
					addRadialResidual(&k)
				}
				seq = append(seq, k)
			}
		}
		if len(seq) != len(newLoops[li].Segments) {
			return prismrecord.CarrierRecord{}, cbRefusal("CB0", "", `the rewritten loop's segments do not match its walks`)
		}
		// Walk index of each emitted segment, to find the receiver's junction.
		walkAt := make([]int, len(seq))
		k := 0
		for i := range walks {
			walkAt[k] = i
			k++
			if corners[li][(i+1)%n] != nil && blendAt[li][(i+1)%n].Connector != nil {
				walkAt[k] = -1
				k++
			}
		}
		for k, e := range seq {
			out.Segment[li][k] = e.eps
			next := seq[(k+1)%len(seq)]
			out.Junction[li][k] = junctionEntry(cr, sections[li], walks, li, nseg, walkAt, k, e, next)
		}
	}
	for l := range out.Segment {
		for j := range out.Segment[l] {
			if proofbound.IsNonFinite(out.Segment[l][j]) || proofbound.IsNonFinite(out.Junction[l][j]) {
				return prismrecord.CarrierRecord{}, cbRefusal("CB7", "", `a displacement of the rewritten section is not finite`)
			}
		}
	}
	h := out.HMax()
	if proofbound.IsNonFinite(h) {
		return prismrecord.CarrierRecord{}, cbRefusal("CB7", "", `an arc of the rewritten section fails Lemma H's condition, so its denoted section cannot be matched to it`)
	}
	out.SectionDelta = math.Max(out.Max(), h)
	return out, nil
}

// junctionEntry is the entry of the junction after emitted segment k.
func junctionEntry(cr prismrecord.CarrierRecord, section sectionrecord.LoopRecord, walks []survey2d.SideWalk, li, nseg int,
	walkAt []int, k int, e, next emitted) float64 {
	wi, wn := walkAt[k], walkAt[(k+1)%len(walkAt)]
	if wi < 0 || wn < 0 {
		return math.Max(e.ed, next.sd) // through a connector: a chord or a point
	}
	_, _, endJ, _ := walkEntries(cr, li, walks[wi], nseg) //nolint:dogsled // only the end junction is read
	if e.unchanged && next.unchanged && next.seam.Read == 0 {
		return endJ
	}
	// A seam snap moved next's start: the receiver's path there may close
	// through two lines' corner, whose legs pathTau measures against the
	// snapped point (§7.7's path rule).
	prevR := section.Segments[walks[wi].Segs[len(walks[wi].Segs)-1]]
	nextR := section.Segments[walks[wn].Segs[0]]
	if tau, ok := pathTau(prevR, nextR, e.seg, next.seg); ok {
		return proofbound.AbsSumUpper(endJ, tau)
	}
	return math.Max(e.ed, next.sd)
}

func isArc(seg sectionrecord.CurveSegment) bool {
	_, ok := seg.(sectionrecord.ArcSeg)
	return ok
}

// naturalArc reports an ArcSeg walked Start → End (its natural direction).
func naturalArc(seg sectionrecord.CurveSegment) bool {
	a, ok := seg.(sectionrecord.ArcSeg)
	return ok && a.TStart == 0 && a.TEnd == 1
}

// addRadialResidual adds an emitted ArcSeg's radial residual ρ to the
// displacement of its read end: Start's radius on End's bearing, which is
// the walk's end for a natural arc and its start for a reversed one (§3.2).
func addRadialResidual(e *emitted) {
	arc, ok := e.seg.(sectionrecord.ArcSeg)
	if !ok {
		return
	}
	rho := circularbounds.ArcRadialResidualUpper(arc)
	if naturalArc(arc) {
		e.ed = proofbound.AbsSumUpper(e.ed, rho)
		return
	}
	e.sd = proofbound.AbsSumUpper(e.sd, rho)
}

// pathTau is the path rule's distance (§3.2, CR3) between a junction's closing
// path in the receiver (prevR to nextR) and in the rewrite (prevN to nextN):
// each path closes by the evaluator's own rule (momentinput.JunctionCorner)
// and is a polyline whose breakpoints are matched. ok=false when the four
// segments are not all lines or a read end is not exact.
func pathTau(prevR, nextR, prevN, nextN sectionrecord.CurveSegment) (float64, bool) {
	r, okR := junctionPath(prevR, nextR)
	n, okN := junctionPath(prevN, nextN)
	if !okR || !okN {
		return 0, false
	}
	return r.distance(n)
}

// jpath is a junction's closing path: its two read ends and, where the
// supports' corner closes it, that corner.
type jpath struct {
	p, q   freeform.RatPoint
	corner *freeform.RatPoint
}

func junctionPath(prev, next sectionrecord.CurveSegment) (jpath, bool) {
	pl, ok1 := prev.(sectionrecord.LineSeg)
	nl, ok2 := next.(sectionrecord.LineSeg)
	if !ok1 || !ok2 {
		return jpath{}, false
	}
	_, p := momentinput.ExactLineEnds(pl)
	q, _ := momentinput.ExactLineEnds(nl)
	if p == nil || q == nil || p.U == nil || q.U == nil {
		return jpath{}, false
	}
	path := jpath{p: *p, q: *q}
	if corner, ok := momentinput.JunctionCorner(prev, next, p, q); ok {
		path.corner = &corner
	}
	return path, true
}

func ratDistUp(a, b freeform.RatPoint) float64 { return pointDistUp(a.U, a.V, b.U, b.V) }

// distance is the largest breakpoint distance between two paths. A chord
// matched to a corner path takes the chord point at the corner's arc-length
// fraction as its breakpoint.
func (a jpath) distance(b jpath) (float64, bool) {
	tau := math.Max(ratDistUp(a.p, b.p), ratDistUp(a.q, b.q))
	switch {
	case a.corner == nil && b.corner == nil:
	case a.corner != nil && b.corner != nil:
		tau = math.Max(tau, ratDistUp(*a.corner, *b.corner))
	case a.corner != nil:
		d, ok := cornerVsChord(a, b)
		if !ok {
			return 0, false
		}
		tau = math.Max(tau, d)
	default:
		d, ok := cornerVsChord(b, a)
		if !ok {
			return 0, false
		}
		tau = math.Max(tau, d)
	}
	return tau, true
}

// cornerVsChord is the distance from a corner path's corner to the chord's
// point at the corner's arc-length fraction, over the fraction's enclosure.
func cornerVsChord(withCorner, chord jpath) (float64, bool) {
	c := withCorner.corner
	d1, ok1 := norm(ivPoint(new(big.Rat).Sub(c.U, withCorner.p.U)), ivPoint(new(big.Rat).Sub(c.V, withCorner.p.V)))
	d2, ok2 := norm(ivPoint(new(big.Rat).Sub(withCorner.q.U, c.U)), ivPoint(new(big.Rat).Sub(withCorner.q.V, c.V)))
	if !ok1 || !ok2 {
		return 0, false
	}
	frac, ok := d1.quo(d1.add(d2))
	if !ok {
		return 0, false
	}
	worst := 0.0
	for _, f := range []*big.Rat{frac.lo, frac.hi} {
		x := new(big.Rat).Add(chord.p.U, new(big.Rat).Mul(f, new(big.Rat).Sub(chord.q.U, chord.p.U)))
		y := new(big.Rat).Add(chord.p.V, new(big.Rat).Mul(f, new(big.Rat).Sub(chord.q.V, chord.p.V)))
		worst = math.Max(worst, pointDistUp(c.U, c.V, x, y))
	}
	return worst, true
}

// wholeSegment reports a line or arc over its natural range, or a whole
// circle.
func wholeSegment(seg sectionrecord.CurveSegment) bool {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		return prismcells.WholeSegmentRange(s.TStart, s.TEnd)
	case sectionrecord.ArcSeg:
		return prismcells.WholeSegmentRange(s.TStart, s.TEnd)
	case sectionrecord.CircleSeg:
		return prismcells.WholeSegmentRange(s.TStart, s.TEnd)
	default:
		return false
	}
}

// carrierAuditTransfer is CB7: with h_loop[l] Lemma H's bound of the result
// record, 4·max h_loop is below the rewritten section's contact floor, every
// two segments sharing no vertex are certified more than the floor apart, and
// every loop's |signed area| exceeds its displacement area, so the audit's S7,
// S8 and S9 verdicts hold for the denoted section.
func carrierAuditTransfer(budget *proofbound.WorkBudget, record prismrecord.CarrierRecord) error {
	if record.SectionDelta == 0 || !rowOn("CB7") {
		return nil
	}
	profile := record.Profile
	loops := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	// With F_out the rewritten section's contact floor: 4·max h_loop < F_out and
	// every two segments sharing no vertex, kept (cloned) loops against
	// rewritten ones included, have certified gap G > F_out (§6.2, §7.6).
	apart, why, err := prismcells.CertifiedContact(budget, loops, record.HMax())
	if err != nil {
		return err
	}
	if !apart {
		return cbRefusal("CB7", "", `%s`, why)
	}
	for li, loop := range loops {
		// The enclosure already carries the junction gaps and the displacement
		// area; the loop's signed area must clear zero as an interval.
		enc, ok, err := prismcells.LoopAreaEnclosure(budget, loop, record.LoopH(li))
		if err != nil {
			return err
		}
		if !ok {
			return cbRefusal("CB7", "", `loop %d's area cannot be enclosed`, li)
		}
		if enc.Lo.Sign() > 0 || enc.Hi.Sign() < 0 {
			continue
		}
		lo, _ := enc.Lo.Float64()
		hi, _ := enc.Hi.Float64()
		return cbRefusal("CB7", "", `loop %d's area lies in [%g, %g], which includes zero`, li, lo, hi)
	}
	return nil
}
