package capblend

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file holds the cap-loop chamfer's constructed-geometry gates
// (docs/modify-reach-design.md Table SX) and the mixed offset section they
// audit.

// RequireCornerLoci is SX14's corner-locus half
// (docs/modify-reach-design.md §4 stage 6). At a miter corner where a circular
// wall meets its neighbour, the cap-level foot rides a conic locus as the
// offset grows, and the band's slant ruling and its circular patch's
// chord-locus term both read a proven bound on that locus
// (capband.MiterLocusUpper, capband.MiterLocusSliverFlux). Two corners leave
// those bounds unbuildable, because the offset carriers' discriminant there is
// zero or its enclosure reaches zero:
//
//   - a tangent cusp, where a line touches the circle but the walls run in
//     opposite directions, so the corner rule does not read it as G1. The
//     carriers meet in a double root at the corner itself and the foot leaves
//     it at unbounded speed (capcontour.LineCircleLocusSpeedUpper's momentary
//     fold);
//   - a corner that turns past the G1 dead zone, yet so slightly that the
//     carriers meet in a near-double root. The arc's own radius is enclosed
//     only to its RadiusBound, and that width exceeds the discriminant, so
//     whether the denoted carriers meet at all is decided below the record's
//     own precision.
//
// The check runs before the body is assembled, so the refusal names the
// corner rather than surfacing as an unbounded measurement. A G1 join and a
// reflex corner's connector arc read no locus and are not checked.
func RequireCornerLoci(budget *proofbound.WorkBudget, cbp Payload) error {
	for li, loop := range cbp.Loops() {
		if !cbp.StartLoops[li] && !cbp.EndLoops[li] {
			continue
		}
		setback := cbp.LoopSetback(li)
		if setback.Dc <= 0 {
			continue
		}
		cl, err := OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
		if err != nil {
			return err
		}
		joins, err := cbp.OffsetJoins(budget, li, cl, setback.Dc)
		if err != nil {
			return err
		}
		walks := cl.Walks
		n := len(walks)
		for i, j := range joins {
			prev, cur := walks[(i+n-1)%n], walks[i]
			if j.Arc || j.G1 || (!prev.IsCircular() && !cur.IsCircular()) {
				continue
			}
			_, ok, err := capband.MiterLocusUpper(budget, prev, cur, j.VertU, j.VertV, setback.AxialUpper(), setback.Dc, setback.DcDelta)
			if err != nil {
				return err
			}
			if !ok {
				return fmt.Errorf(`%w: loop %d of this cap-loop chamfer has a corner at (%v, %v) where a circular wall meets its neighbour tangentially or nearly so without running on as one smooth wall, and this evaluator cannot bound the path the corner's offset foot takes there, so no cap-level ruling at that corner can be built with a proven bound`, decaderr.ErrUnsupported, li, j.VertU, j.VertV)
			}
		}
	}
	return nil
}

// RequireLevelsSeparate is SX13's AXIAL half (Table SX,
// docs/modify-reach-design.md §4 stage 6 and §8.3), the sibling of
// capband.BandRadius. The band has two directrices and the
// setback displaces both: `d` in the plane, which capband.BandRadius proves survived
// float64 at each circular wall's own radius, and `d` along the sweep, which
// carries the original loop from the cap level to the side level. A tall enough
// sweep puts `d` under the float64 spacing of that coordinate, and `z1 - d`
// rounds back onto `z1` (or `z0 + d` onto `z0`).
//
// What that returns is not a coarse body but a wrong one, for the same reason
// the radial collapse is. Every patch of the band comes out flat IN the cap
// plane, and a Plane carries its normal with no bound, so each of those faces
// asserts the chamfer's 45-degree taper as a fact about geometry that has none —
// which is exactly what the DX7 undercut survey reads off the surface. (The
// volume is not what refuses the call: the collapsed level reports the correctly
// rounded volume of the true chamfered solid, and its bound honestly charges the
// collapse.) The requested body exists — a real prism with a real, if tiny,
// chamfer — and only float64 cannot name its side level at that sweep
// coordinate, which is §4's decaderr.ErrUnsupported side of the existence test.
//
// The axial half is a fact about the sweep interval and the cap's own side
// setback ds alone (§8.3.1), so it is decided once per chamfered cap rather
// than per wall. SX7's band-reach gate above is the opposite failure on the
// same two numbers and never overlaps this one: SX7 refuses a setback so LARGE
// beside the sweep that the band passes the far end, this one a setback so
// SMALL beside the sweep's own coordinates that the level it displaces does not
// move.
func RequireLevelsSeparate(cbp Payload) error {
	refuse := func(which string, ds, level float64) error {
		return fmt.Errorf(`%w: the chamfer's side setback %v mm is below the float64 spacing of the %s cap's own sweep level %v mm, so the band's side level rounds back onto the cap level and every patch is emitted flat in the cap plane; a wider setback or a shorter sweep states a chamfer this evaluator can build`, decaderr.ErrUnsupported, ds, which, level)
	}
	if ds := cbp.Start.Ds; anyLoopSelected(cbp.StartLoops) && cbp.Z0+ds == cbp.Z0 {
		return refuse(`start`, ds, cbp.Z0)
	}
	if ds := cbp.End.Ds; anyLoopSelected(cbp.EndLoops) && cbp.Z1-ds == cbp.Z1 {
		return refuse(`end`, ds, cbp.Z1)
	}
	return nil
}

// AuditSetbackSpan runs SX6 and the stage-6 offset audit again at the
// top of each cap's setback span (docs/modify-reach-design.md §8.3.1). A
// setback stated in a unit other than millimetres is held as a float dc some
// dcDelta away from the value the caller stated, so the stated value is any
// point of [dc − dcDelta, dc + dcDelta]. The audit at dc certifies every
// offset up to dc and no further: SX12's monotonicity argument (a crossing or
// contact anywhere in the offset family occurs no later than at its largest
// member) runs the other way only. Auditing the offset at the span's top, the
// smallest float at or above dc + dcDelta, certifies every offset the span
// holds. A setback stated in millimetres has a one-point span and runs no
// second audit.
//
// A refusal here means the held dc builds and some point of the span does
// not, so this evaluator cannot decide whether the stated setback names a
// body: it is decaderr.ErrUnsupported whichever sentinel the audit itself raised, and
// the audit's own error is folded in with %v so the refusal answers to one
// sentinel only.
func AuditSetbackSpan(budget *proofbound.WorkBudget, profile momentinput.Profile, cbp Payload) error {
	top, ok := cbp.setbackSpanTop()
	if !ok {
		return nil
	}
	mixed, err := MixedOffsetProfile(budget, top)
	if err == nil {
		err = offset2d.AuditOffsetSectionBudget(budget, profile, mixed)
	}
	if err == nil {
		return nil
	}
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return fmt.Errorf(`%w: the chamfer setback across the cap converts to a float within its own rounding of the stated value, and at the top of that span the cap contour fails the offset audit (%v); this evaluator cannot decide whether the stated setback builds`, decaderr.ErrUnsupported, err)
}

// setbackSpanTop is cbp with each cap's dc moved to the top of its setback
// span: the smallest float at or above dc + dcDelta, formed exactly. ok is
// false when neither cap's dc carries a conversion rounding, so the span is
// the held point and nothing differs.
func (cbp Payload) setbackSpanTop() (Payload, bool) {
	top := func(s Setback) (Setback, bool) {
		if !(s.DcDelta > 0) {
			return s, false
		}
		rd, rw := proofarith.FloatRat(s.Dc), proofarith.FloatRat(s.DcDelta)
		if rd == nil || rw == nil {
			s.Dc = math.Inf(1)
			return s, true
		}
		s.Dc = proofbound.RatFloatUp(new(big.Rat).Add(rd, rw))
		return s, true
	}
	out := cbp
	var okStart, okEnd bool
	out.Start, okStart = top(cbp.Start)
	out.End, okEnd = top(cbp.End)
	return out, okStart || okEnd
}

// anyLoopSelected reports whether a per-cap loop set names at least one loop.
func anyLoopSelected(loops map[int]bool) bool {
	for _, on := range loops {
		if on {
			return true
		}
	}
	return false
}

// MixedOffsetProfile offsets exactly the loops cbp chamfers (the union of its
// startLoops/endLoops — a loop chamfered on either or both caps takes one
// in-plane offset, loopOffset's dc) into the material, leaving every other
// loop unchanged. It reuses offset2d.BuildLoop's per-feature offset unmodified.
func MixedOffsetProfile(budget *proofbound.WorkBudget, cbp Payload) (momentinput.Profile, error) {
	profile := cbp.Profile
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, profile)
	if err != nil {
		return momentinput.Profile{}, err
	}
	orig := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	out := make([]sectionrecord.LoopRecord, len(orig))
	for li := range orig {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return momentinput.Profile{}, err
		}
		if !cbp.StartLoops[li] && !cbp.EndLoops[li] {
			out[li] = sectionrecord.CloneLoopRecord(orig[li])
			continue
		}
		var segs []sectionrecord.CurveSegment
		if cbp.Fillet && (len(loops[li].Walks) != 1 || !loops[li].Walks[0].Closed) {
			joins, joinErr := filletband.OffsetJoins(budget, loops[li].Walks, cbp.LoopOffset(li), cbp.LoopSetback(li).DcDelta, offset2d.ShellTol)
			if joinErr != nil {
				return momentinput.Profile{}, offset2d.InLoop(joinErr, li)
			}
			segs, err = filletband.OffsetLoop(budget, loops[li].Walks, joins, cbp.LoopOffset(li), offset2d.ShellTol)
		} else {
			segs, err = offset2d.BuildLoop(budget, loops[li].Walks, 1, cbp.LoopOffset(li), offset2d.ShellTol)
		}
		if err != nil {
			return momentinput.Profile{}, offset2d.InLoop(err, li)
		}
		out[li] = sectionrecord.LoopRecord{Segments: segs}
	}
	return momentinput.Profile{Outer: out[0], Holes: out[1:]}, nil
}
