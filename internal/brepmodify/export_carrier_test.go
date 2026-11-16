package brepmodify

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// DisableCarrierRows switches Table CB rows off (agent-docs/carrier-record-design.md
// §10) and returns the function that restores them.
func DisableCarrierRows(rows ...string) func() {
	disabledRows.Store(&rows)
	return func() { disabledRows.Store(nil) }
}

// CarrierCentre is §7.5's centre displacement (CB2) for two recorded circles
// or lines: eps is each carrier's entry, ccw each walk's sense, sign the
// offset side and root the float centre.
func CarrierCentre(a, b sectionrecord.CurveSegment, eps [2]float64, ccw [2]bool, sign float64,
	root [2]float64, r float64) (float64, bool) {
	walk := func(ccw bool) survey2d.SideWalk {
		w := survey2d.SideWalk{}
		w.Th0, w.Th1 = 0, 1
		if !ccw {
			w.Th0, w.Th1 = 1, 0
		}
		w.TanInU, w.TanInV = 1, 0
		return w
	}
	ca, okA := newCBCarrier(a, walk(ccw[0]), eps[0])
	cb, okB := newCBCarrier(b, walk(ccw[1]), eps[1])
	if !okA || !okB {
		return 0, false
	}
	c, ok := centreDisplacement(ca, cb, sign, root, r)
	return c.d, ok
}

// CarrierCB6Margin is CB6's two sides for a recorded circle with carrier entry
// eps and walk sense ccw: the offset radius R − r and tol + ε + D, with d the
// centre displacement D.
func CarrierCB6Margin(seg sectionrecord.CurveSegment, eps float64, ccw bool, sign, r, d float64) (off, need float64, ok bool) {
	w := survey2d.SideWalk{}
	w.Th0, w.Th1 = 0, 1
	if !ccw {
		w.Th0, w.Th1 = 1, 0
	}
	w.TanInU, w.TanInV = 1, 0
	c, okc := newCBCarrier(seg, w, eps)
	if !okc {
		return 0, 0, false
	}
	return c.cb6Margin(sign, r, d)
}

// CarrierAuditTransfer is CB7 (§7.6) on a result record alone.
func CarrierAuditTransfer(budget *proofbound.WorkBudget, record prismrecord.CarrierRecord) error {
	return carrierAuditTransfer(budget, record)
}

// ProductDownF is the rounded-down exact product behind a quotient's denominator.
func ProductDownF(a, b float64) float64 { return productDownF(a, b) }

// CarrierChamferFootDelta is Δf of §7.5 for a chamfer foot on a recorded circle.
func CarrierChamferFootDelta(seg sectionrecord.CurveSegment, eps, rhoF, deltaJ, setback float64) (float64, bool) {
	w := survey2d.SideWalk{}
	w.Th0, w.Th1 = 0, 1
	w.TanInU, w.TanInV = 1, 0
	c, ok := newCBCarrier(seg, w, eps)
	if !ok {
		return 0, false
	}
	return c.chamferFootDelta(rhoF, deltaJ, setback), true
}

// CarrierFilletConnEps builds the fillet of corner ci of loop 0 of a displaced
// profile with every carrier entry eps and every junction entry delta, as
// CarrierLateralBlend does, and returns the connector the construction emits
// and the connector entry of §7.5. A non-nil moveFA first shifts the blend's
// foot FA by that vector, so a charge that reads FA instead of the emitted
// connector's own Start differs.
func CarrierFilletConnEps(budget *proofbound.WorkBudget, profile momentinput.Profile, ci int, eps, delta, r float64,
	moveFA *[2]float64) (sectionrecord.CurveSegment, float64, error) {
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, profile)
	if err != nil {
		return nil, 0, err
	}
	walks := loops[0].Walks
	n := len(walks)
	c := &cbCorner{}
	c.arrive, c.leave = walks[(ci+n-1)%n], walks[ci]
	c.deltaJ = delta
	c.aJ = math.Max(proofbound.WalkEndBoundAllow(c.arrive.EndBound), proofbound.WalkEndBoundAllow(c.leave.StartBound))
	segs := profile.Outer.Segments
	var okA, okB bool
	c.a, okA = newCBCarrier(segs[c.arrive.Segs[len(c.arrive.Segs)-1]], c.arrive, eps)
	c.b, okB = newCBCarrier(segs[c.leave.Segs[0]], c.leave, eps)
	if !okA || !okB {
		return nil, 0, cbRefusal("CB0", "", `carrier`)
	}
	ovA, ovB := &c.a.ov, &c.b.ov
	blend, dec, err := offset2d.FilletWithCarriers([]survey2d.SideWalk{c.arrive, c.leave}, 1, r, sectionaudit.Tolerance, ovA, ovB)
	if err != nil {
		return nil, 0, err
	}
	if moveFA != nil {
		blend.FA.U += moveFA[0]
		blend.FA.V += moveFA[1]
	}
	thetaA := c.a.theta1(c.deltaJ, c.aJ)
	if err := filletCharge(c, blend, dec, r, thetaA); err != nil {
		return nil, 0, err
	}
	return blend.Connector, c.connEps, nil
}

// SeamMove is closeSeams' charge to one segment it moved.
type SeamMove = seamMove

// CloseSeams runs closeSeams on a profile whose loops are marked rewritten.
func CloseSeams(p momentinput.Profile, rewritten []bool) (momentinput.Profile, [][]SeamMove) {
	return closeSeams(p, rewritten)
}

// CarrierKeptRecord is §7.7's record of a profile whose loop 0 is marked
// rewritten but whose every segment is kept whole, with seams as closeSeams'
// moves: the entries a seam snap adds to a kept segment and its junction.
func CarrierKeptRecord(budget *proofbound.WorkBudget, cr prismrecord.CarrierRecord, profile, rewritten momentinput.Profile,
	seams [][]SeamMove) (prismrecord.CarrierRecord, error) {
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, profile)
	if err != nil {
		return prismrecord.CarrierRecord{}, err
	}
	sections := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	blendAt := make([]map[int]*offset2d.Blend, len(loops))
	corners := make([]map[int]*cbCorner, len(loops))
	for i := range blendAt {
		blendAt[i], corners[i] = map[int]*offset2d.Blend{}, map[int]*cbCorner{}
	}
	blendAt[0][0] = nil
	return carrierRewriteRecord(cr, sections, loops, blendAt, corners, rewritten, seams)
}
