package capblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file composes agent-docs/tessellation-reach-design.md §7.1 for a
// cap-loop chamfer whose loops hold an arc restated on its bisector centre
// (capband.OffRadiusShift): the ideal polyhedron B1′ stands on the restated
// section X′, so its stations move from the held mesh's references by at
// most each arc's shift, its chord segments grow by at most a Steiner margin
// of that shift, and the bodies X and X′ denote differ by at most twice the
// shift times the perimeter of every level set the restated loop region's
// signed distance takes.

// bandHeightUpper is the axial height over which a loop's band corners
// charge: a draft view's whole sweep, or the sum of the loop's chamfered
// caps' side setbacks.
func (cbp Payload) bandHeightUpper(lm *LoopMesh) float64 {
	if cbp.Draft {
		return cbp.DraftBandHeightUpper()
	}
	h := 0.0
	if lm.OnStart {
		h = proofbound.AbsSumUpper(h, cbp.Start.AxialUpper())
	}
	if lm.OnEnd {
		h = proofbound.AbsSumUpper(h, cbp.End.AxialUpper())
	}
	return h
}

// loopReach bounds |τ|, the in-plane offset a loop's sections take at any
// level: the larger chamfered cap's |dc| + dcDelta, zero for a loop no band
// chamfers.
func (cbp Payload) loopReach(lm *LoopMesh) float64 {
	reach := 0.0
	for _, on := range []struct {
		yes bool
		s   Setback
	}{{lm.OnStart, cbp.Start}, {lm.OnEnd, cbp.End}} {
		if on.yes {
			reach = math.Max(reach, proofbound.AbsSumUpper(on.s.Dc, on.s.DcDelta))
		}
	}
	return reach
}

// OffRadiusMotion is the largest restatement shift of any loop's arcs, the
// distance every vertex of B1′ may stand from the point the held mesh's own
// motion bound is measured against. It is zero for a payload that restates
// nothing (cornerClass) or a section with no off-radius arc.
func OffRadiusMotion(cbp Payload, lms []LoopMesh) (float64, error) {
	if _, restate := cbp.cornerClass(); !restate {
		return 0, nil
	}
	worst := 0.0
	for li := range lms {
		terms, ok := capband.OffRadiusLoop(lms[li].Loop, cbp.loopReach(&lms[li]))
		if !ok {
			return 0, fmt.Errorf(`%w: this %s's restated arcs state no finite shift`, decaderr.ErrUnsupported, cbp.Noun())
		}
		worst = math.Max(worst, terms.Shift)
	}
	return worst, nil
}

// OffRadiusVolume is §7.1's volume charge over the body's height H: for each
// loop holding a restated arc, 2·Δ·Perimeter (the bodies X and X′ denote)
// plus, per restated arc, 4π·(R_hi + reach)·Δ + n·π·Δ² (each chord segment of
// X′ lies inside the segment of X it pairs with grown by Δ, whose area grows
// by at most its perimeter times Δ plus πΔ²), with n the loop's side-ring
// sample count, which bounds the arc's chord count. It is zero where nothing
// is restated.
func OffRadiusVolume(cbp Payload, lms []LoopMesh) (float64, error) {
	if _, restate := cbp.cornerClass(); !restate {
		return 0, nil
	}
	// 3.1416 > π and 12.5664 > 4π.
	const piUpper, fourPiUpper = 3.1416, 12.5664
	height := cbp.DraftBandHeightUpper()
	total := 0.0
	for li := range lms {
		lm := &lms[li]
		reach := cbp.loopReach(lm)
		terms, ok := capband.OffRadiusLoop(lm.Loop, reach)
		if !ok {
			return 0, fmt.Errorf(`%w: this %s's restated arcs state no finite shift`, decaderr.ErrUnsupported, cbp.Noun())
		}
		if terms.Arcs == 0 {
			continue
		}
		perturb := proofbound.ProductUpper(2, proofbound.ProductUpper(terms.Shift, terms.Perimeter))
		margin := proofbound.ProductUpper(fourPiUpper, proofbound.ProductUpper(proofbound.AbsSumUpper(terms.RadiusHi, reach), terms.Shift))
		corners := proofbound.ProductUpper(float64(len(lm.SidePts)), proofbound.ProductUpper(piUpper, proofbound.ProductUpper(terms.Shift, terms.Shift)))
		chord := proofbound.ProductUpper(float64(terms.Arcs), proofbound.AbsSumUpper(margin, corners))
		total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(height, proofbound.AbsSumUpper(perturb, chord)))
	}
	if proofbound.IsNonFinite(total) {
		return 0, fmt.Errorf(`%w: this %s's restatement charge is not finite`, decaderr.ErrUnsupported, cbp.Noun())
	}
	return total, nil
}
