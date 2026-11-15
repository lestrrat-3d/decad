package tessellation

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// This file is agent-docs/tessellation-reach-design.md §7's "Reflex connector":
// the vertex motion and the chord term of a cap-loop chamfer's apex fan at a
// reflex corner of the material. The ideal polyhedron B1 places the fan's
// vertices as follows. The apex is the denoted junction V, already the side
// ring's B1 vertex. The two end stations PA and PB are the exact wall feet
// V + d·n̂, already the adjacent walls' B1 vertices. Interior station k is the
// radial projection of the held station onto the denoted circle of radius d
// about V. A fan triangle cut at axial fraction s then has both ends on the
// true arc of radius s·d, so B1's section is the chord polygon of the true
// section, and the existing trim and chord legs cover the fan.

// connectorMotion fills the motion of every connector station and of every
// straight wall whose start join is a connector.
//
// A straight wall that starts at a connector starts at PB, the exact foot of
// its own offset line, which the contour displacement (miter) encloses.
//
// Station 0 (PA) is the previous wall's exact foot and takes miter. An
// interior station k takes the gap from its held point to the radial
// projection on the denoted circle about the denoted junction. That gap is at
// most the gap to any point of the circle, so it is at most CapBound (the held
// station against the held circle) plus the junction's own bound plus the
// setback's rounding.
//
// The order check below is what makes the chord-polygon claim sound. B1's
// section is the chord polygon of the true section only when the projected
// stations run in order inside the arc's window, so a loop whose held
// stations do not provably do so gets +Inf, and the boolean refuses
// ErrUnsupported (agent-docs/tessellation-design.md §12, a non-finite store
// bound) instead of composing.
func connectorMotion(budget *proofbound.WorkBudget, in CapBlendMotionInput, miter float64, capMotion []float64) error {
	for i, start := range in.CapArcStart {
		if err := budget.Step(); err != nil {
			return err
		}
		if start < 0 {
			continue
		}
		if in.Joins == nil || in.CapBound == nil || in.SideBound == nil || in.SideStart == nil {
			// An input that does not state its sample record cannot place the
			// stations: leave them unbounded.
			continue
		}
		count := in.ArcCount[i]
		apex := proofbound.WalkEndBoundAllow(in.SideBound[in.SideStart[i]])
		capMotion[start] = miter
		for k := 1; k < count; k++ {
			capMotion[start+k] = proofbound.AbsSumUpper(proofbound.WalkEndBoundAllow(in.CapBound[start+k]), apex, in.DDelta)
		}
		if !connectorOrdered(in, i, start, miter, apex) {
			for k := range count {
				capMotion[start+k] = math.Inf(1)
			}
		}
	}
	return nil
}

// connectorOrdered reports whether, about the junction V of corner i, the
// held stations PA = p_0, p_1, ..., p_m = PB (p_m is the next wall's first cap
// sample) run strictly in the connector's turn direction, every interior
// station inside the sector the two feet span, and that sector is under half
// a turn. Under it the radial projections of the p_k onto the denoted circle
// about the denoted V run in the same order inside the same window, so every
// fan triangle's section is a chord of the true arc.
//
// Let u_k = p_k - V (held). The turn sign s is that of the arc's recorded
// sweep ArcTh1 - ArcTh0. The tests are exactly
//
//	s·cross(u_0, u_k) > 0 and s·cross(u_k, u_m) > 0   (interior k)
//	s·cross(u_k, u_{k+1}) > 0                         (k = 0 .. m-1)
//	s·cross(u_0, u_m) > 0
//
// Each is evaluated in exact rationals over the held floats, and must clear a
// margin for the displacement of its operands. Only the angle of p_k about the
// denoted V' matters for a projected interior station, so its error is the
// junction's bound eV. The end stations stand for the exact feet, which sit
// within miter (the contour displacement) of the held PA and PB, so they carry
// miter + eV. For vectors u, w with displacements eu, ew, the cross changes by
// at most eu·|w| + ew·|u| + eu·ew, and a test passes only when s·cross exceeds
// that. A sign that the margin cannot decide fails the check, and a
// failure is never a pass: a +Inf motion refuses the proof. It is a
// reject-only test of the held geometry against the held window; it admits a
// loop only with margin to spare and never on a residual.
func connectorOrdered(in CapBlendMotionInput, i, start int, miter, eV float64) bool {
	m := in.ArcCount[i]
	if m < 1 || len(in.CapPts) < start+m {
		return false
	}
	j := in.Joins[i]
	sign := 0
	switch {
	case in.ArcTh1[i] > in.ArcTh0[i]:
		sign = 1
	case in.ArcTh1[i] < in.ArcTh0[i]:
		sign = -1
	default:
		return false
	}
	// next is the sample that ends the connector: the corner's own wall start.
	next := in.CapWallStart[i]
	pts := make([]sectionrecord.Point2, m+1)
	errs := make([]float64, m+1)
	for k := range m {
		pts[k] = in.CapPts[start+k]
		errs[k] = eV
	}
	pts[m] = in.CapPts[next]
	endErr := proofbound.AbsSumUpper(miter, eV)
	errs[0], errs[m] = endErr, endErr
	type vec struct {
		x, y *big.Rat
		norm float64 // an upper bound of the vector's length
	}
	vs := make([]vec, m+1)
	for k, p := range pts {
		dx, dy := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
		vx, vy := proofarith.FloatRat(j.VU), proofarith.FloatRat(j.VV)
		if dx == nil || dy == nil || vx == nil || vy == nil {
			return false
		}
		vs[k] = vec{
			x: new(big.Rat).Sub(dx, vx), y: new(big.Rat).Sub(dy, vy),
			norm: distanceUpper(p, sectionrecord.Point2{U: j.VU, V: j.VV}),
		}
	}
	ok := func(a, b int) bool {
		cross := new(big.Rat).Sub(new(big.Rat).Mul(vs[a].x, vs[b].y), new(big.Rat).Mul(vs[a].y, vs[b].x))
		if sign < 0 {
			cross.Neg(cross)
		}
		margin := proofbound.AbsSumUpper(
			proofbound.ProductUpper(errs[a], vs[b].norm),
			proofbound.ProductUpper(errs[b], vs[a].norm),
			proofbound.ProductUpper(errs[a], errs[b]),
		)
		rm := proofarith.FloatRat(margin)
		return rm != nil && cross.Cmp(rm) > 0
	}
	if !ok(0, m) {
		return false
	}
	for k := 1; k < m; k++ {
		if !ok(0, k) || !ok(k, m) {
			return false
		}
	}
	for k := range m {
		if !ok(k, k+1) {
			return false
		}
	}
	return true
}

// CapBlendConnectorVolume bounds, per chamfered cap, the circular segments the
// reflex connectors' chords omit: the connector leg of
// agent-docs/tessellation-reach-design.md §7's Mchord. A chord c on a circle
// of radius ρ omits at most π³c³/(96ρ), because Δ - sin Δ ≤ Δ³/6 and
// Δ = 2·asin(c/2ρ) ≤ πc/(2ρ). Down the cone c and ρ both scale by s, so the
// area scales as s² and the cap level bounds every slice, and the term is
// the band's axial height times that sum. Cell k pairs station k with k+1, the
// last with the corner's own wall start; c is the held chord plus both ends'
// motion, and ρ is bounded below by d - dDelta. 32 bounds π³ above. A loop
// with no connector costs zero; a non-positive ρ or a non-finite motion answers
// +Inf, which the caller's SymDiff rejects.
func CapBlendConnectorVolume(bandHeight [2]float64, in CapBlendMotionInput, capPts []sectionrecord.Point2, capMotion []float64, d, dDelta float64) float64 {
	if !in.Chamfered || in.CapArcStart == nil {
		return 0
	}
	rhoLo := d - dDelta
	slice := 0.0
	for i, start := range in.CapArcStart {
		if start < 0 {
			continue
		}
		if rhoLo <= 0 {
			return math.Inf(1)
		}
		for k := range in.ArcCount[i] {
			a, b := start+k, start+k+1
			if k == in.ArcCount[i]-1 {
				b = in.CapWallStart[i]
			}
			c := proofbound.AbsSumUpper(
				distanceUpper(capPts[a], capPts[b]),
				capMotion[a], capMotion[b])
			cube := proofbound.ProductUpper(c, proofbound.ProductUpper(c, c))
			slice = proofbound.AbsSumUpper(slice, proofbound.UpRound(proofbound.ProductUpper(32.0, cube)/(96*rhoLo)))
		}
	}
	total := 0.0
	for ci, on := range []bool{in.OnStart, in.OnEnd} {
		if on {
			total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(bandHeight[ci], slice))
		}
	}
	return total
}

// distanceUpper is an upper bound of the exact distance between two held
// points: the squared distance is exact over the floats, rounded up, and the
// correctly rounded square root of it is nudged up once more. A non-finite
// point answers +Inf.
func distanceUpper(a, b sectionrecord.Point2) float64 {
	ax, ay, bx, by := proofarith.FloatRat(a.U), proofarith.FloatRat(a.V), proofarith.FloatRat(b.U), proofarith.FloatRat(b.V)
	if ax == nil || ay == nil || bx == nil || by == nil {
		return math.Inf(1)
	}
	dx, dy := new(big.Rat).Sub(ax, bx), new(big.Rat).Sub(ay, by)
	sq := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	return proofbound.UpRound(math.Sqrt(proofbound.RatFloatUp(sq)))
}
