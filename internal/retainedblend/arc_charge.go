package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file charges an arc with C3 ends (agent-docs/boss-root-overflow-design.md
// §5.5.6): σ, how far a row-end vertex or a cap vertex on G* may lie from the true
// crossing, and the arc's area charge, which replaces the rim's.

// piRound is π rounded up.
const piRound = 3.1415926535897936

// endCurveTurn is the most a C3 end curve's tangent turns: π/2 rounded up, as the curve is monotone in both of its
// unrolled coordinates.
const endCurveTurn = 1.5707963267948968

// down is x rounded down past a few ulps of rounding.
func down(x float64) float64 { return x * (1 - 8*ulp) }

// sigmaEnd holds what σ reads of one C3 end besides the row: the end's error e, G*'s
// distance d' and radius r_G, and the profile's least and largest row radius.
type sigmaEnd struct {
	e, d, rG, rhoMin, rhoMax float64
	// sigma2 is the row-free bound σ₂.
	sigma2 float64
}

// aE is a_e: how far e moves arg = (ρ² + d'² − r_G²)/(2ρd') when ρ moves by e, d' by
// 2e and r_G by e, with S² = ρ_max² + d'² + r_G², the first-order bound
// (1 + 2⁻¹⁰)·e·(S²/(2ρ_min²d') + S²/(ρ_min·d'²) + r_G/(ρ_min·d')). The end refuses
// unless e ≤ 2⁻²⁰·min(ρ_min, d', r_G), which keeps the second-order remainder inside the
// factor.
func aE(e, rhoMin, rhoMax, d, rG float64) float64 {
	s2 := up(up(rhoMax*rhoMax) + up(d*d) + up(rG*rG))
	t1 := up(s2 / down(2*rhoMin*rhoMin*d))
	t2 := up(s2 / down(rhoMin*d*d))
	t3 := up(rG / down(rhoMin*d))
	return up((1 + 0x1p-10) * e * up(t1+t2+t3))
}

// newSigmaEnd reads σ₂ = ρ_max·(H(a_e) + asin↑(2e/(d' − 2e))) of a C3 end. It refuses an
// end whose e leaves the second-order remainder of a_e outside its factor.
func newSigmaEnd(e, d, rG, rhoMin, rhoMax float64) (sigmaEnd, error) {
	if !(e <= tangentTilt*math.Min(rhoMin, math.Min(d, rG))) {
		return sigmaEnd{}, fmt.Errorf("%w: an arc end's tags or cliff joint move its crossing too far",
			decaderr.ErrUnsupported)
	}
	turn := math.Asin(math.Min(1, up(2*e/down(d-2*e))))
	s2 := up(rhoMax * up(hUp(aE(e, rhoMin, rhoMax, d, rG))+up(turn)))
	return sigmaEnd{e: e, d: d, rG: rG, rhoMin: rhoMin, rhoMax: rhoMax, sigma2: s2}, nil
}

// at is σ = min(σ₁, σ₂) + e of a row whose crossing has the rounded-down sine qRow.
// σ₁ = 2e(1 + 2⁻¹⁰)/q_row is the first-order move of a crossing at that sine when both
// circles move by at most e, used only when e ≤ 2⁻¹²·q_row²·min(ρ_min, r_G), which keeps
// the curvature remainder inside the factor; the wall's row (q_row = 0) takes σ₂.
func (s sigmaEnd) at(qRow float64) float64 {
	m := s.sigma2
	if qRow > 0 && s.e <= 0x1p-12*qRow*qRow*math.Min(s.rhoMin, s.rG) {
		m = math.Min(m, up(2*s.e*(1+0x1p-10)/down(qRow)))
	}
	return proofbound.AbsSumUpper(m, s.e)
}

// rowQ is a rounded-down lower bound of sin∠OPC, cylCurve.q, at the row of radius rho:
// the cosine's rounding is charged as crossingSine charges it.
func (c *cylCurve) rowQ(rho float64) float64 {
	k := math.Abs((rho*rho + c.rG*c.rG - c.d*c.d) / (2 * rho * c.rG))
	errK := up(16 * ulp * (rho*rho + c.rG*c.rG + c.d*c.d) / (2 * rho * c.rG))
	k = up(k + errK)
	if !(k < 1) {
		return 0
	}
	return down(math.Sqrt(down(1 - k*k)))
}

// sigmaEndAt builds end i's σ reading from its section and the profile's row radii.
func sigmaEndAt(c *cylEnd, rho0, rho1 float64) (sigmaEnd, error) {
	return newSigmaEnd(c.tan.e, c.d, c.rG, math.Min(rho0, rho1), math.Max(rho0, rho1))
}

// chargeTangentEnds adds σ to every row-end vertex of both C3 ends, the wall's cut
// vertices at each end included, and records each end's largest σ for the cap's own
// vertices (capFacets). It returns the largest σ over both ends. rows are the arc's rows
// and ss their profile parameters.
func (a arcRim) chargeTangentEnds(w *work, rows []arcRow, ss []float64, rowCols [][]colPt, ends []endSection) (float64, error) {
	sigmaMax := 0.0
	for i := range 2 {
		c := a.cyl[i]
		rho0, rho1 := rows[0].rho, rows[len(rows)-1].rho
		se, err := sigmaEndAt(c, rho0, rho1)
		if err != nil {
			return 0, err
		}
		cv := ends[i].cyl
		endMax := 0.0
		for j, row := range rows {
			q := 0.0
			if ss[j] < 1 {
				q = cv.rowQ(row.rho)
			}
			sg := se.at(q)
			cols := rowCols[j]
			idx := cols[0].idx
			if i == 1 {
				idx = cols[len(cols)-1].idx
			}
			w.beta[idx] = proofbound.AbsSumUpper(w.beta[idx], sg)
			endMax = max(endMax, sg)
		}
		c.tan.sigmaMax = endMax
		sigmaMax = max(sigmaMax, endMax)
	}
	return sigmaMax, nil
}

// chargePiece is one rewritten piece of an arc's blend: its perimeter P, boundary
// turning Φ and exact area A in its own unrolled coordinates, and its smallest radius of
// curvature rMin, zero for a plane piece, which has no curvature term.
type chargePiece struct {
	perimeter, turning, area, rMin float64
}

// arcAreaCharge is the area bound the C3 ends add to an arc's rewrite: per piece
// 2P·σ_max + (π + Φ)·σ_max² + A·e/r_min. A boundary that moves by at most σ_max sweeps a
// strip of at most 2P·σ_max + (π + Φ)·σ_max² (the strip is the boundary's σ_max-tube), and
// a radius that moves by at most e moves a curved piece's area by A·e/r_min. The bound
// holds for a piece that is not convex.
func arcAreaCharge(pieces []chargePiece, sigmaMax, e float64) float64 {
	total := 0.0
	for _, p := range pieces {
		tube := proofbound.AbsSumUpper(
			proofbound.ProductUpper(proofbound.ProductUpper(2, p.perimeter), sigmaMax),
			proofbound.ProductUpper(proofbound.ProductUpper(proofbound.AbsSumUpper(piRound, p.turning), sigmaMax), sigmaMax))
		total = proofbound.AbsSumUpper(total, tube)
		if p.rMin > 0 {
			total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(p.area, proofbound.DivUpper(e, p.rMin)))
		}
	}
	return total
}

// endLegs are the lengths, in end i's unrolled coordinates, of the legs of the cap's
// end curve: |Δu|, the arc of G* from the wall's tangent point to P_A, and |Δv|, the
// profile's height. The curve is monotone in both, so its length is at most their sum.
func endLegs(cv *cylCurve, tan *tangentEnd, rho0, z0, z1 float64) (du, dv float64) {
	dm := 2*cv.ts.eps*(cv.ts.zHi+cv.rG) + cv.ts.eD
	errPsi, _ := cv.tangentErrors(rho0, rho0, dm)
	psi := math.Abs(cv.psi(rho0) - tan.psiWall())
	du = up(cv.rG * up(psi+hUp(errPsi)))
	dv = up(math.Abs(z1 - z0))
	return du, dv
}

// arcChargeOf collects the pieces an arc with C3 ends rewrites and charges them
// (arcAreaCharge). trueArea is the blend's exact area, tb the wall's setback.
func (a arcRim) arcChargeOf(rows []arcRow, ends []endSection, tb, trueArea, sigmaMax float64,
	capAreas [2]float64) float64 {
	m := len(rows) - 1
	t0, t1 := rows[0].theta[0], rows[0].theta[1]
	dTheta0, dThetaM := math.Abs(t1-t0), math.Abs(rows[m].theta[1]-rows[m].theta[0])
	rhoA := rows[0].rho
	zA, zW := rows[0].z, rows[m].z

	e, rhoMin := 0.0, math.Inf(1)
	for _, r := range rows {
		rhoMin = min(rhoMin, r.rho)
	}
	var du, dv [2]float64
	for i := range 2 {
		c := a.cyl[i]
		e = max(e, c.tan.e)
		du[i], dv[i] = endLegs(ends[i].cyl, c.tan, rows[0].rho, zA, zW)
	}
	// The plane's lost region: the rim's arc, the setback arc and the two arcs of G*.
	pA := proofbound.AbsSumUpper(proofbound.ProductUpper(a.radius, a.span),
		proofbound.ProductUpper(rhoA, dTheta0), du[0], du[1])
	phiA := proofbound.AbsSumUpper(2*math.Pi, a.span, dTheta0, proofbound.DivUpper(du[0], a.cyl[0].rG),
		proofbound.DivUpper(du[1], a.cyl[1].rG))
	// The wall's band: a rectangle in the wall's unrolled coordinates.
	pB := proofbound.AbsSumUpper(proofbound.ProductUpper(2, proofbound.ProductUpper(a.radius, a.span)), 2*tb)
	aB := proofbound.ProductUpper(proofbound.ProductUpper(a.radius, a.span), tb)
	// The torus strip: its two arcs and its two end curves.
	pT := proofbound.AbsSumUpper(proofbound.ProductUpper(rows[0].rho, dTheta0),
		proofbound.ProductUpper(a.radius, dThetaM), du[0], dv[0], du[1], dv[1])
	// Each end curve is monotone in both unrolled coordinates, so its tangent turns by at most π/2 and the piece's
	// turning includes it: the strip has two end curves, each cap one, and the region A loses none.
	phiT := proofbound.AbsSumUpper(2*math.Pi, dTheta0, dThetaM, 2*endCurveTurn)
	pieces := []chargePiece{
		{perimeter: pA, turning: phiA, area: a.planeRegion(rows)},
		{perimeter: pB, turning: 2 * math.Pi, area: aB, rMin: a.radius},
		{perimeter: pT, turning: phiT, area: trueArea, rMin: rhoMin},
	}
	// Each cap: V to P_A along G*, the generator up to the wall's cut and the end curve.
	for i := range 2 {
		c := a.cyl[i]
		pCap := proofbound.AbsSumUpper(proofbound.ProductUpper(2, du[i]), proofbound.ProductUpper(2, dv[i]))
		pieces = append(pieces, chargePiece{perimeter: pCap, turning: 2*math.Pi + endCurveTurn, area: capAreas[i], rMin: c.rG})
	}
	return arcAreaCharge(pieces, sigmaMax, e)
}
