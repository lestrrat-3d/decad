package retainedblend

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// This file charges the plane faces a route rebuilds (agent-docs/retained-edge-blend-design.md §6.3, Claim K,
// "Volume" and "A displaced plane in the area bound"): the volume the new facets of a re-triangulated plane
// face can add over the true plane, and the area a displaced plane moves in the exact pieces of a route.

// planeFacetVolume bounds Σ_t (h_t + δ)·area(t) over the facets tris of verts, from above. h_t is the largest
// exact distance of t's corners from the plane pl (its tag), |n·(v − o)|/|n|, and area(t) the exact area of t,
// both read in rationals from the held coordinates and rounded up; δ is the displacement the plane's face states.
// A facet lies within h_t of the tag plane and so within h_t + δ of the true one, and the slab between it and the
// plane holds at most that times its area. An input no rational reads gives +Inf.
func planeFacetVolume(verts []r3.Vec, tris [][3]int, pl plane, delta float64) float64 {
	n, okN := stitchflux.RatVecOf(pl.n)
	o, okO := stitchflux.RatVecOf(pl.o)
	if !okN || !okO || !(delta >= 0) {
		return math.Inf(1)
	}
	n2 := new(big.Rat)
	for k := range 3 {
		n2.Add(n2, new(big.Rat).Mul(n[k], n[k]))
	}
	norm := proofbound.RatSqrtDown(n2)
	if !(norm > 0) {
		return math.Inf(1)
	}
	dist := map[int]float64{}
	height := func(vi int) float64 {
		if d, ok := dist[vi]; ok {
			return d
		}
		q, ok := stitchflux.RatVecOf(verts[vi])
		if !ok {
			return math.Inf(1)
		}
		sum := new(big.Rat)
		for k := range 3 {
			sum.Add(sum, new(big.Rat).Mul(n[k], new(big.Rat).Sub(q[k], o[k])))
		}
		d := proofbound.DivUpper(proofbound.RatFloatUp(sum.Abs(sum)), norm)
		dist[vi] = d
		return d
	}
	total := 0.0
	for _, t := range tris {
		h := math.Max(height(t[0]), math.Max(height(t[1]), height(t[2])))
		a, okA := facetAreaUpper(verts[t[0]], verts[t[1]], verts[t[2]])
		if !okA {
			return math.Inf(1)
		}
		total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(proofbound.AbsSumUpper(h, delta), a))
	}
	return total
}

// facetAreaUpper is the area of the triangle abc, read exactly from its held corners and rounded up.
func facetAreaUpper(a, b, c r3.Vec) (float64, bool) {
	ra, okA := stitchflux.RatVecOf(a)
	rb, okB := stitchflux.RatVecOf(b)
	rc, okC := stitchflux.RatVecOf(c)
	if !okA || !okB || !okC {
		return 0, false
	}
	var u, v [3]*big.Rat
	for k := range 3 {
		u[k] = new(big.Rat).Sub(rb[k], ra[k])
		v[k] = new(big.Rat).Sub(rc[k], ra[k])
	}
	cross := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(u[i], v[j]), new(big.Rat).Mul(u[j], v[i]))
	}
	sq := new(big.Rat)
	for _, ij := range [][2]int{{1, 2}, {2, 0}, {0, 1}} {
		x := cross(ij[0], ij[1])
		sq.Add(sq, x.Mul(x, x))
	}
	// Halving is exact.
	return proofbound.RatSqrtUp(sq) / 2, true
}

// planeOfGroup reads the plane tag of source group g: its frame, and the displacement the group states, its
// AxialDelta plus its ExtentDelta. It is false for a group whose surface is no plane.
func planeOfGroup(fp facetedrecord.Payload, g int) (plane, float64, bool) {
	if g < 0 || g >= len(fp.Groups) {
		return plane{}, 0, false
	}
	grp := fp.Groups[g]
	pl, ok := grp.Surface.(surfacegeom.Plane)
	if !ok {
		return plane{}, 0, false
	}
	delta := 0.0
	if grp.HasAxialDelta {
		delta = grp.AxialDelta
	}
	if !(delta >= 0) || !(grp.ExtentDelta >= 0) || math.IsInf(delta, 1) || math.IsInf(grp.ExtentDelta, 1) {
		return plane{}, 0, false
	}
	return plane{n: pl.Frame.N(), o: pl.Frame.Origin()}, proofbound.AbsSumUpper(delta, grp.ExtentDelta), true
}

// chargePlaneFace adds to the work's plane volume term the facets of the plane face of source group g that
// tris (indices into w.out) lists (Claim K). A face whose group states no plane tag refuses: no bound reads its
// facets' distance from a plane.
func (w *work) chargePlaneFace(group, from int) error {
	pl, delta, ok := planeOfGroup(w.fp, group)
	if !ok {
		return fmt.Errorf("%w: a re-triangulated face states no plane tag", decaderr.ErrUnsupported)
	}
	vol := planeFacetVolume(w.verts, w.out[from:], pl, delta)
	if proofbound.IsNonFinite(vol) {
		return fmt.Errorf("%w: a re-triangulated face's volume term is not finite", decaderr.ErrUnsupported)
	}
	w.planeVol = proofbound.AbsSumUpper(w.planeVol, vol)
	return nil
}

// planeAreaCharge is the area bound a route's displaced planes add to its exact pieces: per piece
// 2·P·c + (π + Φ)·c² + A·c/r, c the route's plane charge (arcAreaCharge's form, with the plane charge for both
// the boundary move and the radius move). It is zero when c is.
func planeAreaCharge(pieces []chargePiece, c float64) float64 {
	if !(c > 0) || len(pieces) == 0 {
		return 0
	}
	return arcAreaCharge(pieces, c, c)
}

// planePiece is the piece of a planar polygon through pts: its perimeter rounded up, its area and a turning of
// at most π per corner (a simple polygon of k corners turns by no more).
func planePiece(pts ...r3.Vec) chargePiece {
	per := 0.0
	for i, p := range pts {
		per = proofbound.AbsSumUpper(per, lenUp(p, pts[(i+1)%len(pts)]))
	}
	return chargePiece{perimeter: per, turning: proofbound.ProductUpper(float64(len(pts)), piRound),
		area: proofbound.ProductUpper(polyArea(pts...), 1+16*ulp)}
}

// lenUp is |p − q| rounded up past the rounding of the difference and the root.
func lenUp(p, q r3.Vec) float64 {
	d := p.Sub(q)
	return up(math.Sqrt(up(d.Dot(d))))
}

// addPlanePiece records one changed piece of the route for the area term of a displaced plane.
func (w *work) addPlanePiece(p chargePiece) {
	w.planePieces = append(w.planePieces, p)
}

// The pieces below are the changed pieces of each route, in the form arcAreaCharge reads: the perimeter and turning
// are upper bounds on the exact piece's, in its own coordinates, and rMin is zero for a planar piece.

// mulUp and sumUp round a product and a sum of non-negative bounds up.
func mulUp(a, b float64) float64  { return proofbound.ProductUpper(a, b) }
func sumUp(vs ...float64) float64 { return proofbound.AbsSumUpper(vs...) }

// rectPiece is a rectangle of sides s1 and s2 in a piece's own coordinates, on a surface of smallest radius of
// curvature rMin (zero for a plane).
func rectPiece(s1, s2, rMin float64) chargePiece {
	return chargePiece{perimeter: mulUp(2, sumUp(s1, s2)), turning: proofbound.TwoPiUpper(),
		area: mulUp(s1, s2), rMin: rMin}
}

// isoArea bounds the area a closed curve of length per encloses, from above: A ≤ P²/(4π).
func isoArea(per float64) float64 { return proofbound.DivUpper(mulUp(per, per), 12.5) }

// cornerPiece is a region of an end plane bounded by arcs and lines of total length per, which turns by at most
// four right angles and the arcs' spans.
func cornerPiece(per, arcTurn float64) chargePiece {
	return chargePiece{perimeter: per, turning: sumUp(mulUp(4, piRound), arcTurn)}
}

// obliqueCapArea bounds how much more area the true cap of an oblique end plane holds than the cap's projection
// onto the tag (retained-edge-blend §6.3, "An oblique end plane"). The true plane turns from the tag by an angle α
// whose sine is at most the tag's departure θ (two unit normals at a distance θ make such an angle), and a planar
// region's area is its projection's over cos α. For sin α ≤ θ ≤ maxEndTilt, 1/cos α − 1 ≤ θ². The projection holds
// at most the piece's exact area plus its plane charge (planeAreaCharge), or, for a piece that states no area, the
// isoperimetric bound of its perimeter. It is zero for a plane that states no tilt.
func obliqueCapArea(p chargePiece, c, tilt float64) float64 {
	if !(tilt > 0) {
		return 0
	}
	area := p.area
	if !(area > 0) {
		area = isoArea(p.perimeter)
	}
	return mulUp(sumUp(area, planeAreaCharge([]chargePiece{p}, c)), mulUp(tilt, tilt))
}

// lineAreaPieces records the pieces of a straight edge's rewrite (lineAreasAt): the two regions A and B lose, the
// corner each end plane gains or loses, and the blend face.
func (w *work) lineAreaPieces(l line, spec blendSpec, rows [2][]int, mitred [2]bool, blend float64) {
	if !(w.r.charge > 0) {
		return
	}
	m := len(rows[0]) - 1
	pt := func(i, j int) r3.Vec { return w.verts[rows[i][j]] }
	v0, v1 := l.ends[0].exact, l.ends[1].exact
	w.addPlanePiece(planePiece(v0, v1, pt(1, 0), pt(0, 0)))
	w.addPlanePiece(planePiece(v0, v1, pt(1, m), pt(0, m)))
	for i := range 2 {
		if mitred[i] {
			continue
		}
		var cp chargePiece
		if spec.fillet {
			// The fillet's end curve on G is the ball's circle stretched along the edge by 1/|d·n_G| at most.
			stretch := proofbound.DivUpper(1, down(math.Abs(l.d.Dot(l.ends[i].g.n))))
			arc := mulUp(mulUp(spec.r, l.phi), stretch)
			cp = cornerPiece(sumUp(lenUp(l.ends[i].exact, pt(i, 0)), lenUp(l.ends[i].exact, pt(i, m)), arc), piRound)
		} else {
			cp = planePiece(l.ends[i].exact, pt(i, 0), pt(i, m))
		}
		w.addPlanePiece(cp)
		w.tiltArea = sumUp(w.tiltArea, obliqueCapArea(cp, w.r.charge, l.ends[i].tilt))
	}
	if !spec.fillet {
		w.addPlanePiece(planePiece(pt(0, 0), pt(1, 0), pt(1, m), pt(0, m)))
		return
	}
	// The fillet face is a cylinder piece: two generators and two end curves, each at most the arc plus the
	// axial drift of G's plane along it.
	stretch0 := proofbound.DivUpper(1, down(math.Abs(l.d.Dot(l.ends[0].g.n))))
	stretch1 := proofbound.DivUpper(1, down(math.Abs(l.d.Dot(l.ends[1].g.n))))
	arc := mulUp(spec.r, l.phi)
	per := sumUp(lenUp(pt(0, 0), pt(1, 0)), lenUp(pt(0, m), pt(1, m)),
		mulUp(arc, sumUp(1, stretch0)), mulUp(arc, sumUp(1, stretch1)))
	w.addPlanePiece(chargePiece{perimeter: per, turning: mulUp(4, piRound), area: mulUp(blend, 1+16*ulp), rMin: spec.r})
}

// lineAreasAt is lineAreasExact that also records the pieces a displaced plane moves (plane_charge.go).
func (w *work) lineAreasAt(l line, spec blendSpec, convex bool, center, baseA, baseB r3.Vec,
	rows [2][]int, mitred, gains [2]bool) (float64, float64) {
	removed, blend := w.lineAreasExact(l, spec, convex, center, baseA, baseB, rows, mitred, gains)
	w.lineAreaPieces(l, spec, rows, mitred, blend)
	return removed, blend
}

// cylLinePieces records the pieces of a plane–cylinder edge's rewrite (rewriteCylLine): the plane's lost region,
// the wall's band, the corner at each end and the blend face.
func (w *work) cylLinePieces(c cylLine, spec blendSpec, sec section, k0, k1, first, last r3.Vec, length, corner float64) {
	if !(w.r.charge > 0) {
		return
	}
	arcWall := mulUp(c.radius, sec.thetaT)
	w.addPlanePiece(planePiece(k0, k1, last, first))
	w.addPlanePiece(rectPiece(arcWall, length, c.radius))
	blendArc := lenUp(sec.ta, sec.tb)
	if spec.fillet {
		blendArc = mulUp(spec.r, sec.phi)
	}
	cp := cornerPiece(sumUp(lenUp(k0, sec.ta), blendArc, arcWall), sumUp(sec.thetaT, sec.phi))
	cp.area = mulUp(corner, 1+16*ulp)
	w.addPlanePiece(cp)
	w.addPlanePiece(cp)
	rMin := 0.0
	if spec.fillet {
		rMin = spec.r
	}
	w.addPlanePiece(rectPiece(blendArc, length, rMin))
}

// cylCylPieces records the pieces of a cylinder–cylinder edge's rewrite (rewriteCylCyl): each wall's band, the
// corner at each end and the blend face.
func (w *work) cylCylPieces(c cylCyl, spec blendSpec, sec cylSection, length, corner float64) {
	if !(w.r.charge > 0) {
		return
	}
	var arcs [2]float64
	for k := range 2 {
		arcs[k] = mulUp(c.wall[k].radius, sec.theta[k])
		w.addPlanePiece(rectPiece(arcs[k], length, c.wall[k].radius))
	}
	blendArc := lenUp(sec.t[0], sec.t[1])
	if spec.fillet {
		blendArc = mulUp(spec.r, sec.phi)
	}
	cp := cornerPiece(sumUp(arcs[0], blendArc, arcs[1]), sumUp(sec.theta[0], sec.theta[1], sec.phi))
	cp.area = mulUp(corner, 1+16*ulp)
	w.addPlanePiece(cp)
	w.addPlanePiece(cp)
	rMin := 0.0
	if spec.fillet {
		rMin = spec.r
	}
	w.addPlanePiece(rectPiece(blendArc, length, rMin))
}

// arcPlanePieces records the pieces of an open arc's rewrite (rewriteArc, arcChargeOf's list): the plane's lost
// region, the wall's band, the blend face and the corner at each end. An end curve is monotone in both coordinates
// of its end section, so its length is at most the sum of its legs.
func (w *work) arcPlanePieces(a arcRim, rows []arcRow, ends []endSection, tb, blendArea float64, spec blendSpec) {
	if !(w.r.charge > 0) {
		return
	}
	m := len(rows) - 1
	dTheta0, dThetaM := math.Abs(rows[0].theta[1]-rows[0].theta[0]), math.Abs(rows[m].theta[1]-rows[m].theta[0])
	rhoMin := math.Inf(1)
	for _, r := range rows {
		rhoMin = min(rhoMin, r.rho)
	}
	var side, sideTurn, curve [2]float64
	for i := range 2 {
		u0, v0, _ := ends[i].at(0)
		u1, v1, _ := ends[i].at(1)
		curve[i] = up(sumUp(math.Abs(u1-u0), math.Abs(v1-v0)))
		p := a.atTheta(rows[0].rho, rows[0].theta[i], 0)
		if c := a.cyl[i]; c != nil {
			dpsi := math.Abs(wrapPi(a.angleAbout(c.c, a.ends[i].exact) - a.angleAbout(c.c, p)))
			side[i], sideTurn[i] = mulUp(c.rG, dpsi), dpsi
			continue
		}
		side[i] = lenUp(a.ends[i].exact, p)
	}
	regionPer := sumUp(mulUp(a.radius, a.span), mulUp(rows[0].rho, dTheta0), side[0], side[1])
	w.addPlanePiece(chargePiece{perimeter: regionPer,
		turning: sumUp(proofbound.TwoPiUpper(), a.span, dTheta0, sideTurn[0], sideTurn[1]),
		area:    mulUp(a.planeRegion(rows), 1+16*ulp)})
	arcWall := mulUp(a.radius, a.span)
	w.addPlanePiece(rectPiece(arcWall, tb, a.radius))
	rBlend := rhoMin
	if spec.fillet {
		rBlend = min(rBlend, spec.r)
	}
	w.addPlanePiece(chargePiece{
		perimeter: sumUp(mulUp(rows[0].rho, dTheta0), mulUp(a.radius, dThetaM), curve[0], curve[1]),
		turning:   sumUp(proofbound.TwoPiUpper(), dTheta0, dThetaM, 2*endCurveTurn),
		area:      mulUp(blendArea, 1+16*ulp), rMin: rBlend})
	for i := range 2 {
		per := sumUp(side[i], curve[i], tb)
		cp := cornerPiece(per, sumUp(endCurveTurn, sideTurn[i]))
		if c := a.cyl[i]; c != nil {
			cp.rMin, cp.area = c.rG, isoArea(per)
		}
		w.addPlanePiece(cp)
	}
}

// sphereRimPieces records the pieces of a plane–sphere rim's rewrite: the plane's annulus, the sphere's zone, and
// the blend face, a torus or a cone, between circles of radii rhoA and rhoB.
func (w *work) sphereRimPieces(s sphereRim, spec blendSpec, m meridian, tb float64) {
	if !(w.r.charge > 0) {
		return
	}
	hi := max(s.radius, m.rhoA)
	annulus := mulUp(proofbound.TwoPiUpper(), sumUp(s.radius, m.rhoA))
	w.addPlanePiece(chargePiece{perimeter: annulus, turning: mulUp(2, proofbound.TwoPiUpper()),
		area: mulUp(mulUp(math.Pi*(1+4*ulp), math.Abs(m.rhoA-s.radius)), sumUp(hi, hi))})
	zone := mulUp(proofbound.TwoPiUpper(), sumUp(s.radius, m.rhoB))
	w.addPlanePiece(chargePiece{perimeter: zone, turning: mulUp(2, proofbound.TwoPiUpper()),
		area: mulUp(mulUp(proofbound.TwoPiUpper(), s.sphereRadius), math.Abs(tb)), rMin: s.sphereRadius})
	rMin := min(m.rhoA, m.rhoB)
	if spec.fillet {
		rMin = min(rMin, spec.r)
		if low := m.rhoC - spec.r; low > 0 {
			rMin = min(rMin, low)
		}
	}
	w.addPlanePiece(chargePiece{perimeter: mulUp(proofbound.TwoPiUpper(), sumUp(m.rhoA, m.rhoB)),
		turning: mulUp(2, proofbound.TwoPiUpper()), area: mulUp(m.trueArea, 1+16*ulp), rMin: down(rMin)})
}

// cornerPatchPieces records the pieces of a corner where three filleted edges meet: the three squares of side r
// the sphere octant takes from the faces, and the octant.
func (w *work) cornerPatchPieces(r, patch float64) {
	if !(w.r.charge > 0) {
		return
	}
	for range 3 {
		w.addPlanePiece(rectPiece(r, r, 0))
	}
	w.addPlanePiece(chargePiece{perimeter: mulUp(mulUp(3, proofbound.DivUpper(piRound, 2)), r),
		turning: mulUp(4, piRound), area: mulUp(patch, 1+16*ulp), rMin: r})
}

// chainPlanePieces records the pieces of an open chain's rewrite whose end planes may state a displacement: the
// groups chainPieces lists, and per end the cap regions its end plane gains or loses, each bounded by the legs the
// end plane's crossing gate (minSine) allows. A charged chain carries its own area charge (chainCharges.areaCharge).
func (w *work) chainPlanePieces(entries []chainEntry, tP, tW float64, spec blendSpec, open bool) {
	if !open || !(w.r.charge > 0) {
		return
	}
	pieces, _ := chainPieces(entries, tP, tW, spec, open)
	rMin := math.Inf(1)
	for _, p := range pieces {
		w.addPlanePiece(p.geo)
		if p.geo.rMin > 0 {
			rMin = min(rMin, p.geo.rMin)
		}
	}
	hyp := up(math.Hypot(tP, tW))
	leg := proofbound.DivUpper(sumUp(tP, tW, hyp), minSine)
	for range 2 {
		w.addPlanePiece(cornerPiece(mulUp(4, leg), piRound))
		if !math.IsInf(rMin, 1) {
			w.addPlanePiece(chargePiece{perimeter: mulUp(4, leg), turning: mulUp(4, piRound), area: mulUp(leg, leg),
				rMin: rMin})
		}
	}
}
