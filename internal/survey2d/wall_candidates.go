package survey2d

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// pairCands emits the antipodal pair criticals (T2) and the angle-limit
// disks (T3) for one element pair.
func (k *WallKernel) PairCands(a, b SurveyElem, add func(x, y, r, rBound float64)) {
	switch {
	case a.Kind == SurveyLine && b.Kind == SurveyLine:
		k.LineLineCands(a, b, add)
	case a.Kind == SurveyArc && b.Kind == SurveyArc:
		k.ArcArcCands(a, b, add)
	case a.Kind == SurveyLine:
		k.LineArcCands(a, b, add)
	default:
		k.LineArcCands(b, a, add)
	}
}

// lineLineCands: parallel facing lines carry a constant-diameter family;
// the overlap midpoint is its representative (blocked positions surface via
// the triples). Non-parallel lines have no interior critical: their corners
// are junctions, their extent limits vertices, their blockers triples.
//
// The parallel and facing tests read the two elements' own unit normals under
// a 1e-9 slack. Those normals are element coordinates, exact leaves by this
// kernel's stated convention, and the slack is seven orders above their last
// ulp, so neither test can be straddled by the arithmetic it performs.
func (k *WallKernel) LineLineCands(a, b SurveyElem, add func(x, y, r, rBound float64)) {
	cross := a.Nx*b.Ny - a.Ny*b.Nx
	if math.Abs(cross) > 1e-9 {
		return
	}
	if a.Nx*b.Nx+a.Ny*b.Ny > -1+1e-9 {
		return // same-facing parallels never oppose across material
	}
	dBS := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.ExactScalar(a.Nx), proofbound.BoundedSub(proofbound.ExactScalar(b.Ax), proofbound.ExactScalar(a.Ax))),
		proofbound.BoundedMul(proofbound.ExactScalar(a.Ny), proofbound.BoundedSub(proofbound.ExactScalar(b.Ay), proofbound.ExactScalar(a.Ay))),
	)
	d := dBS.Value
	if proofbound.AdmitAbove(dBS, 0) == proofbound.SurvReject {
		return // b is PROVEN not on a's material side
	}
	// Overlap of the two segments along a's tangent.
	tx, ty := -a.Ny, a.Nx
	a0, a1 := tx*a.Ax+ty*a.Ay, tx*a.Bx+ty*a.By
	b0, b1 := tx*b.Ax+ty*b.Ay, tx*b.Bx+ty*b.By
	lo := math.Max(math.Min(a0, a1), math.Min(b0, b1))
	hi := math.Min(math.Max(a0, a1), math.Max(b0, b1))
	if lo > hi {
		return
	}
	m := (lo + hi) / 2
	// Base point on a at parameter m, then half-way toward b.
	base := tx*a.Ax + ty*a.Ay
	px := a.Ax + (m-base)*tx + a.Nx*d/2
	py := a.Ay + (m-base)*ty + a.Ny*d/2
	rBS := proofbound.BoundedQuotient(d, dBS.Bound, 2, 0)
	add(px, py, d/2, rBS.Bound)
}

// lineArcCands: the critical disks sit on the perpendicular from the arc's
// center to the line; the angle-limit disks sit where the contact pair
// reaches exactly the allowance boundary.
//
// The T2 denominator sgn·s − 1 is built from two SIGNS, so it takes one of the
// exactly representable values {0, −2, 2} and its degeneracy reading can never
// straddle. The T3 denominator is 1 + cos α for the caller's own draft
// allowance α, and reaches zero only at α = π; a straddle there needs an
// allowance within a micro-radian of a half turn, which is no draft angle.
func (k *WallKernel) LineArcCands(l, a SurveyElem, add func(x, y, r, rBound float64)) {
	s := a.MatSign()
	e := l.Nx*(a.Qx-l.Ax) + l.Ny*(a.Qy-l.Ay) // signed height of q, material side positive
	eBS := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.ExactScalar(l.Nx), proofbound.BoundedSub(proofbound.ExactScalar(a.Qx), proofbound.ExactScalar(l.Ax))),
		proofbound.BoundedMul(proofbound.ExactScalar(l.Ny), proofbound.BoundedSub(proofbound.ExactScalar(a.Qy), proofbound.ExactScalar(l.Ay))),
	)
	fx := a.Qx - e*l.Nx
	fy := a.Qy - e*l.Ny
	// T2 on the perpendicular from q to the line, t = r, center at f + t·n̂:
	// |e − t| = R − s·t, enumerated as e − t = sgn·(R − s·t).
	for _, sgn := range []float64{1, -1} {
		den := sgn*s - 1
		if proofbound.AdmitMagnitudeAbove(proofbound.ExactScalar(den), SurvTiny) == proofbound.SurvReject {
			continue
		}
		t := (sgn*a.Rr - e) / den
		numBS := proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(sgn), a.RrBS()), eBS)
		tBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, den, 0)
		if proofbound.AdmitAbove(tBS, 0) != proofbound.SurvReject {
			add(fx+t*l.Nx, fy+t*l.Ny, t, tBS.Bound)
		}
	}
	// T3: contact directions exactly π − α apart. The line contact direction
	// is −n̂; the arc contact direction is s·(ĉ−q̂); c = q + (s·R − r)·u2.
	for _, trig := range k.DraftTrig {
		snBS, csBS := trig.sin, trig.cos
		cs, sn := csBS.Value, snBS.Value
		// u2 = rotate(−n̂, ±aStar)
		ux := -(l.Nx*cs - l.Ny*sn)
		uy := -(l.Nx*sn + l.Ny*cs)
		uxBS := proofbound.BoundedMul(proofbound.ExactScalar(-1), proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(l.Nx), csBS), proofbound.BoundedMul(proofbound.ExactScalar(l.Ny), snBS)))
		uyBS := proofbound.BoundedMul(proofbound.ExactScalar(-1), proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(l.Nx), snBS), proofbound.BoundedMul(proofbound.ExactScalar(l.Ny), csBS)))
		ndot := l.Nx*ux + l.Ny*uy // = −cos(aStar)
		ndotBS := proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(l.Nx), uxBS), proofbound.BoundedMul(proofbound.ExactScalar(l.Ny), uyBS))
		denBS := proofbound.BoundedAdd(proofbound.ExactScalar(1), ndotBS)
		if proofbound.AdmitMagnitudeAbove(denBS, SurvTiny) == proofbound.SurvReject {
			continue
		}
		den := denBS.Value
		r := (l.Nx*(a.Qx-l.Ax) + l.Ny*(a.Qy-l.Ay) + s*a.Rr*ndot) / den
		numBS := proofbound.BoundedAdd(eBS, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(s), a.RrBS()), ndotBS))
		rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, denBS.Value, denBS.Bound)
		if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
			add(a.Qx+(s*a.Rr-r)*ux, a.Qy+(s*a.Rr-r)*uy, r, rBS.Bound)
		}
	}
}

// arcArcCands: centerline criticals (T2, the concentric family included) and
// the law-of-cosines angle-limit disks (T3).
//
// The concentric test is the one branch here that is NOT resolved toward
// generating both sides: a centre separation the interval cannot prove positive
// leaves the centreline direction dx/d undefined, so the general branch has no
// candidate to build and the concentric family is the only reading of that
// pair. The T2 denominator −εa·sa − εb·sb is built from four SIGNS and so takes
// one of {0, ±2} exactly, which no interval can straddle.
func (k *WallKernel) ArcArcCands(a, b SurveyElem, add func(x, y, r, rBound float64)) {
	dx, dy := b.Qx-a.Qx, b.Qy-a.Qy
	dBS := proofbound.BoundedHypot(dx, dy)
	d := dBS.Value
	sa, sb := a.MatSign(), b.MatSign()
	if proofbound.AdmitAbove(dBS, SurvTiny*k.Scale) != proofbound.SurvAdmit {
		// Concentric: the family disk at each arc's own angular midpoint. The
		// annulus half-width is |Ra − Rb|/2, and NEITHER step is exact — the
		// difference of two radii rounds outside the Sterbenz range, and each
		// radius carries whatever bound its own element states — so the whole
		// chain runs through the bounded arithmetic. The halving is exact, but
		// proofbound.BoundedQuotient carries the difference's bound through it.
		diffBS := proofbound.BoundedAbs(proofbound.BoundedSub(a.RrBS(), b.RrBS()))
		rBS := proofbound.BoundedQuotient(diffBS.Value, diffBS.Bound, 2, 0)
		r := rBS.Value
		m := (a.Rr + b.Rr) / 2
		for _, e := range []SurveyElem{a, b} {
			lo, hi := e.ArcRange()
			th := (lo + hi) / 2
			add(a.Qx+m*math.Cos(th), a.Qy+m*math.Sin(th), r, rBS.Bound)
		}
		return
	}
	ux, uy := dx/d, dy/d
	// T2 on the centerline: εa(Ra − sa·r) + εb(Rb − sb·r) = d.
	for _, ea := range []float64{1, -1} {
		for _, eb := range []float64{1, -1} {
			den := -ea*sa - eb*sb
			if proofbound.AdmitMagnitudeAbove(proofbound.ExactScalar(den), SurvTiny) == proofbound.SurvReject {
				continue
			}
			r := (d - ea*a.Rr - eb*b.Rr) / den
			numBS := proofbound.BoundedSub(dBS, proofbound.BoundedAdd(
				proofbound.BoundedMul(proofbound.ExactScalar(ea), a.RrBS()),
				proofbound.BoundedMul(proofbound.ExactScalar(eb), b.RrBS()),
			))
			rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, den, 0)
			if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
				continue
			}
			t := ea * (a.Rr - sa*r)
			add(a.Qx+t*ux, a.Qy+t*uy, r, rBS.Bound)
		}
	}
	// T3: angle at the center between (c−qa) and (c−qb) fixed by the
	// allowance boundary; law of cosines in r, then the two mirror centers.
	cosAStarBS := k.DraftTrig[0].cos
	cosThBS := proofbound.BoundedMul(proofbound.ExactScalar(sa*sb), cosAStarBS)
	// d² = Da² + Db² − 2·Da·Db·cosθ with Da = Ra − sa·r, Db = Rb − sb·r.
	raBS, rbBS := a.RrBS(), b.RrBS()
	ABS := proofbound.BoundedSub(proofbound.ExactScalar(2), proofbound.BoundedMul(proofbound.ExactScalar(2*sa*sb), cosThBS))
	BBS := proofbound.BoundedAdd(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(-2*sa), raBS), proofbound.BoundedMul(proofbound.ExactScalar(-2*sb), rbBS)),
		proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.ExactScalar(sb), raBS),
			proofbound.BoundedMul(proofbound.ExactScalar(sa), rbBS),
		)), cosThBS),
	)
	CBS := proofbound.BoundedSub(
		proofbound.BoundedAdd(
			proofbound.BoundedAdd(proofbound.BoundedMul(raBS, raBS), proofbound.BoundedMul(rbBS, rbBS)),
			proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(-2), proofbound.BoundedMul(raBS, rbBS)), cosThBS),
		),
		proofbound.BoundedMul(dBS, dBS),
	)
	for _, rBS := range QuadRootsBounded(ABS, BBS, CBS) {
		if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
			continue
		}
		r := rBS.Value
		da := a.Rr - sa*r
		db := b.Rr - sb*r
		PlaceCircleCircle(a.Qx, a.Qy, da, b.Qx, b.Qy, db, add, r, rBS.Bound)
	}
}

// vertexElemCands: the vertex-line foot and vertex-arc centerline criticals
// (T2) plus their angle-limit disks (T3).
//
// A vertex sitting ON the element it is read against — the ordinary case, since
// a junction vertex IS an endpoint of its own two walks — gives an exactly zero
// height or separation with an exactly zero bound, so the side and degeneracy
// readings below decide it outright. The arc T2 denominator sgn·s − ev is built
// from three SIGNS and takes one of {0, ±2} exactly.
func (k *WallKernel) VertexElemCands(v [2]float64, e SurveyElem, add func(x, y, r, rBound float64)) {
	if e.Kind == SurveyLine {
		// T2: the foot midpoint.
		h := e.Nx*(v[0]-e.Ax) + e.Ny*(v[1]-e.Ay)
		hBS := proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.ExactScalar(e.Nx), proofbound.BoundedSub(proofbound.ExactScalar(v[0]), proofbound.ExactScalar(e.Ax))),
			proofbound.BoundedMul(proofbound.ExactScalar(e.Ny), proofbound.BoundedSub(proofbound.ExactScalar(v[1]), proofbound.ExactScalar(e.Ay))),
		)
		if proofbound.AdmitAbove(hBS, 0) != proofbound.SurvReject {
			rBS := proofbound.BoundedQuotient(hBS.Value, hBS.Bound, 2, 0)
			add(v[0]-h/2*e.Nx, v[1]-h/2*e.Ny, h/2, rBS.Bound)
		}
		// T3: u_v = rotate(−n̂, ±A*), c = v − r·u_v, tangency fixes r.
		for _, trig := range k.DraftTrig {
			snBS, csBS := trig.sin, trig.cos
			cs, sn := csBS.Value, snBS.Value
			ux := -(e.Nx*cs - e.Ny*sn)
			uy := -(e.Nx*sn + e.Ny*cs)
			uxBS := proofbound.BoundedMul(proofbound.ExactScalar(-1), proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(e.Nx), csBS), proofbound.BoundedMul(proofbound.ExactScalar(e.Ny), snBS)))
			uyBS := proofbound.BoundedMul(proofbound.ExactScalar(-1), proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(e.Nx), snBS), proofbound.BoundedMul(proofbound.ExactScalar(e.Ny), csBS)))
			den := 1 + (e.Nx*ux + e.Ny*uy)
			denBS := proofbound.BoundedAdd(proofbound.ExactScalar(1), proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.ExactScalar(e.Nx), uxBS), proofbound.BoundedMul(proofbound.ExactScalar(e.Ny), uyBS)))
			if proofbound.AdmitMagnitudeAbove(denBS, SurvTiny) == proofbound.SurvReject {
				continue
			}
			r := (e.Nx*(v[0]-e.Ax) + e.Ny*(v[1]-e.Ay)) / den
			rBS := proofbound.BoundedQuotient(hBS.Value, hBS.Bound, denBS.Value, denBS.Bound)
			if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
				add(v[0]-r*ux, v[1]-r*uy, r, rBS.Bound)
			}
		}
		return
	}
	s := e.MatSign()
	dx, dy := e.Qx-v[0], e.Qy-v[1]
	dBS := proofbound.BoundedHypot(dx, dy)
	d := dBS.Value
	if proofbound.AdmitAbove(dBS, SurvTiny*k.Scale) != proofbound.SurvAdmit {
		return
	}
	ux, uy := dx/d, dy/d
	// T2 on the line through v and q: |c−v| = r with c = v ± r·û, and
	// |c−q| = R − s·r.
	for _, ev := range []float64{1, -1} {
		// c = v + ev·r·û: |c − q| = |d − ev·r| = R − s·r.
		for _, sgn := range []float64{1, -1} {
			den := sgn*s - ev
			if proofbound.AdmitMagnitudeAbove(proofbound.ExactScalar(den), SurvTiny) == proofbound.SurvReject {
				continue
			}
			r := (sgn*e.Rr - d) / den
			numBS := proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(sgn), e.RrBS()), dBS)
			rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, den, 0)
			if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
				add(v[0]+ev*r*ux, v[1]+ev*r*uy, r, rBS.Bound)
			}
		}
	}
	// T3: law of cosines with sides r and R − s·r.
	cosAStarBS := k.DraftTrig[0].cos
	cosThBS := proofbound.BoundedMul(proofbound.ExactScalar(-s), cosAStarBS)
	ABS := proofbound.BoundedAdd(proofbound.ExactScalar(2), proofbound.BoundedMul(proofbound.ExactScalar(2*s), cosThBS))
	reBS := e.RrBS()
	BBS := proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(-2), reBS), proofbound.BoundedAdd(proofbound.ExactScalar(s), cosThBS))
	CBS := proofbound.BoundedSub(proofbound.BoundedMul(reBS, reBS), proofbound.BoundedMul(dBS, dBS))
	for _, rBS := range QuadRootsBounded(ABS, BBS, CBS) {
		if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
			continue
		}
		r := rBS.Value
		PlaceCircleCircle(v[0], v[1], r, e.Qx, e.Qy, e.Rr-s*r, add, r, rBS.Bound)
	}
}

// vertexVertexCands: the midpoint disk (T2) and the isoceles angle-limit
// disks (T3).
//
// Two coincident vertices give an exactly zero separation with an exactly zero
// bound, so the degeneracy reading decides them. The T3 denominator is
// 2·(1 − cos(π − α)) = 2·(1 + cos α) for the caller's own draft allowance, and
// vanishes only at α = π — an allowance of a half turn, which is no draft.
func (k *WallKernel) VertexVertexCands(a, b [2]float64, add func(x, y, r, rBound float64)) {
	dx, dy := b[0]-a[0], b[1]-a[1]
	dBS := proofbound.BoundedHypot(dx, dy)
	d := dBS.Value
	if proofbound.AdmitAbove(dBS, SurvTiny*k.Scale) != proofbound.SurvAdmit {
		return
	}
	rBS := proofbound.BoundedQuotient(dBS.Value, dBS.Bound, 2, 0)
	add((a[0]+b[0])/2, (a[1]+b[1])/2, d/2, rBS.Bound)
	cosAStarBS := k.DraftTrig[0].cos
	denBS := proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.BoundedSub(proofbound.ExactScalar(1), cosAStarBS))
	if proofbound.AdmitAbove(denBS, SurvTiny) != proofbound.SurvReject {
		sqrtDenBS := proofbound.BoundedSqrt(denBS)
		rr := proofbound.BoundedQuotient(dBS.Value, dBS.Bound, sqrtDenBS.Value, sqrtDenBS.Bound)
		PlaceCircleCircle(a[0], a[1], rr.Value, b[0], b[1], rr.Value, add, rr.Value, rr.Bound)
	}
}

// wedgeCands: the wedge-tangent minima for a partial revolve's cap-cap
// reading — the disk tangent to one element with the wedge constraint
// active (r = wedgeS·y), at its own closed-form ρ-critical. wedgeS carries its
// own bound (a sine its caller proved from a certified trig interval), so BOTH
// quotient paths below compose that bound through their numerator and
// denominator rather than reading the sine as an exact leaf, and the arc path
// reads the element's radius through rrBS for the same reason.
//
// A vertex at or below the axis (v[1] <= 0) is an exact recorded coordinate
// against an exact zero, so that test needs no interval. Every other reading
// here is taken on the proven interval and resolved toward emitting: the wedge
// denominators 1 ∓ sin(Δφ/2) reach zero only for a half sweep at exactly a
// right angle, where the sine is proven 1 and the reading is a proven reject,
// never a straddle.
func (k *WallKernel) WedgeCands(budget *proofbound.WorkBudget, visit func(DiskCand) error) error {
	sBS := k.WedgeS
	s := sBS.Value
	for _, v := range k.Verts {
		if err := WallBudgetStep(budget); err != nil {
			return err
		}
		if v[1] <= 0 {
			continue
		}
		for _, sgn := range []float64{1, -1} {
			denBS := proofbound.BoundedSub(proofbound.ExactScalar(1), proofbound.BoundedMul(proofbound.ExactScalar(sgn), sBS))
			if proofbound.AdmitAbove(denBS, SurvTiny) == proofbound.SurvReject {
				continue
			}
			numBS := proofbound.BoundedMul(sBS, proofbound.ExactScalar(v[1]))
			rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, denBS.Value, denBS.Bound)
			r := rBS.Value
			if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
				if err := visit(DiskCand{X: v[0], Y: r / s, R: r, RBound: rBS.Bound}); err != nil {
					return err
				}
			}
		}
	}
	for _, e := range k.Elems {
		if err := WallBudgetStep(budget); err != nil {
			return err
		}
		if e.Kind != SurveyArc {
			continue // a line's wedge-tangent family has no interior minimum
		}
		se := e.MatSign()
		for _, th := range []float64{math.Pi / 2, -math.Pi / 2} {
			if !e.ArcContains(th) {
				continue
			}
			sinBS, _ := RadianTrigBounds(th)
			numBS := proofbound.BoundedMul(sBS, proofbound.BoundedAdd(proofbound.ExactScalar(e.Qy), proofbound.BoundedMul(e.RrBS(), sinBS)))
			denBS := proofbound.BoundedAdd(proofbound.ExactScalar(1), proofbound.BoundedMul(proofbound.BoundedMul(sBS, proofbound.ExactScalar(se)), sinBS))
			if proofbound.AdmitMagnitudeAbove(denBS, SurvTiny) == proofbound.SurvReject {
				continue
			}
			rBS := proofbound.BoundedQuotient(numBS.Value, numBS.Bound, denBS.Value, denBS.Bound)
			if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
				continue
			}
			r := rBS.Value
			d := e.Rr - se*r
			if err := visit(DiskCand{X: e.Qx + d*math.Cos(th), Y: e.Qy + d*math.Sin(th), R: r, RBound: rBS.Bound}); err != nil {
				return err
			}
		}
	}
	return nil
}
