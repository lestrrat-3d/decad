package survey2d

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// CircEq is one tangency equation for the Apollonius triples: either linear
// (a·cx + b·cy + e·r + f = 0) or quadratic
// (cx² + cy² − r² + g·cx + h·cy + kk·r + m = 0). Every coefficient is a
// proofbound.BoundedScalar, not a bare float: the element, vertex and wedge coordinates
// the equations are built from are exact leaves, but forming a coefficient
// from them rounds (a product of two coordinates, a sum of three such
// products); an arc element's radius is not even a leaf, since it arrives with
// its own bound (SurveyElem.rrBound); and the wedge's own coefficient is a
// sine that arrives already bounded.
// SolveTriple/Solve3Linear/SolveParallelPair compose those bounds through
// every determinant, numerator and division they perform, so a triple's
// published radius interval speaks for the WHOLE chain from coefficient
// construction down to the final quotient.
type CircEq struct {
	Quad        bool
	G, H, Kk, M proofbound.BoundedScalar
	A, B, E, F  proofbound.BoundedScalar
}

// tripleEquations builds the material-side-pinned tangency equation of every
// item: elements, vertices, and the wedge.
func (k *WallKernel) TripleEquations(budget *proofbound.WorkBudget) ([]CircEq, error) {
	var eqs []CircEq
	for _, el := range k.Elems {
		if err := WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if el.Kind == SurveyLine {
			nx, ny := proofbound.ExactScalar(el.Nx), proofbound.ExactScalar(el.Ny)
			eqs = append(eqs, CircEq{
				A: nx, B: ny, E: proofbound.ExactScalar(-1),
				F: proofbound.BoundedNeg(proofbound.BoundedAdd(
					proofbound.BoundedMul(nx, proofbound.ExactScalar(el.Ax)),
					proofbound.BoundedMul(ny, proofbound.ExactScalar(el.Ay)),
				)),
			})
			continue
		}
		s := el.MatSign()
		qx, qy, rr := proofbound.ExactScalar(el.Qx), proofbound.ExactScalar(el.Qy), el.RrBS()
		eqs = append(eqs, CircEq{
			Quad: true,
			G:    proofbound.BoundedMul(proofbound.ExactScalar(-2), qx),
			H:    proofbound.BoundedMul(proofbound.ExactScalar(-2), qy),
			Kk:   proofbound.BoundedMul(proofbound.ExactScalar(2*s), rr),
			M: proofbound.BoundedSub(
				proofbound.BoundedAdd(proofbound.BoundedMul(qx, qx), proofbound.BoundedMul(qy, qy)),
				proofbound.BoundedMul(rr, rr),
			),
		})
	}
	for _, v := range k.Verts {
		if err := WallBudgetStep(budget); err != nil {
			return nil, err
		}
		vx, vy := proofbound.ExactScalar(v[0]), proofbound.ExactScalar(v[1])
		eqs = append(eqs, CircEq{
			Quad: true,
			G:    proofbound.BoundedMul(proofbound.ExactScalar(-2), vx),
			H:    proofbound.BoundedMul(proofbound.ExactScalar(-2), vy),
			M:    proofbound.BoundedAdd(proofbound.BoundedMul(vx, vx), proofbound.BoundedMul(vy, vy)),
		})
	}
	if k.HasWedge() {
		if err := WallBudgetStep(budget); err != nil {
			return nil, err
		}
		// The wedge's coefficient is the caller's already-bounded sine, so a
		// triple that includes the wedge inherits the sine's own error.
		eqs = append(eqs, CircEq{B: k.WedgeS, E: proofbound.ExactScalar(-1)})
	}
	return eqs, nil
}

// SolveTriple solves one triple of tangency equations in closed form:
// quadratic pairs subtract to linear, leaving at most one quadratic; two
// independent linears express the center affinely in r, and substitution
// yields a quadratic in r. Parallel linear pairs pin r instead and leave the
// position as the unknown.
//
// The parallel/independent split is the one reading here that decides between
// two GENERATORS rather than between a candidate and nothing, so an interval
// that cannot separate the pair's determinant from zero runs both: a
// superfluous candidate costs a validate pass and can never reach a reading
// it does not genuinely satisfy, while a missing one is a wall this survey
// would not see.
func SolveTriple(eqs [3]CircEq, scale float64, add func(x, y, r, rBound float64)) {
	var lins []CircEq
	var quad *CircEq
	for i, e := range eqs {
		if !e.Quad {
			lins = append(lins, e)
			continue
		}
		if quad == nil {
			q := eqs[i]
			quad = &q
			continue
		}
		// Subtract: the c·c − r² terms cancel.
		lins = append(lins, CircEq{
			A: proofbound.BoundedSub(e.G, quad.G), B: proofbound.BoundedSub(e.H, quad.H),
			E: proofbound.BoundedSub(e.Kk, quad.Kk), F: proofbound.BoundedSub(e.M, quad.M),
		})
	}
	if len(lins) == 3 {
		Solve3Linear(lins, add)
		return
	}
	if len(lins) != 2 || quad == nil {
		return
	}
	l1, l2 := lins[0], lins[1]
	detBS := proofbound.BoundedSub(proofbound.BoundedMul(l1.A, l2.B), proofbound.BoundedMul(l2.A, l1.B))
	parallel := proofbound.AdmitMagnitudeAbove(detBS, 1e-12*math.Max(1, scale))
	if parallel != proofbound.SurvAdmit {
		SolveParallelPair(l1, l2, *quad, add)
		if parallel == proofbound.SurvReject {
			return
		}
	}
	// (cx, cy) = P + r·Q from the two linears. The triple's OWN division (by
	// det) composes through proofbound.BoundedQuotient like every other division here, so
	// P and Q reach the quadratic below carrying their own error rather than
	// as exact leaves.
	pxBS := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l1.F), l2.B), proofbound.BoundedMul(l2.F, l1.B)), detBS)
	pyBS := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l1.A), l2.F), proofbound.BoundedMul(l2.A, l1.F)), detBS)
	qxBS := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l1.E), l2.B), proofbound.BoundedMul(l2.E, l1.B)), detBS)
	qyBS := proofbound.BoundedDiv(proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l1.A), l2.E), proofbound.BoundedMul(l2.A, l1.E)), detBS)
	// The same denominator guard QuadRootsBounded takes, read off the four
	// divisions themselves: proofbound.BoundedQuotient answers +Inf exactly where the
	// determinant's own interval left it no positive clearance, so a P or Q with
	// no finite bound says this pair is not separated from parallel and the
	// affine centre it would carry into the quadratic is not a disk anyone can
	// state. Dropping THIS generator's candidate here is local — the pair's own
	// reading came from SolveParallelPair a few lines above, which is why the
	// straddle runs both — and it is what keeps one straddled triple from
	// handing runBudget's aggregate an unbounded candidate and leaving the whole
	// survey undecided.
	if proofbound.IsNonFinite(pxBS.Bound) || proofbound.IsNonFinite(pyBS.Bound) || proofbound.IsNonFinite(qxBS.Bound) || proofbound.IsNonFinite(qyBS.Bound) {
		return
	}
	ABS := proofbound.BoundedSub(proofbound.BoundedAdd(proofbound.BoundedMul(qxBS, qxBS), proofbound.BoundedMul(qyBS, qyBS)), proofbound.ExactScalar(1))
	BBS := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.BoundedAdd(proofbound.BoundedMul(pxBS, qxBS), proofbound.BoundedMul(pyBS, qyBS))),
		proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedMul(quad.G, qxBS), proofbound.BoundedMul(quad.H, qyBS)), quad.Kk),
	)
	CBS := proofbound.BoundedAdd(
		proofbound.BoundedAdd(proofbound.BoundedMul(pxBS, pxBS), proofbound.BoundedMul(pyBS, pyBS)),
		proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedMul(quad.G, pxBS), proofbound.BoundedMul(quad.H, pyBS)), quad.M),
	)
	for _, rBS := range QuadRootsBounded(ABS, BBS, CBS) {
		r := rBS.Value
		if proofbound.AdmitAbove(rBS, 0) != proofbound.SurvReject {
			add(pxBS.Value+r*qxBS.Value, pyBS.Value+r*qyBS.Value, r, rBS.Bound)
		}
	}
}

// Solve3Linear solves three linear tangency equations by Cramer's rule. Every
// 2×2 minor, every expansion of it, and the final division compose through the
// bounded arithmetic, so the radius the candidate publishes carries the error
// of the WHOLE rule and not just its last division: a well-conditioned triple
// still rounds six products and three sums into each numerator before the
// quotient ever runs.
//
// A determinant this rule cannot separate from zero is the one reading here
// that stops rather than emits, and it drops no attained extremum: three
// tangency equations whose determinant the arithmetic cannot prove nonzero are
// dependent to within their own error, so they name no ISOLATED disk at all —
// the family they describe reaches the sink through the pair criticals and
// SolveParallelPair, which is what those generators are for. Emitting instead
// would publish a position and radius Cramer's rule divides out of a vanishing
// denominator, whose own +Inf bound would then leave every survey containing
// such a triple undecided — which for a rotated or placed section is most of
// them.
func Solve3Linear(l []CircEq, add func(x, y, r, rBound float64)) {
	// The 2×2 minors of rows 1 and 2 over each column pair, named for the
	// columns they keep.
	minor := func(p, q, s, t proofbound.BoundedScalar) proofbound.BoundedScalar {
		return proofbound.BoundedSub(proofbound.BoundedMul(p, t), proofbound.BoundedMul(s, q))
	}
	be := minor(l[1].B, l[1].E, l[2].B, l[2].E)
	ae := minor(l[1].A, l[1].E, l[2].A, l[2].E)
	ab := minor(l[1].A, l[1].B, l[2].A, l[2].B)
	fe := minor(l[1].F, l[1].E, l[2].F, l[2].E)
	fb := minor(l[1].F, l[1].B, l[2].F, l[2].B)
	af := minor(l[1].A, l[1].F, l[2].A, l[2].F)
	bf := minor(l[1].B, l[1].F, l[2].B, l[2].F)
	detBS := proofbound.BoundedAdd(
		proofbound.BoundedSub(proofbound.BoundedMul(l[0].A, be), proofbound.BoundedMul(l[0].B, ae)),
		proofbound.BoundedMul(l[0].E, ab),
	)
	if proofbound.AdmitMagnitudeAbove(detBS, SurvTiny) != proofbound.SurvAdmit {
		return
	}
	drBS := proofbound.BoundedSub(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l[0].A), bf), proofbound.BoundedMul(l[0].B, af)),
		proofbound.BoundedMul(l[0].F, ab),
	)
	rBS := proofbound.BoundedDiv(drBS, detBS)
	if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
		return
	}
	// The center is a position, never a published reading (see
	// PlaceCircleCircle), so it stays a plain quotient.
	dx := proofbound.BoundedSub(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l[0].F), be), proofbound.BoundedMul(l[0].B, fe)),
		proofbound.BoundedMul(l[0].E, fb),
	).Value
	dy := proofbound.BoundedSub(
		proofbound.BoundedAdd(proofbound.BoundedMul(proofbound.BoundedNeg(l[0].A), fe), proofbound.BoundedMul(l[0].F, ae)),
		proofbound.BoundedMul(l[0].E, af),
	).Value
	add(dx/detBS.Value, dy/detBS.Value, rBS.Value, rBS.Bound)
}

// SolveParallelPair handles two parallel linear tangency equations plus a
// quadratic: the pair fixes r and a line of centers; the quadratic picks the
// positions along it. The normalization, the 2×2 determinant and the final
// division all compose their bounds, so the radius every emitted position
// shares carries the error of the whole reduction; the positions themselves
// never feed a published reading and stay plain floats.
//
// A normal this rule cannot separate from zero is not a line: the equation
// carries no direction to normalize by, so there is nothing to solve and the
// reading stops, the same way Solve3Linear's dependent triple does. The
// orientation sign σ is the one reading here whose straddle is unreachable
// from its own algebra — the two normals are already unit vectors and the
// caller only reaches this function for a pair whose cross product is at or
// below its own parallelism threshold, so their dot product sits within a few
// ulps of ±1 and no interval this arithmetic produces spans zero.
func SolveParallelPair(l1, l2, q CircEq, add func(x, y, r, rBound float64)) {
	nBS := proofbound.BoundedNorm2(l1.A, l1.B)
	if proofbound.AdmitAbove(nBS, SurvTiny) != proofbound.SurvAdmit {
		return
	}
	// Normalize both to unit normals; solve the 2×2 system in (h, r) where
	// h = n̂·c along l1's normal.
	a1, b1 := proofbound.BoundedDiv(l1.A, nBS), proofbound.BoundedDiv(l1.B, nBS)
	e1, f1 := proofbound.BoundedDiv(l1.E, nBS), proofbound.BoundedDiv(l1.F, nBS)
	n2BS := proofbound.BoundedNorm2(l2.A, l2.B)
	if proofbound.AdmitAbove(n2BS, SurvTiny) != proofbound.SurvAdmit {
		return
	}
	a2, b2 := proofbound.BoundedDiv(l2.A, n2BS), proofbound.BoundedDiv(l2.B, n2BS)
	e2, f2 := proofbound.BoundedDiv(l2.E, n2BS), proofbound.BoundedDiv(l2.F, n2BS)
	// n̂2 = ±n̂1: sign σ.
	sigma := -1.0
	if proofbound.AdmitAbove(proofbound.BoundedAdd(proofbound.BoundedMul(a1, a2), proofbound.BoundedMul(b1, b2)), 0) == proofbound.SurvAdmit {
		sigma = 1
	}
	// eq1: h + e1·r + f1 = 0; eq2: σ·h + e2·r + f2 = 0.
	detBS := proofbound.BoundedSub(e2, proofbound.BoundedMul(proofbound.ExactScalar(sigma), e1))
	if proofbound.AdmitMagnitudeAbove(detBS, SurvTiny) == proofbound.SurvReject {
		return
	}
	rBS := proofbound.BoundedDiv(proofbound.BoundedSub(proofbound.BoundedMul(proofbound.ExactScalar(sigma), f1), f2), detBS)
	r := rBS.Value
	if proofbound.AdmitAbove(rBS, 0) == proofbound.SurvReject {
		return
	}
	h := -e1.Value*r - f1.Value
	// Centers: c = h·n̂1 + t·t̂1. Substitute into the quadratic.
	tx, ty := -b1.Value, a1.Value
	bx, by := h*a1.Value, h*b1.Value
	A := 1.0
	B := 2*(bx*tx+by*ty) + q.G.Value*tx + q.H.Value*ty
	C := bx*bx + by*by - r*r + q.G.Value*bx + q.H.Value*by + q.Kk.Value*r + q.M.Value
	for _, t := range QuadRoots(A, B, C) {
		add(bx+t*tx, by+t*ty, r, rBS.Bound)
	}
}

// QuadRoots returns the real roots of A·x² + B·x + C = 0 (both when A ≈ 0 —
// the linear case — and the full quadratic).
func QuadRoots(A, B, C float64) []float64 {
	if math.Abs(A) < SurvTiny {
		if math.Abs(B) < SurvTiny {
			return nil
		}
		return []float64{-C / B}
	}
	disc := B*B - 4*A*C
	if disc < 0 {
		return nil
	}
	s := math.Sqrt(disc)
	return []float64{(-B - s) / (2 * A), (-B + s) / (2 * A)}
}

// PlaceCircleCircle emits the (up to two) points at distance da from
// (ax, ay) and db from (bx, by), as candidate centers of radius r (with its
// own proven bound rBound, unaffected by the position solve below — the
// position never feeds a published reading).
func PlaceCircleCircle(ax, ay, da, bx, by, db float64, add func(x, y, r, rBound float64), r, rBound float64) {
	dx, dy := bx-ax, by-ay
	d := math.Hypot(dx, dy)
	if d < SurvTiny || da < 0 || db < 0 {
		return
	}
	a := (da*da - db*db + d*d) / (2 * d)
	h2 := da*da - a*a
	if h2 < 0 {
		return
	}
	h := math.Sqrt(h2)
	mx, my := ax+a*dx/d, ay+a*dy/d
	px, py := -dy/d, dx/d
	add(mx+h*px, my+h*py, r, rBound)
	if h > 0 {
		add(mx-h*px, my-h*py, r, rBound)
	}
}
