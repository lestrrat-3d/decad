package spine

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/polynomial"
)

var ErrNonFiniteClearancePolynomial = errors.New("decad: non-finite clearance polynomial coefficient")

// CircleParam is an evaluated circle parameterization: center + r(u·cosθ +
// v·sinθ) with u, v unit and orthogonal.
type CircleParam struct {
	C, U, V [3]float64
	R       float64
}

func (cp CircleParam) At(th float64) [3]float64 {
	s, c := math.Sincos(th)
	var out [3]float64
	for i := range out {
		out[i] = cp.C[i] + cp.R*(cp.U[i]*c+cp.V[i]*s)
	}
	return out
}

// LineCircleBracketsContext encloses every critical value of the distance from the
// circle to the infinite line (a, d̂): the P4 cell. Returns the brackets, or
// constant=true when the distance is constant over the circle.
func LineCircleBracketsContext(ctx context.Context, cp CircleParam, a, d [3]float64, slack float64) ([]polynomial.CritBracket, bool, error) {
	m := [3]float64{cp.C[0] - a[0], cp.C[1] - a[1], cp.C[2] - a[2]}
	dot := func(x, y [3]float64) float64 { return x[0]*y[0] + x[1]*y[1] + x[2]*y[2] }
	// D(θ) = |Q − a|² − ((Q − a)·d̂)², a degree-2 trig polynomial.
	baseConst, ok := polynomial.CsConst(dot(m, m) + cp.R*cp.R)
	if !ok {
		return nil, false, ErrNonFiniteClearancePolynomial
	}
	baseLin, ok := polynomial.CsLin(0, 2*cp.R*dot(m, cp.U), 2*cp.R*dot(m, cp.V))
	if !ok {
		return nil, false, ErrNonFiniteClearancePolynomial
	}
	t, ok := polynomial.CsLin(dot(m, d), cp.R*dot(cp.U, d), cp.R*dot(cp.V, d))
	if !ok {
		return nil, false, ErrNonFiniteClearancePolynomial
	}
	base := polynomial.CsAdd(baseConst, baseLin)
	dist2 := polynomial.CsSub(base, polynomial.CsMul(t, t))
	g := func(th float64) float64 {
		q := cp.At(th)
		rel := [3]float64{q[0] - a[0], q[1] - a[1], q[2] - a[2]}
		along := dot(rel, d)
		return math.Sqrt(math.Max(0, dot(rel, rel)-along*along))
	}
	return polynomial.TrigStationaryBracketsContext(ctx, polynomial.CsDerivTheta(dist2), g, cp.R, slack)
}

// CircleCircleBracketsContext encloses every critical value of the distance from
// circle 1 to circle 2 (unit normal n2): the P8 cell. The squared
// stationarity s·(w')² = r2²·(s')² covers every smooth critical point;
// callers guard the ρ = 0 kink separately (a circle meeting the other's
// axis), where the distance is not differentiable.
func CircleCircleBracketsContext(ctx context.Context, c1 CircleParam, c2 CircleParam, n2 [3]float64, slack float64) ([]polynomial.CritBracket, bool, error) {
	m := [3]float64{c1.C[0] - c2.C[0], c1.C[1] - c2.C[1], c1.C[2] - c2.C[2]}
	dot := func(x, y [3]float64) float64 { return x[0]*y[0] + x[1]*y[1] + x[2]*y[2] }
	wConst, ok := polynomial.CsConst(dot(m, m) + c1.R*c1.R)
	if !ok {
		return nil, false, ErrNonFiniteClearancePolynomial
	}
	wLin, ok := polynomial.CsLin(0, 2*c1.R*dot(m, c1.U), 2*c1.R*dot(m, c1.V))
	if !ok {
		return nil, false, ErrNonFiniteClearancePolynomial
	}
	h, ok := polynomial.CsLin(dot(m, n2), c1.R*dot(c1.U, n2), c1.R*dot(c1.V, n2))
	if !ok {
		return nil, false, ErrNonFiniteClearancePolynomial
	}
	w := polynomial.CsAdd(wConst, wLin)
	sp := polynomial.CsSub(w, polynomial.CsMul(h, h))
	wd := polynomial.CsDerivTheta(w)
	sd := polynomial.CsDerivTheta(sp)
	r2, ok := polynomial.CsConst(c2.R * c2.R)
	if !ok {
		return nil, false, ErrNonFiniteClearancePolynomial
	}
	f := polynomial.CsSub(polynomial.CsMul(sp, polynomial.CsMul(wd, wd)), polynomial.CsMul(r2, polynomial.CsMul(sd, sd)))
	g := func(th float64) float64 {
		p := c1.At(th)
		rel := [3]float64{p[0] - c2.C[0], p[1] - c2.C[1], p[2] - c2.C[2]}
		hh := dot(rel, n2)
		rho := math.Sqrt(math.Max(0, dot(rel, rel)-hh*hh))
		return math.Hypot(hh, rho-c2.R)
	}
	return polynomial.TrigStationaryBracketsContext(ctx, f, g, c1.R, slack)
}
