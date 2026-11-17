package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file reads an arc end that meets a retained cylinder G (a window corner
// round, a hole's wall) and sizes the rows' end curve on it. G is parallel to
// the arc's axis. A row, the circle of radius ρ at height z about the rim's
// axis O, meets G where it crosses G's circle of radius r_G about c, the point
// where G's axis meets the rim's plane, at in-plane distance d from O. The
// crossing's angle at c, measured from c→O, is ψ, cos ψ = (d² + r_G² − ρ²)/(2·d·r_G),
// and the row's azimuth about O is θ_c ± α, cos α = (ρ² + d² − r_G²)/(2·ρ·d).

// cylEnd is an arc end on a retained cylinder.
type cylEnd struct {
	face *topo.Face
	// c is G's axis where it meets the rim's plane, axis its unit direction.
	c, axis r3.Vec
	rG      float64
	// d is the in-plane distance from the rim's axis to c.
	d float64
	// thetaC is the signed angle θ = sign·φ of c about the rim's axis, in
	// (−π, π].
	thetaC float64
	// sigma is +1 at end 0 and −1 at end 1: the root side. G is Reversed, so the
	// arc runs outside its disk; in the direction of growing θ the arc leaves
	// the disk at θ_c + α, which is where end 0 lies, and enters it at θ_c − α,
	// where end 1 lies.
	sigma float64
	// eV bounds the rounding of the roots V and V*, sV is the charged crossing
	// sine at the rim, other the root V* the end is not at, eps = |axis × n| the
	// tilt of G off the rim's axis and sFar the scale S_far the bounds use.
	eV, sV float64
	other  r3.Vec
	eps    float64
	sFar   float64
	// dir is +1 or -1: the sense in which A's chain leaves V along G's circle,
	// as the angle about c grows or shrinks (readCylChain). Zero until read.
	dir float64
	// noise lists the held vertices of the wall's boundary beside V that lie within
	// their bounds of the end's radial plane and are read as on it (cylBandEnd), from
	// V outward.
	noise []int
	// tan is set for a C3 end (arc_end_tangent.go): c, d and rG then read G*.
	tan *tangentEnd
}

// farScale is S_far: an upper bound on the coordinates and lengths an end's
// arithmetic reads. scale is the body's largest coordinate.
func farScale(scale float64, center, origin r3.Vec, radius, rG, d float64) float64 {
	return scale + center.Len() + origin.Len() + radius + rG + d
}

// crossingSine is the charged sine s_V of the angle at which the rim's circle
// of radius R meets G's circle of radius rG about a centre at distance d:
// sqrt(1 − (|cos V| + err_c)²) less G's tilt eps (as an angle, eps + eps³) and
// the faces' normal bounds nb. Its sign proves the two circles cross in two
// distinct points (agent-docs/boss-root-overflow-design.md §5.3). sFar is
// S_far; e_d = 16·ulp·S_far bounds the rounding of d.
func crossingSine(R, rG, d, sFar, eps, nb float64) float64 {
	ed := 16 * ulp * sFar
	cosV := (R*R + rG*rG - d*d) / (2 * R * rG)
	errC := (16*ulp*(R*R+rG*rG+d*d) + 2*d*ed + ed*ed) / (2 * R * rG)
	c := math.Abs(cosV) + errC
	return math.Sqrt(math.Max(0, 1-c*c)) - (eps + eps*eps*eps) - nb
}

// rootBound is e_V, the bound on the rounding of the roots V and V*: acos
// amplifies the rounding of its argument by 1/sin α, and sin α ≥ r_G·s_V/d by
// the law of sines; the next term covers atTheta's and θ_c's own rounding at
// the coordinates' scale, and the last the displacement r_G·ε_G² of G's tilted
// section from its circle.
func rootBound(sFar, rG, sV, eps float64) float64 {
	return 64*ulp*sFar*math.Max(1, sFar/rG)/sV + 8*ulp*sFar + rG*eps*eps
}

// cylRoots are the two points of the rim's circle on G's circle, V on the side
// the end's sigma names and V* the other, each angle unwrapped to within π of ref.
func (a arcRim) cylRoots(c *cylEnd, ref float64) (v, other r3.Vec) {
	alpha := math.Acos((a.radius*a.radius + c.d*c.d - c.rG*c.rG) / (2 * a.radius * c.d))
	unwrap := func(t float64) float64 { return t + 2*math.Pi*math.Round((ref-t)/(2*math.Pi)) }
	return a.atTheta(a.radius, unwrap(c.thetaC+c.sigma*alpha), 0),
		a.atTheta(a.radius, unwrap(c.thetaC-c.sigma*alpha), 0)
}

// azimuth is the signed angle at which the circle of radius rho meets G on the
// end's root side, θ_c + σ·α(ρ), within π of ref, and false when the circle
// does not cross G's circle.
func (c *cylEnd) azimuth(rho, ref float64) (float64, bool) {
	arg := (rho*rho + c.d*c.d - c.rG*c.rG) / (2 * rho * c.d)
	if !(math.Abs(arg) < 1) {
		return 0, false
	}
	t := c.thetaC + c.sigma*math.Acos(arg)
	return t + 2*math.Pi*math.Round((ref-t)/(2*math.Pi)), true
}

// readCylEnd reads end i of the arc, whose corner is end, as a meeting with a
// retained cylinder. Every refusal is ErrUnsupported.
func (a *arcRim) readCylEnd(i int, end lineEnd, fp facetedrecord.Payload) (lineEnd, *cylEnd, error) {
	f := end.face
	notKind := fmt.Errorf("%w: an arc end meets a face that is not a retained plane or cylinder", decaderr.ErrUnsupported)
	cyl, ok := f.Surface.(surfacegeom.Cylinder)
	if !ok || !f.RetainedTag || f.TagDeparture != 0 || !tagIsRecord(f) || (f.HasAxialDelta && f.AxialDelta != 0) {
		return lineEnd{}, nil, notKind
	}
	rG, err := cyl.Radius.In(units.Millimeter)
	if err != nil || !(rG > 0) || math.IsInf(rG, 0) {
		return lineEnd{}, nil, notKind
	}
	axis, ok := cyl.Axis.Normalize()
	if !ok {
		return lineEnd{}, nil, notKind
	}
	if !f.Reversed {
		return lineEnd{}, nil, fmt.Errorf("%w: an arc end meets a cylinder whose material lies inside it", decaderr.ErrUnsupported)
	}
	eps := axis.Cross(a.n).Len()
	if !(eps <= parallelTol) {
		return lineEnd{}, nil, fmt.Errorf("%w: an arc end meets a cylinder that is not parallel to the axis", decaderr.ErrUnsupported)
	}
	c := cyl.Origin.Add(axis.Scale(a.n.Dot(a.center.Sub(cyl.Origin)) / a.n.Dot(axis)))
	q := c.Sub(a.center)
	d := math.Hypot(q.Dot(a.u), q.Dot(a.w))
	slack := a.slack()
	if !(d > 64*slack) {
		return lineEnd{}, nil, fmt.Errorf("%w: an arc end meets a cylinder coaxial with the wall", decaderr.ErrUnsupported)
	}
	sFar := farScale(a.scale, a.center, cyl.Origin, a.radius, rG, d)
	sV := crossingSine(a.radius, rG, d, sFar, eps, f.NormalBound+a.wallFace.NormalBound)
	if !(sV > minSine) {
		return lineEnd{}, nil, fmt.Errorf("%w: an arc end face meets the wall at a grazing angle", decaderr.ErrUnsupported)
	}
	ce := &cylEnd{face: f, c: c, axis: axis, rG: rG, d: d, sigma: 1, sV: sV, eps: eps, sFar: sFar}
	if i == 1 {
		ce.sigma = -1
	}
	ce.thetaC = a.sign * math.Atan2(q.Dot(a.w), q.Dot(a.u))
	ce.eV = rootBound(sFar, rG, sV, eps)
	held := fp.Verts[end.v]
	end.exact, ce.other = a.cylRoots(ce, a.sign*a.angle(held))
	if !(end.exact.Sub(held).Len() <= fp.VertexBound[end.v]+slack+ce.eV) {
		return lineEnd{}, nil, fmt.Errorf("%w: an arc end is not where its three tags meet", decaderr.ErrUnsupported)
	}
	return end, ce, nil
}

// cylEndSection is the end curve a blend cuts on the cylinder end c (§6.3, "An
// arc's rows" on a cylinder). The curve is (u, v) = (−κ·r_G·ψ(s), side·z(s)) in
// G's unrolled coordinates, both growing with s. With q = sin∠OPC = d·sin ψ/ρ
// the tangent angle obeys tan ψ_t = g(s)·q(s), as X/ρ does on a plane.
// It refuses a row circle that does not cross G's circle or a profile where
// ρ² + d² ≤ r_G², where cos∠OPC is not monotone in s.
func (a arcRim) cylEndSection(spec blendSpec, c *cylEnd) (endSection, error) {
	if c.tan != nil {
		return a.tangentEndSection(spec, c)
	}
	p, fillet := a.profile(spec)
	rho0, z0 := p(0)
	rho1, z1 := p(1)
	rhoLo, rhoHi := math.Min(rho0, rho1), math.Max(rho0, rho1)
	if !(rhoLo > math.Abs(c.d-c.rG)) || !(rhoHi < c.d+c.rG) {
		return endSection{}, fmt.Errorf("%w: a blend circle does not reach an end face", decaderr.ErrUnsupported)
	}
	if !(rhoLo*rhoLo+c.d*c.d > c.rG*c.rG) {
		return endSection{}, fmt.Errorf("%w: a blend circle meets an end face where the end curve folds back",
			decaderr.ErrUnsupported)
	}
	e := endSection{p: p, kappa: a.kappa, side: a.side, cyl: &cylCurve{rG: c.rG, d: c.d}}
	e.slope = profileSlope(spec, fillet)
	_, _, q0 := e.at(0)
	_, _, q1 := e.at(1)
	q := math.Min(q0, q1)
	zHi := math.Max(math.Abs(z0), math.Abs(z1))
	sFar := c.sFar
	rLo := math.Min(rhoLo, c.rG)
	e.margin = 64*ulp*sFar*(sFar/rLo)*(1+1/q) + 2*c.eps*(zHi+c.rG)*(1+1/q)
	ed := 16 * ulp * sFar
	dc := (8*ulp*(rhoHi*rhoHi+c.rG*c.rG+c.d*c.d)+2*c.d*ed+ed*ed)/(2*rhoLo*c.rG) +
		4*ulp*sFar*(rhoHi*rhoHi+c.d*c.d)/(2*rhoLo*rhoLo*c.rG)
	e.angle = 2 * (dc/(q*q) + 6*ulp/q + 2*ulp)
	return e, nil
}

// cylCurve holds what an end curve on a cylinder needs besides the profile:
// G's radius and the distance of its axis from the rim's.
type cylCurve struct {
	rG, d float64
	// tan and ts are set for a C3 end (arc_end_tangent_section.go).
	tan *tangentEnd
	ts  *tangentSection
}

func clampUnit(x float64) float64 { return math.Max(-1, math.Min(1, x)) }

// psi is the angle at G's axis between the direction to the rim's axis and the
// row's crossing, for a row of radius rho.
func (c cylCurve) psi(rho float64) float64 {
	return math.Acos(clampUnit((c.d*c.d + c.rG*c.rG - rho*rho) / (2 * c.d * c.rG)))
}

// azimuth is the row's azimuth distance α from G's centre about the rim's axis.
func (c cylCurve) azimuth(rho float64) float64 {
	return math.Acos(clampUnit((rho*rho + c.d*c.d - c.rG*c.rG) / (2 * rho * c.d)))
}

// cosOPC is cos∠OPC at the row's crossing P with centre c, O the rim's axis.
func (c cylCurve) cosOPC(rho float64) float64 {
	return (rho*rho + c.rG*c.rG - c.d*c.d) / (2 * rho * c.rG)
}

// q is sin∠OPC.
func (c cylCurve) q(rho float64) float64 {
	k := clampUnit(c.cosOPC(rho))
	return math.Sqrt(math.Max(0, 1-k*k))
}

// curv is the 3D chord's departure from the cylinder image of the unrolled
// chord over [s0, s1]: r_G·Δψ²/2 with Δψ the span of the crossing angle widened
// by twice angleErr, once per gap. The factor (1 + 16·ulp) covers the rounding
// of the product.
func (e endSection) curv(s0, s1 float64) float64 {
	if e.cyl == nil {
		return 0
	}
	dpsi := math.Abs(e.psiAt(s1)-e.psiAt(s0)) + 2*e.curvAngle(s0, s1)
	return e.cyl.rG * dpsi * dpsi / 2 * (1 + 16*ulp)
}

// curvAngle is the angle allowance curv widens the span of ψ by.
func (e endSection) curvAngle(s0, s1 float64) float64 {
	if e.cyl.tan != nil {
		return e.pieceAngle(s0, s1)
	}
	return e.angle
}

// qHi is the largest q over [t0, t1], given q at the piece's ends: on a
// cylinder q reaches 1 inside a piece where cos∠OPC changes sign.
func (e endSection) qHi(t0, t1, q0, q1 float64) float64 {
	hi := math.Max(q0, q1)
	if e.cyl == nil {
		return hi
	}
	rho0, _ := e.p(t0)
	rho1, _ := e.p(t1)
	if e.cyl.cosOPC(rho0)*e.cyl.cosOPC(rho1) < 0 {
		return 1
	}
	return hi
}
