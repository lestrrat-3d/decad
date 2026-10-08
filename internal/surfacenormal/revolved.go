package surfacenormal

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Revolved is the surface a revolve wall or cap DENOTES, enclosed exactly,
// beside the float tag the face publishes. A revolve's tag is not its record: its
// axis coordinates are a float re-expression of the recorded meridian, a
// centre or end within the contact tolerance of the axis is snapped onto it,
// a segment within the classifier's slope tolerance of parallel or
// perpendicular is tagged a Cylinder or a Plane, a cone's half angle is a
// float atan2, and every centre and origin is placed in float. So the normal
// a revolve wall's NormalAt publishes is judged against this surface, never
// against the tag (docs/evaluator-design.md §6).
//
// The surface is the recorded meridian swept about the recorded axis, in the
// form revolvemesh.RevolveLift.MeridianGap compares a carrier with: a point
// at meridian coordinates (z, ρ) and sweep angle φ sits at
//
//	Origin + Basis[0]·ρ·cos φ + Basis[1]·ρ·sin φ + Basis[2]·z
//
// with Basis the placed images of the axis frame's E0, E1 = W × E0 and W.
// Every leaf is the held float the record states, widened by its own proven
// bound — the axis anchor and direction, and each recorded meridian
// coordinate — so the enclosure holds the exact surface even though the
// basis is not exactly orthonormal: the frame's own axes and the placement's
// basis are held floats, and a direction widened by its bound is not unit.
// That is why a normal is mapped through the cofactor matrix of the basis
// rather than the basis itself: a linear map G carries a surface's normal by
// G⁻ᵀ, which is cof(G)/det(G).
//
// A straight wall holds one meridian run (dz, dρ) per recorded segment it
// covers, in walk order, and beside it that segment's two ends (z, ρ). A
// coalesced wall merges segments that are exactly collinear in the float
// axis coordinates, which their records need not be, so the reading takes
// the worst of its runs. A circular wall holds its recorded centre (z, ρ)
// and radius; the radius never reaches the normal.
//
// A partial sweep's planar cap holds no meridian at all: Cap marks it, and
// CapDir is its outward normal in the axis frame, (−sin φ, cos φ) on
// (Basis[0], Basis[1]) for the end cap and the negation for the start cap,
// at the angle φ that end's record denotes. The plane is spanned by the axis
// and the radial direction at φ, so its normal is W × radial(φ), which the
// cofactor matrix carries to exactly that.
type Revolved struct {
	Origin   proofbound.IvVec3
	Basis    [3]proofbound.IvVec3
	Runs     [][2]proofbound.RatInterval
	Ends     [][2][2]proofbound.RatInterval
	Circular bool
	Centre   [2]proofbound.RatInterval
	Radius   proofbound.RatInterval
	Cap      bool
	CapDir   [2]proofbound.RatInterval
	// Valid is false when a leaf was not finite, and every reading then
	// refuses.
	Valid bool
}

// Transformed is r's exact image under xform: the placement's own basis
// and translation are held floats read as exact leaves, the same reading
// revolvemesh.RevolveLift.SweptPointGap takes of a placement.
func (r Revolved) Transformed(xform r3.Transform) Revolved {
	if !r.Valid {
		return r
	}
	basis := xform.Basis()
	var cols [3]proofbound.IvVec3
	for i, v := range [...]r3.Vec{basis.EX, basis.EY, basis.EZ} {
		enc, ok := proofbound.IvVec3Of(v)
		if !ok {
			return Revolved{}
		}
		cols[i] = enc
	}
	tr, ok := proofbound.IvVec3Of(xform.Translation())
	if !ok {
		return Revolved{}
	}
	place := func(v proofbound.IvVec3) proofbound.IvVec3 {
		return proofbound.IvVec3Add(proofbound.IvVec3Add(proofbound.IvVec3Mul(cols[0], v[0]), proofbound.IvVec3Mul(cols[1], v[1])), proofbound.IvVec3Mul(cols[2], v[2]))
	}
	out := r
	out.Origin = proofbound.IvVec3Add(place(r.Origin), tr)
	for i, b := range r.Basis {
		out.Basis[i] = place(b)
	}
	return out
}

// Allow bounds how far held, the outward direction a wall's NormalAt
// computed from its tag at p, sits from the outward unit normal of the
// denoted surface at p. reversed is the face's own outward sign, which a
// circular wall and a cap read: a straight wall's outward side follows from
// its walk, whose material lies on its left. A cap's normal is the same at
// every p.
//
// p is located on the surface the way every NormalAt arm locates it: its
// meridian half-plane is the one through p, and its normal there is
// constant along a straight ruling, or runs from the rotated circle's centre
// for a circular wall. Locating p needs G⁻¹, which is adj(G)/det(G); the
// positive scale 1/|det| changes no direction, so only det's sign is read.
//
// The answer is the distance from held to the enclosure of that unit
// normal, so it covers the tag's whole departure — re-expression, snap,
// classification, angle and placement rounding — and the arithmetic the arm
// ran, at once.
func (r Revolved) Allow(p, held r3.Vec, reversed bool) (float64, Status) {
	if !r.Valid {
		return 0, Unproven
	}
	g0, g1, g2 := r.Basis[0], r.Basis[1], r.Basis[2]
	c0, c1, c2 := proofbound.IvVec3Cross(g1, g2), proofbound.IvVec3Cross(g2, g0), proofbound.IvVec3Cross(g0, g1)
	det := proofbound.IvVec3Dot(g0, c0)
	sigma := new(big.Rat)
	switch {
	case det.Lo.Sign() > 0:
		sigma.SetInt64(1)
	case det.Hi.Sign() < 0:
		sigma.SetInt64(-1)
	default:
		return 0, Unproven
	}
	sig := proofbound.PointInterval(sigma)
	pi, ok := proofbound.IvVec3Of(p)
	if !ok {
		return 0, Unproven
	}
	rel := proofbound.IvVec3Sub(pi, r.Origin)
	// k is det(G) times p's own meridian coordinates, so σ·(k₀, k₁, 0) points
	// along p's radial direction in the axis frame.
	k := proofbound.IvVec3{proofbound.IvVec3Dot(c0, rel), proofbound.IvVec3Dot(c1, rel), proofbound.IvVec3Dot(c2, rel)}
	zero := proofbound.PointInterval(new(big.Rat))
	radial := proofbound.IvVec3Mul(proofbound.IvVec3{k[0], k[1], zero}, sig)
	world := func(n proofbound.IvVec3) proofbound.IvVec3 {
		w := proofbound.IvVec3Add(proofbound.IvVec3Add(proofbound.IvVec3Mul(c0, n[0]), proofbound.IvVec3Mul(c1, n[1])), proofbound.IvVec3Mul(c2, n[2]))
		return proofbound.IvVec3Mul(w, sig)
	}
	if r.Cap {
		n := proofbound.IvVec3{r.CapDir[0], r.CapDir[1], zero}
		if reversed {
			n = proofbound.IvVec3Mul(n, proofbound.PointInterval(big.NewRat(-1, 1)))
		}
		return unitDirAllow(world(n), held)
	}
	if r.Circular {
		return r.circularAllow(k, det, sig, radial, world, held, reversed)
	}
	if len(r.Runs) == 0 {
		return 0, Unproven
	}
	worst := 0.0
	for _, run := range r.Runs {
		dz, drho := run[0], run[1]
		// The outward normal of a run walked with its material on the left
		// is dρ·ẑ − dz·r̂ in the axis frame. An exactly radial or exactly
		// axial run needs no unit radial, which keeps an exact wall's
		// enclosure free of a square root it does not need.
		var n proofbound.IvVec3
		switch {
		case isZero(drho):
			n = proofbound.IvVec3Mul(radial, proofbound.IntervalNeg(dz))
		case isZero(dz):
			n = proofbound.IvVec3{zero, zero, drho}
		default:
			rhat, st := UnitVec3(radial)
			if st != Proven {
				return 0, st
			}
			n = proofbound.IvVec3Sub(proofbound.IvVec3{zero, zero, drho}, proofbound.IvVec3Mul(rhat, dz))
		}
		allow, st := unitDirAllow(world(n), held)
		if st != Proven {
			return 0, st
		}
		worst = math.Max(worst, allow)
	}
	return worst, Proven
}

// circularAllow is Allow's circular-wall arm: the normal runs from the
// rotated circle's centre at p's azimuth, (ρc·r̂, zc), to p. Scaling that
// direction by the positive |det| gives σ·k − |det|·centre and avoids three
// interval divisions. A centre exactly on the axis needs no azimuth at
// all. Off it, a p whose own azimuth the enclosure cannot fix — one on or
// near the axis — still has a centre within ρc of the axis point zc, and
// the reading takes that whole box rather than refuse.
func (r Revolved) circularAllow(
	k proofbound.IvVec3, det, sig proofbound.RatInterval, radial proofbound.IvVec3,
	world func(proofbound.IvVec3) proofbound.IvVec3, held r3.Vec, reversed bool,
) (float64, Status) {
	absDet := det
	if det.Hi.Sign() < 0 {
		absDet = proofbound.IntervalNeg(det)
	}
	scaled := proofbound.IvVec3{radial[0], radial[1], proofbound.IntervalMul(k[2], sig)}
	zc, rc := r.Centre[0], r.Centre[1]
	zero := proofbound.PointInterval(new(big.Rat))
	centre := proofbound.IvVec3{zero, zero, zc}
	if !isZero(rc) {
		if rhat, st := UnitVec3(radial); st == Proven {
			centre = proofbound.IvVec3Add(centre, proofbound.IvVec3Mul(rhat, rc))
		} else {
			reach := proofbound.IntervalAbsUpper(rc)
			box := proofbound.Interval(new(big.Rat).Neg(reach), reach)
			centre = proofbound.IvVec3{box, box, zc}
		}
	}
	n := proofbound.IvVec3Sub(scaled, proofbound.IvVec3Mul(centre, absDet))
	if reversed {
		n = proofbound.IvVec3Mul(n, proofbound.PointInterval(big.NewRat(-1, 1)))
	}
	return unitDirAllow(world(n), held)
}

func isZero(a proofbound.RatInterval) bool { return a.Lo.Sign() == 0 && a.Hi.Sign() == 0 }
