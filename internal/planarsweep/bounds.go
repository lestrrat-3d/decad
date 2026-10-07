package planarsweep

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Motion is one body's exact first-order rigid motion in world coordinates.
// A path that does not rotate has a zero omega, and its center is its first
// start vertex, so every distance below stays defined.
type Motion struct {
	Velocity motionbound.RatVec // mm/s
	Omega    motionbound.RatVec // rad/s
	Center   motionbound.RatVec // world pivot at the start
	Rotating bool               // omega is nonzero
	OmegaSq  *big.Rat           // exact |ω|²
	OmegaUp  *big.Rat           // upper bound on |ω|
	Rho      *big.Rat           // upper bound on the largest start-vertex distance from center
}

// MotionInput contains the prepared path values needed by MotionOf.
type MotionInput struct {
	Points   []proofarith.DyV3
	Delta    [3]proofarith.Dyadic
	Duration *big.Rat
	Velocity motionbound.RatVec
	Frame    motionbound.MotionFrame
	Drift    bool
	Screw    bool
}

// MotionOf bounds the rotation and the largest vertex distance from its pivot.
func MotionOf(p MotionInput) (Motion, bool) {
	if p.Screw || len(p.Points) == 0 {
		return Motion{}, false
	}
	m := Motion{Omega: motionbound.RatVec{new(big.Rat), new(big.Rat), new(big.Rat)},
		OmegaSq: new(big.Rat), OmegaUp: new(big.Rat)}
	if !p.Drift {
		for k := range 3 {
			m.Velocity[k] = new(big.Rat).Quo(p.Delta[k].Rat(), p.Duration)
		}
		m.Center = ratOfDyV3(p.Points[0])
	} else {
		for k := range 3 {
			if p.Velocity[k] == nil || p.Frame.Axis[k] == nil || p.Frame.Center[k] == nil {
				return Motion{}, false
			}
			m.Velocity[k] = new(big.Rat).Set(p.Velocity[k])
			m.Omega[k] = new(big.Rat).Set(p.Frame.Axis[k])
			m.Center[k] = new(big.Rat).Set(p.Frame.Center[k])
		}
		m.Rotating = true
		m.OmegaSq = ratDot3(m.Omega, m.Omega)
		up, ok := ratSqrtUpRat(m.OmegaSq)
		if !ok {
			return Motion{}, false
		}
		m.OmegaUp = up
	}
	rhoSq := new(big.Rat)
	for _, v := range p.Points {
		d := ratSub3(ratOfDyV3(v), m.Center)
		if sq := ratDot3(d, d); sq.Cmp(rhoSq) > 0 {
			rhoSq = sq
		}
	}
	rho, ok := ratSqrtUpRat(rhoSq)
	if !ok {
		return Motion{}, false
	}
	m.Rho = rho
	return m, true
}

// VertexSpins returns, for every start vertex p of a path, an upper bound on
// |ω|·|ω×(p − c)|, the magnitude of p's second derivative under the path's
// drift: p(u) = c + v·u + R(ωu)·(p − c), so p”(u) = R(ωu)·(ω×(ω×(p − c))),
// whose length |ω|·|ω×(p − c)| holds for every u (§10.8). Each factor is
// rounded up. A path that does not rotate has a zero spin at every vertex.
func VertexSpins(points []proofarith.DyV3, motion Motion) ([]*big.Rat, bool) {
	out := make([]*big.Rat, len(points))
	for i, v := range points {
		if !motion.Rotating {
			out[i] = new(big.Rat)
			continue
		}
		arm := ratCross3(motion.Omega, ratSub3(ratOfDyV3(v), motion.Center))
		armUp, ok := ratSqrtUpRat(ratDot3(arm, arm))
		if !ok {
			return nil, false
		}
		out[i] = proofbound.RatMul(motion.OmegaUp, armUp)
	}
	return out, true
}

// Rates returns each guest vertex's exact initial height rate.
func Rates(points []proofarith.DyV3, normal proofarith.DyV3, m, o Motion) []*big.Rat {
	out := make([]*big.Rat, len(points))
	relative := ratSub3(m.Velocity, o.Velocity)
	n := ratOfDyV3(normal)
	for i, v := range points {
		p := ratOfDyV3(v)
		rate := ratAdd3(relative, ratCross3(m.Omega, ratSub3(p, m.Center)))
		rate = ratSub3(rate, ratCross3(o.Omega, ratSub3(p, o.Center)))
		out[i] = ratDot3(n, rate)
	}
	return out
}

// RestedVertices returns the support's lifted vertices that §10.8 rests under
// a rest speed: those whose exact start rate satisfies h'(0) >= −rest·|n|_lo,
// so they close on the plane no faster than rest, or rise. A zero rest speed
// rests none.
func RestedVertices(lifted []int, rates []*big.Rat, nLow, rest *big.Rat) map[int]struct{} {
	if rest.Sign() <= 0 || len(lifted) == 0 {
		return nil
	}
	floor := new(big.Rat).Neg(proofbound.RatMul(rest, nLow))
	out := make(map[int]struct{})
	for _, index := range lifted {
		if rates[index].Cmp(floor) >= 0 {
			out[index] = struct{}{}
		}
	}
	return out
}

// Curvature returns K_p for every guest vertex p, for the unnormalized
// heights over [0, t] seconds: half of |n| times the §10.2 bound on the
// second derivative of p's height,
//
//	|ω_M|·|ω_M×(p − c_M)| + |ω_S|²·ρ_S                     from p'' and q''
//	+ 2·|ω_S|·(|v_M − v_S| + |ω_M|·ρ_M + |ω_S|·ρ_S)       from 2·n'·(p' − q')
//	+ |ω_S|²·D,  D = ρ_M + ρ_S + |c_M − c_S| + |v_M − v_S|·t   from n''·(p − q)
//
// with q the foot of S's pivot on the plane: it moves rigidly with S, lies
// within ρ_S of the pivot (the plane holds an S vertex), and stays within D
// of every M vertex. The p” term is p's own (VertexSpins, §10.8), at most
// §10.2's |ω_M|²·ρ_M; the S terms stay global. The terms are nondecreasing
// in t, so K_p(t) covers [0, t].
func Curvature(m, o Motion, spin []*big.Rat, nHigh, t *big.Rat) ([]*big.Rat, bool) {
	relative := ratSub3(m.Velocity, o.Velocity)
	speed, okSpeed := ratSqrtUpRat(ratDot3(relative, relative))
	offset := ratSub3(m.Center, o.Center)
	distance, okDistance := ratSqrtUpRat(ratDot3(offset, offset))
	if !okSpeed || !okDistance {
		return nil, false
	}
	shared := proofbound.RatMul(o.OmegaSq, o.Rho)
	lever := proofbound.RatAdd(speed, proofbound.RatMul(m.OmegaUp, m.Rho), proofbound.RatMul(o.OmegaUp, o.Rho))
	shared.Add(shared, proofbound.RatMul(big.NewRat(2, 1), o.OmegaUp, lever))
	reach := proofbound.RatAdd(m.Rho, o.Rho, distance, proofbound.RatMul(speed, t))
	shared.Add(shared, proofbound.RatMul(o.OmegaSq, reach))
	half := proofbound.RatMul(nHigh, big.NewRat(1, 2))
	out := make([]*big.Rat, len(spin))
	for i, vertexSpin := range spin {
		out[i] = proofbound.RatMul(proofbound.RatAdd(shared, vertexSpin), half)
	}
	return out, true
}

// ClearAt reports whether every vertex outside the contact set, the lifted
// set's included (§10.5), keeps a positive height through [0, t]:
// h(0) + h'(0)·u − K_p·u² is concave and positive at u = 0, so its value at t
// decides the whole span. A lifted vertex that would reach the plane inside
// the slice therefore ends a band track before it does. Under rest a rested
// vertex (§10.8) is skipped: a band track holds it on both sides of the plane
// (DepthAt). A departure passes rest false, so every vertex outside its
// contact set stays positive, rested or not: it claims strict separation.
func ClearAt(heights, rates []*big.Rat, contact []int, rested map[int]struct{},
	t *big.Rat, k []*big.Rat, rest bool) bool {
	contactIndex := 0
	for i, height := range heights {
		if contactIndex < len(contact) && contact[contactIndex] == i {
			contactIndex++
			continue
		}
		if _, held := rested[i]; rest && held {
			continue
		}
		value := proofbound.RatAdd(height, proofbound.RatMul(rates[i], t))
		value.Sub(value, proofbound.RatMul(k[i], t, t))
		if value.Sign() <= 0 {
			return false
		}
	}
	return true
}

// DepthAt is the band's unnormalized depth at elapsed time t under the
// per-vertex curvature k, read over [0, t], the largest of three bounds:
// r·t + K_p·t² over the contact set, r the largest contact rate magnitude;
// h0 + |h'(0)|·t + K_p·t² over the rested set (§10.8); and
// h0 + max(0, h'(0))·t + K_p·t² over the rest of the lifted set (§10.5). A
// contact vertex's height lies in [−(r·t + K_p·t²), r·t + K_p·t²] and a rested
// vertex's in [−(h0 + |h'(0)|·t + K_p·t²), h0 + |h'(0)|·t + K_p·t²], both by
// Taylor's theorem; any other lifted vertex, which ClearAt keeps above the
// plane, lies in (0, h0 + max(0, h'(0))·t + K_p·t²].
func DepthAt(heights, rates []*big.Rat, contact, lifted []int, rested map[int]struct{},
	t *big.Rat, k []*big.Rat, rate *big.Rat) *big.Rat {
	depth := new(big.Rat)
	for _, index := range contact {
		bound := proofbound.RatAdd(proofbound.RatMul(rate, t), proofbound.RatMul(k[index], t, t))
		if bound.Cmp(depth) > 0 {
			depth = bound
		}
	}
	for _, index := range lifted {
		bound := proofbound.RatAdd(heights[index], proofbound.RatMul(k[index], t, t))
		_, held := rested[index]
		if held || rates[index].Sign() > 0 {
			bound.Add(bound, proofbound.RatMul(new(big.Rat).Abs(rates[index]), t))
		}
		if bound.Cmp(depth) > 0 {
			depth = bound
		}
	}
	return depth
}

func ratOfDyV3(v proofarith.DyV3) motionbound.RatVec {
	return motionbound.RatVec{v[0].Rat(), v[1].Rat(), v[2].Rat()}
}

func ratAdd3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{new(big.Rat).Add(a[0], b[0]), new(big.Rat).Add(a[1], b[1]), new(big.Rat).Add(a[2], b[2])}
}

func ratSub3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func ratDot3(a, b motionbound.RatVec) *big.Rat {
	return proofbound.RatAdd(new(big.Rat).Mul(a[0], b[0]), new(big.Rat).Mul(a[1], b[1]), new(big.Rat).Mul(a[2], b[2]))
}

func ratCross3(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

func ratSqrtUpRat(q *big.Rat) (*big.Rat, bool) {
	up := proofbound.RatSqrtUp(q)
	if math.IsNaN(up) || math.IsInf(up, 0) {
		return nil, false
	}
	return proofarith.FloatRat(up), true
}
