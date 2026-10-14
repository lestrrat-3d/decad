package loftmesh

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// separatingPlaneCertificate is certificate D of AuditLoftPairData: a plane
// through the one shared vertex that has triangle i's two other corners
// strictly on one side and triangle j's two other corners strictly on the
// other. a and b are the vertex indices of triangles i and j, and shared is
// the one index they have in common. It returns true only when the proof below
// holds; false sends the pair down the unchanged path.
//
// # Proof
//
// Let v be the shared vertex, q1 and q2 two float points, and
//
//	f(p) = det[q1 − v, q2 − v, p − v],
//
// the function proof.OrientSignFloat(v, q1, q2, p) signs. f is affine in p and
// f(v) = 0 exactly, whatever the float values of q1 and q2 are. OrientSignFloat
// declines unless every coordinate difference is zero or in [2^-300, 2^250]
// (proof/orientation.go), and its certain sign is then the exact sign of f at
// the stored float point.
//
//  1. Write triangle i's corners as v, a1, a2. Every point of the triangle is
//     p = αv + βa1 + γa2 with α + β + γ = 1 and α, β, γ ≥ 0, so
//     f(p) = βf(a1) + γf(a2). If f(a1) and f(a2) both have the strict sign σ,
//     then σf(p) > 0 unless β = γ = 0, that is unless p = v.
//  2. Likewise, if f(b1) and f(b2) both have the strict sign −σ, every point
//     of triangle j other than v has σf(p) < 0.
//  3. A common point other than v would satisfy both, so the intersection lies
//     in {v}. v is a corner of both triangles (the shared index), so the
//     intersection is exactly {v}.
//  4. {v} is the contact the pair's recorded adjacency expects, the one the
//     exact classification admits. The certificate therefore fires only on
//     pairs the reference path admits.
//
// A degenerate plane (q1 − v parallel to q2 − v) makes f identically zero, so
// every sign is zero or undecided and the certificate declines. A corner of
// one triangle that has the same coordinates as a corner of the other, at a
// different index, would have to lie strictly on both sides at once; the
// signs cannot both hold, so the certificate declines and the classification
// decides as before.
//
// The plane is chosen from the corners' directions at v, and q1, q2 are
// rounded floats. Neither choice is part of the proof: only the four strict
// signs, each certified by proof.OrientSignFloat, prove anything. An
// undecided or zero sign returns false. No exact fallback follows, because
// the pair then goes to the exact classification it would have reached.
//
// The certificate is off (returns false) for data built by
// NewLoftAuditDataWithExact, whose exact lifts may differ from the float
// corners, so a certain float sign need not describe the exact geometry.
func (d *LoftAuditData) separatingPlaneCertificate(a, b [3]int, i, j, shared int) bool {
	if d.exactLifts {
		return false
	}
	ti, tj := d.corners(i), d.corners(j)
	var v r3.Vec
	var ai, bj [2]r3.Vec
	if !splitSharedCorner(a, ti, shared, &v, &ai) {
		return false
	}
	if !splitSharedCorner(b, tj, shared, &v, &bj) {
		return false
	}
	q1, q2, ok := separatingPlanePoints(v, ai, bj)
	if !ok {
		return false
	}
	return separatesAtVertex(v, q1, q2, ai, bj)
}

// separatesAtVertex is the proof's four sign tests for the plane through v,
// q1 and q2: ai against bj, each side's two corners strictly on one side and
// the other side's strictly on the other. It reads nothing but those four
// certified signs, so it is sound for ANY q1 and q2.
func separatesAtVertex(v, q1, q2 r3.Vec, ai, bj [2]r3.Vec) bool {
	sigma, ok := proof.OrientSignFloat(v, q1, q2, ai[0])
	if !ok || sigma == 0 {
		return false
	}
	if s, ok := proof.OrientSignFloat(v, q1, q2, ai[1]); !ok || s != sigma {
		return false
	}
	for _, p := range bj {
		if s, ok := proof.OrientSignFloat(v, q1, q2, p); !ok || s != -sigma {
			return false
		}
	}
	return true
}

// splitSharedCorner stores the corner of tri (whose float corners are
// corners) with vertex index shared in v and the other two in others. It
// returns false unless exactly one corner carries that index.
func splitSharedCorner(tri [3]int, corners [3]r3.Vec, shared int, v *r3.Vec, others *[2]r3.Vec) bool {
	n := 0
	found := false
	for k, vertex := range tri {
		if vertex == shared {
			if found {
				return false
			}
			found = true
			*v = corners[k]
			continue
		}
		if n == len(others) {
			return false
		}
		others[n] = corners[k]
		n++
	}
	return found && n == len(others)
}

// separatingPlanePoints chooses two float points q1 and q2 such that the plane
// through v, q1 and q2 has normal near the difference of the two triangles'
// mean edge directions at v. It is a heuristic: separatingPlaneCertificate's
// proof reads only the signs it computes for the returned points. ok is false
// when a direction is undefined or a result is not finite.
func separatingPlanePoints(v r3.Vec, ai, bj [2]r3.Vec) (r3.Vec, r3.Vec, bool) {
	var dir [4]r3.Vec
	for k, p := range [...]r3.Vec{ai[0], ai[1], bj[0], bj[1]} {
		u, ok := unitVec(p.Sub(v))
		if !ok {
			return r3.Vec{}, r3.Vec{}, false
		}
		dir[k] = u
	}
	s, ok := unitVec(dir[0].Add(dir[1]).Sub(dir[2]).Sub(dir[3]))
	if !ok {
		return r3.Vec{}, r3.Vec{}, false
	}
	axis := r3.Vec{X: 1}
	if math.Abs(s.X) > 0.5 {
		axis = r3.Vec{Y: 1}
	}
	t1, ok := unitVec(s.Cross(axis))
	if !ok {
		return r3.Vec{}, r3.Vec{}, false
	}
	t2 := s.Cross(t1)
	length := ai[0].Sub(v).Len()
	q1 := v.Add(t1.Scale(length))
	q2 := v.Add(t2.Scale(length))
	if !finiteVec(q1) || !finiteVec(q2) {
		return r3.Vec{}, r3.Vec{}, false
	}
	return q1, q2, true
}

// unitVec scales u to length 1. ok is false when u has no finite nonzero length.
func unitVec(u r3.Vec) (r3.Vec, bool) {
	n := u.Len()
	if !(n > 0) || math.IsInf(n, 0) {
		return r3.Vec{}, false
	}
	w := u.Scale(1 / n)
	return w, finiteVec(w)
}
