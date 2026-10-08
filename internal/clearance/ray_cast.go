package clearance

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/polynomial"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// ClrLadder is the deterministic cast-direction ladder of §2: fixed,
// never random, so a replay resolves identically.
func ClrLadder() []r3.Vec {
	out := make([]r3.Vec, 16)
	for i := range out {
		th := 0.5 + float64(i)*2.399963229728653
		z := 1 - 2*(float64(i)+0.5)/16
		r := math.Sqrt(math.Max(0, 1-z*z))
		out[i] = r3.NewVec(r*math.Cos(th), r*math.Sin(th), z)
	}
	return out
}

// survey2d.RayCrossings counts certified transversal crossings of the ray p + t·dir
// (t > tol) with the trimmed face; good is false on any ambiguity.
func (f *CFace) RayCrossings(ctx context.Context, p, dir r3.Vec, tol float64) (int, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	switch f.Kind {
	case CkPlane:
		nd := f.N.Dot(dir)
		if nd == 0 {
			// EXACTLY parallel: the ray never meets the carrier when the
			// start is cleanly off the plane; on the plane it is a graze —
			// ambiguous, retry the ladder.
			if math.Abs(f.N.Dot(p)-f.PlaneOffset()) > tol {
				return 0, true, nil
			}
			return 0, false, nil
		}
		if math.Abs(nd) < 1e-7 {
			// Near-parallel: the crossing exists but sits far away and
			// poorly conditioned — the count is not certified; retry the
			// ladder rather than claim zero.
			return 0, false, nil
		}
		t := (f.PlaneOffset() - f.N.Dot(p)) / nd
		if math.Abs(t) <= tol {
			// The carrier passes through the ray start (a witness on a
			// shared construction plane): a crossing only if the start sits
			// on the trimmed face — cleanly off it, no crossing.
			if f.AdmitPoint(p.Add(dir.Scale(t)), tol) == -1 {
				return 0, true, nil
			}
			return 0, false, nil
		}
		if t < 0 {
			return 0, true, nil
		}
		switch f.AdmitPoint(p.Add(dir.Scale(t)), tol) {
		case 1:
			return 1, true, nil
		case -1:
			return 0, true, nil
		default:
			return 0, false, nil
		}
	case CkCylinder:
		rel := p.Sub(f.Anchor)
		relP := rel.Sub(f.Axis.Scale(rel.Dot(f.Axis)))
		dirP := dir.Sub(f.Axis.Scale(dir.Dot(f.Axis)))
		a := dirP.Dot(dirP)
		b := relP.Dot(dirP)
		c := relP.Dot(relP) - f.Radius*f.Radius
		n, ok := f.QuadraticCrossings(p, dir, a, b, c, tol)
		return n, ok, nil
	case CkSphere:
		rel := p.Sub(f.Anchor)
		n, ok := f.QuadraticCrossings(p, dir, 1, rel.Dot(dir), rel.Dot(rel)-f.Radius*f.Radius, tol)
		return n, ok, nil
	case CkCone:
		rel := p.Sub(f.Anchor)
		k := f.ConeTan()
		az := rel.Dot(f.Axis)
		dz := dir.Dot(f.Axis)
		relP := rel.Sub(f.Axis.Scale(az))
		dirP := dir.Sub(f.Axis.Scale(dz))
		a := dirP.Dot(dirP) - k*k*dz*dz
		b := relP.Dot(dirP) - k*k*az*dz
		c := relP.Dot(relP) - k*k*az*az
		n, ok := f.QuadraticCrossings(p, dir, a, b, c, tol)
		if !ok {
			return 0, false, nil
		}
		return n, true, nil
	default: // CkTorus
		return f.TorusCrossings(ctx, p, dir, tol)
	}
}

// quadraticCrossings counts admitted roots of a·t² + 2b·t + c = 0 along the
// ray; the cone caller relies on admitPoint's z-window (from the apex,
// positive) to reject the wrong nappe.
func (f *CFace) QuadraticCrossings(p, dir r3.Vec, a, b, c, tol float64) (int, bool) {
	if math.Abs(a) < 1e-14 {
		return 0, false
	}
	disc := b*b - a*c
	scale := math.Abs(a)*f.Radius*f.Radius + math.Abs(c) + 1
	if f.Kind == CkCone {
		scale = math.Abs(c) + math.Abs(b) + 1
	}
	if disc <= 0 {
		if disc > -1e-9*scale {
			return 0, false
		}
		return 0, true
	}
	s := math.Sqrt(disc)
	if s <= 1e-9*math.Sqrt(scale) {
		return 0, false
	}
	n := 0
	for _, t := range []float64{(-b - s) / a, (-b + s) / a} {
		q := p.Add(dir.Scale(t))
		if f.Kind == CkCone && q.Sub(f.Anchor).Dot(f.Axis) < 0 {
			continue
		}
		if math.Abs(t) <= tol {
			// The carrier passes through the ray start: cleanly off the
			// trimmed face means no crossing; anything else is ambiguous.
			if f.AdmitPoint(q, tol) == -1 {
				continue
			}
			return 0, false
		}
		if t < 0 {
			continue
		}
		switch f.AdmitPoint(q, tol) {
		case 1:
			n++
		case -1:
		default:
			return 0, false
		}
	}
	return n, true
}

// torusCrossings counts admitted ray roots of the torus quartic with Sturm
// certification (§2): a root count is a proof only when the isolation
// certifies it — a non-square-free quartic (a tangency) is ambiguous and the
// ladder retries.
func (f *CFace) TorusCrossings(ctx context.Context, p, dir r3.Vec, tol float64) (int, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	rel := p.Sub(f.Anchor)
	// |x−C|² = q2 t² + q1 t + q0; axial² = (a0 + a1 t)².
	q2 := dir.Dot(dir)
	q1 := 2 * rel.Dot(dir)
	q0 := rel.Dot(rel)
	a0 := rel.Dot(f.Axis)
	a1 := dir.Dot(f.Axis)
	k := f.Major*f.Major + q0 - f.Radius*f.Radius
	// f(t) = (|x−C|² + R² − r²)² − 4R²(|x−C|² − axial²)
	quad, ok := polynomial.RatPolyOf(k, q1, q2)
	if !ok {
		return 0, false, nil
	}
	sq := polynomial.RpMul(quad, quad)
	perpBase, ok := polynomial.RatPolyOf(q0, q1, q2)
	if !ok {
		return 0, false, nil
	}
	axial, ok := polynomial.RatPolyOf(a0, a1)
	if !ok {
		return 0, false, nil
	}
	perp := polynomial.RpSub(perpBase, polynomial.RpMul(axial, axial))
	four, ok := proofbound.RatOf(4 * f.Major * f.Major)
	if !ok {
		return 0, false, nil
	}
	poly := polynomial.RpTrim(polynomial.RpSub(sq, polynomial.RpScale(perp, four)))
	if polynomial.RpDeg(poly) < 1 {
		return 0, false, nil
	}
	sf := polynomial.RpSquareFree(poly)
	if polynomial.RpDeg(sf) != polynomial.RpDeg(poly) {
		// A repeated root is a tangency somewhere on the line: ambiguous.
		return 0, false, nil
	}
	chain, err := polynomial.SturmChainIntContext(ctx, sf)
	if err != nil {
		return 0, false, err
	}
	n := 0
	ivs, err := polynomial.RpIsolateRootsContext(ctx, sf, chain)
	if err != nil {
		return 0, false, err
	}
	for _, iv := range ivs {
		iv, err = polynomial.RpRefineRootContext(ctx, chain, iv, func(lo, hi float64) bool { return hi-lo <= 1e-11*math.Max(1, math.Abs(lo)) })
		if err != nil {
			return 0, false, err
		}
		tLo, _ := iv.Lo.Float64()
		tHi, _ := iv.Hi.Float64()
		if tHi <= -tol {
			continue // behind the start
		}
		if tLo <= tol {
			// At or straddling the start: a crossing there can only be the
			// carrier passing through the witness — cleanly off the trimmed
			// face means no crossing, anything else is ambiguous.
			q0 := p.Add(dir.Scale((tLo + tHi) / 2))
			if tHi-tLo <= tol && f.AdmitPoint(q0, tol+(tHi-tLo)) == -1 {
				continue
			}
			return 0, false, nil
		}
		q := p.Add(dir.Scale((tLo + tHi) / 2))
		switch f.AdmitPoint(q, tol+(tHi-tLo)) {
		case 1:
			n++
		case -1:
		default:
			return 0, false, nil
		}
	}
	return n, true, nil
}
