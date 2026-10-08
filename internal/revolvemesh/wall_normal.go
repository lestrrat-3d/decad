package revolvemesh

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
)

// StraightWallNormal encloses the surface a straight revolve wall denotes
// (surfacenormal.Revolved): one recorded segment per entry of ends, each its
// start and end point with the walk's own proven bounds, swept about the axis
// the record names. The axis frame, the frame lift and the placement are
// MeridianGap's leaves: the frame's origin, U and V and the placement's basis
// and translation are exact, and the axis anchor and direction are widened by
// ab. Each run is the segment's own (dz, dρ) in that axis frame, re-expressed
// over the widened leaves, so neither the float re-expression nor the
// classifier's own reading of a near-parallel or near-perpendicular segment
// stands between the run and the record.
func (l RevolveLift) StraightWallNormal(ab AxisBound, xform r3.Transform, ends [][2]RecordedMeridian) surfacenormal.Revolved {
	out, ax, ok := l.wallAxis(ab, xform)
	if !ok {
		return surfacenormal.Revolved{}
	}
	for _, seg := range ends {
		var pts [2][2]proofbound.RatInterval
		for i, rec := range seg {
			u, okU := widenLeaf(rec.U, rec.UV.U)
			v, okV := widenLeaf(rec.V, rec.UV.V)
			if !okU || !okV {
				return surfacenormal.Revolved{}
			}
			pts[i] = [2]proofbound.RatInterval{u, v}
		}
		du := proofbound.IntervalSub(pts[1][0], pts[0][0])
		dv := proofbound.IntervalSub(pts[1][1], pts[0][1])
		out.Runs = append(out.Runs, ax.reexpress(du, dv))
		var ends [2][2]proofbound.RatInterval
		for i, pt := range pts {
			ends[i] = ax.reexpress(proofbound.IntervalSub(pt[0], ax.aU), proofbound.IntervalSub(pt[1], ax.aV))
		}
		out.Ends = append(out.Ends, ends)
	}
	out.Valid = len(out.Runs) > 0
	return out
}

// CircularWallNormal is StraightWallNormal's circular twin: the recorded
// circle's centre, re-expressed into the same axis frame, is all a circular
// wall's normal reads. Its radius, R widened by RBound, rides beside it. A
// coalesced wall holds one recorded circle per segment it covers; the
// enclosure is the hull of their centres and of their radii, so it holds
// every one of them.
func (l RevolveLift) CircularWallNormal(ab AxisBound, xform r3.Transform, circles []RecordedMeridian) surfacenormal.Revolved {
	out, ax, ok := l.wallAxis(ab, xform)
	if !ok || len(circles) == 0 {
		return surfacenormal.Revolved{}
	}
	for i, c := range circles {
		u, okU := widenLeaf(c.U, c.UV.U)
		v, okV := widenLeaf(c.V, c.UV.V)
		r, okR := widenLeaf(c.R, c.RBound)
		if !okU || !okV || !okR {
			return surfacenormal.Revolved{}
		}
		centre := ax.reexpress(proofbound.IntervalSub(u, ax.aU), proofbound.IntervalSub(v, ax.aV))
		if i == 0 {
			out.Centre, out.Radius = centre, r
			continue
		}
		out.Centre = [2]proofbound.RatInterval{intervalHull(out.Centre[0], centre[0]), intervalHull(out.Centre[1], centre[1])}
		out.Radius = intervalHull(out.Radius, r)
	}
	out.Circular = true
	out.Valid = true
	return out
}

// intervalHull is the smallest interval holding both a and b.
func intervalHull(a, b proofbound.RatInterval) proofbound.RatInterval {
	lo, hi := a.Lo, a.Hi
	if b.Lo.Cmp(lo) < 0 {
		lo = b.Lo
	}
	if b.Hi.Cmp(hi) > 0 {
		hi = b.Hi
	}
	return proofbound.Interval(lo, hi)
}

// CapNormal encloses the planar cap a partial sweep ends in, at the angle
// whose sine and cosine sin and cos enclose: the plane through the axis the
// record names and the radial direction at that angle, under the same axis
// frame, frame lift and placement leaves as StraightWallNormal. Its outward
// normal is W × radial(φ) at the end cap and the negation at the start cap,
// which is (−sin φ, cos φ) on (E0, E1) or its negation. Neither the float
// sine and cosine of the held angle, the float axis direction nor any
// rounding of the frame the build placed stands between it and the record.
func (l RevolveLift) CapNormal(ab AxisBound, xform r3.Transform, sin, cos proofbound.RatInterval, start bool) surfacenormal.Revolved {
	out, _, ok := l.wallAxis(ab, xform)
	if !ok {
		return surfacenormal.Revolved{}
	}
	dir := [2]proofbound.RatInterval{proofbound.IntervalNeg(sin), cos}
	if start {
		dir = [2]proofbound.RatInterval{sin, proofbound.IntervalNeg(cos)}
	}
	out.Cap = true
	out.CapDir = dir
	out.Valid = true
	return out
}

// wallAxisLeaves are the widened axis leaves a wall's runs and centre are
// re-expressed over.
type wallAxisLeaves struct {
	aU, aV, dU, dV proofbound.RatInterval
}

// reexpress carries a plane-local offset into the axis frame: z = Δ·d and
// ρ = d × Δ, axisFrame's own convention.
func (ax wallAxisLeaves) reexpress(du, dv proofbound.RatInterval) [2]proofbound.RatInterval {
	z := proofbound.IntervalAdd(proofbound.IntervalMul(du, ax.dU), proofbound.IntervalMul(dv, ax.dV))
	rho := proofbound.IntervalSub(proofbound.IntervalMul(dv, ax.dU), proofbound.IntervalMul(du, ax.dV))
	return [2]proofbound.RatInterval{z, rho}
}

// wallAxis builds the placed axis frame every wall of l shares: the origin
// A3 = O + U·aU + V·aV and the basis E0 = −U·dV + V·dU, E1 = W × E0 and
// W = U·dU + V·dV, each carried through xform exactly.
func (l RevolveLift) wallAxis(ab AxisBound, xform r3.Transform) (surfacenormal.Revolved, wallAxisLeaves, bool) {
	vecs := [...]r3.Vec{l.Frame.Origin(), l.Frame.U(), l.Frame.V()}
	var iv [len(vecs)]proofbound.IvVec3
	for i, w := range vecs {
		enc, ok := proofbound.IvVec3Of(w)
		if !ok {
			return surfacenormal.Revolved{}, wallAxisLeaves{}, false
		}
		iv[i] = enc
	}
	o, fu, fv := iv[0], iv[1], iv[2]
	var leaves [4]proofbound.RatInterval
	for i, pair := range [...][2]float64{{l.AU, ab.AU}, {l.AV, ab.AV}, {l.DU, ab.DU}, {l.DV, ab.DV}} {
		enc, ok := widenLeaf(pair[0], pair[1])
		if !ok {
			return surfacenormal.Revolved{}, wallAxisLeaves{}, false
		}
		leaves[i] = enc
	}
	ax := wallAxisLeaves{aU: leaves[0], aV: leaves[1], dU: leaves[2], dV: leaves[3]}
	a3 := proofbound.IvVec3Add(o, proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, ax.aU), proofbound.IvVec3Mul(fv, ax.aV)))
	w := proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, ax.dU), proofbound.IvVec3Mul(fv, ax.dV))
	e0 := proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, proofbound.IntervalNeg(ax.dV)), proofbound.IvVec3Mul(fv, ax.dU))
	unplaced := surfacenormal.Revolved{Origin: a3, Basis: [3]proofbound.IvVec3{e0, proofbound.IvVec3Cross(w, e0), w}, Valid: true}
	placed := unplaced.Transformed(xform)
	if !placed.Valid {
		return surfacenormal.Revolved{}, wallAxisLeaves{}, false
	}
	placed.Valid = false
	return placed, ax, true
}

// widenLeaf encloses the held float x widened by its own bound w.
func widenLeaf(x, w float64) (proofbound.RatInterval, bool) {
	r, b := proofarith.FloatRat(x), proofarith.FloatRat(w)
	if r == nil || b == nil {
		return proofbound.RatInterval{}, false
	}
	if b.Sign() < 0 {
		b.Neg(b)
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(r), b), true
}
