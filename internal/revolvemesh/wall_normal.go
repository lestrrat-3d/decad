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
	}
	out.Valid = len(out.Runs) > 0
	return out
}

// CircularWallNormal is StraightWallNormal's circular twin: the recorded
// circle's centre, re-expressed into the same axis frame, is all a circular
// wall's normal reads.
func (l RevolveLift) CircularWallNormal(ab AxisBound, xform r3.Transform, centre RecordedMeridian) surfacenormal.Revolved {
	out, ax, ok := l.wallAxis(ab, xform)
	if !ok {
		return surfacenormal.Revolved{}
	}
	u, okU := widenLeaf(centre.U, centre.UV.U)
	v, okV := widenLeaf(centre.V, centre.UV.V)
	if !okU || !okV {
		return surfacenormal.Revolved{}
	}
	out.Circular = true
	out.Centre = ax.reexpress(proofbound.IntervalSub(u, ax.aU), proofbound.IntervalSub(v, ax.aV))
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
