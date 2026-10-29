package revolveplan

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CompleteRingRecord holds exact carrier data for a proven full ring.
// Center includes payload placement; axis and radii remain unchanged by translation.
type CompleteRingRecord struct {
	Center       proof.DyV3
	Axis         r3.Vec
	Major, Minor proof.Dyadic
}

// CompleteRing proves a complete regular ring from this current Revolve record.
func (rp Payload) CompleteRing(ctx context.Context) (CompleteRingRecord, bool, error) {
	if err := ctx.Err(); err != nil {
		return CompleteRingRecord{}, false, err
	}
	if !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || rp.SectionWhole ||
		!rp.Xform.IsValid() || !proofbound.FiniteVec(rp.Xform.Translation()) ||
		rp.Xform.Basis() != r3.Identity().Basis() ||
		rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 || rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 ||
		rp.Ax.RadialAdmitAllow != 0 || rp.Ax.Snap != (revolveaxis.SnapAllow{}) {
		return CompleteRingRecord{}, false, nil
	}
	lo, hi := rp.Den.Phi0, rp.Den.Phi1
	if lo.Span != nil || hi.Span != nil || lo.Rad == nil || hi.Rad == nil || lo.Turn == nil || hi.Turn == nil || lo.Rad.Cmp(hi.Rad) != 0 || new(big.Rat).Sub(hi.Turn, lo.Turn).Cmp(big.NewRat(1, 1)) != 0 {
		return CompleteRingRecord{}, false, nil
	}
	if proof.FloatRat(rp.Ax.SnapTol) == nil || rp.Ax.SnapTol < 0 {
		return CompleteRingRecord{}, false, nil
	}
	if len(rp.Profile.Holes) != 0 || len(rp.Profile.Outer.Segments) != 1 {
		return CompleteRingRecord{}, false, nil
	}
	c, ok := rp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || c.TStart != 0 || c.TEnd != 1 {
		return CompleteRingRecord{}, false, nil
	}
	values := []float64{c.Center.U, c.Center.V, rp.Ax.AU, rp.Ax.AV, rp.Ax.DU, rp.Ax.DV}
	d := make([]proof.Dyadic, len(values))
	for i, v := range values {
		var valid bool
		d[i], valid = proof.DyOf(v)
		if !valid {
			return CompleteRingRecord{}, false, nil
		}
	}
	radius := extent.ExactConversion(c.Radius, units.Millimeter)
	if radius == nil {
		return CompleteRingRecord{}, false, nil
	}
	minor, ok := proof.DyOfRat(radius)
	if !ok || minor.Sign() <= 0 {
		return CompleteRingRecord{}, false, nil
	}
	du, dv := proof.DySubScalar(d[0], d[2]), proof.DySubScalar(d[1], d[3])
	z := proof.DyAdd(proof.DyMul(du, d[4]), proof.DyMul(dv, d[5]))
	rho := proof.DySubScalar(proof.DyMul(dv, d[4]), proof.DyMul(du, d[5]))
	if proof.DyCmp(rho, minor) <= 0 {
		return CompleteRingRecord{}, false, nil
	}
	resolved, err := revolveaxis.ResolveLoopWithFreeform(ctx, rp.Profile.Outer, freeform.NewFreeformWork(), "closed ring", rp.ChargedWalk, rp.Ax.SnapTol, rp.Ax.RadialLower)
	if err != nil {
		if ctx.Err() != nil {
			return CompleteRingRecord{}, false, ctx.Err()
		}
		return CompleteRingRecord{}, false, nil
	}
	if !resolved.SingleClosed || len(resolved.Walks) != 1 || len(resolved.Kinds) != 1 || resolved.Kinds[0] != revolveaxis.WallTorus {
		return CompleteRingRecord{}, false, nil
	}
	walk := resolved.Walks[0]
	wz, zok := proof.DyOf(walk.CU)
	wr, rok := proof.DyOf(walk.CV)
	wm, mok := proof.DyOf(walk.Radius)
	if !zok || !rok || !mok || proof.DyCmp(z, wz) != 0 || proof.DyCmp(rho, wr) != 0 || proof.DyCmp(minor, wm) != 0 {
		return CompleteRingRecord{}, false, nil
	}
	if !proofbound.FiniteVec(rp.Frame.Origin()) || !proofbound.FiniteVec(rp.Frame.U()) || !proofbound.FiniteVec(rp.Frame.V()) || !closedCardinalBasis(rp.Frame.U(), rp.Frame.V(), rp.Frame.N()) {
		return CompleteRingRecord{}, false, nil
	}
	anchor := proof.DvAdd(proof.DyVec(rp.Frame.Origin()), proof.DvAdd(proof.DvScale(proof.DyVec(rp.Frame.U()), d[2]), proof.DvScale(proof.DyVec(rp.Frame.V()), d[3])))
	axis := proof.DvAdd(proof.DvScale(proof.DyVec(rp.Frame.U()), d[4]), proof.DvScale(proof.DyVec(rp.Frame.V()), d[5]))
	basis := rp.Basis()
	for i, v := range []float64{basis.A3.X, basis.A3.Y, basis.A3.Z} {
		x, ok := proof.DyOf(v)
		if !ok || proof.DyCmp(anchor[i], x) != 0 {
			return CompleteRingRecord{}, false, nil
		}
	}
	for i, v := range []float64{basis.W.X, basis.W.Y, basis.W.Z} {
		x, ok := proof.DyOf(v)
		if !ok || proof.DyCmp(axis[i], x) != 0 {
			return CompleteRingRecord{}, false, nil
		}
	}
	w := basis.W
	if w != r3.NewVec(1, 0, 0) && w != r3.NewVec(-1, 0, 0) && w != r3.NewVec(0, 1, 0) && w != r3.NewVec(0, -1, 0) && w != r3.NewVec(0, 0, 1) && w != r3.NewVec(0, 0, -1) {
		return CompleteRingRecord{}, false, nil
	}
	center := proof.DvAdd(anchor, proof.DvScale(axis, z))
	if !rp.ProvesSimple(ctx) {
		return CompleteRingRecord{}, false, ctx.Err()
	}
	return CompleteRingRecord{Center: proof.DvTransform(rp.Xform, center), Axis: w, Major: rho, Minor: minor}, true, nil
}

func closedCardinalBasis(u, v, n r3.Vec) bool {
	cardinal := func(p r3.Vec) bool {
		return p == r3.NewVec(1, 0, 0) || p == r3.NewVec(-1, 0, 0) || p == r3.NewVec(0, 1, 0) || p == r3.NewVec(0, -1, 0) || p == r3.NewVec(0, 0, 1) || p == r3.NewVec(0, 0, -1)
	}
	return cardinal(u) && cardinal(v) && cardinal(n) && u.Cross(v) == n
}
