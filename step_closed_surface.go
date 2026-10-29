package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/closedstep"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/facedeparture"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Root ownership is needed to authenticate the private body payload and face node.
func init() { closedstep.Reader = readClosedSurface }
func readClosedSurface(ctx context.Context, body, face any) (closedstep.Coverage, bool, error) {
	if err := ctx.Err(); err != nil {
		return closedstep.Coverage{}, false, err
	}
	b, ok := body.(*Body)
	if !ok || b == nil {
		return closedstep.Coverage{}, false, nil
	}
	f, ok := face.(*Face)
	if !ok || f == nil {
		return closedstep.Coverage{}, false, nil
	}
	faces := b.Faces()
	shells := b.Shells()
	if !b.IsSolid() || b.n.Kind != topo.BodySolid || len(b.Lumps()) != 1 || len(shells) != 1 || shells[0].IsVoid() || shells[0].IsOpen() || len(faces) != 1 || faces[0] != f || len(b.Edges()) != 0 || len(f.Loops()) != 0 {
		return closedstep.Coverage{}, false, nil
	}
	rp, ok := b.n.Payload.(revolveplan.Payload)
	if !ok || !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || rp.SectionWhole || rp.Xform != r3.Identity() || rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 || rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 || rp.Ax.RadialAdmitAllow != 0 || rp.Ax.Snap != (revolveaxis.SnapAllow{}) {
		return closedstep.Coverage{}, false, nil
	}
	if proof.FloatRat(rp.Ax.AU) == nil || proof.FloatRat(rp.Ax.AV) == nil || proof.FloatRat(rp.Ax.DU) == nil || proof.FloatRat(rp.Ax.DV) == nil || proof.FloatRat(rp.Ax.SnapTol) == nil || rp.Ax.SnapTol < 0 {
		return closedstep.Coverage{}, false, nil
	}
	lo, hi := rp.Den.Phi0, rp.Den.Phi1
	if lo.Span != nil || hi.Span != nil || lo.Rad == nil || hi.Rad == nil || lo.Turn == nil || hi.Turn == nil || lo.Rad.Cmp(hi.Rad) != 0 || new(big.Rat).Sub(hi.Turn, lo.Turn).Cmp(big.NewRat(1, 1)) != 0 {
		return closedstep.Coverage{}, false, nil
	}
	if tag, ok := f.Surface().(Torus); ok {
		if facedeparture.TagDeparts(&f.n) {
			return closedstep.Coverage{}, false, nil
		}
		rec, ok, err := rp.CompleteRing(ctx)
		if !ok || err != nil {
			return closedstep.Coverage{}, false, err
		}
		x, xok := rec.Center[0].Float64()
		y, yok := rec.Center[1].Float64()
		z, zok := rec.Center[2].Float64()
		major, mok := rec.Major.Float64()
		minor, nok := rec.Minor.Float64()
		if !xok || !yok || !zok || !mok || !nok || math.IsInf(x, 0) || math.IsInf(y, 0) || math.IsInf(z, 0) || math.IsInf(major, 0) || math.IsInf(minor, 0) {
			return closedstep.Coverage{}, false, nil
		}
		center := r3.NewVec(x, y, z)
		ma, maok := stitchflux.RatMillimetres(tag.Major)
		mi, miok := stitchflux.RatMillimetres(tag.Minor)
		if !maok || !miok || ma.Cmp(rec.Major.Rat()) != 0 || mi.Cmp(rec.Minor.Rat()) != 0 || center != tag.Center || tag.Axis != rec.Axis {
			return closedstep.Coverage{}, false, nil
		}
		ref := r3.NewVec(1, 0, 0)
		if rec.Axis.X != 0 {
			ref = r3.NewVec(0, 1, 0)
		}
		return closedstep.Coverage{Ring: true, Center: center, Axis: rec.Axis, Reference: ref, Major: major, Minor: minor}, true, nil
	}
	p, ok := contactround.SphereRecord(&b.n)
	if !ok || p.Face != &f.n || facedeparture.TagDeparts(&f.n) {
		return closedstep.Coverage{}, false, nil
	}
	x, xok := p.Center[0].Float64()
	y, yok := p.Center[1].Float64()
	z, zok := p.Center[2].Float64()
	radius, rok := p.Radius.Float64()
	if !xok || !yok || !zok || !rok || math.IsInf(x, 0) || math.IsInf(y, 0) || math.IsInf(z, 0) || math.IsInf(radius, 0) || radius <= 0 {
		return closedstep.Coverage{}, false, nil
	}
	tag, ok := f.Surface().(Sphere)
	if !ok {
		return closedstep.Coverage{}, false, nil
	}
	r, rok := stitchflux.RatMillimetres(tag.Radius)
	center := r3.NewVec(x, y, z)
	if !rok || tag.Center != center || r.Cmp(p.Radius.Rat()) != 0 {
		return closedstep.Coverage{}, false, nil
	}
	if !rp.ProvesSimple(ctx) {
		return closedstep.Coverage{}, false, ctx.Err()
	}
	return closedstep.Coverage{Center: center, Radius: radius}, true, nil
}
