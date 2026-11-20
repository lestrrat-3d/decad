package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// On an aligned join, the native cheek top and the chamfer heel round to
// the same held Y. The true cheek is at or below the heel. A lower edge of
// the held cone can therefore touch a retained cheek facet at that one Y.
type alignedRimFoot struct {
	held  *big.Rat
	cheek map[*topo.Face]*topo.Face
	// The true cylinder can span the narrow interval from the actual cheek
	// top to the exact heel, including where the held cheek meets the band.
	gap, displacement float64
}

func nativeRimFoot(ctx context.Context, b, native *topo.Body, faces map[*topo.Face]*topo.Face,
	center rimPoint, hinge *big.Rat, radius float64, m *proofbound.ReplayBudget) (*alignedRimFoot, bool, error) {
	if b == nil || native == nil || m == nil || !native.Solid || len(faces) == 0 || len(faces) > 65536 ||
		proofbound.IsNonFinite(radius) || radius <= 0 {
		return nil, false, nil
	}
	p, ok := native.Payload.(prismrecord.Payload)
	if !ok || p.SurfaceResult || p.Xform != r3.Identity() || p.SectionDelta != 0 || len(p.Profile.Holes) != 0 ||
		len(p.Profile.Outer.Segments) == 0 || len(p.Profile.Outer.Segments) > 64 || p.Frame.U() != (r3.Vec{X: 1}) || p.Frame.V() != (r3.Vec{Z: 1}) {
		return nil, false, nil
	}
	for _, q := range []*big.Rat{center[0], center[1], center[2], hinge} {
		if q == nil || q.Num().BitLen() > 192 || q.Denom().BitLen() > 192 {
			return nil, false, nil
		}
	}
	n := uint64(len(p.Profile.Outer.Segments))
	if e := m.ReserveBundle(ctx, n*80+uint64(len(faces))*4, n+uint64(len(faces)), n*32768+uint64(len(faces))*64); e != nil {
		return nil, false, e
	}
	ok, e := native.ReplayPrismBuildPayload(ctx, p.Frame, p.Xform, p.Z0, p.Z1, p.Z0Delta, p.Z1Delta, p.SectionDelta, p.Z0Denoted, p.Z1Denoted,
		[]sectionrecord.AnalyticProfileView{p.Profile.AnalyticSealView()}, m)
	if !ok || e != nil {
		return nil, false, e
	}
	z0, z1 := p.DenotedZ0(), p.DenotedZ1()
	if z0 == nil || z1 == nil {
		return nil, false, nil
	}
	if z0.Cmp(z1) > 0 {
		z0 = z1
	}
	top := rimSub(rimRat(p.Frame.Origin().Y), z0)
	y, _ := hinge.Float64()
	held := rimRat(y)
	yt, _ := top.Float64()
	if top.Cmp(hinge) > 0 || y != yt {
		return nil, false, nil
	}
	r := rimRat(radius)
	r2 := rimMul(r, r)
	near := rimRat(math.Nextafter(radius, 0))
	near2 := rimMul(near, near)
	matched := false
	for _, s := range p.Profile.Outer.Segments {
		arc, ok := s.(momentinput.ArcSeg)
		if !ok {
			continue
		}
		if rimAdd(rimRat(p.Frame.Origin().X), rimRat(arc.Center.U)).Cmp(center[0]) != 0 ||
			rimAdd(rimRat(p.Frame.Origin().Z), rimRat(arc.Center.V)).Cmp(center[2]) != 0 {
			continue
		}
		inside := true
		for _, end := range []momentinput.Point2{arc.Start, arc.End} {
			x, z := rimSub(rimRat(end.U), rimRat(arc.Center.U)), rimSub(rimRat(end.V), rimRat(arc.Center.V))
			rs := rimAdd(rimMul(x, x), rimMul(z, z))
			inside = inside && rs.Cmp(near2) > 0 && rs.Cmp(r2) <= 0
		}
		if !inside {
			return nil, false, nil
		}
		matched = true
	}
	if !matched {
		return nil, false, nil
	}
	for final, src := range faces {
		if final == nil || src == nil || final.Body != b || src.Body != native {
			return nil, false, nil
		}
		ok, e = native.ReplayPrismBuiltFace(ctx, src, m)
		if !ok || e != nil {
			return nil, false, e
		}
	}
	return &alignedRimFoot{held: held, cheek: faces, gap: proofbound.RatFloatUp(rimSub(hinge, top)),
		displacement: proofbound.RatFloatUp(new(big.Rat).Abs(rimSub(held, hinge)))}, true, nil
}

// directUpperRimAligned consumes the caller's authenticated stock history.
// Each entry in cheekSources is the unshifted native counterpart of the
// final face. The caller has excluded all other material from the edit.
func directUpperRimAligned(ctx context.Context, env topo.Env, b *topo.Body, capFace, wall *topo.Face, center rimPoint,
	hinge *big.Rat, radius, d, height float64, nativeCheek *topo.Body, cheekSources map[*topo.Face]*topo.Face, budget *proofbound.ReplayBudget) (brepmodify.Result, bool, error) {
	foot, ok, e := nativeRimFoot(ctx, b, nativeCheek, cheekSources, center, hinge, radius, budget)
	if !ok || e != nil {
		return brepmodify.Result{}, false, e
	}
	return directUpperRimAt(ctx, env, b, capFace, wall, center, hinge, radius, d, height, budget, foot)
}
