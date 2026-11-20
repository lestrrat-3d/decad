package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/rimextension"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/r3"
)

type nativeAnnulus struct {
	center                rimPoint
	radius, d             float64
	top, hinge            *big.Rat
	cap, wall             *topo.Face
	capSource, wallSource rimextension.BoundFace
}

func alignedNativeRim(ctx context.Context, d *Document, b *Body, e *topo.Edge, setback float64) (brepmodify.Result, bool, error) {
	var none brepmodify.Result
	if e == nil || !e.Convex || e.Start == nil || e.Start != e.End || len(e.Faces) != 2 || !e.RetainedCurve {
		return none, false, nil
	}
	if _, ok := e.Curve.(surfacegeom.Circle3); !ok {
		return none, false, nil
	}
	if _, suppressed := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture); suppressed {
		return none, false, nil
	}
	a := nativeAnnulus{d: setback}
	for _, f := range e.Faces {
		if !f.RetainedTag || f.Reversed || f.TagDeparture != 0 || f.AxialDelta != 0 || f.ExtentDelta != 0 {
			return none, false, nil
		}
		switch s := f.Surface.(type) {
		case surfacegeom.Plane:
			if s.Frame.N() != (r3.Vec{Y: 1}) {
				return none, false, nil
			}
			a.cap = f
		case surfacegeom.Cylinder:
			if math.Abs(s.Axis.Y) != 1 || s.Axis.X != 0 || s.Axis.Z != 0 {
				return none, false, nil
			}
			a.wall = f
		default:
			return none, false, nil
		}
	}
	if a.cap == nil || a.wall == nil {
		return none, false, nil
	}
	m := proofbound.NewStrictReplayBudget()
	h, ok, err := rimHistorySnapshot(ctx, d, m)
	if !ok || err != nil {
		return none, false, err
	}
	reader := rimFaceReader{pairedHistorySource{h}, m, map[*topo.Body][]*topo.Face{}}
	a.capSource, ok, err = reader.bind(ctx, b, a.cap, false)
	if !ok || err != nil {
		return none, false, err
	}
	a.wallSource, ok, err = reader.bind(ctx, b, a.wall, false)
	if !ok || err != nil {
		return none, false, err
	}
	pad := a.capSource.Body
	if pad != a.wallSource.Body || a.capSource.Translation != (r3.Vec{}) || a.wallSource.Translation != (r3.Vec{}) {
		return none, false, nil
	}
	p, ok := pad.Payload.(prismrecord.Payload)
	if !ok || p.SurfaceResult || p.Xform != r3.Identity() || p.SectionDelta != 0 || p.Frame.U() != (r3.Vec{X: 1}) || p.Frame.V() != (r3.Vec{Z: 1}) || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) != 1 {
		return none, false, nil
	}
	sealed, err := pad.ReplayPrismBuildPayload(ctx, p.Frame, p.Xform, p.Z0, p.Z1, p.Z0Delta, p.Z1Delta, p.SectionDelta, p.Z0Denoted, p.Z1Denoted, []sectionrecord.AnalyticProfileView{p.Profile.AnalyticSealView()}, m)
	if !sealed || err != nil {
		return none, false, err
	}
	seg, ok := p.Profile.Outer.Segments[0].(momentinput.CircleSeg)
	if !ok || seg.Center != (sectionrecord.Point2{}) || !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 {
		return none, false, nil
	}
	if len(a.capSource.Native.Loops) != 1 || len(a.capSource.Native.Loops[0].CoEdges) != 1 {
		return none, false, nil
	}
	edge := a.capSource.Native.Loops[0].CoEdges[0].Edge
	if edge == nil || edge.Start != edge.End || len(edge.Faces) != 2 || (edge.Faces[0] != a.wallSource.Native && edge.Faces[1] != a.wallSource.Native) {
		return none, false, nil
	}
	a.radius = seg.Radius.Base()
	a.center = rimVec(p.Frame.Origin())
	a.top = upperRimLevel(a.capSource.Native.CapRecord)
	if a.top == nil || !(setback > 0 && setback < a.radius) {
		return none, false, nil
	}
	a.hinge = rimSub(a.top, rimRat(setback))
	lo, hi := p.DenotedZ0(), p.DenotedZ1()
	if lo == nil || hi == nil {
		return none, false, nil
	}
	height := new(big.Rat).Abs(rimSub(hi, lo))
	if height.Cmp(rimRat(setback)) <= 0 {
		return none, false, nil
	}
	cheek, faces, ok, err := a.cover(ctx, reader, b)
	if !ok || err != nil {
		return none, false, err
	}
	result, ok, err := directUpperRimAligned(ctx, envOf(b.n.Doc, d.nextProducerID()), &b.n, a.cap, a.wall, a.center, a.hinge, a.radius, setback, proofbound.RatFloatUp(height), cheek, faces, m)
	if !ok || err != nil {
		return result, ok, err
	}
	cone := upperRimCone(result.Body)
	if cone != nil {
		target := bodyHandle(result.Body)
		target.upperRimProof = &upperRimProof{
			stock: b, result: target, pad: pad, wall: a.wallSource.Native, sourceWall: a.wall, cone: cone,
			geometry: rimextension.OuterAdmission{
				CenterX: a.center[0], CenterZ: a.center[2], Hinge: a.hinge,
				PadBottom: rimSub(a.top, height), PadTop: a.top, Radius: rimRat(a.radius), NoseSetback: rimRat(setback),
			},
		}
	}
	return result, true, nil
}

// Each facet radius pairs its held triangle with a subset of its denoted
// source face in both directions. Above the cheek's top only the cap and
// wall can therefore meet this annulus. Their strict witnesses fix material
// orientation on the two connected regions inside and outside the cylinder.
func (a nativeAnnulus) cover(ctx context.Context, r rimFaceReader, b *Body) (*topo.Body, map[*topo.Face]*topo.Face, bool, error) {
	p, ok := b.n.Payload.(facetedrecord.Payload)
	if !ok || !b.n.Solid || p.Xform != r3.Identity() || !p.BooleanSimilaritySource || len(p.Tris) > 65536 || len(p.Tris) != len(p.FaceOf) || len(p.VertexBound) != len(p.Verts) {
		return nil, nil, false, nil
	}
	fs, err := r.faceList(ctx, &b.n)
	if err != nil {
		return nil, nil, false, err
	}
	if !facetedrecord.CertifiedFacetedSimilaritySource(p, len(fs)) {
		return nil, nil, false, nil
	}
	if err := r.meter.ReserveBundle(ctx, uint64(32*len(p.Tris)), uint64(len(fs)), uint64(96*len(fs))); err != nil {
		return nil, nil, false, err
	}
	levels := map[*topo.Face]*topo.Face{}
	var cheek *topo.Body
	var capWitness, wallWitness bool
	radius := rimRat(a.radius)
	inner := rimSub(radius, rimRat(a.d))
	support := rimMul(radius, big.NewRat(9, 8))
	for i, t := range p.Tris {
		if i%128 == 0 && ctx.Err() != nil {
			return nil, nil, false, ctx.Err()
		}
		if p.FaceOf[i] < 0 || p.FaceOf[i] >= len(fs) {
			return nil, nil, false, nil
		}
		f := fs[p.FaceOf[i]]
		var points [3]r3.Vec
		var beta float64
		for j, id := range t {
			if id < 0 || id >= len(p.Verts) || !proofbound.FiniteVec(p.Verts[id]) || proofbound.IsNonFinite(p.VertexBound[id]) || p.VertexBound[id] < 0 {
				return nil, nil, false, nil
			}
			points[j] = p.Verts[id]
			beta = max(beta, p.VertexBound[id])
		}
		v, w, z := points[0], points[1], points[2]
		minY := min(v.Y, w.Y, z.Y)
		maxY := max(v.Y, w.Y, z.Y)
		if math.Nextafter(maxY+beta, math.Inf(1)) < proofbound.RatFloatDown(a.hinge) {
			continue
		}
		if err := r.meter.ReserveBundle(ctx, 164, 1, 6144); err != nil {
			return nil, nil, false, err
		}
		// The radial map tapers to the identity at 9R/8. Exclude foreign
		// material in its full support above the heel, including a central hole.
		nearSquared := func(l, h float64, c *big.Rat) *big.Rat {
			near := new(big.Rat)
			if m := rimSub(rimRat(l), c); m.Sign() > 0 {
				near = m
			} else if n := rimSub(rimRat(h), c); n.Sign() < 0 {
				near = n
			}
			return rimMul(near, near)
		}
		x0 := nearSquared(min(v.X, w.X, z.X), max(v.X, w.X, z.X), a.center[0])
		z0 := nearSquared(min(v.Z, w.Z, z.Z), max(v.Z, w.Z, z.Z), a.center[2])
		be := rimRat(beta)
		outer := rimAdd(support, be)
		if rimAdd(x0, z0).Cmp(rimMul(outer, outer)) > 0 {
			continue
		}
		if f == a.wall {
			if rimSub(a.top, a.hinge).Cmp(rimMul(rimRat(2), be)) > 0 &&
				rimRat(maxY).Cmp(rimAdd(a.hinge, be)) > 0 && rimRat(minY).Cmp(rimSub(a.top, be)) < 0 {
				wallWitness = true
			}
			continue
		}
		if f == a.cap {
			lo, hi := rimAdd(inner, be), rimSub(radius, be)
			if lo.Cmp(hi) >= 0 {
				continue
			}
			lo2, hi2 := rimMul(lo, lo), rimMul(hi, hi)
			belowHigh, aboveLow := false, false
			for j, v := range points {
				x, z := rimSub(rimRat(v.X), a.center[0]), rimSub(rimRat(v.Z), a.center[2])
				next := points[(j+1)%3]
				mx := rimMul(rimRat(.5), rimAdd(x, rimSub(rimRat(next.X), a.center[0])))
				mz := rimMul(rimRat(.5), rimAdd(z, rimSub(rimRat(next.Z), a.center[2])))
				for _, q := range []*big.Rat{rimAdd(rimMul(x, x), rimMul(z, z)), rimAdd(rimMul(mx, mx), rimMul(mz, mz))} {
					belowHigh = belowHigh || q.Cmp(hi2) < 0
					aboveLow = aboveLow || q.Cmp(lo2) > 0
				}
			}
			capWitness = capWitness || belowHigh && aboveLow
			continue
		}
		if _, seen := levels[f]; seen {
			continue
		}
		src, ok, err := r.bind(ctx, b, f, false)
		if !ok || err != nil {
			return nil, nil, false, err
		}
		if src.Translation != (r3.Vec{}) || cheek != nil && src.Body != cheek {
			return nil, nil, false, nil
		}
		pp, ok := src.Body.Payload.(prismrecord.Payload)
		if !ok || pp.Xform != r3.Identity() || pp.Frame.N() != (r3.Vec{Y: -1}) || pp.SectionDelta != 0 {
			return nil, nil, false, nil
		}
		lo, hi := pp.DenotedZ0(), pp.DenotedZ1()
		if lo == nil || hi == nil {
			return nil, nil, false, nil
		}
		origin := rimRat(pp.Frame.Origin().Y)
		if rimSub(origin, lo).Cmp(a.hinge) > 0 || rimSub(origin, hi).Cmp(a.hinge) > 0 {
			return nil, nil, false, nil
		}
		if cheek == nil {
			sealed, err := src.Body.ReplayPrismBuildPayload(ctx, pp.Frame, pp.Xform, pp.Z0, pp.Z1, pp.Z0Delta, pp.Z1Delta, pp.SectionDelta, pp.Z0Denoted, pp.Z1Denoted, []sectionrecord.AnalyticProfileView{pp.Profile.AnalyticSealView()}, r.meter)
			if !sealed || err != nil {
				return nil, nil, false, err
			}
		}
		cheek = src.Body
		levels[f] = src.Native
	}
	return cheek, levels, cheek != nil && capWitness && wallWitness, nil
}
