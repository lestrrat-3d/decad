package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/rimextension"
	"github.com/lestrrat-3d/decad/internal/rimstrip"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// registeredBand checks a single current face's complete held cylindrical
// chart against its original swept arc, including its surviving trim.
func (s *cheekCutSource) registeredBand(ctx context.Context, mesh *topo.Mesh, vertexBounds []float64, f rimextension.BoundFace, n rimextension.PolarSource, low, high *big.Rat, meter *proofbound.ReplayBudget) (map[int]meshbool.CutFacetBound, bool, error) {
	if mesh == nil || low == nil || high == nil || low.Cmp(high) >= 0 || len(mesh.Source) != len(mesh.Triangles) {
		return nil, false, nil
	}
	// The held top is the nearest binary64 representation of the exact cap.
	// Register it to the true cap whether that rounding went up or down.
	nativeLow := proofbound.RatMax(n.Bottom, low)
	nativeHigh := n.Top
	capGap, ok := roundedCapGap(nativeHigh, high)
	if !ok || nativeLow.Cmp(nativeHigh) >= 0 || nativeLow.Cmp(low) != 0 {
		return nil, false, nil
	}
	// The sum of the endpoint directions lies inside this minor arc.
	// The source radius may differ at a pinned end by rounding. The explicit
	// half-plane and endpoint checks below establish the chosen chart anyway.
	x := new(big.Rat).Add(n.Rays[0][0], n.Rays[1][0])
	z := new(big.Rat).Add(n.Rays[0][1], n.Rays[1][1])
	chart := rimstrip.Polar{CenterX: n.CenterX, CenterZ: n.CenterZ, RadiusSquared: n.RadiusSquared, Bottom: low, Top: high, NormalX: x, NormalZ: z}
	if err := meter.ReserveBundle(ctx, uint64(len(mesh.Source)), uint64(len(mesh.Source)), uint64(len(mesh.Source))*8); err != nil {
		return nil, false, err
	}
	ids := make([]int, 0, len(mesh.Source))
	for i, source := range mesh.Source {
		if source == f.Final {
			ids = append(ids, i)
		}
	}
	bounds, ends, ok, err := chart.Chart(ctx, mesh.Vertices, mesh.Triangles, ids, meter)
	if !ok || err != nil {
		return nil, false, err
	}
	axial := capGap
	domain := rimstrip.PolarDomain{Polar: chart, Rays: n.Rays}
	domain.Bottom, domain.Top = nativeLow, nativeHigh
	inset := -1
	present, err := s.checkBandLimits(ctx, mesh, vertexBounds, f, n, domain, ids, meter)
	if err != nil {
		return nil, false, err
	}
	if !present {
		for end := range 2 {
			core, ok := domain.InsetEnd(end)
			if !ok {
				continue
			}
			coreNative := n
			coreNative.Rays = core.Rays
			present, err = s.checkBandLimits(ctx, mesh, vertexBounds, f, coreNative, core, ids, meter)
			if err != nil {
				return nil, false, err
			}
			if present {
				domain = core
				inset = end
				break
			}
		}
	}
	if !present {
		return nil, false, nil
	}
	delta, ok := chart.EndpointRegistration(ends, n.Rays)
	if !ok {
		return nil, false, nil
	}
	endpointDelta := proofbound.RatFloatUp(delta)
	axialDelta := proofbound.RatFloatUp(axial)
	highFloat, _ := high.Float64()
	// If the entire exterior collar is empty, an old true point cannot
	// cross that endpoint even when its error ball crosses the radial plane.
	empty := [2]bool{}
	for end := range 2 {
		if end == inset {
			continue
		}
		collar := domain.ExteriorEnd(end)
		sealed := true
		for _, i := range ids {
			t := mesh.Triangles[i]
			p := [3]r3.Vec{mesh.Vertices[t[0]], mesh.Vertices[t[1]], mesh.Vertices[t[2]]}
			if err := meter.ReserveBundle(ctx, 256, 0, 16384); err != nil {
				return nil, false, err
			}
			if !collar.InsideAngularBoundary(p, max(vertexBounds[t[0]], vertexBounds[t[1]], vertexBounds[t[2]]), end) {
				sealed = false
				break
			}
		}
		if !sealed {
			continue
		}
		foreign := n
		foreign.Rays = collar.Rays
		empty[end], err = s.nativeBandPresent(ctx, mesh, vertexBounds, f, foreign, collar, false, meter)
		if err != nil {
			return nil, false, err
		}
	}
	// Keep the original assignment on the overlap and give its facets
	// enough gate allowance for the registered inverse as well. Every old
	// true point displaced from a replaced facet lies in the native core;
	// its new inverse can land on either kind of facet.
	levels := make(map[int]meshbool.CutFacetBound, len(bounds))
	for i, b := range bounds {
		t := mesh.Triangles[i]
		tri := [3]r3.Vec{mesh.Vertices[t[0]], mesh.Vertices[t[1]], mesh.Vertices[t[2]]}
		old := max(vertexBounds[t[0]], vertexBounds[t[1]], vertexBounds[t[2]])
		level := proofbound.AbsSumUpper(b, endpointDelta, axialDelta, n.EndClosure)
		preserve := max(tri[0].Y, tri[1].Y, tri[2].Y) > highFloat
		if err := meter.ReserveBundle(ctx, 512, 1, 32768); err != nil {
			return nil, false, err
		}
		for end := range 2 {
			if !empty[end] && !domain.InsideAngularBoundary(tri, max(old, level), end) {
				preserve = true
				break
			}
		}
		levels[i] = meshbool.CutFacetBound{Distance: level, Preserve: preserve}
	}
	return levels, true, nil
}

// A second open band above the exact cap contains no terminal face and
// reaches beyond the old bound of every held facet on this cylinder. The
// cylinder cannot extend into that band without either reaching its empty
// upper end or meeting another current face inside it.
func (s *cheekCutSource) checkBandLimits(ctx context.Context, mesh *topo.Mesh, vs []float64, owner rimextension.BoundFace, n rimextension.PolarSource, g rimstrip.PolarDomain, ids []int, m *proofbound.ReplayBudget) (bool, error) {
	present, err := s.nativeBandPresent(ctx, mesh, vs, owner, n, g, true, m)
	if !present || err != nil {
		return false, err
	}
	upper := math.Inf(-1)
	for _, i := range ids {
		t := mesh.Triangles[i]
		b := max(vs[t[0]], vs[t[1]], vs[t[2]])
		upper = max(upper, math.Nextafter(max(mesh.Vertices[t[0]].Y, mesh.Vertices[t[1]].Y, mesh.Vertices[t[2]].Y)+b, math.Inf(1)))
	}
	upper = math.Nextafter(upper, math.Inf(1))
	if proofbound.IsNonFinite(upper) {
		return false, nil
	}
	above := g
	above.Bottom = g.Top
	above.Top = new(big.Rat).SetFloat64(upper)
	if above.Top.Cmp(above.Bottom) <= 0 {
		return true, nil
	}
	return s.nativeBandPresent(ctx, mesh, vs, owner, n, above, false, m)
}

// The interior of a minor-arc prism side is a connected chart. A true trim
// boundary inside it would belong to a different face of the current shell.
// Exclude all such faces, then use a same-face witness to select the occupied
// component. Other unchanged faces of this same simple native prism only
// meet the open side at its native boundary.
func (s *cheekCutSource) nativeBandPresent(ctx context.Context, mesh *topo.Mesh, vs []float64, owner rimextension.BoundFace, n rimextension.PolarSource, g rimstrip.PolarDomain, needWitness bool, m *proofbound.ReplayBudget) (bool, error) {
	allowed := map[*topo.Face]bool{}
	witness := !needWitness
	for i, t := range mesh.Triangles {
		if err := m.SpendWork(ctx, 8); err != nil {
			return false, err
		}
		f := mesh.Source[i]
		if allowed[f] || f == owner.Final && witness {
			continue
		}
		p := [3]r3.Vec{mesh.Vertices[t[0]], mesh.Vertices[t[1]], mesh.Vertices[t[2]]}
		b := max(vs[t[0]], vs[t[1]], vs[t[2]])
		if f == owner.Final {
			if err := m.ReserveBundle(ctx, 128, 0, 8192); err != nil {
				return false, err
			}
			witness = g.Witness(p, b)
			continue
		}
		away, err := g.Away(ctx, p, b, m)
		if err != nil {
			return false, err
		}
		if away {
			continue
		}
		bound, ok, err := s.reader.bindSource(ctx, s.target, f, false, true)
		if err != nil {
			return false, err
		}
		if !ok || bound.Body != owner.Body || bound.Native == owner.Native || bound.Translation != (r3.Vec{}) || bound.Final.Reversed != bound.Native.Reversed {
			return false, nil
		}
		off, err := rimextension.PolarCarrierAway(ctx, bound, n, g.Bottom, g.Top, m)
		if !off || err != nil {
			return false, err
		}
		allowed[f] = true
	}
	return witness, nil
}

func roundedCapGap(exact, held *big.Rat) (*big.Rat, bool) {
	level, _ := exact.Float64()
	r := new(big.Rat).SetFloat64(level)
	if r == nil || r.Cmp(held) != 0 {
		return nil, false
	}
	return new(big.Rat).Abs(new(big.Rat).Sub(exact, held)), true
}
