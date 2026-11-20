package rimextension

import (
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Prepared carries the bounded held mesh and the original facet identities
// needed to exclude unaffected pairs during the final embedding check.
type Prepared struct {
	Payload  facetedrecord.Payload
	Min, Max r3.Vec
	Previous map[[3]int]bool
}

// Prepare rewrites and bounds the new facets after native window admission.
func Prepare(ctx context.Context, env topo.Env, a Admission, c Candidate, w WindowBounds, m *proofbound.ReplayBudget) (Prepared, bool, error) {
	old, ok := a.Stock.Payload.(facetedrecord.Payload)
	if !ok {
		return Prepared{}, false, nil
	}
	out, ok, err := Apply(ctx, Mesh{Vertices: old.Verts, Triangles: old.Tris, Sources: old.Src}, c.Section, c.Groups, c.StopFacets, w.support, m)
	if !ok || err != nil {
		return Prepared{}, false, err
	}
	radial, ok, err := ExtensionBound(ctx, a, out, c.Groups.Extension, m)
	if !ok || err != nil {
		return Prepared{}, false, err
	}
	setback := a.Setback
	if err := m.ReserveBundle(ctx, uint64(12*(len(old.Tris)+len(out.Triangles))), uint64(4*(len(old.Tris)+len(out.Triangles))), uint64(256*(len(old.Tris)+len(out.Triangles)))); err != nil {
		return Prepared{}, false, err
	}
	p := facetedrecord.Payload{
		Verts: out.Vertices, Tris: out.Triangles, Src: out.Sources, Groups: slices.Clone(old.Groups),
		VertexBound: slices.Clone(old.VertexBound), MeshBound: old.MeshBound, Xform: r3.Identity(),
	}
	for i := range 2 {
		p.Groups = append(p.Groups, facetedrecord.Group{Origins: []provenance.Ref{{Producer: env.Ref, Role: "chamfer(0)"}}, Source: facetedrecord.BlendSource(env.Ref, i)})
	}
	for i := range p.Groups {
		p.Groups[i].HasCentroid = false
	}
	p.VertexBound = append(p.VertexBound, make([]float64, len(out.Vertices)-len(old.Verts))...)
	previous, current := map[[3]int]bool{}, map[[3]int]bool{}
	for _, t := range old.Tris {
		previous[t] = true
	}
	for _, t := range p.Tris {
		current[t] = true
	}
	inf := math.Inf(1)
	lo, hi := r3.NewVec(inf, inf, inf), r3.NewVec(-inf, -inf, -inf)
	area := w.Area
	visit := func(t [3]int) {
		for _, i := range t {
			v := p.Verts[i]
			lo = r3.NewVec(min(lo.X, v.X), min(lo.Y, v.Y), min(lo.Z, v.Z))
			hi = r3.NewVec(max(hi.X, v.X), max(hi.Y, v.Y), max(hi.Z, v.Z))
		}
		x, y, z := p.Verts[t[0]], p.Verts[t[1]], p.Verts[t[2]]
		area = proofbound.AbsSumUpper(area, proofbound.ProductUpper(proofbound.DvLenUpper(proofbound.HeldDelta(y, x)), proofbound.DvLenUpper(proofbound.HeldDelta(z, x)))/2)
	}
	for _, t := range old.Tris {
		if !current[t] {
			visit(t)
		}
	}
	for _, t := range p.Tris {
		if !previous[t] {
			visit(t)
		}
	}
	span := func(l, h float64) float64 { return math.Nextafter(h-l, math.Inf(1)) }
	// Each new coordinate is the nearest binary64 rounding of its rational
	// chart point. Three whole ulps bound the Euclidean rounding displacement.
	maxCoord := max(math.Abs(lo.X), math.Abs(lo.Y), math.Abs(lo.Z), math.Abs(hi.X), math.Abs(hi.Y), math.Abs(hi.Z))
	rounding := proofbound.ProductUpper(3, math.Nextafter(maxCoord, inf)-maxCoord)
	bound := proofbound.AbsSumUpper(old.MeshBound, radial, w.Deviation, rounding)
	for _, t := range p.Tris {
		if !previous[t] {
			for _, j := range t {
				p.VertexBound[j] = max(p.VertexBound[j], bound)
			}
		}
	}
	p.MeshBound = max(p.MeshBound, bound)
	p.DPair = proofbound.AbsSumUpper(old.DPair, 2*p.MeshBound, 2*setback)
	p.VolSymDiff = proofbound.AbsSumUpper(old.VolSymDiff, w.Volume, proofbound.ProductUpper(span(lo.X, hi.X), proofbound.ProductUpper(span(lo.Y, hi.Y), span(lo.Z, hi.Z))))
	p.AreaSlack = proofbound.AbsSumUpper(old.AreaSlack, area)
	return Prepared{p, lo, hi, previous}, true, nil
}
