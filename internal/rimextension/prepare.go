package rimextension

import (
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/topobuild"

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
	return prepareOutput(ctx, env, a, old, out, radial, w, 2, 0, m)
}

// PrepareOuter maps the held mesh into the shoulder chart and returns the
// result in world coordinates before building its error and moment bounds.
func PrepareOuter(ctx context.Context, env topo.Env, a OuterAdmission, c Candidate, w WindowBounds, m *proofbound.ReplayBudget) (Prepared, bool, error) {
	old, ok := a.Stock.Payload.(facetedrecord.Payload)
	if !ok || a.Wing != 1 && a.Wing != -1 {
		return Prepared{}, false, nil
	}
	if err := m.ReserveBundle(ctx, uint64(8*len(old.Verts)), uint64(len(old.Verts)), uint64(len(old.Verts))*24); err != nil {
		return Prepared{}, false, err
	}
	v := make([]r3.Vec, len(old.Verts))
	for i, p := range old.Verts {
		v[i] = ShoulderPoint(p, a.Wing)
	}
	c.Groups.ClosedTerminal = a.Round.Final == nil && a.Boss.Final == a.Cone
	if c.Groups.ClosedTerminal {
		if err := m.ReserveBundle(ctx, uint64(len(old.Tris)*4), uint64(len(old.Groups)), uint64(len(old.Groups)*96)); err != nil {
			return Prepared{}, false, err
		}
		// Require every facet sharing a protected group to belong to a registered disk wall.
		fs := topobuild.Faces(a.Stock)
		walls := map[*topo.Face]bool{}
		for _, f := range a.RetainedDiskWalls {
			walls[f] = true
		}
		keep := map[int]bool{}
		exclude := map[int]bool{}
		for i, j := range old.FaceOf {
			g := old.Src[i]
			if walls[fs[j]] {
				keep[g] = true
			} else {
				exclude[g] = true
			}
		}
		for g := range exclude {
			delete(keep, g)
		}
		c.Groups.RetainedDiskGroups = keep
	}
	out, ok, err := Apply(ctx, Mesh{Vertices: v, Triangles: old.Tris, Sources: old.Src}, c.Section, c.Groups, c.StopFacets, w.support, m)
	if !ok || err != nil {
		return Prepared{}, false, err
	}
	n, _, ok, err := outerNative(ctx, a, m)
	if !ok || err != nil {
		return Prepared{}, false, err
	}
	// A finite end face has its own blend identity and its own surface
	// witness. Only the longitudinal wall extension follows the cylinder.
	endFaces := 0
	for i, t := range out.Triangles {
		if !c.Groups.ClosedTerminal && out.Sources[i] == c.Groups.Extension && out.Vertices[t[0]].X == c.Section.End.X && out.Vertices[t[1]].X == c.Section.End.X && out.Vertices[t[2]].X == c.Section.End.X {
			out.Sources[i] = len(old.Groups) + 2
			endFaces++
		}
	}
	if !c.Groups.ClosedTerminal && endFaces == 0 {
		return Prepared{}, false, nil
	}
	radial, ok, err := cylinderExtensionBound(ctx, c.Section, n, out, c.Groups.Extension, m)
	if !ok || err != nil {
		return Prepared{}, false, err
	}
	for i, p := range out.Vertices {
		out.Vertices[i] = r3.NewVec(-p.Z, p.Y, -float64(a.Wing)*p.X)
	}
	ng := 3
	if c.Groups.ClosedTerminal {
		ng = 2
	}
	return prepareOutput(ctx, env, a.Admission, old, out, radial, w, ng, a.Wing, m)
}

func prepareOutput(ctx context.Context, env topo.Env, a Admission, old facetedrecord.Payload, out Mesh, radial float64, w WindowBounds, newGroups, wing int, m *proofbound.ReplayBudget) (Prepared, bool, error) {
	setback := a.Setback
	if err := m.ReserveBundle(ctx, uint64(12*(len(old.Tris)+len(out.Triangles))), uint64(4*(len(old.Tris)+len(out.Triangles))), uint64(256*(len(old.Tris)+len(out.Triangles)))); err != nil {
		return Prepared{}, false, err
	}
	p := facetedrecord.Payload{
		Verts: out.Vertices, Tris: out.Triangles, Src: out.Sources, Groups: slices.Clone(old.Groups),
		VertexBound: slices.Clone(old.VertexBound), MeshBound: old.MeshBound, Xform: r3.Identity(),
	}
	for i := range newGroups {
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
	inherited := old.MeshBound
	if wing != 0 {
		var ok bool
		var err error
		inherited, ok, err = localInheritedBound(ctx, old, current, w.support, wing, m)
		if !ok || err != nil {
			return Prepared{}, false, err
		}
	}
	bound := proofbound.AbsSumUpper(inherited, radial, w.Deviation, rounding)
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
