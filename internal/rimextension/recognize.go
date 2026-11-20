package rimextension

import (
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Candidate identifies the existing stop and the native carriers needed to
// certify it. Faces and Allowed are indexed by the input's source groups.
type Candidate struct {
	Section                          Section
	Groups                           Groups
	StopFacets                       map[int]bool
	Cap, Side, Boss, Round, StopWall *topo.Face
	Stops                            []*topo.Face
	Allowed                          map[int]bool
}

func exactCap(f *topo.Face) bool {
	pl, ok := f.Surface.(surfacegeom.Plane)
	return ok && f.RetainedTag && pl.Frame.U().Y == 0 && pl.Frame.V().Y == 0 && math.Abs(pl.Frame.N().Y) == 1
}
func verticalSide(f *topo.Face) bool {
	pl, ok := f.Surface.(surfacegeom.Plane)
	return ok && f.RetainedTag && pl.Frame.N().Y == 0
}
func cylinder(f *topo.Face) (surfacegeom.Cylinder, bool) {
	cf, ok := f.Surface.(surfacegeom.Cylinder)
	return cf, ok && f.RetainedTag && cf.Axis.X == 0 && cf.Axis.Z == 0 && math.Abs(cf.Axis.Y) == 1
}

type stockFaces struct {
	p      facetedrecord.Payload
	fs     []*topo.Face
	groups map[*topo.Face]int
	owner  map[edge]int
}

func loadStock(ctx context.Context, b *topo.Body, budget *proofbound.ReplayBudget) (stockFaces, bool, error) {
	var s stockFaces
	p, ok := b.Payload.(facetedrecord.Payload)
	if !ok || !b.Solid || p.Xform != r3.Identity() || (!p.BooleanSimilaritySource && !p.ContactAudited) || len(p.Tris) > 32768 || len(p.Verts) > 16384 || len(p.Verts) != len(p.VertexBound) || len(p.FaceOf) != len(p.Tris) || len(p.Src) != len(p.Tris) {
		return s, false, nil
	}
	n := uint64(len(p.Tris))
	if err := budget.ReserveBundle(ctx, 8*n, 4*n, 256*n); err != nil {
		return s, false, err
	}
	s = stockFaces{p: p, fs: topobuild.Faces(b), groups: map[*topo.Face]int{}, owner: map[edge]int{}}
	for i, t := range p.Tris {
		if i%128 == 0 {
			if e := ctx.Err(); e != nil {
				return s, false, e
			}
		}
		f, g := p.FaceOf[i], p.Src[i]
		if f < 0 || f >= len(s.fs) || g < 0 || g >= len(p.Groups) {
			return s, false, nil
		}
		face := s.fs[f]
		if prev, set := s.groups[face]; set && prev != g {
			s.groups[face] = -1
		} else {
			s.groups[face] = g
		}
		for j, a := range t {
			if a < 0 || a >= len(p.Verts) {
				return s, false, nil
			}
			e := edge{a, t[(j+1)%3]}
			if _, exists := s.owner[e]; exists {
				return s, false, nil
			}
			s.owner[e] = i
		}
	}
	if !facetedrecord.CertifiedFacetedSimilaritySource(p, len(s.fs)) {
		return s, false, nil
	}
	return s, true, nil
}
func (s stockFaces) adjacent(i int, j int) (int, bool) {
	t := s.p.Tris[i]
	k, ok := s.owner[edge{t[(j+1)%3], t[j]}]
	return k, ok
}
func (s stockFaces) onStop(i int, x float64) bool {
	t := s.p.Tris[i]
	return s.p.Verts[t[0]].X == x && s.p.Verts[t[1]].X == x && s.p.Verts[t[2]].X == x
}
func (s stockFaces) stop(ctx context.Context, c *Candidate, start, end r3.Vec, roundFace func(*topo.Face) bool, budget *proofbound.ReplayBudget) (bool, error) {
	seed := -1
	var round *topo.Face
	for i, t := range s.p.Tris {
		if s.fs[s.p.FaceOf[i]] != c.Cap {
			continue
		}
		for j, a := range t {
			v, w := s.p.Verts[a], s.p.Verts[t[(j+1)%3]]
			if v != start && w != start && v != end && w != end {
				continue
			}
			k, ok := s.adjacent(i, j)
			if !ok {
				return false, nil
			}
			f := s.fs[s.p.FaceOf[k]]
			if (v == start || w == start) && s.onStop(k, start.X) {
				seed = k
			}
			if v == end || w == end {
				if roundFace(f) {
					if round != nil && round != f {
						return false, nil
					}
					round = f
				}
			}
		}
	}
	if seed < 0 || round == nil {
		return false, nil
	}
	c.Round = round
	c.StopWall = s.fs[s.p.FaceOf[seed]]
	pending := []int{seed}
	c.StopFacets = map[int]bool{seed: true}
	stops := map[*topo.Face]bool{}
	var heel, toe r3.Vec
	seams := 0
	c.Groups.Stop = map[int]bool{}
	for len(pending) > 0 {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return false, err
		}
		i := pending[len(pending)-1]
		pending = pending[:len(pending)-1]
		f := s.fs[s.p.FaceOf[i]]
		stops[f] = true
		c.Groups.Stop[s.p.Src[i]] = true
		for j := range 3 {
			k, ok := s.adjacent(i, j)
			if !ok {
				return false, nil
			}
			if s.onStop(k, start.X) {
				if !c.StopFacets[k] {
					if len(c.StopFacets) >= 512 {
						return false, nil
					}
					c.StopFacets[k] = true
					pending = append(pending, k)
				}
				continue
			}
			other := s.fs[s.p.FaceOf[k]]
			if other == c.Cap || other == c.Side {
				continue
			}
			t := s.p.Tris[i]
			a, b := s.p.Verts[t[j]], s.p.Verts[t[(j+1)%3]]
			if a.Y != start.Y {
				a, b = b, a
			}
			if a.Y != start.Y || b.Y == start.Y || b.Z != start.Z {
				return false, nil
			}
			toe, heel = a, b
			seams++
			stops[other] = true
		}
	}
	if seams != 1 {
		return false, nil
	}
	c.Section = Section{Stop: start, End: end, Toe: toe, Heel: heel}
	if _, ok := newChart(c.Section); !ok {
		return false, nil
	}
	for _, f := range s.fs {
		if stops[f] {
			c.Stops = append(c.Stops, f)
		}
	}
	return true, nil
}

// Identify follows held adjacency only. The resulting candidate is never
// sufficient to admit a modification without the independent source checks.
func Identify(ctx context.Context, b *topo.Body, e *topo.Edge, budget *proofbound.ReplayBudget) (Candidate, bool, error) {
	return identify(ctx, b, e, 0, budget)
}

// IdentifyOuter follows adjacency in the signed longitudinal chart.
func IdentifyOuter(ctx context.Context, b *topo.Body, e *topo.Edge, wing int, budget *proofbound.ReplayBudget) (Candidate, bool, error) {
	if wing != -1 && wing != 1 {
		return Candidate{}, false, nil
	}
	return identify(ctx, b, e, wing, budget)
}

func identify(ctx context.Context, b *topo.Body, e *topo.Edge, wing int, budget *proofbound.ReplayBudget) (Candidate, bool, error) {
	var c Candidate
	if b == nil || !b.Solid || e == nil || !e.Convex || len(e.Faces) != 2 || e.Start == nil || e.End == nil || e.Start == e.End {
		return c, false, nil
	}
	if _, ok := e.Curve.(surfacegeom.Line3); !ok || budget == nil {
		return c, false, nil
	}
	for _, f := range e.Faces {
		if f == nil || f.Body != b {
			return c, false, nil
		}
		if exactCap(f) {
			c.Cap = f
		}
		if verticalSide(f) {
			c.Side = f
		}
	}
	if c.Cap == nil || c.Side == nil {
		return c, false, nil
	}
	if wing != 0 {
		pl, isPlane := c.Cap.Surface.(surfacegeom.Plane)
		if !isPlane || (pl.Frame.N().Y < 0) != c.Cap.Reversed {
			return c, false, nil
		}
	}
	a, bv := e.Start.Position, e.End.Position
	if a.X == bv.X || a.Y != bv.Y {
		return c, false, nil
	}
	s, ok, err := loadStock(ctx, b, budget)
	if !ok || err != nil {
		return c, false, err
	}
	if wing != 0 {
		if err := budget.ReserveBundle(ctx, 4*uint64(len(s.p.Verts)), uint64(len(s.p.Verts)), 24*uint64(len(s.p.Verts))); err != nil {
			return c, false, err
		}
		s.p.Verts = slices.Clone(s.p.Verts)
		for i, v := range s.p.Verts {
			s.p.Verts[i] = ShoulderPoint(v, wing)
		}
		a, bv = ShoulderPoint(a, wing), ShoulderPoint(bv, wing)
	}
	roundFace := func(f *topo.Face) bool { _, yes := cylinder(f); return yes }
	ok, err = s.stop(ctx, &c, a, bv, roundFace, budget)
	if !ok && err == nil {
		c.Stops = nil
		ok, err = s.stop(ctx, &c, bv, a, roundFace, budget)
	}
	if !ok || err != nil {
		return c, false, err
	}
	cr, _ := cylinder(c.Round)
	start, end := c.Section.Stop, c.Section.End
	// The support extends to the outside of the cap, unlike the profile's
	// own round end. Require adjacency to the cap in the selected X interval.
	for i, t := range s.p.Tris {
		if s.fs[s.p.FaceOf[i]] != c.Cap {
			continue
		}
		for j, a := range t {
			v := s.p.Verts[a]
			if v.X <= min(start.X, end.X) || v.X >= max(start.X, end.X) {
				continue
			}
			k, exists := s.adjacent(i, j)
			if !exists {
				return c, false, nil
			}
			f := s.fs[s.p.FaceOf[k]]
			cf, found := cylinder(f)
			if !found || f == c.Round || wing == 0 && (cf.Origin.X != cr.Origin.X || cf.Origin.Z != cr.Origin.Z) || math.Abs(cf.Radius.Base()-cr.Radius.Base()) > s.p.MeshBound {
				continue
			}
			if c.Boss != nil && c.Boss != f {
				return c, false, nil
			}
			c.Boss = f
		}
	}
	if c.Boss == nil {
		return c, false, nil
	}
	c.Allowed = map[int]bool{}
	for _, f := range append(slices.Clone(c.Stops), c.Cap, c.Side, c.Round, c.Boss) {
		g, ok := s.groups[f]
		if !ok || g < 0 {
			return c, false, nil
		}
		c.Allowed[g] = true
	}
	c.Groups.Cap, c.Groups.Side, c.Groups.Boss = s.groups[c.Cap], s.groups[c.Side], s.groups[c.Boss]
	c.Groups.Bevel = len(s.p.Groups)
	c.Groups.Extension = c.Groups.Bevel + 1
	return c, true, nil
}

// IdentifyTransverse finds a stopped rim joining two coordinate planes to a
// cylinder parallel to X. Its world-to-chart permutation preserves orientation.
func IdentifyTransverse(ctx context.Context, b *topo.Body, e *topo.Edge, m *proofbound.ReplayBudget) (Candidate, bool, error) {
	var c Candidate
	if m == nil || b == nil || !b.Solid || e == nil || !e.Convex || len(e.Faces) != 2 || e.Start == nil || e.End == nil {
		return c, false, nil
	}
	if _, ok := e.Curve.(surfacegeom.Line3); !ok {
		return c, false, nil
	}
	a, bv := e.Start.Position, e.End.Position
	if a.X != bv.X || a.Y != bv.Y || a.Z == bv.Z {
		return c, false, nil
	}
	for _, f := range e.Faces {
		if f == nil || f.Body != b || !f.RetainedTag {
			return c, false, nil
		}
		pl, ok := f.Surface.(surfacegeom.Plane)
		if !ok {
			return c, false, nil
		}
		n := pl.Frame.N()
		if f.Reversed {
			n = n.Scale(-1)
		}
		switch n {
		case r3.Vec{X: 1}:
			c.Cap = f
		case r3.Vec{Y: 1}:
			c.Side = f
		default:
			return c, false, nil
		}
	}
	if c.Cap == nil || c.Side == nil {
		return c, false, nil
	}
	s, ok, err := loadStock(ctx, b, m)
	if !ok || err != nil {
		return c, false, err
	}
	if err := m.ReserveBundle(ctx, 4*uint64(len(s.p.Verts)), uint64(len(s.p.Verts)), 24*uint64(len(s.p.Verts))); err != nil {
		return c, false, err
	}
	s.p.Verts = slices.Clone(s.p.Verts)
	for i, v := range s.p.Verts {
		s.p.Verts[i] = TransversePoint(v)
	}
	a, bv = TransversePoint(a), TransversePoint(bv)
	round := func(f *topo.Face) bool {
		v, ok := f.Surface.(surfacegeom.Cylinder)
		return ok && f.RetainedTag && !f.Reversed && v.Axis == (r3.Vec{X: 1})
	}
	ok, err = s.stop(ctx, &c, a, bv, round, m)
	if !ok && err == nil {
		c.Stops = nil
		ok, err = s.stop(ctx, &c, bv, a, round, m)
	}
	if !ok || err != nil {
		return c, false, err
	}
	c.Boss = c.Round
	c.Allowed = map[int]bool{}
	for _, f := range append(slices.Clone(c.Stops), c.Cap, c.Side, c.Boss) {
		g, ok := s.groups[f]
		if !ok || g < 0 {
			return c, false, nil
		}
		c.Allowed[g] = true
	}
	c.Groups.Cap, c.Groups.Side, c.Groups.Boss = s.groups[c.Cap], s.groups[c.Side], s.groups[c.Boss]
	c.Groups.Bevel = len(s.p.Groups)
	c.Groups.Extension = c.Groups.Bevel + 1
	c.Groups.OpenCylinderEnd = true
	return c, true, nil
}

// TransversePoint rotates a world-space point into the transverse rim chart.
func TransversePoint(v r3.Vec) r3.Vec { return r3.Vec{X: v.Z, Y: v.X, Z: v.Y} }
