package booleaneval

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// placedCarriers reports, per facet of m, whether the facet lies exactly in the true plane of its source face when
// body is a placed analytic operand (agent-docs/coplanar-contact-design.md §17.2). Such a body denotes its record at
// the identity placement carried through its payload's placement read as exact rationals, and its float tags only
// approximate that image. A plane face whose identity rebuild is an exact plane P₀ (identityPlane) has its true piece
// in the exact image A(P₀); a facet whose three corners lie in A(P₀), decided in exact rational arithmetic, is
// carrier-exact.
//
// nil means no facet is: no payload, a faceted payload (its placed groups state their own claim, Group.PlacedPlane),
// the identity placement (the analytic reading applies), a rebuild that fails, or faces that do not pair one for one
// with the placed body's, the same pairing placedOperandDepartures reads.
func placedCarriers(ctx context.Context, body *topo.Body, m *topo.Mesh) ([]bool, error) {
	if body == nil || body.Payload == nil || m == nil {
		return nil, nil
	}
	if _, faceted := body.Payload.(facetedrecord.Payload); faceted {
		return nil, nil
	}
	place := body.Payload.Transform()
	if place == r3.Identity() {
		return nil, nil
	}
	linear, err := massmoment.PlacementRotation(place)
	if err != nil {
		return nil, nil //nolint:nilerr // a placement with no exact map leaves every facet unread
	}
	shift, ok := massmoment.ExactVec(place.Translation())
	if !ok {
		return nil, nil
	}
	env := topo.Env{Doc: &topo.Document{}, Ref: body.Origin.Producer, Alloc: scratchAlloc{}}
	twin, err := body.Payload.Placed(ctx, env, r3.Identity())
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, nil
	}
	unplaced, placed := topobuild.Faces(twin), topobuild.Faces(body)
	if len(unplaced) != len(placed) {
		return nil, nil
	}
	index := make(map[*topo.Face]int, len(placed))
	for i, f := range placed {
		index[f] = i
	}
	cof := cofactor(linear)
	exact := exactVerts{verts: m.Vertices}
	planes := make(map[int]*ratPlane)
	var out []bool
	for t, f := range m.Source {
		if f == nil || t >= len(m.Triangles) {
			continue
		}
		i, ok := index[f]
		if !ok {
			continue
		}
		pl, seen := planes[i]
		if !seen {
			if _, isPlane := f.Surface.(surfacegeom.Plane); isPlane && unplaced[i].Surface.Kind() == f.Surface.Kind() {
				if p0, ok := identityPlane(unplaced[i]); ok {
					pl = p0.imageWith(linear, cof, shift)
				}
			}
			planes[i] = pl
		}
		if pl == nil || !pl.holdsExact(&exact, m.Triangles[t]) {
			continue
		}
		if out == nil {
			out = make([]bool, len(m.Triangles))
		}
		out[t] = true
	}
	return out, nil
}

// identityPlane returns the plane a face of an unplaced rebuild lies in exactly: a Plane tag that states no normal
// departure, no displacement and no denoted patch, which for an analytic face is its own surface and for a retained
// one the retained-plane reading (retainedPlaneOf) at a zero displacement.
func identityPlane(f *topo.Face) (ratPlane, bool) {
	if f == nil || f.NormalBound != 0 || f.TagDeparture != 0 || f.AxialDelta != 0 || f.Denoted != nil {
		return ratPlane{}, false
	}
	pl, ok := f.Surface.(surfacegeom.Plane)
	if !ok {
		return ratPlane{}, false
	}
	o, okO := massmoment.ExactVec(pl.Frame.Origin())
	u, okU := massmoment.ExactVec(pl.Frame.U())
	v, okV := massmoment.ExactVec(pl.Frame.V())
	if !okO || !okU || !okV {
		return ratPlane{}, false
	}
	return ratPlane{o: o, n: ratCross3(u, v)}, true
}

// ratPlane is the plane through o normal to n, exact rationals. d is n·o
// when image built the plane, and nil otherwise.
type ratPlane struct {
	o, n [3]*big.Rat
	d    *big.Rat
}

// image is the plane's exact image under x ↦ L·x + t: through L·o + t, normal to (L·u)×(L·v) for any u, v spanning
// it, which is cof(L)·n. nil when L maps the plane to no plane.
func (p ratPlane) image(linear [3][3]*big.Rat, shift [3]*big.Rat) *ratPlane {
	return p.imageWith(linear, cofactor(linear), shift)
}

// cofactor is cof(L): column k is the cross product of the other two columns of L, in cyclic order. The image of
// every plane under one placement shares it.
func cofactor(linear [3][3]*big.Rat) [3][3]*big.Rat {
	col := func(k int) [3]*big.Rat { return [3]*big.Rat{linear[0][k], linear[1][k], linear[2][k]} }
	return [3][3]*big.Rat{ratCross3(col(1), col(2)), ratCross3(col(2), col(0)), ratCross3(col(0), col(1))}
}

// imageWith is image with cof(L) given as c.
func (p ratPlane) imageWith(linear, c [3][3]*big.Rat, shift [3]*big.Rat) *ratPlane {
	var o [3]*big.Rat
	for i := range 3 {
		s := new(big.Rat).Set(shift[i])
		for k := range 3 {
			s.Add(s, new(big.Rat).Mul(linear[i][k], p.o[k]))
		}
		o[i] = s
	}
	var n [3]*big.Rat
	for i := range 3 {
		s := new(big.Rat)
		for k := range 3 {
			s.Add(s, new(big.Rat).Mul(c[k][i], p.n[k]))
		}
		n[i] = s
	}
	if n[0].Sign() == 0 && n[1].Sign() == 0 && n[2].Sign() == 0 {
		return nil
	}
	d := new(big.Rat)
	for i := range 3 {
		d.Add(d, new(big.Rat).Mul(n[i], o[i]))
	}
	return &ratPlane{o: o, n: n, d: d}
}

// holdsExact reports whether every corner of triangle tri lies exactly in a
// plane image built, reading each corner's exact value once per mesh through
// exact: n·e = d is n·(e − o) = 0.
func (p *ratPlane) holdsExact(exact *exactVerts, tri [3]int) bool {
	s, term := new(big.Rat), new(big.Rat)
	for _, v := range tri {
		e, ok := exact.at(v)
		if !ok {
			return false
		}
		s.SetInt64(0)
		for i := range 3 {
			s.Add(s, term.Mul(p.n[i], e[i]))
		}
		if s.Cmp(p.d) != 0 {
			return false
		}
	}
	return true
}

// exactVerts reads mesh vertices as exact rationals on first use and keeps
// them for the other triangles that share each vertex.
type exactVerts struct {
	verts []r3.Vec
	vals  [][3]*big.Rat
	state []int8 // 0 unread, 1 exact, -1 not exactly representable
}

func (x *exactVerts) at(v int) ([3]*big.Rat, bool) {
	if x.state == nil {
		x.vals = make([][3]*big.Rat, len(x.verts))
		x.state = make([]int8, len(x.verts))
	}
	switch x.state[v] {
	case 1:
		return x.vals[v], true
	case -1:
		return [3]*big.Rat{}, false
	}
	e, ok := massmoment.ExactVec(x.verts[v])
	if !ok {
		x.state[v] = -1
		return [3]*big.Rat{}, false
	}
	x.vals[v], x.state[v] = e, 1
	return e, true
}

func ratCross3(a, b [3]*big.Rat) [3]*big.Rat {
	m := func(x, y, z, w *big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(x, y), new(big.Rat).Mul(z, w))
	}
	return [3]*big.Rat{m(a[1], b[2], a[2], b[1]), m(a[2], b[0], a[0], b[2]), m(a[0], b[1], a[1], b[0])}
}
