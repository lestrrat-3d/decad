package trimshell

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/anasurf"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facemesh"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/trimgraph"
	"github.com/lestrrat-3d/decad/internal/trimrebuild"
	"github.com/lestrrat-3d/r3"
)

// Call is a Shell request already past modify §4 stage 1.
type Call struct {
	Inward bool
	T      float64 // mm
	TDelta float64
}

func refuse(format string, args ...any) error {
	return fmt.Errorf("%w: "+format, append([]any{decaderr.ErrUnsupported}, args...)...)
}

// Closed builds the closed shell of b over fp. matched is false when b does not lift (trimgraph.ErrNotLiftable),
// when Call.Inward is false, or when some edge of the graph is concave; the caller then keeps its refusal.
func Closed(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, call Call) (*topo.Body, bool, error) {
	// Step 1.
	if !call.Inward {
		return nil, false, nil
	}
	g, err := trimgraph.Lift(ctx, b, fp)
	if errors.Is(err, trimgraph.ErrNotLiftable) {
		return nil, false, nil
	}
	if err != nil {
		return nil, false, err
	}
	for _, e := range g.Edges {
		if !e.Convex {
			return nil, false, nil
		}
	}
	for _, f := range g.Faces {
		if len(f.Loops) != 1 {
			return nil, false, nil
		}
	}
	// Step 2.
	t, ok := proofbound.RatOf(call.T)
	if call.TDelta != 0 || !ok || t.Sign() <= 0 {
		return nil, true, refuse("a trim-graph shell needs a thickness that converts exactly to millimetres (trim TG7)")
	}
	body, err := closed(ctx, env, g, fp, t)
	return body, true, err
}

func closed(ctx context.Context, env topo.Env, g trimgraph.Graph, fp facetedrecord.Payload, t *big.Rat) (*topo.Body, error) {
	// Step 3.
	outer := make([]anasurf.Plane, len(g.Faces))
	inner := make([]anasurf.Plane, len(g.Faces))
	for i, f := range g.Faces {
		tag, ok := f.PlaneTag()
		if !ok {
			return nil, refuse("face %d is not a plane (trim TG3)", i)
		}
		p, err := anasurf.PlaneOf(tag, f.Reversed)
		if err != nil {
			return nil, refuse("the plane of face %d has no exact form (trim TG3): %w", i, err)
		}
		q, err := p.Inward(t)
		if err != nil {
			return nil, refuse("the offset of face %d has no exact form (trim TG3): %w", i, err)
		}
		outer[i], inner[i] = p, q
	}
	// Step 4.
	meets := make([]anasurf.Point, len(g.Vertices))
	for vi, v := range g.Vertices {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		pt, err := anasurf.Meet3(inner[v.Faces[0]], inner[v.Faces[1]], inner[v.Faces[2]])
		if err != nil {
			return nil, refuse("the offset faces at %v do not meet in one point (trim TG3)", v.Pos)
		}
		meets[vi] = pt
	}
	// Step 5, gate C1.
	for vi, v := range g.Vertices {
		for j := range g.Faces {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			if j == v.Faces[0] || j == v.Faces[1] || j == v.Faces[2] {
				continue
			}
			if !anasurf.StrictlyInside(meets[vi], inner[j]) {
				held, _ := meets[vi].Held()
				return nil, refuse("the cavity vertex at %v leaves the offset half-space of face %d (trim TG6)", held, j)
			}
		}
	}
	// Step 6 and 7.
	s := trimrebuild.Splice{Keep: make([]int, len(fp.Tris))}
	for k := range s.Keep {
		s.Keep[k] = k
	}
	for _, pt := range meets {
		held, bound := pt.Held()
		s.Verts = append(s.Verts, held)
		s.Bounds = append(s.Bounds, bound)
	}
	for i, f := range g.Faces {
		nf, err := cavityFace(ctx, env, g, fp, i, f, outer[i], inner[i], s, t)
		if err != nil {
			return nil, err
		}
		s.Faces = append(s.Faces, nf)
	}
	// Step 8.
	return trimrebuild.Build(ctx, env, fp, s)
}

// startOf is the vertex a loop use leaves from.
func startOf(g trimgraph.Graph, u trimgraph.Use) int {
	if u.Forward {
		return g.Edges[u.Edge].Start
	}
	return g.Edges[u.Edge].End
}

// cavityFace builds cavity face i: the outer loop's vertex cycle over the meets, reversed, meshed on the offset
// plane.
func cavityFace(ctx context.Context, env topo.Env, g trimgraph.Graph, fp facetedrecord.Payload, i int,
	f trimgraph.Face, p, q anasurf.Plane, s trimrebuild.Splice, t *big.Rat) (trimrebuild.NewFace, error) {
	walk := f.Loops[0]
	cycle := make([]int, len(walk))
	for k, u := range walk {
		cycle[len(walk)-1-k] = startOf(g, u)
	}
	loop := facemesh.Loop{}
	for _, vi := range cycle {
		loop.Points = append(loop.Points, s.Verts[vi])
		loop.Bounds = append(loop.Bounds, s.Bounds[vi])
	}
	negN := [3]*big.Rat{new(big.Rat).Neg(p.N[0]), new(big.Rat).Neg(p.N[1]), new(big.Rat).Neg(p.N[2])}
	if err := checkOrientation(i, negN, loop); err != nil {
		return trimrebuild.NewFace{}, err
	}
	tag, ok := f.PlaneTag()
	if !ok {
		return trimrebuild.NewFace{}, refuse("face %d is not a plane (trim TG3)", i)
	}
	local, err := facemesh.Planar(ctx, negN, tag.Frame.U(), tag.Frame.V(), []facemesh.Loop{loop})
	if err != nil {
		switch {
		case errors.Is(err, facemesh.ErrMargin):
			return trimrebuild.NewFace{}, refuse("a rebuilt facet of face %d fails its orientation margin (trim TG5)", i)
		case errors.Is(err, facemesh.ErrDropped):
			return trimrebuild.NewFace{}, refuse("the cavity mesh of face %d drops a boundary point (trim TG5)", i)
		case ctx.Err() != nil:
			return trimrebuild.NewFace{}, ctx.Err()
		}
		return trimrebuild.NewFace{}, refuse("the cavity of face %d cannot be triangulated (trim TG5): %w", i, err)
	}
	tris := make([][3]int, len(local))
	for k, tri := range local {
		for c, li := range tri {
			tris[k][c] = len(fp.Verts) + cycle[li]
		}
	}
	group, err := cavityGroup(env, i, f, p, q, t)
	if err != nil {
		return trimrebuild.NewFace{}, err
	}
	return trimrebuild.NewFace{Group: group, Tris: tris}, nil
}

// checkOrientation is TG4: the loop's exact signed area about n must be positive beyond perimeter·β.
func checkOrientation(face int, n [3]*big.Rat, loop facemesh.Loop) error {
	drop := refuse("the shell offset drops face %d of the receiver (trim TG4)", face)
	pts := make([][3]*big.Rat, len(loop.Points))
	for k, p := range loop.Points {
		x, okx := proofbound.RatOf(p.X)
		y, oky := proofbound.RatOf(p.Y)
		z, okz := proofbound.RatOf(p.Z)
		if !okx || !oky || !okz {
			return drop
		}
		pts[k] = [3]*big.Rat{x, y, z}
	}
	// twice the area vector A2 = sum p_k x p_{k+1}; signed area about n is A2·n / (2|n|).
	dotN := new(big.Rat)
	perimeter := 0.0
	beta := 0.0
	for k := range pts {
		a, b := pts[k], pts[(k+1)%len(pts)]
		c := [3]*big.Rat{
			new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
			new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
			new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
		}
		for i := range c {
			dotN.Add(dotN, new(big.Rat).Mul(c[i], n[i]))
		}
		edge := new(big.Rat)
		for i := range a {
			d := new(big.Rat).Sub(b[i], a[i])
			edge.Add(edge, d.Mul(d, d))
		}
		perimeter = proofbound.AbsSumUpper(perimeter, proofbound.RatSqrtUp(edge))
		beta = max(beta, loop.Bounds[k])
	}
	nn := new(big.Rat)
	for _, c := range n {
		nn.Add(nn, new(big.Rat).Mul(c, c))
	}
	need, ok := proofbound.RatOf(proofbound.ProductUpper(proofbound.ProductUpper(perimeter, beta), 2*proofbound.RatSqrtUp(nn)))
	if !ok || dotN.Cmp(need) <= 0 {
		return drop
	}
	return nil
}

// cavityGroup is step 7: the group of cavity face i on the offset plane, stating its departure from the true plane.
func cavityGroup(env topo.Env, i int, f trimgraph.Face, p, q anasurf.Plane, t *big.Rat) (facetedrecord.Group, error) {
	tag, ok := f.PlaneTag()
	if !ok {
		return facetedrecord.Group{}, refuse("face %d is not a plane (trim TG3)", i)
	}
	norm, err := p.Norm()
	if err != nil {
		return facetedrecord.Group{}, refuse("the plane of face %d has no exact form (trim TG3): %w", i, err)
	}
	invNorm, ok := proofbound.IntervalQuo(proofbound.PointInterval(big.NewRat(1, 1)), norm)
	if !ok {
		return facetedrecord.Group{}, refuse("the plane of face %d has no exact form (trim TG3)", i)
	}
	mid := new(big.Rat).Add(invNorm.Lo, invNorm.Hi)
	mid.Quo(mid, big.NewRat(2, 1))
	// O' = O - t * Nout / |Nout|, taken at the midpoint of the 1/|N| enclosure and rounded to the nearest float.
	origin := tag.Frame.Origin()
	var moved [3]float64
	for k, c := range [3]float64{origin.X, origin.Y, origin.Z} {
		o, ok := proofbound.RatOf(c)
		if !ok {
			return facetedrecord.Group{}, refuse("the plane of face %d has a non-finite origin (trim TG3)", i)
		}
		step := new(big.Rat).Mul(t, mid)
		step.Mul(step, p.N[k])
		moved[k], _ = o.Sub(o, step).Float64()
	}
	newOrigin := r3.NewVec(moved[0], moved[1], moved[2])
	frame, err := r3.NewFrame(newOrigin, tag.Frame.U(), tag.Frame.V())
	if err != nil {
		return facetedrecord.Group{}, refuse("the offset frame of face %d is degenerate (trim TG3): %w", i, err)
	}
	departure, err := departureOf(p, q, tag, frame, newOrigin, norm.Lo)
	if err != nil {
		return facetedrecord.Group{}, refuse("the offset plane of face %d has no departure bound (trim TG3): %w", i, err)
	}
	return facetedrecord.Group{
		Origins:     []provenance.Ref{{Producer: env.Ref, Role: fmt.Sprintf("cavity(%d)", i)}},
		Source:      facetedrecord.FaceSource(env.Ref, 0, i, f.Node),
		Surface:     surfacegeom.Plane{Frame: frame},
		Retained:    true,
		Reversed:    !f.Reversed,
		Planar:      true,
		NormalBound: departure,
	}, nil
}

// departureOf bounds how far the stored plane (frame through origin) lies from the true offset plane q: the tilt of
// its normal when NewFrame changed the frame vectors, plus the distance of its origin from every plane of q.
func departureOf(p, q anasurf.Plane, tag surfacegeom.Plane, frame r3.Frame, origin r3.Vec, normLow *big.Rat) (float64, error) {
	// Position: |N·O' - c'| / |N| over c' in q.C.
	var dot big.Rat
	for k, c := range [3]float64{origin.X, origin.Y, origin.Z} {
		r, ok := proofbound.RatOf(c)
		if !ok {
			return 0, errors.New("the origin is not finite")
		}
		dot.Add(&dot, new(big.Rat).Mul(p.N[k], r))
	}
	far := new(big.Rat).Abs(new(big.Rat).Sub(&dot, q.C.Lo))
	if other := new(big.Rat).Abs(new(big.Rat).Sub(&dot, q.C.Hi)); other.Cmp(far) > 0 {
		far = other
	}
	far.Quo(far, normLow)
	position := proofbound.RatFloatUp(far)

	// Tilt: zero when the stored frame vectors are the receiver's, else 2|N0' - N0| / |N0| for the unreversed normals.
	if frame.U() == tag.Frame.U() && frame.V() == tag.Frame.V() {
		return position, nil
	}
	var diff big.Rat
	n0, ok0 := ratNormal(tag.Frame.U(), tag.Frame.V())
	n1, ok1 := ratNormal(frame.U(), frame.V())
	if !ok0 || !ok1 {
		return 0, errors.New("a frame vector is not finite")
	}
	for k := range n0 {
		d := new(big.Rat).Sub(n1[k], n0[k])
		diff.Add(&diff, d.Mul(d, d))
	}
	tilt := new(big.Rat).SetFloat64(proofbound.RatSqrtUp(&diff))
	tilt.Mul(tilt, big.NewRat(2, 1))
	tilt.Quo(tilt, normLow)
	return proofbound.AbsSumUpper(position, proofbound.RatFloatUp(tilt)), nil
}

func ratNormal(u, v r3.Vec) ([3]*big.Rat, bool) {
	f := func(x float64) *big.Rat { r, _ := proofbound.RatOf(x); return r }
	for _, c := range []float64{u.X, u.Y, u.Z, v.X, v.Y, v.Z} {
		if _, ok := proofbound.RatOf(c); !ok {
			return [3]*big.Rat{}, false
		}
	}
	cross := func(a, b, c, d float64) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(f(a), f(b)), new(big.Rat).Mul(f(c), f(d)))
	}
	return [3]*big.Rat{cross(u.Y, v.Z, u.Z, v.Y), cross(u.Z, v.X, u.X, v.Z), cross(u.X, v.Y, u.Y, v.X)}, true
}
