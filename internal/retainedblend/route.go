package retainedblend

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Fillet fillets the selected edges of b, a mesh Boolean result with retained
// tags. recognized is false when some selected edge is in no class of
// agent-docs/retained-edge-blend-design.md Table RR; the caller then keeps its own
// refusal. All edges must be non-nil.
func Fillet(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	radius, radiusDelta float64) (*topo.Body, bool, error) {
	return blend(ctx, env, b, fp, edges, blendSpec{fillet: true, r: radius}, radiusDelta, "fillet")
}

// Reference is a resolved two-distance chamfer (agent-docs/retained-modify-reach-design.md
// §4.1): Refs maps each selected edge to its one reference face of b, which takes
// the positional distance d. The other face adjacent to the edge takes Other.
type Reference struct {
	Refs              map[*topo.Edge]*topo.Face
	Other, OtherDelta float64
}

// Chamfer bevels the selected edges of b. Without ref the setback is d on both
// faces; with ref each edge's reference face takes d and its other face takes
// ref.Other. All edges must be non-nil, and every edge of an asymmetric chamfer
// must have an entry in ref.Refs.
func Chamfer(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	d, dDelta float64, ref *Reference) (*topo.Body, bool, error) {
	spec := blendSpec{dA: d, dB: d}
	if ref != nil {
		spec.asym = &asymSpec{refs: ref.Refs, other: ref.Other}
		dDelta = math.Max(math.Abs(dDelta), math.Abs(ref.OtherDelta))
	}
	return blend(ctx, env, b, fp, edges, spec, dDelta, "chamfer")
}

// edgeKind is the class of a selected edge.
type edgeKind int

const (
	closedRim edgeKind = iota
	sphereRimKind
	straightLine
	cylinderLine
	openArc
	cylinderPair
)

func blend(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	spec blendSpec, delta float64, op string) (*topo.Body, bool, error) {
	if len(edges) == 0 || !b.Solid {
		return nil, false, nil
	}
	for _, e := range edges {
		if !looksLikeRim(e) && !looksLikeSphereRim(e) && !looksLikeLine(e) && !looksLikeCylLine(e) &&
			!looksLikeArc(e) && !looksLikeCylCyl(e) {
			return nil, false, nil
		}
	}
	if delta != 0 {
		return nil, true, fmt.Errorf("%w: a retained edge blend needs a size that converts exactly to millimetres", decaderr.ErrUnsupported)
	}
	if spec.asym != nil && !(spec.asym.other > 0) {
		return nil, true, fmt.Errorf("%w: a blend needs a positive size", decaderr.ErrDegenerate)
	}
	if err := checkFinite(spec.r, spec.dA, spec.dB, spec.otherSize()); err != nil {
		return nil, true, err
	}
	faceIndex := faceIndexOf(b)
	type target struct {
		rimKey  rimKey
		lineKey lineKey
		kind    edgeKind
		ref     refTag // the tag of the edge's reference face
	}
	targets := make([]target, len(edges))
	for i, e := range edges {
		if spec.asym == nil {
			break
		}
		f := spec.asym.refs[e]
		if _, ok := faceIndex[f]; f == nil || !ok {
			return nil, true, fmt.Errorf("%w: a two-distance chamfer has no reference face for a selected edge",
				decaderr.ErrUnsupported)
		}
		targets[i].ref = tagOf(f)
	}
	chains, handled, err := planChains(b, edges)
	if err != nil {
		return nil, true, err
	}
	if handled {
		refs := make([]refTag, len(edges))
		for i := range edges {
			refs[i] = targets[i].ref
		}
		out, err := blendChains(ctx, env, b, fp, edges, refs, chains, spec, op)
		return out, true, err
	}
	var corners []r3.Vec
	claim := func(ends [2]lineEnd, scale float64) error {
		for _, end := range ends {
			for _, c := range corners {
				if c.Sub(end.exact).Len() <= 1e-9*scale {
					return fmt.Errorf("%w: two selected edges share a corner", decaderr.ErrUnsupported)
				}
			}
			corners = append(corners, end.exact)
		}
		return nil
	}
	for i, e := range edges {
		switch {
		case looksLikeLine(e):
			l, err := readLine(e, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if err := claim(l.ends, l.scale); err != nil {
				return nil, true, err
			}
			targets[i].lineKey, targets[i].kind = l.key(), straightLine
		case looksLikeArc(e):
			a, err := readArc(e, fp, faceIndex)
			if err != nil {
				// An arc this route cannot read is left to the generic refusal, which
				// names the whole selection.
				return nil, false, nil //nolint:nilerr // declining is the generic refusal's cue, not a swallowed failure
			}
			if err := a.checkSize(spec); err != nil {
				return nil, true, err
			}
			if err := claim(a.ends, a.scale); err != nil {
				return nil, true, err
			}
			targets[i].lineKey, targets[i].kind = a.arcKey(), openArc
		case looksLikeCylCyl(e):
			c, err := readCylCyl(e, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if err := claim(c.ends, c.scale); err != nil {
				return nil, true, err
			}
			targets[i].lineKey, targets[i].kind = c.key(), cylinderPair
		case looksLikeCylLine(e):
			c, err := readCylLine(e, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if err := claim(c.ends, c.scale); err != nil {
				return nil, true, err
			}
			targets[i].lineKey, targets[i].kind = c.key(), cylinderLine
		case looksLikeSphereRim(e):
			s, err := readSphereRim(e, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if err := s.checkSize(spec); err != nil {
				return nil, true, err
			}
			targets[i].rimKey, targets[i].kind = s.key(), sphereRimKind
		default:
			r, err := readRim(e, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if err := r.checkSize(spec); err != nil {
				return nil, true, err
			}
			targets[i].rimKey, targets[i].kind = r.key(), closedRim
		}
		for _, earlier := range targets[:i] {
			if (earlier.kind == closedRim || earlier.kind == sphereRimKind) && earlier.kind == targets[i].kind &&
				earlier.rimKey == targets[i].rimKey {
				return nil, true, fmt.Errorf("%w: the selection names one rim twice", decaderr.ErrDegenerate)
			}
		}
	}
	cur, curFP := b, fp
	for i, e := range edges {
		if i > 0 {
			var ok bool
			switch targets[i].kind {
			case straightLine:
				e, ok = findLine(cur, targets[i].lineKey, 64*ulp*math.Max(1, math.Abs(targets[i].lineKey.p0.X)))
			case openArc:
				e, ok = findArc(cur, targets[i].lineKey, 64*ulp*math.Max(1, math.Abs(targets[i].lineKey.p0.X)))
			case cylinderPair:
				e, ok = findCylCyl(cur, targets[i].lineKey, 64*ulp*math.Max(1, math.Abs(targets[i].lineKey.p0.X)))
			case cylinderLine:
				e, ok = findCylLine(cur, targets[i].lineKey, 64*ulp*math.Max(1, math.Abs(targets[i].lineKey.p0.X)))
			case sphereRimKind:
				e, ok = findRim(cur, targets[i].rimKey, 64*ulp*math.Max(1, math.Abs(targets[i].rimKey.radius)),
					looksLikeSphereRim)
			default:
				e, ok = findRim(cur, targets[i].rimKey, 64*ulp*math.Max(1, math.Abs(targets[i].rimKey.radius)),
					looksLikeRim)
			}
			if !ok {
				return nil, true, fmt.Errorf("%w: a selected edge is no longer an edge of the partly blended body", decaderr.ErrUnsupported)
			}
		}
		faceIndex = faceIndexOf(cur)
		role := fmt.Sprintf("%s(%d)", op, i)
		var next *topo.Body
		var err error
		step := spec
		switch targets[i].kind {
		case straightLine:
			var l line
			if l, err = readLine(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, l.faceA, l.faceB)
			}
			if err == nil {
				next, err = rewriteLineEdge(ctx, env, curFP, l, step, role, i, faceIndex)
			}
		case openArc:
			var a arcRim
			if a, err = readArc(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, a.planeFace, a.wallFace)
			}
			if err == nil {
				next, err = rewriteArc(ctx, env, curFP, a, step, role, i, faceIndex)
			}
		case cylinderPair:
			var c cylCyl
			if c, err = readCylCyl(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, c.wall[0].face, c.wall[1].face)
			}
			if err == nil {
				next, err = rewriteCylCyl(ctx, env, curFP, c, step, role, i, faceIndex)
			}
		case cylinderLine:
			var c cylLine
			if c, err = readCylLine(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, c.faceA, c.faceB)
			}
			if err == nil {
				next, err = rewriteCylLine(ctx, env, curFP, c, step, role, i, faceIndex)
			}
		case sphereRimKind:
			var s sphereRim
			if s, err = readSphereRim(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, s.planeFace, s.wallFace)
			}
			if err == nil {
				next, err = rewriteSphereRim(ctx, env, curFP, s, step, role, i, faceIndex)
			}
		default:
			var r rim
			if r, err = readRim(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, r.planeFace, r.wallFace)
			}
			if err == nil {
				next, err = rewriteRim(ctx, env, curFP, r, step, role, i, faceIndex)
			}
		}
		if err != nil {
			return nil, true, err
		}
		cur = next
		var ok bool
		if curFP, ok = next.Payload.(facetedrecord.Payload); !ok {
			return nil, true, fmt.Errorf("%w: the blended body lost its payload", decaderr.ErrBooleanFailed)
		}
	}
	return cur, true, nil
}

// rewriteRim blends one rim and builds the body over the rewritten mesh.
func rewriteRim(ctx context.Context, env topo.Env, fp facetedrecord.Payload, r rim,
	spec blendSpec, role string, ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	if err := r.checkSize(spec); err != nil {
		return nil, err
	}
	w := newWork(fp, r, spec)
	// looksLikeRim admitted the face, so its plane tag reads.
	planeTag, _ := retainedPlane(r.planeFace)
	ringA, groupA, err := w.rewritePlane(faceIndex[r.planeFace], planeTag, spec.planeSetback())
	if err != nil {
		return nil, err
	}
	ringB, groupB, err := w.rewriteWall(faceIndex[r.wallFace], spec.wallSetback(), r.radius, nil)
	if err != nil {
		return nil, err
	}
	return w.finishRim(ctx, env, w.tag(), ringA, ringB, [2]int{groupA, groupB}, role, ord, faceIndex)
}

// finishRim adds the blend face's group, emits the strip between the plane's
// ring and the wall's, and builds the body. shrunk are the groups of the plane
// and the wall.
func (w *work) finishRim(ctx context.Context, env topo.Env, tag blendTag, ringA, ringB []ringPt, shrunk [2]int,
	role string, ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	groups := append([]facetedrecord.Group(nil), w.fp.Groups...)
	blendGroup := len(groups)
	groups = append(groups, facetedrecord.Group{
		Origins:  []provenance.Ref{{Producer: env.Ref, Role: role}},
		Source:   facetedrecord.BlendSource(env.Ref, ord),
		Surface:  tag.surface,
		Retained: true,
		Reversed: tag.reversed,
	})
	// The two shrunk faces keep their surfaces but lose the whole-group
	// centroid certificate.
	for _, g := range shrunk {
		groups[g].HasCentroid = false
		groups[g].Centroid = measurement.VecMeasurement{}
	}
	stripFrom := len(w.out)
	w.strip(tag, ringA, ringB, blendGroup)
	return w.build(ctx, env, groups, stripFrom, tag, faceIndex)
}

// build assembles the rewritten payload and the body over it. faceIndex
// numbers the faces of the body w.fp holds, as w.fp.FaceOf does.
func (w *work) build(ctx context.Context, env topo.Env, groups []facetedrecord.Group, stripFrom int,
	tag blendTag, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	fp := w.fp
	gone := map[int]bool{}
	for _, f := range w.removed {
		gone[f] = true
	}
	var facets [][3]int
	var src []int
	for i, t := range fp.Tris {
		if !gone[i] {
			facets = append(facets, t)
			src = append(src, fp.Src[i])
		}
	}
	facets = append(facets, w.out...)
	src = append(src, w.outSrc...)

	verts, beta, facets := compact(w.verts, w.beta, facets)
	meshBound := facetproof.FacetBoundMax(facets, beta)
	area, vol, err := w.measureChange(ctx, tag, stripFrom, faceIndex)
	if err != nil {
		return nil, err
	}
	next := facetedrecord.Payload{
		Verts: verts, VertexBound: beta, Tris: facets, Src: src, Groups: groups,
		MeshBound:   meshBound,
		VolSymDiff:  proofbound.AbsSumUpper(fp.VolSymDiff, vol),
		AreaSlack:   proofbound.AbsSumUpper(fp.AreaSlack, area),
		DPair:       facetedrecord.SourceDiameterUpper(verts, meshBound),
		Xform:       fp.Xform,
		ExactPlaced: fp.ExactPlaced,
	}
	body, err := facetedrecord.Build(ctx, env, next)
	if err != nil {
		return nil, fmt.Errorf("%w: the blended mesh fails its held audit: %w", decaderr.ErrUnsupported, err)
	}
	return body, nil
}

// compact drops the vertices no facet uses and renumbers the rest.
func compact(verts []r3.Vec, beta []float64, facets [][3]int) ([]r3.Vec, []float64, [][3]int) {
	remap := make([]int, len(verts))
	for i := range remap {
		remap[i] = -1
	}
	var outV []r3.Vec
	var outB []float64
	out := make([][3]int, len(facets))
	for i, t := range facets {
		for k, v := range t {
			if remap[v] < 0 {
				remap[v] = len(outV)
				outV = append(outV, verts[v])
				outB = append(outB, beta[v])
			}
			out[i][k] = remap[v]
		}
	}
	return outV, outB, out
}

// heldArea is the float area of the facets with the rounding it can carry.
func heldArea(verts []r3.Vec, facets [][3]int) (float64, float64) {
	area, slop := 0.0, 0.0
	for _, t := range facets {
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		area += b.Sub(a).Cross(c.Sub(a)).Len() / 2
		slop = proofbound.AbsSumUpper(slop, proofbound.FacetAreaTermSlop(a, b, c))
	}
	slop = proofbound.AbsSumUpper(slop, proofbound.ProductUpper(float64(len(facets)+2)*0x1p-52, area))
	return area, slop
}

// measureChange returns the area slack and volume bound the rewrite adds
// (agent-docs/retained-edge-blend-design.md §6.3, Claim K). The area change of the
// held mesh is compared with the exact change: the annulus and band removed
// and the blend face added. The old body's area error was proven only with
// the geometric terms of every face, and the rebuilt body charges the
// rewritten faces' terms afresh from their new facets, which can be smaller:
// the slack therefore carries the old terms of the rewritten faces.
func (w *work) measureChange(ctx context.Context, tag blendTag, stripFrom int,
	faceIndex map[*topo.Face]int) (areaSlack, volSymDiff float64, err error) {
	oldArea, oldSlop := heldArea(w.fp.Verts, pick(w.fp.Tris, w.removed))
	newArea, newSlop := heldArea(w.verts, w.out)
	stripArea, _ := heldArea(w.verts, w.out[stripFrom:])

	deltaHeld := newArea - oldArea
	deltaTrue := tag.trueArea - w.removedTrue
	// Every term is a float evaluation; charge the areas' own rounding.
	scale := math.Max(math.Abs(deltaHeld), math.Abs(deltaTrue)) + oldArea + newArea + w.removedTrue
	oldGeom, err := w.rewrittenGeom(ctx, faceIndex)
	if err != nil {
		return 0, 0, err
	}
	areaSlack = proofbound.AbsSumUpper(math.Abs(deltaHeld-deltaTrue), oldSlop, newSlop, 64*0x1p-52*scale, oldGeom)

	// A raised inner vertex of a curved wall bounds the facets beside the
	// strip that now face the blend.
	eps := w.raised
	for _, t := range w.out[stripFrom:] {
		eps = max(eps, w.beta[t[0]], w.beta[t[1]], w.beta[t[2]])
	}
	volSymDiff = proofbound.AbsSumUpper(proofbound.ProductUpper(eps, proofbound.AbsSumUpper(stripArea, tag.trueArea)),
		64*0x1p-52*eps*scale)
	return areaSlack, volSymDiff, nil
}

// rewrittenGeom sums the geometric area term the old body published for each
// face the rewrite takes facets from (facetedrecord.FaceAreaGeom).
func (w *work) rewrittenGeom(ctx context.Context, faceIndex map[*topo.Face]int) (float64, error) {
	faces := map[int]*topo.Face{}
	for f, i := range faceIndex {
		faces[i] = f
	}
	var rewritten []int
	seen := map[int]struct{}{}
	for _, t := range w.removed {
		fi := w.fp.FaceOf[t]
		if _, ok := seen[fi]; !ok {
			seen[fi] = struct{}{}
			rewritten = append(rewritten, fi)
		}
	}
	slices.Sort(rewritten)
	budget := proofbound.NewWorkBudget(ctx)
	total := 0.0
	for _, fi := range rewritten {
		f, ok := faces[fi]
		if !ok {
			return 0, fmt.Errorf("%w: a rewritten face is not a face of the body", decaderr.ErrUnsupported)
		}
		g, err := facetedrecord.FaceAreaGeom(budget, f, w.fp.Verts, w.fp.VertexBound, w.fp.Tris,
			facetsOfFace(w.fp.FaceOf, fi))
		if err != nil {
			return 0, err
		}
		total = proofbound.AbsSumUpper(total, g)
	}
	return total, nil
}

func pick(all [][3]int, idx []int) [][3]int {
	out := make([][3]int, len(idx))
	for i, f := range idx {
		out[i] = all[f]
	}
	return out
}

// blendChains blends each closed tangent chain in turn. The edges of a later
// chain are found again on the partly blended body by their end points.
func blendChains(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	refs []refTag, chains []chainCycle, spec blendSpec, op string) (*topo.Body, error) {
	ordOf := map[*topo.Edge]int{}
	for i, e := range edges {
		ordOf[e] = i
	}
	type key struct {
		k   lineKey
		arc bool
	}
	keys := make([][]key, len(chains))
	for ci, c := range chains {
		for _, e := range c.edges {
			_, arc := e.Curve.(surfacegeom.Arc3)
			keys[ci] = append(keys[ci], key{lineKey{e.Start.Position, e.End.Position}, arc})
		}
	}
	cur, curFP := b, fp
	for ci, c := range chains {
		cyc := c
		if ci > 0 {
			cyc = chainCycle{}
			var found []*topo.Edge
			for _, k := range keys[ci] {
				ok := looksLikeLine
				if k.arc {
					ok = looksLikeArc
				}
				e, found1 := findEdge(cur, k.k, 64*ulp*math.Max(1, math.Abs(k.k.p0.X)), ok)
				if !found1 {
					return nil, fmt.Errorf("%w: a chain edge is no longer an edge of the partly blended body",
						decaderr.ErrUnsupported)
				}
				found = append(found, e)
			}
			var err error
			if cyc, _, err = replanChain(cur, found); err != nil {
				return nil, err
			}
		}
		ords := make([]int, len(cyc.edges))
		cref := make([]refTag, len(cyc.edges))
		for i, e := range cyc.edges {
			if ci == 0 {
				ords[i], cref[i] = ordOf[e], refs[ordOf[e]]
				continue
			}
			for _, e0 := range chains[ci].edges {
				if sameEdgeKey(e0, e) {
					ords[i], cref[i] = ordOf[e0], refs[ordOf[e0]]
				}
			}
		}
		next, err := rewriteChain(ctx, env, curFP, cyc, ords, cref, spec, op, faceIndexOf(cur))
		if err != nil {
			return nil, err
		}
		cur = next
		var ok bool
		if curFP, ok = next.Payload.(facetedrecord.Payload); !ok {
			return nil, fmt.Errorf("%w: the blended body lost its payload", decaderr.ErrBooleanFailed)
		}
	}
	return cur, nil
}

// sameEdgeKey reports whether a and b join the same two points, whichever way
// each is directed: replanning a chain on the partly blended body can direct
// an edge opposite to the selected one.
func sameEdgeKey(a, b *topo.Edge) bool {
	return (a.Start.Position == b.Start.Position && a.End.Position == b.End.Position) ||
		(a.Start.Position == b.End.Position && a.End.Position == b.Start.Position)
}

// replanChain orders edges of body that form one chain.
func replanChain(body *topo.Body, edges []*topo.Edge) (chainCycle, bool, error) {
	chains, _, err := planChains(body, edges)
	if err != nil {
		return chainCycle{}, false, err
	}
	if len(chains) != 1 {
		return chainCycle{}, false, fmt.Errorf("%w: a chain changed shape under an earlier blend", decaderr.ErrUnsupported)
	}
	return chains[0], true, nil
}
