package retainedblend

import (
	"cmp"
	"context"
	"fmt"
	"maps"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Fillet fillets the selected edges of b, a mesh Boolean result with retained
// tags. recognized is false when some selected edge is in no class of
// agent-docs/modify-design.md Part E Table RR; the caller then keeps its own
// refusal. All edges must be non-nil.
func Fillet(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	radius, radiusDelta float64) (*topo.Body, bool, error) {
	return FilletTangent(ctx, env, b, fp, edges, radius, radiusDelta, nil)
}

// publishCharged publishes the charged walls of a built body
// (publishChargedWalls) and passes any other result through.
func publishCharged(ctx context.Context, env topo.Env, out *topo.Body, recognized bool,
	err error) (*topo.Body, bool, error) {
	if err != nil || out == nil {
		return out, recognized, err
	}
	out, err = publishChargedWalls(ctx, env, out)
	if err != nil {
		return nil, true, err
	}
	return out, recognized, nil
}

// Reference is a resolved two-distance chamfer (agent-docs/modify-design.md Part M
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
	out, recognized, err := blend(ctx, env, b, fp, edges, spec, dDelta, "chamfer")
	return publishCharged(ctx, env, out, recognized, err)
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
	// chainOnly are the edges no per-edge route recognises, which only a charged chain may read
	// (chainCandidate): planChains decides them, and they never reach a per-edge route.
	var chainOnly []*topo.Edge
	for _, e := range edges {
		if !looksLikeRim(e) && !looksLikeSphereRim(e) && !looksLikeLine(e) && !looksLikeCylLine(e) &&
			!looksLikeArc(e) && !looksLikeCylCyl(e) {
			if !chainCandidate(e) {
				return nil, false, nil
			}
			chainOnly = append(chainOnly, e)
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
	chains, rest, handled, err := planChains(b, edges, spec.tangent, spec.fillet)
	if err != nil {
		return nil, true, err
	}
	if !handled && len(chainOnly) > 0 {
		return nil, false, nil
	}
	ords := make([]int, len(edges))
	for i := range ords {
		ords[i] = i
	}
	var mixed *mixedChains
	if handled {
		refs := make([]refTag, len(edges))
		for i := range edges {
			refs[i] = targets[i].ref
		}
		if len(rest) == 0 {
			out, err := blendChains(ctx, env, b, fp, edges, refs, chains, spec, op, false)
			return out, true, err
		}
		// The other edges take their own routes on b first (chain_mixed.go).
		if slices.ContainsFunc(rest, func(e *topo.Edge) bool { return slices.Contains(chainOnly, e) }) {
			return nil, false, nil
		}
		mixed = &mixedChains{edges: edges, refs: refs, chains: chains}
		sub := make([]target, len(rest))
		ords = make([]int, len(rest))
		for k, e := range rest {
			ords[k] = slices.Index(edges, e)
			sub[k] = targets[ords[k]]
		}
		edges, targets = rest, sub
	}
	if spec.fillet && spec.asym == nil && len(edges) >= 2 {
		// Straight convex edges that meet end to end on one face, at corners of three perpendicular faces, are
		// filleted as one run with a mitre at each joint (line_mitre.go).
		allLines := true
		for _, e := range edges {
			allLines = allLines && looksLikeLine(e)
		}
		if allLines {
			run, ok, err := planMitre(edges, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if ok {
				out, err := rewriteMitreRun(ctx, env, fp, run, spec, op, faceIndex)
				return out, true, err
			}
		}
	}
	if spec.fillet && spec.asym == nil && len(edges) >= 2 {
		// Straight convex edges of one radius that meet at perpendicular corners are filleted together, each corner by
		// the number of edges at it (corner_fillet.go).
		allLines := true
		for _, e := range edges {
			allLines = allLines && looksLikeLine(e)
		}
		if allLines {
			plan, ok, err := planCorners(edges, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if ok {
				ords := make([]int, len(edges))
				for i := range ords {
					ords[i] = i
				}
				out, err := rewriteCorners(ctx, env, fp, plan, spec, op, ords, faceIndex)
				return out, true, err
			}
		}
	}
	// Several arcs of one wall each read their wall by its facets along the arc
	// (readArc's local) only when every selected edge is an open arc; any other
	// selection keeps the single-edge rule.
	local := len(edges) == 1 || !slices.ContainsFunc(edges, func(e *topo.Edge) bool { return !looksLikeArc(e) })
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
			if err := refuseChargedArcProven(e, allEdges(b, spec.tangent), spec.tangent); err != nil {
				return nil, true, err
			}
			a, err := readArcTangent(e, fp, faceIndex, local, newTangentRead(spec.tangent, b, e))
			if err != nil {
				if isTangentRefusal(err) {
					return nil, true, err
				}
				// An arc this route cannot read is left to the generic refusal, which
				// names the whole selection.
				return nil, false, nil
			}
			// Several arcs share a two-sided wall only where their ends are cylinders; plane-ended arcs
			// keep the whole-face refusal when more than one edge is selected.
			if len(edges) > 1 && a.twoSided && a.cyl[0] == nil {
				return nil, false, nil
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
		ord := ords[i]
		role := fmt.Sprintf("%s(%d)", op, ord)
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
				next, err = rewriteLineEdge(ctx, env, curFP, l, step, role, ord, faceIndex)
			}
		case openArc:
			var a arcRim
			if a, err = readArcTangent(e, curFP, faceIndex, local, newTangentRead(spec.tangent, cur, e)); err == nil {
				step, err = spec.assign(targets[i].ref, a.planeFace, a.wallFace)
			}
			if err == nil {
				next, err = rewriteArc(ctx, env, curFP, a, step, role, ord, faceIndex)
			}
		case cylinderPair:
			var c cylCyl
			if c, err = readCylCyl(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, c.wall[0].face, c.wall[1].face)
			}
			if err == nil {
				next, err = rewriteCylCyl(ctx, env, curFP, c, step, role, ord, faceIndex)
			}
		case cylinderLine:
			var c cylLine
			if c, err = readCylLine(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, c.faceA, c.faceB)
			}
			if err == nil {
				next, err = rewriteCylLine(ctx, env, curFP, c, step, role, ord, faceIndex)
			}
		case sphereRimKind:
			var s sphereRim
			if s, err = readSphereRim(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, s.planeFace, s.wallFace)
			}
			if err == nil {
				next, err = rewriteSphereRim(ctx, env, curFP, s, step, role, ord, faceIndex)
			}
		default:
			var r rim
			if r, err = readRim(e, curFP, faceIndex); err == nil {
				step, err = spec.assign(targets[i].ref, r.planeFace, r.wallFace)
			}
			if err == nil {
				next, err = rewriteRim(ctx, env, curFP, r, step, role, ord, faceIndex)
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
	if mixed != nil {
		out, err := blendChains(ctx, env, cur, curFP, mixed.edges, mixed.refs, mixed.chains, spec, op, true)
		return out, true, err
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
	ringA, groupA, err := w.rewritePlane(faceIndex[r.planeFace], planeTag, spec.planeSetback(),
		max(r.radius, r.radius+r.kappa*spec.planeSetback()))
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
	groups = append(groups, w.tagGroup(facetedrecord.Group{
		Origins: []provenance.Ref{{Producer: env.Ref, Role: role}},
		Source:  facetedrecord.BlendSource(env.Ref, ord),
	}, tag))
	// The two shrunk faces keep their surfaces but lose the whole-group
	// centroid certificate.
	for _, g := range shrunk {
		groups[g].HasCentroid = false
		groups[g].Centroid = measurement.VecMeasurement{}
	}
	stripFrom := len(w.out)
	w.strip(tag, ringA, ringB, blendGroup)
	return w.buildClear(ctx, env, groups, stripFrom, tag, faceIndex)
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
	facets, src = w.splitSeams(facets, src)

	verts, beta, facets := compact(w.verts, w.beta, facets)
	meshBound := facetproof.FacetBoundMax(facets, beta)
	area, vol := w.measureChange(tag, stripFrom)
	oldGeom, err := groupGeom(ctx, faceIndex, fp, true)
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
	return buildCharged(ctx, env, next, oldGeom)
}

// splitSeams (src may be nil) splits every facet that keeps a held edge a band cut made a vertex
// on along an end plane (w.seam), of any face and whether the cut emitted it
// unchanged or the mesh kept it: the facet is fanned from its opposite corner
// through the vertices on the edge, in order along it. The vertex lies on the
// held edge within its bound, so the facet's region and claims do not change;
// without the split the facet meets the strip across a T-junction.
func (w *work) splitSeams(facets [][3]int, src []int) ([][3]int, []int) {
	if len(w.seam) == 0 {
		return facets, src
	}
	var outF [][3]int
	var outS []int
	var split func(t [3]int, s int)
	split = func(t [3]int, s int) {
		for k := range 3 {
			a, b, c := t[k], t[(k+1)%3], t[(k+2)%3]
			ids := w.seam[[2]int{min(a, b), max(a, b)}]
			if len(ids) == 0 {
				continue
			}
			dir := w.verts[b].Sub(w.verts[a])
			pts := slices.Clone(ids)
			slices.SortFunc(pts, func(x, y int) int {
				return cmp.Compare(w.verts[x].Sub(w.verts[a]).Dot(dir), w.verts[y].Sub(w.verts[a]).Dot(dir))
			})
			pts = append(append([]int{a}, pts...), b)
			for j := range len(pts) - 1 {
				split([3]int{pts[j], pts[j+1], c}, s)
			}
			return
		}
		outF, outS = append(outF, t), append(outS, s)
	}
	for i, t := range facets {
		s := 0
		if src != nil {
			s = src[i]
		}
		split(t, s)
	}
	if src == nil {
		outS = nil
	}
	return outF, outS
}

// buildCharged builds the rewritten body from next and charges the shortfall
// of its geometric area terms against oldGeom, the old body's terms by group
// (agent-docs/modify-design.md Part E §6.3, Area). next.AreaSlack holds
// the old slack and the change's own terms; the shortfall joins it and every
// area bound of the built body (facetedrecord.ChargeAreaSlack).
func buildCharged(ctx context.Context, env topo.Env, next facetedrecord.Payload,
	oldGeom map[int]float64) (*topo.Body, error) {
	body, err := facetedrecord.Build(ctx, env, next)
	if err != nil {
		return nil, fmt.Errorf("%w: the blended mesh fails its held audit: %w", decaderr.ErrUnsupported, err)
	}
	built, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, fmt.Errorf("%w: the blended body lost its payload", decaderr.ErrBooleanFailed)
	}
	newGeom, err := groupGeom(ctx, faceIndexOf(body), built, false)
	if err != nil {
		return nil, err
	}
	if err := facetedrecord.ChargeAreaSlack(body, geomShortfall(oldGeom, newGeom)); err != nil {
		return nil, err
	}
	return body, nil
}

// geomShortfall is an upper bound on the sum, over the groups of oldGeom, of
// how far each group's old terms exceed its new ones. oldGeom holds upper
// bounds and newGeom lower bounds of the exact sums, so each positive
// difference rounded up covers the exact one; a group the new body lacks
// reads zero. A difference that is not a number, as of two infinite terms,
// bounds nothing and charges +Inf, which the build refuses.
func geomShortfall(oldGeom, newGeom map[int]float64) float64 {
	total := 0.0
	for _, g := range slices.Sorted(maps.Keys(oldGeom)) {
		d := oldGeom[g] - newGeom[g]
		if math.IsNaN(d) {
			return math.Inf(1)
		}
		if d > 0 {
			total = proofbound.AbsSumUpper(total, proofbound.UpRound(d))
		}
	}
	return total
}

// groupGeom sums the geometric area term AssembleBody charged each face of a
// body (facetedrecord.FaceAreaGeom), keyed by the source group of the face's
// first facet. faces numbers the body's faces as fp.FaceOf does. With up each
// sum is rounded up, else down, so it bounds the exact sum of the terms from
// that side.
func groupGeom(ctx context.Context, faces map[*topo.Face]int, fp facetedrecord.Payload,
	up bool) (map[int]float64, error) {
	byIndex := make(map[int]*topo.Face, len(faces))
	for f, i := range faces {
		byIndex[i] = f
	}
	own := map[int][]int{}
	for t, fi := range fp.FaceOf {
		own[fi] = append(own[fi], t)
	}
	budget := proofbound.NewWorkBudget(ctx)
	terms := map[int][]float64{}
	for _, fi := range slices.Sorted(maps.Keys(own)) {
		f, ok := byIndex[fi]
		if !ok {
			return nil, fmt.Errorf("%w: a facet's face is not a face of the body", decaderr.ErrUnsupported)
		}
		g, err := facetedrecord.FaceAreaGeom(budget, f, fp.Verts, fp.VertexBound, fp.Tris, own[fi])
		if err != nil {
			return nil, err
		}
		key := fp.Src[own[fi][0]]
		terms[key] = append(terms[key], g)
	}
	out := make(map[int]float64, len(terms))
	for key, ts := range terms {
		if up {
			out[key] = proofbound.AbsSumUpper(ts...)
			continue
		}
		out[key] = sumDown(ts)
	}
	return out, nil
}

// sumDown is a lower bound on the exact sum of non-negative terms: each
// partial sum is rounded and then stepped down, so it never passes the exact
// one, and the exact sum is at least zero.
func sumDown(terms []float64) float64 {
	total := 0.0
	for _, t := range terms {
		total = max(0, proofarith.NextDown(total+t))
	}
	return total
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
// (agent-docs/modify-design.md Part E §6.3, Claim K). The area change of the
// held mesh is compared with the exact change: the annulus and band removed
// and the blend face added, widened by the pieces a displaced plane moves
// (planeAreaCharge). The volume bound adds the slab each re-triangulated plane
// face's facets can span over its true plane (planeFacetVolume). The shortfall
// of the rebuilt faces' geometric terms is charged after the build
// (buildCharged).
func (w *work) measureChange(tag blendTag, stripFrom int) (float64, float64) {
	oldArea, oldSlop := heldArea(w.fp.Verts, pick(w.fp.Tris, w.removed))
	newArea, newSlop := heldArea(w.verts, w.out)
	stripArea, _ := heldArea(w.verts, w.out[stripFrom:])

	deltaHeld := newArea - oldArea
	deltaTrue := tag.trueArea - w.removedTrue
	// Every term is a float evaluation; charge the areas' own rounding.
	scale := math.Max(math.Abs(deltaHeld), math.Abs(deltaTrue)) + oldArea + newArea + w.removedTrue
	areaSlack := proofbound.AbsSumUpper(math.Abs(deltaHeld-deltaTrue), oldSlop, newSlop, 64*0x1p-52*scale)
	// A charged wall moves the exact area change (rimAreaCharge); the charge is
	// zero on every route whose wall states no displacement.
	trueArea := tag.trueArea
	charge := rimAreaCharge(w.r, w.spec)
	if w.arcCharged {
		charge = w.arcCharge
	}
	if charge != 0 {
		areaSlack = proofbound.AbsSumUpper(areaSlack, charge)
		trueArea = proofbound.AbsSumUpper(tag.trueArea, charge)
	}
	// A displaced plane moves the exact pieces' boundaries (plane_charge.go); the term is zero without one.
	if planeArea := planeAreaCharge(w.planePieces, w.r.charge); planeArea != 0 {
		areaSlack = proofbound.AbsSumUpper(areaSlack, planeArea)
		trueArea = proofbound.AbsSumUpper(trueArea, planeArea)
	}
	// An oblique end plane's true cap is tilted from its tag (obliqueCapArea); zero without one.
	if w.tiltArea != 0 {
		areaSlack = proofbound.AbsSumUpper(areaSlack, w.tiltArea)
		trueArea = proofbound.AbsSumUpper(trueArea, w.tiltArea)
	}

	// A raised inner vertex of a curved wall bounds the facets beside the
	// strip that now face the blend.
	eps := w.raised
	for _, t := range w.out[stripFrom:] {
		eps = max(eps, w.beta[t[0]], w.beta[t[1]], w.beta[t[2]])
	}
	volSymDiff := proofbound.AbsSumUpper(proofbound.ProductUpper(eps, proofbound.AbsSumUpper(stripArea, trueArea)),
		64*0x1p-52*eps*scale)
	// An end cap on a cylinder is new surface bounding K, and the split of G's held
	// facets moves their volume (arc_end_cyl_cap.go).
	if w.capEps > 0 || w.splitVol > 0 {
		capVol := proofbound.ProductUpper(w.capEps, proofbound.AbsSumUpper(w.capHeld, w.capTrue))
		volSymDiff = proofbound.AbsSumUpper(volSymDiff, capVol, w.splitVol)
	}
	// The facets of every re-triangulated plane face lie off the true plane by their height over the tag plane
	// and the face's stated displacement (Claim K, plane_charge.go).
	volSymDiff = proofbound.AbsSumUpper(volSymDiff, w.planeVol)
	return areaSlack, volSymDiff
}

func pick(all [][3]int, idx []int) [][3]int {
	out := make([][3]int, len(idx))
	for i, f := range idx {
		out[i] = all[f]
	}
	return out
}

// blendChains blends each closed tangent chain in turn. The edges of a later
// chain are found again on the partly blended body by their end points, and with
// refind so are the first chain's: b is then the body the selection's other edges
// have already blended (chain_mixed.go), and edges, refs and chains are those of the
// body they were planned on.
func blendChains(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	refs []refTag, chains []chainCycle, spec blendSpec, op string, refind bool) (*topo.Body, error) {
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
		again := ci > 0 || refind
		if again {
			cyc = chainCycle{}
			var found []*topo.Edge
			for _, k := range keys[ci] {
				e, found1 := findEdge(cur, k.k, 64*ulp*math.Max(1, math.Abs(k.k.p0.X)), chainEdgeFinder(k.arc))
				if !found1 {
					return nil, fmt.Errorf("%w: a chain edge is no longer an edge of the partly blended body",
						decaderr.ErrUnsupported)
				}
				found = append(found, e)
			}
			var err error
			if cyc, _, err = replanChain(cur, found, spec.fillet); err != nil {
				return nil, err
			}
		}
		ords := make([]int, len(cyc.edges))
		cref := make([]refTag, len(cyc.edges))
		for i, e := range cyc.edges {
			if !again {
				ords[i], cref[i] = ordOf[e], refs[ordOf[e]]
				continue
			}
			matched := false
			for _, e0 := range chains[ci].edges {
				if sameEdgeKey(e0, e) {
					ords[i], cref[i] = ordOf[e0], refs[ordOf[e0]]
					matched = true
				}
			}
			if !matched {
				return nil, fmt.Errorf("%w: a chain edge's selection ordinal is not found", decaderr.ErrUnsupported)
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
func replanChain(body *topo.Body, edges []*topo.Edge, fillet bool) (chainCycle, bool, error) {
	chains, rest, _, err := planChains(body, edges, nil, fillet)
	if err != nil {
		return chainCycle{}, false, err
	}
	if len(chains) != 1 || len(rest) != 0 {
		return chainCycle{}, false, fmt.Errorf("%w: a chain changed shape under an earlier blend", decaderr.ErrUnsupported)
	}
	return chains[0], true, nil
}
