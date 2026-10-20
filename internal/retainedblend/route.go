package retainedblend

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Fillet fillets the selected rims of b, a mesh Boolean result with retained
// tags. recognized is false when some selected edge is not a closed circular
// rim between a retained plane and a coaxial retained cylinder; the caller
// then keeps its own refusal. All edges must be non-nil.
func Fillet(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	radius, radiusDelta float64) (*topo.Body, bool, error) {
	return blend(ctx, env, b, fp, edges, blendSpec{fillet: true, r: radius}, radiusDelta, "fillet")
}

// Chamfer bevels the selected rims of b with an equal setback d along the
// plane and the wall. An asymmetric chamfer is not recognized.
func Chamfer(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	d, dDelta float64, asymmetric bool) (*topo.Body, bool, error) {
	if asymmetric {
		return nil, false, nil
	}
	return blend(ctx, env, b, fp, edges, blendSpec{dA: d, dB: d}, dDelta, "chamfer")
}

// edgeKind is the class of a selected edge.
type edgeKind int

const (
	closedRim edgeKind = iota
	straightLine
	cylinderLine
	openArc
)

func blend(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	spec blendSpec, delta float64, op string) (*topo.Body, bool, error) {
	if len(edges) == 0 || !b.Solid {
		return nil, false, nil
	}
	for _, e := range edges {
		if !looksLikeRim(e) && !looksLikeLine(e) && !looksLikeCylLine(e) && !looksLikeArc(e) {
			return nil, false, nil
		}
	}
	if delta != 0 {
		return nil, true, fmt.Errorf("%w: a retained edge blend needs a size that converts exactly to millimetres", decaderr.ErrUnsupported)
	}
	if err := checkFinite(spec.r, spec.dA, spec.dB); err != nil {
		return nil, true, err
	}
	faceIndex := faceIndexOf(b)
	type target struct {
		rimKey  rimKey
		lineKey lineKey
		kind    edgeKind
	}
	targets := make([]target, len(edges))
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
			targets[i] = target{lineKey: l.key(), kind: straightLine}
		case looksLikeArc(e):
			a, err := readArc(e, fp, faceIndex)
			if err != nil {
				// An arc this route cannot read is left to the generic refusal, which
				// names the whole selection.
				return nil, false, nil
			}
			if err := a.checkSize(spec); err != nil {
				return nil, true, err
			}
			if err := claim(a.ends, a.scale); err != nil {
				return nil, true, err
			}
			targets[i] = target{lineKey: a.arcKey(), kind: openArc}
		case looksLikeCylLine(e):
			c, err := readCylLine(e, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if err := claim(c.ends, c.scale); err != nil {
				return nil, true, err
			}
			targets[i] = target{lineKey: c.key(), kind: cylinderLine}
		default:
			r, err := readRim(e, fp, faceIndex)
			if err != nil {
				return nil, true, err
			}
			if err := r.checkSize(spec); err != nil {
				return nil, true, err
			}
			targets[i] = target{rimKey: r.key(), kind: closedRim}
			for _, earlier := range targets[:i] {
				if earlier.kind == closedRim && earlier.rimKey == targets[i].rimKey {
					return nil, true, fmt.Errorf("%w: the selection names one rim twice", decaderr.ErrDegenerate)
				}
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
			case cylinderLine:
				e, ok = findCylLine(cur, targets[i].lineKey, 64*ulp*math.Max(1, math.Abs(targets[i].lineKey.p0.X)))
			default:
				e, ok = findRim(cur, targets[i].rimKey, 64*ulp*math.Max(1, math.Abs(targets[i].rimKey.radius)))
			}
			if !ok {
				return nil, true, fmt.Errorf("%w: a selected edge is no longer an edge of the partly blended body", decaderr.ErrUnsupported)
			}
		}
		faceIndex = faceIndexOf(cur)
		role := fmt.Sprintf("%s(%d)", op, i)
		var next *topo.Body
		var err error
		switch targets[i].kind {
		case straightLine:
			var l line
			if l, err = readLine(e, curFP, faceIndex); err == nil {
				next, err = rewriteLineEdge(ctx, env, curFP, l, spec, role, faceIndex)
			}
		case openArc:
			var a arcRim
			if a, err = readArc(e, curFP, faceIndex); err == nil {
				next, err = rewriteArc(ctx, env, curFP, a, spec, role, faceIndex)
			}
		case cylinderLine:
			var c cylLine
			if c, err = readCylLine(e, curFP, faceIndex); err == nil {
				next, err = rewriteCylLine(ctx, env, curFP, c, spec, role, faceIndex)
			}
		default:
			var r rim
			if r, err = readRim(e, curFP, faceIndex); err == nil {
				next, err = rewriteRim(ctx, env, curFP, r, spec, role, faceIndex)
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
	spec blendSpec, role string, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	if err := r.checkSize(spec); err != nil {
		return nil, err
	}
	w := newWork(fp, r, spec)
	ringA, groupA, err := w.rewritePlane(faceIndex[r.planeFace])
	if err != nil {
		return nil, err
	}
	ringB, groupB, err := w.rewriteWall(faceIndex[r.wallFace])
	if err != nil {
		return nil, err
	}
	tag := w.tag()
	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	blendGroup := len(groups)
	groups = append(groups, facetedrecord.Group{
		Origins:  []provenance.Ref{{Producer: env.Ref, Role: role}},
		Surface:  tag.surface,
		Retained: true,
		Reversed: tag.reversed,
	})
	// The two shrunk faces keep their planes and cylinders but lose the
	// whole-group centroid certificate.
	for _, g := range []int{groupA, groupB} {
		groups[g].HasCentroid = false
		groups[g].Centroid = measurement.VecMeasurement{}
	}
	stripFrom := len(w.out)
	w.strip(tag, ringA, ringB, blendGroup)
	return w.build(ctx, env, groups, stripFrom, tag)
}

// build assembles the rewritten payload and the body over it.
func (w *work) build(ctx context.Context, env topo.Env, groups []facetedrecord.Group, stripFrom int,
	tag blendTag) (*topo.Body, error) {
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
	area, vol := w.measureChange(tag, stripFrom)
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
// (docs/retained-edge-blend-design.md §6.3, Claim K). The area change of the
// held mesh is compared with the exact change: the annulus and band removed
// and the blend face added.
func (w *work) measureChange(tag blendTag, stripFrom int) (areaSlack, volSymDiff float64) {
	oldArea, oldSlop := heldArea(w.fp.Verts, pick(w.fp.Tris, w.removed))
	newArea, newSlop := heldArea(w.verts, w.out)
	stripArea, _ := heldArea(w.verts, w.out[stripFrom:])

	deltaHeld := newArea - oldArea
	deltaTrue := tag.trueArea - w.removedTrue
	// Every term is a float evaluation; charge the areas' own rounding.
	scale := math.Max(math.Abs(deltaHeld), math.Abs(deltaTrue)) + oldArea + newArea + w.removedTrue
	areaSlack = proofbound.AbsSumUpper(math.Abs(deltaHeld-deltaTrue), oldSlop, newSlop, 64*0x1p-52*scale)

	eps := 0.0
	for _, t := range w.out[stripFrom:] {
		eps = max(eps, w.beta[t[0]], w.beta[t[1]], w.beta[t[2]])
	}
	volSymDiff = proofbound.AbsSumUpper(proofbound.ProductUpper(eps, proofbound.AbsSumUpper(stripArea, tag.trueArea)),
		64*0x1p-52*eps*scale)
	return areaSlack, volSymDiff
}

func pick(all [][3]int, idx []int) [][3]int {
	out := make([][3]int, len(idx))
	for i, f := range idx {
		out[i] = all[f]
	}
	return out
}
