package trimrebuild

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// NewFace is one face the operation adds: its group (Surface, Retained, Reversed, Planar, NormalBound, Origins,
// Source) and its facets. Tris index the receiver's fp.Verts followed by Splice.Verts.
type NewFace struct {
	Group facetedrecord.Group
	Tris  [][3]int
}

// Splice is the rebuilt held mesh: the receiver's kept facets plus the new faces.
type Splice struct {
	Keep   []int     // receiver facet indices kept unchanged, with their groups
	Verts  []r3.Vec  // new vertices; index len(fp.Verts)+i in NewFace.Tris
	Bounds []float64 // their bounds
	Faces  []NewFace
}

// Build assembles the payload, composes its bounds, runs the held-embedding gate over every facet that touches a
// new vertex, and builds the body. A failed gate or held audit is ErrUnsupported (TG5, TG8).
func Build(ctx context.Context, env topo.Env, fp facetedrecord.Payload, s Splice) (*topo.Body, error) {
	if len(s.Verts) != len(s.Bounds) {
		return nil, fmt.Errorf("%w: a splice states %d vertices and %d bounds", decaderr.ErrUnsupported,
			len(s.Verts), len(s.Bounds))
	}
	// A splice that drops receiver facets charges AreaSlack the shortfall of the rebuilt faces' geometric area
	// terms against the receiver's by source group (agent-docs/retained-edge-blend-design.md §6.3, Area). That
	// needs the receiver's faces, which Build does not take, so a splice that drops a facet is refused.
	if len(s.Keep) != len(fp.Tris) {
		return nil, fmt.Errorf("%w: a trim-graph splice must keep every receiver facet", decaderr.ErrUnsupported)
	}
	verts := append(append([]r3.Vec(nil), fp.Verts...), s.Verts...)
	beta := append(append([]float64(nil), fp.VertexBound...), s.Bounds...)
	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	var tris [][3]int
	var src []int
	for _, k := range s.Keep {
		tris = append(tris, fp.Tris[k])
		src = append(src, fp.Src[k])
	}
	newFrom := len(tris)
	for _, nf := range s.Faces {
		g := len(groups)
		groups = append(groups, nf.Group)
		for _, t := range nf.Tris {
			tris = append(tris, t)
			src = append(src, g)
		}
	}
	for _, t := range tris {
		for _, v := range t {
			if v < 0 || v >= len(verts) {
				return nil, fmt.Errorf("%w: a splice facet names vertex %d of %d", decaderr.ErrUnsupported, v, len(verts))
			}
		}
	}
	verts, beta, tris, moved := compact(verts, beta, tris, len(fp.Verts))
	newTris := tris[newFrom:]

	if err := checkEmbedding(ctx, verts, tris, moved); err != nil {
		return nil, err
	}

	meshBound := facetproof.FacetBoundMax(tris, beta)
	betaNew := 0.0
	for _, t := range newTris {
		betaNew = max(betaNew, beta[t[0]], beta[t[1]], beta[t[2]])
	}
	perturbed, err := proofbound.PerturbedAreaUpperContext(ctx, verts, newTris, betaNew)
	if err != nil {
		return nil, err
	}
	held, slop := heldArea(verts, newTris)
	extra := math.Max(0, perturbed-held)
	next := facetedrecord.Payload{
		Verts: verts, VertexBound: beta, Tris: tris, Src: src, Groups: groups,
		MeshBound:   meshBound,
		VolSymDiff:  proofbound.AbsSumUpper(fp.VolSymDiff, proofbound.SweptVolumeAllow(betaNew, perturbed)),
		AreaSlack:   proofbound.AbsSumUpper(fp.AreaSlack, proofbound.UpRound(extra), slop),
		DPair:       facetedrecord.SourceDiameterUpper(verts, meshBound),
		Xform:       fp.Xform,
		ExactPlaced: fp.ExactPlaced,
	}
	body, err := facetedrecord.Build(ctx, env, next)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf("%w: the rebuilt mesh fails its held audit (trim TG8): %w", decaderr.ErrUnsupported, err)
	}
	return body, nil
}

// checkEmbedding is the held-embedding gate (TG5): the whole new mesh, with every new vertex Moved.
func checkEmbedding(ctx context.Context, verts []r3.Vec, tris [][3]int, moved []bool) error {
	h := meshbool.HeldRounding{
		Verts: verts, Tris: tris,
		Exact: make([]proof.Xpt, len(verts)), Moved: moved, Movable: make([]bool, len(verts)),
	}
	for i, v := range verts {
		h.Exact[i] = proof.XptOf(v)
	}
	folded, err := meshbool.HasHeldFold(ctx, h)
	if err != nil {
		return err
	}
	if !folded {
		return nil
	}
	folds, err := meshbool.HeldFolds(ctx, h)
	if err != nil {
		return err
	}
	if len(folds) == 0 {
		return fmt.Errorf("%w: the rebuilt held mesh folds (trim TG5)", decaderr.ErrUnsupported)
	}
	return fmt.Errorf("%w: the rebuilt held mesh folds at facets %d and %d (trim TG5)", decaderr.ErrUnsupported,
		folds[0][0], folds[0][1])
}

// compact drops the vertices no facet uses and renumbers the rest. moved marks the vertices whose original index is
// at least firstNew.
func compact(verts []r3.Vec, beta []float64, facets [][3]int, firstNew int) ([]r3.Vec, []float64, [][3]int, []bool) {
	remap := make([]int, len(verts))
	for i := range remap {
		remap[i] = -1
	}
	var outV []r3.Vec
	var outB []float64
	var moved []bool
	out := make([][3]int, len(facets))
	for i, t := range facets {
		for k, v := range t {
			if remap[v] < 0 {
				remap[v] = len(outV)
				outV = append(outV, verts[v])
				outB = append(outB, beta[v])
				moved = append(moved, v >= firstNew)
			}
			out[i][k] = remap[v]
		}
	}
	return outV, outB, out, moved
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
