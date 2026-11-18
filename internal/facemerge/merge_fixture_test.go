package facemerge_test

import (
	"fmt"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facemerge"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// meshBuilder collects a closed mesh facet by facet; vertices are shared by exact position.
type meshBuilder struct {
	verts []r3.Vec
	index map[r3.Vec]int
	tris  [][3]int
	src   []int
}

func newMeshBuilder() *meshBuilder {
	return &meshBuilder{index: map[r3.Vec]int{}}
}

func (b *meshBuilder) vertex(p r3.Vec) int {
	if i, ok := b.index[p]; ok {
		return i
	}
	b.verts = append(b.verts, p)
	b.index[p] = len(b.verts) - 1
	return len(b.verts) - 1
}

func (b *meshBuilder) tri(group int, p, q, r r3.Vec) {
	b.tris = append(b.tris, [3]int{b.vertex(p), b.vertex(q), b.vertex(r)})
	b.src = append(b.src, group)
}

// gridFace adds an nu by nv grid of unit cells spanned by du and dv from origin; du × dv is the outward normal.
// group picks each cell's source group. A cell for which fan is true is cut into four facets around its centre.
func (b *meshBuilder) gridFace(origin, du, dv r3.Vec, nu, nv int, group func(i, j int) int, fan func(i, j int) bool) {
	at := func(i, j int) r3.Vec {
		return origin.Add(du.Scale(float64(i))).Add(dv.Scale(float64(j)))
	}
	for i := range nu {
		for j := range nv {
			g := group(i, j)
			p00, p10, p11, p01 := at(i, j), at(i+1, j), at(i+1, j+1), at(i, j+1)
			if fan != nil && fan(i, j) {
				c := origin.Add(du.Scale(float64(i) + 0.5)).Add(dv.Scale(float64(j) + 0.5))
				b.tri(g, p00, p10, c)
				b.tri(g, p10, p11, c)
				b.tri(g, p11, p01, c)
				b.tri(g, p01, p00, c)
				continue
			}
			b.tri(g, p00, p10, p11)
			b.tri(g, p00, p11, p01)
		}
	}
}

func one(g int) func(int, int) int { return func(int, int) int { return g } }

// splitBox is the base fixture: a 2×1×1 box whose four faces parallel to x are split at x = 1. The groups are
// A: 0 (x = 0), 1 bottom, 2 top, 3 front, 4 back; B: 5 (x = 2), 6 bottom, 7 top, 8 front, 9 back. perm renames them.
const (
	gX0, gBotA, gTopA, gFrontA, gBackA = 0, 1, 2, 3, 4
	gX2, gBotB, gTopB, gFrontB, gBackB = 5, 6, 7, 8, 9
	splitBoxA                          = 5
)

func splitBox(perm []int, fan func(i, j int) bool) *meshBuilder {
	p := func(g int) int {
		if perm == nil {
			return g
		}
		return perm[g]
	}
	// halves names the group of cell k along the split axis.
	halves := func(a, b int) func(k int) int {
		return func(k int) int {
			if k == 0 {
				return p(a)
			}
			return p(b)
		}
	}
	v := r3.NewVec
	m := newMeshBuilder()
	bottom, top, front, back := halves(gBotA, gBotB), halves(gTopA, gTopB), halves(gFrontA, gFrontB), halves(gBackA, gBackB)
	m.gridFace(v(0, 0, 0), v(0, 1, 0), v(1, 0, 0), 1, 2, func(_, j int) int { return bottom(j) }, nil)
	m.gridFace(v(0, 0, 1), v(1, 0, 0), v(0, 1, 0), 2, 1, func(i, _ int) int { return top(i) }, fan)
	m.gridFace(v(0, 0, 0), v(1, 0, 0), v(0, 0, 1), 2, 1, func(i, _ int) int { return front(i) }, nil)
	m.gridFace(v(0, 1, 0), v(0, 0, 1), v(1, 0, 0), 1, 2, func(_, j int) int { return back(j) }, nil)
	m.gridFace(v(0, 0, 0), v(0, 0, 1), v(0, 1, 0), 1, 1, one(p(gX0)), nil)
	m.gridFace(v(2, 0, 0), v(0, 1, 0), v(0, 0, 1), 1, 1, one(p(gX2)), nil)
	return m
}

// plainGroups returns one planar group per id, with a distinct origin role each.
func plainGroups(n int) []facetedrecord.Group {
	groups := make([]facetedrecord.Group, n)
	for g := range groups {
		groups[g] = facetedrecord.Group{
			Origins: []provenance.Ref{{Producer: 1, Role: fmt.Sprintf("face(%d)", g)}},
			Planar:  true,
		}
	}
	return groups
}

// input audits the mesh and returns the Merge input with every vertex bound zero.
func (b *meshBuilder) input(t *testing.T, groups []facetedrecord.Group, operandA int) facemerge.Input {
	t.Helper()
	audit, err := facetproof.AuditFacetedMesh(t.Context(), b.verts, b.tris)
	require.NoError(t, err)
	return facemerge.Input{
		Verts: b.verts, VertexBound: make([]float64, len(b.verts)), Tris: b.tris, Src: b.src,
		Groups: groups, Adj: audit.Adj, OperandAGroups: operandA,
	}
}

// assemble builds the body of the result and returns each facet's face index and the face count.
func assemble(t *testing.T, in facemerge.Input, src []int, groups []facetedrecord.Group) (*topo.Body, []int) {
	t.Helper()
	m := facetedrecord.Mesh{Verts: in.Verts, VertexBound: in.VertexBound, Tris: in.Tris, Src: src, Groups: groups}
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	body, faceOf, _, err := facetedrecord.AssembleBody(t.Context(), env, m, nil, measurement.Measurement{}, nil)
	require.NoError(t, err)
	return body, faceOf
}

// retainedPlane tags the group with a retained plane at height z, as a Boolean copies an operand's plane tag.
func retainedPlane(t *testing.T, g facetedrecord.Group, z float64) facetedrecord.Group {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, z), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	g.Surface = surfacegeom.Plane{Frame: frame}
	g.Retained = true
	return g
}

// markerGroup is a retained plane with an extent-dependent tag, which design §4.2 rule 2 never merges.
func markerGroup(t *testing.T, g facetedrecord.Group) facetedrecord.Group {
	t.Helper()
	g = retainedPlane(t, g, 1)
	g.ExtentDelta = 1e-6
	return g
}

// pairs lists the face pairs of splitBox that can merge, A group then B group.
var splitPairs = [4][2]int{{gBotA, gBotB}, {gTopA, gTopB}, {gFrontA, gFrontB}, {gBackA, gBackB}}

// facetsOf returns the facets of src labelled with any of the given groups.
func facetsOf(src []int, groups ...int) []int {
	var out []int
	for t, g := range src {
		if slices.Contains(groups, g) {
			out = append(out, t)
		}
	}
	return out
}
