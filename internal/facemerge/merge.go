// Package facemerge relabels the source groups of a mesh Boolean result so
// that adjacent, exactly coplanar, same-facing planar faces of the two
// operands become one face (agent-docs/union-face-merge-design.md). It reads
// the stitched result and returns new Src and Groups; it changes no vertex,
// facet or bound.
package facemerge

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshplane"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// Input is a mesh Boolean result's held mesh, source groups and closure-audit adjacency.
type Input struct {
	Verts          []r3.Vec
	VertexBound    []float64
	Tris           [][3]int
	Src            []int
	Groups         []facetedrecord.Group
	Adj            [][]int
	OperandAGroups int
}

// Merge relabels every admitted component (agent-docs/union-face-merge-design.md §4) to one appended group.
// It never modifies in. changed is false when nothing merged; src and groups are then in.Src and in.Groups.
// A length mismatch among Tris, Src, Adj, Verts and VertexBound, an out-of-range Src, vertex or neighbour index,
// or an OperandAGroups outside [0, len(Groups)] is decaderr.ErrBooleanFailed.
func Merge(ctx context.Context, in Input) (src []int, groups []facetedrecord.Group, changed bool, err error) {
	if err := validate(in); err != nil {
		return nil, nil, false, err
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, false, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	candidate, err := candidateGroups(budget, in)
	if err != nil {
		return nil, nil, false, err
	}
	plane, err := facetPlanes(budget, in, candidate)
	if err != nil {
		return nil, nil, false, err
	}
	if plane == nil {
		return in.Src, in.Groups, false, nil
	}
	patch, err := facetproof.SourcePatches(budget, len(in.Tris), in.Src, in.Adj)
	if err != nil {
		return nil, nil, false, err
	}
	comps, err := components(budget, in, candidate, plane, patch)
	if err != nil {
		return nil, nil, false, err
	}
	if len(comps) == 0 {
		return in.Src, in.Groups, false, nil
	}
	src = slices.Clone(in.Src)
	groups = slices.Clone(in.Groups)
	for _, facets := range comps {
		id := len(groups)
		g, err := mergedGroup(budget, in, facets)
		if err != nil {
			return nil, nil, false, err
		}
		groups = append(groups, g)
		for _, t := range facets {
			src[t] = id
		}
	}
	return src, groups, true, nil
}

func failed(format string, args ...any) error {
	return fmt.Errorf("%w: face merge: "+format, append([]any{decaderr.ErrBooleanFailed}, args...)...)
}

// validate checks the shapes Merge indexes with.
func validate(in Input) error {
	n := len(in.Tris)
	if len(in.Src) != n || len(in.Adj) != n || len(in.VertexBound) != len(in.Verts) {
		return failed("input lengths disagree")
	}
	if in.OperandAGroups < 0 || in.OperandAGroups > len(in.Groups) {
		return failed("operand group count %d is outside [0, %d]", in.OperandAGroups, len(in.Groups))
	}
	for t, tri := range in.Tris {
		if s := in.Src[t]; s < 0 || s >= len(in.Groups) {
			return failed("facet %d names source group %d of %d", t, s, len(in.Groups))
		}
		for _, v := range tri {
			if v < 0 || v >= len(in.Verts) {
				return failed("facet %d names vertex %d of %d", t, v, len(in.Verts))
			}
		}
		for _, u := range in.Adj[t] {
			if u < 0 || u >= n {
				return failed("facet %d names neighbour %d of %d", t, u, n)
			}
		}
	}
	return nil
}

// candidateGroups applies design §4.2 rules 1 to 3 and the finite-vertex precondition of meshplane.Groups. A group
// is a candidate when it owns a facet, is planar, has a tag that depends on no extent, every vertex of every facet
// has a finite coordinate and a finite non-negative bound, and at least one facet has bound zero at all three
// corners. That facet is its true piece exactly, so it fixes the true face's plane; rule 4 (facetPlanes) then
// requires every other facet of the group to lie in that plane.
func candidateGroups(budget *proofbound.WorkBudget, in Input) ([]bool, error) {
	owns := make([]bool, len(in.Groups))
	for _, g := range in.Src {
		owns[g] = true
	}
	candidate := make([]bool, len(in.Groups))
	for g, group := range in.Groups {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		candidate[g] = owns[g] && group.Planar && tagIsExtentFree(group)
	}
	anchored := make([]bool, len(in.Groups))
	for t, tri := range in.Tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		g := in.Src[t]
		if !candidate[g] {
			continue
		}
		exact := true
		for _, v := range tri {
			b := in.VertexBound[v]
			// Written so that a NaN bound or coordinate rejects the group.
			if !(b >= 0) || math.IsInf(b, 1) || !finite(in.Verts[v]) {
				candidate[g] = false
				break
			}
			exact = exact && b == 0
		}
		if candidate[g] && exact {
			anchored[g] = true
		}
	}
	for g := range candidate {
		candidate[g] = candidate[g] && anchored[g]
	}
	return candidate, nil
}

func finite(v r3.Vec) bool {
	return !math.IsNaN(v.X+v.Y+v.Z) && !math.IsInf(v.X+v.Y+v.Z, 0)
}

// tagIsExtentFree reports design §4.2 rule 2: no surface, or a retained plane whose validity does not depend on
// the face's extent. A tag that does cannot move to a larger face.
func tagIsExtentFree(g facetedrecord.Group) bool {
	if g.Denoted != nil || g.ExtentDelta != 0 || g.ViewCharge {
		return false
	}
	if g.Surface == nil {
		return true
	}
	return g.Retained && isPlane(g)
}

func isPlane(g facetedrecord.Group) bool {
	_, ok := g.Surface.(surfacegeom.Plane)
	return ok
}

// facetPlanes applies design §4.2 rule 4. It returns each facet's exact oriented plane group (meshplane.Groups over
// the facets of candidate groups only), or nil when no group is a candidate. A candidate group whose facets lie in
// more than one plane group, or in a non-planar one, stops being a candidate.
func facetPlanes(budget *proofbound.WorkBudget, in Input, candidate []bool) ([]int, error) {
	var facets []int
	var verts []r3.Vec
	local := make(map[int]int)
	var tris [][3]int
	for t, tri := range in.Tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if !candidate[in.Src[t]] {
			continue
		}
		var mapped [3]int
		for k, v := range tri {
			i, ok := local[v]
			if !ok {
				i = len(verts)
				local[v] = i
				verts = append(verts, in.Verts[v])
			}
			mapped[k] = i
		}
		facets = append(facets, t)
		tris = append(tris, mapped)
	}
	if len(facets) == 0 {
		return nil, nil
	}
	sub, planes := meshplane.Groups(verts, tris)
	plane := make([]int, len(in.Tris))
	for i := range plane {
		plane[i] = -1
	}
	first := make([]int, len(in.Groups))
	for g := range first {
		first[g] = -1
	}
	for i, t := range facets {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		plane[t] = sub[i]
		g := in.Src[t]
		switch {
		case !planes[sub[i]].Planar:
			candidate[g] = false
		case first[g] == -1:
			first[g] = sub[i]
		case first[g] != sub[i]:
			candidate[g] = false
		}
	}
	return plane, nil
}

// components finds the sets of patches to merge (design §4.3), keeps those whose region is bounded by
// vertex-disjoint simple loops (§4.4), and returns each admitted component's facets in facet order, the
// components ordered by their smallest facet.
func components(budget *proofbound.WorkBudget, in Input, candidate []bool, plane, patch []int) ([][]int, error) {
	nPatch := 0
	for _, p := range patch {
		nPatch = max(nPatch, p+1)
	}
	uf := make(unionFind, nPatch)
	for i := range uf {
		uf[i] = i
	}
	joined := make([]bool, nPatch)
	for t, adj := range in.Adj {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		g := in.Src[t]
		if !candidate[g] {
			continue
		}
		for _, u := range adj {
			h := in.Src[u]
			if u < t || g == h || !candidate[h] || (g < in.OperandAGroups) == (h < in.OperandAGroups) || plane[t] != plane[u] {
				continue
			}
			uf.union(patch[t], patch[u])
			joined[patch[t]], joined[patch[u]] = true, true
		}
	}
	// Component ids follow the smallest facet of each class.
	compOf := make(map[int]int)
	var facets [][]int
	comp := make([]int, len(in.Tris))
	for t := range in.Tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		comp[t] = -1
		if !joined[patch[t]] {
			continue
		}
		root := uf.find(patch[t])
		c, ok := compOf[root]
		if !ok {
			c = len(facets)
			compOf[root] = c
			facets = append(facets, nil)
		}
		comp[t] = c
		facets[c] = append(facets[c], t)
	}
	// boundary counts, per component and vertex, the region-boundary edges that end there.
	boundary := make(map[[2]int]int)
	for t, c := range comp {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if c < 0 {
			continue
		}
		if len(in.Adj[t]) != 3 {
			return nil, failed("facet %d of a merged region has %d neighbours", t, len(in.Adj[t]))
		}
		for k, u := range in.Adj[t] {
			if comp[u] == c {
				continue
			}
			boundary[[2]int{c, in.Tris[t][k]}]++
			boundary[[2]int{c, in.Tris[t][(k+1)%3]}]++
		}
	}
	rejected := make([]bool, len(facets))
	for key, n := range boundary {
		if n != 2 {
			rejected[key[0]] = true
		}
	}
	var admitted [][]int
	for c, f := range facets {
		if !rejected[c] {
			admitted = append(admitted, f)
		}
	}
	return admitted, nil
}

// unionFind is a disjoint-set forest over patch ids.
type unionFind []int

func (u unionFind) find(x int) int {
	for u[x] != x {
		u[x] = u[u[x]]
		x = u[x]
	}
	return x
}

func (u unionFind) union(a, b int) {
	ra, rb := u.find(a), u.find(b)
	if ra != rb {
		u[max(ra, rb)] = min(ra, rb)
	}
}

// mergedGroup builds the group of one admitted component (design §4.6) from the distinct groups of its facets.
func mergedGroup(budget *proofbound.WorkBudget, in Input, facets []int) (facetedrecord.Group, error) {
	seen := make(map[int]struct{})
	var members []int
	for _, t := range facets {
		if err := budget.Step(); err != nil {
			return facetedrecord.Group{}, err
		}
		g := in.Src[t]
		if _, ok := seen[g]; !ok {
			seen[g] = struct{}{}
			members = append(members, g)
		}
	}
	slices.Sort(members)
	merged := facetedrecord.Group{Planar: true}
	var sources []*facetedrecord.SourcePatch
	seenOrigin, seenUpstream := map[provenance.Ref]struct{}{}, map[provenance.Ref]struct{}{}
	donor := -1
	for _, g := range members {
		group := in.Groups[g]
		merged.Origins = appendNew(merged.Origins, seenOrigin, group.Origins)
		merged.Upstream = appendNew(merged.Upstream, seenUpstream, group.Upstream)
		if group.Source != nil {
			sources = append(sources, group.Source)
		}
		if donor < 0 && group.Retained && isPlane(group) {
			donor = g
		}
	}
	if len(sources) > 0 {
		first := sources[0]
		merged.Source = &facetedrecord.SourcePatch{
			Producer: first.Producer, Operand: first.Operand, Ordinal: first.Ordinal, Members: sources,
		}
	}
	if donor >= 0 {
		d := in.Groups[donor]
		merged.Surface, merged.Retained, merged.CapRecord = d.Surface, d.Retained, d.CapRecord
		merged.AxialDelta, merged.HasAxialDelta = d.AxialDelta, d.HasAxialDelta
		merged.NormalBound, merged.Reversed = d.NormalBound, d.Reversed
	}
	return merged, nil
}

// appendNew appends the refs of add that seen does not hold yet, keeping first occurrences in order.
func appendNew(dst []provenance.Ref, seen map[provenance.Ref]struct{}, add []provenance.Ref) []provenance.Ref {
	for _, r := range add {
		if _, ok := seen[r]; !ok {
			seen[r] = struct{}{}
			dst = append(dst, r)
		}
	}
	return dst
}
