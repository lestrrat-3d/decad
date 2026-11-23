package facemerge_test

import (
	"context"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facemerge"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireMergedPairs checks that exactly the listed pairs of splitBox merged: each listed pair's facets share one
// appended group, distinct per pair, and every other facet keeps its group.
func requireMergedPairs(t *testing.T, in facemerge.Input, src []int, groups []facetedrecord.Group, merged ...[2]int) {
	t.Helper()
	require.Len(t, groups, len(in.Groups)+len(merged))
	require.Len(t, src, len(in.Src))
	inPair := map[int]struct{}{}
	seen := map[int]struct{}{}
	for _, pair := range merged {
		facets := facetsOf(in.Src, pair[0], pair[1])
		id := src[facets[0]]
		require.GreaterOrEqual(t, id, len(in.Groups), "pair %v keeps an original group", pair)
		for _, f := range facets {
			require.Equal(t, id, src[f], "pair %v", pair)
			inPair[f] = struct{}{}
		}
		_, dup := seen[id]
		require.False(t, dup, "pair %v shares a group with another pair", pair)
		seen[id] = struct{}{}
	}
	for f := range in.Src {
		if _, ok := inPair[f]; !ok {
			require.Equal(t, in.Src[f], src[f], "facet %d", f)
		}
	}
}

func TestMergeSplitBox(t *testing.T) {
	t.Parallel()
	in := splitBox(nil, nil).input(t, plainGroups(10), splitBoxA)
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	requireMergedPairs(t, in, src, groups, splitPairs[:]...)

	body, _ := assemble(t, in, src, groups)
	require.Len(t, topobuild.Faces(body), 6)
	require.Len(t, topobuild.Edges(body), 12)
	require.Len(t, topobuild.Vertices(body), 8)

	// The unmerged mesh holds the four seams as separate faces.
	split, _ := assemble(t, in, in.Src, in.Groups)
	require.Len(t, topobuild.Faces(split), 10)
}

func TestMergeSameOperand(t *testing.T) {
	t.Parallel()
	in := splitBox(nil, nil).input(t, plainGroups(10), 10)
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.False(t, changed)
	require.Same(t, &in.Src[0], &src[0])
	require.Same(t, &in.Groups[0], &groups[0])

	in.OperandAGroups = 0
	_, _, changed, err = facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.False(t, changed, "every group in operand B")
}

func TestMergePositiveBound(t *testing.T) {
	t.Parallel()
	m := splitBox(nil, func(i, _ int) bool { return i == 0 })
	in := m.input(t, plainGroups(10), splitBoxA)
	in.VertexBound[m.index[r3.NewVec(0.5, 0.5, 1)]] = 1e-15
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	requireMergedPairs(t, in, src, groups, splitPairs[0], splitPairs[2], splitPairs[3])
}

func TestMergeBoundedCorner(t *testing.T) {
	t.Parallel()
	// The box corner (0, 1, 1) is in one facet of the top's A half and one of the back's A half; each half keeps a
	// facet whose corners are all exact, which fixes its plane, so every pair merges. The x = 0 face, whose two
	// facets both hold the corner, has no partner and keeps its group.
	m := splitBox(nil, nil)
	in := m.input(t, plainGroups(10), splitBoxA)
	in.VertexBound[m.index[r3.NewVec(0, 1, 1)]] = 1e-15
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	requireMergedPairs(t, in, src, groups, splitPairs[:]...)
	body, _ := assemble(t, in, src, groups)
	require.Len(t, topobuild.Faces(body), 6)

	// The corner (2, 1, 1) is in every facet of the top's and the back's B halves, which then fix no plane.
	in = m.input(t, plainGroups(10), splitBoxA)
	in.VertexBound[m.index[r3.NewVec(2, 1, 1)]] = 1e-15
	src, groups, changed, err = facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	requireMergedPairs(t, in, src, groups, splitPairs[0], splitPairs[2])

	// A NaN or infinite bound rejects its group.
	for _, bad := range []float64{math.NaN(), math.Inf(1), -1} {
		in = m.input(t, plainGroups(10), splitBoxA)
		in.VertexBound[m.index[r3.NewVec(0, 1, 1)]] = bad
		src, groups, changed, err = facemerge.Merge(t.Context(), in)
		require.NoError(t, err)
		require.True(t, changed)
		requireMergedPairs(t, in, src, groups, splitPairs[0], splitPairs[2])
	}
}

func TestMergeTagAnchor(t *testing.T) {
	t.Parallel()
	// Every vertex carries a bound, so no facet anchors a plane; only the top groups' exact retained tags can.
	bounded := func(top func(facetedrecord.Group) facetedrecord.Group) facemerge.Input {
		m := splitBox(nil, nil)
		groups := plainGroups(10)
		groups[gTopA], groups[gTopB] = top(groups[gTopA]), top(groups[gTopB])
		in := m.input(t, groups, splitBoxA)
		for v := range in.VertexBound {
			in.VertexBound[v] = 1e-6
		}
		return in
	}
	in := bounded(func(g facetedrecord.Group) facetedrecord.Group { return retainedPlane(t, g, 1) })
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	requireMergedPairs(t, in, src, groups, splitPairs[1])
	body, _ := assemble(t, in, src, groups)
	require.Len(t, topobuild.Faces(body), 9)

	// A tag that states a normal departure, or one an ulp-scale step off the facets' plane, anchors nothing.
	for name, top := range map[string]func(facetedrecord.Group) facetedrecord.Group{
		"departure": func(g facetedrecord.Group) facetedrecord.Group {
			g = retainedPlane(t, g, 1)
			g.NormalBound = 1e-12
			return g
		},
		"off plane": func(g facetedrecord.Group) facetedrecord.Group { return retainedPlane(t, g, 1+math.Ldexp(1, -40)) },
		"displaced": func(g facetedrecord.Group) facetedrecord.Group {
			g = retainedPlane(t, g, 1)
			g.AxialDelta, g.HasAxialDelta = 1e-9, true
			return g
		},
	} {
		in := bounded(top)
		_, _, changed, err := facemerge.Merge(t.Context(), in)
		require.NoError(t, err, name)
		require.False(t, changed, name)
	}
}

func TestMergeOffsetPlane(t *testing.T) {
	t.Parallel()
	m := splitBox(nil, nil)
	raise := math.Ldexp(1, -40)
	for _, y := range []float64{0, 1} {
		m.verts[m.index[r3.NewVec(2, y, 1)]] = r3.NewVec(2, y, 1+raise)
	}
	in := m.input(t, plainGroups(10), splitBoxA)
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	requireMergedPairs(t, in, src, groups, splitPairs[0], splitPairs[2], splitPairs[3])
}

// newCellBox is the input of a 3×3×1 box whose top is a 3×3 grid of cells, one source group each. Cells whose
// index (i+j) is even belong to operand A, the others to B; marker cells keep a group no merge may take. The bottom
// and four sides are groups 0 to 4 of operand A.
func newCellBox(t *testing.T, markers map[[2]int]bool) facemerge.Input {
	t.Helper()
	id := map[[2]int]int{}
	next := 5
	for pass := range 2 {
		for i := range 3 {
			for j := range 3 {
				isA := (i+j)%2 == 0 || markers[[2]int{i, j}]
				if isA == (pass == 0) {
					id[[2]int{i, j}] = next
					next++
				}
			}
		}
		if pass == 0 {
			// operand A's groups end here.
			id[[2]int{-1, -1}] = next
		}
	}
	v := r3.NewVec
	m := newMeshBuilder()
	m.gridFace(v(0, 0, 1), v(1, 0, 0), v(0, 1, 0), 3, 3, func(i, j int) int { return id[[2]int{i, j}] }, nil)
	m.gridFace(v(0, 0, 0), v(0, 1, 0), v(1, 0, 0), 3, 3, one(0), nil)
	m.gridFace(v(0, 0, 0), v(1, 0, 0), v(0, 0, 1), 3, 1, one(1), nil)
	m.gridFace(v(0, 3, 0), v(0, 0, 1), v(1, 0, 0), 1, 3, one(2), nil)
	m.gridFace(v(0, 0, 0), v(0, 0, 1), v(0, 1, 0), 1, 3, one(3), nil)
	m.gridFace(v(3, 0, 0), v(0, 1, 0), v(0, 0, 1), 3, 1, one(4), nil)
	groups := plainGroups(next)
	for cell, g := range id {
		if markers[cell] {
			groups[g] = markerGroup(t, groups[g])
		}
	}
	operandA := id[[2]int{-1, -1}]
	return m.input(t, groups, operandA)
}

func TestMergePinch(t *testing.T) {
	t.Parallel()
	in := newCellBox(t, map[[2]int]bool{{1, 1}: true, {2, 2}: true})
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.False(t, changed, "the C-shaped region touches itself at vertex (2, 2, 1)")
	require.Equal(t, in.Src, src)
	require.Len(t, groups, len(in.Groups))
}

func TestMergeRing(t *testing.T) {
	t.Parallel()
	in := newCellBox(t, map[[2]int]bool{{1, 1}: true})
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	require.Len(t, groups, len(in.Groups)+1)
	id := len(in.Groups)

	var ring, rest []int
	for t2, g := range src {
		switch {
		case g == id:
			ring = append(ring, t2)
		case g != in.Src[t2]:
			rest = append(rest, t2)
		}
	}
	require.Len(t, ring, 16, "eight ring cells of two facets")
	require.Empty(t, rest, "no other facet changes group")

	body, faceOf := assemble(t, in, src, groups)
	faces := topobuild.Faces(body)
	require.Len(t, faces, 7, "bottom, four sides, ring and the marker cell")
	ringFace := faces[faceOf[ring[0]]]
	for _, f := range ring {
		require.Equal(t, ringFace, faces[faceOf[f]])
	}
	require.Len(t, ringFace.Loops, 2)
	require.True(t, ringFace.Loops[0].Outer)
	require.False(t, ringFace.Loops[1].Outer)
}

func TestMergeGroupFields(t *testing.T) {
	t.Parallel()
	groups := plainGroups(10)
	capA, capB := &topo.CapRecord{Z: 1}, &topo.CapRecord{Z: 2}
	srcTopA := &provenance.SourcePatch{Producer: 1, Operand: 0, Ordinal: 7}
	srcTopB := &provenance.SourcePatch{Producer: 2, Operand: 1, Ordinal: 3, Parent: &provenance.SourcePatch{Producer: 9}}

	groups[gTopA].Origins = []provenance.Ref{{Producer: 1, Role: "a"}, {Producer: 1, Role: "shared"}}
	groups[gTopB].Origins = []provenance.Ref{{Producer: 2, Role: "b"}, {Producer: 1, Role: "shared"}}
	groups[gTopA].Upstream = []provenance.Ref{{Producer: 1, Role: "up"}}
	groups[gTopB].Upstream = []provenance.Ref{{Producer: 2, Role: "up"}, {Producer: 1, Role: "up"}}
	groups[gTopA].Source, groups[gTopB].Source = srcTopA, srcTopB
	groups[gTopA].HasCentroid = true
	groups[gTopA].Centroid = measurement.VecMeasurement{Value: r3.NewVec(1, 2, 3)}
	groups[gTopB] = retainedPlane(t, groups[gTopB], 1)
	groups[gTopB].CapRecord = capB
	groups[gTopB].AxialDelta, groups[gTopB].HasAxialDelta = 0.5, true
	groups[gTopB].NormalBound, groups[gTopB].Reversed = 1e-9, true
	// Both front groups carry a tag; the first member's is the donor.
	groups[gFrontA] = retainedPlane(t, groups[gFrontA], 1)
	groups[gFrontA].CapRecord = capA
	groups[gFrontB] = retainedPlane(t, groups[gFrontB], 2)
	groups[gFrontB].CapRecord = capB
	groups[gFrontB].Source = srcTopB

	in := splitBox(nil, nil).input(t, groups, splitBoxA)
	srcBefore := slices.Clone(in.Src)
	src, merged, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)

	top := merged[src[facetsOf(in.Src, gTopA)[0]]]
	require.Equal(t, []provenance.Ref{{Producer: 1, Role: "a"}, {Producer: 1, Role: "shared"}, {Producer: 2, Role: "b"}}, top.Origins)
	require.Equal(t, []provenance.Ref{{Producer: 1, Role: "up"}, {Producer: 2, Role: "up"}}, top.Upstream)
	require.NotNil(t, top.Source)
	require.Equal(t, provenance.Producer(1), top.Source.Producer)
	require.Equal(t, 0, top.Source.Operand)
	require.Equal(t, 7, top.Source.Ordinal)
	require.Nil(t, top.Source.Parent)
	require.False(t, top.Source.Blend)
	require.Len(t, top.Source.Members, 2)
	require.Same(t, srcTopA, top.Source.Members[0])
	require.Same(t, srcTopB, top.Source.Members[1])
	require.Equal(t, groups[gTopB].Surface, top.Surface)
	require.True(t, top.Retained)
	require.Same(t, capB, top.CapRecord)
	require.Equal(t, 0.5, top.AxialDelta)
	require.True(t, top.HasAxialDelta)
	require.Equal(t, 1e-9, top.NormalBound)
	require.True(t, top.Reversed)
	require.True(t, top.Planar)
	require.False(t, top.HasCentroid)
	require.Equal(t, measurement.VecMeasurement{}, top.Centroid)
	require.Nil(t, top.Denoted)
	require.Zero(t, top.ExtentDelta)
	require.False(t, top.ViewCharge)

	front := merged[src[facetsOf(in.Src, gFrontA)[0]]]
	require.Same(t, capA, front.CapRecord, "the first member with a plane tag is the donor")
	require.Equal(t, groups[gFrontA].Surface, front.Surface)
	require.Len(t, front.Source.Members, 1, "a member without a record adds none")
	require.Same(t, srcTopB, front.Source.Members[0])

	bottom := merged[src[facetsOf(in.Src, gBotA)[0]]]
	require.Nil(t, bottom.Source, "no member has a record")
	require.Nil(t, bottom.Surface)
	require.False(t, bottom.Retained)
	require.Nil(t, bottom.CapRecord)

	// The input is untouched.
	require.Len(t, in.Groups, 10)
	require.Equal(t, srcBefore, in.Src)
}

func TestMergeTagGates(t *testing.T) {
	t.Parallel()
	cases := map[string]func(*testing.T, facetedrecord.Group) facetedrecord.Group{
		"extent delta": func(t *testing.T, g facetedrecord.Group) facetedrecord.Group {
			g = retainedPlane(t, g, 1)
			g.ExtentDelta = 1e-6
			return g
		},
		"view charge": func(t *testing.T, g facetedrecord.Group) facetedrecord.Group {
			g = retainedPlane(t, g, 1)
			g.ViewCharge = true
			return g
		},
		"denoted": func(_ *testing.T, g facetedrecord.Group) facetedrecord.Group {
			g.Denoted = &surfacenormal.Revolved{Valid: true}
			return g
		},
		"cylinder": func(_ *testing.T, g facetedrecord.Group) facetedrecord.Group {
			g.Surface = surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(1)}
			g.Retained = true
			return g
		},
		"authenticated plane": func(t *testing.T, g facetedrecord.Group) facetedrecord.Group {
			g = retainedPlane(t, g, 1)
			g.Retained = false
			return g
		},
		"not planar": func(_ *testing.T, g facetedrecord.Group) facetedrecord.Group {
			g.Planar = false
			return g
		},
	}
	for name, mark := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			groups := plainGroups(10)
			groups[gTopB] = mark(t, groups[gTopB])
			in := splitBox(nil, nil).input(t, groups, splitBoxA)
			src, merged, changed, err := facemerge.Merge(t.Context(), in)
			require.NoError(t, err)
			require.True(t, changed)
			requireMergedPairs(t, in, src, merged, splitPairs[0], splitPairs[2], splitPairs[3])
		})
	}
}

func TestMergeBentGroup(t *testing.T) {
	t.Parallel()
	// Operand A's top and front halves share group 2, so that group's facets lie on two planes.
	perm := []int{0, 1, 2, 2, 4, 5, 6, 7, 8, 9}
	in := splitBox(perm, nil).input(t, plainGroups(10), splitBoxA)
	src, groups, changed, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.True(t, changed)
	requireMergedPairs(t, in, src, groups, splitPairs[0], splitPairs[3])
}

// partition returns the facets of each appended group, in group order.
func partition(src []int, base int) [][]int {
	byGroup := map[int][]int{}
	for f, g := range src {
		if g >= base {
			byGroup[g] = append(byGroup[g], f)
		}
	}
	ids := make([]int, 0, len(byGroup))
	for g := range byGroup {
		ids = append(ids, g)
	}
	slices.Sort(ids)
	out := make([][]int, len(ids))
	for i, g := range ids {
		out[i] = byGroup[g]
	}
	return out
}

func TestMergeDeterministic(t *testing.T) {
	t.Parallel()
	in := splitBox(nil, nil).input(t, plainGroups(10), splitBoxA)
	src1, groups1, _, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	src2, groups2, _, err := facemerge.Merge(t.Context(), in)
	require.NoError(t, err)
	require.Equal(t, src1, src2)
	require.Equal(t, groups1, groups2)

	perm := []int{3, 0, 4, 1, 2, 8, 5, 9, 6, 7}
	pin := splitBox(perm, nil).input(t, plainGroups(10), splitBoxA)
	psrc, _, changed, err := facemerge.Merge(t.Context(), pin)
	require.NoError(t, err)
	require.True(t, changed)
	want, got := partition(src1, 10), partition(psrc, 10)
	require.Len(t, want, 4)
	require.Equal(t, want, got)
	for _, facets := range [][][]int{want, got} {
		mins := make([]int, len(facets))
		for i, f := range facets {
			mins[i] = f[0]
		}
		require.True(t, slices.IsSorted(mins), "merged groups follow their smallest facet")
	}
}

func TestMergeInvalidInput(t *testing.T) {
	t.Parallel()
	base := func() facemerge.Input { return splitBox(nil, nil).input(t, plainGroups(10), splitBoxA) }
	cases := map[string]func(*facemerge.Input){
		"short src":         func(in *facemerge.Input) { in.Src = in.Src[1:] },
		"short adjacency":   func(in *facemerge.Input) { in.Adj = in.Adj[1:] },
		"short bounds":      func(in *facemerge.Input) { in.VertexBound = in.VertexBound[1:] },
		"src out of range":  func(in *facemerge.Input) { in.Src[0] = len(in.Groups) },
		"negative src":      func(in *facemerge.Input) { in.Src[0] = -1 },
		"operand count":     func(in *facemerge.Input) { in.OperandAGroups = len(in.Groups) + 1 },
		"negative operands": func(in *facemerge.Input) { in.OperandAGroups = -1 },
		"vertex index":      func(in *facemerge.Input) { in.Tris[0][0] = len(in.Verts) },
		"neighbour index":   func(in *facemerge.Input) { in.Adj[0] = []int{len(in.Tris), 0, 0} },
	}
	for name, mutate := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			in := base()
			in.Src, in.Tris, in.Adj = slices.Clone(in.Src), slices.Clone(in.Tris), slices.Clone(in.Adj)
			mutate(&in)
			_, _, changed, err := facemerge.Merge(t.Context(), in)
			require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
			require.False(t, changed)
		})
	}
}

func TestMergeCancelled(t *testing.T) {
	t.Parallel()
	in := splitBox(nil, nil).input(t, plainGroups(10), splitBoxA)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, changed, err := facemerge.Merge(ctx, in)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, changed)
}
