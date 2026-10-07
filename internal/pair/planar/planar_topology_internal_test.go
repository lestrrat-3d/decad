package planar

import (
	"fmt"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// preparePlanarRebuilt is preparePlanar as one pass over the solid, before its
// combinatorial half moved into PlanarTopology. The split forms must give the
// same data, field for field and in the same order.
func preparePlanarRebuilt(s *PlanarSolid) *planarPrep {
	p := &planarPrep{s: s, normal: make([]proof.DyV3, len(s.Tris)),
		triLo: make([][3]proof.Dyadic, len(s.Tris)), triHi: make([][3]proof.Dyadic, len(s.Tris))}
	parent := make([]int, len(s.Verts))
	for i := range parent {
		parent[i] = i
	}
	find := func(v int) int {
		for parent[v] != v {
			parent[v] = parent[parent[v]]
			v = parent[v]
		}
		return v
	}
	seen := make(map[[2]int]int, 3*len(s.Tris)/2)
	for t, tri := range s.Tris {
		a := s.Verts[tri[0]]
		p.normal[t] = proof.DvCross(proof.DvSub(s.Verts[tri[1]], a), proof.DvSub(s.Verts[tri[2]], a))
		p.triLo[t], p.triHi[t] = pointBox(s.Verts, tri[:])
		for i := range 3 {
			u, w := tri[i], tri[(i+1)%3]
			parent[find(u)] = find(w)
			key := [2]int{min(u, w), max(u, w)}
			if e, ok := seen[key]; ok {
				p.edgeFacets[e][1] = t
				continue
			}
			seen[key] = len(p.edges)
			p.edges = append(p.edges, key)
			p.edgeFacets = append(p.edgeFacets, [2]int{t, t})
			lo, hi := pointBox(s.Verts, key[:])
			p.edgeLo, p.edgeHi = append(p.edgeLo, lo), append(p.edgeHi, hi)
		}
	}
	p.shellOf = make([]int, len(s.Verts))
	ids := make(map[int]int)
	for v := range s.Verts {
		root := find(v)
		id, ok := ids[root]
		if !ok {
			id = len(ids)
			ids[root] = id
		}
		p.shellOf[v] = id
	}
	p.shells = len(ids)
	return p
}

func requireDyadicsEqual(t *testing.T, want, got []proof.Dyadic, label string) {
	t.Helper()
	require.Len(t, got, len(want), label)
	for i := range want {
		require.Zero(t, proof.DyCmp(want[i], got[i]), "%s [%d]", label, i)
	}
}

func flatV3(vs []proof.DyV3) []proof.Dyadic {
	out := make([]proof.Dyadic, 0, 3*len(vs))
	for _, v := range vs {
		out = append(out, v[:]...)
	}
	return out
}

func flatBox(bs [][3]proof.Dyadic) []proof.Dyadic {
	out := make([]proof.Dyadic, 0, 3*len(bs))
	for _, b := range bs {
		out = append(out, b[:]...)
	}
	return out
}

// requirePrepEqual asserts two derived-data sets agree field for field.
func requirePrepEqual(t *testing.T, want, got *planarPrep, label string) {
	t.Helper()
	require.Same(t, want.s, got.s, label)
	requireDyadicsEqual(t, flatV3(want.normal), flatV3(got.normal), label+" normal")
	requireDyadicsEqual(t, flatBox(want.triLo), flatBox(got.triLo), label+" triLo")
	requireDyadicsEqual(t, flatBox(want.triHi), flatBox(got.triHi), label+" triHi")
	requireDyadicsEqual(t, flatBox(want.edgeLo), flatBox(got.edgeLo), label+" edgeLo")
	requireDyadicsEqual(t, flatBox(want.edgeHi), flatBox(got.edgeHi), label+" edgeHi")
	require.Equal(t, want.edges, got.edges, label+" edges")
	require.Equal(t, want.edgeFacets, got.edgeFacets, label+" edgeFacets")
	require.Equal(t, want.shellOf, got.shellOf, label+" shellOf")
	require.Equal(t, want.shells, got.shells, label+" shells")
}

// topologyAffine is an exact affine map x ↦ M·x + c with dyadic entries.
type topologyAffine struct {
	m [3]proof.DyV3
	c proof.DyV3
}

func (a topologyAffine) apply(v proof.DyV3) proof.DyV3 {
	var out proof.DyV3
	for i := range 3 {
		out[i] = proof.DyAdd(proof.DvDot(a.m[i], v), a.c[i])
	}
	return out
}

// TestPlanarTopologyMatchesFullAudit holds the split audit and derived data to
// the one-pass forms they replace. Solids are sheared boxes (some with a
// fanned top), two-box pairs (two shells), and their mutants: a flipped
// triangle, an index out of range, a repeated corner, a dropped triangle.
// Each is placed under exact affine maps of positive, negative and zero
// determinant, rank one included, so the coordinate half meets zero normals
// and nonpositive volumes. For every placement, NewPlanarTopology plus
// CheckPlanarPose must accept exactly when CheckPlanarSolid does;
// preparePlanar must match preparePlanarRebuilt with the pose data attached
// and without it; ClassifyPlanar must decide an attached pair as it decides
// fresh copies; and a solid whose Verts was swapped after the audit must not
// be served the stale data.
//
// Legs shown to fail: CheckPlanarPose without its volume test accepts the
// negative-determinant placements; planarPose.serves always true serves the
// swapped solid stale boxes; planarTopologyOf keyed by (u, w) unsorted gives
// every edge twice; newPlanarPose reading edge boxes off the triangle corners
// gives wrong edge boxes.
func TestPlanarTopologyMatchesFullAudit(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(71, 73))
	quarter := func(lo, hi int) proof.Dyadic {
		return proof.DyShift(proof.DyInt(int64(lo+rng.IntN(hi-lo+1))), -2)
	}
	fine := func() proof.Dyadic {
		return proof.DyShift(proof.DyInt(int64(rng.IntN(2049)-1024)), -rng.IntN(40))
	}
	randomBox := func() PlanarSolid {
		size := [3]proof.Dyadic{quarter(2, 16), quarter(2, 16), quarter(2, 16)}
		lo := [3]proof.Dyadic{quarter(-8, 8), quarter(-8, 8), quarter(-8, 8)}
		return shearedBox(lo, size, quarter(-2, 2), quarter(-2, 2), rng.IntN(3), rng.IntN(2) == 0)
	}
	solid := func(kind int) PlanarSolid {
		s := randomBox()
		if kind%2 == 1 {
			other := randomBox()
			base := len(s.Verts)
			s.Verts = append(s.Verts, other.Verts...)
			for _, tri := range other.Tris {
				s.Tris = append(s.Tris, [3]int{tri[0] + base, tri[1] + base, tri[2] + base})
			}
			s.Faces = append(s.Faces, other.Faces...)
		}
		t := rng.IntN(len(s.Tris))
		switch kind / 2 {
		case 1:
			s.Tris[t] = [3]int{s.Tris[t][0], s.Tris[t][2], s.Tris[t][1]}
		case 2:
			s.Tris[t][rng.IntN(3)] = len(s.Verts) + rng.IntN(2)
		case 3:
			s.Tris[t][1] = s.Tris[t][0]
		case 4:
			s.Tris = append(s.Tris[:t], s.Tris[t+1:]...)
			s.Faces = append(s.Faces[:t], s.Faces[t+1:]...)
		}
		return s
	}
	affine := func(kind int) topologyAffine {
		var a topologyAffine
		for {
			for i := range 3 {
				for j := range 3 {
					a.m[i][j] = quarter(-4, 4)
				}
				a.c[i] = fine()
			}
			if kind == 2 {
				// Rank one: every row a multiple of the first.
				a.m[1] = proof.DyV3{proof.DyShift(a.m[0][0], 1), proof.DyShift(a.m[0][1], 1), proof.DyShift(a.m[0][2], 1)}
				a.m[2] = proof.DyV3{proof.DyNeg(a.m[0][0]), proof.DyNeg(a.m[0][1]), proof.DyNeg(a.m[0][2])}
				return a
			}
			det := proof.DvDot(a.m[0], proof.DvCross(a.m[1], a.m[2])).Sign()
			if kind == 0 && det > 0 || kind == 1 && det < 0 || kind == 3 && det == 0 {
				return a
			}
		}
	}
	accepted, refused, stale := 0, 0, 0
	relations := map[pair.Relation]int{}
	for trial := range 1000 {
		kind := rng.IntN(20)
		if kind >= 10 {
			kind %= 2
		}
		snapshot := solid(kind)
		topo, admitted, err := NewPlanarTopology(len(snapshot.Verts), snapshot.Tris, noPollInternal)
		require.NoError(t, err)
		var posed [2]PlanarSolid
		for k := range posed {
			kind := 0
			if k == 0 {
				kind = rng.IntN(5) % 4
			}
			place := affine(kind)
			verts := make([]proof.DyV3, len(snapshot.Verts))
			for i, v := range snapshot.Verts {
				verts[i] = place.apply(v)
			}
			s := PlanarSolid{Verts: verts, Tris: snapshot.Tris, Faces: snapshot.Faces}
			fresh := PlanarSolid{Verts: verts, Tris: snapshot.Tris, Faces: snapshot.Faces}
			want, err := CheckPlanarSolid(&fresh, noPollInternal)
			require.NoError(t, err)
			got := false
			if admitted {
				got, err = CheckPlanarPose(&s, topo, noPollInternal)
				require.NoError(t, err)
			}
			require.Equal(t, want, got, "trial %d pose %d", trial, k)
			require.Equal(t, got, s.pose != nil, "trial %d pose %d: pose data attached on acceptance", trial, k)
			if !got {
				refused++
				continue
			}
			accepted++
			requirePrepEqual(t, preparePlanarRebuilt(&s), preparePlanar(&s), fmt.Sprintf("attached trial %d pose %d", trial, k))
			requirePrepEqual(t, preparePlanarRebuilt(&fresh), preparePlanar(&fresh), fmt.Sprintf("fresh trial %d pose %d", trial, k))
			posed[k] = s

			swapped := s
			swapped.Verts = append([]proof.DyV3(nil), s.Verts...)
			swapped.Verts[0] = proof.DvAdd(swapped.Verts[0], proof.DyV3{fine(), fine(), fine()})
			requirePrepEqual(t, preparePlanarRebuilt(&swapped), preparePlanar(&swapped), fmt.Sprintf("swapped trial %d", trial))
			stale++
		}
		if posed[0].pose == nil || posed[1].pose == nil {
			continue
		}
		freshA := PlanarSolid{Verts: posed[0].Verts, Tris: posed[0].Tris, Faces: posed[0].Faces}
		freshB := PlanarSolid{Verts: posed[1].Verts, Tris: posed[1].Tris, Faces: posed[1].Faces}
		want, err := ClassifyPlanar(&freshA, &freshB, noPollInternal)
		require.NoError(t, err)
		got, err := ClassifyPlanar(&posed[0], &posed[1], noPollInternal)
		require.NoError(t, err)
		require.Equal(t, want.Relation, got.Relation, "trial %d", trial)
		require.Equal(t, want.Reason, got.Reason, "trial %d", trial)
		require.Equal(t, want.Gap, got.Gap, "trial %d", trial)
		require.Equal(t, want.Contacts, got.Contacts, "trial %d", trial)
		require.Equal(t, want.Crossings, got.Crossings, "trial %d", trial)
		relations[got.Relation]++
	}
	t.Logf("accepted %d refused %d relations %v", accepted, refused, relations)
	require.Positive(t, accepted, "premise: some placements are accepted")
	require.Positive(t, refused, "premise: some placements are refused")
	require.Positive(t, stale, "premise: some solids are swapped after the audit")
	require.Positive(t, relations[pair.Separated], "premise: some pairs are separated")
	require.Positive(t, relations[pair.Overlapping], "premise: some pairs overlap")
}
