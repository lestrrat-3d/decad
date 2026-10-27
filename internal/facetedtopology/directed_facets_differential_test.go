package facetedtopology

import (
	"context"
	"errors"
	"fmt"
	"maps"
	"math"
	"math/rand"
	"sort"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func (c topologyCase) clone(name string) topologyCase {
	out := topologyCase{name: c.name + "/" + name, verts: c.verts, planar: maps.Clone(c.planar)}
	out.tris = append([][3]int(nil), c.tris...)
	out.facetFace = append([]int(nil), c.facetFace...)
	return out
}

// variants derives open, non-manifold, duplicated, reordered and
// out-of-range forms of a closed case.
func (c topologyCase) variants(rng *rand.Rand) []topologyCase {
	out := []topologyCase{c}
	shuffled := c.clone("shuffled")
	rng.Shuffle(len(shuffled.tris), func(i, j int) {
		shuffled.tris[i], shuffled.tris[j] = shuffled.tris[j], shuffled.tris[i]
		shuffled.facetFace[i], shuffled.facetFace[j] = shuffled.facetFace[j], shuffled.facetFace[i]
	})
	for i, t := range shuffled.tris {
		k := rng.Intn(3)
		shuffled.tris[i] = [3]int{t[k], t[(k+1)%3], t[(k+2)%3]}
	}
	out = append(out, shuffled)
	open := c.clone("open")
	for range 1 + len(open.tris)/50 {
		i := rng.Intn(len(open.tris))
		open.tris = append(open.tris[:i], open.tris[i+1:]...)
		open.facetFace = append(open.facetFace[:i], open.facetFace[i+1:]...)
	}
	out = append(out, open)
	duplicate := c.clone("duplicate")
	for range 3 {
		i := rng.Intn(len(duplicate.tris))
		duplicate.tris = append(duplicate.tris, duplicate.tris[i])
		duplicate.facetFace = append(duplicate.facetFace, rng.Intn(4))
	}
	out = append(out, duplicate)
	nonManifold := c.clone("nonmanifold")
	for range 3 {
		t := nonManifold.tris[rng.Intn(len(nonManifold.tris))]
		nonManifold.tris = append(nonManifold.tris, [3]int{t[1], t[0], rng.Intn(len(c.verts))})
		nonManifold.facetFace = append(nonManifold.facetFace, rng.Intn(4))
	}
	out = append(out, nonManifold)
	flipped := c.clone("flipped")
	i := rng.Intn(len(flipped.tris))
	flipped.tris[i] = [3]int{flipped.tris[i][0], flipped.tris[i][2], flipped.tris[i][1]}
	out = append(out, flipped)
	relabeled := c.clone("relabeled")
	for range 1 + len(relabeled.tris)/20 {
		relabeled.facetFace[rng.Intn(len(relabeled.tris))] = rng.Intn(6)
	}
	relabeled.planar[3] = true
	out = append(out, relabeled)
	degenerate := c.clone("degenerate")
	t := degenerate.tris[0]
	degenerate.tris = append(degenerate.tris, [3]int{t[0], t[0], t[1]}, [3]int{t[2], t[2], t[2]})
	degenerate.facetFace = append(degenerate.facetFace, 0, 1)
	out = append(out, degenerate)
	for _, bad := range []int{-1, len(c.verts), -(1 << 32)} {
		outside := c.clone(fmt.Sprintf("outside%d", bad))
		outside.tris[rng.Intn(len(outside.tris))][rng.Intn(3)] = bad
		out = append(out, outside)
	}
	short := c.clone("shortfaces")
	short.facetFace = short.facetFace[:len(short.facetFace)-1]
	out = append(out, short)
	return out
}

// randomTopologyCases are tiny meshes over few vertices, so self-edges,
// repeated directions, missing twins and lost facets all occur.
func randomTopologyCases(rng *rand.Rand, count int) []topologyCase {
	var out []topologyCase
	for k := range count {
		c := topologyCase{name: fmt.Sprintf("random-%d", k), planar: map[int]bool{}}
		for range 3 + rng.Intn(4) {
			c.verts = append(c.verts, r3.NewVec(rng.Float64(), rng.Float64(), rng.Float64()))
		}
		for range 1 + rng.Intn(10) {
			var t [3]int
			for j := range t {
				t[j] = rng.Intn(len(c.verts))
			}
			c.tris = append(c.tris, t)
			c.facetFace = append(c.facetFace, rng.Intn(3))
		}
		c.planar[rng.Intn(3)] = true
		out = append(out, c)
	}
	return out
}

// topologyRun is everything one build call shows its caller: the result,
// the error, a panic, and how many budget steps it took.
type topologyRun struct {
	lines []string
	err   string
	panic string
	steps int
}

var errStepLimit = errors.New("step limit")

func runTopology(ctx context.Context, build func(context.Context, *proofbound.WorkBudget, []r3.Vec, [][3]int, []int, map[int]bool,
	[]float64, *Result) error, c topologyCase, limit int) topologyRun {
	run := &topologyRun{}
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			run.steps++
			if run.steps == limit {
				return errStepLimit
			}
			return nil
		},
		ErrFn: func() error { return nil },
	}
	bounds := make([]float64, len(c.verts))
	for i := range bounds {
		bounds[i] = float64(i%7) * 1e-9
	}
	func() {
		defer func() {
			if p := recover(); p != nil {
				run.panic = fmt.Sprint(p)
			}
		}()
		var result Result
		if err := build(ctx, budget, c.verts, c.tris, c.facetFace, c.planar, bounds, &result); err != nil {
			run.err = err.Error()
			return
		}
		run.lines = encodeResult(result)
	}()
	return *run
}

// encodeResult lists a result line by line: every edge's fields with exact
// float bits, then every face's loops in face order with each coedge's edge
// index and direction.
func encodeResult(r Result) []string {
	index := map[*Edge]int{}
	var lines []string
	for i, e := range r.Edges {
		index[e] = i
		lines = append(lines, fmt.Sprintf("edge %d %v %v %v %x %x %x %v", i, e.Vertices, e.Faces, e.Convex,
			math.Float64bits(e.Bound), math.Float64bits(e.Length), math.Float64bits(e.LengthBound), e.LengthUnbounded))
	}
	faces := make([]int, 0, len(r.FaceLoops))
	for f := range r.FaceLoops {
		faces = append(faces, f)
	}
	sort.Ints(faces)
	lines = append(lines, fmt.Sprintf("faceloops nil=%v len=%d", r.FaceLoops == nil, len(r.FaceLoops)))
	for _, f := range faces {
		lines = append(lines, fmt.Sprintf("face %d nil=%v loops=%d", f, r.FaceLoops[f] == nil, len(r.FaceLoops[f])))
		for li, l := range r.FaceLoops[f] {
			line := fmt.Sprintf("face %d loop %d outer=%v", f, li, l.Outer)
			for _, ce := range l.Coedges {
				line += fmt.Sprintf(" %d/%v", index[ce.Edge], ce.Forward)
			}
			lines = append(lines, line)
		}
	}
	return lines
}

// requireSameRun compares two runs and reports only the first differing line.
func requireSameRun(t *testing.T, name string, want, got topologyRun) {
	t.Helper()
	require.Equal(t, want.panic, got.panic, "%s: panic", name)
	require.Equal(t, want.err, got.err, "%s: error", name)
	require.Equal(t, want.steps, got.steps, "%s: budget steps", name)
	require.Equal(t, len(want.lines), len(got.lines), "%s: result lines", name)
	for i := range want.lines {
		if want.lines[i] != got.lines[i] {
			require.Failf(t, "result differs", "%s: line %d: want %.200q, got %.200q", name, i, want.lines[i], got.lines[i])
		}
	}
}

func topologyCases(t *testing.T) []topologyCase {
	t.Helper()
	rng := rand.New(rand.NewSource(2647))
	var cases []topologyCase
	for _, base := range []topologyCase{
		cylinderCase(24, 6, 2, 3),
		cylinderCase(5, 1, 1, 1),
		sphereCase(16, 8, 1),
		sphereCase(16, 8, 3),
		sphereCase(3, 2, 2),
	} {
		cases = append(cases, base.variants(rng)...)
	}
	return append(cases, randomTopologyCases(rng, 3000)...)
}

func TestBuildMatchesDirectedFacetsReference(t *testing.T) {
	t.Parallel()
	statuses := map[string]int{}
	for _, c := range topologyCases(t) {
		want := runTopology(t.Context(), referenceBuild, c, -1)
		got := runTopology(t.Context(), build, c, -1)
		if strings.Contains(want.err, "has no boundary loop") {
			// build visits faces in map order to find one without a loop, so
			// the steps taken before that refusal vary from run to run in the
			// reference too. Only the refusal itself is comparable.
			require.Equal(t, want.err, got.err, c.name)
			statuses[want.err]++
			continue
		}
		requireSameRun(t, c.name, want, got)
		if !strings.Contains(c.name, "/") && !strings.HasPrefix(c.name, "random-") {
			// Every unmodified closed mesh builds, so its edges and loops compare.
			require.Empty(t, want.err+want.panic, c.name)
		}
		switch {
		case want.panic != "":
			statuses["panic"]++
		case want.err != "":
			statuses[want.err]++
		default:
			statuses["ok"]++
		}
		// The same budget exhaustion must stop both at the same step.
		for _, limit := range []int{1, want.steps / 3, want.steps / 2, want.steps - 1, want.steps} {
			if limit < 1 {
				continue
			}
			requireSameRun(t, fmt.Sprintf("%s@%d", c.name, limit), runTopology(t.Context(), referenceBuild, c, limit),
				runTopology(t.Context(), build, c, limit))
		}
	}
	t.Logf("outcomes: %v", statuses)
	// The cases reach success, every refusal shape and panics alike.
	require.Greater(t, statuses["ok"], 20)
	require.Greater(t, statuses["panic"], 5)
	require.GreaterOrEqual(t, len(statuses), 6, "outcomes %v", statuses)
}

func TestHalfedgeOwnersMatchDirectedFacets(t *testing.T) {
	t.Parallel()
	repeated := 0
	for _, c := range topologyCases(t) {
		directed := newDirectedFacets(len(c.verts), 0)
		for i, tri := range c.tris {
			for k := range 3 {
				directed.set(tri[k], tri[(k+1)%3], i)
			}
		}
		got := newHalfedgeOwners(directed, c.tris)
		require.Equal(t, directed.len(), got.distinct, c.name)
		if got.own != nil {
			repeated++
		}
		for i, tri := range c.tris {
			for k := range 3 {
				o := 3*i + k
				own, _ := directed.get(tri[k], tri[(k+1)%3])
				twin, ok := directed.get(tri[(k+1)%3], tri[k])
				if !ok {
					twin = -1
				}
				if own != got.owner(o) || twin != got.twin[o] {
					require.Failf(t, "owner differs", "%s: ordinal %d: want %d/%d, got %d/%d", c.name, o,
						own, twin, got.owner(o), got.twin[o])
				}
			}
		}
	}
	// Both the repeated-edge table and the implied-owner shortcut run.
	require.Greater(t, repeated, 100)
}
