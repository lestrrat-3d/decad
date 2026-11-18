package retainedblend_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// shearedPlate is bevelledPlate sheared so that its end planes meet the bevel edge obliquely: the end plane
// x − k·z = c0 has the normal (1, 0, −k)/N and the edge's sine to it is 1/N, N = √(1 + k²). The held mesh stands
// on the true end planes G*, each moved outward by delta along its normal, by sliding every end vertex along the
// edge direction x (so every side plane keeps its place). With tagged set the end plane tags are the displaced
// planes' own and state nothing (the twin, an exact solid); else they are the unmoved planes G, which state
// AxialDelta delta. With onTags set (and tagged clear) the held mesh stays on the tag planes instead of the true
// planes, and every end vertex states a bound of the slide, which the true plane G* is from it.
func shearedPlate(t *testing.T, k, delta float64, tagged, onTags bool) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	norm := math.Hypot(1, k)
	slide := delta * norm
	section := [][2]float64{{0, 0}, {20, 0}, {20, 20}, {2, 20}, {0, 18}}
	n := len(section)
	var fp facetedrecord.Payload
	for _, x := range []float64{0, 40} {
		for _, p := range section {
			off := -slide
			if x > 0 {
				off = slide
			}
			bound := 0.0
			if onTags {
				off, bound = 0, slide
			}
			fp.Verts = append(fp.Verts, r3.NewVec(x+k*p[1]+off, p[0], p[1]))
			fp.VertexBound = append(fp.VertexBound, bound)
		}
	}
	tri := func(g, a, b, c int) {
		fp.Tris = append(fp.Tris, [3]int{a, b, c})
		fp.Src = append(fp.Src, g)
	}
	for i := range n {
		j := (i + 1) % n
		fp.Groups = append(fp.Groups, axialPlane(t, "side"+string(rune('0'+i)), fp.Verts[i], fp.Verts[j], fp.Verts[i+n]))
		tri(i, i, j, j+n)
		tri(i, i, j+n, i+n)
	}
	end, start := len(fp.Groups), len(fp.Groups)+1
	endX, startX := 40.0, 0.0
	if tagged {
		endX, startX = 40+slide, -slide
	}
	fp.Groups = append(fp.Groups,
		axialPlane(t, "capEnd", r3.NewVec(endX, 0, 0), r3.NewVec(endX, 1, 0), r3.NewVec(endX+k, 0, 1)),
		axialPlane(t, "capStart", r3.NewVec(startX, 0, 0), r3.NewVec(startX+k, 0, 1), r3.NewVec(startX, 1, 0)))
	if !tagged {
		fp.Groups[end].HasAxialDelta, fp.Groups[end].AxialDelta = true, delta
		fp.Groups[start].HasAxialDelta, fp.Groups[start].AxialDelta = true, delta
	}
	for i := 1; i < n-1; i++ {
		tri(end, n, n+i, n+i+1)
		tri(start, 0, i+1, i)
	}
	fp.DPair = r3.NewVec(40, 40, 40).Len()
	b, err := facetedrecord.Build(t.Context(), axialEnv(), fp)
	require.NoError(t, err)
	return b, payloadOf(t, b)
}

// TestChamferEndPlaneDisplacementIsInAreaSlack chamfers the edge of a plate whose end planes meet it at a sine of
// about 0.3 and whose held mesh stands on the true end planes, which lie delta off their tags. The chamfer of the
// exact twin (tags on the true planes) is the ground truth of the true area change; the displaced build's held
// area change must lie within the area slack it publishes of that truth (agent-docs/modify-design.md Part E
// §6.3, "A displaced plane in the area bound").
func TestChamferEndPlaneDisplacementIsInAreaSlack(t *testing.T) {
	t.Parallel()
	const k, delta = 3.0, 1e-5
	chamfer := func(t *testing.T, delta float64, tagged, onTags bool) (before, after facetedrecord.Payload) {
		t.Helper()
		b, built := shearedPlate(t, k, delta, tagged, onTags)
		out, recognized, err := retainedblend.Chamfer(t.Context(), axialEnv(), b, built,
			[]*topo.Edge{topBevelEdge(t, b)}, 1, 0, nil)
		require.True(t, recognized)
		require.NoError(t, err)
		return built, payloadOf(t, out)
	}
	change := func(before, after facetedrecord.Payload) float64 { return meshArea(after) - meshArea(before) }
	// The exact twins: end planes at the tags, and at the true planes.
	atTag := change(chamfer(t, 0, true, false))
	atTrue := change(chamfer(t, delta, true, false))
	before, after := chamfer(t, delta, false, false)
	held := change(before, after)
	slack := after.AreaSlack - before.AreaSlack
	// The twin whose held mesh stands on the tag planes, delta short of the true planes G*, with vertex bounds that
	// cover the slide: its held change is the tags' own, so only the area term lets it reach the true change.
	beforeTags, afterTags := chamfer(t, delta, false, true)
	heldTags := change(beforeTags, afterTags)
	slackTags := afterTags.AreaSlack - beforeTags.AreaSlack
	t.Logf("on tags: held %.9g slack %.3g gap to true %.3g", heldTags, slackTags, math.Abs(heldTags-atTrue))
	require.LessOrEqual(t, math.Abs(heldTags-atTrue), slackTags)
	measured, move := math.Abs(held-atTag), math.Abs(atTrue-atTag)
	t.Logf("tag %.9g true %.9g held %.9g measured %.3g move %.3g slack %.3g", atTag, atTrue, held, measured, move, slack)
	require.LessOrEqual(t, math.Abs(held-atTrue), slack)
	// Without the displaced-plane term the slack is the measured gap to the tags' change alone, which is short of
	// the true change by the move of the exact pieces when the end planes shift.
	require.Greater(t, move, 1e-6, "the displacement moves the exact area change")
	require.GreaterOrEqual(t, slack, measured+move)
}
