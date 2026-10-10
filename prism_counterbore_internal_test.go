package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCounterboreArrangementPreservesWholeLoops(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, -10, -10, 10, 10, 10)
	hole := internalCircleBody(t, doc, 0, 2, 0, Distance{D: units.Millimeters(10), Dir: Along})
	drilled, err := Cut(t.Context(), plate, hole)
	require.NoError(t, err)
	tool := internalCircleBody(t, doc, 0, 4, 7, Distance{D: units.Millimeters(3), Dir: Along})

	budget := proofbound.NewWorkBudget(t.Context())
	target, cutter, admitted, err := admitPrismPairBudget(budget, drilled, tool)
	require.NoError(t, err)
	require.True(t, admitted)
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(target), prismPlacementOf(cutter))
	require.NoError(t, err)
	scene, tags, _, err := buildPrismScene(budget, target, cutter, reexpress)
	require.NoError(t, err)
	profiles, err := prismcells.ProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)

	targetOuter, err := prismcells.LoopEntitySet(budget, tags, false, -1)
	require.NoError(t, err)
	targetHole, err := prismcells.LoopEntitySet(budget, tags, false, 0)
	require.NoError(t, err)
	toolOuter, err := prismcells.LoopEntitySet(budget, tags, true, -1)
	require.NoError(t, err)
	outside, found, err := prismcells.FindLoopMatch(budget, profiles, targetOuter,
		[]map[sketch.Entity]struct{}{toolOuter})
	require.NoError(t, err)
	require.True(t, found)
	require.True(t, outside.Valid)
	inside, found, err := prismcells.FindLoopMatch(budget, profiles, toolOuter,
		[]map[sketch.Entity]struct{}{targetHole})
	require.NoError(t, err)
	require.True(t, found)
	require.True(t, inside.Valid)
	match, found, err := prismcells.MatchEnclosingCut(budget, tags, profiles, 1)
	require.NoError(t, err)
	require.True(t, found)
	require.Same(t, outside, match.Outside)
	require.Same(t, inside, match.Inside)
	require.Empty(t, match.OutsideHoles)
	require.Equal(t, []int{0}, match.EnclosedHoles)
}

func TestCounterboreReverseArrangementProvesEmptyCut(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, -10, -10, 10, 10, 10)
	wide := internalCircleBody(t, doc, 0, 4, 7, Distance{D: units.Millimeters(3), Dir: Along})
	pocket, err := Cut(t.Context(), plate, wide)
	require.NoError(t, err)
	sp, ok := pocket.payload.(stackedPrismPayload)
	require.True(t, ok)
	tool := internalCircleBody(t, doc, 0, 2, 0, Distance{D: units.Millimeters(10), Dir: Along})
	budget := proofbound.NewWorkBudget(t.Context())
	proxy := &Body{payload: sp.outerPrism()}
	target, cutter, admitted, err := admitPrismPairBudget(budget, proxy, tool)
	require.NoError(t, err)
	require.True(t, admitted)
	target.profile = sp.slabs[1].Regions[0]
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(target), prismPlacementOf(cutter))
	require.NoError(t, err)
	scene, tags, _, err := buildPrismScene(budget, target, cutter, reexpress)
	require.NoError(t, err)
	profiles, err := prismcells.ProfilesContext(t.Context(), scene.Profiles)
	require.NoError(t, err)
	targetOuter, err := prismcells.LoopEntitySet(budget, tags, false, -1)
	require.NoError(t, err)
	targetHole, err := prismcells.LoopEntitySet(budget, tags, false, 0)
	require.NoError(t, err)
	toolOuter, err := prismcells.LoopEntitySet(budget, tags, true, -1)
	require.NoError(t, err)
	material, found, err := prismcells.FindLoopMatch(budget, profiles, targetOuter,
		[]map[sketch.Entity]struct{}{targetHole})
	require.NoError(t, err)
	require.True(t, found)
	require.True(t, material.Valid)
	annulus, found, err := prismcells.FindLoopMatch(budget, profiles, targetHole,
		[]map[sketch.Entity]struct{}{toolOuter})
	require.NoError(t, err)
	require.True(t, found)
	require.True(t, annulus.Valid)
	noOp, err := prismcells.MatchCutNoOp(budget, tags, profiles, 1)
	require.NoError(t, err)
	require.True(t, noOp)
}
