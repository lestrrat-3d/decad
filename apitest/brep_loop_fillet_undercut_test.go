package apitest_test

import (
	"regexp"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var filletLoopRole = regexp.MustCompile(`^filletLoop\(\d+,0,(\d)\)$`)

// filletPatchIndexes lists, sorted, the walk index of every fillet-loop patch
// among faces.
func filletPatchIndexes(faces []*decad.Face) []string {
	var out []string
	for _, f := range faces {
		for _, o := range f.Origins() {
			if m := filletLoopRole.FindStringSubmatch(o.Role); m != nil {
				out = append(out, m[1])
			}
		}
	}
	slices.Sort(out)
	return out
}

// TestBrepLoopFilletUndercutReadsEachPipePatch pulls along the six axes and a
// diagonal at the pocketed plate whose top loop carries four quarter-cylinder
// fillet patches (loop-fillet Table DF's DF7). A patch is an undercut exactly
// when some point of its outward normal opposes the pull in the sense the
// walls' own readings use: pulling straight down leaves all four, pulling
// straight up leaves none, and a sideways pull leaves only the patch on the
// wall that faces away from it.
func TestBrepLoopFilletUndercutReadsEachPipePatch(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 40, 10)
	pocket, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, 10, 15, 30, 25, 5, 5))
	require.NoError(t, err)
	top := planeFacing(t, pocket, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10))
	got, err := decad.Fillet(t.Context(), pocket, loopEdgeQuery(top.Loops()[0]), units.Millimeters(1.5))
	require.NoError(t, err)
	require.Len(t, filletLoopPatches(got), 4)

	tests := []struct {
		pull r3.Vec
		want []string
	}{
		{r3.Vec{Z: 1}, nil},
		{r3.Vec{Z: -1}, []string{"0", "1", "2", "3"}},
		{r3.Vec{X: 1}, []string{"3"}},
		{r3.Vec{X: -1}, []string{"1"}},
		{r3.Vec{Y: 1}, []string{"0"}},
		{r3.Vec{Y: -1}, []string{"2"}},
		{r3.Vec{X: 1, Y: 1}, []string{"0", "3"}},
	}
	for _, tc := range tests {
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(tc.pull))
		require.NoError(t, err)
		require.Len(t, report.Bodies, 1)
		br := report.Bodies[0]
		require.NotNil(t, br.Undercut.Faces, "pull %v", tc.pull)
		require.Equal(t, tc.want, filletPatchIndexes(br.Undercut.Faces), "pull %v", tc.pull)
	}
}

// TestBrepLoopFilletUndercutReflexCornerAndBottomLoop fillets the L prism's
// reflex corner (the torus patch, read through the turn of the corner) on its
// top loop, and the same loop on the bottom cap, whose face frame points the
// other way. Each patch's outward normal sweeps from the wall's horizontal
// normal to the cap's, so the lists follow from that sweep: the wall facing
// away from a sideways pull, the torus when the pull opposes any of the
// directions between its two walls, and every patch when the pull runs
// against a bottom rim's downward normals.
func TestBrepLoopFilletUndercutReflexCornerAndBottomLoop(t *testing.T) {
	t.Parallel()
	all := []string{"0", "1", "2", "3", "4", "5", "6"}
	tests := []struct {
		name   string
		normal r3.Vec
		at     r3.Vec
		pulls  map[r3.Vec][]string
	}{
		{"top loop", r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10), map[r3.Vec][]string{
			{Z: 1}: nil, {Z: -1}: all,
			{X: 1}: {"6"}, {X: -1}: {"1", "3", "4"},
			{Y: 1}: {"0"}, {Y: -1}: {"2", "3", "5"},
			{X: 1, Y: 1}: {"0", "6"}, {X: -1, Y: -1}: {"1", "2", "3", "4", "5"},
			{X: 1, Y: -1}: {"2", "3", "5", "6"}, {X: -1, Y: 1}: {"0", "1", "3", "4"},
		}},
		{"bottom loop", r3.NewVec(0, 0, -1), r3.NewVec(0, 0, 0), map[r3.Vec][]string{
			{Z: 1}: all, {Z: -1}: nil,
			{X: 1}: {"6"}, {X: -1}: {"1", "3", "4"},
		}},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, l := polygonPrism(t, lPrism)
			face := planeFacing(t, l, tc.normal, tc.at)
			got, err := decad.Fillet(t.Context(), l, loopEdgeQuery(face.Loops()[0]), units.Millimeters(1.5))
			require.NoError(t, err)
			patches := filletLoopPatches(got)
			require.Len(t, patches, 7)
			_, torus := patches[3].Surface().(decad.Torus)
			require.True(t, torus, "the reflex corner's patch is a torus")
			for pull, want := range tc.pulls {
				report, err := doc.Verify(t.Context(), decad.WithPullDirection(pull))
				require.NoError(t, err)
				br := report.Bodies[0]
				require.NotNil(t, br.Undercut.Faces, "pull %v", pull)
				require.Equal(t, want, filletPatchIndexes(br.Undercut.Faces), "pull %v", pull)
			}
		})
	}
}
