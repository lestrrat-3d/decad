package main

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// TestMotionClipsKeepTheMarkedFrame builds each linkage GIF's clip as the
// motion subcommand does and asserts that the GIF keeps the frame of the
// first collision VerifyLinkage proves, where the colliding body first turns
// coral: frame 92 of the arm's clip and frame 120 of the rocker's.
//
// It was seen red with the arm's stride set to 8, which drops frame 92.
func TestMotionClipsKeepTheMarkedFrame(t *testing.T) {
	t.Parallel()
	scenes := map[string]func(*testing.T) *linkageScene{
		"arm": func(t *testing.T) *linkageScene {
			scene, err := foldingArmScene(t.Context())
			require.NoError(t, err)
			return scene
		},
		"rocker": func(t *testing.T) *linkageScene {
			scene, err := crankRockerScene(t.Context())
			require.NoError(t, err)
			return scene
		},
	}
	want := map[string]int{"arm": 92, "rocker": 120}
	for _, mc := range motionClips {
		build, ok := scenes[mc.name]
		if !ok {
			continue
		}
		wantFrame := want[mc.name]
		delete(want, mc.name)
		t.Run(mc.name, func(t *testing.T) {
			t.Parallel()
			scene := build(t)
			report, err := scene.verify(t.Context())
			require.NoError(t, err)
			require.NotEmpty(t, report.Collisions)
			clip, _, err := mc.build(t.Context(), smokeWidth, smokeHeight)
			require.NoError(t, err)
			marked, err := firstFrameAt(clip, report.Collisions[0].At.Mag())
			require.NoError(t, err)
			require.Equal(t, wantFrame, marked)
			require.Zero(t, clip.FPS()%mc.stride)
			require.Zero(t, marked%mc.stride, "the GIF drops frame %d", marked)
		})
	}
	require.Empty(t, want, "a linkage GIF is missing from motionClips")
}
