package main

import (
	"strings"
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

// TestMotionClipsKeepTheTurnaround builds each linkage GIF's clip as the
// motion subcommand does and asserts that the GIF keeps the first and last
// frames of the hold at the turnaround, frames 192 and 224, and that a part
// flashes in the hit colour on both.
//
// It was seen red with the arm's stride set to 64, which keeps only frame
// 192 of the two.
func TestMotionClipsKeepTheTurnaround(t *testing.T) {
	t.Parallel()
	want := map[string]struct{}{"arm": {}, "rocker": {}}
	frame := time.Second / linkageFPS
	for _, mc := range motionClips {
		if _, ok := want[mc.name]; !ok {
			continue
		}
		delete(want, mc.name)
		t.Run(mc.name, func(t *testing.T) {
			t.Parallel()
			clip, style, err := mc.build(t.Context(), smokeWidth, smokeHeight)
			require.NoError(t, err)
			require.Zero(t, clip.FPS()%mc.stride)
			flashes := 0
			for _, held := range []int{int(linkageHoldStart / frame), int(linkageHoldEnd / frame)} {
				require.Zero(t, held%mc.stride, "the GIF drops frame %d", held)
				for name, part := range style.Parts {
					if !strings.HasSuffix(name, "-hit") {
						continue
					}
					at, err := part.Fade.At(clip.FrameTime(held))
					require.NoError(t, err)
					if at.Mag() == 1 {
						flashes++
					}
				}
			}
			require.Equal(t, 4, flashes, "the colliding link and the stop flash on both held frames")
		})
	}
	require.Empty(t, want, "a linkage GIF is missing from motionClips")
}
