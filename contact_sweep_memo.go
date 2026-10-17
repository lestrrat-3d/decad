package decad

import "github.com/lestrrat-3d/decad/internal/sweepmemo"

type sweepPathMemo = sweepmemo.PathMemo
type sweepDeviationKey = sweepmemo.DeviationKey
type sweepDeviation = sweepmemo.Deviation
type sweepIdealPose = sweepmemo.IdealPose
type cornerSpans = sweepmemo.CornerSpans

func newSweepPathMemo() *sweepPathMemo { return sweepmemo.NewPathMemo() }

// attachSweepMemos gives both of a run's paths their own memo; closeSweepMemos
// closes them. Each sweep entry point attaches before its run and closes,
// deferred, before it returns.
func attachSweepMemos(paths ...*rotationalSweepPath) {
	for _, path := range paths {
		path.memo = newSweepPathMemo()
	}
}

func closeSweepMemos(paths ...*rotationalSweepPath) {
	for _, path := range paths {
		if path.memo != nil {
			path.memo.Close()
		}
	}
}
