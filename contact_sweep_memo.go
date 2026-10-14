package decad

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
	"github.com/lestrrat-3d/r3"
)

type sweepPathMemo = sweepmemo.PathMemo
type sweepDeviationKey = sweepmemo.DeviationKey
type sweepDeviation = sweepmemo.Deviation
type sweepRadiusKey = sweepmemo.RadiusKey
type sweepRadiusMemo = sweepmemo.RadiusMemo
type sweepIdealPose = sweepmemo.IdealPose
type cornerSpans = sweepmemo.CornerSpans

func newSweepPathMemo() *sweepPathMemo { return sweepmemo.NewPathMemo() }

func newSweepRadiusKey(from r3.Transform, center r3.Vec, axis proofarith.DyV3) sweepRadiusKey {
	return sweepmemo.NewRadiusKey(from, center, axis)
}

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
