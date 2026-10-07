package decad

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
	"github.com/lestrrat-3d/r3"
)

type sweepPathMemo = sweepmemo.PathMemo
type sweepMemoStats = sweepmemo.Stats
type sweepDeviationKey = sweepmemo.DeviationKey
type sweepDeviation = sweepmemo.Deviation
type sweepRadiusKey = sweepmemo.RadiusKey
type sweepRadiusMemo = sweepmemo.RadiusMemo
type sweepIdealPose = sweepmemo.IdealPose
type cornerSpans = sweepmemo.CornerSpans

const sweepMemoCap = sweepmemo.PathCap
const sweepRadiusMemoCap = sweepmemo.RadiusCap

func newSweepPathMemo() *sweepPathMemo { return sweepmemo.NewPathMemo() }

func newSweepRadiusKey(from r3.Transform, center r3.Vec, axis proofarith.DyV3) sweepRadiusKey {
	return sweepmemo.NewRadiusKey(from, center, axis)
}

func poseBits(pose r3.Transform) [12]uint64 { return sweepmemo.PoseBits(pose) }

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
