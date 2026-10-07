package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ClrAngTol is the dimensionless angular tolerance for parallelism and
// window-membership decisions, matching the evaluator's own 1e-9-relative
// classification style (revolve.go's snapTol).
const ClrAngTol = 1e-9

// AngWindow is a closed angular window [lo, hi] (hi − lo < 2π) or the full
// circle.
type AngWindow struct {
	Lo, Hi float64
	Full   bool
}

// NewAngWindow builds the window. A window is FULL only when it really closes
// on itself: `full` is an ADMISSION — it says every azimuth lies in the trim —
// and a tolerance that rounds a window up to full admits carrier points inside
// the sliver the window is missing. The closed carriers say so themselves (a
// Circle3 edge, a closed sphere/torus meridian, a full revolve) and pass
// `full` directly; nothing else earns it on a near miss.
func NewAngWindow(a, b float64) AngWindow {
	lo, hi := math.Min(a, b), math.Max(a, b)
	if hi-lo >= 2*math.Pi {
		return AngWindow{Full: true}
	}
	return AngWindow{Lo: lo, Hi: hi}
}

// classify reports +1 when th lies in the window with more than margin to
// spare, −1 when it is out by more than margin, 0 in between.
func (w AngWindow) Classify(th, margin float64) int {
	if w.Full {
		return 1
	}
	off := survey2d.Mod2pi(th - w.Lo)
	ext := w.Hi - w.Lo
	if off <= ext {
		if off >= margin && ext-off >= margin {
			return 1
		}
		return 0
	}
	if off-ext > margin && 2*math.Pi-off > margin {
		return -1
	}
	return 0
}

// LinWindow is a closed interval on a linear coordinate.
type LinWindow struct{ Lo, Hi float64 }

func NewLinWindow(a, b float64) LinWindow {
	return LinWindow{Lo: math.Min(a, b), Hi: math.Max(a, b)}
}

func (w LinWindow) Classify(z, margin float64) int {
	if z >= w.Lo+margin && z <= w.Hi-margin {
		return 1
	}
	if z < w.Lo-margin || z > w.Hi+margin {
		return -1
	}
	return 0
}
