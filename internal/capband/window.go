package capband

import "math"

// WallSweep returns a circular wall's cap-level angular window from the
// offset corner feet, unwrapped nearest the wall's recorded sweep. The wrap
// count lets the exact arc bracket reproduce the same branch.
func WallSweep(cU, cV float64, start, end Point, refSweep float64) (float64, float64, int) {
	capTh0 := math.Atan2(start.V-cV, start.U-cU)
	raw1 := math.Atan2(end.V-cV, end.U-cU)
	diff := raw1 - capTh0
	wraps := 0
	for diff-refSweep > math.Pi {
		diff -= 2 * math.Pi
		wraps--
	}
	for diff-refSweep < -math.Pi {
		diff += 2 * math.Pi
		wraps++
	}
	return capTh0, capTh0 + diff, wraps
}
