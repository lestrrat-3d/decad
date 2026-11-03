package revolvemesh

import "github.com/lestrrat-3d/decad/internal/proofbound"

// RaceDetector exposes raceDetector so external differential sweeps can run a
// smaller case set under the race detector.
const RaceDetector = raceDetector

// RevolveAngularHomotopyFactorUncached exposes the memo-free reading so the
// external oracle test can compare the memoised answer against it.
var RevolveAngularHomotopyFactorUncached = revolveAngularHomotopyFactorUncached

// RevolveHomotopyMemoHolds reports whether the memo holds a reading for step.
func RevolveHomotopyMemoHolds(step proofbound.RatInterval) bool {
	revolveHomotopyMemo.mu.Lock()
	defer revolveHomotopyMemo.mu.Unlock()
	_, ok := revolveHomotopyMemo.entries[step.Lo.RatString()+"|"+step.Hi.RatString()]
	return ok
}
