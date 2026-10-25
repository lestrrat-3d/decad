package proofbound

import "math/big"

// RadSinCosIntervalUncached exposes the memo-free reading so the external
// oracle test can compare the memoised answer against it.
var RadSinCosIntervalUncached = radSinCosIntervalUncached

// TrigWidening exposes the upward-rounded width for exact enclosure checks.
var TrigWidening = trigWidening

// RadSinCosMemoHolds reports whether the memo holds a reading for x.
func RadSinCosMemoHolds(x *big.Rat) bool {
	radSinCosMemo.mu.Lock()
	defer radSinCosMemo.mu.Unlock()
	_, ok := radSinCosMemo.entries[radSinCosKey(x)]
	return ok
}

// TurnTrigMemoLen reports how many first-octant residuals m holds.
func TurnTrigMemoLen(m *TurnTrigMemo) int { return len(m.octant) }
