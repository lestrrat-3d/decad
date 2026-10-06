package motionbound

import (
	"math/big"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// memosOff is the test switch SetMemos flips. Production never sets it.
var memosOff atomic.Bool

// SetMemos is a test hook. Passing false makes RadianSinCos and the root
// package's sweep memos compute every value afresh, as if no memo existed;
// true, the default, restores them. It returns the previous setting. A memo
// changes no value, so a test flipping it changes no other test's outcome.
func SetMemos(on bool) bool {
	return !memosOff.Swap(!on)
}

// MemosOn reports whether the memos are in use (SetMemos).
func MemosOn() bool {
	return !memosOff.Load()
}

// AppendRatKey appends a byte encoding of r to b: its numerator and
// denominator in hexadecimal, the numerator signed, ending in ';'. Two
// encodings are equal exactly when the values are, since a big.Rat is held in
// lowest terms with a positive denominator, and the terminator keeps keys
// built from several values from running together.
func AppendRatKey(b []byte, r *big.Rat) []byte {
	b = r.Num().Append(b, 16)
	b = append(b, '/')
	b = r.Denom().Append(b, 16)
	return append(b, ';')
}

// radianSinCosMemoCap bounds the process-wide RadianSinCos memo. Of the angles
// the dynamics scenes read past the sweep paths' own memos, about one in
// three repeats an angle another sweep read (a span edge or midpoint that is
// also a sample). The wedge scene reads about 4000 distinct angles, so this
// holds one such scene's angles; tests running side by side share it and
// cycle the ring sooner, which costs repeats but no value.
const radianSinCosMemoCap = 4096

type sinCosEntry struct {
	sin, cos proofbound.RatInterval
}

// radianSinCosMemo holds recent RadianSinCos enclosures by their exact angle.
// RadianSinCos is a pure function of the angle, so one memo serves every
// caller in the process: the sweep paths' ideal poses and corner spans, and
// ParamSinCos. Callers never see the stored endpoints; each gets its own
// copies. When full, the oldest entry leaves first.
type radianSinCosMemo struct {
	mu           sync.Mutex
	entries      map[string]sinCosEntry
	order        []string // insertion order; a ring once it holds radianSinCosMemoCap keys
	next         int
	hits, misses uint64
}

var sinCosMemo radianSinCosMemo

func (m *radianSinCosMemo) load(key string) (sinCosEntry, bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	entry, ok := m.entries[key]
	if !ok {
		m.misses++
		return sinCosEntry{}, false
	}
	m.hits++
	return sinCosEntry{sin: CloneInterval(entry.sin), cos: CloneInterval(entry.cos)}, true
}

// store keeps a copy of entry under key, evicting the oldest entry when full.
// A key already present keeps its entry: two concurrent misses computed the
// same one.
func (m *radianSinCosMemo) store(key string, entry sinCosEntry) {
	entry = sinCosEntry{sin: CloneInterval(entry.sin), cos: CloneInterval(entry.cos)}
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.entries[key]; ok {
		return
	}
	if m.entries == nil {
		m.entries = make(map[string]sinCosEntry)
	}
	if len(m.order) < radianSinCosMemoCap {
		m.order = append(m.order, key)
	} else {
		delete(m.entries, m.order[m.next])
		m.order[m.next] = key
		m.next = (m.next + 1) % radianSinCosMemoCap
	}
	m.entries[key] = entry
}

// CloneInterval copies an interval's endpoints into values the caller owns.
func CloneInterval(iv proofbound.RatInterval) proofbound.RatInterval {
	return proofbound.IntervalOwned(new(big.Rat).Set(iv.Lo), new(big.Rat).Set(iv.Hi))
}
