package sweepmemo

import (
	"math/big"
	"sync"

	"github.com/lestrrat-3d/r3"
)

// ReplayPoseCap bounds how many fractions one sweep's replay memo keeps.
const ReplayPoseCap = 32

// ReplayPoses is one replay answer: both certified poses, or the refusal.
type ReplayPoses struct {
	A, B r3.Transform
	Err  error
}

// ReplayPoseMemo keeps recent replay answers by exact fraction. A mutex lets
// concurrent readers of one report share the same answers and refusals.
type ReplayPoseMemo struct {
	mu      sync.Mutex
	entries map[string]ReplayPoses
	order   []string // insertion order; a ring once it holds ReplayPoseCap keys
	next    int
	hits    uint64
	misses  uint64
}

// ReplayFractionKey encodes a fraction's sign, numerator and denominator.
// big.Rat keeps lowest terms, so equal fractions share one key.
func ReplayFractionKey(f *big.Rat) string {
	num, den := f.Num().Bytes(), f.Denom().Bytes()
	key := make([]byte, 0, 5+len(num)+len(den))
	key = append(key, byte(f.Sign()+1), byte(len(num)>>24), byte(len(num)>>16), byte(len(num)>>8), byte(len(num)))
	key = append(key, num...)
	return string(append(key, den...))
}

// Load returns a cached answer and records a hit or miss.
func (m *ReplayPoseMemo) Load(key string) (ReplayPoses, bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	value, ok := m.entries[key]
	if ok {
		m.hits++
	} else {
		m.misses++
	}
	return value, ok
}

// Store keeps the first answer under a key, evicting the oldest when full.
func (m *ReplayPoseMemo) Store(key string, value ReplayPoses) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.entries[key]; ok {
		return
	}
	if m.entries == nil {
		m.entries = make(map[string]ReplayPoses)
	}
	if len(m.order) < ReplayPoseCap {
		m.order = append(m.order, key)
	} else {
		delete(m.entries, m.order[m.next])
		m.order[m.next] = key
		m.next = (m.next + 1) % ReplayPoseCap
	}
	m.entries[key] = value
}

// Counts returns the memo's hit and miss counts.
func (m *ReplayPoseMemo) Counts() (uint64, uint64) {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.hits, m.misses
}

// Len returns the number of retained replay answers.
func (m *ReplayPoseMemo) Len() int {
	m.mu.Lock()
	defer m.mu.Unlock()
	return len(m.entries)
}
