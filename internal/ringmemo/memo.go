// Package ringmemo keeps a bounded memo of recent answers.
package ringmemo

import "sync"

// Memo keeps the first value stored under each key. Once it holds a caller's
// limit of keys, each new key evicts the oldest. A mutex lets concurrent
// readers share the same answers. The zero value is ready to use.
type Memo[K comparable, V any] struct {
	mu      sync.Mutex
	entries map[K]V
	order   []K // insertion order; a ring once it holds limit keys
	next    int
}

// Load returns the value stored under key.
func (m *Memo[K, V]) Load(key K) (V, bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	value, ok := m.entries[key]
	return value, ok
}

// Store keeps the first value under key, evicting the oldest key when the
// memo already holds limit keys. Every call on one memo passes the same limit.
func (m *Memo[K, V]) Store(key K, value V, limit int) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.entries[key]; ok {
		return
	}
	if m.entries == nil {
		m.entries = make(map[K]V)
	}
	if len(m.order) < limit {
		m.order = append(m.order, key)
	} else {
		delete(m.entries, m.order[m.next])
		m.order[m.next] = key
		m.next = (m.next + 1) % limit
	}
	m.entries[key] = value
}
