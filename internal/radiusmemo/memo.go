// Package radiusmemo stores recent rotational sweep radius readings on a body.
package radiusmemo

import (
	"math"
	"sync"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// RadiusCap bounds how many radii one body's memo keeps. A step
// reads a path's radius for its swept box and again for its sweep, so a
// body's repeats lie within the few paths of one or two steps.
const RadiusCap = 256

// RadiusKey names one rotationalSweepRadius reading on its body: the
// from pose's and the pivot's bits and the exact dyadic axis.
type RadiusKey struct {
	from   [12]uint64
	center [3]uint64
	axis   string
}

func NewRadiusKey(from r3.Transform, center r3.Vec, axis proofarith.DyV3) RadiusKey {
	var fromBits [12]uint64
	basis := from.Basis()
	for i, v := range [4]r3.Vec{basis.EX, basis.EY, basis.EZ, from.Translation()} {
		fromBits[3*i] = math.Float64bits(v.X)
		fromBits[3*i+1] = math.Float64bits(v.Y)
		fromBits[3*i+2] = math.Float64bits(v.Z)
	}
	var b []byte
	for _, component := range axis {
		b = component.AppendKey(b)
	}
	return RadiusKey{from: fromBits,
		center: [3]uint64{math.Float64bits(center.X), math.Float64bits(center.Y), math.Float64bits(center.Z)},
		axis:   string(b)}
}

type sweepRadius struct {
	radius float64 // the float rotationalSweepRadius returns exactly
	ok     bool
}

// RadiusMemo holds a body's recent rotationalSweepRadius readings. A body
// is immutable, so a reading depends on nothing but its key; the memo changes
// no value, and each caller gets a fresh rational. When full, the oldest
// entry leaves first.
type RadiusMemo struct {
	Mu      sync.Mutex
	Entries map[RadiusKey]sweepRadius
	Order   []RadiusKey // insertion order; a ring once it holds RadiusCap keys
	Next    int
	Hits    int
	Misses  int
}

// Load returns the radius stored under key; hit is false when none is.
func (m *RadiusMemo) Load(key RadiusKey) (float64, bool, bool) {
	m.Mu.Lock()
	defer m.Mu.Unlock()
	entry, hit := m.Entries[key]
	if !hit {
		m.Misses++
		return 0, false, false
	}
	m.Hits++
	return entry.radius, entry.ok, true
}

// Store keeps a reading under key, evicting the oldest entry when full. A key
// already present keeps its reading: two concurrent misses computed the same
// one.
func (m *RadiusMemo) Store(key RadiusKey, radius float64, ok bool) {
	m.Mu.Lock()
	defer m.Mu.Unlock()
	if _, hit := m.Entries[key]; hit {
		return
	}
	if m.Entries == nil {
		m.Entries = make(map[RadiusKey]sweepRadius)
	}
	if len(m.Order) < RadiusCap {
		m.Order = append(m.Order, key)
	} else {
		delete(m.Entries, m.Order[m.Next])
		m.Order[m.Next] = key
		m.Next = (m.Next + 1) % RadiusCap
	}
	m.Entries[key] = sweepRadius{radius: radius, ok: ok}
}
