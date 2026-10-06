package dynamics

import (
	"math"
	"sync"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
)

// worldInertiaMemoCap bounds how many rotated tensors one World keeps. A
// resting body keeps its exact pose basis from event to event and step to
// step, so its repeats come within a few hundred later reads; a body that
// turns reads a new basis each time and only cycles the ring.
const worldInertiaMemoCap = 1024

// worldInertiaKey names one body's inertia rotated into one pose basis: the
// body's world index and the bits of the basis vectors. Bits keep a −0 apart
// from a +0. Translation does not enter Rotate beyond the finiteness check
// that IsValid repeats, so a key leaves it out.
type worldInertiaKey struct {
	body  int
	basis [9]uint64
}

// worldInertia is a body's inertia tensor in world axes and its inverse.
type worldInertia struct {
	tensor, inverse r3.SymmetricTensor
}

// worldInertiaMemo holds a World's recent world-axis inertia tensors. A
// body's admitted mass reading never changes, so the tensors depend on
// nothing but the key, and a repeat returns the same bits r3 computes. Only
// a successful rotation and inversion enters it; an error is recomputed
// every time. The values are copied out, so no caller shares one. When full,
// the oldest entry leaves first. Step may run concurrently on one World, so
// a mutex guards it.
type worldInertiaMemo struct {
	mu      sync.Mutex
	entries map[worldInertiaKey]worldInertia
	order   []worldInertiaKey // insertion order; a ring once it holds worldInertiaMemoCap keys
	next    int
	hits    uint64
	misses  uint64
}

func newWorldInertiaKey(body int, pose r3.Transform) worldInertiaKey {
	key := worldInertiaKey{body: body}
	basis := pose.Basis()
	for i, v := range [3]r3.Vec{basis.EX, basis.EY, basis.EZ} {
		key.basis[3*i] = math.Float64bits(v.X)
		key.basis[3*i+1] = math.Float64bits(v.Y)
		key.basis[3*i+2] = math.Float64bits(v.Z)
	}
	return key
}

func (m *worldInertiaMemo) load(key worldInertiaKey) (worldInertia, bool) {
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

// store keeps value under key, evicting the oldest entry when full. A key
// already present keeps its value: two concurrent misses computed the same
// one.
func (m *worldInertiaMemo) store(key worldInertiaKey, value worldInertia) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.entries[key]; ok {
		return
	}
	if m.entries == nil {
		m.entries = make(map[worldInertiaKey]worldInertia)
	}
	if len(m.order) < worldInertiaMemoCap {
		m.order = append(m.order, key)
	} else {
		delete(m.entries, m.order[m.next])
		m.order[m.next] = key
		m.next = (m.next + 1) % worldInertiaMemoCap
	}
	m.entries[key] = value
}

// rotatedInertia rotates a body-frame inertia reading into pose's world axes
// through r3 and inverts it there.
func rotatedInertia(inertia decad.InertiaReading, pose r3.Transform) (worldInertia, error) {
	local, err := r3.NewSymmetricTensor(inertia.XX.Value.Base(), inertia.YY.Value.Base(),
		inertia.ZZ.Value.Base(), inertia.XY.Value.Base(), inertia.XZ.Value.Base(), inertia.YZ.Value.Base())
	if err != nil {
		return worldInertia{}, err
	}
	world, err := local.Rotate(pose)
	if err != nil {
		return worldInertia{}, err
	}
	inverse, err := world.Inverse()
	if err != nil {
		return worldInertia{}, err
	}
	return worldInertia{tensor: world, inverse: inverse}, nil
}

// worldInertia returns dynamic body index's inertia in pose's world axes and
// its inverse, from the World's memo when it holds them. An invalid pose
// bypasses the memo, so the error r3 raises for it is raised again.
func (w *World) worldInertia(index int, pose r3.Transform) (worldInertia, error) {
	inertia := w.bodies[index].mass.Inertia
	if w.inertia == nil || !pose.IsValid() {
		return rotatedInertia(inertia, pose)
	}
	key := newWorldInertiaKey(index, pose)
	if value, ok := w.inertia.load(key); ok {
		return value, nil
	}
	value, err := rotatedInertia(inertia, pose)
	if err != nil {
		return worldInertia{}, err
	}
	w.inertia.store(key, value)
	return value, nil
}
