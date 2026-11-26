// Package statedigest encodes a Go value graph to a canonical byte string and
// hashes it. Two graphs get the same digest exactly when they have the same
// shape: the same types, scalars, pointer sharing and identity-equality
// pattern, whatever their addresses and whatever the numeric values of their
// identities (agent-docs/body-digest-design.md).
//
// The package names no decad type. A caller passes the identity types, the
// skipped fields, the opaque pointer types, the owner field that maps a node
// to its handle, and a hook that adds side-table values after a pointer's
// first visit.
package statedigest

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"errors"
	"fmt"
	"hash"
	"math"
	"reflect"
	"slices"
	"strings"
	"sync"
	"unsafe"
)

// ErrUnsupported is wrapped by every error the encoder returns for a value it
// cannot state: a function, a channel, an unsafe pointer, an unclassified
// synchronization value, or two map keys that encode alike.
var ErrUnsupported = errors.New("statedigest: unsupported value")

// isolatedLimit bounds the bytes of one isolated encoding (a map key ordered
// before the main walk has met it).
const isolatedLimit = 4 << 20

// pollEvery is the number of first visits between two ctx checks.
const pollEvery = 4096

// Config says what the encoder skips, relabels and extends. The zero Config
// encodes every field of every reachable value. A Config is read-only once
// passed to New.
type Config struct {
	// Skip lists, per struct type, the fields left out: result-neutral caches
	// and back-pointers.
	Skip map[reflect.Type][]string
	// RefuseTypes lists struct types the encoder will not state, with the
	// reason. A graph that reaches a value of one is ErrUnsupported: the type
	// holds live or lazily built state the digest cannot capture.
	RefuseTypes map[reflect.Type]string
	// RefuseFields lists, per struct type, fields refused with a reason while
	// they hold a non-zero value.
	RefuseFields map[reflect.Type]map[string]string
	// Opaque lists pointer types encoded as one fixed tag, never followed.
	Opaque []reflect.Type
	// IDs lists the integer types whose values of 1 or more are replaced by
	// the order of their first visit. Zero and negative values stay raw.
	IDs []reflect.Type
	// IDFields lists, per struct type, integer fields that hold a value of an
	// identity type as a plain integer. The field is relabeled in that
	// identity type's order, so it stays equal to the identity it copies.
	IDFields map[reflect.Type]map[string]reflect.Type
	// OwnerField names an interface field of a node struct. When it holds a
	// non-nil pointer, a pointer to the node is encoded as that pointer (the
	// node's handle), so a node and its handle are one entry. The field
	// itself is never encoded.
	OwnerField string
	// Extra is called with each pointer after its pointee was encoded, on its
	// first visit only. The values it returns are encoded next. It may return
	// nil.
	Extra func(p reflect.Value) []reflect.Value
}

// Encoder holds a compiled Config. It is safe for concurrent use.
type Encoder struct {
	cfg    Config
	opaque map[reflect.Type]struct{}
	ids    map[reflect.Type]struct{}
	infos  sync.Map // reflect.Type -> *structInfo
	plain  sync.Map // reflect.Type -> bool
}

type structInfo struct {
	skip   []bool
	refuse []string // reason per field, "" for none
	idOf   []reflect.Type
	owner  int // index of the owner field, or -1
	sync   bool
}

// New compiles cfg.
func New(cfg Config) *Encoder {
	e := &Encoder{cfg: cfg, opaque: map[reflect.Type]struct{}{}, ids: map[reflect.Type]struct{}{}}
	for _, t := range cfg.Opaque {
		e.opaque[t] = struct{}{}
	}
	for _, t := range cfg.IDs {
		e.ids[t] = struct{}{}
	}
	return e
}

type nodeKey struct {
	p uintptr
	n int
	t reflect.Type
}

type idKey struct {
	t reflect.Type
	v int64
}

// Walk is the result of one encoding: the digest and the first-visit order of
// every node and identity the walk met. A Walk is read-only.
type Walk struct {
	Sum   [32]byte
	nodes map[nodeKey]int
	ids   map[idKey]int
}

// Nodes returns how many pointer and slice nodes the walk visited.
func (w *Walk) Nodes() int { return len(w.nodes) }

type segment struct {
	t reflect.Type
	i int
}

type walker struct {
	e      *Encoder
	check  func() error // ctx.Err
	h      hash.Hash    // nil for an isolated encoding
	buf    []byte
	seen   map[nodeKey]int
	ids    map[idKey]int
	nextID map[reflect.Type]int
	path   []segment
	visits int
	err    error
}

// Encode walks root and returns its Walk. prefix is hashed first, so a caller
// can bind settings the graph does not hold. The walk polls ctx and returns
// ctx.Err() unchanged when it is done.
func (e *Encoder) Encode(ctx context.Context, prefix []byte, root reflect.Value) (*Walk, error) {
	w := &walker{
		e: e, check: ctx.Err, h: sha256.New(), seen: map[nodeKey]int{}, ids: map[idKey]int{},
		nextID: map[reflect.Type]int{},
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	w.str(string(prefix))
	w.walk(root)
	if w.err != nil {
		return nil, w.err
	}
	w.flush()
	out := &Walk{nodes: w.seen, ids: w.ids}
	w.h.Sum(out.Sum[:0])
	return out, nil
}

// Shared returns the canonical encoding of what a and b have in common: each
// node and each identity both walks met, as the pair of its first-visit
// orders. Two pairs of walks with equal digests and equal Shared bytes have the
// same joint shape.
func Shared(a, b *Walk) []byte {
	var out []byte
	small, large, swapped := a.nodes, b.nodes, false
	if len(small) > len(large) {
		small, large, swapped = large, small, true
	}
	type pair struct{ a, b int }
	var np []pair
	for k, i := range small {
		j, ok := large[k]
		if !ok {
			continue
		}
		if swapped {
			np = append(np, pair{j, i})
			continue
		}
		np = append(np, pair{i, j})
	}
	out = appendPairs(out, 'N', np, func(p pair) (int, int) { return p.a, p.b })
	type idPair struct {
		t    string
		a, b int
	}
	var ip []idPair
	for k, i := range a.ids {
		if j, ok := b.ids[k]; ok {
			ip = append(ip, idPair{typeName(k.t), i, j})
		}
	}
	slices.SortFunc(ip, func(x, y idPair) int {
		if c := strings.Compare(x.t, y.t); c != 0 {
			return c
		}
		return x.a - y.a
	})
	out = append(out, 'I')
	out = binary.LittleEndian.AppendUint64(out, uint64(len(ip)))
	for _, p := range ip {
		out = binary.LittleEndian.AppendUint64(out, uint64(len(p.t)))
		out = append(out, p.t...)
		out = binary.LittleEndian.AppendUint64(out, uint64(p.a))
		out = binary.LittleEndian.AppendUint64(out, uint64(p.b))
	}
	return out
}

func appendPairs[T any](out []byte, tag byte, ps []T, get func(T) (int, int)) []byte {
	slices.SortFunc(ps, func(x, y T) int {
		xa, _ := get(x)
		ya, _ := get(y)
		return xa - ya
	})
	out = append(out, tag)
	out = binary.LittleEndian.AppendUint64(out, uint64(len(ps)))
	for _, p := range ps {
		a, b := get(p)
		out = binary.LittleEndian.AppendUint64(out, uint64(a))
		out = binary.LittleEndian.AppendUint64(out, uint64(b))
	}
	return out
}

func typeName(t reflect.Type) string { return t.PkgPath() + "|" + t.String() }

func (w *walker) failf(format string, args ...any) {
	if w.err != nil {
		return
	}
	var sb strings.Builder
	for i, s := range w.path {
		if i > 0 {
			sb.WriteByte('.')
		}
		sb.WriteString(s.t.Name())
		if s.i >= 0 {
			sb.WriteByte('.')
			sb.WriteString(s.t.Field(s.i).Name)
		}
	}
	w.err = fmt.Errorf("%w: %s at %s", ErrUnsupported, fmt.Sprintf(format, args...), sb.String())
}

func (w *walker) flush() {
	if w.h == nil || len(w.buf) == 0 {
		return
	}
	w.h.Write(w.buf)
	w.buf = w.buf[:0]
}

func (w *walker) room() {
	if w.h != nil && len(w.buf) > 1<<16 {
		w.flush()
	}
	if w.h == nil && len(w.buf) > isolatedLimit {
		w.failf("isolated encoding exceeds %d bytes", isolatedLimit)
	}
}

func (w *walker) u64(x uint64) {
	w.buf = binary.LittleEndian.AppendUint64(w.buf, x)
	w.room()
}

func (w *walker) tag(b byte) { w.buf = append(w.buf, b) }

func (w *walker) str(s string) {
	w.u64(uint64(len(s)))
	w.buf = append(w.buf, s...)
	w.room()
}

func (w *walker) raw(p unsafe.Pointer, n uintptr) {
	if n == 0 {
		return
	}
	b := unsafe.Slice((*byte)(p), n)
	if w.h == nil {
		w.buf = append(w.buf, b...)
		w.room()
		return
	}
	w.flush()
	w.h.Write(b)
}

func (w *walker) poll() {
	w.visits++
	if w.visits%pollEvery != 0 || w.err != nil {
		return
	}
	if err := w.check(); err != nil {
		w.err = err
	}
}

func (e *Encoder) info(t reflect.Type) *structInfo {
	if v, ok := e.infos.Load(t); ok {
		return v.(*structInfo) //nolint:forcetypeassert // the map holds only *structInfo
	}
	si := &structInfo{skip: make([]bool, t.NumField()), refuse: make([]string, t.NumField()),
		idOf: make([]reflect.Type, t.NumField()), owner: -1}
	switch t.PkgPath() {
	case "sync", "sync/atomic":
		si.sync = true
	}
	skipNames := e.cfg.Skip[t]
	for i := range t.NumField() {
		name := t.Field(i).Name
		si.skip[i] = slices.Contains(skipNames, name)
		si.refuse[i] = e.cfg.RefuseFields[t][name]
		si.idOf[i] = e.cfg.IDFields[t][name]
		if e.cfg.OwnerField != "" && name == e.cfg.OwnerField && t.Field(i).Type.Kind() == reflect.Interface {
			si.owner = i
		}
	}
	e.infos.Store(t, si)
	return si
}

func (e *Encoder) isID(t reflect.Type) bool {
	_, ok := e.ids[t]
	return ok
}

// plainType reports a type whose values are hashable as raw bytes: scalars,
// arrays and structs of them with no padding, no identity type and no skipped
// field.
func (e *Encoder) plainType(t reflect.Type) bool {
	if v, ok := e.plain.Load(t); ok {
		return v.(bool) //nolint:forcetypeassert // the map holds only bool
	}
	var r bool
	switch t.Kind() {
	case reflect.Bool, reflect.Int8, reflect.Uint8, reflect.Int16, reflect.Uint16, reflect.Int32, reflect.Uint32,
		reflect.Int64, reflect.Uint64, reflect.Int, reflect.Uint, reflect.Float32, reflect.Float64:
		r = !e.isID(t)
	case reflect.Array:
		r = e.plainType(t.Elem())
	case reflect.Struct:
		si := e.info(t)
		r = !si.sync
		var sum uintptr
		for i := 0; r && i < t.NumField(); i++ {
			r = !si.skip[i] && si.refuse[i] == "" && si.idOf[i] == nil && i != si.owner && e.plainType(t.Field(i).Type)
			sum += t.Field(i).Type.Size()
		}
		r = r && sum == t.Size()
	}
	e.plain.Store(t, r)
	return r
}

func (w *walker) id(v reflect.Value) { w.idOf(v.Type(), v.Int()) }

func (w *walker) idOf(t reflect.Type, x int64) {
	if x <= 0 {
		w.tag('-')
		w.u64(uint64(x))
		return
	}
	k := idKey{t, x}
	c, ok := w.ids[k]
	if !ok {
		w.nextID[k.t]++
		c = w.nextID[k.t]
		w.ids[k] = c
	}
	w.tag('#')
	w.u64(uint64(c))
}

// canon maps a pointer to a node onto its handle.
func (w *walker) canon(v reflect.Value) reflect.Value {
	et := v.Type().Elem()
	if et.Kind() != reflect.Struct {
		return v
	}
	si := w.e.info(et)
	if si.owner < 0 {
		return v
	}
	o := v.Elem().Field(si.owner)
	if o.IsNil() {
		return v
	}
	h := o.Elem()
	if h.Kind() != reflect.Pointer || h.IsNil() {
		return v
	}
	return h
}

func (w *walker) walk(v reflect.Value) {
	if w.err != nil {
		return
	}
	t := v.Type()
	if w.e.isID(t) {
		w.id(v)
		return
	}
	switch v.Kind() {
	case reflect.Bool:
		if v.Bool() {
			w.tag(1)
		} else {
			w.tag(0)
		}
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		w.u64(uint64(v.Int()))
	case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64, reflect.Uintptr:
		w.u64(v.Uint())
	case reflect.Float32, reflect.Float64:
		w.u64(math.Float64bits(v.Float()))
	case reflect.Complex64, reflect.Complex128:
		c := v.Complex()
		w.u64(math.Float64bits(real(c)))
		w.u64(math.Float64bits(imag(c)))
	case reflect.String:
		w.str(v.String())
	case reflect.Pointer:
		w.pointer(v)
	case reflect.Interface:
		if v.IsNil() {
			w.tag('n')
			return
		}
		e := v.Elem()
		w.tag('i')
		w.str(typeName(e.Type()))
		w.walk(e)
	case reflect.Struct:
		w.structure(v, t)
	case reflect.Slice:
		w.slice(v, t)
	case reflect.Array:
		if w.e.plainType(t) && v.CanAddr() {
			w.raw(v.Addr().UnsafePointer(), t.Size())
			return
		}
		for i := range v.Len() {
			w.walk(v.Index(i))
		}
	case reflect.Map:
		if v.IsNil() {
			w.tag('n')
			return
		}
		w.mapping(v)
	case reflect.Func, reflect.Chan, reflect.UnsafePointer:
		if v.IsNil() {
			w.tag('n')
			return
		}
		w.failf("non-nil %s value", t)
	default:
		w.failf("kind %v", v.Kind())
	}
}

func (w *walker) pointer(v reflect.Value) {
	if v.IsNil() {
		w.tag('n')
		return
	}
	if _, ok := w.e.opaque[v.Type()]; ok {
		w.tag('D')
		return
	}
	v = w.canon(v)
	if _, ok := w.e.opaque[v.Type()]; ok {
		w.tag('D')
		return
	}
	k := nodeKey{p: v.Pointer(), t: v.Type()}
	if i, ok := w.seen[k]; ok {
		w.tag('r')
		w.u64(uint64(i))
		return
	}
	w.seen[k] = len(w.seen)
	w.poll()
	w.tag('p')
	w.walk(v.Elem())
	if w.e.cfg.Extra == nil || w.err != nil {
		return
	}
	extra := w.e.cfg.Extra(v)
	if len(extra) == 0 {
		return
	}
	w.tag('S')
	for _, x := range extra {
		w.walk(x)
	}
}

func (w *walker) structure(v reflect.Value, t reflect.Type) {
	si := w.e.info(t)
	if si.sync {
		w.failf("unclassified synchronization value %s", t)
		return
	}
	if reason, ok := w.e.cfg.RefuseTypes[t]; ok {
		w.failf("%s: %s", t, reason)
		return
	}
	for i := range t.NumField() {
		if si.skip[i] || i == si.owner {
			continue
		}
		w.path = append(w.path, segment{t, i})
		if si.refuse[i] != "" && !v.Field(i).IsZero() {
			w.failf("%s", si.refuse[i])
			return
		}
		w.walk(v.Field(i))
		w.path = w.path[:len(w.path)-1]
	}
}

func (w *walker) slice(v reflect.Value, t reflect.Type) {
	if v.IsNil() {
		w.tag('n')
		return
	}
	n := v.Len()
	if n == 0 {
		w.tag('e')
		return
	}
	k := nodeKey{p: v.Pointer(), n: n, t: t}
	if i, ok := w.seen[k]; ok {
		w.tag('r')
		w.u64(uint64(i))
		return
	}
	w.seen[k] = len(w.seen)
	w.poll()
	w.tag('s')
	w.u64(uint64(n))
	if w.e.plainType(t.Elem()) {
		w.raw(v.UnsafePointer(), uintptr(n)*t.Elem().Size())
		return
	}
	for i := range n {
		w.walk(v.Index(i))
	}
}

// mapping encodes a map as its length and entries in a canonical order. A key
// that is a pointer the walk already met sorts by its visit order. Any other
// key sorts by its isolated encoding, and two keys that encode alike are an
// error, because their order could then depend on map iteration.
func (w *walker) mapping(v reflect.Value) {
	type entry struct {
		key  string
		k, e reflect.Value
	}
	es := make([]entry, 0, v.Len())
	it := v.MapRange()
	for it.Next() {
		k := it.Key()
		if k.Kind() == reflect.Pointer && !k.IsNil() {
			if _, opaque := w.e.opaque[k.Type()]; !opaque {
				c := w.canon(k)
				if i, ok := w.seen[nodeKey{p: c.Pointer(), t: c.Type()}]; ok {
					es = append(es, entry{string(binary.BigEndian.AppendUint64([]byte{0}, uint64(i))), k, it.Value()})
					continue
				}
			}
		}
		sub := &walker{e: w.e, check: w.check, seen: map[nodeKey]int{}, ids: map[idKey]int{}, nextID: map[reflect.Type]int{},
			path: slices.Clone(w.path)}
		sub.walk(k)
		if sub.err != nil {
			w.err = sub.err
			return
		}
		es = append(es, entry{string(append([]byte{1}, sub.buf...)), k, it.Value()})
	}
	slices.SortFunc(es, func(a, b entry) int { return strings.Compare(a.key, b.key) })
	for i := 1; i < len(es); i++ {
		if es[i].key == es[i-1].key {
			w.failf("two keys of %s encode alike", v.Type())
			return
		}
	}
	w.tag('m')
	w.u64(uint64(len(es)))
	for _, x := range es {
		w.walk(x.k)
		w.walk(x.e)
	}
}
