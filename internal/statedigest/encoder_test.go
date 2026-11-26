package statedigest_test

import (
	"context"
	"reflect"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/statedigest"
	"github.com/stretchr/testify/require"
)

type id int

type node struct {
	Name  string
	ID    id
	Next  *node
	Vals  []float64
	Table map[string]int
	Cache *int
	Owner any
}

type handle struct {
	n node
}

type holder struct {
	A, B *node
}

func encoder() *statedigest.Encoder {
	return statedigest.New(statedigest.Config{
		Skip: map[reflect.Type][]string{reflect.TypeFor[node](): {"Cache"}},
		IDs:  []reflect.Type{reflect.TypeFor[id]()},
	})
}

func sum(t *testing.T, e *statedigest.Encoder, v any) [32]byte {
	t.Helper()
	w, err := e.Encode(t.Context(), nil, reflect.ValueOf(v))
	require.NoError(t, err)
	return w.Sum
}

func TestEncoderDistinguishesValues(t *testing.T) {
	e := encoder()
	base := sum(t, e, &node{Name: "a", Vals: []float64{1, 2}})
	require.Equal(t, base, sum(t, e, &node{Name: "a", Vals: []float64{1, 2}}))
	for name, other := range map[string]*node{
		"name":      {Name: "b", Vals: []float64{1, 2}},
		"value":     {Name: "a", Vals: []float64{1, 3}},
		"length":    {Name: "a", Vals: []float64{1, 2, 0}},
		"nil slice": {Name: "a"},
		"empty":     {Name: "a", Vals: []float64{}},
	} {
		require.NotEqual(t, base, sum(t, e, other), name)
	}
	// Signed zero is a different value.
	zero, negative := 0.0, 0.0
	negative = -negative
	require.NotEqual(t, sum(t, e, &node{Vals: []float64{zero}}), sum(t, e, &node{Vals: []float64{negative}}))
	// A nil map and an empty map differ, as do a nil and an empty string key set.
	require.NotEqual(t, sum(t, e, &node{}), sum(t, e, &node{Table: map[string]int{}}))
}

func TestEncoderSkipsCaches(t *testing.T) {
	e := encoder()
	one, two := 1, 2
	require.Equal(t, sum(t, e, &node{Name: "a", Cache: &one}), sum(t, e, &node{Name: "a", Cache: &two}))
}

func TestEncoderRelabelsIDs(t *testing.T) {
	e := encoder()
	require.Equal(t,
		sum(t, e, &node{ID: 5, Next: &node{ID: 9}}),
		sum(t, e, &node{ID: 100, Next: &node{ID: 3}}), "ids are named by first visit")
	require.NotEqual(t,
		sum(t, e, &node{ID: 5, Next: &node{ID: 5}}),
		sum(t, e, &node{ID: 5, Next: &node{ID: 6}}), "equal ids stay equal and distinct ids stay distinct")
	require.NotEqual(t, sum(t, e, &node{ID: 0}), sum(t, e, &node{ID: 1}), "zero stays raw")
	require.NotEqual(t, sum(t, e, &node{ID: -1}), sum(t, e, &node{ID: -2}), "negative ids stay raw")
	require.NotEqual(t, sum(t, e, &node{ID: 0}), sum(t, e, &node{ID: -1}))
}

func TestEncoderPointerSharing(t *testing.T) {
	e := encoder()
	shared := &node{Name: "x"}
	require.NotEqual(t,
		sum(t, e, &holder{A: shared, B: shared}),
		sum(t, e, &holder{A: &node{Name: "x"}, B: &node{Name: "x"}}), "one node twice is not two equal nodes")

	cycle := &node{Name: "c"}
	cycle.Next = cycle
	other := &node{Name: "c"}
	other.Next = other
	require.Equal(t, sum(t, e, cycle), sum(t, e, other))
	long := &node{Name: "c", Next: &node{Name: "c"}}
	long.Next.Next = long
	require.NotEqual(t, sum(t, e, cycle), sum(t, e, long), "a one-node cycle differs from a two-node cycle")

	vals := []float64{1, 2, 3}
	type pair struct{ A, B []float64 }
	require.NotEqual(t, sum(t, e, &pair{vals, vals}), sum(t, e, &pair{[]float64{1, 2, 3}, []float64{1, 2, 3}}))
}

func TestEncoderMapOrderIsCanonical(t *testing.T) {
	e := encoder()
	build := func() *node {
		m := map[string]int{}
		for _, k := range []string{"e", "a", "d", "b", "c", "z", "y"} {
			m[k] = len(k) + int(k[0])
		}
		return &node{Table: m}
	}
	want := sum(t, e, build())
	for range 20 {
		require.Equal(t, want, sum(t, e, build()))
	}
	require.NotEqual(t, want, sum(t, e, &node{Table: map[string]int{"a": 1}}))
}

func TestEncoderPointerKeyedMap(t *testing.T) {
	e := encoder()
	type reg struct {
		Keep  []*node
		ByPtr map[*node]string
	}
	build := func() *reg {
		keep := []*node{{Name: "k1"}, {Name: "k2"}, {Name: "k3"}}
		return &reg{Keep: keep, ByPtr: map[*node]string{keep[2]: "c", keep[0]: "a", keep[1]: "b"}}
	}
	want := sum(t, e, build())
	for range 20 {
		require.Equal(t, want, sum(t, e, build()))
	}
	// The same names attached to different nodes give a different digest.
	r := build()
	r.ByPtr = map[*node]string{r.Keep[0]: "b", r.Keep[1]: "a", r.Keep[2]: "c"}
	require.NotEqual(t, want, sum(t, e, r))
}

func TestEncoderRefusals(t *testing.T) {
	e := encoder()
	_, err := e.Encode(t.Context(), nil, reflect.ValueOf(&struct{ F func() }{F: func() {}}))
	require.ErrorIs(t, err, statedigest.ErrUnsupported)
	_, err = e.Encode(t.Context(), nil, reflect.ValueOf(&struct{ C chan int }{C: make(chan int)}))
	require.ErrorIs(t, err, statedigest.ErrUnsupported)
	_, err = e.Encode(t.Context(), nil, reflect.ValueOf(&struct{ M sync.Mutex }{}))
	require.ErrorIs(t, err, statedigest.ErrUnsupported)
	require.ErrorContains(t, err, "M", "the field path is named")
	// A nil func is a value like any other.
	_, err = e.Encode(t.Context(), nil, reflect.ValueOf(&struct{ F func() }{}))
	require.NoError(t, err)

	// Two keys that encode alike cannot be ordered.
	_, err = e.Encode(t.Context(), nil, reflect.ValueOf(&struct{ M map[id]int }{M: map[id]int{4: 1, 7: 2}}))
	require.ErrorIs(t, err, statedigest.ErrUnsupported)

	refusing := statedigest.New(statedigest.Config{
		RefuseTypes:  map[reflect.Type]string{reflect.TypeFor[holder](): "live"},
		RefuseFields: map[reflect.Type]map[string]string{reflect.TypeFor[node](): {"Next": "lazy"}},
	})
	_, err = refusing.Encode(t.Context(), nil, reflect.ValueOf(&holder{}))
	require.ErrorIs(t, err, statedigest.ErrUnsupported)
	_, err = refusing.Encode(t.Context(), nil, reflect.ValueOf(&node{Next: &node{}}))
	require.ErrorIs(t, err, statedigest.ErrUnsupported)
	_, err = refusing.Encode(t.Context(), nil, reflect.ValueOf(&node{}))
	require.NoError(t, err, "a refused field is refused only while set")
}

func TestEncoderOwnerAndOpaque(t *testing.T) {
	e := statedigest.New(statedigest.Config{
		OwnerField: "Owner",
		Opaque:     []reflect.Type{reflect.TypeFor[*holder]()},
	})
	build := func(name string) *handle {
		h := &handle{n: node{Name: name}}
		h.n.Owner = h
		return h
	}
	// A pointer to a node is encoded as its handle, so the node and the handle are one entry.
	type both struct {
		H *handle
		N *node
	}
	h := build("x")
	g := build("x")
	w1, err := e.Encode(t.Context(), nil, reflect.ValueOf(&both{H: h, N: &h.n}))
	require.NoError(t, err)
	w2, err := e.Encode(t.Context(), nil, reflect.ValueOf(&both{H: g, N: &g.n}))
	require.NoError(t, err)
	require.Equal(t, w1.Sum, w2.Sum)
	w3, err := e.Encode(t.Context(), nil, reflect.ValueOf(&both{H: g, N: &build("x").n}))
	require.NoError(t, err)
	require.NotEqual(t, w1.Sum, w3.Sum, "a node of another handle is another entry")

	type withOpaque struct{ P *holder }
	a, err := e.Encode(t.Context(), nil, reflect.ValueOf(&withOpaque{P: &holder{A: &node{Name: "a"}}}))
	require.NoError(t, err)
	b, err := e.Encode(t.Context(), nil, reflect.ValueOf(&withOpaque{P: &holder{A: &node{Name: "b"}}}))
	require.NoError(t, err)
	require.Equal(t, a.Sum, b.Sum, "an opaque pointer is not followed")
	c, err := e.Encode(t.Context(), nil, reflect.ValueOf(&withOpaque{}))
	require.NoError(t, err)
	require.NotEqual(t, a.Sum, c.Sum, "but nil differs from set")
}

func TestEncoderExtraAndPrefix(t *testing.T) {
	extra := map[*node]string{}
	e := statedigest.New(statedigest.Config{
		Extra: func(p reflect.Value) []reflect.Value {
			n, ok := reflect.TypeAssert[*node](p)
			if !ok {
				return nil
			}
			return []reflect.Value{reflect.ValueOf(extra[n])}
		},
	})
	n := &node{Name: "a"}
	extra[n] = "one"
	one, err := e.Encode(t.Context(), nil, reflect.ValueOf(n))
	require.NoError(t, err)
	extra[n] = "two"
	two, err := e.Encode(t.Context(), nil, reflect.ValueOf(n))
	require.NoError(t, err)
	require.NotEqual(t, one.Sum, two.Sum)
	pre, err := e.Encode(t.Context(), []byte("settings"), reflect.ValueOf(n))
	require.NoError(t, err)
	require.NotEqual(t, two.Sum, pre.Sum)
}

func TestEncoderCanceled(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := encoder().Encode(ctx, nil, reflect.ValueOf(&node{}))
	require.ErrorIs(t, err, context.Canceled)

	// A long walk stops partway.
	ctx, cancel = context.WithCancel(t.Context())
	defer cancel()
	var head *node
	for i := range 20000 {
		head = &node{Name: "n", Next: head, Vals: []float64{float64(i)}}
		if i == 100 {
			cancel()
		}
	}
	_, err = encoder().Encode(ctx, nil, reflect.ValueOf(head))
	require.ErrorIs(t, err, context.Canceled)
}

func TestSharedRelation(t *testing.T) {
	e := encoder()
	build := func(shared bool) (*statedigest.Walk, *statedigest.Walk) {
		s1 := &node{Name: "s", ID: 7}
		s2 := s1
		if !shared {
			s2 = &node{Name: "s", ID: 8}
		}
		a, err := e.Encode(t.Context(), nil, reflect.ValueOf(&holder{A: &node{Name: "a"}, B: s1}))
		require.NoError(t, err)
		b, err := e.Encode(t.Context(), nil, reflect.ValueOf(&holder{A: &node{Name: "b"}, B: s2}))
		require.NoError(t, err)
		return a, b
	}
	sa, sb := build(true)
	pa, pb := build(false)
	require.Equal(t, sa.Sum, pa.Sum)
	require.Equal(t, sb.Sum, pb.Sum)
	require.NotEqual(t, statedigest.Shared(sa, sb), statedigest.Shared(pa, pb))
	require.Equal(t, statedigest.Shared(sa, sb), statedigest.Shared(sa, sb))
}

type lineage struct {
	A, B id
	Copy int // holds an id as a plain int
}

func TestEncoderIDFieldsRelabel(t *testing.T) {
	e := statedigest.New(statedigest.Config{
		IDs:      []reflect.Type{reflect.TypeFor[id]()},
		IDFields: map[reflect.Type]map[string]reflect.Type{reflect.TypeFor[lineage](): {"Copy": reflect.TypeFor[id]()}},
	})
	one := sum(t, e, &lineage{A: 5, B: 9, Copy: 5})
	require.NotEqual(t, one, sum(t, e, &lineage{A: 9, B: 5, Copy: 5}), "Copy==A and Copy==B differ")
	require.Equal(t, one, sum(t, e, &lineage{A: 105, B: 109, Copy: 105}), "the same relation renumbered is equal")
	require.Equal(t, sum(t, e, &lineage{A: 5, B: 9, Copy: 0}), sum(t, e, &lineage{A: 7, B: 8, Copy: 0}))
	require.NotEqual(t, sum(t, e, &lineage{A: 5, B: 9, Copy: 0}), sum(t, e, &lineage{A: 5, B: 9, Copy: -1}))
}

func TestEncoderStrayNodeCopyStaysSeparate(t *testing.T) {
	e := statedigest.New(statedigest.Config{OwnerField: "Owner"})
	type pair struct{ A, B *node }
	h := &handle{n: node{Name: "a"}}
	h.n.Owner = h
	stray := h.n // keeps Owner naming h, but h does not hold it
	stray.Name = "b"
	g1 := sum(t, e, &pair{A: &h.n, B: &stray})
	h2 := &handle{n: node{Name: "a"}}
	h2.n.Owner = h2
	g2 := sum(t, e, &pair{A: &h2.n, B: &h2.n})
	require.NotEqual(t, g1, g2, "a by-value copy is not its source")
	h3 := &handle{n: node{Name: "a"}}
	h3.n.Owner = h3
	require.Equal(t, sum(t, e, &pair{A: &h3.n, B: &h3.n}), g2, "the handle's own node still folds")
}
