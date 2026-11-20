package proofbound

// CI: isolated-tests

import "testing"

func TestRetainedRimPublicationCapacity(t *testing.T) {
	for _, test := range []struct {
		vertices, facets int
		ok               bool
	}{{16384, 32768, true}, {16385, 32768, false}, {20000, 40000, true}, {20001, 40000, false}, {20000, 40001, false}} {
		m := NewStrictReplayBudget()
		if err := m.SpendWork(t.Context(), 7); err != nil {
			t.Fatal(err)
		}
		ok, err := m.BeginRetainedRimPublication(t.Context(), test.vertices, test.facets)
		if err != nil || ok != test.ok {
			t.Fatalf("%d vertices, %d facets: ok=%v err=%v", test.vertices, test.facets, ok, err)
		}
		work, _ := m.Usage()
		if work != 7 {
			t.Fatalf("lost work before publication: %d", work)
		}
	}
}
