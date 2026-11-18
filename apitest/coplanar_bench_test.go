package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/coplanarcontact"
)

// BenchmarkFlushUnion times the Union of two boxes meeting face to face under
// the default gap tolerance (agent-docs/coplanar-contact-design.md §10): with
// the switch off the gap-retry ladder runs, with it on the exact coplanar pass
// classifies the contact.
func BenchmarkFlushUnion(b *testing.B) {
	for _, mode := range []struct {
		name string
		on   bool
	}{{"off", false}, {"on", true}} {
		b.Run(mode.name, func(b *testing.B) {
			ctx := coplanarcontact.With(context.Background(), mode.on)
			for range b.N {
				// A Union consumes its operands, so each run builds its own.
				b.StopTimer()
				doc := decad.New()
				lower := boxBodyAtZ(b, doc, 0, 0, 10, 10, 0, 10)
				upper := boxBodyAtZ(b, doc, 0.5, 0, 10.5, 10, 10, 10)
				b.StartTimer()
				if _, err := decad.Union(ctx, lower, upper); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
