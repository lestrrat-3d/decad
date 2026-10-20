package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// prismOverlapVolume is docs/prism-boolean-design.md §4.5's overlap-area
// reading (prismcells.OverlapVolume) as a Measurement. Each cell's scratch body
// takes a fresh producer identity from a's document.
func prismOverlapVolume(ctx context.Context, a, b *Body) (Measurement, bool, error) {
	sum, bound, ok, err := prismcells.OverlapVolume(ctx, a.n.Payload, b.n.Payload, func() topo.Env {
		d := documentHandle(a.n.Doc)
		return envOf(documentNode(d), d.nextProducerID())
	})
	if err != nil || !ok {
		return Measurement{}, false, err
	}
	return Measurement{
		Value:     units.CubicMillimeters(sum),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}, true, nil
}
