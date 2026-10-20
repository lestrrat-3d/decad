package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// TestOffsetAdmissionGates feeds the Offset dispatcher sheet payloads that
// break one admission fact at a time. Each is refused with ErrUnsupported
// before any builder runs, so the payloads carry no topology.
func TestOffsetAdmissionGates(t *testing.T) {
	t.Parallel()

	prism := func(edit func(*prismrecord.Payload)) topo.Payload {
		p := prismrecord.Payload{Z0: 0, Z1: 10, SurfaceResult: true}
		edit(&p)
		return p
	}
	const noSection = "no admitted Offset section"
	cases := []struct {
		name         string
		payload      topo.Payload
		dDelta       float64
		wantContains string
	}{
		{"prism that is no sheet", prism(func(p *prismrecord.Payload) { p.SurfaceResult = false }), 0, noSection},
		{"prism with a section displacement", prism(func(p *prismrecord.Payload) { p.SectionDelta = 1 }), 0, noSection},
		{"prism with a hole", prism(func(p *prismrecord.Payload) {
			p.Profile.Holes = []sectionrecord.LoopRecord{{}}
		}), 0, noSection},
		{"prism with no sweep height", prism(func(p *prismrecord.Payload) { p.Z1 = p.Z0 }), 0,
			"no proven positive sweep height"},
		{"prism with a converted distance", prism(func(*prismrecord.Payload) {}), 1, "cannot be generated exactly"},
		{"patch with a converted distance", patchchain.Payload{}, 1, "carries no term for it"},
		{"payload with no generator", facetedrecord.Payload{}, 0, "no admitted Offset generator"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := New()
			_, err := thickenbuild.Offset(t.Context(), envOf(documentNode(doc), doc.nextProducerID()),
				&topo.Body{Payload: tc.payload}, false, 1, tc.dDelta)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, tc.wantContains)
		})
	}
}
