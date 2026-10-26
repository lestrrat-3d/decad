package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestThickenAdmissionGates feeds the Thicken dispatcher sheet payloads that
// break one admission fact at a time. Each is refused with ErrUnsupported
// before any builder runs, so the payloads carry no topology.
func TestThickenAdmissionGates(t *testing.T) {
	t.Parallel()

	const noSection = "no admitted Thicken section"

	prism := func(edit func(*prismrecord.Payload)) topo.Payload {
		p := prismrecord.Payload{Z0: 0, Z1: 10, SurfaceResult: true}
		edit(&p)
		return p
	}
	oneArc := &sweepinput.Path{Records: make([]sweepinput.PathRecord, 1)}
	compositeSpans := func(edit func(*sweeprecord.ChainPayload)) topo.Payload {
		line := chainrecord.Payload{Chains: make([]chainRecord, 1), Xform: r3.Identity()}
		arc := revolveplan.ChainPayload{Chains: make([]chainRecord, 1), Xform: r3.Identity()}
		sp := sweeprecord.ChainPayload{
			Path:  oneArc,
			Spans: []sweeprecord.ChainSpanPayload{{Chain: line}, {Arc: true, Revolve: arc}},
		}
		edit(&sp)
		return sp
	}
	arcSweep := func(phi0, phi1 float64, path *sweepinput.Path) topo.Payload {
		sp := sweeprecord.ChainPayload{Arc: true, Path: path}
		sp.Revolve.Phi0, sp.Revolve.Phi1 = phi0, phi1
		return sp
	}
	cases := []struct {
		name         string
		payload      topo.Payload
		tmm, tDelta  float64
		wantContains string
	}{
		{"prism that is no sheet", prism(func(p *prismrecord.Payload) { p.SurfaceResult = false }), 1, 0, noSection},
		{"prism with a section displacement", prism(func(p *prismrecord.Payload) { p.SectionDelta = 1 }), 1, 0, noSection},
		{"prism with a hole", prism(func(p *prismrecord.Payload) {
			p.Profile.Holes = []sectionrecord.LoopRecord{{}}
		}), 1, 0, noSection},
		{"prism with no sweep height", prism(func(p *prismrecord.Payload) { p.Z1 = p.Z0 }), 1, 0, "no proven positive sweep height"},
		{"patch whose thickness cannot be proven positive", patchchain.Payload{}, 1, 2, "no proven positive height"},
		{"sweep that is no sheet", sweeprecord.Payload{}, 1, 0, "no admitted Thicken join proof"},
		{"composite sweep sheet", sweeprecord.Payload{SurfaceResult: true, Spans: make([]sweeprecord.SpanPayload, 1)}, 1, 0,
			"no admitted Thicken join proof"},
		{"composite sweep sheet with a closed path", sweeprecord.Payload{SurfaceResult: true, ClosedComposite: true,
			Spans: make([]sweeprecord.SpanPayload, 2), Path: &sweepinput.Path{}}, 1, 0, "no admitted Thicken join proof"},
		{"composite sweep sheet with a section displacement", sweeprecord.Payload{SurfaceResult: true,
			Prism: prismrecord.Payload{SectionDelta: 1},
			Spans: make([]sweeprecord.SpanPayload, 2), Path: &sweepinput.Path{}}, 1, 0, "no admitted Thicken join proof"},
		{"arc chain sweep with no path", arcSweep(0, 1, nil), 1, 0, "no one-arc Thicken path"},
		{"arc chain sweep with two arcs", arcSweep(0, 1, &sweepinput.Path{Records: make([]sweepinput.PathRecord, 2)}), 1, 0,
			"no one-arc Thicken path"},
		{"arc chain sweep starting off zero", arcSweep(1, 2, oneArc), 1, 0, "no path-order cap proof"},
		{"arc chain sweep of no angle", arcSweep(0, 0, oneArc), 1, 0, "no path-order cap proof"},
		{"composite chain sweep with one span", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans = sp.Spans[:1]
		}), 1, 0, noSection},
		{"composite chain sweep with an arc first", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans[0].Arc = true
		}), 1, 0, noSection},
		{"composite chain sweep with a line second", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans[1].Arc = false
		}), 1, 0, noSection},
		{"composite chain sweep with no path", compositeSpans(func(sp *sweeprecord.ChainPayload) { sp.Path = nil }), 1, 0,
			noSection},
		{"composite chain sweep with two line chains", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans[0].Chain.Chains = make([]chainRecord, 2)
		}), 1, 0, noSection},
		{"composite chain sweep with two arc chains", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans[1].Revolve.Chains = make([]chainRecord, 2)
		}), 1, 0, noSection},
		{"composite chain sweep with a line displacement", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans[0].Chain.SectionDelta = 1
		}), 1, 0, noSection},
		{"composite chain sweep with an arc displacement", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans[1].Revolve.SectionDelta = 1
		}), 1, 0, noSection},
		{"composite chain sweep with a placed line", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans[0].Chain.Xform, _ = r3.Translation(r3.Vec{X: 1})
		}), 1, 0, noSection},
		{"composite chain sweep with a placed arc", compositeSpans(func(sp *sweeprecord.ChainPayload) {
			sp.Spans[1].Revolve.Xform, _ = r3.Translation(r3.Vec{X: 1})
		}), 1, 0, noSection},
		{"payload with no generator", facetedrecord.Payload{}, 1, 0, "no admitted Thicken generator"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := New()
			_, err := thickenbuild.Thicken(t.Context(), envOf(documentNode(doc), doc.nextProducerID()),
				&topo.Body{Payload: tc.payload}, thickenbuild.Positive, tc.tmm, tc.tDelta)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, tc.wantContains)
		})
	}
}
