package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// editedMesher meshes as Body.Tessellate does and then edits what it returns,
// numbering the calls from one.
type editedMesher struct {
	calls int
	edit  func(call int, m *topo.Mesh)
}

func (e *editedMesher) VerifyAll(ctx context.Context, b *topo.Body, tolMM float64) (*topo.Mesh, error) {
	m, err := verifyAllMesher{}.VerifyAll(ctx, b, tolMM)
	e.calls++
	if err == nil && e.edit != nil {
		e.edit(e.calls, m)
	}
	return m, err
}

func prismPatternUnionMany(t *testing.T, mesher interface {
	VerifyAll(context.Context, *topo.Body, float64) (*topo.Mesh, error)
}, bodies ...*Body) (bool, error) {
	t.Helper()
	d := documentHandle(bodies[0].n.Doc)
	_, ok, err := prismpattern.UnionMany(t.Context(), envOf(documentNode(d), d.nextProducerID()), mesher, bodyNodes(bodies))
	return ok, err
}

// The copied-prism join refuses every operand the circular-join class does
// not cover: a blank or section with a displacement or sheet flag, a section
// whose mesh lacks its boundary or volume proof, and an operand set that is
// not a blank with copies of one section.
func TestPrismPatternUnionManyGates(t *testing.T) {
	setup := func(t *testing.T) (blank, a, b *Body) {
		t.Helper()
		_, blank, a, b = prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
		return blank, a, b
	}
	edit := func(body *Body, f func(*prismPayload)) {
		p := body.n.Payload.(prismPayload)
		f(&p)
		body.n.Payload = p
	}

	ok, err := prismPatternUnionMany(t, verifyAllMesher{}, mustBodies(setup(t))...)
	require.NoError(t, err)
	require.True(t, ok)

	const blankMsg = "needs a positive axial blank and trailing inward root arc"
	const sectionMsg = "has a displaced or open source"
	tests := map[string]struct {
		edit func(blank, a, b *Body)
		msg  string
	}{
		"blank is a sheet":         {func(blank, _, _ *Body) { edit(blank, func(p *prismPayload) { p.SurfaceResult = true }) }, blankMsg},
		"blank section displaced":  {func(blank, _, _ *Body) { edit(blank, func(p *prismPayload) { p.SectionDelta = 1e-3 }) }, blankMsg},
		"blank start displaced":    {func(blank, _, _ *Body) { edit(blank, func(p *prismPayload) { p.Z0Delta = 1e-3 }) }, blankMsg},
		"blank end displaced":      {func(blank, _, _ *Body) { edit(blank, func(p *prismPayload) { p.Z1Delta = 1e-3 }) }, blankMsg},
		"first section is a sheet": {func(_, a, _ *Body) { edit(a, func(p *prismPayload) { p.SurfaceResult = true }) }, sectionMsg},
		"first section displaced":  {func(_, a, _ *Body) { edit(a, func(p *prismPayload) { p.SectionDelta = 1e-3 }) }, sectionMsg},
		"first start displaced":    {func(_, a, _ *Body) { edit(a, func(p *prismPayload) { p.Z0Delta = 1e-3 }) }, sectionMsg},
		"second end displaced":     {func(_, _, b *Body) { edit(b, func(p *prismPayload) { p.Z1Delta = 1e-3 }) }, sectionMsg},
		"second section displaced": {func(_, _, b *Body) { edit(b, func(p *prismPayload) { p.SectionDelta = 1e-3 }) }, sectionMsg},
	}
	for name, tc := range tests {
		blank, a, b := setup(t)
		tc.edit(blank, a, b)
		ok, err := prismPatternUnionMany(t, verifyAllMesher{}, blank, a, b)
		require.True(t, ok, name)
		require.ErrorIs(t, err, ErrUnsupported, name)
		require.ErrorContains(t, err, tc.msg, name)
	}

	for name, tc := range map[string]struct {
		edit func(call int, m *topo.Mesh)
		msg  string
	}{
		"first mesh boundary unproven":  {func(call int, m *topo.Mesh) { m.BoundaryOK = m.BoundaryOK && call != 1 }, "copied-prism section 1 lacks a complete mesh proof"},
		"first mesh volume unproven":    {func(call int, m *topo.Mesh) { m.SymDiffOK = m.SymDiffOK && call != 1 }, "copied-prism section 1 lacks a complete mesh proof"},
		"second mesh boundary unproven": {func(call int, m *topo.Mesh) { m.BoundaryOK = m.BoundaryOK && call != 2 }, "copied-prism section 2 lacks a complete mesh proof"},
		"second mesh volume unproven":   {func(call int, m *topo.Mesh) { m.SymDiffOK = m.SymDiffOK && call != 2 }, "copied-prism section 2 lacks a complete mesh proof"},
	} {
		blank, a, b := setup(t)
		ok, err := prismPatternUnionMany(t, &editedMesher{edit: tc.edit}, blank, a, b)
		require.True(t, ok, name)
		require.ErrorIs(t, err, ErrUnsupported, name)
		require.ErrorContains(t, err, tc.msg, name)
	}

	for name, tc := range map[string]func(blank, a, b *Body) []*Body{
		"blank alone":   func(blank, _, _ *Body) []*Body { return []*Body{blank} },
		"sections only": func(_, a, b *Body) []*Body { return []*Body{a, b} },
		"blank starts low": func(blank, a, b *Body) []*Body {
			edit(blank, func(p *prismPayload) { p.Z0 = -1 })
			return []*Body{blank, a, b}
		},
		"blank has no extent": func(blank, a, b *Body) []*Body {
			edit(blank, func(p *prismPayload) { p.Z1 = 0 })
			return []*Body{blank, a, b}
		},
		"section is not a prism": func(blank, a, b *Body) []*Body { a.n.Payload = nil; return []*Body{blank, a, b} },
	} {
		blank, a, b := setup(t)
		ok, err := prismPatternUnionMany(t, verifyAllMesher{}, tc(blank, a, b)...)
		require.False(t, ok, name)
		require.NoError(t, err, name)
	}
}

func mustBodies(blank, a, b *Body) []*Body { return []*Body{blank, a, b} }

// The joined-prism fillet refuses a displaced radius, a body with no joined
// source, an empty selection, a source whose boundary proof is already past
// 0.1 mm, and a result whose full verification fails.
func TestPrismPatternFilletBodyGates(t *testing.T) {
	setup := func(t *testing.T) (*Body, []*Edge) {
		t.Helper()
		_, blank, section, rotated := prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
		joined, err := Union(t.Context(), blank, section, rotated)
		require.NoError(t, err)
		edges, err := prismPatternRootSelection(t, joined).SelectEdges(joined)
		require.NoError(t, err)
		return joined, edges
	}
	fillet := func(t *testing.T, mesher interface {
		VerifyAll(context.Context, *topo.Body, float64) (*topo.Mesh, error)
	}, body *Body, edges []*Edge, radius, delta float64) (*Body, error) {
		t.Helper()
		out, err := prismpattern.FilletBody(t.Context(), envOf(body.n.Doc, body.n.Doc.NextProducer), mesher,
			&body.n, edgeNodes(edges), radius, delta)
		return bodyHandle(out), err
	}

	body, edges := setup(t)
	out, err := fillet(t, verifyAllMesher{}, body, edges, 0.14, 0)
	require.NoError(t, err)
	require.NotNil(t, out)

	tests := map[string]struct {
		edit func(body *Body, edges *[]*Edge) (radius, delta float64)
		msg  string
	}{
		"displaced radius": {func(*Body, *[]*Edge) (float64, float64) { return 0.14, 1e-9 }, "radius carries a displacement"},
		"no joined source": {func(b *Body, _ *[]*Edge) (float64, float64) {
			p := b.n.Payload.(facetedPayload)
			p.PrismPatternJoin = nil
			b.n.Payload = p
			return 0.14, 0
		}, "no certified joined-prism source"},
		"not a faceted body": {func(b *Body, _ *[]*Edge) (float64, float64) {
			b.n.Payload = nil
			return 0.14, 0
		}, "no certified joined-prism source"},
		"no selected roots": {func(_ *Body, e *[]*Edge) (float64, float64) {
			*e = nil
			return 0.14, 0
		}, "no selected roots"},
		"source boundary past 0.1": {func(b *Body, _ *[]*Edge) (float64, float64) {
			p := b.n.Payload.(facetedPayload)
			p.MeshBound = 0.2
			b.n.Payload = p
			return 0.14, 0
		}, "exceeds its 0.1 mm boundary proof"},
		"source boundary not finite": {func(b *Body, _ *[]*Edge) (float64, float64) {
			p := b.n.Payload.(facetedPayload)
			p.MeshBound = math.Inf(1)
			b.n.Payload = p
			return 0.14, 0
		}, "exceeds its 0.1 mm boundary proof"},
	}
	for name, tc := range tests {
		body, edges := setup(t)
		radius, delta := tc.edit(body, &edges)
		_, err := fillet(t, verifyAllMesher{}, body, edges, radius, delta)
		require.ErrorIs(t, err, ErrUnsupported, name)
		require.ErrorContains(t, err, tc.msg, name)
	}

	for name, edit := range map[string]func(m *topo.Mesh){
		"boundary unproven": func(m *topo.Mesh) { m.BoundaryOK = false },
		"volume unproven":   func(m *topo.Mesh) { m.SymDiffOK = false },
	} {
		body, edges := setup(t)
		_, err := fillet(t, &editedMesher{edit: func(_ int, m *topo.Mesh) { edit(m) }}, body, edges, 0.14, 0)
		require.ErrorIs(t, err, ErrUnsupported, name)
		require.ErrorContains(t, err, "lacks complete verification", name)
	}
}
