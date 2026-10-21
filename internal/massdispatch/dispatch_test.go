package massdispatch_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massdispatch"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// refusingMesher fails the test if a gate lets a body through to a mesh.
type refusingMesher struct{ t *testing.T }

func (m refusingMesher) VerifyAll(context.Context, *topo.Body, float64) (*topo.Mesh, error) {
	m.t.Helper()
	m.t.Fatal("a refused body must not be meshed")
	return nil, errors.New("unreachable")
}

// unprovenMesher returns a mesh that carries no proofs.
type unprovenMesher struct{}

func (unprovenMesher) VerifyAll(context.Context, *topo.Body, float64) (*topo.Mesh, error) {
	return &topo.Mesh{}, nil
}

func solid(payload topo.Payload) *topo.Body {
	return &topo.Body{Solid: true, Kind: topo.BodySolid, Payload: payload,
		Bounds: measurement.Box{Max: r3.Vec{X: 1, Y: 1, Z: 1}}}
}

const noCertificate = "occupied-volume certificate"

func TestPropertiesGates(t *testing.T) {
	t.Parallel()
	density := units.KilogramsPerCubicMillimeter(1)
	faceted := func(edit func(*facetedrecord.Payload)) *topo.Body {
		p := facetedrecord.Payload{Verts: make([]r3.Vec, 3), Tris: [][3]int{{0, 1, 2}}}
		if edit != nil {
			edit(&p)
		}
		return solid(p)
	}
	tests := []struct {
		name    string
		body    *topo.Body
		density units.Value
		want    error
		msg     string
	}{
		{"nil body", nil, density, decaderr.ErrDegenerate, "nil body"},
		{"density of another kind", solid(prismrecord.Payload{}), units.Millimeters(1), decaderr.ErrUnitKind, "mass density"},
		{"non-finite density", solid(prismrecord.Payload{}), units.KilogramsPerCubicMillimeter(math.Inf(1)),
			decaderr.ErrNotFinite, "density is not finite"},
		{"negative density", solid(prismrecord.Payload{}), units.KilogramsPerCubicMillimeter(-1),
			decaderr.ErrNegativeMagnitude, "negative"},
		{"zero density", solid(prismrecord.Payload{}), units.KilogramsPerCubicMillimeter(0), decaderr.ErrDegenerate, "zero"},
		{"not a solid", func() *topo.Body { b := solid(prismrecord.Payload{}); b.Solid = false; return b }(), density,
			decaderr.ErrNotSolid, ""},
		{"sheet kind", func() *topo.Body { b := solid(prismrecord.Payload{}); b.Kind = topo.BodySheet; return b }(), density,
			decaderr.ErrNotSolid, ""},
		{"no payload", solid(nil), density, decaderr.ErrUnsupported, "no evaluator payload"},
		{"prism sheet result", solid(prismrecord.Payload{SurfaceResult: true}), density, decaderr.ErrUnsupported,
			"certified volume moments"},
		{"faceted without vertices", faceted(func(p *facetedrecord.Payload) { p.Verts = nil }), density,
			decaderr.ErrUnsupported, noCertificate},
		{"faceted without triangles", faceted(func(p *facetedrecord.Payload) { p.Tris = nil }), density,
			decaderr.ErrUnsupported, noCertificate},
		{"faceted with a non-finite mesh bound", faceted(func(p *facetedrecord.Payload) { p.MeshBound = math.NaN() }),
			density, decaderr.ErrUnsupported, noCertificate},
		{"faceted with a negative mesh bound", faceted(func(p *facetedrecord.Payload) { p.MeshBound = -1 }),
			density, decaderr.ErrUnsupported, noCertificate},
		{"faceted with a non-finite volume difference", faceted(func(p *facetedrecord.Payload) { p.VolSymDiff = math.Inf(1) }),
			density, decaderr.ErrUnsupported, noCertificate},
		{"faceted with a negative volume difference", faceted(func(p *facetedrecord.Payload) { p.VolSymDiff = -1 }),
			density, decaderr.ErrUnsupported, noCertificate},
		{"faceted with an unproven mesh", faceted(nil), density, decaderr.ErrUnsupported, "no matching verified mesh"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			var mesher interface {
				VerifyAll(context.Context, *topo.Body, float64) (*topo.Mesh, error)
			} = refusingMesher{t}
			if tc.msg == "no matching verified mesh" {
				mesher = unprovenMesher{}
			}
			_, err := massdispatch.Properties(t.Context(), mesher, tc.body, tc.density)
			require.Error(t, err)
			require.True(t, errors.Is(err, tc.want), "got %v", err)
			if tc.msg != "" {
				require.ErrorContains(t, err, tc.msg)
			}
		})
	}
}
