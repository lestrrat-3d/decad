package compositesweep_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// placeless supplies the topo.Payload methods the audit never calls.
type placeless struct{}

func (placeless) Transform() r3.Transform { return r3.Identity() }
func (placeless) Placed(context.Context, topo.Env, r3.Transform) (*topo.Body, error) {
	return nil, decaderr.ErrUnsupported
}

// extentPayload is a span payload that states its own extent.
type extentPayload struct {
	placeless
	lo, hi, bound float64
}

func (p extentPayload) AuditExtent(context.Context, r3.Vec, *freeform.FreeformWork) (float64, float64, float64, error) {
	return p.lo, p.hi, p.bound, nil
}

type supportingPayload struct{ extentPayload }

func (supportingPayload) AuditEndpointSupports() bool { return true }

// auditPart is one span between z0 and z1 whose caps are planes facing +z,
// holding payload.
func auditPart(t *testing.T, payload topo.Payload, z0, z1 float64) compositesweep.SpanPart {
	t.Helper()
	capAt := func(z float64) *topo.Face {
		frame, err := r3.NewFrame(r3.NewVec(0, 0, z), r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		return &topo.Face{Surface: surfacegeom.Plane{Frame: frame}}
	}
	body := &topo.Body{Payload: payload, Bounds: measurement.Box{Min: r3.NewVec(0, 0, z0), Max: r3.NewVec(1, 1, z1)}}
	start, end := capAt(z0), capAt(z1)
	start.Body, end.Body = body, body
	return compositesweep.SpanPart{Body: body, StartCap: start, EndCap: end}
}

func TestAuditBodiesReadsPayloadExtent(t *testing.T) {
	t.Parallel()
	// The shared section is the plane z = 1, so the audit direction is -z and
	// the plane sits at -1 along it. Each span reports an extent end of -1 with
	// a 0.5 bound, which clears the plane only through the endpoint-support
	// certificate.
	extent := extentPayload{lo: -1, hi: -1, bound: 0.5}
	payload := func(supports bool) topo.Payload {
		if supports {
			return supportingPayload{extent}
		}
		return extent
	}
	for _, tc := range []struct {
		name          string
		before, after bool
		admitted      bool
	}{
		{"both support", true, true, true},
		{"before does not support", false, true, false},
		{"after does not support", true, false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			parts := []compositesweep.SpanPart{
				auditPart(t, payload(tc.before), 0, 1), auditPart(t, payload(tc.after), 1, 2),
			}
			err := compositesweep.AuditBodies(t.Context(), parts)
			if tc.admitted {
				require.NoError(t, err)
				return
			}
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
	t.Run("no extent certificate", func(t *testing.T) {
		t.Parallel()
		parts := []compositesweep.SpanPart{auditPart(t, payload(true), 0, 1), auditPart(t, placeless{}, 1, 2)}
		err := compositesweep.AuditBodies(t.Context(), parts)
		require.EqualError(t, err, decaderr.ErrUnsupported.Error()+": a composite sweep span has no analytic extent certificate")
	})
}

func TestAuditBodiesRefusals(t *testing.T) {
	t.Parallel()
	t.Run("canceled", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		require.ErrorIs(t, compositesweep.AuditBodies(ctx, []compositesweep.SpanPart{{}, {}}), context.Canceled)
		require.ErrorIs(t, compositesweep.AuditClosedBodies(ctx, []compositesweep.SpanPart{{}, {}}), context.Canceled)
	})
	t.Run("single span", func(t *testing.T) {
		t.Parallel()
		err := compositesweep.AuditBodies(t.Context(), nil)
		require.EqualError(t, err, decaderr.ErrDegenerate.Error()+": a composite sweep audit requires at least two spans")
	})
	t.Run("incomplete geometry", func(t *testing.T) {
		t.Parallel()
		err := compositesweep.AuditBodies(t.Context(), []compositesweep.SpanPart{{}, {}})
		require.EqualError(t, err, decaderr.ErrUnsupported.Error()+": composite sweep span 0 has incomplete audit geometry")
	})
	t.Run("foreign cap", func(t *testing.T) {
		t.Parallel()
		parts := []compositesweep.SpanPart{auditPart(t, placeless{}, 0, 1), auditPart(t, placeless{}, 1, 2)}
		parts[1].EndCap = parts[0].EndCap
		err := compositesweep.AuditBodies(t.Context(), parts)
		require.EqualError(t, err, decaderr.ErrUnsupported.Error()+": composite sweep span 1 has a cap from another body")
	})
}

func TestAggregateBody(t *testing.T) {
	t.Parallel()
	aggregate := func(t *testing.T, surfaceResult bool) *topo.Body {
		t.Helper()
		parts := boxParts(t, 2)
		for i, part := range parts {
			z := float64(i)
			part.Body.Solid = true
			part.Body.Volume = measurement.Measurement{Value: units.CubicMillimeters(1), Exactness: measurement.Exact}
			part.Body.Centroid = measurement.VecMeasurement{Value: r3.NewVec(0.5, 0.5, z+0.5), Exactness: measurement.Exact}
			part.Body.Bounds = measurement.Box{Min: r3.NewVec(0, 0, z), Max: r3.NewVec(1, 1, z+1), Exactness: measurement.Exact}
		}
		body, err := compositesweep.Assemble(t.Context(), topo.Env{Alloc: ownerAlloc{}}, parts, surfaceResult,
			acceptSpans, compositesweep.SewJoin, false)
		require.NoError(t, err)
		require.NoError(t, compositesweep.AggregateBody(t.Context(), body, parts, surfaceResult))
		return body
	}
	t.Run("solid", func(t *testing.T) {
		t.Parallel()
		body := aggregate(t, false)
		require.Equal(t, 2.0, body.Volume.Value.Base())
		require.Equal(t, measurement.Exact, body.Volume.Exactness)
		// Two caps and eight walls of unit area.
		require.Equal(t, 10.0, body.Area.Value.Base())
		require.Equal(t, r3.NewVec(0.5, 0.5, 1), body.Centroid.Value)
		require.Equal(t, r3.NewVec(0, 0, 0), body.Bounds.Min)
		require.Equal(t, r3.NewVec(1, 1, 2), body.Bounds.Max)
		require.Equal(t, measurement.Exact, body.Bounds.Exactness)
	})
	t.Run("surface result", func(t *testing.T) {
		t.Parallel()
		body := aggregate(t, true)
		require.Equal(t, 8.0, body.Area.Value.Base())
		require.Equal(t, 2.0, body.Volume.Value.Base())
	})
	t.Run("no spans", func(t *testing.T) {
		t.Parallel()
		err := compositesweep.AggregateBody(t.Context(), &topo.Body{}, nil, false)
		require.EqualError(t, err, decaderr.ErrDegenerate.Error()+": a composite sweep requires at least one built span")
	})
}

func TestScalarMeasurement(t *testing.T) {
	t.Parallel()
	exact := compositesweep.ScalarMeasurement(proofbound.BoundedScalar{Value: 2}, units.SquareMillimeter)
	require.Equal(t, measurement.Measurement{
		Value: units.New(2, units.SquareMillimeter), Exactness: measurement.Exact, Bound: units.New(0, units.SquareMillimeter),
	}, exact)
	loose := compositesweep.ScalarMeasurement(proofbound.BoundedScalar{Value: 2, Bound: 0.25}, units.CubicMillimeter)
	require.Equal(t, measurement.Approximate, loose.Exactness)
	require.Equal(t, 0.25, loose.Bound.Base())
}
