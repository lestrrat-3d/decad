package dynamics

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestTwoDynamicPatchUsesBothBodiesMassAndInertia(t *testing.T) {
	doc := decad.New()
	a := sourceBoxForFriction(t, doc, -5, -5, 5, 5, -10)
	b := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), r3.Identity(), req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	density := units.KilogramsPerCubicMillimeter(.001)
	ma, err := a.MassProperties(t.Context(), density)
	require.NoError(t, err)
	mb, err := b.MassProperties(t.Context(), density)
	require.NoError(t, err)
	pre := [2]QuantityVec{
		{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)},
		{X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)},
	}
	cfg := StepConfig{Contact: req, VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6),
		ImpactSpeed:             units.MillimetersPerSecond(0), MaxIterations: 64}
	response, ok := solveTwoDynamicFrictionPatch(contact.Manifold,
		[2]decad.MassProperties{ma, mb}, [2]r3.Transform{r3.Identity(), r3.Identity()},
		pre, exactFrictionCoefficient(units.Scalar(.5)), units.Scalar(0), cfg)
	require.True(t, ok)
	require.Len(t, response.Points, 4)
	require.InDelta(t, 20, response.Post[0].X.Base(), 1e-6)
	require.InDelta(t, 80, response.Post[1].X.Base(), 1e-6)
	require.InDelta(t, -50, response.Post[0].Z.Base(), 1e-6)
	require.InDelta(t, -50, response.Post[1].Z.Base(), 1e-6)
	require.InDelta(t, 6, response.PostAngular[0].Y.Base(), 1e-6)
	require.InDelta(t, 6, response.PostAngular[1].Y.Base(), 1e-6)
	var normal, tangent float64
	for _, point := range response.Points {
		normal += point.Normal.Base()
		tangent += point.TangentX.Base()
		require.LessOrEqual(t, math.Hypot(point.TangentX.Base(), point.TangentY.Base()),
			.5*point.Normal.Base()+cfg.ImpulseResidual.Base())
	}
	require.InDelta(t, 50, normal, 1e-6)
	require.InDelta(t, -20, tangent, 1e-6)
	require.InDelta(t, -tangent, ma.Mass.Value.Base()*response.Post[0].X.Base(), 1e-6)
	require.InDelta(t, tangent, mb.Mass.Value.Base()*(response.Post[1].X.Base()-100), 1e-6)
	require.LessOrEqual(t, response.AngularError[0].Base(), cfg.AngularVelocityResidual.Base())
	require.LessOrEqual(t, response.AngularError[1].Base(), cfg.AngularVelocityResidual.Base())

	heavierDensity := units.KilogramsPerCubicMillimeter(.002)
	heavyA, err := a.MassProperties(t.Context(), heavierDensity)
	require.NoError(t, err)
	unequal, ok := solveTwoDynamicFrictionPatch(contact.Manifold,
		[2]decad.MassProperties{heavyA, mb}, [2]r3.Transform{r3.Identity(), r3.Identity()},
		pre, exactFrictionCoefficient(units.Scalar(.5)), units.Scalar(0), cfg)
	require.True(t, ok)
	var unequalNormal, unequalTangent float64
	for _, point := range unequal.Points {
		unequalNormal += point.Normal.Base()
		unequalTangent += point.TangentX.Base()
	}
	require.InDelta(t, 100.0/(.5+1), unequalNormal, 1e-6)
	require.InDelta(t, -100.0/3, unequal.Post[0].Z.Base(), 1e-6)
	require.InDelta(t, -100.0/3, unequal.Post[1].Z.Base(), 1e-6)
	require.InDelta(t, -unequalTangent,
		heavyA.Mass.Value.Base()*unequal.Post[0].X.Base(), 1e-6)
	require.InDelta(t, unequalTangent,
		mb.Mass.Value.Base()*(unequal.Post[1].X.Base()-100), 1e-6)

	bouncing, ok := solveTwoDynamicFrictionPatch(contact.Manifold,
		[2]decad.MassProperties{ma, mb}, [2]r3.Transform{r3.Identity(), r3.Identity()},
		pre, exactFrictionCoefficient(units.Scalar(.5)), units.Scalar(.5), cfg)
	require.True(t, ok)
	require.InDelta(t, 60, bouncing.Post[1].X.Base()-bouncing.Post[0].X.Base(), 1e-6)
	require.InDelta(t, 50, bouncing.Post[1].Z.Base()-bouncing.Post[0].Z.Base(), 1e-6)
	require.InDelta(t, 6, bouncing.PostAngular[0].Y.Base(), 1e-6)
	require.InDelta(t, bouncing.PostAngular[0].Y.Base(), bouncing.PostAngular[1].Y.Base(), 1e-6)
	require.LessOrEqual(t, bouncing.NormalResidual.Base(), cfg.VelocityResidual.Base())
}
