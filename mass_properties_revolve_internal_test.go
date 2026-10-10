package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestRevolveMassPropertiesRefusesUnchargedTerms checks each reject-only gate
// of massmoment.RevolveProperties on an otherwise admitted quarter revolve: every
// payload field whose effect the mass path does not charge must refuse with
// ErrUnsupported rather than publish a tensor missing that term.
func TestRevolveMassPropertiesRefusesUnchargedTerms(t *testing.T) {
	w := sketch.NewWorld()
	sk, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := sk.CreateRectangle(0, 5, 10, 15)
	sk.Fix(rect.A)
	_, err = sk.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Revolve(sk, sk.Profiles()[0],
		SketchLine{Start: Point2{}, End: Point2{U: 1}},
		AngleExtent{A: units.Degrees(90), Dir: Along})
	require.NoError(t, err)
	base, ok := body.payload.(revolvePayload)
	require.True(t, ok)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	_, err = massmoment.RevolveProperties(t.Context(), massRevolveRecord(base), vecMeasurementToInternal(body.centroid), density)
	require.NoError(t, err, `the unmodified payload is admitted`)

	for name, edit := range map[string]func(*revolvePayload){
		"section displacement": func(rp *revolvePayload) { rp.sectionDelta = 1e-9 },
		"anchor bound":         func(rp *revolvePayload) { rp.ax.AUBound = 1e-15 },
		"direction bound":      func(rp *revolvePayload) { rp.ax.DVBound = 1e-16 },
		"admitted band":        func(rp *revolvePayload) { rp.ax.RadialAdmitAllow = 1e-12 },
		"axis snap":            func(rp *revolvePayload) { rp.ax.Snap.Second = 1e-12 },
		"undenoted sweep end":  func(rp *revolvePayload) { rp.den.Phi1 = revolveangle.Angle{} },
	} {
		t.Run(name, func(t *testing.T) {
			rp := base
			edit(&rp)
			got, err := massPropertiesFromReadings(
				massmoment.RevolveProperties(t.Context(), massRevolveRecord(rp), vecMeasurementToInternal(body.centroid), density))
			require.ErrorIs(t, err, ErrUnsupported)
			require.Equal(t, MassProperties{}, got)
		})
	}
}
