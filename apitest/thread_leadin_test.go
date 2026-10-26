package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/thread-design.md §7's public-surface tests for
// ThreadSpec.LeadIn, on the rod of thread_test.go.

// leadInRod is the volume of the 5 mm radius, 20 mm rod threaded at pitch 1.5 after
// both its ends are chamfered by d at 45 degrees: the chamfered rod less the groove.
// At radius rho the groove covers width(rho)/pitch of the rod's length at that radius,
// the length the chamfer leaves: 20 less 2·max(0, d − (5 − rho)).
func leadInRod(d float64) float64 {
	const R, L, pitch = 5.0, 20.0, threadPitch
	const steps = 200000
	// The chamfered rod.
	rod := math.Pi * R * R * L
	ring := 0.0
	step := d / steps
	for i := range steps {
		rho := R - d + (float64(i)+0.5)*step
		ring += 2 * math.Pi * rho * (rho - (R - d)) * step
	}
	rod -= 2 * ring
	// The groove inside it.
	depth := threadHeight * 17 / 24 * pitch
	root := R - depth
	step = depth / steps
	groove := 0.0
	for i := range steps {
		rho := root + (float64(i)+0.5)*step
		width := pitch/8 + 2*(rho-root)*math.Tan(math.Pi/6)
		length := L - 2*math.Max(0, d-(R-rho))
		groove += 2 * math.Pi * rho * length * width / pitch * step
	}
	return rod - groove
}

func TestThreadLeadInChamfersBothEndsBeforeTheThread(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
		Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch), LeadIn: units.Millimeters(0.8)})
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1, "the rod, the chamfer and the trial all leave the document")
	require.True(t, doc.Bodies()[0] == out)
	history := doc.OperationHistory()
	consumers, err := history.Consumers(rod)
	require.NoError(t, err)
	require.Len(t, consumers, 1)
	require.Equal(t, decad.OperationThread, consumers[0].Kind())
	producer, err := history.Producer(out)
	require.NoError(t, err)
	require.True(t, consumers[0] == producer)
	volume, err := out.Volume()
	require.NoError(t, err)
	requireThreadVolume(t, volume, leadInRod(0.8))
	// The chamfer takes more than the thread alone leaves.
	plain := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	threaded, err := decad.Thread(t.Context(), plain, decad.ThreadSpec{
		Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch)})
	require.NoError(t, err)
	pv, err := threaded.Volume()
	require.NoError(t, err)
	require.Less(t, volume.Value.Base(), pv.Value.Base()-1)
}

func TestThreadLeadInRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	spec := decad.ThreadSpec{Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch)}
	spec.LeadIn = units.Millimeters(-1)
	_, err := decad.Thread(t.Context(), rod, spec)
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	spec.LeadIn = units.Degrees(1)
	_, err = decad.Thread(t.Context(), rod, spec)
	require.ErrorIs(t, err, decad.ErrUnitKind)
	// A chamfer as deep as the rod's radius is the chamfer's own refusal, and the rod stays live.
	spec.LeadIn = units.Millimeters(6)
	_, err = decad.Thread(t.Context(), rod, spec)
	require.Error(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == rod)
}
